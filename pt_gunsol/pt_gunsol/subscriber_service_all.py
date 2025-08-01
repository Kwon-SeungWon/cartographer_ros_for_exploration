import os
import asyncio
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
import aiohttp
from aiohttp import ClientConnectionError, ClientTimeout, ClientError
import concurrent.futures
import yaml


def setup_logging(config: dict) -> logging.Logger:
    """
    로깅 설정 함수.
    PRODUCT 모드에서는 ERROR 레벨 이상만 출력.
    """
    if not config or not isinstance(config, dict):
        return logging.getLogger(__name__)

    log_dir = config['log_dir']
    os.makedirs(log_dir, exist_ok=True)
    now = datetime.now()
    log_filename = config['node_name'] + '_' + config['mode'] + '_' + now.strftime('_%Y%m%d_%H%M%S_') + config['log_suffix']
    log_file = os.path.join(log_dir, log_filename)

    MAX_BYTES = 10 * 1024 * 1024  # 10MB
    BACKUP_COUNT = 5

    logger = logging.getLogger(__name__)
    logger.setLevel(config['log_level'])

    file_handler = RotatingFileHandler(log_file, maxBytes=MAX_BYTES, backupCount=BACKUP_COUNT, encoding='utf-8')
    file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(filename)s - %(lineno)d - %(message)s')
    file_handler.setFormatter(file_formatter)
    file_handler.setLevel(logging.ERROR if config['mode'] == "PRODUCT" else config['file_log_level'])
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.ERROR if config['mode'] == "PRODUCT" else config['log_level'])
    stream_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    stream_handler.setFormatter(stream_formatter)
    logger.addHandler(stream_handler)

    return logger


class SubscriberServiceAll(Node):
    def __init__(self, config: dict):
        """
        ROS 토픽 구독 및 REST API 전송 노드 초기화.
        """
        super().__init__(config['node_name'])
        self.logger = setup_logging(config)
        self.api_url = config['api_url']
        self.action_url = config['action_url']
        self.robot_name = config.get('robot_name', 'unknown_robot')  # robot_name 추가
        self.topics = config['topics']
        self.file_topics = config.get('file_topics', [])
        self.file_img_dir = config.get('file_img_dir')
        self.shutdown_requested = False
        self.loop = asyncio.get_event_loop()

        for topic in self.topics:
            if topic == "/camera/image_raw/compressed":
                self.create_subscription(CompressedImage, topic, self.compressed_image_callback, 10)
            elif topic == "/map":
                self.create_subscription(OccupancyGrid, topic, self.occupancy_grid_callback, 10)
            else:
                self.create_subscription(String, topic, lambda msg, t=topic: self.listener_callback(msg, t), 10)
        self.logger.info(f"SubscriberServiceAll 노드가 시작되었습니다. (robot_name: {self.robot_name})")


    async def send_data_to_api(self, message: str, topic: str) -> None:
        """
        데이터를 REST API로 비동기 전송.
        """
        try:
            async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=2)) as session:
                if topic == "/vda5050/instantactions/response":
                    headers = {'Content-Type': 'application/json'}
                    async with session.post(self.action_url, data=message, headers=headers) as response:
                        if response.status == 200:
                            data = await response.text()
                            self.logger.debug(f"데이터 전송 성공 - topic: {topic}, message: {message}, response: {data}")
                        elif 400 <= response.status < 500:
                            error_message = await response.text()
                            self.logger.warning(f"클라이언트 에러 - topic: {topic}, message: {message}, 상태 코드: {response.status}, 에러 메시지: {error_message}")
                        elif response.status >= 500:
                            error_message = await response.text()
                            self.logger.error(f"서버 에러 - topic: {topic}, message: {message}, 상태 코드: {response.status}, 에러 메시지: {error_message}")
                        else:
                            self.logger.warning(f"예상치 못한 응답 상태 코드 - topic: {topic}, message: {message}, 상태 코드: {response.status}")
                else:
                    # robot_name을 topic 앞에 붙여서 전송
                    full_topic = f"{self.robot_name}{topic}"
                    self.logger.debug(f"API 전송 - 원본 topic: {topic}, 전체 topic: {full_topic}, robot_name: {self.robot_name}")
                    async with session.post(self.api_url, json={'message': message, 'topic': full_topic}) as response:
                        if response.status == 200:
                            data = await response.text()
                            self.logger.debug(f"데이터 전송 성공 - topic: {full_topic}, message: {message}")
                        elif 400 <= response.status < 500:
                            error_message = await response.text()
                            self.logger.warning(f"클라이언트 에러 - topic: {full_topic}, message: {message}, 상태 코드: {response.status}, 에러 메시지: {error_message}")
                        elif response.status >= 500:
                            error_message = await response.text()
                            self.logger.error(f"서버 에러 - topic: {full_topic}, message: {message}, 상태 코드: {response.status}, 에러 메시지: {error_message}")
                        else:
                            self.logger.warning(f"예상치 못한 응답 상태 코드 - topic: {full_topic}, message: {message}, 상태 코드: {response.status}")
        except ClientConnectionError as e:
            self.logger.error(f"API 연결 실패 - topic: {topic}, robot_name: {self.robot_name}, message: {message}: {e}")
            raise
        except ClientTimeout as e:
            self.logger.error(f"API 요청 시간 초과 - topic: {topic}, robot_name: {self.robot_name}, message: {message}: {e}")
            raise
        except ClientError as e:
            self.logger.error(f"aiohttp 에러 - topic: {topic}, robot_name: {self.robot_name}, message: {message}: {e}")
            raise
        except Exception as e:
            self.logger.exception(f"예상치 못한 오류 - topic: {topic}, robot_name: {self.robot_name}, message: {message}: {e}")
            raise

    async def _async_save_data_to_file(self, data: bytes, topic: str, filepath: str) -> None:
        """
        실제로 파일을 저장하는 비동기 코루틴 (asyncio.to_thread 용).
        """
        try:
            with open(filepath, 'wb') as f:
                f.write(data)
            self.logger.info(f"'{topic}' 토픽의 데이터가 '{filepath}'에 저장되었습니다.")
        except Exception as e:
            self.logger.error(f"'{topic}' 토픽의 데이터를 '{filepath}'에 저장하는 데 실패했습니다: {e}")

    async def save_compressed_image(self, data: bytes, topic: str) -> None:
        """
        압축된 이미지 데이터를 비동기적으로 파일로 저장.
        """
        if not self.file_img_dir:
            self.logger.warning(f"'file_img_dir'이 설정되지 않아 '{topic}' 토픽의 데이터를 파일로 저장할 수 없습니다.")
            return
            
        now = datetime.now()
        timestamp = now.strftime('%Y%m%d_%H%M%S_%f')
        filename = f"{topic.replace('/', '_')}_{timestamp}.jpg"  # 확장자를 .jpg로 가정
        filepath = os.path.join(self.file_img_dir, filename)

        await asyncio.to_thread(self._async_save_data_to_file, data, topic, filepath)

    def compressed_image_callback(self, msg: CompressedImage) -> None:
        """
        '/camera/image_raw/compressed' 토픽 메시지 수신 시 호출되는 콜백 함수.
        """
        if self.shutdown_requested:
            return
        self.logger.debug(f'압축된 이미지 수신 - 토픽: "/camera/image_raw/compressed", 데이터 길이: {len(msg.data)}')
        asyncio.create_task(self.save_compressed_image(msg.data, "/camera/image_raw/compressed"))

    async def save_occupancy_grid(self, msg: OccupancyGrid, topic: str) -> None:
        """
        OccupancyGrid 데이터를 비동기적으로 파일로 저장.
        """
        if not self.file_img_dir:
            self.logger.warning(f"'file_img_dir'이 설정되지 않아 '{topic}' 토픽의 데이터를 파일로 저장할 수 없습니다.")
            return
            
        now = datetime.now()
        timestamp = now.strftime('%Y%m%d_%H%M%S_%f')
        filename = f"{topic.replace('/', '_')}_{timestamp}.bin"
        filepath = os.path.join(self.file_img_dir, filename)

        # 맵 데이터(int8 리스트)를 바이트로 변환하여 저장
        data = bytes(msg.data)
        await asyncio.to_thread(self._async_save_data_to_file, data, topic, filepath)

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """
        '/map' 토픽 메시지 수신 시 호출되는 콜백 함수.
        """
        if self.shutdown_requested:
            return
        self.logger.debug(f'맵 데이터 수신 - 토픽: "/map", 가로: {msg.info.width}, 세로: {msg.info.height}, 데이터 길이: {len(msg.data)}')
        asyncio.create_task(self.save_occupancy_grid(msg, "/map"))

    async def save_generic_data(self, data: bytes, topic: str) -> None:
        """
        일반적인 바이너리 데이터를 비동기적으로 파일로 저장.
        """
        if not self.file_img_dir:
            self.logger.warning(f"'file_img_dir'이 설정되지 않아 '{topic}' 토픽의 데이터를 파일로 저장할 수 없습니다.")
            return
        now = datetime.now()
        timestamp = now.strftime('%Y%m%d_%H%M%S_%f')
        filename = f"{topic.replace('/', '_')}_{timestamp}.dat"
        filepath = os.path.join(self.file_img_dir, filename)

        await asyncio.to_thread(self._async_save_data_to_file, data, topic, filepath)

    def listener_callback(self, msg: String, topic: str) -> None:
        """
        ROS 메시지 수신 시 호출되는 콜백 함수.
        """
        if self.shutdown_requested:
            return
        #self.logger.debug(f'메시지 수신 - 토픽: "{topic}", 메시지: "{msg.data}"')
        if topic in self.file_topics:
            asyncio.create_task(self.save_generic_data(msg.data.encode('utf-8'), topic))
        else:
            asyncio.run(self.send_data_to_api(msg.data, topic))


    def destroy_node(self) -> None:
        """
        노드 종료 전 리소스 정리.
        """
        self.shutdown_requested = True
        super().destroy_node()


def main(args=None):
    """
    메인 함수.
    """
    rclpy.init(args=args)
    subscriber = None
    try:
        config_path = "/home/caselab/Wheelchair_Robot_ROS2_Project_backup/pt_gunsol/pt_gunsol/config.yaml"  #config 파일 경로
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        # config 유효성 검사
        required_keys = ['api_url', 'topics', 'log_dir', 'node_name', 'robot_name', 'log_suffix', 'log_level', 'file_log_level', 'max_workers', 'file_topics', 'file_img_dir']
        if not all(key in config for key in required_keys):
            raise ValueError("config.yaml에 필수 키가 없습니다.")
        
        # robot_name 유효성 검사
        if not config.get('robot_name') or config['robot_name'].strip() == '':
            raise ValueError("robot_name이 비어있거나 설정되지 않았습니다.")

        subscriber = SubscriberServiceAll(config)
        rclpy.spin(subscriber)
    except FileNotFoundError:
        logger = logging.getLogger(__name__)
        logger.error(f"config 파일을 찾을 수 없습니다: {config_path}")
    except ValueError as e:
        logger = logging.getLogger(__name__)
        logger.error(f"config 파일 검증 실패: {e}")
    except Exception as e:
        logger = logging.getLogger(__name__)
        logger.exception(f"예상치 못한 오류 발생: {e}")
    finally:
        if subscriber:
            subscriber.destroy_node()


if __name__ == '__main__':
    main()
    

'''
# 개발자 실수를 방지 하기 위한 log file size에 제약 설정
class SizeLimitedHandler(logging.Handler):
    def __init__(self, filename, max_size_bytes):
        super().__init__()
        self.filename = filename
        self.max_size_bytes = max_size_bytes
        self.file = None
        self.lock = threading.Lock()
        self.open_file()

    def emit(self, record):
        try:
            with self.lock:
                if self.file.tell() < self.max_size_bytes:
                    msg = self.format(record)
                    self.file.write(msg + '\n')
                    self.file.flush()
                else:
                    self.close() # 파일 크기 초과 시 즉시 닫음
                    print(f"로그 파일 크기 ({self.max_size_bytes}바이트) 초과. 로그 기록 중지.")
        except Exception as e:
            self.handleError(record)
            self.close() # 에러 발생 시 닫음

    def open_file(self):
        if self.file is None:
            try:
                self.file = open(self.filename, 'a', encoding='utf-8')
                self.file.seek(0, os.SEEK_END)
            except Exception as e:
                print(f"로그 파일 열기 실패: {e}")

    def close(self):
        if self.file:
            try:
                with self.lock:
                    self.file.close()
            except Exception as e:
                print(f"로그 파일 닫기 실패: {e}")
            finally:
                self.file = None
'''
''' 재시도용으로 검토 하였으나, 문제 발생 시키는 코드로 논의 필요, - 웹 서버 오동작 또는 off 일시 및 잦은 retry로 인한 리소스 낭비 및 ros와의 연동 손해 
    async def send_data_to_api(self, message, topic):
        url = 'http://192.168.45.201:18089/data'
        retries = 3  # 재시도 횟수
        for attempt in range(retries):
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.post(url, json={'message': message, 'topic': topic}, timeout=aiohttp.ClientTimeout(total=5)) as response:
                        if response.status == 200:
                            data = await response.json()
                            self.get_logger().debug(f"데이터가 REST API로 성공적으로 전송되었습니다: {data}") # get_logger() 사용
                            return  # 성공 시 함수 종료
                        else:
                            error_text = await response.text()
                            self.get_logger().error(f"REST API로 데이터 전송 실패 (상태코드: {response.status}): {error_text}")
                            if attempt < retries -1:
                                await asyncio.sleep(2**attempt) # 지수 백오프
                            else:
                                raise Exception("API 요청 실패") # 마지막 시도 실패시 예외 발생
            except (ClientConnectionError, ClientResponseError, ClientPayloadError, asyncio.TimeoutError, aiohttp.ClientError) as e:
                self.get_logger().exception(f"REST API 통신 오류 (시도: {attempt+1}/{retries}): {e}")
                if attempt < retries - 1:
                    await asyncio.sleep(2**attempt) # 지수 백오프
                else:
                    raise  # 마지막 시도 실패시 예외 발생

            except Exception as e:
                self.get_logger().exception(f"예상치 못한 오류 (시도: {attempt+1}/{retries}): {e}")
                if attempt < retries - 1:
                    await asyncio.sleep(2**attempt)  # 지수 백오프
                else:
                    raise # 마지막 시도 실패시 예외 발생

    def listener_callback(self, msg, topic):
        if self.shutdown_requested:
            return
        self.get_logger().debug(f'토픽 [{topic}] 수신된 메시지: "{msg.data}"') # get_logger() 사용
        asyncio.run(self.send_data_to_api(msg.data, topic)) # asyncio.run 사용
'''
