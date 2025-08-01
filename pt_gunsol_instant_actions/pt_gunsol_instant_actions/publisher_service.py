import os
import uvicorn
import json
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq
from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI, HTTPException, Request
import traceback
import threading
import time
import signal
import sys
import yaml
import asyncio
import aiohttp
import queue  # queue 모듈 임포트

def setup_logging(config: dict) -> logging.Logger:
    """
    로깅 설정 함수.
    PRODUCT 모드에서는 ERROR 레벨 이상만 출력.
    """
    log_dir = config['log_dir']
    os.makedirs(log_dir, exist_ok=True)
    now = datetime.now()
    log_filename = config['node_name'] + '_' + config['mode'] + '_' + now.strftime('%Y%m%d_%H%M%S') + config['log_suffix']
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
    stream_handler.setLevel(logging.ERROR if config['mode'] == "PRODUCT" else config['log_level'])  # PRODUCT 모드에선 streamhandler도 error레벨만
    stream_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    stream_handler.setFormatter(stream_formatter)
    logger.addHandler(stream_handler)

    return logger


class RosPublisher:
    """
    ZMQ 메시지를 수신하여 ROS 메시지로 발행하는 클래스.
    """

    def __init__(self, zmq_context: zmq.Context, config: dict, logger: logging.Logger, shutdown_event: threading.Event, result_queue: queue.Queue):
        """
        RosPublisher 초기화.
        """
        self.context = zmq_context
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{config['zmq_host']}:{config['zmq_port']}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.node = Node(config['node_name'] + '_publisher')
        self.publisher = self.node.create_publisher(String, config['ros_topic'], 10)
        self.logger = logger
        self.shutdown_event = shutdown_event
        self._stopped = False
        self.result_queue = result_queue  # 결과 큐 추가

    def run(self):
        """
        ZMQ 메시지를 수신하고 ROS 메시지로 발행하는 메인 루프.
        """
        try:
            poller = zmq.Poller()
            poller.register(self.socket, zmq.POLLIN)

            while rclpy.ok() and not self.shutdown_event.is_set():
                socks = dict(poller.poll(10))

                if self.socket in socks and socks[self.socket] == zmq.POLLIN:
                    try:
                        msg = self.socket.recv()
                        if msg:
                            msg_ = String()
                            msg_.data = msg.decode()
                            self.publisher.publish(msg_)
                            self.logger.info(f'Publishing: "{msg_.data}"')

                            # ROS 작업 완료 후 결과를 큐에 넣음 (문자열 형태로 변환하여 큐에 넣음)
                            self.result_queue.put(msg_.data)


                    except zmq.ZMQError as e:
                        self.logger.error(f"ZMQ 수신 오류: {e}")
                        if e.errno == zmq.ETERM:
                            self.stop()  # ETERM 발생 시 stop() 호출, 여기서 _stopped = True 설정됨
                            break  # while 루프 종료
                    except Exception as e:
                        self.logger.error(f"ROS 발행 중 오류 발생: {e}\n{traceback.format_exc()}")
        except Exception as e:
            self.logger.critical(f"ROS Publisher 치명적 오류: {e}\n{traceback.format_exc()}")
        finally:
            self.stop()


    def stop(self):
        """
        RosPublisher 종료.
        """
        if self._stopped:
            return

        self._stopped = True
        self.shutdown_event.set()  # 스레드 종료 이벤트 설정
        self.socket.close()     # ZMQ 소켓 닫기
        self.node.destroy_node()  # ROS 노드 종료
        self.logger.info("ROS Publisher 종료")


class FastApiRosBridge:
    """
    FastAPI와 ROS를 연결하는 브리지 클래스.
    """

    def __init__(self, config: dict):
        """
        FastApiRosBridge 초기화.
        """
        self.config = config
        self.logger = setup_logging(config)
        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        self.zmq_context = zmq.Context()
        self.zmq_pub = self.zmq_context.socket(zmq.PUB)
        self.zmq_pub.bind(f"tcp://*:{config['zmq_port']}")
        self.ros_publisher = None
        self.shutdown_event = threading.Event()
        self.result_queue = queue.Queue() # 결과 큐 생성
        self.api_server_url = config.get('api_server_url')
        self.action_ids_to_report = config.get('action_ids_to_report', []) # 보낼 actionId 목록

        try:
            # RosPublisher 객체 생성 및 스레드 시작 (결과 큐 전달)
            self.ros_publisher = RosPublisher(self.zmq_context, config, self.logger, self.shutdown_event, self.result_queue)
            self.ros_publisher_thread = threading.Thread(target=self.ros_publisher.run, daemon=True)
            self.ros_publisher_thread.start()


            # 결과 처리 스레드 시작
            self.result_handler_thread = threading.Thread(target=self.process_results, daemon=True)
            self.result_handler_thread.start()

            # FastAPI 엔드포인트 설정
            self.app.post(config['api_endpoint'])(self.receive_data)
        except Exception as e:
            self.logger.critical(f"ROS 프로세스 시작 실패: {e}\n{traceback.format_exc()}")

    def process_results(self):
        """
              결과 큐에서 결과를 가져와 API 서버로 전송할지 여부를 결정
        """
        
        # modified by jcw
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        while not self.shutdown_event.is_set():
            try:
                try:
                    result = self.result_queue.get(timeout=1)  # 1초 타임아웃 설정
                    try:  # try-finally 구조 추가
                        try:
                            self.logger.info(f'result Publishing: "{result}"')
                            result_json = json.loads(result)
                            # actionId 추출 로직 변경: actions 리스트 순회
                            if 'actions' in result_json and isinstance(result_json['actions'], list):
                                #for action in result_json['actions']:
                                #    action_id = action.get('actionId')
                                #    if action_id is not None and str(action_id) in self.action_ids_to_report:
                                #        loop.run_until_complete(self.send_result_to_api_server(result_json))

                                #        break  # 하나라도 일치하면 전송하고 루프 종료
                                #else:
                                self.logger.info("actionId가 보고 목록에 없어 API 서버로 전송하지 않습니다.")

                            else:
                                self.logger.warning("올바른 'actions' 형식이 아닙니다.")


                        except json.JSONDecodeError:
                            self.logger.error(f"JSON 디코딩 오류: {result}")
                        except Exception as e:
                            self.logger.error(f"결과 처리 중 오류 발생: {e}\n{traceback.format_exc()}")
                    finally:
                        self.result_queue.task_done()  # 항상 task_done() 호출
                except queue.Empty:
                    # 큐가 비어있으면 1초 대기 후 다시 시도
                    pass

            except Exception as e:
                self.logger.error(f"결과 처리 스레드 오류: {e}\n{traceback.format_exc()}")
                break

    async def send_result_to_api_server(self, result: dict):
        """
        결과 데이터를 API 서버로 전송.
        """
        if not self.api_server_url:
            self.logger.warning("API 서버 URL이 설정되지 않았습니다. 결과를 전송하지 않습니다.")
            return

        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(self.api_server_url, json=result) as response:
                    if response.status == 200:
                        self.logger.info(f"결과를 API 서버로 성공적으로 전송: {result}")
                    else:
                        self.logger.error(f"API 서버로 결과 전송 실패 (상태 코드: {response.status}): {response.text()}")
        except aiohttp.ClientError as e:
            self.logger.error(f"API 서버 통신 오류: {e}")
        except Exception as e:
            self.logger.error(f"API 서버로 결과 전송 중 오류 발생: {e}\n{traceback.format_exc()}")

    async def receive_data(self, request: Request):
        """
        POST 요청으로 데이터를 수신하고 ZMQ를 통해 ROS로 전송.
        """
        try:
            body = await request.body()
            message = body.decode('utf-8')
            if not message:
                raise HTTPException(status_code=400, detail="메시지가 필요합니다")

            try:
                json.loads(message)  # JSON 유효성 검사
            except json.JSONDecodeError as e:
                raise HTTPException(status_code=400, detail=f"잘못된 JSON 형식입니다: {e}")

            self.logger.info(f"Received data: message={message}")
            self.zmq_pub.send_string(message)  # ZMQ 메시지 발행
            return {"status": "success", "message": "데이터를 수신하여 ROS 전송 Que에 추가하였습니다."}
        except Exception as e:
            self.logger.error(f"데이터 수신 중 오류: {e}\n{traceback.format_exc()}")


    def start(self):
        """
        FastAPI 서버 시작.
        """
        config = uvicorn.Config(self.app, host=self.config['host'], port=self.config['port'])
        server = uvicorn.Server(config)
        try:
            server.run()  # 동기적으로 실행
        except Exception as e:
            self.logger.critical(f"FastAPI 서버 시작 실패: {e}\n{traceback.format_exc()}")


    def stop(self):
        """
        FastAPI 서버 및 ROS 노드, ZMQ 관련 리소스 종료
        """
        self.logger.info("종료를 시작합니다...")
        self.shutdown_event.set()  # 스레드 종료 이벤트 설정, ros_publisher.run()에서 while 루프 종료

        # RosPublisher 스레드 종료
        self.ros_publisher.stop()
        self.ros_publisher_thread.join()
        if self.ros_publisher_thread.is_alive():
            self.logger.warning("ROS Publisher 스레드 종료에 실패했습니다.")

        # result_handler 스레드 종료 대기
        self.result_handler_thread.join()
        if self.result_handler_thread.is_alive():
            self.logger.warning("Result Handler 스레드 종료에 실패했습니다.")

        self.zmq_pub.close()
        self.zmq_context.term()


def signal_handler(sig, frame):
    """
    SIGINT 및 SIGTERM 시그널 처리 핸들러.
    프로그램 종료 시 리소스를 정리.
    """
    print('You pressed Ctrl+C or received SIGTERM!')
    fastapi_bridge.stop() # 동기적으로 호출


fastapi_bridge = None

def main(args=None):
    global fastapi_bridge
    rclpy.init(args=args)

    try:
        config_path = "/home/caselab/catkin_ws/src/Wheelchair_Robot_ROS2_Project/pt_gunsol_instant_actions/pt_gunsol_instant_actions/config.yaml"  # Config file path
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        # config 파일 유효성 검증
        required_keys = ['host', 'port', 'ros_topic', 'zmq_port', 'zmq_host', 'log_dir', 'node_name', 'log_suffix', 'log_level', 'file_log_level', 'api_endpoint', 'mode', 'api_server_url']
        if not all(key in config for key in required_keys):
            raise ValueError("config.yaml에 필수 키가 없습니다.")

        fastapi_bridge = FastApiRosBridge(config)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        fastapi_bridge.start()

    except FileNotFoundError:
        logging.error(f"config 파일을 찾을 수 없습니다: {config_path}")
    except ValueError as e:
        logging.error(f"config 파일 검증 실패: {e}")
    except Exception as e:
        logging.exception(f"예상치 못한 오류 발생: {e}")
    finally:
        # 프로그램 종료 시 리소스 정리
        if fastapi_bridge:
            fastapi_bridge.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

'''
# IPC 방법

import os
import uvicorn
import json
import logging
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import multiprocessing
from fastapi import FastAPI, HTTPException, Request
import traceback

# 로깅 설정 함수
def setup_logging(log_dir):
    # 로깅 설정 (절대 경로 사용 권장)
    log_dir = "./logs/pt_gunsol"  # 로깅 디렉토리 경로
    os.makedirs(log_dir, exist_ok=True)

    # 환경 변수를 통해 모드 설정 (테스트 모드: TEST, 프로덕트 모드: PRODUCT)
    mode = os.environ.get("PT_GUNSOL_MODE", "TEST").upper()

    # 로깅 레벨 설정
    if mode == "PRODUCT":
        log_level = logging.ERROR
        log_suffix = "_product.log"
    else:
        log_level = logging.DEBUG
        log_suffix = "_test.log"
    
    # 현재 날짜와 시간을 포함한 파일 이름 생성
    now = datetime.now()
    log_filename = now.strftime('%Y%m%d_%H%M%S') + log_suffix
    log_file = os.path.join(log_dir, log_filename)
    logging.basicConfig(filename=log_file, level=log_level,
                    format='%(asctime)s - %(levelname)s - %(filename)s - %(lineno)d - %(message)s')
    return logging.getLogger(__name__)

class RosPublisherProcess:
    def __init__(self, topic, receive_pipe, shutdown_event, logger):
        self.node = Node('fastapi_ros_bridge_process')
        self.publisher = self.node.create_publisher(String, topic, 10)
        self.receive_pipe = receive_pipe
        self.shutdown_event = shutdown_event
        self.logger = logger

    def publish(self):
        try:
            while not self.shutdown_event.is_set():
                try:
                    msg = self.receive_pipe.recv() # 파이프를 통해 메시지 수신
                    if msg == "shutdown":  # 종료 신호 처리
                        break
                    msg_ = String()
                    msg_.data = msg
                    self.publisher.publish(msg_)
                    self.logger.info(f'Publishing: "{msg}"')
                except EOFError: # 파이프 종료 감지
                    self.logger.warning("Receive pipe closed. Shutting down ROS publisher.")
                    break
                except Exception as e:
                    self.logger.error(f"ROS 발행 중 오류 발생: {e}\n{traceback.format_exc()}")
        except Exception as e:
            self.logger.critical(f"ROS Publisher 프로세스 치명적 오류: {e}\n{traceback.format_exc()}")
        finally:
            self.node.destroy_node()
            self.logger.info("ROS Publisher Process 종료")


class FastApiRosBridge:
    def __init__(self, host="127.0.0.1", port=28089, topic="/vda5050/instantactions", log_dir="./logs/pt_gunsol"):
        self.logger = setup_logging(log_dir)
        self.app = FastAPI()
        self.host = host
        self.port = port
        self.topic = topic
        self.shutdown_event = multiprocessing.Event()
        self.receive_pipe, self.send_pipe = multiprocessing.Pipe() # 파이프 생성
        self.ros_process = None
        rclpy.init() # rclpy init

        try:
            self.ros_process = multiprocessing.Process(target=self._run_ros_publisher, args=(self.topic, self.receive_pipe, self.shutdown_event, self.logger))
            self.ros_process.start()
            self.app.post("/instantactions")(self.receive_data)
        except Exception as e:
            self.logger.critical(f"ROS 프로세스 시작 실패: {e}\n{traceback.format_exc()}")

    def _run_ros_publisher(self, topic, receive_pipe, shutdown_event, logger):
        publisher_process = RosPublisherProcess(topic, receive_pipe, shutdown_event, logger)
        publisher_process.publish()

    async def receive_data(self, request: Request):
        try:
            body = await request.body()
            message = body.decode('utf-8')
            if not message:
                raise HTTPException(status_code=400, detail="메시지가 필요합니다")

            self.logger.info(f"Received data: message={message}")
            self.send_pipe.send(message) # 파이프를 통해 메시지 전송
            return {"status": "success", "message": "데이터를 수신하여 ROS 전송 Que에 추가하였습니다."}
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON 디코딩 오류: {e}\n{traceback.format_exc()}")
            raise HTTPException(status_code=400, detail=f"잘못된 JSON 데이터: {e}")
        except Exception as e:
            self.logger.error(f"데이터 처리 중 오류 발생: {e}\n{traceback.format_exc()}")
            raise HTTPException(status_code=500, detail=f"내부 서버 오류: {e}")


    def start(self):
        try:
            uvicorn.run(self.app, host=self.host, port=self.port)
        except Exception as e:
            self.logger.critical(f"FastAPI 서버 시작 실패: {e}\n{traceback.format_exc()}")

    def stop(self):
        self.logger.info("ROS 노드를 종료합니다...")
        self.shutdown_event.set()
        self.send_pipe.send("shutdown") # 종료 신호 전송
        self.send_pipe.close()
        self.receive_pipe.close()
        if self.ros_process:
            self.ros_process.join()
        rclpy.shutdown()

def main(args=None):
    fastapi_bridge = FastApiRosBridge()
    try:
        fastapi_bridge.start()
    except KeyboardInterrupt:
        fastapi_bridge.stop()
    except Exception as e:
        fastapi_bridge.logger.critical(f"메인 함수에서 오류 발생: {e}\n{traceback.format_exc()}")


if __name__ == "__main__":
    main()
'''
