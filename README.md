# MOBILE_ROBOT_BASIC

ROS2 Humble 기반 모바일 로봇 시스템의 핵심 패키지로써 모바일 로봇의 자율주행 및 제어를 위한 통합 솔루션

## 시스템 아키텍처

```
MOBILE_ROBOT_BASIC/
├── amr/                    # 메타 패키지 (시스템 통합)
├── amr_cartographer/       # SLAM 및 맵핑 시스템
├── amr_core/              # 로봇 제어 시스템
├── amr_description/       # 로봇 모델링 및 시각화
├── amr_interface/         # 로봇 상태 관리 및 작업 실행 시스템
├── amr_navigation/        # 자율주행 시스템
├── amr_serial/            # 조이스틱, 스위치 시리얼 통신
└── amr_odometry/          # 위치 추정 시스템
```

## 최근 변경 사항 (2025.07.22)

- **CAN 통신**: 확장(Extended) ID 사용, 수신 시 29비트 마스킹 비교. 송신 데이터는 PID_PNT_VEL_CMD(207) 등 8바이트 포맷 사용.
- **Kinematics**: calculateRPM, calculateVW 함수가 int16_t 타입 지원. 파라미터(wheel_radius, distance_wheels, gear_ratio 등)는 실제 하드웨어와 일치시켜야 함.
- **amr_serial 패키지**: serial_all.py에서 GPIO/조이스틱 상태를 읽어 joy_vel, joystick_states 토픽 퍼블리시. get_select_switch_status 서비스 제공. param/serial.yaml은 ROS2 표준 포맷(ros__parameters)로 작성 필요. 시리얼 포트 권한(dialout 그룹) 및 포트명 확인 필수.
- **서비스 연동**: amr_core의 ModeManager는 get_select_switch_status 서비스와 비동기 콜백 방식으로 연동. spin_until_future_complete 사용 금지, async_send_request + 콜백 사용. 서비스 응답이 없거나 예외 발생시 state_ = AUTO로 fallback.
- **디버깅/로그**: CAN 수신 콜백에서 raw data, RPM, 계산된 속도(v, w) 모두 로그 출력 가능. 파라미터, 수신 데이터, 서비스 응답 등 단계별로 상세 로그 추가 권장.
- **빌드/실행**: pyserial 등 파이썬 의존성 설치 필요: pip3 install pyserial. 시리얼 포트 권한: sudo usermod -aG dialout $USER (로그아웃/재로그인 필요). colcon build 후 source install/setup.bash 필수. launch 파일/ros2 run으로 실행 시 파라미터 파일 경로, 노드명, 서비스명 일치 확인.

## 시스템 구성요소

### amr_core

로봇 제어 시스템의 중앙 처리 유닛

- 상태 머신 기반 제어 로직 (INIT, IDLE, AUTO, MANUAL, DOCKING, CHARGING, STOP, EMERGENCY)
- 동적 속도 프로파일링 (VelocityProfile 클래스)
- 실시간 제어 명령 처리 (20Hz 제어 주기)
- EKF 기반 상태 추정 (선택적)
- 멀티스레드 안전한 데이터 처리

### amr_odometry

정밀 위치 추정 시스템

- 기본 오도메트리: 단순 적분 기반 위치 추정
- EKF 오도메트리:
  - 상태 벡터: [x, y, yaw, v, w]
  - 프로세스 노이즈: Q = diag(0.01, 0.01, 0.01, 0.05, 0.05)
  - 측정 노이즈: R = diag(0.01, 0.01, 0.01)
  - 초기 공분산: P = 0.01 * I
- 실시간 TF 브로드캐스팅
- 동적 게인 조정 (enc_vel_gain, enc_wvel_gain)

### amr_navigation

자율주행 시스템

- Nav2 기반 경로 계획 및 실행
- 실시간 파라미터 업데이트 (NavigationParameterUpdater)
- TF2 기반 위치 추적
- 목표 지점 도달 제어
- 코스트맵 관리 (clear_costmap_node)

### amr_interface

로봇 상태 관리 및 작업 실행 시스템

- 상태 머신 기반 작업 관리 (INIT, IDLE, AUTO, MANUAL, DOCKING, CHARGING, STOP, EMERGENCY, SLAM)
- 작업 실행 관리 (LOAD, UNLOAD, HOME, MOVE)
- 실시간 센서 데이터 처리
  - 초음파 센서 (8채널)
  - 충돌 센서
  - 배터리 상태 모니터링
- 장애물 감지 및 회피
- 도킹/언도킹 제어
- 맵 관리 (생성, 저장, 변경)
- 파라미터 동적 업데이트
- 비상 상황 처리 (비상 정지, 충돌 감지)

### amr_description

로봇 모델링 시스템

- URDF 기반 로봇 모델
- 센서 TF 설정
- 충돌 모델 정의
- 시각화 설정

### amr_cartographer

SLAM 시스템

- 실시간 맵핑
- 루프 클로저 검출
- 맵 최적화
- 위치 추정

### amr_serial

시리얼 통신 및 외부 인터페이스 연동 시스템

- 시리얼 포트(GPIO, 조이스틱 등) 상태를 읽어 ROS2 토픽(joy_vel, joystick_states)으로 퍼블리시
- get_select_switch_status 서비스 제공 (로봇 모드 전환 연동)
- 파라미터 파일(param/serial.yaml)은 ROS2 표준 포맷(ros__parameters)로 작성 필요
- 시리얼 포트 권한(dialout 그룹) 및 포트명 확인 필수

## 시스템 요구사항

### 필수 소프트웨어

- ROS2 Humble
- Ubuntu 22.04 LTS
- CMake 3.8 이상
- C++14 컴파일러

### 필수 라이브러리

- Eigen3
- Boost
- tf2
- nav2
- cartographer
- can_msgs

## 설치 및 빌드

### 1. 시스템 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    libeigen3-dev \
    libboost-all-dev
```

### 2. ROS2 환경 설정

```bash
source /opt/ros/humble/setup.bash
```

### 3. 워크스페이스 빌드

```bash
mkdir -p catkin_ws/src
cd ~/catkin_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4. 환경 설정

```bash
source ~/catkin_ws/install/setup.bash
```

## 시스템 실행

### 기본 제어 시스템 실행

```bash
ros2 launch amr_core amr_bringup.launch.py
```

### 조작 실행

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=manual_vel
```

### 자율주행 시스템 실행 (TBD)

```bash
ros2 launch amr_navigation amr_navigation.launch.py
```

### SLAM 시스템 실행 (TBD)

```bash
ros2 launch amr_cartographer amr_cartographer.launch.py
```

## 시스템 구성

### 오도메트리 시스템

- 기본 오도메트리: 단순 적분 기반 위치 추정
- EKF 오도메트리:
  - 상태 벡터: [x, y, yaw, v, w]
  - 프로세스 노이즈: Q = diag(0.01, 0.01, 0.01, 0.05, 0.05)
  - 측정 노이즈: R = diag(0.01, 0.01, 0.01)

### 내비게이션 시스템

- Nav2 기반 경로 계획
- 실시간 파라미터 업데이트
- TF2 기반 위치 추적
- 목표 지점 도달 제어

### SLAM 시스템

- 실시간 맵핑
- 루프 클로저 검출
- 맵 최적화
- 위치 추정

## 파라미터 설정

### 핵심 파라미터

- `amr_core/param/`:
  - 제어 주기: 20Hz
  - 휠 파라미터: 반경, 거리, 기어비
  - 모터 파라미터: 최대 RPM, 정격 전류
  - EKF 노이즈 파라미터

### 내비게이션 파라미터

- `amr_navigation/param/`:
  - 경로 계획 파라미터
  - 장애물 회피 파라미터
  - 제어 게인

## 시스템 모니터링

### 로깅

- ROS2 로깅 시스템 활용
- 실시간 상태 모니터링
- 에러 추적 및 디버깅

### 시각화

- RViz2 기반 시각화
- 실시간 데이터 플로팅
- 시스템 상태 모니터링

## 라이선스

Apache License 2.0

## 기술 지원

- 담당자: ksw
- 이메일: ksw7384@hanyang.ac.kr
- 버그 리포트: GitHub Issues
