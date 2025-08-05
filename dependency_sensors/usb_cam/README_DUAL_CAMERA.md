# Dual USB Camera Launch

이 문서는 두 개의 USB 카메라를 동시에 실행하는 방법을 설명합니다.

## 설정

### 카메라 장치
- **Camera Right**: `/dev/video0` 사용
- **Camera Left**: `/dev/video4` 사용

### URDF 링크 매핑
- **Camera Right**: `camera_r_optical_link` (우측 카메라)
- **Camera Left**: `camera_l_optical_link` (좌측 카메라)

### 토픽 구조
각 카메라는 다음과 같은 토픽을 발행합니다:

#### Camera Right
- `camera_right/image_raw` - 원본 이미지
- `camera_right/image_compressed` - 압축된 이미지
- `camera_right/camera_info` - 카메라 정보

#### Camera Left
- `camera_left/image_raw` - 원본 이미지
- `camera_left/image_compressed` - 압축된 이미지
- `camera_left/camera_info` - 카메라 정보

## 사용법

### 1. 단일 카메라 실행
```bash
ros2 launch usb_cam camera.launch.py
```

### 2. 듀얼 카메라 실행
```bash
ros2 launch usb_cam dual_camera.launch.py
```

### 3. 토픽 확인
```bash
# 모든 카메라 토픽 확인
ros2 topic list | grep camera

# 특정 카메라 이미지 확인
ros2 run rqt_image_view rqt_image_view
```

### 4. 카메라 정보 확인
```bash
# 카메라 1 정보
ros2 topic echo /camera1/camera_info

# 카메라 2 정보
ros2 topic echo /camera2/camera_info
```

### 5. TF 트리 확인
```bash
# TF 트리 시각화
ros2 run tf2_tools view_frames

# 특정 프레임 관계 확인
ros2 run tf2_ros tf2_echo base_link camera_r_optical_link
ros2 run tf2_ros tf2_echo base_link camera_l_optical_link
```

## 설정 파일

### camera_right.yaml (Camera - Right)
- 장치: `/dev/video0`
- 프레임레이트: 30 FPS
- 해상도: 640x480
- 프레임 ID: `camera_r_optical_link`

### camera_left.yaml (Camera - Left)
- 장치: `/dev/video4`
- 프레임레이트: 30 FPS
- 해상도: 640x480
- 프레임 ID: `camera_l_optical_link`

## 문제 해결

### 1. 카메라가 인식되지 않는 경우
```bash
# 사용 가능한 비디오 장치 확인
ls /dev/video*

# 카메라 권한 확인
sudo usermod -a -G video $USER
```

### 2. 권한 문제
```bash
# 카메라 장치에 대한 권한 설정
sudo chmod 666 /dev/video0 /dev/video1
```

### 3. 카메라가 사용 중인 경우
```bash
# 카메라 사용 프로세스 확인
lsof /dev/video0
lsof /dev/video4
```

### 4. TF 프레임 문제
```bash
# TF 프레임 확인
ros2 run tf2_ros tf2_echo base_link camera_r_optical_link
ros2 run tf2_ros tf2_echo base_link camera_l_optical_link
```

## 추가 설정

### 다른 해상도 사용
`params_1.yaml`과 `params_2.yaml`에서 다음 값을 수정:
```yaml
image_width: 1280
image_height: 720
```

### 다른 프레임레이트 사용
```yaml
framerate: 15.0  # 15 FPS로 변경
```

### 다른 비디오 장치 사용
```yaml
video_device: "/dev/video2"  # 다른 장치 사용
``` 