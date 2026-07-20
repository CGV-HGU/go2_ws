# IMU 및 카메라 디버깅 가이드 (IMU & Camera Debugging Guide)

`run_map_default.sh` 실행 시 **"2단계 릴레이 노드 실행 후 데이터 수신 대기 상태(still waiting for imu/data_raw)"**에서 멈추는 문제를 해결하기 위한 자가 진단 및 해결 가이드입니다.

---

## 📋 3단계 디버깅 체크리스트

### 1단계: 카메라 노드(1번째 탭) 로그 확인
Gnome Terminal의 첫 번째 탭(RealSense 카메라 구동 화면)에 에러 메시지가 있는지 확인합니다.
* **주요 확인 항목**: `Hardware Notification: USB ...`, `failed to open device`, 또는 특정 파라미터 에러.
* **의심되는 원인**: 
  - Jetson 보드와 카메라가 USB 2.0 포트로 연결되어 대역폭 부족으로 작동이 멈춘 경우.
  - 이 경우 해상도 프레임 프로필을 `30`에서 `15`로 낮춰야 정상 작동합니다. (예: `depth_module.profile:=640x480x15`)

---

### 2단계: 카메라 IMU 토픽 생존 여부 확인
새 터미널을 열고 아래 명령어를 입력하여 카메라 센서에서 실제 IMU 데이터가 나오고 있는지 확인합니다.
```bash
# 1. ROS 2 및 워크스페이스 환경 소싱
source /opt/ros/foxy/setup.bash
source ~/go2_ws/install/setup.bash

# 2. 카메라 IMU 토픽 수신 확인
ros2 topic hz /camera/imu
```
* **결과 판별**:
  - **아무 반응이 없거나 에러 발생 (0 Hz)**: 카메라 하드웨어 자체 혹은 드라이버 구동 단계의 문제입니다. (1단계를 다시 확인해야 함)
  - **정상적으로 숫자가 올라감 (200~400 Hz)**: 카메라는 정상 구동 중이나, 릴레이 노드(`imu_relay.py`) 혹은 그 이후 필터 노드와의 통신 연결에 문제가 있습니다. (3단계 진행)

---

### 3단계: 토픽 QoS 정보 확인
카메라는 동작하지만 릴레이 노드가 받지 못한다면 QoS(통신 정책) 불일치 문제일 가능성이 매우 높습니다.
```bash
ros2 topic info /camera/imu -v
```
* **확인할 항목**: `Reliability` 정책
  - `imu_relay.py`는 기본적으로 **`BEST_EFFORT`**로 구독하게 되어 있습니다.
  - 만약 카메라 구동 옵션 중 `accel_qos:=SYSTEM_DEFAULT` 설정으로 인해 카메라가 **`RELIABLE`** 또는 다른 커스텀 QoS로 토픽을 발행하고 있고 이로 인해 매칭에 실패하는지 확인합니다.

---

## 🛠️ 예상되는 해결 방안 (Troubleshooting)

### 시나리오 A: USB 2.0 연결로 인해 카메라가 켜지지 않는 경우
`run_map_default.sh` 내의 카메라 프로필 설정을 `15 FPS`로 낮추어 해결합니다.
```bash
# run_map_default.sh 수정 부분
depth_module.profile:=640x480x15 \
rgb_camera.profile:=640x480x15 \
```

### 시나리오 B: QoS 불일치로 릴레이가 동작하지 않는 경우
`imu_relay.py` 코드 내부의 `camera_qos` 신뢰성 옵션을 변경해 줍니다.
```python
# imu_relay.py 수정 제안 (BEST_EFFORT -> SYSTEM_DEFAULT 매칭 필요 시)
camera_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, # 또는 SYSTEM_DEFAULT에 맞춤
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```
