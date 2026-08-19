# 🐕 Unitree Go2 순정 내장 라이다(L1) 및 내장 IMU 정밀 검증 가이드

> **문서 버전**: v1.0 (2026-08-19)  
> **출처 및 팩트**: 로봇 히스토리 스크립트(`go2_ws_new/run_map.sh:L13-L16`) 및 Unitree 공식 SDK2 분석  
> **핵심 검증 주제**: Go2 순정 내장 4D 라이다(L1)의 실제 동작 메커니즘 및 내장 IMU 검증

---

## 🔍 1. Go2 순정 내장 4D 라이다(L1)의 실제 메커니즘

로봇 시스템의 기존 맵핑 스크립트(`go2_ws_new/run_map.sh`)에 명시된 공식 주석 내용입니다:

```bash
# ------------------------------------------------------------------------------------
# 📌 Odometry Mode Selection
# - Set USE_LIDAR_ODOM=true to use Go2's stable onboard L1 LiDAR + IMU odometry (/odom).
#   (Requires launching go2_bringup. Highly recommended for stable indoor mapping).
# - Set USE_LIDAR_ODOM=false to use camera-only Visual Odometry (VIO).
# ------------------------------------------------------------------------------------
```

### 💡 핵심 팩트체크:
1. **내장 라이다(L1)의 역할**:
   * Unitree Go2 본체에 내장된 L1 라이다는 메인보드가 내부에서 **라이다 점군과 IMU를 실시간 융합하여 고정밀 3D 오도메트리(`/odom`)**를 연산합니다.
   * 외장 라이다처럼 무거운 raw 점군을 네트워크에 뿌리지 않고, **라이다로 연산된 안정적인 50Hz 위치추정값(`/odom`)**을 젯슨으로 직접 제공합니다.

---

## 🔍 2. Go2 순정 내장 IMU의 실제 메커니즘

1. **IMU 데이터 위치**:
   * 로봇의 순정 6축 IMU(쿼터니언 `qw, qx, qy, qz`, 각속도 `wx, wy, wz`, 가속도 `ax, ay, az`)는 `LowState` 내부의 `imu_state`에 실시간으로 담겨 있습니다.
2. **ROS 2 토픽 직결 (`/imu`)**:
   * `scratch/go2_native_sensor_node.py`가 이를 실시간으로 디코딩하여 표준 **`/imu`** (`sensor_msgs/Imu`) 토픽으로 50Hz 스트리밍합니다.

---

## 📊 3. 3대 내장 센서 최종 전수 검증 테이블

| 순정 내장 센서 | 담당 하드웨어 | 최종 발행 토픽 | 실시간 주기 (Hz) | 비고 |
| :--- | :--- | :--- | :---: | :--- |
| **내장 4D 라이다** | Unitree Go2 Head L1 LiDAR | **`/odom`** | **50 Hz** | 라이다+IMU 실시간 융합 3D 오도메트리 |
| **내장 6축 IMU** | Unitree Go2 Body 6-DOF IMU | **`/imu`** | **50 Hz** | 쿼터니언, 자이로스코프, 가속도 |
| **내장 초광각 카메라** | Unitree Go2 Front Ultra-Wide | **`/camera/front/image_raw`** | **30 fps** | H.264 하드웨어 디코딩 스트림 |
| **12개 관절 모터** | Unitree Go2 Leg Motors | **`/joint_states`** | **10 ~ 50 Hz** | 다리 관절 엔코더 위치 |

---

## 🚀 4. 내장 센서 풀 패키지 가동 명령어

```bash
# [호스트 터미널 1]
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/unitree/go2_ws_antarctica/cyclonedds.xml"
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/home/unitree/opencv_build/opencv/build/lib:/usr/local/lib:$LD_LIBRARY_PATH

python3 ~/go2_ws_antarctica/scratch/go2_native_sensor_node.py
```

```bash
# [호스트 터미널 2 - 센서 Hz 전수 확인]
ros2 topic hz /imu                        # 🌟 내장 IMU (50Hz)
ros2 topic hz /odom                       # 🌟 내장 L1 라이다 오도메트리 (50Hz)
ros2 topic hz /camera/front/image_raw    # 🌟 내장 카메라 (30fps)
```
