# 📡 [Unitree Go2] 4D LiDAR L2 & High-Rate IMU/Odom Topic Publishing & Cross-Verification Plan

> **문서 버전**: v1.0.0  
> **최종 수정일**: 2026-08-21  
> **타겟 브랜치**: `antarctica` (`origin/antarctica`)  
> **시스템 아키텍처**: Unitree Go2 (Jetson Orin NX Foxy ➔ Docker Jazzy ➔ Remote RTX Pro 6000)

---

## 1. 🎯 목표 (Goal Description)
본 계획서는 Unitree Go2 순정 하드웨어의 **4D LiDAR L2 (UTLiDAR)** 및 **초고속 IMU (500Hz) / 오도메트리 (50Hz)**의 실시간 토픽 발행 주파수를 100% 정상화하고, RTAB-Map LIVO 3D SLAM 파이프라인과의 동기화 성능을 교차 검증하는 것을 목표로 합니다.

* **4D LiDAR L2 3D 점군 (`/utlidar/cloud`)**: **10 ~ 15 Hz**
* **바디 IMU (`/imu`, `/utlidar/imu`)**: **50 ~ 500 Hz**
* **다리 기구학 융합 오도메트리 (`/odom`)**: **50 Hz**
* **전면 광각 RGB 카메라 (`/camera/front/image_raw`)**: **30 fps (30 Hz)**
* **6자유도 TF 좌표계 변환 (`/tf`)**: **70 ~ 140 Hz**

---

## 2. 🔍 하드웨어 및 통신 구조 근본 원인 분석 (Root Cause Analysis)

### 2.1 독립형 라이다(`192.168.1.62`) vs Go2 본체 내장 라이다(`192.168.123.161`)의 차이
```
 ┌─────────────────────────────────────────────────────────────────────────────────────────────┐
 │                         Unitree Go2 Hardware Routing Architecture                           │
 ├──────────────────────────────────────────────────────────────┬──────────────────────────────┤
 │ ❌ Standalone Unilidar Setup                                 │ 🟢 Go2 Robot-Integrated Setup│
 │ (unilidar_sdk2 default config)                               │ (Our actual robot hardware)  │
 │                                                              │                              │
 │  [Jetson eth0: 192.168.1.2]                                  │  [Jetson eth0:192.168.123.99]│
 │          │ (Direct UDP cable)                                │               │              │
 │          ▼                                                   │               ▼ (Switch Hub) │
 │  [LiDAR: 192.168.1.62:6101]                                  │  [Go2 Mainboard:123.161]     │
 │  * ARP Scan: <incomplete> (Not wired as standalone bench unit)│     ├── 4D LiDAR L2 (Internal)│
 │                                                              │     ├── Body IMU 500Hz       │
 │                                                              │     └── 12 Motor Encoders    │
 └──────────────────────────────────────────────────────────────┴──────────────────────────────┘
```

1. **라이다 패킷 수신 문제 (`bind udp port failed` / 0Hz)**:
   * `unitree_lidar_ros2_node`의 초기 런치 파일은 독립 벤치탑 IP(`192.168.1.62`)를 타겟팅하였으나, 실물 Go2 로봇에서는 L2 라이다가 **메인보드 MCU(`192.168.123.161`)와 직결**되어 내부 처리 후 브로드캐스트됨.
   * 백그라운드 좀비 프로세스가 UDP `6201` 포트를 점유하고 있던 문제를 `fuser -k 6201/udp`로 해제 조치.

2. **IMU 및 오도메트리 수신 문제 (Python `rclpy` 드랍)**:
   * Go2 메인보드는 `rt/lowstate`와 `rt/sportmodestate`를 순수 C++ IDL 바이너리 구조체로 송출함.
   * Python `rclpy`에서는 DDS 타입 해시 불일치로 드랍되므로, **C++ Native 드라이버(`unitree_sdk2`) 기반 브리지 노드**로 직접 바인딩하여 표준 ROS 2 토픽으로 변환 송출함.

3. **OpenCV glibc 링킹 및 QoS 정책 불일치**:
   * 터미널 CLI 도구(`ros2 topic hz`)는 기본적으로 `RELIABLE`을 요구하는데 퍼블리셔가 `BEST_EFFORT`로 발행하여 DDS 계층에서 드랍됨 ➔ 표준 `RELIABLE` QoS로 전면 개편.
   * `/etc/ld.so.conf.d/opencv.conf` 등록 및 `sudo ldconfig`로 OpenCV `dlopen` 라이브러리 경로 영구 해결.

---

## 3. 🛠️ 구현 및 통합 계획 (Implementation Strategy)

### 3.1 C++ Native 센서 브리지 및 4D 라이다 연동
1. **[`scratch/go2_sdk2_sensor_bridge.cpp`](file:///home/unitree/go2_ws_antarctica/scratch/go2_sdk2_sensor_bridge.cpp)**:
   - `unitree_sdk2`를 통해 `eth0` DDS 도메인 0 직결.
   - `rt/lowstate` (500Hz) ➔ `/imu` (`sensor_msgs/Imu` @ 500Hz)
   - `rt/sportmodestate` (50Hz) ➔ `/odom` (`nav_msgs/Odometry` @ 50Hz) 및 `odom` ➔ `base_link` TF 연속 브로드캐스트.
2. **[`src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py`](file:///home/unitree/go2_ws_antarctica/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py)**:
   - `subscribe_scan_cloud: true`, `scan_cloud_topic: /utlidar/cloud` 기본 활성화.
   - `Grid/Sensor: "0"` (3D Point Cloud LiDAR 그리드 생성기) 및 ICP Point-to-Plane 3D 매칭 활성화.

---

## 4. 🧪 교차 검증 프로토콜 (Cross-Verification Protocol)

| 검증 단계 | 검증 항목 | 명령어 | 기대 수치 (Target) |
| :---: | :--- | :--- | :---: |
| **Step 1** | 전면 카메라 영상 | `ros2 topic hz /camera/front/image_raw` | **30.0 Hz** |
| **Step 2** | 카메라 내부 파라미터 | `ros2 topic hz /camera/front/camera_info` | **30.0 Hz** |
| **Step 3** | 바디 IMU 스트림 | `ros2 topic hz /imu` | **50 ~ 500 Hz** |
| **Step 4** | 다리 기구학 오도메트리 | `ros2 topic hz /odom` | **50 Hz** |
| **Step 5** | 6자유도 TF 좌표 변환 | `ros2 topic hz /tf` | **70 ~ 140 Hz** |
| **Step 6** | RTAB-Map 3D 점군 맵 | `ros2 topic hz /cloud_map` | **2 ~ 5 Hz** |
| **Step 7** | 2D 점유 격자 지도 | `ros2 topic hz /map` | **Latched / 동적 갱신** |
| **Step 8** | 3D SLAM DB 생성 | `python3 scratch/inspect_rtabmap_db.py` | **10+ 키프레임, PLY 점군 생성** |

---

## 5. 🚀 커밋 및 푸시 이력
* **계획서 등록 커밋**: `docs: add comprehensive 4D LiDAR L2 and high-rate IMU/Odom topic publishing plan`
* **타겟 리포지토리**: `https://github.com/CGV-HGU/go2_ws.git` (`branch: antarctica`)
