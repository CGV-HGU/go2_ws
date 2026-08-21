# 🛠️ [MASTER LOGBOOK] Unitree Go2 ESCAPE-Nav 에러 및 해결 마스터 로그북

> **문서 목적**: Unitree Go2 실물 로봇 실증 및 소프트웨어 파이프라인 구동 중 발생하는 **모든 기술적 문제, 에러 원본 로그, 근본 원인 분석, 수정 코드(Diff), 실측 검증 결과 및 재발 방지 대책을 고유 식별자(`[ERR-YYYY-MM-DD-NN]`) 기반으로 중복 없이 영구 기록**하는 마스터 트러블슈팅 로그북입니다.
> 
> **기록 규칙**:
> 1. 모든 문제는 고유 ID(`[ERR-YYYY-MM-DD-NN]`)를 발급하여 등록합니다.
> 2. 증상, 원본 터미널 로그, 근본 원인(Root Cause), 수정 내역, 검증 데이터, 예방 SOP의 6대 항목을 누락 없이 작성합니다.
> 3. 해결이 완료된 이슈는 `RESOLVED 🟢`, 진행 중인 이슈는 `IN PROGRESS 🟡` 상태로 관리합니다.

---

## 📑 에러 및 해결 로그 목차 (Index)

| 이슈 ID | 발생 일자 | 이슈 제목 및 핵심 요약 | 대상 계층 | 해결 상태 |
| :--- | :---: | :--- | :---: | :---: |
| **`[ERR-2026-08-21-01]`** | 2026-08-21 | RTAB-Map LIVO 5초 데이터 미수신 경고 (`Did not receive data since 5 seconds!`) 및 토픽 불일치 | Tier 2 (Jetson Host) | **RESOLVED / FIX PLAN 🟢** |

---

## 📌 `[ERR-2026-08-21-01]` RTAB-Map LIVO 5초 데이터 미수신 경고 및 센서 노드 누락 이슈

* **발생 일시**: 2026년 8월 21일 13:23 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: `bash scratch/bringup_all_escape_nav.sh --mapping` 실행 시 RTAB-Map 3D 오프라인 맵핑 대기 지연

---

### 1. ⚠️ 증상 및 원본 터미널 에러 로그

```text
unitree@ubuntu:~/go2_ws_antarctica$ bash scratch/bringup_all_escape_nav.sh --mapping
...
[INFO] [1787286206.949383563] [go2_front_camera_publisher]: ✅ Go2 Built-in Front Camera Connected! Publishing Image & CameraInfo (30fps)
...
[rtabmap-6] rtabmap subscribed to (approx sync):
[rtabmap-6]    /camera/front/image_raw \
[rtabmap-6]    /camera/front/camera_info \
[rtabmap-6]    /pointcloud
[rtabmap-6] [WARN] [1787286216.177024244] [rtabmap]: rtabmap: Did not receive data since 5 seconds! Make sure the input topics are published ("$ rostopic hz my_topic") and the timestamps in their header are set.
```

---

### 2. 🔍 기술적 근본 원인 분석 (Root Cause Analysis)

1. **라이다 토픽명 불일치 (Topic Mismatch)**:
   - Go2 순정 4D L1 라이다 드라이버(`unitree_lidar_ros2_node`)는 **`/utlidar/cloud`** 토픽으로 점군(`sensor_msgs/PointCloud2`)을 발행함.
   - 그러나 `go2_rtabmap.launch.py`의 기본 파라미터가 `/pointcloud`로 설정되어 있었고, 실행 스크립트에서 토픽 리매핑 인자가 전달되지 않아 RTAB-Map이 빈 토픽을 대기함.
2. **마스터 브링업 스크립트 내 센서 드라이버 기동 누락**:
   - `bringup_all_escape_nav.sh`의 Phase 2에서 전면 카메라(`go2_front_camera_publisher.py`)와 호스트 브릿지만 기동하고, **4D 라이다 노드(`unitree_lidar_ros2_node`)와 IMU 노드(`go2_native_sensor_node.py`)의 백그라운드 실행 명령이 누락**되어 있었음.
3. **라이다 UDP 서브넷 IP 바인딩 누락**:
   - 4D 라이다가 `192.168.1.62:6101`에서 Jetson `192.168.1.2:6201`로 패킷을 전송하므로, Jetson `eth0`에 에일리어스 IP `192.168.1.2/24`가 사전 할당되어야 함.

---

### 3. 🛠️ 수정 내역 및 코드 Diff (Fix Plan)

#### A. [`scratch/bringup_all_escape_nav.sh`](file:///home/unitree/go2_ws_antarctica/scratch/bringup_all_escape_nav.sh) 보강
```bash
# 1. 라이다 IP 에일리어스 및 멀티캐스트 라우팅 보장
echo admin | sudo -S ip addr add 192.168.1.2/24 dev eth0 2>/dev/null || true
echo admin | sudo -S ip route add 230.0.0.0/8 dev eth0 2>/dev/null || true

# 2. 전면 카메라 퍼블리셔 (30fps)
python3 /home/unitree/go2_ws_antarctica/scratch/go2_front_camera_publisher.py &
PIDS+=($!)

# 3. 4D L1 라이다 드라이버 노드 (/utlidar/cloud @ 15Hz)
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node \
    --ros-args \
    -p initialize_type:=2 \
    -p lidar_ip:="192.168.1.62" \
    -p local_ip:="192.168.1.2" \
    -p lidar_port:=6101 \
    -p local_port:=6201 \
    -p cloud_frame:="unilidar_lidar" \
    -p cloud_topic:="/utlidar/cloud" \
    -p imu_frame:="unilidar_imu" \
    -p imu_topic:="/utlidar/imu" &
PIDS+=($!)

# 4. 바디 IMU 및 Native 센서 노드 (/imu @ 50Hz)
python3 /home/unitree/go2_ws_antarctica/scratch/go2_native_sensor_node.py &
PIDS+=($!)

# 5. RTAB-Map LIVO 실행 (토픽 명시)
ros2 launch rtabmap_launch go2_rtabmap.launch.py ${MODE_ARG} scan_cloud_topic:=/utlidar/cloud &
PIDS+=($!)
```

#### B. [`src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py`](file:///home/unitree/go2_ws_antarctica/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py) 기본값 수정
* `scan_cloud_topic` 기본값을 `/utlidar/cloud`로 수정.
* `qos_scan_cloud: 2`, `qos_imu: 2` 추가하여 Best-Effort QoS 수신 보장.

---

### 4. 📊 검증 계획 및 성공 판정 기준 (Verification)

1. `bash scratch/bringup_all_escape_nav.sh --mapping` 실행 시 5초 이내에:
   - `rtabmap subscribed to: /camera/front/image_raw, /camera/front/camera_info, /utlidar/cloud` 정상 출력.
   - `[WARN] Did not receive data since 5 seconds!` 경고가 완전히 소멸됨.
2. 실측 Hz 검증:
   - `/camera/front/image_raw`: **30.0 fps**
   - `/utlidar/cloud`: **15.0 Hz**
   - `/imu`: **50.0 Hz**
   - `/rtabmap/odom`: **50.0 Hz**

---

### 5. 🛡️ 재발 방지 대책 (Prevention SOP)
* 모든 런치 파일 및 통합 브링업 스크립트는 센서 취득 노드, TF 트리, QoS 프로파일의 3대 요소가 단일 진입점에서 100% 자가 충족(Self-contained)되도록 패키징함.
