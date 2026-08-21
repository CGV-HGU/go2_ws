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
| **`[ERR-2026-08-21-01]`** | 2026-08-21 | RTAB-Map LIVO 5초 데이터 미수신 경고 및 라이다/IMU 노드 기동 누락 | Tier 2 (Jetson Host) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-02]`** | 2026-08-21 | RTAB-Map QoS 파라미터명 불일치(`qos_scan_cloud` vs `qos_scan`) 및 큐 크기(10 ➔ 50) 협소 이슈 | Tier 2 (Jetson Host) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-03]`** | 2026-08-21 | 카메라 스트림 중복 취득 충돌(`Messages out of order`) 및 라이다 SDK/DDS 50Hz 융합 오도메트리 연동 | Tier 2 (Jetson Host) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-04]`** | 2026-08-21 | 정지/와상 상태 TF (`odom` ➔ `base_link`) 단절 경고 및 스레드 기반 카메라/카메라인포 30fps 동기화 완성 | Tier 2 (Jetson Host) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-05]`** | 2026-08-21 | RTAB-Map 3D 점군 맵 및 2D 점유격자지도(Occupancy Grid) 미생성 이슈 및 `Grid/Sensor` 3D 라이다 융합 활성화 | Tier 2 (Jetson Host) | **RESOLVED 🟢** |

---

## 📌 `[ERR-2026-08-21-01]` RTAB-Map LIVO 5초 데이터 미수신 경고 및 센서 노드 누락 이슈

* **발생 일시**: 2026년 8월 21일 13:23 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: `bash scratch/bringup_all_escape_nav.sh --mapping` 실행 시 RTAB-Map 3D 오프라인 맵핑 대기 지연

### 1. ⚠️ 증상 및 원본 터미널 에러 로그
```text
[rtabmap-6] rtabmap subscribed to (approx sync):
[rtabmap-6]    /camera/front/image_raw \
[rtabmap-6]    /camera/front/camera_info \
[rtabmap-6]    /pointcloud
[rtabmap-6] [WARN] [1787286216.177024244] [rtabmap]: rtabmap: Did not receive data since 5 seconds!
```

### 2. 🔍 기술적 근본 원인 분석
1. **라이다 토픽명 불일치**: Go2 순정 4D 라이다 드라이버는 `/utlidar/cloud`로 발행하나 RTAB-Map이 기본값 `/pointcloud`를 대기함.
2. **센서 드라이버 기동 누락**: `bringup_all_escape_nav.sh`에서 `unitree_lidar_ros2_node` 및 `go2_native_sensor_node.py` 실행 누락.
3. **라이다 IP 바인딩 누락**: `eth0`에 `192.168.1.2/24` 미할당.

### 3. 🛠️ 해결 조치
* `bringup_all_escape_nav.sh`에 IP 에일리어스, 라이다 노드(15Hz), IMU 노드(50Hz) 기동 및 `scan_cloud_topic:=/utlidar/cloud` 인자 결합.

---

## 📌 `[ERR-2026-08-21-02]` RTAB-Map QoS 파라미터명 불일치 및 큐 크기 협소 이슈

* **발생 일시**: 2026년 8월 21일 13:37 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: `/utlidar/cloud` 토픽 연결 후에도 `qos_scan = 0` (Reliable)으로 고정되어 Best-Effort 라이다 점군이 드랍되는 현상

### 1. ⚠️ 증상 및 원본 터미널 에러 로그
```text
[rtabmap-6] [INFO] rtabmap: qos_image       = 2
[rtabmap-6] [INFO] rtabmap: qos_camera_info = 2
[rtabmap-6] [INFO] rtabmap: qos_scan        = 0  <-- Reliable(0)로 남아 라이다 패킷 Drop!
[rtabmap-6] [INFO] [WARN] Messages of type 0 arrived out of order
[rtabmap-6] [WARN] rtabmap: Did not receive data since 5 seconds!
```

### 2. 🔍 기술적 근본 원인 분석
1. **C++ 파라미터명 불일치**: `rtabmap_sync/CommonDataSubscriber.cpp` 라인 382에 정의된 파라미터명은 `qos_scan_cloud`가 아닌 **`qos_scan`**임.
2. **DDS QoS 불호환**: Best-Effort(SensorDataQoS)로 발행되는 라이다 점군을 Reliable(0) 구독자가 거부함.

### 3. 🛠️ 해결 조치 및 수정 코드
[`src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py`](file:///home/unitree/go2_ws_antarctica/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py):
```python
'qos': 2, 'qos_scan': 2, 'qos_imu': 2, 'qos_image': 2, 'qos_camera_info': 2, 'qos_odom': 2,
'approx_sync': True, 'approx_sync_max_interval': 0.1, 'queue_size': 50
```

---

## 📌 `[ERR-2026-08-21-03]` 카메라 스트림 중복 취득 충돌 및 SDK/DDS 50Hz 융합 오도메트리 파이프라인 확립

* **발생 일시**: 2026년 8월 21일 13:43 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: 카메라 타임스탬프 역전(`Messages of type 0 arrived out of order`) 및 라이다 Raw 스트리밍 미가동 시 맵핑 블로킹

### 1. ⚠️ 증상 및 원본 터미널 에러 로그
```text
[INFO] [go2_native_sensor_node]: ✅ [CAMERA] Unitree Go2 Front Camera Connected! (30 fps)
[WARN] Messages of type 0 arrived out of order (will print only once)
[WARN] rtabmap: Did not receive data since 5 seconds!
```

### 2. 🔍 기술적 근본 원인 및 SDK 심층 분석
1. **카메라 이중 오픈 충돌**:
   - `go2_front_camera_publisher.py`와 `go2_native_sensor_node.py`가 동시에 GStreamer 포트 1720을 오픈하고 `/camera/front/image_raw`를 동시 발행함.
2. **Unitree L1 라이다 하드웨어 및 SDK 스트리밍 구조**:
   - Unitree 공식 SDK (`src/unilidar_sdk2/unitree_lidar_sdk/examples/example_lidar_udp.cpp`) 분석 결과, 라이다는 UDP 포트 6101/6201로 `startLidarRotation()` 제어 패킷을 수신해야 모터가 회전함.
   - Go2 메인보드는 DSP 내부에서 라이다+IMU+엔코더를 **50Hz 고정밀 오도메트리(`/odom`)**로 실시간 융합하여 CycloneDDS로 송출함.

### 3. 🛠️ 해결 조치 및 수정 코드
1. **[`scratch/go2_native_sensor_node.py`](file:///home/unitree/go2_ws_antarctica/scratch/go2_native_sensor_node.py)**:
   - 중복 카메라 코드 제거 및 `lowstate`/`sportmodestate`에 Best-Effort QoS 적용.
2. **[`go2_rtabmap.launch.py`](file:///home/unitree/go2_ws_antarctica/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py)**:
   - `subscribe_odom: True`, `subscribe_scan_cloud: False`, `('odom', '/odom')` 리매핑.

---

## 📌 `[ERR-2026-08-21-04]` 정지/와상 상태 TF (`odom` ➔ `base_link`) 단절 경고 및 스레드 기반 30fps 동기화 완성

* **발생 일시**: 2026년 8월 21일 13:53 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: 로봇 와상/대기 상태에서 RTAB-Map TF 트리 단절(`Could not find a connection between odom and base_link`) 및 5초 동기화 지연

### 1. ⚠️ 증상 및 원본 터미널 에러 로그
```text
[rtabmap-6] [ WARN] (2026-08-21 13:56:29.120) MsgConversion.cpp:1758::getTransform() (can transform odom -> base_link?) Could not find a connection between 'odom' and 'base_link' because they are not part of the same tree.
```

### 2. 🔍 기술적 근본 원인 분석
1. **정지/와상 상태 TF 퍼블리시 부재**:
   - `sportmodestate` 패킷 수신 콜백 안에서만 TF를 쐈기 때문에, 로봇이 눕혀져 있거나 대기 상태일 때는 TF가 발행되지 않아 RTAB-Map의 TF 트리가 끊어짐.
2. **카메라 인포 동기화 안정성**:
   - 단일 인스턴스 `CameraInfo`를 재사용하면서 타임스탬프 갱신이 간헐적으로 지연됨.

### 3. 🛠️ 해결 조치 및 코드 보강
1. **[`scratch/go2_native_sensor_node.py`](file:///home/unitree/go2_ws_antarctica/scratch/go2_native_sensor_node.py)**:
   - 전용 **50Hz 고정 주기 타이머(`self.tf_timer = self.create_timer(1.0/50.0, self.tf_timer_callback)`)**를 탑재하여 로봇이 눕혀진 대기 상태에서도 `odom` ➔ `base_link` TF가 영구 지속되도록 구현.
2. **[`scratch/go2_front_camera_publisher.py`](file:///home/unitree/go2_ws_antarctica/scratch/go2_front_camera_publisher.py)**:
   - 전용 워커 스레드(`threading.Thread`)를 분리하여 블로킹 없는 30fps 하드웨어 디코딩 보장.
   - 매 프레임 신규 `CameraInfo` 객체 생성 및 SensorData QoS 발행.

---

## 📌 `[ERR-2026-08-21-05]` RTAB-Map 3D 점군 맵 및 2D 점유격자지도(Occupancy Grid) 미생성 이슈 및 `Grid/Sensor` 3D 라이다 융합 활성화

* **발생 일시**: 2026년 8월 21일 14:43 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: RTAB-Map GUI 실행 시 노드/외형 그래프만 생성되고 3D 점군 맵 및 2D 격자 맵이 화면에 렌더링되지 않는 현상

### 1. ⚠️ 증상 및 현장 보고
* RTAB-Map 3D Visualizer(`rtabmap_viz`)를 확인한 결과, 3D Point Cloud Map과 2D Occupancy Grid Map은 비어 있고 카메라 기반의 외형 노드(Node ID 1, 2...)만 누적되는 현상 발생.

### 2. 🔍 기술적 근본 원인 분석
1. **단안 RGB 카메라의 공간 정보 부재**:
   - Go2 전면 카메라는 컬러 영상(2D RGB)만 취득하는 단안 카메라이므로 픽셀별 거리(Depth) 정보가 없음.
   - 거리 정보가 없는 상태에서는 3D 공간 복원이나 2D 점유 격자 지도를 투영할 수 없음.
2. **라이다 점군 구독 및 Grid 파라미터 비활성화**:
   - `go2_rtabmap.launch.py`에 `'subscribe_scan_cloud': False`로 되어 있어 점군 수신이 차단되어 있었음.

### 3. 🛠️ 해결 조치 및 수정 코드
[`src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py`](file:///home/unitree/go2_ws_antarctica/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py):
```python
# 3D Point Cloud Map & 2D Occupancy Grid Generation Parameters
'subscribe_scan_cloud': True,
'Grid/Sensor': '0',            # 0 = scan_cloud (3D Point Cloud LiDAR)
'Grid/RangeMax': '15.0',       # Max range 15m
'Grid/RangeMin': '0.2',
'Grid/CellSize': '0.05',       # 5cm grid resolution
'Grid/3D': 'true',             # Real-time 3D voxel/octomap
'Grid/RayTracing': 'true',     # Ray tracing for clearing free space
'Grid/NormalsSegmentation': 'false',
'Icp/PointToPlane': 'true',
```

### 4. 📱 Go2 본체 라이다 송출 활성화 방법 (SOP)
* 스마트폰 **Unitree Go2 공식 앱 ➔ [Device] ➔ [Data] ➔ [Unitree Perception LiDAR]** 토글 스위치 **ON**.
* 메인보드(`192.168.123.161`)가 `/utlidar/cloud` 3D 점군을 CycloneDDS로 즉시 송출하여 RTAB-Map 3D/2D 지도가 실시간으로 생성됨.
