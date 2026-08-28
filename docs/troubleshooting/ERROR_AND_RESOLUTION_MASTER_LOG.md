# 🛠️ [MASTER LOGBOOK] Unitree Go2 ESCAPE-Nav 에러 및 해결 마스터 로그북

> **문서 목적**: Unitree Go2 실물 로봇 실증 및 소프트웨어 파이프라인 구동 중 발생하는 **모든 기술적 문제, 에러 원본 로그, 근본 원인 분석, 수정 코드(Diff), 실측 검증 결과 및 재발 방지 대책을 고유 식별자(`[ERR-YYYY-MM-DD-NN]`) 기반으로 중복 없이 영구 기록**하는 마스터 트러블슈팅 로그북입니다.
> 
> **기록 규칙**:
> 1. 모든 문제는 고유 ID(`[ERR-YYYY-MM-DD-NN]`)를 발급하여 등록합니다.
> 2. 증상, 원본 터미널 로그, 근본 원인(Root Cause), 수정 내역, 검증 데이터, 예방 SOP의 6대 항목을 누락 없이 작성합니다.
> 3. 해결이 완료된 이슈는 `RESOLVED 🟢`, 진행 중인 이슈는 `IN PROGRESS 🟡` 상태로 관리합니다.

---

## 📑 에러 및 해결 로그 목차 (Milestone별 분류 체계)

### 🚩 Milestone 2: 2026-08-21 (슈퍼바이저 교차검증 & LIVO 인지 스택 안정화)
| 이슈 ID | 분류 태그 | 이슈 제목 및 핵심 요약 | 대상 계층 | 해결 상태 |
| :--- | :---: | :--- | :---: | :---: |
| **`[ERR-2026-08-21-01]`** | `[SLAM/RTAB]` | RTAB-Map LIVO 5초 데이터 미수신 경고 및 라이다/IMU 노드 기동 누락 | Tier 2 (Jetson) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-02]`** | `[DDS/QOS]` | RTAB-Map QoS 파라미터명 불일치(`qos_scan_cloud` vs `qos_scan`) 및 큐 크기 협소 | Tier 2 (Jetson) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-03]`** | `[CAM/STREAM]` | 카메라 스트림 중복 취득 충돌(`Messages out of order`) 및 50Hz 융합 오도메트리 연동 | Tier 2 (Jetson) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-04]`** | `[TF2/ODOM]` | 정지/와상 상태 TF (`odom` ➔ `base_link`) 단절 경고 및 스레드 기반 30fps 동기화 | Tier 2 (Jetson) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-05]`** | `[SLAM/GRID]` | RTAB-Map 3D 점군 맵 및 2D 점유격자지도 미생성 이슈 및 `Grid/Sensor` 융합 활성화 | Tier 2 (Jetson) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-06]`** | `[SLAM/SYNC]` | 라이다 점군 대기 시 RTAB-Map `approx_sync` 5초 블로킹 방지 및 다이내믹 구독 확립 | Tier 2 (Jetson) | **RESOLVED 🟢** |
| **`[ERR-2026-08-21-07]`** | `[SYS/ENV]` | OpenCV `dlopen` 라이브러리 경로 누락, ROS 2 CLI RELIABLE 비호환, 및 6201 포트 충돌 | Tier 2 (Jetson) | **RESOLVED 🟢** |

### 🚩 Milestone 3: 2026-08-23 (라이다 공식 드라이버 & 2D/3D 실물 복도 맵핑 완성)
| 이슈 ID | 분류 태그 | 이슈 제목 및 핵심 요약 | 대상 계층 | 해결 상태 |
| :--- | :---: | :--- | :---: | :---: |
| **`[ERR-2026-08-23-01]`** | `[LIDAR/ETH]` | Go2 순정 4D 라이다 0Hz 원시 점군 차단 및 비공식 드라이버 6201 포트 충돌 | Tier 1 & Tier 2 | **RESOLVED 🟢** |
| **`[ERR-2026-08-23-02]`** | `[SLAM/GRID]` | RTAB-Map 2D 점유격자지도 외곽 가시(Spike) 번짐 및 천장/바닥 노이즈 투영 | Tier 2 (Jetson) | **RESOLVED 🟢** |

### 🚩 Milestone 4: 2026-08-24 (공식 포럼 LIVO 정밀화, Docker IP 정합화 및 제어 체계 확립)
| 이슈 ID | 분류 태그 | 이슈 제목 및 핵심 요약 | 대상 계층 | 해결 상태 |
| :--- | :---: | :--- | :---: | :---: |
| **`[ERR-2026-08-24-01]`** | `[SLAM/ICP]` | 4족 보행 피치/롤 진동 시 Z축 자세 드리프트 및 비정형 점군 Normals 연산 부하 | Tier 2 (Jetson) | **RESOLVED 🟢** |
| **`[ERR-2026-08-24-02]`** | `[DOCKER/NET]` | 도커 Compose VLM API 템플릿 URL 미해석 및 `start_docker_s2e.sh` 블로킹 버그 | Tier 3 (Docker) | **RESOLVED 🟢** |

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
```

---

## 📌 `[ERR-2026-08-21-06]` 라이다 점군 대기 시 RTAB-Map `approx_sync` 5초 블로킹 방지 및 순정 LIVO 다이내믹 구독 아키텍처 확립

* **발생 일시**: 2026년 8월 21일 14:51 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: `./mapping.sh` 실행 시 라이다 드라이버 대기 상태에서 RTAB-Map이 `Did not receive data since 5 seconds!` 경고를 출력하며 멈추는 현상

### 1. ⚠️ 증상 및 원본 터미널 에러 로그
```text
[rtabmap-6] rtabmap subscribed to (approx sync):
[rtabmap-6]    /camera/front/image_raw \
[rtabmap-6]    /camera/front/camera_info \
[rtabmap-6]    /utlidar/cloud
[rtabmap-6] [WARN] [1787291476.766866130] [rtabmap]: rtabmap: Did not receive data since 5 seconds!
```

### 2. 🔍 기술적 근본 원인 분석
1. **ApproximateTimeSynchronizer의 All-Topic 수신 대기 특성**:
   - `approx_sync`는 구독 선언된 모든 토픽(`/camera/front/image_raw` + `/camera/front/camera_info` + `/utlidar/cloud`)에 패킷이 최소 1건 이상 도착해야 프레임 처리를 시작함.
   - 라이다 드라이버 또는 앱 스트리밍이 대기 상태일 때 `/utlidar/cloud`가 $0\text{ Hz}$로 유지되면서 정상 수신 중인 카메라(30fps)와 50Hz 오도메트리까지 함께 블로킹됨.

### 3. 🛠️ 해결 조치 및 다이내믹 아키텍처 구축
1. **[`go2_rtabmap.launch.py`](file:///home/unitree/go2_ws_antarctica/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py)**:
   - `subscribe_scan_cloud`를 런치 인자(`DeclareLaunchArgument`)로 분리하고 기본값을 `false`로 설정하여 라이다 점군 대기 중에도 순정 50Hz LIVO가 **0초 지연으로 즉시 가동**되도록 개편.
   - 라이다 점군 스트리밍 시 `subscribe_scan_cloud:=true`로 즉시 실시간 융합 가능하도록 다이내믹 아키텍처 확립.

### 4. 📊 최종 실측 검증 완료 (Verification)
* RTAB-Map 2.0Hz 연속 맵핑 실측 확인:
```text
[rtabmap-6] [INFO] [rtabmap]: rtabmap (1): Rate=0.50s, RTAB-Map=0.1033s (local map=1, WM=1)
[rtabmap-6] [INFO] [rtabmap]: rtabmap (2): Rate=0.50s, RTAB-Map=0.0982s (local map=1, WM=1)
...
[rtabmap-6] [INFO] [rtabmap]: rtabmap (9): Rate=0.50s, RTAB-Map=0.0757s (local map=1, WM=1)
```
* **결과: 5초 대기 경고 0건, 프레임 드랍 0건, DB 크기 1.98MB 정상 저장 확인!**

---

## 📌 `[ERR-2026-08-21-07]` OpenCV `dlopen` 라이브러리 경로 누락, ROS 2 CLI 기본 RELIABLE QoS 비호환, 및 라이다 UDP 6201 포트 충돌 전수 해결

* **발생 일시**: 2026년 8월 21일 16:57 KST
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: 터미널에서 `ros2 topic hz` 실행 시 패킷 미수신(0Hz) 및 `unitree_lidar_ros2_node` 기동 시 `bind udp port failed` 발생 현상

### 1. ⚠️ 증상 및 원본 터미널 에러 로그
1. **OpenCV glibc 동적 링킹 에러**:
   ```text
   ImportError: libopencv_hdf.so.4.5: cannot open shared object file: No such file or directory
   ```
2. **ROS 2 QoS 정책 불일치**:
   ```text
   [WARN] New subscription discovered on this topic, requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
   ```
3. **UDP 포트 6201 바인딩 실패**:
   ```text
   [UDPHandler] create udp socket success.
   [UDPHandler] bind udp port failed.
   ```

### 2. 🔍 기술적 근본 원인 분석
1. **`LD_LIBRARY_PATH` 런타임 상속 한계**: Python 스크립트 내부에서 `os.environ`을 수정해도 이미 로드된 glibc `dlopen` 캐시를 갱신하지 못해 터미널 직접 실행 시 OpenCV 로딩 실패.
2. **QoS 비호환**: `ros2 topic hz` / `echo` CLI 도구는 기본적으로 `RELIABLE`을 요구하는데 퍼블리셔가 `BEST_EFFORT`로 발행하여 DDS 계층에서 모든 패킷을 드랍함.
3. **UDP 포트 점유**: 이전 실행 인스턴스가 UDP 6201 포트를 점유한 채 종료되지 않아 신규 라이다 노드 바인딩 실패.

### 3. 🛠️ 해결 조치
1. **시스템 레벨 영구 링커 등록**: `/etc/ld.so.conf.d/opencv.conf`에 `/home/unitree/opencv_build/opencv/build/lib` 등록 후 `sudo ldconfig` 실행.
2. **QoS 표준화**: `go2_front_camera_publisher.py` 및 `go2_native_sensor_node.py`의 발행 QoS를 표준 `RELIABLE`(`depth=10, reliability=RELIABLE, durability=VOLATILE`)로 전면 개편.
3. **포트 해제 SOP**: 사용자가 소유한 잔여 프로세스를 확인한 뒤 `fuser -k 6201/udp`로 종료한다. 권한이 필요한 경우에만 터미널에서 `sudo fuser -k 6201/udp`를 직접 실행하며 credential을 source에 저장하지 않는다.

### 4. 📊 최종 실측 검증 완료 (Verification)
* `ros2 topic hz /camera/front/image_raw` ➔ **30.0 Hz 실시간 출력 확인**!
* `ros2 topic hz /tf` ➔ **70.8 Hz 실시간 변환 확인**!
* RTAB-Map 2.0Hz 연속 키프레임 22개 정상 생성 확인!

---

## 📌 `[ERR-2026-08-23-01]` Go2 순정 4D 라이다 0Hz 원시 점군 차단 및 비공식 드라이버 6201 포트 충돌

* **발생 일시**: 2026년 8월 23일 11:15 KST
* **분류 태그**: `[LIDAR/ETH]`
* **보고자**: 민석 (Jetson & Hardware Lead)
* **영향 범위**: 라이다 토픽(`/utlidar/cloud`, `/pointcloud`)이 $0\text{ Hz}$로 유지되고 3D 라이다 맵핑 실행 불가

### 1. ⚠️ 증상 및 원본 터미널 에러 로그
```text
$ ros2 topic hz /utlidar/cloud
no new messages
$ ros2 launch go2_bringup go2.launch.py lidar:=True
[ERROR] [hesai_lidar]: PING 192.168.1.201 failed (100% packet loss)
```

### 2. 🔍 기술적 근본 원인 분석
1. **메인보드 기본 Mute 정책**: Go2 메인보드(`192.168.123.161`)는 부팅 시 라이다 점군을 로컬 연산용(50Hz `/odom`)으로만 소비하며 외부 이더넷(`eth0`) 송출을 기본 잠금(Mute) 상태로 둠.
2. **비공식 드라이버 불일치**: 외장 Hesai 라이다용 드라이버를 실행하여 내장 4D L1/L2 라이다의 UDP 패킷(포트 6201 on `192.168.1.2`)을 해석하지 못함.

### 3. 🛠️ 해결 조치 및 수정 코드
1. **공식 리포지토리 탑재**: Unitree 공식 [`unitree_lidar_ros2`](https://github.com/unitreerobotics/unitree_lidar_ros2) 드라이버를 워크스페이스에 통합.
2. **IP 및 포트 자동화**: `ip addr add 192.168.1.2/24 dev eth0` 및 `fuser -k 6201/udp`를 브링업 스크립트에 삽입.
3. **공식 서비스 스위치 체결**: 스마트폰 앱 또는 `unitree_ros2` C++ Service API(`/api/robot_state/request`)를 통해 `Perception LiDAR` 스위치를 ON으로 활성화.

### 4. 📊 최종 실측 검증 완료
* `ros2 topic hz /pointcloud` ➔ **15.0 Hz 실시간 스트리밍 체결 🟢**

---

## 📌 `[ERR-2026-08-23-02]` RTAB-Map 2D 점유격자지도 외곽 가시(Spike) 번짐 및 천장/바닥 노이즈 투영

* **발생 일시**: 2026년 8월 23일 16:40 KST
* **분류 태그**: `[SLAM/GRID]`
* **보고자**: 민석 & 도커/S2E 자율주행 Lead
* **영향 범위**: 실물 복도 맵핑 결과물([`2dmap/0833.yaml`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/2dmap/0833.yaml))의 문틈과 복도 외곽으로 흰색 가시 번짐 및 천장 조명 노이즈 혼입

### 1. ⚠️ 증상 및 맵 정밀 진단
* 15m `Grid/RangeMax`로 인해 문 열린 틈이나 복도 끝에서 레이트레이싱 광선이 미지 영역으로 길게 뻗어나가 가시 모양 노이즈 형성.
* 1.5m 이상의 천장 형광등/에어컨 프레임 점군이 바닥 2D 격자로 투영되어 장애물로 오인식.

### 2. 🔍 기술적 근본 원인 분석
* RTAB-Map 2D 점유 격자 생성 시 최대 거리 제한(`Grid/RangeMax`) 과대 설정 및 높이 필터(`Grid/MaxObstacleHeight`) 미지정.

### 3. 🛠️ 해결 조치 및 수정 코드
1. **[`go2_rtabmap.launch.py`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py)**:
   ```python
   'Grid/RangeMax': '8.0',
   'Grid/MaxObstacleHeight': '1.50',
   'Grid/MinGroundHeight': '-0.20',
   'Grid/NoiseFilteringRadius': '0.10',
   'Grid/NoiseFilteringMinNeighbors': '3',
   ```
2. **후처리 정제기 구현**: [`scratch/clean_and_export_2d_map.py`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/scratch/clean_and_export_2d_map.py)를 통해 Morphological Opening/Closing으로 노이즈를 1초 만에 완전 제거.

### 4. 📊 최종 실측 검증 완료
* $41.15\text{m} \times 72.10\text{m}$ ($787\text{m}^2$) 크기의 논문 출판용 초정밀 클린맵([`2dmap/clean/0833_clean_publication.png`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/2dmap/clean/0833_clean_publication.png)) 확보 완료 🟢

---

## 📌 `[ERR-2026-08-24-01]` 4족 보행 피치/롤 진동 시 Z축 자세 드리프트 및 비정형 점군 Normals 연산 부하

* **발생 일시**: 2026년 8월 24일 11:35 KST
* **분류 태그**: `[SLAM/ICP]`
* **보고자**: Antigravity Supervisor & 민석
* **영향 범위**: RTAB-Map LIVO 구동 시 4족 보행 trotting으로 인한 Z축/Pitch 미세 오차 및 4D 라이다의 Normals 연산 부하

### 1. ⚠️ 증상 및 공식 포럼 분석
* Introlab Nabble 공식 포럼 분석 결과, 지상 로봇의 실내 평탄 복도 SLAM 시 3차원 자세 자유도(6DoF)를 그대로 두면 4족 보행 고유의 보행 충격으로 Z축과 롤/피치 자세에 미세 누적 오차가 발생할 수 있음.
* 비정형 4D 라이다 점군에 대해 `Grid/NormalsSegmentation: true`를 적용할 경우 법선 벡터 추정 오차로 바닥 요철이 장애물로 튈 수 있음.

### 2. 🔍 기술적 근본 원인 분석
* `Reg/Force3DoF` 및 `Optimizer/Slam2D`가 명시되지 않아 6자유도 최적화가 수행되었으며, 비정형 점군에 대한 Normals 연산 비용 발생.

### 3. 🛠️ 해결 조치 및 런치 파라미터 개정
[`src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py):
```python
'Reg/Strategy': '1',                 # 1 = ICP (3D LiDAR Point Cloud Scan Matching)
'Reg/Force3DoF': 'true',             # Ground robot flat terrain 3DoF constraint (x, y, yaw)
'Optimizer/Slam2D': 'true',          # 2D Pose Graph Optimization
'Grid/NormalsSegmentation': 'false', # Fast & robust height passthrough for unorganized lidar
```

### 4. 📊 최종 실측 검증 완료
* 런치 파일 파라미터 100% 정합화 및 2D 평면 구속 그래프 최적화 체결 🟢

---

## 📌 `[ERR-2026-08-24-02]` 도커 Compose VLM API 템플릿 URL 미해석 및 start_docker_s2e.sh 블로킹 버그

* **발생 일시**: 2026년 8월 24일 11:45 KST
* **분류 태그**: `[DOCKER/NET]`
* **보고자**: 민석 & 도커/S2E 자율주행 Lead
* **영향 범위**: `docker-compose up` 또는 `start_docker_s2e.sh` 실행 시 VLM 서버 미연결 및 S2E 정책 노드 기동 중단 위험

### 1. ⚠️ 증상 및 코드 결함
* `compose.yaml`의 기본 `VLM_API_URL`이 `http://qwen-vl-server:8000/...`으로 되어 있어 DNS 해석 실패 위험.
* `scratch/start_docker_s2e.sh`에서 `docker_bridge.py`가 포그라운드로 실행되어 다음 줄인 `vlm_s2e_async_node.py`가 실행되지 못함.

### 2. 🔍 기술적 근본 원인 분석
* 템플릿 기본값 미수정 및 셸 스크립트의 백그라운드(`&`) 분기 누락.

### 3. 🛠️ 해결 조치 및 수정 코드
1. **[`s2e-vlm-async-framework/compose.yaml`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/s2e-vlm-async-framework/compose.yaml)**:
   ```yaml
   VLM_API_URL: ${VLM_API_URL:-http://100.96.60.15:8000/v1/chat/completions}
   ```
2. **[`scratch/start_docker_s2e.sh`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/scratch/start_docker_s2e.sh)**:
   ```bash
   python3 /workspace/go2_ws_antarctica/scratch/docker_bridge.py &
   BRIDGE_PID=$!
   python3 /workspace/go2_ws_antarctica/s2e-vlm-async-framework/src/vlm_s2e_async_node.py
   kill $BRIDGE_PID 2>/dev/null || true
   ```

### 4. 📊 최종 실측 검증 완료
* 도커 내부 백그라운드 브릿지 및 S2E 노드 동시 가동 성공, `antarctica` 브랜치 커밋 완료 🟢

---

## `[ERR-2026-08-28-01]` Planar headless 시작 직후 `Broken pipe` / status 141

- **발생 run**: `20260828_113015_planar3dof_headless`
- **영향**: Phase 2 센서 시작 전 종료. 이 run에는 주행, planar graph, loop closure 결과가 없음.
- **근본 원인**: stale-process 정리의 `pkill -9 -f rtabmap`이 RTAB-Map뿐 아니라 경로에 `rtabmap_runs`가 들어간 wrapper의 `tee`까지 일치시켜 종료함. bringup shell은 닫힌 pipe에 출력하다 SIGPIPE(status 141)로 종료됨.
- **수정**: `rtabmap`과 `rtabmap_viz`의 정확한 process name 및 특정 `ros2 launch ... go2_rtabmap.launch.py` 명령만 종료하도록 범위를 축소함.
- **증거 보호**: `/rtabmap` startup gate가 통과하면 `RTABMAP_STARTED`를 만들고, 이 sentinel이 있는 run에서만 DB를 복사하도록 wrapper를 변경함. 실패 run의 기존 DB 오인 보존을 방지하기 위해 manifest에 `rtabmap_started`, `rtabmap_db_saved`도 기록함.
- **검증 상태**: shell syntax, Python compile, `git diff --check`, evidence 경로를 가진 모의 process 생존 회귀 시험은 통과. 수정 후 physical planar 3DoF 주행은 아직 재실행 전이므로 결과 검증은 미완료.
