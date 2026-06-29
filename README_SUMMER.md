# 🚀 Unitree Go2 SLAM & Navigation Execution Guide (Branch: `summer`)

본 가이드는 `summer` 브랜치에서 업데이트된 **비주얼-관성 융합 SLAM 및 Nav2 자율주행**의 구동 절차를 설명합니다. 원본 코드는 `master` 브랜치에 그대로 보존되어 있으며, 모든 개선 및 디버깅 사항은 `summer` 브랜치에만 반영되었습니다.

---

## 1. 주요 변경 및 개선 사항 (Summary)
* **오도메트리 치명적 버그 수정**: `go2_driver`에서 최초 1회만 `/odom` 토픽을 보낸 후 침묵하는 버그를 해결하여 고주파로 오도메트리 토픽이 발행되도록 수정했습니다.
* **매핑 모드 듀얼 설정 제공 (라이다-IMU 오도메트리 지원)**: `run_map.sh`에서 리얼센서 카메라의 Visual Odometry 뿐만 아니라, Go2 본체의 **4D LiDAR L1 + IMU 결합 오도메트리**(`/odom`)를 기반으로 매핑할 수 있도록 선택식 스위치(`USE_LIDAR_ODOM`)를 적용했습니다.
* **토픽 정렬**: 카메라 깊이 맵이 깨지거나 어긋나는 문제를 방지하기 위해 정렬된 깊이 이미지 토픽(`/camera/aligned_depth_to_color/image_raw`)으로 일치화했습니다.
* **TF 트리 구조 안정화**: 매핑과 주행 모드 간의 좌표계 기준점을 `base_link`로 통일하고 다중 부모 오류를 예방했습니다.

---

## 2. 매핑 모드 실행 (Mapping / SLAM)

실외 또는 실내 3D 지도를 새롭게 빌드할 때 실행합니다.

### 📌 오도메트리 모드 스위칭 설정 (매우 중요)
[run_map.sh](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/run_map.sh) 파일 상단에서 매핑 시 오도메트리 소스를 선택할 수 있습니다.
```bash
# 기본값은 true (리얼센서 대역폭이 USB 2.0으로 낮을 때 오도메트리 유실을 완전히 극복함)
USE_LIDAR_ODOM=true
```
* **`USE_LIDAR_ODOM=true` (권장)**: Go2의 온보드 **LiDAR + IMU 결합 오도메트리**를 SLAM의 기준축으로 사용하여 D435i 카메라의 대역폭 저하/프레임 드랍 환경에서도 안정적으로 맵을 작성합니다.
* **`USE_LIDAR_ODOM=false`**: 카메라의 순수 Visual Odometry(VIO)만을 사용하여 매핑합니다. (로봇 통신 없이 오프라인 구동 시 유용)

### 실행 순서

1. 터미널을 열고 `go2_ws` 워크스페이스로 이동하여 `summer` 브랜치 상태인지 확인합니다.
   ```bash
   cd ~/go2_ws
   git checkout summer
   ```
2. 매핑 스크립트를 구동합니다.
   ```bash
   ./run_map.sh
   ```

---

## 3. 로컬라이제이션 & 자율주행 모드 실행 (Localization & Nav2)

저장된 지도를 바탕으로 현재 위치를 추정하고 Nav2를 통해 자율주행을 수행합니다.

### 실행 전 체크리스트
* 생성된 rtabmap DB 파일(`rtabmap.db`)과 지도 메타데이터가 `~/.ros/` 경로에 존재하는지 확인합니다.
  - DB 경로: `/home/unitree/.ros/rtabmap.db`
  - 지도 경로: `/home/unitree/.ros/rtabmap.yaml` 및 `rtabmap.pgm`
* 만약 다른 경로에 생성했다면, `run_localization.sh` 내부의 `RTABMAP_DB_PATH` 환경 변수 값을 해당 경로로 수정해야 합니다.

### 실행 순서

1. 로아스 수입 Go2의 모터 제어 및 하드웨어 통신 노드를 구동합니다. (반드시 최초 1회 실행)
   ```bash
   ros2 launch go2_bringup go2.launch.py
   ```
2. 새 터미널에서 로컬라이제이션 및 내비게이션 스크립트를 실행합니다.
   ```bash
   cd ~/go2_ws
   ./run_localization.sh
   ```

### 구동 노드 구성
* **[1] Go2 Bringup**: 로봇 드라이버 및 로봇 상태 퍼블리셔 기동
* **[2] RealSense & LaserScan**: 카메라 드라이버 기동 및 깊이 이미지를 2D 레이저 스캔(`/scan`)으로 변환 기동
* **[3] IMU Filter**: 카메라 관성 필터 기동
* **[4] RTAB-Map Localization**: 위치 추정 모드로 기동 (Incremental Memory Off)
* **[5] Map Server**: 2D 지도 서버 기동 및 수명주기(Lifecycle) 활성화
* **[6] Navigation2**: Nav2 내비게이션 스택 기동 (MPPI/DWB 제어 알고리즘 활성화)
* **[7] RViz**: Nav2 관제 화면 기동 (RViz 상단의 `2D Nav Goal`로 목적지 지정 가능)

---

## 4. 트러블슈팅 가이드 (Troubleshooting)

### ⚠️ CycloneDDS 통신 불통 문제
* **증상**: 다른 PC나 노트북에서 로봇의 ROS 토픽이 조회되지 않거나 스크립트 실행 시 노드 검색이 안 되는 경우.
* **원인**: `cyclonedds.xml` 파일 내 네트워크 카드 이름이 `eth0`로 고정되어 있기 때문입니다.
* **해결**: 본인의 실제 네트워크 디바이스명(예: `wlan0`, `enp3s0`)에 맞춰 수정한 뒤 실행하거나, 환경변수 설정을 해제하십시오.
  ```bash
  # cyclonedds.xml 설정 일시 해제
  unset CYCLONEDDS_URI
  ```
