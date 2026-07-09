# ❄️ Unitree Go2 Antarctic Navigation Project

본 저장소는 남극 및 극한 지형 환경에서 사족보행 로봇 **Unitree Go2**의 자율주행을 제어하기 위한 ROS 2 Humble/Foxy 기반의 전용 워크스페이스(`go2_ws`)임.

---

## 🚀 1. 현재 개발 및 연동 진행상황 (Current Progress)

화요일(7월 14일) 실물 로봇 연동 테스트를 앞두고 완료된 개발 현황 및 필드 검증 예정 태스크 요약.

| 개발 모듈 / 태스크 | 상태 | 상세 내용 |
| :--- | :--- | :--- |
| **실하드웨어 LIO 오도메트리 파싱** | 🟢 **완료 (Completed)** | `/utlidar/robot_pose` 수신 및 `/s2e/odometry/pose` 변환 퍼블리시 구현 완료 |
| **실하드웨어 PD/PID 주행 제어 루프** | 🟢 **완료 (Completed)** | AI 궤적 추종을 위한 $K_{p}$, $K_{d}$ 에러 댐핑 제어 알고리즘 구현 완료 |
| **폐루프 제자리 회전 제어 (Action)** | 🟢 **완료 (Completed)** | 오도메트리 피드백 기반 P-Control 실물 회전 액션 서버 구현 완료 |
| **Foxy-Jazzy 통신 바이패스 브릿지** | 🟢 **완료 (Completed)** | UDP 바이패스 바이너리 직렬화 송수신 스크립트 작성 완료 (`scratch/`) |
| **파이썬 Direct 구동 드라이버** | 🟢 **완료 (Completed)** | C++ 빌드 실패에 대비한 파이썬 기반 `SportClient` 긴급 구동 노드 완료 |
| **화요일 DDS / 소켓 연동 테스트** | 🟡 **대기 (Ready)** | Foxy(호스트)-Jazzy(도커) 간 실물 CycloneDDS 통신 상태 검증 예정 |
| **센서 드라이버 토픽 리매핑** | 🟡 **대기 (Ready)** | D435i 카메라 RGB 이미지 및 LiDAR 점군 토픽 실하드웨어 매핑 예정 |

---

## 📋 2. 하이브리드 연동 및 배포 워크플로우 (Quick Run Guide)

화요일 실하드웨어 젯슨 보드에서 각 단계별로 복사-붙여넣기하여 즉시 실행할 명령어 세트.

### 1단계: 호스트 OS 준비 (Foxy 네이티브)
1. **CUDA 11.4 / TensorRT 8.5.2 사양 검증**:
   ```bash
   cat /etc/nv_tegra_release && nvcc --version && dpkg -l | grep nvinfer
   ```
2. **GPU 가속 PyTorch/ONNX Runtime 수동 설치**:
   ```bash
   # PyTorch ARM64 휠 설치
   wget https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
   pip3 install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
   
   # ONNX Runtime GPU 버전 설치
   pip3 install onnxruntime-gpu --index-url https://aiinfra.pkgs.visualstudio.com/PublicPackages/_packaging/onnxruntime-cuda-11/pypi/simple/
   ```
3. **로봇 구동 Foxy 드라이버 실행**:
   ```bash
   cd ~/go2_ws && colcon build --packages-select go2_robot go2_driver && source install/setup.bash
   ros2 launch go2_bringup go2.launch.py
   ```

### 2단계: 도커 컨테이너 준비 (Jazzy CPU 전용)
1. **Jazzy 공식 이미지 Pull 및 기동 (CPU 모드 / 루프백 공유)**:
   ```bash
   docker pull arm64v8/ros:jazzy-ros-base
   docker run -it --name go2_jazzy_cpu --net=host -v ~/go2_ws:/workspace/go2_ws arm64v8/ros:jazzy-ros-base bash
   ```
2. **도커 컨테이너 빌드 및 유닛 테스트**:
   ```bash
   # 도커 진입 후 실행
   apt-get update && apt-get install -y python3-colcon-common-extensions python3-pip
   cd /workspace/go2_ws/s2e-vlm-async-framework && colcon build && source install/setup.bash
   python3 -m unittest discover -s src/s2e_vlm_nodes/test -p "test_*.py" -v
   ```

### 3단계: 통신 브릿지 가동 (Bridge Test)
*   **옵션 A (Zenoh Bridge)**:
    ```bash
    # 호스트 및 컨테이너 각각 실행
    ./zenoh-bridge-dds -d 0
    ```
*   **옵션 B (파이썬 소켓 브릿지 - C++ 빌드 우회)**:
    *   호스트 터미널: `python3 ~/go2_ws/scratch/host_bridge.py`
    *   컨테이너 터미널: `python3 /workspace/go2_ws/scratch/docker_bridge.py`

### 4단계: 실물 자율주행 제어 검증 (Air & Ground Run)
```bash
# 도커 컨테이너 내부에서 실제 제어 파라미터를 인가해 구동 (거치대 테스트 선행)
ros2 launch s2e_vlm_bringup robot_side.launch.py use_mock_hardware:=false
```

---

## 🔍 3. 세부 기술 전략 및 매핑 명세 (Details)

각 항목을 클릭하면 리드미 페이지 이동 없이 세부 상세 내용이 아코디언 형태로 펼쳐짐.

<details>
<summary><b>📖 3.1 시스템 아키텍처 및 코어 파이프라인</b></summary>

### 3.1.1 3대 핵심 모듈 정의
*   **Odometry**: LiDAR L2 + IMU ➔ 3D Pose (`/s2e/odometry/pose`)
*   **Controller**: 궤적 & Pose ➔ `cmd_vel`
*   **Action API**: `cmd_vel` ➔ Sport API 구동 (`go2_driver`)

### 3.1.2 Trajectory 정규화 및 물리 복원 공식
모델이 일정한 속도 스케일에 종속되지 않고 순수 기하학적 궤적 형태를 학습할 수 있도록 학습 단계에서 정규화하고 제어 단에서 속도 스케일을 복원함.

*   **학습 단계 (정규화)**:
    $$\text{Normalized Trajectory} = \frac{\text{GT Trajectory}}{\Delta t \times v_{GT}}$$
*   **제어 단계 (물리 복원)**:
    *   **E2E 모델 노드(외장 GPU/컨테이너 단)**에서 추론 시 기하학적 궤적과 속도 스케일($v_{pred}$)을 곱해 최종 물리 단위 궤적으로 복원하여 토픽을 발행함.
    $$\text{Recovered Trajectory} = \text{Normalized Trajectory} \times \Delta t \times v_{pred}$$
    *   실물 로봇 측 제어기는 복원이 끝난 미터 단위의 실제 궤적 좌표(`Trajectory2D`)를 수신하므로 추가 복원 연산 없이 즉시 PD 주행 추종을 수행함.
</details>

<details>
<summary><b>📖 3.2 네트워크 & DDS 환경 트러블슈팅</b></summary>

*   **CycloneDDS 인터페이스 바인딩 오류**: VPN망이 켜질 때 내부 이더넷 로컬 DDS 통신이 두절되는 현상을 막기 위해 `cyclonedds.xml` 내 `<NetworkInterfaceAddress>` 태그에 로봇 내부 포트인 `eth0`를 하드코딩 바인딩함.
*   **멀티캐스트 차단 (원격 노드 검색 실패)**: 학교 무선망이나 VPN망은 멀티캐스트를 차단하므로, `cyclonedds.xml`의 유니캐스트 피어 주소 리스트(`<Peers>`)로 양단 PC 고정 IP를 수동 매핑하여 탐색 문제를 해결함.
*   **CPU 과부하 차단**: 원격 VPN 통로로는 제어 정보(10x2 Trajectory 및 Odom)만 송수신하고 대용량 센서 데이터는 호스트 내부에서 격리하여 CPU 부하를 방지함.
</details>

<details>
<summary><b>📖 3.3 비동기 프레임워크 설계 분석 보고서</b></summary>

*   **DDS 역직렬화 오류 입증**: 서로 다른 ROS 2 배포판 간 DDS 통신 시 역직렬화 오류 발생 가능성이 크므로 Zenoh / Socket Bridge 활용을 기본으로 전제함.
*   **전역 지도 의존성 배제**: 본 주행계는 글로벌 SLAM 맵 없이 로컬 상대 좌표계 `base_link` 상에서 완전히 독립적인 클로즈드 루프 방식으로 가동함.
*   **Rotate-to-Front 정책**: VLM이 비정면 회전각을 지시하면 즉시 제자리 회전 액션 서버(`/s2e/controller/rotate`)를 쏘아 정면으로 몸체를 돌려놓은 뒤 주행을 속행함.
*   **오도메트리 child_frame_id**: LIO/VIO 최종 결과물의 포즈는 센서 축이 아닌 로봇 몸체 중심인 `base_link` 기준 좌표로 변환되어 발행됨.
</details>

<details>
<summary><b>📖 3.4 탑재 센서 하드웨어 세부 제원표</b></summary>

*   **Unitree Go2 전면 내장 RGB 카메라**: 해상도 1280 × 720, 화각 120° (초광각)
*   **Unitree Go2 내장 4D LiDAR L2**: 수평 360° × 수직 96° (수직 광화각을 통해 발끝 주변 사각지대 해소), 유효 주파수 64,000 pts/s, 정밀 오차 4.5mm 이내, 감지 범위 최대 30m / 최소 5cm.
*   **Intel RealSense D435i 깊이 카메라**: Active IR Stereo Depth 방식, RGB 1280 × 720 @ 90fps (화각 69° × 42°), 스캔 범위 0.11m ~ 10m+, 내장 IMU Bosch BMI055 탑재.
</details>

<details>
<summary><b>📖 3.5 젯슨 온보드 하이브리드 분리 아키텍처 전략</b></summary>

*   **구조 설계**: 무거운 AI 모델 추론 루프는 호스트 OS(CUDA 11.4 네이티브)에서 연산하고, 비동기 프레임워크 제어 노드(Jazzy)는 도커 내부에서 CPU-only 모드로 기동함.
*   **DDS 격리 우회**: 두 환경 간의 제어 및 상태 좌표 정보는 로컬 루프백(`127.0.0.1`) 상에서 Python Socket Bridge 또는 Zenoh Bridge로 실시간 송수신하여 젯슨 내부에서 자율주행 루프를 완전히 종결함.
</details>

<details>
<summary><b>📖 3.6 ViNT 궤적 제어 매핑 및 튜닝 전략</b></summary>

*   **제어 알고리즘 개량**: 변위를 무조건 추론 주기로 나눠 속도가 튀는 ViNT 순정 방식 대신, 비례-미분 각속도 제어($w = K_{p\_ang} \times \text{atan2}(dy, dx) + K_{d\_ang} \times d\_heading$)를 탑재하여 로봇 엉덩이가 흔들리는 피쉬테일링 현상을 댐핑 억제함.
*   **보행 안정화 ($v_y = 0.0$)**: Go2는 측면 게걸음 보행 시 지상고 마진 부족으로 야외 험지에서 전도 위험이 매우 큼. 따라서 횡속도($v_y$)는 강제 차단하고 오직 전진($v_x$)과 회전($v_{yaw}$)만으로 선회 비행 구동하도록 매핑함.
*   **필드 튜닝 가이드**:
    *   *보행 중 오실레이션(Fishtailing) 발생 시*: `kp_angular` 감소 및 `kd_angular` 증가.
    *   *회전 지연으로 박치기(Understeering) 발생 시*: `lookahead_index`를 3에서 2 또는 1로 감소시켜 빠른 반경 선회 유도.
</details>

<details>
<summary><b>📖 3.7 유니트리 공식 SDK 레퍼런스 코드 및 족보</b></summary>

*   **Python SDK High-Level 모션 제어**: `ChannelFactoryInitialize` 채널 초기화 및 `SportClient` 모션 API(`StandUp`, `Move`, `Damp`) 사용 가이드.
*   **Python SDK 센서 상태 수신 (`SportStateClient`)**: IMU 자세 쿼터니언, 로컬 추정 속도, 포즈 위치 데이터 실시간 콜백 수신 코드.
*   **C++ cmd_vel ROS 2 핸들러 구조**: cmd_vel 수신 시 횡속도 격리 및 JSON 직렬화 후 Request 패킷 전송 구조 아카이빙.
</details>

---

## 📂 4. antarctica 브랜치 디렉토리 구조 (Clean Workspace)

```text
go2_ws/
├── README.md                  <- [최신화] 본 가이드 포털 문서 (진행 상황 최상단 배치)
├── cyclonedds.xml             <- CycloneDDS 네트워크 바인딩 설정 파일
├── docs/                      <- 백업 상세 세부 기술 문서 폴더 (참조용)
├── scratch/                   <- 실물 연동 검증용 UDP 소켓/파이썬 백업 드라이버 스크립트 폴더
│   ├── host_bridge.py         <- 호스트 단(Foxy) UDP 통신 송수신 브릿지
│   ├── docker_bridge.py       <- 도커 내부(Jazzy) UDP 통신 송수신 브릿지
│   └── python_direct_driver.py<- ROS 2 C++ 빌드 에러 시 대체 구동 가능한 파이썬 Direct 드라이버
├── visualnav-transformer/     <- ViNT / NoMAD 모델 구현 및 pd_controller.py 코드
├── qwen_nav_memory_framework_v3/ <- 상위 VLM 기반 에피소딕 메모리 프레임워크 패키지
├── s2e-vlm-async-framework/   <- ROS 2 비동기 통합 프레임워크 패키지
└── src/
    ├── HesaiLidar_ROS_2.0/    <- Hesai 라이다 연동 ROS 2 드라이버
    ├── rtabmap_ros/           <- rtabmap SLAM 패키지 (이민석 개인 VIO 연구용)
    └── go2_robot/             <- Unitree Go2 ROS 2 통신 패키지
```
