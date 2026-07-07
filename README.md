# ❄️ Unitree Go2 Antarctic LIO Navigation Workspace

본 저장소는 남극 및 실외 야외 지형에서 **LIO(라이다-관성 오도메트리)**를 주축으로 하여 사족보행 로봇 **Unitree Go2**의 자율주행을 제어하기 위한 ROS 2 Humble/Foxy 기반의 워크스페이스(`go2_ws`)입니다.

모방 학습(IL) 모델의 **Trajectory 정규화 및 물리 복원 공식**과 실제 하드웨어 구동을 위한 3대 핵심 모듈 연동 정보가 담겨 있습니다.

---

## 📌 1. 시스템 제어 및 데이터 흐름

```
[ 10x2 Normalized Trajectory ] (모델 예측)  ──┐
                                              ├──> [ 물리 궤적 복원 (Trajectory Recovery) ]
[ Velocity (m/s) ] (모델 예측)                 ──┘           │
                                                            ▼ (10x2 Recovered Trajectory)
[ LIO Pose (FAST-LIO2 /odom) ] ───────────────────────────> [ 2) PD Controller ]
                                                            │
                                                            ▼ cmd_vel (vx, vy, yaw)
                                                     [ 3) go2_driver (DDS Bridge) ]
                                                            │
                                                            ▼ Sport API
                                                     [ Unitree Go2 Robot ]
```

---

## ⚙️ 2. 3대 핵심 모듈 정의 (LIO 중심)

### 1) Odometry Module (위치 추정)
실외/남극의 열악한 외부 조도 및 보행 진동을 극복하기 위해 **LIO(LiDAR-Inertial Odometry)**를 오도메트리의 메인 소스로 채택합니다.
*   **LIO (주축)**: 로봇 내장 4D LiDAR L1 + 내부 IMU 데이터를 통한 **FAST-LIO2** 오도메트리 연산 (50~100Hz 고주파수 Pose 추정).
*   **VIO (보조)**: 추가 부착된 RealSense D435i 카메라 특징점 + 내장 IMU 데이터 기반 위치 추정.
*   **Leg Odometry**: `lf/sportmodestate` (다리 관절 엔코더 + 바디 IMU) 값을 `go2_odom_bridge`를 통해 표준 ROS 2 `/go2_odom` 토픽으로 변환 및 발행.

### 2) PID Controller (경로 추종기)
*   **ViNT / NoMAD 기반 PD 제어기 (`pd_controller.py`)**:
    *   공식 레포지토리(`visualnav-transformer/deployment/src/pd_controller.py`)의 비례-미분 제어 코드를 뼈대로 활용합니다.
    *   **입력**: 복원된 10x2 물리 궤적(Waypoints) 및 LIO 현재 Pose
    *   **제어 메커니즘**:
        *   현재 위치와 목표 Waypoint 사이의 거리 오차 ➔ 선속도($v_x, v_y$) 제어
        *   로봇 헤딩 방향과 목표 Waypoint 방향 사이의 각도 오차 ➔ 각속도($\omega_z$, yaw rate) 제어
*   **Go2 내장 기능 (비교군)**: Unitree SDK2 Sport API의 `TrajectoryFollow` 함수 (로컬 데드레커닝 기반 경로 추종).

### 3) Unitree Go2 Action Command API
*   **실제 구동 API**: Unitree SDK2 고수준 제어 API인 **`SportClient.Move(vx, vy, vyaw)`**
*   **연동 오픈소스 (`src/go2_robot/go2_driver`)**:
    *   상위 단(Nav2 또는 PD 제어기)에서 연산된 ROS 2 `/cmd_vel` 속도 지령을 구독하여, Go2 내부 모션 제어기가 읽을 수 있는 DDS Request (`api/sport/request`, JSON 포맷) 패킷으로 변환하여 실물 로봇을 보행시킵니다.

---

## 📈 3. Trajectory 정규화 및 복원 파이프라인 (IL)

모델이 일정한 속도 스케일에 종속되지 않고 순수 **기하학적 궤적 형태(Geometric Path)**를 효과적으로 모방 학습(Imitation Learning)할 수 있도록, 속도 성분을 분리한 정규화 방식을 적용합니다.

### 3.1 학습 단계: 정규화 (Normalization)
로봇이 이동한 실제 물리 궤적 $P = \{(x_1, y_1), ..., (x_{10}, y_{10})\}$에서 속도 스케일을 소거합니다.

1.  **속도 스케일 추출 ($v_{GT}$)**:
    *   로봇이 궤적 구간 동안 이동한 실제 평균 속력($m/s$)을 계산하여 모델의 Scalar Speed 예측 헤드의 정답(Ground Truth)으로 삼습니다.
2.  **시간 스텝 정규화 ($\Delta t$)**:
    *   기록 주기(예: 5Hz 데이터셋의 경우 $\Delta t = 0.2$초)를 시간 간격으로 반영합니다.
3.  **정규화 공식**:
    $$\text{Normalized Trajectory} = \frac{\text{GT Trajectory}}{\Delta t \times v_{GT}}$$

### 3.2 제어 단계: 물리 궤적 복원 (Recovery)
추론 시 모델이 기하학적 궤적(Normalized Trajectory)과 속도 스케일($v_{pred}$)을 예측하면, 이를 제어기에 인가하기 전 물리적 크기를 가진 좌표계로 복원합니다.

$$\text{Recovered Trajectory} = \text{Normalized Trajectory} \times \Delta t \times v_{pred}$$

---

## 🏃 4. 실행 및 배포 스크립트 (Workspace Root)
*   **`run_localization_outdoor.sh`**: 실외 및 광역 환경에서 가장 안정적인 L1 라이다 주도의 자율주행 및 위치 추정(Nav2 연동)용 쉘 스크립트.
*   **`run_map_outdoor.sh`**: D435i 카메라 대역폭 유실을 회피하고 로봇 내장 L1 LiDAR SLAM 오도메트리(`/odom`)를 매핑의 뼈대로 사용하여 안정적으로 3D 환경 맵을 그릴 때 사용하는 스크립트.

---

## 🔬 5. 학술적 차별성 및 오픈소스 레포지토리 (References)

### 5.1 학술적 차별성 (Novelty)
*   **기존 ViNT/NoMAD의 한계**: 원본 연구들은 평탄한 실내외 바닥에서 주로 바퀴형 로봇과 단순 휠 오도메트리를 기반으로 경로를 추적했습니다. 이 방식은 미끄러짐이 심한 실외 흙길이나 남극의 눈밭 환경에서 오도메트리가 급격히 발산하는 치명적인 한계가 있습니다.
*   **본 시스템의 차별성 (LIO 결합)**: 미끄러짐이 심한 극한 지형에서 경로 추종 성능을 보장하기 위해, 기존의 단순 다리/휠 오도메트리를 배제하고 **FAST-LIO2(라이다-관성 오도메트리)** 기반의 초고정밀 3D Pose 피드백을 PD 제어 루프에 직접 결합합니다. 이를 통해 지면 슬립과 흔들림이 심한 사족보행 로봇에서도 오차 없는 강인한 궤적 추종 주행을 실현합니다.

### 5.2 관련 주요 오픈소스 레포지토리
*   [visualnav-transformer (ViNT/NoMAD/GNM)](https://github.com/robodhruv/visualnav-transformer): 이미지 기반 시각 네비게이션 모델 및 PD 제어기 공식 저장소
*   [FAST_LIO (FAST-LIO2)](https://github.com/hku-mars/FAST_LIO): 초고속 고정밀 라이다-관성 오도메트리(LIO) 솔루션
*   [go2_robot (IntelligentRoboticsLabs)](https://github.com/IntelligentRoboticsLabs/go2_robot): ROS 2 Humble 기반 Unitree Go2 제어 및 통신 드라이버
*   [rtabmap_ros](https://github.com/introlab/rtabmap_ros): 3D 맵핑 및 비주얼/라이다 루프 클로저 결합 패키지

