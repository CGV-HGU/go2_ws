# ❄️ Unitree Go2 Antarctic Autonomous Navigation Project (Investigation Phase)

본 문서는 남극 과제 연동 및 통합을 위해 상준님과의 카카오톡 논의 사항을 토대로 **Unitree Go2의 3대 핵심 모듈(Odometry, Controller, Action API)의 후보군을 조사하고, 모방 학습(IL)용 Trajectory 정규화 방식을 기술 검토**하기 위한 조사 보고서 문서입니다.

---

## 📌 1. 검토 중인 3대 핵심 모듈 후보군

남극과 같은 극한 환경에서 자율주행을 구현하기 위해 아래의 기술 후보군들을 매핑하고 비교 검토하고 있습니다.

### 1) Odometry Module (위치 추정 후보군)
남극의 적설, 빙판, 강한 반사광(화이트아웃) 하에서 안정적으로 Pose를 추정하기 위한 후보들입니다.
*   **VIO (Visual-Inertial Odometry)**:
    *   *센서*: 추가 장착된 RealSense D435i 카메라 + 내장 IMU.
    *   *검토*: 눈밭의 시각적 특징점 부족(Low-texture) 및 강한 반사광에 의한 워시아웃 노이즈 취약성 대비 필요.
*   **LIO (LiDAR-Inertial Odometry)**:
    *   *센서*: 로봇 내장 4D LiDAR L1 + 내부 IMU.
    *   *검토*: 빛의 영향을 받지 않는 ToF 레이저 기반 기하학적 매핑(예: FAST-LIO2 등)으로, 반사광이 심한 실외/남극 지형에서 안정적인 대안으로 유력 검토 중.
*   **Leg Odometry**:
    *   *센서*: 로봇 다리 관절 엔코더 + IMU.
    *   *검토*: 지면 미끄러짐(Slip)이 매우 심한 남극 빙판/눈길 특성상 누적 오차가 커질 수 있어 VIO/LIO의 보조 보정 수단으로만 검토.

### 2) PID Controller (경로 추종 제어기 후보군)
모델이 출력한 Trajectory(경로)를 로봇이 실제로 따라가게 만들기 위한 제어기 후보군입니다.
*   **ViNT / NoMAD 내장 PD 제어기 (`pd_controller.py`)**:
    *   *출처*: `visualnav-transformer` 공식 레포지토리의 배포 패키지.
    *   *특징*: 목표 궤적(Waypoint)과 현재 Pose 간의 거리/각도 방향 오차를 계산하여 로봇의 선속도와 각속도를 출력하는 간단한 비례-미분(PD) 제어 모델.
*   **Go2 SDK 내장 `TrajectoryFollow`**:
    *   *출처*: Unitree SDK2 Sport API 제공 함수.
    *   *특징*: 좌표점 배열(`PathPoint[]`)을 로봇 내부 모션 보드에 직접 주입하여 자체 제어 루프로 추종을 제어하는 방식.

### 3) Unitree Go2 Action Command API (구동 인터페이스)
속도 지령을 받아 실물 로봇을 구동하기 위한 연동 API 방식입니다.
*   **Sport API (`SportClient.Move(vx, vy, vyaw)`)**:
    *   `vx, vy, yaw` 속도를 입력받아 로봇 본체를 움직이는 고유 구동 API.
*   **ROS 2 Velocity Bridge**:
    *   ROS 2의 표준 `/cmd_vel` (Twist) 속도 명령을 구독하여 위 Sport API로 매핑해 주는 오픈소스 브릿지 노드(`go2_robot` 등) 검토.

---

## 📈 2. Trajectory 정규화 및 복원 공식 검토 (모방 학습 연동)

모방 학습(IL) 모델이 로봇의 절대 속도 스케일에 영향을 받지 않고 순수 **기하학적 경로(Normalized Trajectory)**를 학습할 수 있도록, 속도($v$) 성분을 소거 및 복원하는 전처리 파이프라인을 검토하고 있습니다 (수식 정합성 유건민 연구원 확인 완료).

### 2.1 학습 단계: 정규화 (Normalization)
로봇이 이동한 실제 물리 궤적에서 속도 스케일을 소거합니다. ($\Delta t$는 기록 주기로, 5Hz 데이터셋의 경우 $0.2\text{ s}$ 반영)

1.  **속도 스케일 추출 ($v_{GT}$)**:
    *   로봇이 해당 궤적 구간 동안 이동한 실제 평균 속력($m/s$)을 계산하여 모델 속도 예측 헤드의 정답으로 삼음.
2.  **정규화 공식**:
    $$\text{Normalized Trajectory} = \frac{\text{GT Trajectory}}{\Delta t \times v_{GT}}$$

### 2.2 제어 단계: 물리 궤적 복원 (Recovery)
추론 시 모델이 기하학적 궤적(Normalized Trajectory)과 속도 스케일($v_{pred}$)을 예측하면, 제어기에 인가하기 전 실제 물리적 좌표계로 복원합니다.

$$\text{Recovered Trajectory} = \text{Normalized Trajectory} \times \Delta t \times v_{pred}$$

---

## 📂 3. 조사 목적의 수록 레포지토리 정보
본 브랜치 하위에는 3대 모듈 및 제어 성능 검토를 분석하기 위해 아래 레포지토리가 클론되어 조사를 진행하고 있습니다.
*   `visualnav-transformer/`: ViNT / NoMAD 모델 구현 및 공식 `pd_controller.py`가 포함된 배포 패키지.
