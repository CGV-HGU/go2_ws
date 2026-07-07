# ❄️ Unitree Go2 Antarctic Autonomous Navigation Project (Strategy Proposal)

본 저장소는 남극 및 극한 지형 환경에서 자율주행을 성공시키기 위해, ROS 2 개발 환경을 기반으로 수립한 **기술 제안 전략(Technology Strategy) 및 아키텍처 다이어그램**을 수록한 문서입니다. 

---

## 📌 1. 시스템 아키텍처 다이어그램 (Proposed Architecture)

모방 학습(IL) 기반 모델의 경로 추론 결과와 로봇 내부/외부 센서 피드백을 결합하여 속도 명령을 최종 도출하는 시스템 구성 제안안입니다.

```mermaid
graph TD
    %% 1. Perception & Model Inference
    subgraph Model [1. AI Inference Head]
        VLM[VLM / ViNT / NoMAD Model] -->|Inference| TrajRaw[10x2 Normalized Trajectory]
        VLM -->|Inference| VelPred[Predicted Velocity (m/s)]
    end

    %% 2. Trajectory Recovery
    subgraph Recovery [2. Trajectory Recovery Pipeline]
        TrajRaw & VelPred -->|Recovery Formula| TrajPhys[10x2 Recovered Trajectory]
    end

    %% 3. Control & Feedback Loop
    subgraph Control [3. Control & Feedback Loop]
        TrajPhys -->|Goal Input| PD[PD Controller]
        PD -->|cmd_vel| Bridge[go2_driver / cmd_vel Bridge]
    end

    %% 4. Robot & Sensors
    subgraph Hardware [4. Go2 Robot Hardware]
        Bridge -->|Sport API| Go2[Unitree Go2 Robot]
        Go2 -->|L1 LiDAR + IMU| LIO[LIO Strategy: FAST-LIO / LIVO2]
        Go2 -->|Encoders + Foot Force| Leg[Leg Feedback: Slip/Contact]
    end

    %% 5. Feedback Path
    LIO -->|Pose Feedback| PD
    Leg -.->|Auxiliary Slip Check| PD
    Leg -.->|Covariance Scaling| LIO

    %% Style
    style VLM fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    style PD fill:#e8f5e9,stroke:#1b5e20,stroke-width:2px
    style Go2 fill:#ffe0b2,stroke:#e65100,stroke-width:2px
```

---

## 🧭 2. 분야별 기술 제안 전략 (Technology Strategy)

로봇 사양 및 사용 예정인 ROS 2 Humble 환경을 고려해 제 제안하는 3대 분야별 전략 사안입니다.

### 1) Odometry Strategy (위치 추정 제안안)
남극 환경(눈밭에 의한 극심한 빛 반사, 시각 특징점 결여)에서는 카메라 기반의 VIO를 배제하고, 아래의 **LIO 및 Leg 센서 결합 전략**을 제안합니다.
*   **라이다 관성 오도메트리 (LIO)**:
    *   *제안 솔루션*: **FAST-LIO / FAST-LIVO2** 계열
    *   *선정 전략*: 외부 조도 노이즈에 무관한 ToF LiDAR L1 데이터와 IMU 데이터를 밀결합하여, 눈밭 화이트아웃 환경에서도 유실 없는 견고한 3D Pose 추정을 실현하고자 하는 전략적 제안입니다.
*   **다리 관절 및 접지 피드백 (Leg Feedback)**:
    *   *제안 솔루션*: 관절 엔코더 값 기반 Leg Kinematics 및 발끝 3축 힘 센서 피드백
    *   *선정 전략*: 빙판길이나 눈밭 주행 시 필연적으로 발생하는 슬립(Slip)을 감지하고, 단차 임팩트를 흡수할 때 LIO의 오차 가중치를 실시간 조율하는 안전 백업용 연계 제안입니다.

### 2) Controller Strategy (제어 알고리즘 제안안)
*   **ViNT / NoMAD 내장 PD 제어기 (`pd_controller.py`)**:
    *   *전략*: 모델이 쏴주는 relative waypoint를 추종하기 위해 비례-미분 제어를 사용합니다. 로봇이 로컬 좌표계를 기반으로 자율 이동하므로, 글로벌 지도 위치 추정이 미세하게 흔들려도 안정적으로 전방 경로를 유지할 수 있는 제어 전략입니다.
*   **Go2 SDK 내장 `TrajectoryFollow` (비교 제안)**:
    *   *전략*: 외부 ROS 2 제어 없이 SDK 단독으로 단순 절대 경로 추종 테스트를 수행할 때의 비교군으로 활용하는 제안입니다.

### 3) Action Command API (구동 인터페이스 제안안)
*   **Sport API (`SportClient.Move`)**: 
    *   *전략*: 로봇의 보행/자세 균형 제어는 로봇 내장 제어기에 전적으로 위임하고, 상위 제어기는 100Hz 수준의 유속 속도 명령(`vx, vy, yaw`)만 전달하는 결합 방식입니다.
*   **DDS command bridge**: 
    *   *전략*: ROS 2의 `/cmd_vel`을 Sport API로 연결해주는 오픈소스 브릿지 노드를 구축하여 제어 레이어와 통신 레이어를 격리하는 개발 효율화 전략입니다.

---

## 📈 3. Trajectory 정규화 및 복원 공식 검토 (모방 학습 연동)

모델이 일정한 속도 스케일에 종속되지 않고 순수 **기하학적 궤적 형태(Geometric Path)**를 효과적으로 모방 학습(Imitation Learning)할 수 있도록, 속도 성분을 분리한 정규화 방식을 적용합니다.

### 3.1 학습 단계: 정규화 (Normalization)
로봇이 이동한 실제 물리 궤적에서 속도 스케일을 소거합니다. ($\Delta t$는 기록 주기로, 5Hz 데이터셋의 경우 $0.2\text{ s}$ 반영)

1.  **속도 스케일 추출 ($v_{GT}$)**:
    *   로봇이 해당 궤적 구간 동안 이동한 실제 평균 속력($m/s$)을 계산하여 모델 속도 예측 헤드의 정답으로 삼음.
2.  **정규화 공식**:
    $$\text{Normalized Trajectory} = \frac{\text{GT Trajectory}}{\Delta t \times v_{GT}}$$

### 3.2 제어 단계: 물리 궤적 복원 (Recovery)
추론 시 모델이 기하학적 궤적(Normalized Trajectory)과 속도 스케일($v_{pred}$)을 예측하면, 제어기에 인가하기 전 실제 물리적 좌표계로 복원합니다.

$$\text{Recovered Trajectory} = \text{Normalized Trajectory} \times \Delta t \times v_{pred}$$

---

## 📂 4. 조사 목적의 수록 레포지토리 정보
*   `visualnav-transformer/`: ViNT / NoMAD 모델 구현 및 공식 `pd_controller.py`가 포함된 배포 패키지.
