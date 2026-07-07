# ❄️ Unitree Go2 Antarctic LIO Navigation Project (Investigation Phase)

본 저장소는 남극 및 극한 지형 환경에서 **LIO(라이다-관성 오도메트리)를 단독 주축으로 활용**하고, 험지 극복을 위해 **Leg(다리 상태 및 발끝 접지력) 피드백을 보조 연동**하여 자율주행을 구현하기 위한 기술 검토 및 아키텍처 조사 보고서입니다.

---

## 📌 1. LIO 및 Leg 기반의 오도메트리 아키텍처

남극 환경의 특성(눈밭 반사광, 무특징 지형, 빙판 슬립)을 극복하기 위해 VIO(카메라 기반)를 아예 배제하고 **LIO + Leg 정보 결합**에 집중합니다.

```
                  [ Unitree Go2 Sensors ]
                             │
            ┌────────────────┴────────────────┐
            ▼ (LiDAR L1 + IMU)                ▼ (Encoder + Foot Force)
    [ 1) LIO Module (주축) ]           [ 2) Leg Feedback (보조) ]
  (3D Geometry / Pose 추정)           (지면 접지력 및 슬립 판단)
            │                                 │
            └────────────────┬────────────────┘
                             ▼ (융합)
                   [ 현재 Pose 피드백 ]
                             │
                             ▼ (오차 보정)
                    [ PD Controller ]
```

### 1) LIO (LiDAR-Inertial Odometry) ➔ **오도메트리 주축**
*   **센서 구성**: 내장 4D LiDAR L1 + 내부 고주파 IMU
*   **기술 검토**:
    *   ToF(비행시간 측정) 레이저를 사용하므로 남극의 화이트아웃(햇빛 반사 노이즈)이나 가시거리 제약에 영향을 받지 않고 절대적인 3D 공간 기하 구조를 정밀하게 측정합니다.
    *   FAST-LIO2 등 tightly-coupled 필터링 기반의 오도메트리 스택을 탑재하여 보행 진동을 IMU 연산으로 상쇄하며 주행 위치(Pose)를 추정합니다.

### 2) Leg Feedback (다리 관절 및 접지 상태) ➔ **험지/빙판 백업 및 보조**
*   **센서 구성**: 12개 조인트 모터 엔코더 + 발끝 3축 힘 센서 (Foot-end Force Sensor)
*   **기술 검토**:
    *   **접지력 피드백**: 남극 빙판이나 경사지에서 로봇이 미끄러질 때(Slip), 발끝 힘 센서의 접선력/법선력 비율($F_{tangent} / F_z$)을 감지하여 실시간으로 슬립 여부를 판별합니다.
    *   **LIO 신뢰성 보완**: 슬립이 감지되거나 불규칙한 단차 충격이 올 때, LIO 필터 내의 상태 추정 공분산 가중치를 동적으로 스케일링하여 상태 적분이 붕괴하는 것을 방지합니다.

---

## ⚙️ 2. 제어기 및 구동 API

### 1) PD Controller (경로 추종 제어기)
*   **ViNT / NoMAD 내장 PD 제어기 (`pd_controller.py`)**:
    *   모델이 출력한 로컬 궤적(Waypoint)과 1)에서 LIO로 계산된 현재 Pose 간의 오차를 계산하여 `cmd_vel` 속도 명령을 생성합니다.
    *   미끄러짐이 심한 남극 지면(Leg 피드백으로 감지) 진입 시 제어 게인(P, D)을 동적으로 인하하여 모터 출력을 부드럽게 제어하는 연계 전략을 검토합니다.

### 2) Unitree Go2 Action Command API
*   **Sport API (`SportClient.Move`)**: `vx, vy, yaw` 속도를 입력받아 로봇을 움직이는 고유 API.
*   **ROS 2 Velocity Bridge (`go2_driver`)**: ROS 2 `/cmd_vel` 토픽을 위 Sport API로 변환하여 실물 로봇을 구동합니다.

---

## 📈 3. Trajectory 정규화 및 복원 공식 검토 (모방 학습 연동)

모방 학습(IL) 모델이 로봇의 절대 속도 스케일에 영향을 받지 않고 순수 **기하학적 경로(Normalized Trajectory)**를 학습할 수 있도록, 속도($v$) 성분을 소거 및 복원하는 전처리 파이프라인을 검토하고 있습니다 (수식 정합성 유건민 연구원 확인 완료).

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
