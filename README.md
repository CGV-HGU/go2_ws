# 🎮 Unitree Go2 자율비행/자율주행 남극 프로젝트 (`antarctica-simul` Branch)

본 브랜치는 **VOCA(VLM)와 S2E(End-to-End Control) 모델**의 시뮬레이션 검증, 물리적/화학적 통합 결합, 그리고 실제 사족보행 로봇(Unitree Go2) 배포를 주도하기 위한 이민석 사용자 전용 마스터 워크스페이스 공간임.

---

## 🎯 1. 이민석 마스터 Action Plan (7월 말 ~ 8월 말 전수 정리)

사용자님이 7월 말 실물 로봇 배포 완료부터 시작하여, 8월 말 화학적 결합(Chemical Coupling)의 최종 구현까지 완수해야 할 상세 태스크 마일스톤입니다.

### [Phase 1] 실물 Go2 배포 완료 및 주행 안정화 (7월 말 데드라인)
*   **[ ] 호스트-도커 UDP 브릿지 무결성 검증**
    *   로봇개 온보드 PC에서 `host_bridge.py`를 기동하고, 도커 내에서 `/s2e/odometry/pose`가 누락 없이 50Hz로 수신되는지 확인.
*   **[ ] 최저 기동 속도 데드밴드(Deadband) 필터 튜닝**
    *   [go2_pd_controller_node.py](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/go2_pd_controller_node.py)의 `min_walk_speed` (기본값 `0.05` m/s)와 `waypoint_dt` (`0.4`s)를 실물 로봇 주행 성능을 보며 최적화.
    *   로봇이 좁은 틈새를 만났을 때 굳어버리거나(Freeze) 껑충 뛰는(Jerk) 현상이 없는지 현장 디버깅.
*   **[ ] 물리적 결합 (Physical Coupling) 최종 현장 테스트**
    *   서버 2번의 VLM Async Node에서 나오는 텍스트/웨이포인트 출력을 로봇개의 PD 제어기가 받아 장애물을 성공적으로 우회하고 보도로 복귀하는지 야외 자율주행 실증.

### [Phase 2] 건민 님과의 맵/학습 인수인계 (8월 초)
*   **[ ] Closed-loop 시뮬레이션 평가 코드 인수**
    *   건민 님이 구축한 시뮬레이션 환경 내 자율주행 성능 평가(Navbench-GS 기반) 스크립트 실행법 전수받기.
*   **[ ] S2E 강화학습 모델 사양 분석**
    *   건민 님이 훈련시킨 PPO 정책 모델의 Observation Space(리더 정보, IMU, 상태) 및 Reward Weights 수식 파악 및 문서화.
*   **[ ] 4단계 커리큘럼 튜닝 및 학습 모니터링**
    *   [maps/curriculum/](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/maps/curriculum/) 하위 설정들을 건민 님과 의논하여 수정하며 PPO 모델의 수렴 속도 및 충돌률 관찰.

### [Phase 3] VLM 피처 아카이빙 파이프라인 개발 (8월 중순)
*   **[ ] VLM 임베딩 덤프 코드 작성**
    *   `vlm_node.py`가 Qwen3-VL로 추론을 수행할 때, 최종 출력 텍스트뿐만 아니라 마지막 레이어의 **비주얼 토큰 특징 맵(Visual Latent Embeddings)**을 넘파이(`.npy`) 혹은 데이터베이스에 자동 기록하는 기능 구현.
*   **[ ] 차원 압축 인코더(MLP) 설계**
    *   VLM의 거대한 레이턴트 차원을 S2E가 받아먹기 편하게 64 or 128차원 수준으로 낮춰 주는 소형 차원 축소 네트워크(Feature Compression MLP) 구현.

### [Phase 4] 완전한 화학적 결합 (Chemical Coupling) 구현 및 학습 (8월 말)
*   **[ ] S2E Policy 아키텍처 확장 (Cross-Attention)**
    *   기존 S2E 네트워크 내부에 VLM의 압축 레이턴트 벡터와 상호 연산하는 **Cross-Attention 레이어**를 이식.
    *   이를 통해 로봇 제어망이 단순 (x,y) 좌표 대신 VLM의 시각적 판단 맥락(Reasoning)을 뇌 수준에서 이해하도록 결합.
*   **[ ] 레이턴트 데이터 기반 모방/강화 파인튜닝**
    *   VLM 백본은 凍結(Freeze)하여 GPU 메모리를 아끼고, 우리가 추가한 Cross-Attention 레이어와 S2E 제어 레이어만 LoRA 변형 방식으로 파인튜닝 및 강화학습 수렴 검증.

---

## 🔬 2. 4단계 커리큘럼 시나리오 공동 제안 (S2E V2 가이드라인)

학습 성능을 보며 건민 님과 협의하여 조율할 수 있는 4단계 훈련 코스 템플릿입니다.

*   **장애물 튜닝 매뉴얼**: [URBAN-SIM V2 장애물 세부 튜닝 가이드](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/maps/guides/urbansim_obstacle_tuning_guide.md) 참조.

| 훈련 단계 (Stage) | 보도 형태 (`vary_sidewalk_width`) | 경로 유형 (직선/곡선 확률) | 주력 장애물 시나리오 (`obstacle_scenarios`) | 훈련 구성 및 협의 제안 사항 |
| :--- | :---: | :--- | :--- | :--- |
| **1단계: 직선 보행 기초** | `False` (3.5m 고정) | 직선 100% | `clean` 100% (장애물 없음) | 보도 중심 인지 및 탈선 없는 직선 보행 마스터 목적. |
| **2단계: 곡선 및 기본 회피** | `True` (2.5~4.0m) | 직선 34% / 곡선 66% | `clean` 20% / `sparse` 80% (랜덤 2~5개) | 곡선 조향 및 드문드문 위치한 단일 정적 장애물 우회 훈련. |
| **3단계: 정체로 및 위기 우회** | `True` (2.5~4.0m) | 직선 34% / 곡선 66% | `dense` 40% / `bottleneck` 30% / `slalom` 30% | 장애물 차단 시 **인도 이탈 Done 없이 차도로 우회 복귀(Recovery)**를 유도하는 코스. |
| **4단계: 종합 난관 주행** | `True` (2.5~4.0m) | 직선 34% / 곡선 66% | 전체 시나리오 믹싱 | V2 공식 벤치마크 환경의 스폰 확률을 모사하여 자율주행 성능을 평가하는 최종 테스트베드. |

---

## 🔌 3. 실물 로봇(Go2) 듀얼 네트워크 통신 및 DDS 격리 설계

DDS 연동이 외부 인터넷 통신과 혼선되는 것을 방지하기 위해 다음과 같은 네트워크 격리 설계를 반영하였음.

### 3.1 물리적 네트워크 이중화 스펙
1.  **로봇 내부망 (Static IP)**: 
    *   **인터페이스**: 유선 이더넷 (`eth0`) / **대역**: `192.168.123.XX` 고정 대역
    *   **용도**: 로봇개의 모터 속도 제어(`Sport API`) 및 센서(LiDAR) raw 데이터 송수신.
2.  **외부 인터넷망 (DHCP & VPN)**:
    *   **인터페이스**: 무선 와이파이 (`wlan0`) 및 Netbird VPN 가상 카드 (`wt0`)
    *   **대역**: 공유기 유동 대역 및 가상 VPN 대역
    *   **용도**: 원격지 VLM 서버와의 비동기 HTTP 추론 통신 수행.

### 3.2 DDS 바인딩 격리 설정 (`cyclonedds.xml`)
DDS 패킷이 외부 공유기로 누수되는 현상을 방지하기 위해 CycloneDDS가 오직 유선망(`eth0`)으로만 통신하도록 제한함.
*   **설정 방식**: 로봇 사이드 Docker 및 호스트 환경에서 `CYCLONEDDS_URI` 환경변수가 루트의 [cyclonedds.xml](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/cyclonedds.xml)을 바라보게 설정.
*   **XML 핵심 정의**: `<NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>`

---

## 🚀 4. 실물 로봇(Go2 Jetson Orin) 실하드웨어 배포 및 구동 가이드 (Step-by-Step)

### 4.1 사전 검증 단계 (Jetson 호스트 터미널)
```bash
# GPU 컨테이너 가속 검증
docker run --rm --runtime nvidia --gpus all xavier-l4t-base:latest nvidia-smi
```

### 4.2 [Step 1] 최신 워크스페이스 동기화
```bash
cd ~/go2_ws
git checkout antarctica-simul
git pull origin antarctica-simul
```

### 4.3 [Step 2] Docker Compose 가동 (도커 백그라운드 구동)
```bash
# 컴포즈 데몬 실행
docker compose up -d
# 실행 상태 확인 (4개 컨테이너 검증)
docker compose ps
```

### 4.4 [Step 3] 호스트 단 UDP 브릿지 실행 (호스트 터미널)
```bash
# 호스트 터미널에서 오도메트리/cmd_vel 실시간 송수신 로그 확인
python3 ~/go2_ws/scratch/host_bridge.py
```

### 4.5 [Step 4] 통신 및 제어 루프 정상 작동 검증
```bash
# 1. 도커 내부로 실시간 위치(Odometry)가 전달되는지 에코 검사
docker exec -it go2_docker_bridge ros2 topic echo /s2e/odometry/pose

# 2. PD 제어기가 S2E 웨이포인트를 받아 로봇 속도 지령으로 잘 번역하는지 검증
docker exec -it go2_pd_controller ros2 topic echo /s2e/controller/command
```
