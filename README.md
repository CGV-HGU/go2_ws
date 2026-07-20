# 🎮 Unitree Go2 Antarctic Navigation Project (Simulation Branch)

본 브랜치(`antarctica-simul`)는 **Bolei Zhou 교수 연구진의 URBAN-SIM 기반 강화학습(PPO/RAM)** 시뮬레이션 환경에서 사족보행 로봇 Unitree Go2의 주행 강건성을 키우기 위한 **난이도별 커리큘럼 시나리오 환경 개발 및 검증** 전용 워크스페이스 공간임.

---

## 🎯 1. 사용자(이민석)의 핵심 역할 및 미션

사용자님의 본업은 학습 보상 코드를 직접 구현하는 것이 아니라, **"로봇이 안전 주행(Normal) 및 위기 극복(Recovery) 능력을 배울 수 있도록 가상 세계의 물리적 환경(훈련 세트장)을 설계하고 코딩하는 것"**임.

1.  **정상 주행 훈련장**: 보도(Sidewalk)의 곡선과 교차로가 매끄럽게 흐르는 도시 도로 구조 설계.
2.  **회복 주행(Recovery) 훈련장**: 보도 폭의 중앙을 장애물로 차단하여, 로봇개가 어쩔 수 없이 도로(Lane)로 비켜갔다가 다시 보도로 안전하게 복귀하는 행동을 배울 수밖에 없도록 장애물을 배치하는 물리적 계기 제공.
3.  **스테이지별 훈련 패키징**: 난이도에 대응하는 4개의 환경 YAML 설정 파일(`go2_s2e_stage1.yaml` ~ `stage4.yaml`) 작성 및 검증.

---

## 🏃‍♂️ 2. 절차적 생성(PG) 환경 테스트 및 검증법

작성한 시나리오 및 장애물 배치 코드가 Isaac Sim 5.1 상에 에러 없이 렌더링되고 로봇이 정상 스폰되는지 확인하는 명령어.

```bash
# 16개 병렬 환경에 가상 카메라를 켜고 비동기 스태핑 모드로 시나리오 렌더링 확인
cd ~/go2_ws/scratch/s2e-urban-rl
python urbansim/envs/separate_envs/pg_env.py --enable_cameras --num_envs 16 --use_async
```

---

## 🔬 3. 4단계 커리큘럼 시나리오 설정 가이드 (Target Parameters)

`maps/curriculum/pg_env_cfg.py` 파일 내의 `pg_config` 및 난이도별 YAML 파일에 주입해야 할 물리 설정 스펙.

| 훈련 단계 (Stage) | 환경 타입 (`type`) | 보도 레이아웃 (`map`) | 장애물 밀도 (`object_density`) | 보행자 수 (`spawn_human_num`) | 훈련장 구축 의도 (이민석 설계 목적) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1단계: 직선 보행로** | `'clean'` | `'S'` (직선형) | `0.0` (없음) | `0` | 보도 중심을 인지하고 목적지까지 탈선 없이 일직선으로 걷는 훈련. |
| **2단계: 곡선 및 회피** | `'static'` | `'SCS'` (곡선형) | `0.3` (낮음) | `0` | 곡선로를 따라 조향하고, 간헐적 장애물을 부드럽게 피해 걷는 훈련. |
| **3단계: 교차로 및 정체** | `'static'` | `'XSX'` (교차로) | `0.5` (보통) | `0` | 교차로 정션 모퉁이 선회 및 보도를 벤치 등으로 인위 막아 우회-복귀(Recovery) 유도. |
| **4단계: 보행자 정체** | `'dynamic'` | `'XSX'` (복합로) | `0.7` (높음) | `10` (보행 지능) | 좁은 길목에 마주 오는 보행자를 요동 없이 피하는 최종 실전 테스트 환경. |

---

## 📂 4. 코드 관리 및 랩실 공유 방식

*   **독립적 이력 관리**: 우리는 건민 님의 원격 저장소를 오염시키지 않기 위해 우리 브랜치 상에서만 안전하게 개발을 수행함.
*   **시나리오 설정 트래킹**: `scratch/s2e-urban-rl` 내부에서 실시간으로 수정한 튜닝 코드들은 우리 레포 아래인 [maps/curriculum/](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/maps/curriculum/)에 카피해 두어 깃 버전 관리를 유지함.

### 🗂️ 핵심 파일 경로
*   [pg_env_cfg.py (시나리오 설정 베이스라인)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/maps/curriculum/pg_env_cfg.py): 보행로 모양, 장애물 분포 파라미터 조율
*   [check_repo_updates.py (원격 업데이트 확인)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/check_repo_updates.py): 외부 연동 리포지토리의 변동 사항 상시 추적 도구

---

## 📡 5. 연동 저장소 원격 변경 점검
본 워크스페이스와 연동되어 있는 데이터 처리 및 시뮬레이션 저장소의 최신 업데이트 발생 여부를 모니터링할 때 아래 파이썬 스크립트를 기동함.

```bash
python scratch/check_repo_updates.py
```

---

## 🔌 6. 실물 로봇(Go2) 듀얼 네트워크 통신 및 DDS 격리 설계

실하드웨어 배포 시, Go2 로봇개의 Jetson 온보드 컴퓨터와 로봇 제어 보드 간의 DDS 연동이 외부 인터넷 통신과 혼선되는 것을 방지하기 위해 다음과 같은 네트워크 격리 설계를 반영하였음.

### 6.1 물리적 네트워크 이중화 스펙
1. **로봇 내부망 (Static IP)**: 
   * **인터페이스**: 유선 이더넷 (`eth0`)
   * **대역**: `192.168.123.XX` 고정 대역
   * **용도**: 로봇개의 모터 속도 제어(`Sport API`) 및 센서(LiDAR) raw 데이터 송수신. 실시간성 보장을 위해 외부 인터넷 트래픽 유입이 완전히 차단되어야 함.
2. **외부 인터넷망 (DHCP & VPN)**:
   * **인터페이스**: 무선 와이파이 (`wlan0`) 및 Netbird VPN 가상 카드 (`wt0`)
   * **대역**: 연구실 공유기 유동 대역 및 가상 VPN 대역
   * **용도**: 원격지에 있는 대형 비전 언어 모델(VLM Qwen-32B) API 서버(`http://server-02.cgv:8000`)와의 비동기 HTTP 추론 통신 수행.

### 6.2 DDS 바인딩 격리 설정 (`cyclonedds.xml`)
이 두 망의 ROS 2 DDS 패킷이 꼬이는 현상을 방지하기 위해, 로봇 단에서 실행되는 CycloneDDS가 오직 로봇 내부 고정 IP 유선망(`eth0`)으로만 통신하도록 제한함.

* **설정 방식**: 로봇 사이드 Docker 및 호스트 실행 환경에서 `CYCLONEDDS_URI` 환경변수가 루트의 [cyclonedds.xml](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/cyclonedds.xml)을 바라보게 설정.
* **XML 핵심 정의**:
  ```xml
  <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
  ```
* **기대 효과**: 
  1. 외부 와이파이망(DHCP) 공유기에 초고주기 라이다 및 제어 DDS 패킷이 무작위로 방출되어 대역폭이 뻗어버리는 현상 완벽 방지.
  2. 로봇 제어 명령(`cmd_vel`)이 VPN 인터페이스(`wt0`)로 누수되어 로봇이 뇌정지에 빠지는 통신 장애를 원천 차단하고 자율주행의 안정성 확보.

---

## 🚀 7. 실물 로봇(Go2 Jetson Orin) 실하드웨어 배포 및 구동 가이드 (Step-by-Step)

로봇개 본체(Jetson Orin)에 접속하여 본 지능형 자율주행 시스템(S2E + VOCA)을 띄우는 순차적 과정입니다. 모든 노드는 로컬 소스 변경을 즉각 반영하는 핫 마운팅(Hot-Mount) 형태로 배포됩니다.

### 7.1 사전 검증 단계 (Jetson 호스트 터미널)
컨테이너에서 GPU 가속이 가능하도록 NVIDIA 컨테이너 런타임 활성화 상태를 테스트합니다.
```bash
# 런타임 테스트 실행 (성공 시 CUDA 가속 정보가 출력됨)
docker run --rm --runtime nvidia --gpus all xavier-l4t-base:latest nvidia-smi
```

### 7.2 [Step 1] 최신 워크스페이스 동기화
로봇개 Jetson에 SSH로 원격 접속하여, 깃 브랜치의 실하드웨어 배포용 최신 코드를 당겨옵니다.
```bash
# 로봇개 내부 작업 디렉토리로 이동
cd ~/go2_ws

# 배포 브랜치 전환 및 업데이트
git checkout antarctica-simul
git pull origin antarctica-simul
```

### 7.3 [Step 2] Docker Compose 가동 (컨테이너 백그라운드 구동)
컨테이너 가상 네트워크 공간에서 UDP 브릿지, PD 제어기, 데드락 감지기 및 S2E 추론기 4대 핵심 프로세스를 일제히 가동합니다.
```bash
# 컴포즈 데몬(배경) 실행
docker compose up -d

# 실행 상태 확인 (4개 컨테이너가 Up 상태인지 검증)
docker compose ps
```

### 7.4 [Step 3] 호스트 단 UDP 브릿지 실행 (호스트 터미널)
로봇개 메인 시스템(ROS 2 Foxy)에서 나오는 오도메트리를 도커로 밀어 넣어주고, 도커에서 뱉은 제어 명령을 실물 로봇 다리 모터로 이어주는 호스트 브릿지를 기동합니다.
```bash
# 호스트 터미널에서 백그라운드가 아닌 포그라운드로 실행하여 실시간 연결 패킷 확인
python3 ~/go2_ws/scratch/host_bridge.py
```

### 7.5 [Step 4] 통신 및 제어 루프 정상 작동 검증
정상적으로 데이터가 매끄럽게 흐르는지 확인하려면 새 터미널을 열어 아래 토픽들을 에코(echo)해 봅니다.

```bash
# 1. 컨테이너 내부로 호스트의 실시간 위치(Odometry)가 흐르는지 검증 (50Hz)
docker exec -it go2_docker_bridge ros2 topic echo /s2e/odometry/pose

# 2. PD 제어기가 S2E 웨이포인트를 받아 로봇개 속도 지령으로 잘 번역하는지 검증
docker exec -it go2_pd_controller ros2 topic echo /s2e/controller/command

# 3. 데드락 감지 노드가 정상 스캔 중인지 검증 (stuck 시 True 방출)
docker exec -it go2_deadlock_detector ros2 topic echo /robot/status/deadlock
```

### 7.6 시스템 종료 방법
안전하게 모든 자율주행 프로세스를 종료하고 컨테이너를 내리는 방법입니다.
```bash
# 1. 호스트 터미널의 host_bridge.py 프로세스를 Ctrl+C로 종료

# 2. 도커 컨테이너 서비스들을 완전 중지 및 해제
cd ~/go2_ws
docker compose down
```


