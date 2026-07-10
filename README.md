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

`scratch/sim_curriculum/pg_env_cfg.py` 파일 내의 `pg_config` 및 난이도별 YAML 파일에 주입해야 할 물리 설정 스펙.

| 훈련 단계 (Stage) | 환경 타입 (`type`) | 보도 레이아웃 (`map`) | 장애물 밀도 (`object_density`) | 보행자 수 (`spawn_human_num`) | 훈련장 구축 의도 (이민석 설계 목적) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1단계: 직선 보행로** | `'clean'` | `'S'` (직선형) | `0.0` (없음) | `0` | 보도 중심을 인지하고 목적지까지 탈선 없이 일직선으로 걷는 훈련. |
| **2단계: 곡선 및 회피** | `'static'` | `'SCS'` (곡선형) | `0.3` (낮음) | `0` | 곡선로를 따라 조향하고, 간헐적 장애물을 부드럽게 피해 걷는 훈련. |
| **3단계: 교차로 및 정체** | `'static'` | `'XSX'` (교차로) | `0.5` (보통) | `0` | 교차로 정션 모퉁이 선회 및 보도를 벤치 등으로 인위 막아 우회-복귀(Recovery) 유도. |
| **4단계: 보행자 정체** | `'dynamic'` | `'XSX'` (복합로) | `0.7` (높음) | `10` (보행 지능) | 좁은 길목에 마주 오는 보행자를 요동 없이 피하는 최종 실전 테스트 환경. |

---

## 📂 4. 코드 관리 및 랩실 공유 방식

*   **독립적 이력 관리**: 우리는 건민 님의 원격 저장소를 오염시키지 않기 위해 우리 브랜치 상에서만 안전하게 개발을 수행함.
*   **시나리오 설정 트래킹**: `scratch/s2e-urban-rl` 내부에서 실시간으로 수정한 튜닝 코드들은 우리 레포 아래인 [scratch/sim_curriculum/](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/sim_curriculum/)에 카피해 두어 깃 버전 관리를 유지함.

### 🗂️ 핵심 파일 경로
*   [pg_env_cfg.py (시나리오 설정 베이스라인)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/sim_curriculum/pg_env_cfg.py): 보행로 모양, 장애물 분포 파라미터 조율
*   [check_repo_updates.py (원격 업데이트 확인)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/check_repo_updates.py): 외부 연동 리포지토리의 변동 사항 상시 추적 도구

---

## 📡 5. 연동 저장소 원격 변경 점검
본 워크스페이스와 연동되어 있는 데이터 처리 및 시뮬레이션 저장소의 최신 업데이트 발생 여부를 모니터링할 때 아래 파이썬 스크립트를 기동함.

```bash
python scratch/check_repo_updates.py
```
