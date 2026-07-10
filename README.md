# 🎮 Unitree Go2 Antarctic Navigation Project (Simulation Branch)

본 브랜치(`antarctica-simul`)는 **Bolei Zhou 교수 연구진의 URBAN-SIM 기반 강화학습(PPO/RAM)** 시뮬레이션 환경에서 사족보행 로봇 Unitree Go2의 주행 강건성을 키우기 위한 **난이도별 커리큘럼 시나리오 개발 및 검증** 전용 워크스페이스 공간임.

---

## 🛠️ 1. 시뮬레이션 환경 준비 (Isaac Sim 5.1 & Conda Setup)

서버 또는 고성능 개발 PC 환경에서 Isaac Sim 5.1 및 패키지들을 빌드하기 위해 복사-붙여넣기하여 실행할 명령어 세트.

### 1단계: 콘다 가상환경 구성
```bash
# 콘다 가상환경 생성 및 활성화
conda create -n env_s2e_rl python=3.11 -y
conda activate env_s2e_rl
python -m pip install --upgrade pip

# 필수 컴파일 도구 설치 (Ubuntu 기준)
sudo apt-get update && sudo apt-get install -y build-essential cmake make gcc g++
```

### 2단계: pip Isaac Sim 5.1 설치 및 검증
```bash
# NVIDIA PyPi 레포지토리를 통한 설치
python -m pip install "isaacsim[all,extscache]==5.1.0.0" --extra-index-url https://pypi.nvidia.com

# 설치 상태 정상 인식 여부 검증
python -c "import isaacsim; print(isaacsim.__file__)"
```

### 3단계: URBAN-SIM 의존성 패키지 빌드
```bash
# s2e-urban-rl 폴더 진입 후 셋업 스크립트 실행
cd ~/go2_ws/scratch/s2e-urban-rl
bash urbansim.sh -i
bash urbansim.sh -a

# 런타임 주요 라이브러리 버전 고정
python -m pip install numpy==1.26.0 gymnasium==1.2.1 click==8.1.7 psutil==5.9.8 Pillow==11.3.0 packaging==23.0

# 3D 에셋 데이터 다운로드
python scripts/tools/collectors/collect_asset.py
python scripts/tools/converters/convert_asset.py
```

---

## 🏃‍♂️ 2. 절차적 생성(PG) 시나리오 검증 워크플로우

개발 중인 절차적 생성(Procedural Generation) 환경이 에러 없이 렌더링되고 로봇이 정상 스폰하는지 확인하기 위한 검증 스크립트 실행법.

```bash
# 가상 카메라 뷰를 켜고 16개 환경을 비동기 스태핑 모드로 모의 실행
cd ~/go2_ws/scratch/s2e-urban-rl
python urbansim/envs/separate_envs/pg_env.py --enable_cameras --num_envs 16 --use_async
```

*   `--enable_cameras`: 시각 기반 관측(Observation) 공간을 로드합니다.
*   `--use_async`: 각 병렬 환경이 독립적인 물리 주기로 동기화 없이 진행되게 끔 설정합니다.

---

## 📂 3. 우리 브랜치의 역할 및 코드 관리 방식

*   **독립적 이력 관리**: 우리는 건민 님의 원격 저장소를 오염시키지 않기 위해 우리 브랜치 상에서만 안전하게 개발을 수행함.
*   **시나리오 설정 트래킹**: `scratch/s2e-urban-rl` 내부에서 실시간으로 수정한 튜닝 코드들은 우리 레포 아래인 [scratch/sim_curriculum/](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/sim_curriculum/)에 카피해 두어 깃 버전 관리를 유지함.

### 🗂️ 핵심 파일 경로
*   [pg_env_cfg.py (시나리오 설정 베이스라인)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/sim_curriculum/pg_env_cfg.py): 직선/교차로 구조, 장애물 밀도, 보행자 조건 조율
*   [check_repo_updates.py (원격 업데이트 확인)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/check_repo_updates.py): 외부 연동 리포지토리의 변동 사항 상시 추적 도구

---

## 🔬 4. 시나리오 커리큘럼 설계 방향 (Draft)

강건한 주행 신경망 학습을 위해 우리가 단계별로 튜닝해나갈 4단계 시나리오 설정값 명세서.

| 커리큘럼 단계 | 환경 타입 (`type`) | 보행로 형태 (`map`) | 장애물 밀도 (`object_density`) | 보행자 수 (`spawn_human_num`) |
| :--- | :--- | :--- | :--- | :--- |
| **1단계 (입문 직선)** | `'clean'` | `'S'` (직선) | `0.0` (없음) | `0` |
| **2단계 (기초 회피)** | `'static'` | `'SCS'` (곡선 정션) | `0.3` (낮음) | `0` |
| **3단계 (심화 정션)** | `'static'` | `'XSX'` (교차로) | `0.5` (보통) | `0` |
| **4단계 (동적 보행)** | `'dynamic'` | `'XSX'` (복합 보행로) | `0.7` (높음) | `10` (보행 지능 활성화) |

---

## 📡 5. 연동 저장소 원격 변경 점검
본 워크스페이스와 연동되어 있는 데이터 처리 및 시뮬레이션 저장소의 최신 업데이트 발생 여부를 모니터링할 때 아래 파이썬 스크립트를 기동함.

```bash
python scratch/check_repo_updates.py
```
