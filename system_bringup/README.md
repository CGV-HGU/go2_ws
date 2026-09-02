# 🚀 Unitree Go2 ESCAPE-Nav & PixNav 시스템 브링업 마스터 가이드

> **문서 버전**: v1.0.0 (ICRA 2026 Production Standard)  
> **대상 플랫폼**: Unitree Go2 EDU (Jetson Orin NX 16GB + 4D LiDAR L2 + Front RGB Camera)  
> **주요 독자**: 연구팀원, 로봇 운영자, 벤치마크 평가자  
> **기준 브랜치**: `antarctica`

---

## 📌 목차 (Table of Contents)

1. [시스템 개요 및 아키텍처](#1-시스템-개요-및-아키텍처)
2. [하드웨어 & 네트워크 사전 준비 (Pre-Flight)](#2-하드웨어--네트워크-사전-준비-pre-flight)
3. [4대 표준 실행 스크립트 요약](#3-4대-표준-실행-스크립트-요약)
4. [엔드투엔드 운영 파이프라인 (SOP 5단계)](#4-엔드투엔드-운영-파이프라인-sop-5단계)
5. [핵심 연구 문서 안내 (PixNav 실체 & SLAM 치트키 분석)](#5-핵심-연구-문서-안내)
6. [비상 조치 및 E-Stop](#6-비상-조치-및-e-stop)

---

## 1. 🏗️ 시스템 개요 및 아키텍처

본 워크스페이스(`go2_ws_antarctica`)는 사족보행 로봇 **Unitree Go2 EDU** 상에서 **ICRA 2026 자율주행 연구(*ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation*)**의 실물 로봇 온보드 배포 및 베이스라인(**PixNav**) 비교 평가를 수행하기 위한 전용 프로덕션 환경입니다.

```text
                                [Unitree Go2 EDU 실물 로봇]
                                             │
      ┌──────────────────────────────────────┼──────────────────────────────────────┐
      ▼                                      ▼                                      ▼
[4D 라이다 L2 & IMU]                  [전면 광각 RGB 카메라]                 [Jetson Orin NX 16GB]
  • /livo/cloud (10Hz 점군)            • /camera/front/image_raw (30fps)     • RTAB-Map 3DoF SLAM (1Hz)
  • /livo/odom  (50Hz 오도메트리)       • H.264 RTP 멀티캐스트 수신            • Checkpoint_A 온보드 CUDA (54ms)
      │                                      │                                      │
      └──────────────────────────────────────┼──────────────────────────────────────┘
                                             │
                                             ▼
                        [통합 자율주행 제어기 (go2_autonomous_navigator.py)]
                         ├── [Proposed: ESCAPE-Nav] ──▶ 외부 GPU 서버 (Qwen-VL 9B)
                         └── [Baseline: Direct PixNav] ──▶ 온보드 PyTorch CUDA 직접 실행
```

---

## 2. ⚡ 하드웨어 & 네트워크 사전 준비 (Pre-Flight)

### 2.1 하드웨어 부팅 순서
1. **Go2 로봇 본체 전원 켜기**:
   * 배터리 전원 버튼 1회 짧게 + 1회 길게(3초) 눌러 로봇 기동.
   * 로봇이 서서히 일어서며 자세를 유지하는지 확인.
2. **Jetson Orin NX 연결 확인**:
   * 로봇 내부의 젯슨에 모니터/HDMI 또는 SSH로 접속:
     ```bash
     ssh unitree@192.168.123.18  # 유선 이더넷 직결 시
     # 또는 NetBird VPN IP로 접속
     ```

### 2.2 모바일 핫스팟 및 NetBird VPN 원터치 전환
복도 주행 중 무선 통신을 위해 유선 랜선을 뽑기 전, 반드시 핫스팟 연결 스크립트를 실행합니다:

```bash
cd /home/unitree/go2_ws_antarctica
./connect_hotspot.sh
```
* **검증 확인 사항**:
  * `wlan0` 무선랜 연결 성공 여부
  * NetBird VPN 가상 인터페이스 `wt0` 활성화 여부
  * 외부 VLM 워크스테이션 핑 점검:
    ```bash
    ping -c 3 100.96.60.15
    curl -s http://100.96.60.15:8000/v1/models | grep qwen
    ```

---

## 3. 🎯 4대 표준 실행 스크립트 요약

루트 디렉토리에 직관적인 정식 표준 명칭과 하위 호환용 단축 별칭(Symlink)이 완비되어 있습니다:

| 표준 실행 명령어 | 단축 별칭 | 대상 모드 및 세부 역할 |
|---|---|---|
| **`./run_mapping.sh`** | `./run_map.sh` | **3D/2D 복도 매핑 (SLAM)**<br>• L2 라이다 + IMU + 전면 카메라 정합 신규 지도 생성<br>• `--view` 옵션으로 3D GUI 뷰어 확인 가능 |
| **`./run_localization.sh`** | `./run_local.sh` | **위치추정 HUD & 골 매니저**<br>• RTAB-Map 3DoF 전역 위치추정 실시간 모니터링<br>• 엔터([ENTER]) 입력 시 현 위치 + 카메라 사진을 신규 골로 자동 등록 |
| **`./run_escape_nav.sh`** | `./run_our.sh` | **[Proposed] Full ESCAPE-Nav 주행**<br>• Qwen-VL 비동기 시각 서보잉 + Checkpoint_A 정책 협업<br>• 4D 라이다 능동 장애물 회피 및 벤치마크 대시보드 자동 저장 |
| **`./run_pixnav.sh`** | `./run_pix.sh` | **[Baseline] Direct PixNav 주행**<br>• 온보드 CUDA Checkpoint_A 가속 주행 (~54ms 추론)<br>• 라이다 긴급 제동 가드레일 적용 |

---

## 4. 📋 엔드투엔드 운영 파이프라인 (SOP 5단계)

### 🔹 [1단계] 복도 매핑 (SLAM Mapping)
새로운 복도나 환경에서 신규 맵을 생성할 때 실행합니다:

```bash
./run_mapping.sh
```
1. 조종기로 로봇을 복도 시작점(Node 1)에서 끝점까지 천천히 왕복 주행시킵니다.
2. 루프 클로저가 충분히 형성되면 `Ctrl+C`를 눌러 맵을 안전하게 저장합니다 (`~/.ros/rtabmap.db`).

---

### 🔹 [2단계] 2D 골든 맵 자동 추출
저장된 DB로부터 내비게이션용 고해상도 2D 평면도(`2d.png`, `2d_metadata.json`)를 추출합니다:

```bash
python3 scratch/extract_final_golden_map.py
```
* `2dmap/2d.png` 및 `2dmap/2d_metadata.json`이 자동 갱신됩니다.

---

### 🔹 [3단계] 위치추정 가동 및 대화형 골 등록 (Interactive HUD)
로봇을 복도 시작 지점에 두고 위치추정 HUD를 실행합니다:

```bash
./run_localization.sh
```
1. **5초 안정화 모니터**: HUD에서 지터(Jitter)가 안정되고 `LOCALIZED`가 뜨는지 확인합니다.
2. **골 등록**: 조종기로 로봇을 원하는 목적지(Waypoint 1)로 이동시킨 후 **`[ENTER]`**를 누릅니다:
   * 현재 $(X, Y, Z, \text{Yaw})$ 좌표가 `config/navigation_goals.json`에 저장됩니다.
   * 전면 카메라 사진(`config/goals/goal_01_Waypoint_1.jpg`)이 자동 캡처됩니다.
   * 2D 지도에 골 핀이 오버레이된 `2dmap/2d_goals_map.png`가 자동 생성됩니다.
3. 필요한 만큼 웨이포인트를 등록한 후 `q`를 눌러 종료합니다.

---

### 🔹 [4단계] 자율주행 실행 (ESCAPE-Nav vs PixNav)

#### 제안 기법 주행 (ESCAPE-Nav)
```bash
./run_escape_nav.sh 1       # Goal 1번으로 즉시 주행
./run_escape_nav.sh 1,2     # Goal 1 -> Goal 2 순차 패트롤 주행
```

#### 베이스라인 주행 (PixNav)
```bash
./run_pixnav.sh 1           # Goal 1번으로 PixNav 단독 주행
```

* 주행 중 터미널에 실시간 속도($v_x, w_z$), 남은 거리, 추론 레이턴시가 10Hz로 스트리밍됩니다.
* 목표 지점 $0.35\text{m}$ 이내에 도달하면 자동으로 감속 정지하며 미션이 성공(`ARRIVED`) 처리됩니다.

---

### 🔹 [5단계] 실험 결과 및 벤치마크 아티팩트 확인
주행이 완료되면 `experiments/pixnav/` 또는 `experiments/ours/` 아래에 타임스탬프 폴더가 자동 생성됩니다:

* `trial_trajectory_on_2d_map.png`: 2D 평면도 위에 실제 주행 궤적이 오버레이된 이미지.
* `trial_benchmark_dashboard.png`: 4분할 연구용 출판 품질 대시보드 그래프.
* `trajectory_raw.csv`: 10Hz 원시 포즈 및 속도 시계열 데이터.
* `vlm_decisions.jsonl`: 매 추론 스텝별 행동, 확률, 레이턴시 기록.
* `camera_snapshots/`: 실시간 카메라 판단 뷰 저장 디렉토리.

---

## 5. 📚 핵심 연구 문서 안내

본 폴더에는 이번 연구 세션에서 규명된 **PixNav의 내부 실체와 SLAM 치트키 논쟁에 대한 심층 보고서**가 보존되어 있습니다:

* 👉 [`01_pixnav_deep_synthesis_and_slam_cheat_key.md`](01_pixnav_deep_synthesis_and_slam_cheat_key.md)
  * **PixNav 정책 신경망 구조**: 4채널 ResNet-18 + 트랜스포머 디코더 분석
  * **3D ➔ 2D 픽셀 투영 기하학**: 가로 픽셀 $u$ 계산식 및 $v=360$ 중앙 고정 이유
  * **등 뒤(180°)의 딜레마**: 화각 밖 $u=160$ 억지 클램핑과 25° 강제 제자리 회전 가드의 실체
  * **10Hz 상태 추정**: 1Hz SLAM 보정과 50Hz 오도메트리 적분의 결합 원리
  * **주행 성공 판정의 진실**: 비전 개입 0%, 100% 유클리드 거리 판정 메커니즘
  * **"SLAM 치트키" vs ESCAPE-Nav 비교**: 왜 메트릭 흉내내기가 직선 복도에서만 강하고 실세계에서는 자멸하는가?
* 👉 [`02_fail_safes_and_troubleshooting.md`](02_fail_safes_and_troubleshooting.md)
  * 대칭 복도에서의 20m 텔레포트 오인 락온 방지법
  * 토픽 네임스페이스 점검 가이드

---

## 6. 🛑 비상 조치 및 E-Stop

주행 도중 로봇이 이상 거동을 보이거나 비상 정지가 필요한 경우:
1. **키보드 비상 정지**: 터미널에서 **`Ctrl + C`**를 누르면 트랩 핸들러가 발동하여 로봇 속도를 $0.0$으로 즉시 초기화하고 모든 프로세스를 안전하게 종료합니다.
2. **조종기 인터럽트**: Unitree 무선 조종기의 비상 정지 버튼(L2+B 또는 스탠드 모드 전환)을 누르면 모터 레벨에서 하드웨어 락이 걸립니다.
3. **긴급 강제 종료 명령어**:
   ```bash
   pkill -9 -f go2_autonomous_navigator
   pkill -9 -f rtabmap
   ```
