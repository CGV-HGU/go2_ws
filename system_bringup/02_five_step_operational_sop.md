# 📋 [Part 2] 엔드투엔드 운영 파이프라인 (5-Step Operational SOP)

> **문서 버전**: v1.0.0 (ICRA 2026 Production Standard)  
> **대상 플랫폼**: Unitree Go2 EDU (Jetson Orin NX + 4D LiDAR L2)  
> **상위 허브**: [`system_bringup/README.md`](README.md)

---

## 🎯 5단계 운영 개요

```text
[1단계: 복도 매핑] ──▶ [2단계: 2D 맵 추출] ──▶ [3단계: 위치추정 & 골 등록] ──▶ [4단계: 자율주행 실행] ──▶ [5단계: 벤치마크 평가]
  ./run_mapping.sh       extract_final_golden_map.py    ./run_localization.sh         ./run_escape_nav.sh          experiments/
                                                                                     ./run_pixnav.sh
```

---

## 🔹 [1단계] 3D/2D 복도 매핑 (SLAM Mapping)

새로운 환경이나 맵을 재구축할 때 실행합니다:

```bash
cd /home/unitree/go2_ws_antarctica
./run_mapping.sh
```

* **옵션**:
  * `./run_mapping.sh` : 콘솔 텍스트 기반 헤드리스 매핑 (기본)
  * `./run_mapping.sh --view` : 데스크톱 GUI 환경에서 RTAB-Map 3D 점군 뷰어 동시 실행
* **조작 요령**:
  1. Go2 무선 조종기를 잡고, 복도 시작점(Node 1)에서 끝점까지 **일정한 보행 속도(약 0.4 m/s)**로 천천히 주행합니다.
  2. 복도 끝에서 180도 선회하여 다시 시작점으로 복귀하면서 **루프 클로저(Loop Closure)**를 닫아줍니다.
  3. 콘솔에 루프 클로저 성공 메시지가 출력되면 **`Ctrl + C`**를 눌러 매핑을 안전하게 종료합니다.
* **산출물**:
  * `~/.ros/rtabmap.db` (정합된 키프레임 및 링크 그래프 저장)

---

## 🔹 [2단계] 2D 골든 맵 자동 추출

생성된 3D 데이터베이스로부터 내비게이션용 고해상도 2D 평면 격자 지도를 일괄 추출합니다:

```bash
python3 scratch/extract_final_golden_map.py
```

* **추출 산출물**:
  * `2dmap/2d.png` : 4cm 해상도의 클린 2D 평면도
  * `2dmap/2d_metadata.json` : 맵 원점, 해상도, 유효 바운딩 박스 메타데이터
  * `2dmap/0833.pgm` 및 `0833.yaml` : ROS 표준 점유 격자 지도 파일

---

## 🔹 [3단계] 위치추정 가동 및 대화형 골 등록 (Interactive HUD)

로봇을 복도 시작 지점에 두고 위치추정 HUD를 실행합니다:

```bash
./run_localization.sh
```

1. **5초 안정화 웜업**:
   * 터미널 HUD에 지터(Jitter)가 안정되고 `🟢 LOCALIZED`가 출력될 때까지 5초간 대기합니다.
2. **원터치 골 등록 ([ENTER])**:
   * 조종기로 로봇을 이동시켜 원하는 목적지(Waypoint 1)에 세웁니다.
   * 터미널에서 **`[ENTER]`**를 누릅니다:
     * 현재 로봇의 정밀 좌표 $(X, Y, Z, \text{Yaw})$가 `config/navigation_goals.json`에 저장됩니다.
     * 전면 카메라 뷰(`config/goals/goal_01_Waypoint_1.jpg`)가 자동으로 스냅샷 캡처됩니다.
     * 2D 평면도 상에 골 핀이 마킹된 `2dmap/2d_goals_map.png`가 즉시 갱신됩니다.
3. 원하는 수만큼 추가 웨이포인트를 등록한 후 **`q`**를 눌러 종료합니다.

---

## 🔹 [4단계] 자율주행 실행 (ESCAPE-Nav vs PixNav)

### A. 제안 기법: Full ESCAPE-Nav 주행 (`./run_escape_nav.sh`)
* **동작**: 외부 Qwen-VL 비동기 비전 서보잉 + Checkpoint_A 정책 협업 주행
```bash
./run_escape_nav.sh 1       # Goal 1번으로 단독 주행
./run_escape_nav.sh 1,2     # Goal 1 -> Goal 2 순차 패트롤 주행
```

### B. 베이스라인: Direct PixNav 온보드 CUDA 주행 (`./run_pixnav.sh`)
* **동작**: Jetson Orin NX 온보드 PyTorch CUDA 직접 가속 (~54ms 추론)
```bash
./run_pixnav.sh 1           # Goal 1번으로 PixNav 단독 주행
```

* **실시간 터미널 모니터링**:
  * 10Hz 속도로 실시간 로봇 좌표, 목표까지의 유클리드 거리, $v_x, w_z$ 명령, 추론 레이턴시가 스트리밍됩니다.
  * 목표 지점 $0.35\text{m}$ 이내에 도달하면 감속 정지하며 `🎉 [GOAL REACHED] ARRIVED` 메시지와 함께 자동 종료됩니다.

---

## 🔹 [5단계] 실험 결과 및 벤치마크 아티팩트 확인

주행이 종료되면 `experiments/pixnav/` 또는 `experiments/ours/` 아래에 타임스탬프 기반 결과 폴더가 자동 생성됩니다:

```text
experiments/pixnav/20260902_11_goal2_Waypoint_2/
├── trial_summary.md              # 성공 여부, 소요 시간, 이동 거리, 평균 속도, 오차 요약
├── trial_trajectory_on_2d_map.png # 2D 맵 위에 로봇 주행 궤적이 실선으로 오버레이된 이미지
├── trial_benchmark_dashboard.png # 4분할 연구용 출판 품질 그래프 (Trajectory, Velocities, Distance, Actions)
├── trajectory_raw.csv            # 10Hz 원시 시계열 데이터 (x, y, yaw, vx, wz, dist, heading)
├── vlm_decisions.jsonl           # 매 추론 스텝별 정책 결정 및 지연 시간 로그
└── camera_snapshots/             # 주행 중 실시간 카메라 판단 뷰 모음
```
