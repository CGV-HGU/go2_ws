# 2026-09-13 Unitree Go2 실제 로봇 네비게이션 실험 결과 보고서 (SR & SPL 분석)

> 다음 기록용 실험: [새 매핑·목표 5개·에피소드 기록 준비](five_goal_recording_prep/README.md). 새 지도와 실제 목표 캡처를 기다리는 상태이며 준비 과정에서 로봇을 움직이지 않았습니다.

> 후속 검토: [Jetson 로그 분석·수정·ESCAPE/Direct 비교 준비](jetson_review/README.md). 기존 수치의 측정 기준과 제한은 해당 문서에 구분했습니다. 다음 단계는 에피소드 주행이며, 추가 지표 검토는 주행 후 진행합니다.

본 보고서는 2026년 9월 13일 Unitree Go2 실제 로봇 플랫폼에서 수행된 **Full 비동기 S2E-VLM 네비게이션 6회 주행 평가**의 공식 학술 벤치마크 지표(**SR: Success Rate, SPL: Success weighted by Path Length**) 및 세부 주행 궤적 데이터를 정리한 결과입니다.

---

## 🏆 1. 핵심 학술 벤치마크 지표 종합 요약 (SR & SPL)

> **SPL 계산 정의**: $SPL = \frac{1}{N} \sum_{i=1}^N S_i \frac{l_i}{\max(p_i, l_i)}$  
> ($S_i \in \{0, 1\}$: 1.0m 목표 반경 도착 여부, $l_i$: 시작-목표 최단 직선거리, $p_i$: 실제 누적 주행거리)

| 목표 구분 | 시도 횟수(N) | 성공 횟수 | **SR (성공률)** | **SPL (경로 가중 성공률)** | 평균 소요시간 | 평균 주행거리 | 최단 직선거리($l_i$) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Goal 1** | 3 | 2 | **66.7%** | **50.3%** | 170.6s | 4.49m | 4.841m |
| **Goal 2** | 3 | 3 | **100.0%** | **100.0%** | 62.2s | 2.75m | 3.115m |
| **전체 (Total)** | 6 | 5 | **83.3%** | **75.1%** | 116.4s | 3.62m | - |

---

## 2. 개별 회차별 세부 주행 지표 (Trial-by-Trial Breakdown)

| 목표 | 회차 | 성공($S_i$) | **SPL** | 최종 결과 상태 | 소요시간(s) | 누적 주행($p_i$) | 최단거리($l_i$) | 잔여거리(m) | 최종 방향 오차 | 현장 관찰 보고 |
|:---:|:---:|:---:|:---:|:---|---:|---:|---:|---:|---:|:---|
| **Goal 1** | Trial 01 | ❌ 0 | **0.0%** | `CONTROLLER_INHIBITED:MOTION_TIMEOUT` | 56.7s | 0.56m | 4.841m | 4.777m | -48.3° | 조작은 없었고, 몸전체가 로봇기준으로 오른쪽 방향으로 약간 회전하고 멈췄어 |
| **Goal 1** | Trial 02 | ✅ 1 | **69.9%** | `GOAL_DISTANCE_REACHED` | 259.9s | 6.92m | 4.841m | 0.999m | -172.1° | 약 1m부근에 멈추긴했는데 골포즈를 찍을때 바라보던 방향과 반대 방향처럼 보고 멈췄어. 개입접촉 진로방해는 없었어 |
| **Goal 1** | Trial 03 | ✅ 1 | **81.0%** | `GOAL_DISTANCE_REACHED` | 195.3s | 5.98m | 4.841m | 0.979m | -40.4° | 개입접촉없었고 1m부근에 있어 |
| **Goal 2** | Trial 01 | ✅ 1 | **100.0%** | `GOAL_DISTANCE_REACHED` | 88.9s | 3.10m | 3.115m | 0.989m | +44.8° | 직접개입 접촉 진로 방해는 없었는데 약 1m안에 들어가자마자 멈춘것 같긴한데 정확히 얼마나 남았는지는 모르겠어. 그래도 육안으로는 약 1m같긴해 |
| **Goal 2** | Trial 02 | ✅ 1 | **100.0%** | `GOAL_DISTANCE_REACHED` | 47.5s | 2.55m | 3.115m | 0.956m | +40.9° | 아까 보다 좀 오차가 있어보이는데? 좀 1m보다는 살짝 먼감도 있는데 대충은 맞는것 같아. 개입접촉 진로방해는 없었어. |
| **Goal 2** | Trial 03 | ✅ 1 | **100.0%** | `GOAL_DISTANCE_REACHED` | 50.3s | 2.61m | 3.115m | 0.939m | +42.1° | 1m 부근에서 멈췄어. 진로 방해는 없었어. |

---

## 3. 실험 환경 및 제어 파라미터

- **프레임워크**: S2E-VLM Full 비동기 정책 (`async_true`)
- **로봇 플랫폼**: Unitree Go2 EDU (Lidar + Front Camera + IMU + Odom)
- **좌표계 방식**: `fixed_start_odometry` (고정 시작점 원점 기준 상대 좌표)
- **도착 판정 반경**: `1.0 m` (자동 거리 도달 시 정지)
- **최종 방향 제어**: 미적용 (PointGoal 도착 거리 기준 정지)
- **Look 동작 정책**: `forward_0p1` (카메라 look down/up 대신 0.1m 미세 전진 대체)
- **최대 주행 허용 시간**: `360 s`

---

## 4. 세부 분석 및 고찰

### 1) Goal 2 분석: SR 100.0%, SPL 100.0%
- Goal 2(직선거리 3.115m) 주행 3회는 **모두 100% 성공**하였으며, 이동 경로가 최단 직선거리와 거의 일치(평균 2.75m 주행 후 1m 반경 진입 정지)하여 **SPL 역시 만점인 100.0%**를 기록했습니다.
- 평균 도달 시간은 **62.2초**로 매우 빠르고 안정적인 주행을 보였습니다.

### 2) Goal 1 분석: SR 66.7%, SPL 50.3%
- Goal 1(직선거리 4.841m) 주행은 3회 중 2회 성공(**SR 66.7%**)했습니다.
- 첫 번째 회차(Trial 1)는 초기 관측 회전 도중 타임아웃(`MOTION_TIMEOUT`)이 발생하여 조기 종료(SPL 0%)되었습니다.
- 이후 2회차(Trial 2, SPL 69.9%) 및 3회차(Trial 3, SPL 81.0%)는 모두 정상적으로 1.0m 목표 반경에 도달하였으며, 성공 회차 평균 SPL은 **75.5%**를 기록했습니다.

### 3) 무간섭 안전성 (Zero Intervention)
- 6회 주행 전 구간에서 작업자의 수동 개입(Direct Intervention), 장애물 충돌, 전도 등 비정상 상황이 **단 1건도 발생하지 않았습니다** (Intervention/Run = 0.00).

---

## 5. 시각화 결과

### 1) 전체 6회 주행 궤적 통합 오버레이
![All Trajectories Overlay](visualizations/all_trajectories_overlay.png)

### 2) 2D Grid 맵 상의 실제 로봇 궤적 투영
![Trajectories on 2D Map](visualizations/trajectories_on_2d_map.png)

### 3) Goal 1 vs Goal 2 개별 비교
| Goal 1 (3회 반복) | Goal 2 (3회 반복) |
|:---:|:---:|
| ![Goal 1](visualizations/goal_1_trajectories.png) | ![Goal 2](visualizations/goal_2_trajectories.png) |

---

## 6. 디렉토리 구조 및 데이터 링크
```
experiments/0913/
├── README.md                     # 본 종합 보고서 (SR & SPL 포함)
├── benchmark_metrics.json        # [신규] 공식 학술 SR & SPL JSON 데이터
├── episodes_summary.csv          # [업데이트] SR, SPL 컬럼 추가 지표 테이블
├── episode_index.json            # JSON 포맷 전체 메타데이터
├── goal_plan.json                # 골 좌표 및 시작 원점 정의
├── origin.json                   # 원점 오도메트리 스냅샷
├── session_log.md                # 현장 주행 세션 원문 기록
├── visualizations/               # 통합 시각화 플롯
│   ├── all_trajectories_overlay.png
│   ├── goal_1_trajectories.png
│   ├── goal_2_trajectories.png
│   └── trajectories_on_2d_map.png
├── goal_1/
│   ├── trial_01/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)
│   ├── trial_02/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)
│   └── trial_03/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)
└── goal_2/
    ├── trial_01/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)
    ├── trial_02/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)
    └── trial_03/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)
```

- [종합 벤치마크 지표 JSON](file:///home/unitree/go2_ws_antarctica/experiments/0913/benchmark_metrics.json)
- [종합 주행 지표 CSV (SR & SPL 포함)](file:///home/unitree/go2_ws_antarctica/experiments/0913/episodes_summary.csv)
- [Goal 1 Trial 1 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_01/trajectory.csv)
- [Goal 1 Trial 2 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_02/trajectory.csv)
- [Goal 1 Trial 3 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_03/trajectory.csv)
- [Goal 2 Trial 1 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_01/trajectory.csv)
- [Goal 2 Trial 2 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_02/trajectory.csv)
- [Goal 2 Trial 3 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_03/trajectory.csv)