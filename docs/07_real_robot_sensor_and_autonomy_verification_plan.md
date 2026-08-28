# Unitree Go2 실센서·RTAB-Map 서브시스템 검증 계획서

> 문서 버전: v2.2 (2026-08-28 KST)
> 대상: Unitree Go2 EDU Plus, 내장 4D LiDAR L2, Jetson Orin NX 16GB
> 호스트: Ubuntu 20.04 / ROS 2 Foxy / CycloneDDS
> 현재 범위: RTAB-Map 골든 맵 자격 검증. recorder, Docker 자율주행, 모터 command bridge는 OFF
> 실험 근거: `20260828_113542_planar3dof_headless`, `20260828_124601_planar3dof_headless`, geometry FAIL run `20260828_141247_planar3dof_headless` 및 link-type ablation

> **범위 안내**: 이 문서는 전체 실로봇 계획 중 센서·mapping·localization Gate의 세부 계획이다. PixelNav/S2E, Jetson–Docker–Server, 안전 제어, pilot와 논문 campaign을 포함한 전체 계획은 [`experiments/00_real_robot_end_to_end_master_test_plan.md`](experiments/00_real_robot_end_to_end_master_test_plan.md)를 따른다.

## 1. 최종 선택: 단층 평면 맵은 3DoF

현재 연구실의 단일 층 복도와 2D/2.5D 내비게이션 지도를 만들 때는 **planar 3DoF를 기본 프로파일로 사용한다.** 4DoF는 경사로, 층간 이동, 실제 고도 변화를 지도 좌표에 보존해야 하는 별도 실험에서만 사용한다.

여기서 3DoF는 RTAB-Map의 포즈 그래프를 `x/y/yaw`로 제한한다는 뜻이다. 입력은 계속 4D L2의 deskewed **3D point cloud**, Unitree LIO odometry, IMU이고, 3D occupancy/cloud 생성도 유지된다. 라이다를 2D 스캔으로 축소하는 구성이 아니다.

| 환경·목표 | 프로파일 | 판정 |
|---|---|---|
| 같은 층의 평평한 복도, PointNav용 2D 맵 | planar 3DoF | **기본값** |
| 사족보행에 의한 작은 상하 진동만 존재 | planar 3DoF | **기본값** |
| 실제 경사로·단차·고도 변화가 평가 대상 | 4DoF | 별도 DB와 별도 run ID로만 시험 |
| 6DoF 지형 SLAM | 현 구성의 목표 아님 | 별도 센서/추정기 검증 필요 |

### 1.1 실측 근거

| 항목 | 4DoF 비교 주행 (2026-08-27) | 3DoF 자격 주행 (2026-08-28) | 해석 |
|---|---:|---:|---|
| raw LIO Z 범위 | 0.0212 m | 0.0335 m | 실제 로봇 높이는 수 cm 범위로 안정 |
| RTAB-Map Z 범위 | **6.452 m** | sampled map-to-base 0.0175 m | 4DoF의 수직 발산이 3DoF에서 제거됨 |
| RTAB-Map 최종 Z | -6.068 m | 약 0.31 m 부근 | 평면 환경에는 3DoF가 물리적으로 타당 |
| local LiDAR proximity closure (Type-2) | 5 | 9 | 기능은 동작하지만 전체 run 왜곡의 주원인이라 canonical OFF |
| global visual closure (Type-1) | 0 | 최신 재시험 2회 | identity-guess + LiDAR ICP 경로 실측 PASS |
| odometry lost / optimizer failure / NaN | 없음 | 없음 | 3DoF 주행 자체는 안정 |

따라서 4DoF를 기본으로 유지할 근거는 없다. 4DoF 결과는 삭제하지 않고 경사 코스용 비교군으로만 보존한다.

## 2. 실제 LIVO 구성

현재 구성은 다음과 같다.

```text
Go2 내장 4D L2 DDS
  ├─ /utlidar/cloud_deskewed ─┐
  ├─ /utlidar/robot_odom ─────┼─ go2_livo_sensor_bridge.py
  └─ /utlidar/imu ────────────┘       │
                                       ├─ /livo/cloud → 3D LiDAR ICP/occupancy
                                       ├─ /livo/odom  → 외부 Unitree LIO odometry
                                       └─ /livo/imu   → gravity constraint

Go2 전면 단안 RGB → /camera/front/image_raw → visual place recognition
```

- 별도 Hesai 드라이버나 외장 L2 Ethernet SDK 프로세스를 실행하는 구성이 아니다.
- RTAB-Map이 visual odometry를 새로 만드는 구성도 아니다. 전면 단안 RGB는 장소 후보를 찾고, 3D LiDAR ICP가 기하 검증을 담당한다.
- recorder, Docker/VLM, `host_bridge.py`, `/cmd_vel` 경로는 맵 자격 시험 동안 실행하지 않는다.

### 2.1 canonical planar 설정

```text
Reg/Strategy=1
Reg/Force3DoF=true
Icp/Force4DoF=false
RGBD/NeighborLinkRefining=false
RGBD/LoopClosureIdentityGuess=true
RGBD/ProximityBySpace=false
```

설치된 RTAB-Map 0.21.1에서는 `Optimizer/Slam2D`를 독립적인 canonical 설정으로 판정하지 않는다. 현재 두 mapping 진입점과 manifest에서는 이 legacy 이름을 제거했으며 실제 planar 기준은 `Reg/Force3DoF=true`, `Icp/Force4DoF=false`이다.

## 3. 현재 주행 판정

짧은 `20260828_124601_planar3dof_headless`는 **planar + global-loop 기능 PASS**로 분류한다.

| 검사 | 결과 | 판정 |
|---|---|---|
| RTAB-Map 시작 및 249 node 기록 | 정상 | PASS |
| DB 무결성 | Python sqlite3 read-only 검사 `ok` | PASS |
| optimized Z span | 0.0235 m, 목표 0.05 m 이내 | PASS |
| optimized pose / path | 164 / 41.04 m | PASS |
| optimized 출발–복귀 오차 | 0.0335 m | PASS |
| Type-2 proximity closure | logger 8회, DB unique 9개 | 기록; canonical에서는 OFF |
| Type-1 global visual closure | 174→61, 211→1 | **PASS** |
| rejected hypothesis | 6개, graph 삽입 전 전부 거부 | PASS |
| odom lost / optimizer failure / NaN | 0 / 0 / 0 | PASS |
| DB 저장 / 최적화 재로딩 | saved=true / 164 poses | PASS |
| wrapper 종료 | status 141, operator `Ctrl+C` logging pipeline 원인 | 코드 수정, 다음 run 재확인 |

`negative hessian index (-1)` 경고 118회는 마지막 pose covariance를 계산하지 못했다는 뜻이다. 이번 DB에서는 optimizer failure, NaN, invalid information matrix 또는 map jump가 동반되지 않았으므로 이 경고만으로 실패 처리하지 않는다. 다만 발생 횟수는 매 run에 기록한다.

DB에는 Type-1 link가 양방향 저장되어 4 row이지만 unique closure는 2개다. 최종 `211→1` visual score는 0.8717, 저장 transform은 0.030 m이고 최적화된 start/end 거리는 0.0335 m였다. `174→61` transform도 0.1885 m로 실제 재방문 궤적과 일치한다.

후속 전체 run `20260828_141247_planar3dof_headless`는 Z와 DB 저장은 정상이지만 실제 두 90도 코너가 접혀 **geometry FAIL**이다. 원본 보존 ablation에서 Type-1을 제거해도 접힘이 남고 Type-2만 제거하면 직교 형상이 복구됐다. 따라서 이 DB는 localization에 쓰지 않으며, 낮은 graph residual이나 많은 loop 수를 물리적 정확도 근거로 사용하지 않는다.

## 4. 수정된 실험 계획표

모든 Gate는 이전 Gate를 통과한 뒤에만 진행한다.

| Gate | 실험 | 실행/조건 | 합격 기준 | 현재 상태 |
|---|---|---|---|---|
| 0 | 종료·DB 저장 검증 | `Ctrl+C` 1회 | status 0, DB integrity OK, optimized pose 저장 | 최신 전체 run에서 PASS |
| 1 | Type-2 OFF 두-코너 loop | `./map_headless.sh`, 1~2분, 실제 두 90도 코너 포함 | Type-2=0, odom lost/NaN=0, Z span ≤0.05 m, 두 코너 보존 | **다음 실행** |
| 2 | global visual loop | 시작/복귀 위치와 camera heading 동일, 각 3~5초 정지 | 올바른 Type-1 ≥1, false closure/큰 correction jump=0 | Gate 1과 동시 확인 |
| 3 | 전체 골든 remap | Gate 1~2 통과 설정으로 전체 경로 1회 | 직선·평행 벽과 두 코너 보존, Type-2=0, DB/PGM/YAML/3D map 정상 | 대기 |
| 4 | localization | Gate 3 DB를 hash 동결 후 cold start | 반복 재시작 성공, false relocalization/map jump=0 | 대기 |
| 5 | 4DoF 선택 실험 | 실제 경사 코스가 연구 범위일 때만 별도 수행 | 기준 고도와 궤적 비교, Z 발산·false closure 없음 | 선택 사항 |
| 6 | 4-Tier 무구동 폐루프 | 골든 맵 동결 후 Jetson→Docker→server→command sink | stale/timeout/malformed 응답이 항상 zero command, 실제 S2E checkpoint 확인 | 대기 |

## 5. 다음 두-코너 자격 실험

내장 RGB에는 metric depth가 없지만 `RGBD/LoopClosureIdentityGuess=true`에서 RGB 장소 후보를 3D LiDAR ICP가 검증하는 경로가 실제로 성공했다. D435i를 추가하지 않고 Type-2만 끈 최소 변경으로 짧은 자격 run을 먼저 수행한다.

1. manifest에서 `RGBD/LoopClosureIdentityGuess=true`, `RGBD/ProximityBySpace=false`를 확인한다.
2. 시작 위치에서 특징적인 장면을 같은 heading으로 3~5초 관측한다.
3. 실제 두 90도 코너가 포함되도록 1~2분 짧은 loop를 0.2~0.3 m/s로 주행한다.
4. 시작 pose와 같은 위치·heading으로 복귀해 3~5초 정지한다.
5. Type-1 후보가 identity guess에서 3D LiDAR ICP로 올바르게 검증되는지 확인한다.
6. Type-2 accepted/link가 0인지, 폐쇄 전후 endpoint gap, 코너, 이중벽, graph error, Z span을 비교한다.
7. Type-1이 올바르고 두 코너가 보존되면 전체 remap으로 바로 이동한다.

현재 baseline에는 D435i를 추가하지 않는다. Type-2 OFF에서도 Type-1이 반복 실패하거나 큰 heading 차이 재방문이 실험 요구사항이 될 때만 별도 RGB-D branch를 검토한다.

## 6. 계측 항목과 합격 규칙

각 run에서 다음을 같은 표에 남긴다.

| 분류 | 필수 기록 |
|---|---|
| 재현성 | run ID, git HEAD, 설정 snapshot, 시작/종료 시각, 주행 경로 |
| 센서 | cloud/image/odom/IMU rate, timestamp 역행, IMU interpolation warning |
| 궤적 | node 수, raw/optimized endpoint gap, x/y/z range |
| 루프 | Type-1 승인 수, Type-2=0 확인, rejected 후보 수와 이유, 최고 hypothesis score |
| 그래프 | max linear/angular graph error, optimizer failure, NaN, map jump |
| 종료 | wrapper status, DB integrity, optimized pose 수, hash, 잔여 프로세스 |

실패 규칙은 다음과 같다.

- odometry lost, optimizer failure, NaN, false global closure, 갑작스러운 지도 접힘 중 하나라도 발생하면 FAIL.
- 평면 코스에서 raw 또는 optimized Z span은 0.05 m 이하면 PASS, 0.05~0.10 m는 WARN 후 재시험, 0.10 m 초과는 FAIL.
- `negative hessian index`만 있고 지도/최적화 실패가 없으면 WARN으로 기록하고 계속 판정한다.
- canonical Type-2 OFF run에서 Type-2 event/link가 하나라도 생기면 설정 적용 실패다.
- 정상 종료와 최종 optimized graph 저장이 확인되지 않은 DB는 최종 지도나 localization DB로 동결하지 않는다.

## 7. 현장 실행 순서

```bash
cd /home/unitree/go2_ws_antarctica
./map_headless.sh
```

시작 배너에서 다음을 확인한다.

```text
profile=planar3dof
Reg/Force3DoF=true
Icp/Force4DoF=false
Recorder=false
Docker/motor=false
```

주행 후 출발 pose에서 정지한 상태로 `Ctrl+C`를 한 번만 누른다. 증거 디렉터리는 다음 위치다.

```text
/home/unitree/.ros/rtabmap_runs/<timestamp>_planar3dof_headless/
```

이 문서의 Gate 0~3을 통과하기 전에는 180m 전체 맵을 최종 자산으로 촬영하거나 localization용 DB로 승격하지 않는다. 통과 후에도 곧바로 자율주행하지 않고 전체 E2E 계획의 PixelNav/S2E와 4-Tier command-sink Gate로 이동한다.
