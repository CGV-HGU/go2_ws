# Unitree Go2 실센서·RTAB-Map 서브시스템 검증 계획서

> 문서 버전: v2.0 (2026-08-28 KST)
> 대상: Unitree Go2 EDU Plus, 내장 4D LiDAR L2, Jetson Orin NX 16GB
> 호스트: Ubuntu 20.04 / ROS 2 Foxy / CycloneDDS
> 현재 범위: RTAB-Map 골든 맵 자격 검증. recorder, Docker 자율주행, 모터 command bridge는 OFF
> 실험 근거: `20260828_113542_planar3dof_headless` 및 2026-08-27 4DoF 비교 주행

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
| local LiDAR proximity closure (Type-2) | 5 | 9 | 3D LiDAR 기하 폐쇄가 실제로 동작 |
| global visual closure (Type-1) | 0 | 0 | 다음 Gate에서 별도 해결 필요 |
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
```

설치된 RTAB-Map 0.21.1에서는 `Optimizer/Slam2D`가 제거된 legacy 이름이며 독립적인 유효 설정으로 판정하지 않는다. 실행 배너와 manifest에 남아 있더라도 실제 planar 동작의 canonical 기준은 `Reg/Force3DoF=true`, `Icp/Force4DoF=false`이다.

## 3. 현재 주행 판정

`20260828_113542_planar3dof_headless`는 **planar 동작 자격 PASS, 최종 골든 DB FAIL**로 분류한다.

| 검사 | 결과 | 판정 |
|---|---|---|
| RTAB-Map 시작 및 352 node 기록 | 정상 | PASS |
| DB 무결성 및 SHA-256 | 정상 | PASS |
| raw Z span 0.0335 m | 목표 0.05 m 이내 | PASS |
| sampled map Z span 0.0175 m | 목표 0.05 m 이내 | PASS |
| Type-2 proximity closure | 9개 | PASS |
| Type-1 global visual closure | 0개 | **미통과** |
| 정상 종료 | wrapper status 141 | **미통과** |
| 최종 optimized pose 저장 | 0 pose | **미통과** |

`negative hessian index (-1)` 경고 179회는 마지막 pose covariance를 계산하지 못했다는 뜻이다. 이번 DB에서는 optimizer failure, NaN, invalid information matrix 또는 map jump가 동반되지 않았으므로 이 경고만으로 실패 처리하지 않는다. 다만 발생 횟수는 매 run에 기록한다.

## 4. 수정된 실험 계획표

모든 Gate는 이전 Gate를 통과한 뒤에만 진행한다.

| Gate | 실험 | 실행/조건 | 합격 기준 | 현재 상태 |
|---|---|---|---|---|
| 0 | 종료·DB 저장 검증 | 30~60초 정지/짧은 이동 후 `Ctrl+C` 1회 | status 0 또는 의도된 130, DB integrity OK, optimized pose 저장, 잔여 RTAB-Map 프로세스 없음 | **재시험 필요** |
| 1 | planar 3DoF 짧은 loop | `./mapping_planar_headless.sh`, 0.2~0.3 m/s | odom lost=0, optimizer failure/NaN=0, Z span ≤0.05 m, 지도 접힘 없음 | 1회 부분 PASS |
| 2 | global visual loop | 시작/복귀 위치와 카메라 heading 동일, 각 3~5초 정지 | 올바른 Type-1 ≥1, false closure=0, 폐쇄 후 지도 품질 악화 없음 | **미통과** |
| 3 | 반복성 | 같은 짧은 경로 3회, 각 run 별도 DB | 3회 모두 Gate 0/1 통과, Type-1은 최소 2/3 run에서 성공, false closure=0 | 대기 |
| 4 | 전체 골든 맵 | Gate 0~3 통과 설정만 사용 | DB/PGM/YAML/3D map export, hash 고정, localization 재시작 성공 | 대기 |
| 5 | 4DoF 선택 실험 | 실제 경사 코스가 연구 범위일 때만 별도 수행 | 기준 고도와 궤적 비교, Z 발산·false closure 없음 | 선택 사항 |
| 6 | 4-Tier 무구동 폐루프 | 골든 맵 동결 후 Jetson→Docker→server→command sink | stale/timeout/malformed 응답이 항상 zero command, 실제 S2E checkpoint 확인 | 대기 |

## 5. 다음 global loop 실험

현재 내장 RGB에는 metric depth가 없어서 기본 visual PnP가 `Not enough features in images (old=0, new=...)`로 후보를 기각했다. 데이터베이스에는 2D visual feature가 충분히 기록되었으므로 다음 실험은 센서를 추가하기 전에 아래 최소 변경만 A/B 검증한다.

1. `RGBD/LoopClosureIdentityGuess=true`를 별도 시험 프로파일에 적용한다.
2. 시작 위치에서 특징적인 장면을 같은 heading으로 3~5초 관측한다.
3. 0.2~0.3 m/s로 짧은 loop를 주행하고 급회전·제자리 회전을 피한다.
4. 시작 pose와 같은 위치·heading으로 복귀해 3~5초 정지한다.
5. Type-1 후보가 identity guess에서 3D LiDAR ICP로 올바르게 검증되는지 확인한다.
6. 폐쇄 전후 endpoint gap, 이중벽, graph error, Z span을 비교한다.

이 방식이 반복적으로 실패하거나 큰 heading 차이의 재방문이 필수일 때만 D435i RGB-D를 다음 branch로 검토한다. 현재 baseline에는 D435i를 추가하지 않는다.

## 6. 계측 항목과 합격 규칙

각 run에서 다음을 같은 표에 남긴다.

| 분류 | 필수 기록 |
|---|---|
| 재현성 | run ID, git HEAD, 설정 snapshot, 시작/종료 시각, 주행 경로 |
| 센서 | cloud/image/odom/IMU rate, timestamp 역행, IMU interpolation warning |
| 궤적 | node 수, raw/optimized endpoint gap, x/y/z range |
| 루프 | Type-1/Type-2 승인 수, rejected 후보 수와 이유, 최고 hypothesis score |
| 그래프 | max linear/angular graph error, optimizer failure, NaN, map jump |
| 종료 | wrapper status, DB integrity, optimized pose 수, hash, 잔여 프로세스 |

실패 규칙은 다음과 같다.

- odometry lost, optimizer failure, NaN, false global closure, 갑작스러운 지도 접힘 중 하나라도 발생하면 FAIL.
- 평면 코스에서 raw 또는 optimized Z span은 0.05 m 이하면 PASS, 0.05~0.10 m는 WARN 후 재시험, 0.10 m 초과는 FAIL.
- `negative hessian index`만 있고 지도/최적화 실패가 없으면 WARN으로 기록하고 계속 판정한다.
- Type-2만 발생한 run은 LiDAR local closure PASS일 수 있지만 global visual loop PASS로 부르지 않는다.
- 정상 종료와 최종 optimized graph 저장이 확인되지 않은 DB는 최종 지도나 localization DB로 동결하지 않는다.

## 7. 현장 실행 순서

```bash
cd /home/unitree/go2_ws_antarctica
./mapping_planar_headless.sh
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
