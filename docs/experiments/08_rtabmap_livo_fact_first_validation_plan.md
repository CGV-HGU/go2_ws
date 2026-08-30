# RTAB-Map LIVO 맵 왜곡 원인 검증 계획서

> 작성일: 2026-08-28 KST  
> 대상: Unitree Go2 EDU Plus + Jetson Orin NX + ROS 2 Foxy + 내장 4D L2  
> 범위: RTAB-Map 매핑 품질과 처리 안정성 검증  
> 원칙: **현재 기준값을 먼저 재현하고, 실패가 관측된 층의 변수 하나만 바꾼다.**

## 1. 목적과 최종 산출물

이 계획의 목적은 맵이 찌그러지거나 중첩되는 원인을 다음 네 층으로 분리해 검증하는 것이다.

1. Unitree LIO 원 궤적 자체의 드리프트
2. RTAB-Map 처리 지연 또는 입력 동기화 문제
3. 잘못 승인된 loop constraint
4. pose graph는 정상이나 2D/3D map 표현만 나쁜 경우

검증을 마치면 다음 중 하나로 명확히 판정한다.

- **BASELINE PASS**: 현재 설정으로 golden map 촬영 가능
- **SPEED-LIMITED**: 저속은 통과하지만 목표 속도에서 upstream LIO 또는 처리율이 실패
- **LOOP FAILURE**: 잘못된 Type-1 global loop 또는 올바른 loop 미검출
- **TIMING/COMPUTE FAILURE**: 동기화 누락이나 RTAB-Map backlog가 원인
- **GRID-ONLY FAILURE**: 궤적은 정상이고 2D/3D 표현 파라미터만 조정 필요

## 2. 현재 고정할 기준값

아래 값은 짧은 실로봇 자격시험이 끝날 때까지 바꾸지 않는다.

| 항목 | 기준값 | 근거 |
|---|---:|---|
| graph | planar 3DoF (`x/y/yaw`) | 4DoF에서 발생한 비현실적 Z 발산을 제거함 |
| 3D 입력 | `/livo/cloud` | L2 deskewed 3D cloud를 `base_link`로 변환한 입력 |
| odometry | `/livo/odom` | Unitree LiDAR+IMU odometry를 외부 odom으로 사용 |
| visual 역할 | RGB place recognition | 단안 RGB이므로 metric visual odometry가 아님 |
| global loop | Type-1 visual retrieval + 3D LiDAR ICP | 전체 재처리에서 정상 형상을 유지함 |
| spatial proximity | Type-2 OFF | 이전 전체 맵 접힘의 주원인으로 재현됨 |
| neighbor refining | OFF | Unitree LIO 연속 링크를 희소 ICP로 재작성하지 않음 |
| detection rate | 2.0 Hz | 전체 맵 후반 처리 p95가 이미 0.397~0.451 s |
| ICP voxel | 0.05 m | 0.08 m A/B는 처리시간 이득 없이 graph가 발산함 |
| recorder | OFF | mapping DB, runtime 및 loop log만 증거로 저장 |
| Docker/VLM/motor | OFF | mapping 원인과 부하를 분리하고 물리 명령을 차단 |

현재 설정 확인 명령은 다음 하나만 사용한다.

```bash
cd /home/unitree/go2_ws_antarctica
./map_headless.sh --print-config
```

다음 변경은 기준 자격시험 전에 금지한다.

- `Rtabmap/DetectionRate`를 3 Hz 이상으로 올리기
- bridge와 RTAB-Map 양쪽에서 중복 downsample하기
- `Icp/VoxelSize=0.08` 또는 `0.10` 적용하기
- `RGBD/ProximityBySpace=true`를 일반 mapping에 다시 적용하기
- 3DoF와 4DoF를 같은 DB에서 섞기
- loop threshold와 ICP 조건을 여러 개 동시에 완화하기

## 3. 이미 확인되어 재조사하지 않을 사실

1. 3DoF 전환은 최신 맵 접힘의 원인이 아니다. 3DoF는 Z 발산을 억제했다.
2. 느린 전체 run의 두 90도 코너 접힘은 공격적인 Type-2 spatial proximity link가 주원인이었다.
3. 같은 1,707-node DB를 Type-2 OFF로 전체 재처리했을 때 Type-1 41개, Type-2 0개와 물리적으로 타당한 직교 형상이 유지됐다.
4. `Icp/VoxelSize=0.08` 재처리는 endpoint gap 82.783 m로 실패했으므로 5 cm를 유지한다.
5. 빠른 전체 run의 raw LIO endpoint gap 10.961 m는 RTAB-Map 출력 주기만 높여 직접 고칠 수 있는 문제가 아니다.
6. 현재 source와 설치 launch의 Type-2 OFF 설정은 일치한다.

위 결과는 저장 데이터에 대한 강한 소프트웨어 증거지만, **수정 후 실제 센서로 만든 새 지도**를 대신하지는 않는다.

## 4. 검증 단계

### Gate 0 — 실행 전 무변경 확인

로봇 이동 없이 다음을 기록한다.

- 현재 Git commit과 `git status --short`
- `./map_headless.sh --print-config` 출력
- 디스크 여유 공간
- 기존 `/home/unitree/.ros/rtabmap.db`의 backup 여부
- L2, odom, IMU, RGB topic의 존재·rate·timestamp freshness
- 실행 폴더가 `/home/unitree/.ros/rtabmap_runs/<run_id>/`로 생성되는지

합격 조건:

- 출력 profile이 `planar3dof`
- `Reg/Force3DoF=true`, `Icp/Force4DoF=false`
- `RGBD/ProximityBySpace=false`
- `Rtabmap/DetectionRate=2.0`, `Icp/VoxelSize=0.05`
- recorder, Docker/VLM, command bridge가 모두 꺼져 있음

하나라도 다르면 주행하지 않고 configuration mismatch로 종료한다.

### Gate 1 — 정지 센서 자격시험

로봇을 정상 기립시킨 뒤 움직이지 않고 약 30초 관측한다. 이 단계에서는 주행 명령을 보내지 않는다.

측정 항목:

- cloud/odom/IMU stamp가 단조 증가하는가
- cloud와 대응 odom의 stamp 차이
- 정지 중 odom step과 비현실적 Z drift
- quaternion order 자동 판정과 gravity residual
- bridge error, clock jump, NaN/Inf, odometry reset/lost
- RTAB-Map 입력 callback이 지속되는가

합격 조건:

- cloud, odom, IMU가 중단 없이 갱신됨
- odometry lost/reset, clock jump, NaN/Inf가 0건
- 정지 상태에서 지속적인 위치 발산이 없음
- 카메라와 L2가 모두 RTAB-Map DB node에 저장됨

### Gate 2 — 1~2분 짧은 geometry/loop 자격시험

운영자가 E-stop/즉시 정지 수단을 확보하고 수동 조종한다. 자율주행 명령은 사용하지 않는다.

경로:

1. 바닥에 출발 위치와 heading을 표시한다.
2. 출발점에서 같은 방향을 보며 3~5초 정지한다.
3. 직선 구간과 실제 90도 코너 두 개를 지난다.
4. 초기 자격시험은 직진 `0.2~0.3 m/s`, 회전 `15~25 deg/s`로 제한한다.
5. 코너에서는 전진과 회전을 분리한다.
6. 출발점에 처음과 같은 heading으로 접근해 3~5초 정지한다.
7. 터미널에서 `Ctrl+C`를 한 번 눌러 정상 종료한다.

실행 명령:

```bash
cd /home/unitree/go2_ws_antarctica
./map_headless.sh
```

`map_headless.sh`가 DB, runtime log, loop event, config snapshot, manifest와 SHA-256을 자동 보존한다. 별도 rosbag recorder는 켜지 않는다.

### Gate 3 — 주행 직후 read-only 판정

원본 run DB에서 `rtabmap-export`를 직접 실행하지 않는다. 일부 RTAB-Map 도구는 DB의 Admin cache를 갱신할 수 있으므로, export가 필요하면 반드시 복사본이나 `/tmp` 재처리 출력만 사용한다.

자동 판정기는 SQLite를 `mode=ro&immutable=1`로 열고 분석 전후 DB SHA-256이 같은지 검사한다.

```bash
cd /home/unitree/go2_ws_antarctica
./analyze_map_run.py                           # ~/.ros/rtabmap_runs/latest
./analyze_map_run.py /path/to/saved/run_dir   # 특정 run
```

결과는 `~/.ros/rtabmap_analysis_runs/<analysis_id>/`의 `report.json`, `report.md`,
`SHA256SUMS`에 저장된다. 이 도구는 `rtabmap-export`를 호출하지 않는다.

필수 측정값:

| 분류 | 필수 값 |
|---|---|
| 무결성 | run ID, DB SHA-256, `PRAGMA integrity_check`, node 수, duration |
| 입력 | cloud/odom/IMU rate, sync 누락, IMU interpolation warning |
| 안정성 | odometry lost/reset, optimizer failure, NaN/Inf |
| 처리율 | RTAB-Map mean/p95/max, 0.5초 초과 횟수, 연속 backlog 여부 |
| graph | Type-0/1/2/9 unique link 수와 Type-1 node pair |
| 궤적 | raw/optimized path length, endpoint XY gap, Z span |
| 보정량 | raw 대비 optimized XY correction p95/max와 큰 map-to-odom jump |
| 형상 | 두 90도 코너, 평행 벽, 자기교차, 이중벽, 3D 점군의 층 분리/기울기 |

Type-1은 개수만 세지 않는다. 연결된 두 node의 RGB keyframe, 위치와 L2 중첩을 확인해 실제 같은 장소인지 판정한다.

2026-08-30 기존 실패 run `20260828_141247` 재검증에서 analyzer는 1,707 nodes, Type-1 36,
Type-2 159, optimized Z span 0.031804 m, 처리 p95 0.3973 s를 복원했고 Type-2 nonzero로
`FAIL_OR_INCOMPLETE_AUTOMATED_GATES`를 출력했다. 분석 전후 source DB hash는 동일했다.

### Gate 4 — baseline 합격 기준

다음 조건을 모두 만족해야 현재 설정을 `BASELINE PASS`로 판정한다.

- DB integrity `ok`, hash manifest 검증 성공
- Type-2 unique link **0개**
- 올바른 Type-1 global loop **1개 이상**
- odometry lost/reset, optimizer failure, NaN/Inf **0건**
- 승인된 loop 직후 비물리적 순간 이동이나 graph 접힘이 없음
- 평탄 코스 optimized Z span이 0.10 m 이내
- 표시한 출발점 기준 optimized endpoint gap이 0.25 m 이내이고 raw gap보다 악화되지 않음
- RTAB-Map 처리 p95가 0.5초 미만이며 연속 backlog가 없음
- 실제 두 90도 코너와 평행 벽이 유지되고 자기교차가 없음
- 2D 중첩벽과 3D 기울기가 같은 pose 오류에서 발생하지 않음

한 번의 통과는 기능 확인이다. Golden map 촬영 전 같은 짧은 경로를 총 3회 수행해 3/3 통과해야 반복성 합격으로 기록한다.

## 5. 실패 시 원인 분기

```text
raw LIO 궤적부터 틀림?
  ├─ 예 → 속도/보행 진동/Unitree LIO 층 조사
  └─ 아니오
       ├─ 처리 p95≥0.5 s 또는 연속 backlog? → compute/WM/rate 층 조사
       ├─ 잘못된 Type-1 승인? → RGB 후보·ICP 검증·카메라 보정 조사
       ├─ graph는 정상, PGM/3D cloud만 나쁨? → Grid 파라미터 조사
       └─ sync/IMU 누락이 반복됨? → bridge timing/cache 조사
```

### A. raw LIO가 먼저 실패한 경우

RTAB-Map 파라미터를 바꾸지 않는다. 같은 짧은 경로에서 속도만 바꾼다.

- A0: `0.2~0.3 m/s`
- A1: 실제 목표 운용 속도
- 각 조건 최소 3회

raw endpoint gap, 코너 각도, yaw drift와 보행 구간의 순간 pose step을 비교한다. 빠른 조건에서만 raw LIO가 실패하면 `SPEED-LIMITED`이며 RTAB-Map detection rate 변경으로 해결됐다고 주장하지 않는다.

### B. 처리율이 실패한 경우

저장 DB로 먼저 오프라인 A/B한 뒤 실로봇 후보로 올린다.

우선순위:

1. graph 크기 증가에 따른 처리시간과 Working Memory 크기 상관 확인
2. `Rtabmap/TimeThr` 또는 `Mem/MemoryThr`를 한 변수씩 시험
3. global Type-1 수와 정합 품질이 보존되는지 확인
4. 그 뒤에만 동일 짧은 경로에서 2.0 Hz 대 2.5 Hz 비교

3 Hz부터 시작하지 않는다. 2.5 Hz는 프레임당 0.4초 예산이므로 p95, sync drop과 오래된 callback 여부가 모두 기준값보다 나빠지지 않아야 한다.

### C. 점군 처리량을 줄이고 싶은 경우

현재 ICP는 이미 5 cm voxel filtering을 수행한다. bridge에서 무조건 점을 버리면 중복 downsample이 된다.

후보 변경은 다음 순서로만 검증한다.

1. CPU profiler로 bridge serialization/DDS가 실제 병목인지 증명
2. 원본 DB를 immutable/read-only 입력으로 고정
3. 한 후보만 `/tmp` 전체 재처리
4. 처리시간, Type-1 pair, endpoint gap, correction p95/max, 코너 형상 비교
5. 모든 품질 지표가 baseline 비열등일 때만 짧은 live A/B

8 cm와 10 cm는 현재 후보에서 제외한다.

### D. loop만 실패한 경우

`Rtabmap/LoopThr`를 먼저 낮추지 않는다.

1. 출발/복귀 heading과 RGB keyframe이 실제로 유사한지 확인
2. 두 node의 L2 scan 중첩을 확인
3. camera/cloud timestamp 간격을 확인
4. 유리문, 반복 복도와 움직이는 물체의 영향을 확인
5. 추정 상태인 카메라 intrinsics/extrinsics를 실측 보정
6. 이후에 ICP 초기화 또는 threshold 하나만 A/B

### E. graph는 정상이고 map 표현만 실패한 경우

raw/optimized 궤적과 loop가 합격한 뒤에만 아래를 각각 별도 run으로 비교한다.

- `Grid/RayTracing`
- `Grid/NoiseFilteringRadius`
- `Grid/NoiseFilteringMinNeighbors`
- ground/obstacle height 범위

PGM 외형 개선을 SLAM 정확도 개선으로 혼동하지 않는다. 2D PGM, optimized trajectory와 3D PLY를 함께 판정한다.

## 6. Golden map과 목표 속도 검증 순서

1. 기준 설정 짧은 loop 3/3 통과
2. 같은 짧은 코스에서 저속 대 목표 속도 A/B 각 3회
3. 목표 속도에서도 raw LIO, 처리율과 global loop Gate 통과
4. 전체 구역을 `0.2~0.3 m/s`로 한 번 mapping
5. 전체 지도에서 물리 코너·복도 폭·기준점 거리 검증
6. 합격 DB를 immutable golden DB로 복사하고 SHA-256 고정
7. 별도 localization 모드 자격시험

RTAB-Map endpoint gap만으로 절대 정확도를 주장하지 않는다. 논문용 평가는 AprilTag, 실측 기준점, PixNav waypoint 오차 또는 독립 reference trajectory 중 하나와 비교한다.

## 7. 실행 기록 양식

| 항목 | 기록 |
|---|---|
| 날짜/운영자 | |
| run ID | |
| Git commit / dirty state | |
| profile | planar3dof / 2 Hz / 5 cm / Type-2 OFF |
| 경로와 속도 | |
| DB SHA-256 / integrity | |
| raw path / gap / Z span | |
| optimized path / gap / Z span | |
| Type-1 pairs / Type-2 count | |
| processing mean / p95 / max | |
| odom lost / IMU warning / optimizer error | |
| 2D/3D 형상 판정 | |
| 최종 판정 | PASS / SPEED-LIMITED / LOOP / TIMING / GRID |
| 다음 단일 변경 | |

## 8. 다음 작업 한 줄 요약

**파라미터를 더 바꾸지 말고, 현재 3DoF·2 Hz·5 cm·Type-2 OFF 설정으로 두 90도 코너와 출발점 재방문이 포함된 1~2분 짧은 loop를 먼저 1회 실행해 물리 baseline을 확인한다.**
