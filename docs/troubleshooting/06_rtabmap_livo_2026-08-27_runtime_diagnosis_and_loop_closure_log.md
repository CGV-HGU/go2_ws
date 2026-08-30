# RTAB-Map LIVO `rtabmap0827` 실측 진단 및 Loop Closure 로그

> 기록 시각: 2026-08-27T15:00:22+09:00  
> 대상 Git commit: `c977dee555ad396aab1483eecccd6631737abe8c`  
> 대상 플랫폼: Unitree Go2 EDU Plus, Jetson Orin NX, Ubuntu 20.04, ROS 2 Foxy  
> 대상 센서: Go2 내장 Unitree 4D LiDAR L2, 내장 LiDAR IMU, 전방 단안 RGB 카메라  
> 안전 범위: 저장 파일과 로그의 read-only 진단. 로봇 구동 명령은 발행하지 않음.  
> 문서 상태: 이번 실행의 직접 측정값은 `실측`, 아직 재주행하지 않은 변경은 `권장/미검증`으로 표시함.

> **최신 상태 안내 (2026-08-28 18:50 KST)**: 이 문서는 누적 진단 기록이다. 1~20절의 중간 판정은
> 이후 실측으로 일부 철회됐으며, 현재 판정은 **21~22절이 우선**한다. planar 3DoF는 Z 발산을
> 막았고 맵 접힘의 원인이 아니었다. 느린 전체 주행의 심한 접힘은 Type-2 spatial proximity
> constraint가 원인이었으며, 현재 canonical profile은 `RGBD/ProximityBySpace=false`다. 동일 DB
> 전체 재처리에서 Type-2 0건, Type-1 41건과 물리적으로 타당한 코너 형상을 확인했다. 5 cm ICP
> voxel은 유지하고 8 cm 후보는 오프라인 A/B 실패로 기각했다. 수정 후 짧은 실로봇 자격 loop는
> 아직 필요하다. 한눈에 보는 과거 문제→해결 표는
> [`master_plan/[2026-08-27]_RTAB-Map_LIVO_문제_원인_해결_및_재검증_총정리.md`](../master_plan/[2026-08-27]_RTAB-Map_LIVO_문제_원인_해결_및_재검증_총정리.md)도 참조한다.

## 1. 결론

이번 `rtabmap0827` 실행에서는 **승인된 loop closure가 0건**이었다. 따라서 현재 2D 지도는 loop closure로 좋아지거나 나빠진 결과가 아니라, Unitree LIO 궤적과 RTAB-Map의 이웃 링크 ICP 보정이 누적된 결과다.

Loop closure는 다음 조건을 모두 만족하면 시작점과 종료점 사이의 누적 드리프트를 전역적으로 분산시켜 지도를 더 정확하게 만들 가능성이 높다.

1. 로봇이 과거 장소를 다시 관측한다.
2. RGB bag-of-words가 과거 노드를 loop 후보로 찾는다.
3. 해당 후보의 L2 점군 사이에 충분한 중첩이 있다.
4. LiDAR ICP 기하 검증이 성공한다.
5. 올바른 loop constraint가 pose graph에 추가된다.

반대로 잘못된 후보가 승인되면 전체 지도를 접거나 비틀 수 있다. 따라서 loop closure는 항상 정확도를 높이는 기능이 아니라, **정확한 constraint가 승인됐을 때만** 정확도를 높이는 기능이다.

이번 지도에서 더 큰 문제는 loop closure 부재보다 다음 설정이었다.

```python
'RGBD/NeighborLinkRefining': 'true'
```

이 설정이 RTAB-Map의 외부 odometry 입력으로 들어오는 내장 Unitree LIO의 연속 이웃 링크를 희소한 L2 점군 ICP로 219회 다시 추정했다. 그 결과 RTAB-Map 최적화 궤적이 원본 LIO보다 크게 변형되었다. 다음 A/B 실행에서는 이 항목만 먼저 `false`로 변경하는 것이 1순위다. 이 변경은 이 문서 작성 시점에는 아직 적용하지 않았다. 이 값을 `false`로 바꿔도 RGB loop 후보 검색과 후보에 대한 LiDAR ICP 검증은 계속 동작하며, 연속 neighbor link의 재보정만 꺼진다.

## 2. 현재 시스템의 정확한 명칭과 데이터 경로

현재 구성은 연속 프레임마다 카메라 자세를 추정하는 metric visual odometry가 아니다. 정확한 구성은 다음과 같다.

```text
/utlidar/robot_odom ─┐
/utlidar/imu        ─┼─> go2_livo_sensor_bridge.py
/utlidar/cloud_deskewed ┘          │
                                   ├─> /livo/odom
                                   ├─> /livo/imu
                                   └─> /livo/cloud (base_link)

/camera/front/image_raw + camera_info
    └─> RGB visual place recognition / loop 후보 검색

/livo/cloud
    └─> LiDAR ICP 기하 검증 및 3D/2D map 생성
```

따라서 현재 시스템은 다음처럼 부르는 것이 가장 정확하다.

> **Unitree LiDAR+IMU odometry(LIO) + RGB visual place recognition + LiDAR ICP loop validation**

전방 카메라는 단안 RGB이고 calibrated depth/stereo가 없으므로 현재 RTAB-Map에 metric visual odometry를 공급하지 않는다. `Reg/Strategy=1`은 registration에 ICP를 사용한다. RGB는 visual words와 장소 후보 검색에 사용된다.

관련 코드:

- `scratch/go2_livo_sensor_bridge.py`
- `src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py`
- `run_map.sh`
- `scratch/bringup_all_escape_nav.sh`

공식 근거:

- RTAB-Map 파라미터 정의: <https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h>
- RTAB-Map ROS 2 3D LiDAR 예제: <https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/lidar3d.launch.py>
- RTAB-Map link type 구현: <https://github.com/introlab/rtabmap/blob/master/corelib/src/Link.cpp>

## 3. 분석 대상 아티팩트와 무결성

| 아티팩트 | 크기/시각 | SHA-256 |
|---|---|---|
| `/home/unitree/.ros/rtabmap0827.pgm` | 244,843 bytes, 14:44:37 KST | `27e00f1d36755de6f165a32969f090fe4ee9c97a0bbe673da2cf0437a59afe17` |
| `/home/unitree/.ros/rtabmap0827.yaml` | 125 bytes, 14:44:37 KST | `9cbe953f74f8f9912b33527a1dcd62aa3cea1b2b6c1dae5e8ceaf568b5f008b7` |
| `/home/unitree/.ros/rtabmap.db` | 112,697,344 bytes | `3226b9f6d1fa12a3f5848aafdfddfa10c8960df0700698bb62b1f45ffc18d813` |
| `/home/unitree/.ros/log/rtabmap_35212_1787809142102.log` | 77,665 bytes, 종료 14:42:56 KST | `187fa6c50d070557c7eb5846a0c4ac5e31cd5d953081db020989fbb87441ab34` |

DB SQLite `PRAGMA integrity_check` 결과는 `ok`다.

주의: 진단 중 `rtabmap-export`로 raw/optimized pose를 추출했을 때 이 도구가 `Admin.opt_poses`, `Admin.opt_map`, `Admin.time_enter` 캐시를 갱신하여 DB 수정 시각이 14:54:20 KST로 변경되었다. `Node`, `Data`, `Feature`, `Statistics`, `Link` 행 수와 graph/sensor 데이터는 바뀌지 않았으며 무결성도 `ok`다. 향후 동일 DB 진단은 SQLite URI `mode=ro`를 우선 사용한다.

## 4. 이번 실행에서 loop closure가 실제로 동작했는가

### 4.1 DB 실측값

| 항목 | 실측값 | 해석 |
|---|---:|---|
| 전체 입력 프레임/노드 | 402 | 약 232.38초 실행 |
| RTAB-Map graph keyframe | 220 | `weight=0` |
| intermediate node | 182 | `weight=-9` |
| RGB JPEG image | 402 | RGB 구독 정상 |
| LiDAR scan | 402 | scan cloud 구독 정상 |
| ORB feature | 350,459 | 노드당 평균 약 872개 |
| 3D depth가 있는 visual feature | 0 | 단안 RGB이므로 정상적인 결과 |
| neighbor link, type 0 | 219 | 연속 keyframe 사이 링크 |
| gravity link, type 9 | 395 | IMU가 graph에 포함된 노드 |
| global closure, type 1 | **0** | 승인된 visual/global loop 없음 |
| local-space closure, type 2 | **0** | 승인된 proximity loop 없음 |

`Link.cpp`의 공식 type 이름 기준으로 type 0은 `Neighbor`, type 1은 `GlobalClosure`, type 2는 `LocalSpaceClosure`, type 9는 `Gravity`다.

### 4.2 visual 후보 통계

- `Loop/Highest_hypothesis_id`가 0이 아닌 프레임: 329/402
- 최고 visual hypothesis score: 약 0.1524
- `Loop/Accepted_hypothesis_id`가 0이 아닌 프레임: 0
- `Loop/Id`가 0이 아닌 프레임: 0
- `Loop/RejectedHypothesis=1`: 13개 프레임
- 마지막 402번 프레임의 최고 후보: node 242, score 약 0.0628
- 설정된 `Rtabmap/LoopThr`: 0.11

즉 RGB 특징 추출과 장소 후보 검색은 실행됐다. 다만 마지막 구간은 threshold보다 낮았고, threshold를 넘은 일부 후보도 LiDAR ICP 검증에서 거부됐다. 이번 경로는 같은 장소를 같은 시야와 충분한 점군 중첩으로 다시 보는 구간이 부족했으므로, 사용자의 관찰과 DB 결과가 일치한다.

### 4.3 “한 바퀴 돌았다”와 “loop closure가 가능하다”의 차이

좌표상 출발점 근처로 돌아오는 것만으로는 visual loop가 보장되지 않는다.

- 카메라가 반대 방향을 보고 있으면 이미지 외형이 크게 다르다.
- 복도처럼 반복 무늬가 많으면 후보 score가 낮거나 잘못된 후보가 생긴다.
- L2 scan이 짧은 순간만 겹치거나 가림이 많으면 ICP 검증이 실패한다.
- 현재 detection rate는 2 Hz이므로 재방문 구간이 너무 짧으면 비교할 keyframe이 부족할 수 있다.
- 회전 직후 한 프레임만 시작점을 스쳐 지나가면 동일 방향 재관측보다 성공률이 낮다.

따라서 이번 데이터는 “loop closure가 정확도를 개선하지 못했다”가 아니라, **loop closure가 한 번도 승인되지 않아 개선 효과가 적용되지 않았다**고 해석해야 한다.

## 5. 왜 `rtabmap0827.pgm`이 삐뚤어졌는가

### 5.1 raw LIO와 RTAB-Map optimized pose 비교

| 지표 | raw Unitree LIO | RTAB-Map optimized |
|---|---:|---:|
| 시작점-종료점 XY gap | 약 0.772 m | 약 2.289 m |
| 전체 z 범위 | 약 0.226 m | 약 1.434 m |
| 시작 z | 약 0.307 m | 약 0.307 m |
| 종료 z | 약 0.090 m | 약 -1.119 m |

동일 timestamp의 optimized keyframe을 raw LIO와 비교한 추가 지표:

- XY correction RMS: 약 1.059 m
- XY correction maximum: 약 2.097 m
- yaw correction RMS: 약 7.42도
- yaw correction maximum: 약 16.23도
- z correction RMS: 약 0.621 m

raw LIO도 실제 손조종 경로와 센서 드리프트 때문에 완전한 직선은 아니다. 그러나 RTAB-Map 최적화 후 변형량이 훨씬 크며, 평탄한 동일 층에서 종료 z가 약 -1.12 m로 내려간 것은 실제 구조보다 잘못된 graph correction을 강하게 시사한다.

### 5.2 직접 원인: neighbor-link ICP 재보정

현재 launch의 관련 설정:

```python
'Reg/Strategy': '1'
'Icp/Force4DoF': 'true'
'RGBD/NeighborLinkRefining': 'true'
'Icp/CorrespondenceRatio': '0.15'
'Icp/PointToPlane': 'true'
'Icp/VoxelSize': '0.05'
'Icp/MaxCorrespondenceDistance': '0.20'
```

실측된 neighbor refinement 통계:

- 219개 neighbor link가 대상이 됨.
- ICP inlier ratio가 0보다 큰 링크는 125개뿐임.
- ICP 회전 correction 최대 약 0.312 rad, 즉 약 17.9도.
- ICP translation correction 최대 약 0.287 m.
- 복도 scan은 평행 벽/바닥 위주라 point-to-plane ICP의 특정 축 관측성이 약함.

L2 점군은 희소하고, 긴 복도는 기하학적으로 퇴화하기 쉬운 환경이다. 이런 조건에서 외부 LIO의 모든 연속 링크를 다시 ICP로 고치면 작은 오차가 누적된다. `Icp/Force4DoF=true`는 x, y, z, yaw correction을 허용하므로 z 오차도 누적될 수 있다. `Optimizer/GravitySigma`는 주로 roll/pitch 중력 정렬을 돕지만 z translation을 고정하지 않는다.

## 6. 2D map의 방사형 흰 줄과 끊어진 벽

`rtabmap0827.pgm`은 다음 세 값으로 구성된다.

- occupied/black: 2,296 cells
- unknown/gray: 172,557 cells
- free/white: 69,947 cells

방사형 흰 줄은 주로 `Grid/RayTracing=true`가 희소한 L2 return 각각의 free ray를 격자에 투영한 결과다. 이는 pose graph가 휘는 문제와 별개의 grid 표현 문제다.

또한 다음 설정은 희소 점군에서 벽 점을 과도하게 제거할 수 있으므로 별도 A/B 확인이 필요하다.

```python
'Grid/NoiseFilteringRadius': '0.15'
'Grid/NoiseFilteringMinNeighbors': '5'
```

권장 순서는 pose graph를 먼저 안정화한 뒤 grid를 조정하는 것이다. 두 문제를 동시에 바꾸면 어떤 변경이 효과를 냈는지 알 수 없다.

## 7. IMU 상태

RTAB-Map 로그에 다음 경고가 총 7회 있었다.

```text
We are receiving imu data (...), but cannot interpolate imu transform at time ...
IMU won't be added to graph.
```

경고 timestamp:

1. `1787809143.210297`
2. `1787809144.251863`
3. `1787809144.837805`
4. `1787809145.358309`
5. `1787809146.008810`
6. `1787809237.526817`
7. `1787809282.633962`

402개 노드 중 gravity link가 395개인 것과 정확히 일치한다. 초기 5개와 실행 중 2개 노드에는 IMU constraint가 추가되지 않았다.

가능성이 높은 원인은 RTAB-Map이 RGB/cloud synchronized callback을 처리하는 순간 해당 시각보다 뒤의 IMU sample이 아직 도착하지 않아 보간할 수 없었던 것이다. 전체 노드의 대부분에는 gravity link가 있으므로 이번 대규모 변형의 1차 원인은 아니지만 보완 대상이다.

권장/미검증 보완:

- bridge가 cloud를 즉시 publish하지 않고 약 20~30 ms 보관한다.
- 최신 IMU/odom source stamp가 cloud stamp 이상이 된 뒤 cloud를 publish한다.
- 다음 실행에서 `cannot interpolate imu transform`가 0건인지 확인한다.

## 8. 문제 및 해결 상태 마스터 로그

| ID | 문제 | 증거/영향 | 해결 또는 다음 조치 | 상태 |
|---|---|---|---|---|
| MAP-001 | deskewed cloud XYZ는 `odom`인데 frame만 sensor로 바꾸던 오류 | 이동 시 점군 중첩 붕괴 가능 | 시간 대응 odom pose의 역변환으로 `base_link` cloud 생성 | 적용 및 정지 상태 실측 완료 |
| MAP-002 | cloud의 10,000 zero-padding record | 유효하지 않은 원점 점들이 map/ICP에 혼입 | non-finite 및 zero point 제거 | 적용 및 실측 완료 |
| MAP-003 | LiDAR clock과 Jetson camera clock 불일치 | approximate sync 불안정 | odom/IMU/cloud에 하나의 공통 LiDAR-to-host offset 적용 | 적용 완료 |
| MAP-004 | 내장 IMU quaternion field 순서 불일치 | 잘못된 중력 방향과 graph 기울기 | gravity residual로 `xyzw`/`wxyz` 자동 판정, 이번 장치에서 `wxyz` 선택 | 적용 및 실측 완료 |
| MAP-005 | built-in DDS와 external Ethernet SDK 경로 혼용 위험 | topic/frame/clock이 섞일 수 있음 | 주 mapping은 `/utlidar/*`; external SDK는 `/external_l2/*`로 분리 | 적용 완료 |
| MAP-006 | mapping launch의 `-d`가 DB 삭제 | 이전 map 유실 위험 | mapping wrapper가 기존 DB를 timestamp backup한 뒤 시작 | 적용 완료; wrapper 우회 금지 |
| MAP-007 | mapping 중 recorder와 command bridge 불필요 | 저장공간/안전 범위 확대 | mapping mode에서 recorder/actuation bridge 비활성, recorder는 명시적 `--record`만 | 적용 완료 |
| MAP-008 | 단안 RGB를 visual odometry로 오해 | 시스템 역할 판단 오류 | RGB는 visual place recognition, metric registration은 L2 ICP라고 명시 | 문서/launch 주석 반영 |
| MAP-009 | `NeighborLinkRefining=true`가 내장 LIO를 RTAB-Map에서 재보정 | XY 최대 2.097 m, yaw 최대 16.23도, 첫 지도 z 범위 1.434 m | 이 값을 `false`로 변경하고 A/B 실행 | **적용 및 2차 실주행 확인; 2D 개선, 별도 3D z 문제 잔존** |
| MAP-010 | 첫 실행의 승인 loop closure 0건 | 첫 DB type 1/2 link 0 | 동일 장소·유사 시야·충분한 scan overlap으로 loop test | **2차 실행 type 2 근접 폐쇄 5건; type 1 전역 시각 폐쇄는 여전히 0건** |
| MAP-011 | IMU interpolation 7회 실패 | gravity link 395/402 | cloud publish를 IMU보다 20~30 ms 늦추는 cache 추가 | 2차 실행도 6회; 권장, 미구현 |
| MAP-012 | ray tracing 흰색 spike | PGM free-space 방사선 | graph 안정화 후 `RayTracing` 및 noise filter를 별도 A/B | 보류 |
| MAP-013 | camera intrinsics/extrinsics가 추정값 | viewpoint/향후 VisIcp 신뢰성 제한 | checkerboard/AprilTag 기반 실측 calibration | 미해결 |
| MAP-014 | 일부 기존 문서가 `NeighborLinkRefining=true`를 무조건 권장 | 이번 L2+외부 LIO 실측과 충돌 | 이번 실행 보고서를 우선 증거로 사용하고 설정을 A/B 검증 | 이 문서로 정정 |
| MAP-015 | 진단 export가 DB Admin cache를 갱신 | DB mtime 14:54로 변경 | graph/sensor 행 불변과 integrity 확인; 이후 SQLite `mode=ro` 우선 | 영향 확인 완료 |
| MAP-016 | 4DoF graph의 z 발산 | 2차 raw LIO z 0.0212 m 대비 RTAB map z 6.452 m | 단층 실내에서 3DoF/Slam2D A/B 후 deployment 설정 고정 | **미해결, 다음 최우선 실험** |
| MAP-017 | logger가 RTAB 0.21.1 trailing `/` 통계 키를 놓침 | text log가 rejected=0으로 오기록 | trailing `/`, GUI/headless label, signal SUMMARY 처리 수정 | 코드/모의시험 완료; 다음 실주행 확인 필요 |
| MAP-018 | 과거 문서의 “RTAB-Map 50 Hz” 표현 | 실제 `Rtabmap/DetectionRate=2.0` | 50 Hz LIO pose와 2 Hz graph update를 구분 | 문서 교정 |
| MAP-019 | `localization:=true`를 pure odometry로 오해 | 실제 기존 DB를 읽는 localization 모드 | mapping/localization 역할과 논문 사용 범위 분리 | 문서 교정 |

## 9. 다음 loop closure 검증 절차

이 절차는 Codex가 로봇을 구동한다는 의미가 아니다. 실제 이동은 사용자가 감독하고 E-stop/정지 수단을 확보한 상태에서 수동으로 수행해야 한다.

### 단계 A: local trajectory 안정화

다음 한 항목만 변경한다.

```python
'RGBD/NeighborLinkRefining': 'false'
```

다른 ICP, grid, camera 파라미터는 그대로 둔다. 목적은 raw LIO와 RTAB-Map optimized path가 거의 같은지 확인하는 것이다.

기대 결과:

- 길고 곧은 벽의 굽음 감소
- optimized z range가 raw LIO z range에 가까워짐
- loop가 없어도 endpoint gap이 이번 optimized 값 2.289 m보다 raw LIO 값 0.772 m에 가까워짐

### 단계 B: 명시적인 재방문 구간 만들기

수동 주행 시 다음 조건을 만든다.

1. 출발 장소에서 2~3초 정지하여 여러 keyframe을 남긴다.
2. 한 바퀴를 돈다.
3. 마지막 2~3 m는 출발 때와 가능한 한 같은 방향/시야로 접근한다.
4. 출발 위치 근처에서 다시 2~3초 정지한다.
5. 단순히 좌표만 겹치지 말고 카메라 영상과 L2 scan이 모두 충분히 겹치게 한다.

### 단계 C: 성공 판정

최소 성공 조건:

- DB에 `Link.type=1` 또는 `Link.type=2`가 1개 이상 존재
- `Loop/Accepted_hypothesis_id` 또는 `Loop/Id`가 0이 아닌 통계 존재
- loop 승인 직후 map이 접히거나 순간 이동하지 않음
- optimized endpoint gap이 raw LIO보다 감소
- optimized z가 비현실적으로 1 m 이상 변형되지 않음
- PGM의 동일 벽이 이중선으로 크게 벌어지지 않음

loop link가 없으면 loop closure 튜닝 결과로 판정하지 않는다.

### 단계 D: loop 후보는 있으나 ICP가 계속 거부될 때

`Rtabmap/LoopThr`부터 무작정 낮추지 않는다. 먼저 다음을 확인한다.

1. 재방문 방향과 RGB 시야가 유사한가.
2. L2 scan 중첩이 충분한가.
3. camera image와 scan timestamp 차이가 허용 범위 안인가.
4. 동일 장소에서 glass/open doorway 등으로 point geometry가 바뀌지 않았는가.
5. `Icp/MaxCorrespondenceDistance=0.20 m`가 초기 오차에 비해 너무 좁지 않은가.

그다음 별도 실행에서만 voxel size와 ICP correspondence distance를 조정한다. 파라미터를 여러 개 동시에 바꾸지 않는다.

## 10. 권장 파라미터 변경 우선순위

| 순서 | 변경 | 목적 | 현재 상태 |
|---:|---|---|---|
| 1 | `RGBD/NeighborLinkRefining=false` | 내장 LIO 연속 링크 보존 | 소스 적용, 다음 실행 검증 |
| 2 | bridge cloud 20~30 ms cache | IMU interpolation 누락 제거 | 코드 변경 필요 |
| 3 | 명시적 loop 재방문 실행 | 실제 closure 효과 평가 | 실물 테스트 필요 |
| 4 | 평탄 단일층에서만 `Reg/Force3DoF=true`, `Icp/Force4DoF=false` A/B | z correction 차단 | 1~3 결과 후 판단 |
| 5 | ICP voxel/correspondence 범위 A/B | sparse L2 loop 검증률 개선 | closure 후보 확인 후 |
| 6 | grid ray tracing/noise filter A/B | 2D PGM 외형 개선 | pose graph 안정화 후 |

`Reg/Force3DoF`와 `Icp/Force4DoF`를 1단계부터 동시에 바꾸지 않는 이유는 `NeighborLinkRefining`의 영향과 평면 구속의 영향을 분리하기 위해서다.

## 11. read-only 재확인 명령

아래 명령은 DB를 수정하지 않는다.

```bash
python3 - <<'PY'
import sqlite3

path = "/home/unitree/.ros/rtabmap.db"
con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
print("integrity:", con.execute("pragma integrity_check").fetchone()[0])
print("nodes:", con.execute("select count(*) from Node").fetchone()[0])
print("links:", con.execute(
    "select type, count(*) from Link group by type order by type"
).fetchall())
con.close()
PY

rg -n "cannot interpolate imu transform" \
  /home/unitree/.ros/log/rtabmap_35212_1787809142102.log
```

현재 mapping launch는 mapping mode에서 `-d`를 사용한다. 새 map을 시작할 때는 반드시 `run_map.sh` 또는 backup이 보장된 wrapper를 사용하고 launch 파일을 직접 실행하지 않는다.

## 12. 현재 종료 상태

문서 작성 시점에 다음 프로세스는 실행 중이지 않았다.

- `rtabmap`
- `rtabmap_viz`
- `go2_livo_sensor_bridge.py`
- `ros2 bag record`

Recorder는 mapping 기본 경로에서 계속 비활성 상태다.

## 13. 다음 업데이트 규칙

다음 실행 후 이 문서에 아래 항목을 추가한다.

1. 실행 시작/종료 KST와 Git commit
2. 사용한 launch parameter diff
3. DB, PGM, YAML, RTAB-Map 로그의 절대 경로와 SHA-256
4. `Link.type`별 개수
5. accepted/rejected loop 수
6. raw LIO와 optimized trajectory의 endpoint gap, z range, XY/yaw correction
7. IMU interpolation warning 수
8. 같은 구간 PGM의 직선성 비교

실제 loop link가 생성되고 지표가 개선되기 전에는 “loop closure 검증 완료” 또는 “정확도 향상 완료”라고 기록하지 않는다.

## 14. Headless loop-event logger 추가 (2026-08-27 15:13 KST)

디스플레이 없이 수동 mapping을 수행하면서 loop 결과를 보존할 수 있도록 다음을 적용했다.

- `scratch/rtabmap_loop_logger.py` 추가
- `map_headless.sh`가 사용하는 mapping mode에 logger 자동 통합
- `Rtabmap/PublishStats=true` 명시
- `RGBD/NeighborLinkRefining=false` 적용
- recorder, Docker/VLM, host command bridge는 계속 비활성

Logger는 RTAB-Map `/info`를 읽기만 하며 ROS publisher나 제어 인터페이스가 없다. 기존 4DoF wrapper의 출력 경로:

```text
/home/unitree/.ros/rtabmap_loop_logs/loop_events_<time>_headless_mapping.jsonl
/home/unitree/.ros/rtabmap_loop_logs/loop_events_<time>_headless_mapping.log
```

기록 이벤트:

- `ACCEPTED_GLOBAL`: RGB 장소 후보가 LiDAR ICP 검증까지 통과
- `ACCEPTED_PROXIMITY`: pose proximity 후보가 LiDAR ICP 검증 통과
- `REJECTED`: visual hypothesis가 최종 검증에서 거부
- `HEARTBEAT`: 30초마다 frame/승인/거부 누계
- `SUMMARY`: 정상 종료 시 최종 누계

승인/거부 이벤트는 매번 `flush`와 `fsync`하여 SSH 단절이나 비정상 종료 시에도 이미 기록된 event가 남을 가능성을 높였다. 모의 `Info` 입력 정적 시험에서 `START`, global 승인, proximity 승인, 거부, `SUMMARY` 순서와 각 누계 1건을 확인했다. 이 시험은 logger 코드만 검증하며 실제 Go2 loop closure 성공 증거는 아니다.

Headless 실행:

```bash
cd /home/unitree/go2_ws_antarctica
./map_headless.sh
```

현재 canonical `map_headless.sh`는 `Reg/Force3DoF=true`, `Icp/Force4DoF=false`, `RGBD/LoopClosureIdentityGuess=true`를 사용한다. `Optimizer/Slam2D` legacy 표기는 제거했다. DB, 전체 console, loop event, config snapshot과 SHA-256은 `/home/unitree/.ros/rtabmap_runs/<run_id>/`에 보존한다.

SSH 단절 가능성이 있으면 foreground mapping을 임의로 `nohup` 처리하기보다 `tmux` 안에서 실행하고, 종료 시 다시 attach하여 `Ctrl+C`로 RTAB-Map과 logger를 정상 종료한다.

실시간 확인 예시:

```bash
latest_log=$(ls -1t /home/unitree/.ros/rtabmap_runs/latest/loop_logs/loop_events_*.log | head -1)
tail -F "$latest_log"
```

## 15. `rtabmap0827_2` 재주행 결과 (2026-08-27 15:16~15:22 KST)

`RGBD/NeighborLinkRefining=false`와 loop logger를 적용한 뒤 GUI mapping으로 약 308.44초 동안 563개 입력 노드를 생성했다. 물리 주행은 사용자가 수행했고, Codex 분석은 종료 후 artifact를 read-only로 확인했다.

### 15.1 아티팩트

| 파일 | SHA-256 |
|---|---|
| `/home/unitree/.ros/rtabmap0827_2.pgm` | `97e589e6140345afc2337029836e24766eaea809c4c490fa8dbd49abaee675d4` |
| `/home/unitree/.ros/rtabmap0827_2.yaml` | `cc9320d7f491c7acd12716713c271279c839c0599e531853bac74a9d5c07b709` |
| `/home/unitree/.ros/rtabmap.db` | `81337b7e66812c65aba12a96880a84312a8659e61a53d5b1fa545ff748050c62` |
| `loop_events_20260827_151630_headless_mapping.jsonl` | `85acb0dc5621f3d48cea7ef5332c115cace48fe6fae231ba0bb4ea529f1e080f` |
| `/home/unitree/.ros/log/rtabmap_44398_1787811394968.log` | `4055df35c64f99d40484cff6c0847be9e86f83608613fbb1e3f5e864bbfaa44c` |

DB `PRAGMA integrity_check`는 `ok`다.

### 15.2 Loop 결과

| 구분 | 결과 |
|---|---:|
| global visual closure, link type 1 | **0** |
| local-space proximity closure, link type 2 | **5** |
| visual 최고 후보가 존재한 노드 | 479/563 |
| visual 최고 hypothesis score | 0.844551 |
| ICP 검증에서 거부된 visual hypothesis | 83 |
| gravity link | 557/563 |
| IMU interpolation warning | 6 |

승인된 proximity links:

```text
287 -> 276
291 -> 276
292 -> 275
297 -> 270
303 -> 265
```

노드 ID 간격이 11~38이므로 full-lap 시작점 closure가 아니라 최근에 지나간 가까운 구간을 scan ICP로 닫은 local closure다. 모두 DB의 type 2 link로 재확인했다.

RGB 카메라 기반 appearance retrieval은 강하게 동작했다. 예를 들어 node 239는 node 139를 score 0.844551로 찾았고 node 411은 node 357을 score 0.672243으로 찾았다. 그러나 `Reg/Strategy=1`의 LiDAR ICP 검증이 실패하여 global closure로 추가되지 않았다. 즉 이번 실행은 카메라가 과거 장소를 못 찾은 것이 아니라, 찾은 global 후보를 L2 기하 검증이 승인하지 못한 결과다.

### 15.3 2D PGM 비교

| 항목 | `rtabmap0827` | `rtabmap0827_2` |
|---|---:|---:|
| 크기 | 544x450 | 532x504 |
| occupied cells | 2,296 | 3,262 |
| occupied 비율 | 0.938% | 1.217% |
| 입력 노드 | 402 | 563 |

새 PGM은 육안상 벽 점이 더 연속적이고 세부 구조가 더 많이 남았다. 다만 노드 수도 약 40% 증가했으므로 occupied cell 증가만으로 절대 정확도를 증명할 수는 없다. 개선의 주된 후보는 잘못된 연속 neighbor ICP를 끈 것이며, 5개의 local proximity closure도 해당 구간 정합에 기여했을 수 있다.

출발점과 종료점이 물리적으로 같은 장소였다는 전제에서 raw LIO의 시작-종료 XY gap은 약 1.471 m, RTAB-Map graph pose의 XY gap은 약 0.895 m로 약 39.2% 감소했다. 이는 당시 4DoF RTAB graph 내부 보정이 XY gap을 줄인 증거지만, ground-truth 위치 측정이 없으므로 절대 위치 정확도 수치로 사용하지 않는다.

### 15.4 3D graph 주의사항

2D 투영 지도는 개선됐지만 3D graph는 아직 합격이 아니다.

- raw Unitree LIO z range: 약 0.0212 m
- RTAB-Map `MapToBase_z` range: 약 6.452 m
- 최종 raw LIO z: 약 0.310 m
- 최종 RTAB-Map map pose z: 약 -6.068 m

수직 변형은 첫 proximity closure 전부터 누적되어 있었으므로 5개 local closure만의 문제는 아니다. gravity-constrained 4DoF graph가 보행 중 roll/pitch를 반영하면서 긴 평면 경로를 z 방향으로 기울인 가능성이 크다. 당시 다음 A/B 후보는 planar graph였고, 이후 `Reg/Force3DoF=true`, `Icp/Force4DoF=false`로 Z 안정성이 확인됐다. 현재는 두 canonical mapping 진입점이 이 설정을 사용하며 global Type-1과 정상 종료 재검증을 기다린다.

### 15.5 Logger 정정

이번 text log는 RTAB-Map 0.21.1 통계 키 끝의 `/`를 인식하지 못해 rejected count와 최고 visual score를 0으로 표시했다. JSONL은 원본 `statistics`를 보존했으므로 위 83건과 최고 0.844551을 복구할 수 있었다. 이후 logger를 다음처럼 수정하고 모의 시험했다.

- trailing `/`가 있는 statistics key 지원
- GUI 실행 파일명은 `gui_mapping`, headless 실행은 `headless_mapping`으로 구분
- SIGINT/SIGTERM 수신 시 `SUMMARY`를 먼저 fsync한 뒤 종료

수정 후 모의 입력에서 node 239 후보 139, score 0.844551의 `REJECTED`와 최종 `SUMMARY rejected=1` 기록을 확인했다. 실제 다음 mapping run에서 다시 검증해야 한다.

## 16. 2026-08-28 GUI 시작 중 RTAB-Map abort와 수정

11:00~11:01 KST에 당시 사용하던 구형 GUI wrapper의 두 실행은 loop closure가 단순히 0건인 주행 결과가 아니다. 두 loop log 모두 `frames=0`, 기존 DB의 mtime도 2026-08-27 그대로였고, launch log에서 `/opt/ros/foxy/lib/rtabmap_slam/rtabmap`이 각각 `exit code -6`으로 종료됐다. GUI와 static TF만 남았으므로 지도나 loop 판정에 사용할 수 없는 시작 실패다.

원인은 planar A/B를 위해 추가한 `LaunchConfiguration` 세 값이 Foxy launch의 YAML coercion을 거쳐 ROS boolean으로 전달된 것이다. 생성된 임시 YAML은 `Reg/Force3DoF: false`, `Icp/Force4DoF: true`, `Optimizer/Slam2D: false`였지만 RTAB-Map core parameter table은 이 값을 문자열로 선언한다. 실제 Apport core의 GDB backtrace도 `rclcpp::Node::declare_parameter<std::string>`에서 SIGABRT가 발생했음을 보였다. `ParameterValue(..., value_type=str)`로 세 값을 감싸 4DoF와 planar profile 모두 실제 `str`인 것을 정적 평가 시험으로 확인했다.

수정된 4DoF string parameter YAML은 동일한 Foxy/CycloneDDS 환경에서 별도 `/tmp` DB를 사용해 SLAM mode, callback setup, RGB/scan-cloud subscription까지 완료했고 12초 probe 제한시간까지 생존했다(`timeout` status 124). 이 probe는 파라미터 초기화만 검증하며 센서 처리나 실제 loop closure 증거는 아니다.

또한 GUI나 launch parent만 살아 있어도 LIVE banner가 표시되던 문제를 막기 위해, bringup은 parameter 초기화 시간을 기다린 뒤 실제 `/rtabmap` 노드가 발견될 때만 startup gate를 통과한다. 실패하면 mapping 시작 전 종료되며 motor/command 경로는 생성하지 않는다. 이 수정은 build/static/startup-probe 검증을 통과했지만 2026-08-28 physical planar 결과는 아직 없다.

## 17. 2026-08-28 planar headless 즉시 종료(`Broken pipe`)와 수정

11:30 KST의 `20260828_113015_planar3dof_headless` 실행은 Phase 2 진입 직후 종료됐다. `runtime.log`는 센서 시작 전 문장에서 끝났고, loop log는 생성되지 않았으며, manifest의 `wrapper_exit_status=141`은 출력 pipeline의 SIGPIPE를 뜻한다. 따라서 이 실행에는 주행, planar graph, loop closure를 평가할 데이터가 없다.

원인은 stale-process 정리에 사용한 `pkill -9 -f rtabmap`이었다. evidence logger의 `tee` 명령행에도 `/home/unitree/.ros/rtabmap_runs/.../runtime.log`가 포함되므로, 이 패턴이 RTAB-Map이 아니라 자기 부모 pipeline의 `tee`를 종료했다. 그 결과 bringup shell은 닫힌 pipe에 출력하다 SIGPIPE로 끝났다.

수정 내용:

- RTAB-Map core와 GUI는 각각 정확한 process name인 `rtabmap`, `rtabmap_viz`만 종료한다.
- launch parent는 `ros2 launch rtabmap_launch go2_rtabmap.launch.py` 명령 형태만 선택한다.
- 실제 `/rtabmap` node startup gate를 통과한 뒤에만 run directory에 `RTABMAP_STARTED` sentinel을 만든다.
- wrapper는 sentinel이 있는 실행에서만 현재 DB를 run directory로 복사하고, manifest에 `rtabmap_started`와 `rtabmap_db_saved`를 기록한다.

실패한 run directory에 들어 있는 `rtabmap.db`는 wrapper의 기존 무조건 복사 동작으로 보존된 이전 DB이며, 이 실행에서 생성된 DB가 아니다. 기존 실패 artifact는 장애 증거와 SHA-256 일관성을 위해 수정하거나 삭제하지 않는다. 수정 이후 최초 실주행 결과가 나오기 전까지 physical planar 3DoF 검증 상태는 여전히 미완료다.

## 18. planar 3DoF global-loop 자격 run (2026-08-28 12:46 KST)

`20260828_124601_planar3dof_headless`는 planar 설정에서 실제 global visual loop가 수락되고 DB에
저장되는 것을 처음 확인한 자격 run이다.

| 항목 | 결과 |
|---|---:|
| 입력 node | 249 |
| 주행 시간 | 130.894 s |
| raw LIO path length | 41.142 m |
| 평균 path speed | 약 0.314 m/s |
| raw start/end XY gap | 0.312 m |
| raw start/end yaw gap | 8.16° |
| global type-1 link | 2 unique, DB 양방향 row 4 |
| proximity type-2 link | 9 unique, DB 양방향 row 18 |
| DB integrity | `ok` |

이 결과는 내장 RGB의 장소 후보와 L2 ICP 검증을 함께 쓰는 global loop 경로가 실제로 작동함을
증명한다. 다만 한 번의 짧은 run이므로 전체 주행구역 golden map의 정확도나 반복성까지 증명한
것은 아니다.

## 19. 빠른 전체 mapping 중 중첩 원인 (2026-08-28 13:38 KST)

`20260828_133817_planar3dof_headless`는 약 한 바퀴 전체 주행 후 2D map에서 평행 복도와 벽이
중복되어 golden map 후보에서 제외됐다. read-only DB/loop-log 분석 결과는 다음과 같다.

| 항목 | 결과 |
|---|---:|
| 입력 node | 1,064 |
| 주행 시간 | 568.435 s |
| raw LIO path length | 395.883 m |
| 평균 path speed | 약 0.697 m/s |
| raw start/end XY gap | 10.961 m |
| raw start/end yaw gap | 1.72° |
| median node displacement | 0.426 m |
| global visual accepted | **0** |
| proximity accepted | 32 unique |
| visual/global rejected | 19 |
| odometry-lost log | 0 |

출발점 node 1은 여러 번 visual 후보로 검색됐지만 최종 graph/ICP 검증에서 모두 거부됐다.

- node 1037: ICP correspondence ratio `0.149470 < 0.15`
- node 1049: graph error ratio `10.063 > 3.0`
- node 1060: graph error ratio `6.394 > 3.0`
- node 1062: graph error ratio `6.097 > 3.0`
- node 1063: ICP correspondence ratio `0.118 < 0.15`
- node 1064: graph error ratio `5.709 > 3.0`

즉 카메라가 시작 장소를 전혀 찾지 못한 것이 아니다. 장거리·고속 보행에서 LIO 누적오차가 약
10.96 m까지 커졌고, RTAB-Map 처리율 약 1.87 Hz에서 node 간격도 넓어져 올바른 후보가 현재
graph와 기하학적으로 일관되지 않았다. 안전 검증이 잘못된 강제 loop를 막은 결과다.

`Icp/CorrespondenceRatio`를 0.15 아래로 낮추거나 `RGBD/OptimizeMaxError`를 3보다 크게 만들어
loop를 억지로 수락시키지 않는다. 잘못된 global loop 한 건은 맵 전체를 더 심하게 변형할 수 있다.
현재 golden-map 시작 protocol은 직선 `0.2~0.3 m/s`, 회전 `15~25°/s`, 모서리에서 전진과 회전을
분리하고, 출발점에 같은 방향으로 돌아와 약 3초 정지하는 것이다. 이 속도는 증거 기반 시작값이며
golden map 동결 후 localization은 `0.2 → 0.35 → 0.5 m/s`의 별도 A/B로 검증한다.

loop closure가 **수락되면** 과거 pose와 현재 pose 사이에 constraint가 추가되고 pose graph가
재최적화된다. 그 결과 누적된 위치·방향 오차가 여러 node에 분산되고 각 pose에 붙은 2D/3D map도
함께 재정렬된다. 후보 검색 또는 `REJECTED` 이벤트만으로는 이 보정이 실행되지 않는다.

## 20. 느린 전체-map 재주행 결과 (2026-08-28 14:12~14:28 KST)

`20260828_141247_planar3dof_headless`는 직전 빠른 run과 같은 planar 설정을 유지하고 속도를
낮춰 수행한 전체-map 후보 run이다. wrapper는 operator `Ctrl+C`를 정상 처리해 status 0으로
종료했고 DB copy, integrity, manifest와 전체 `SHA256SUMS` 검증이 모두 통과했다.

```text
DB: /home/unitree/.ros/rtabmap_runs/20260828_141247_planar3dof_headless/rtabmap.db
size: 523153408 bytes
sha256: c4862d88d98ba4a14e8e725bfd6879d688778cdfa591ecd7c694cc5f343bd953
integrity: ok
```

| 항목 | 결과 |
|---|---:|
| DB node / logger frame | 1,707 / 1,706 |
| 주행 시간 | 916.184 s |
| raw LIO path length | 358.186 m |
| 평균 path speed | 약 0.391 m/s |
| median node displacement | 0.225 m |
| logger accepted global / proximity | 81 / 156 |
| DB unique Type-1 / Type-2 | **36 / 159** |
| logger rejected | 183 |
| odometry lost | 0 |
| cannot-compute-transform | 25 |
| negative-hessian covariance warning | 1,145 |
| optimized Z span | **0.0318 m** |

Logger 수와 DB unique link 수가 다른 것은 logger event가 같은 closure의 반복 승인 상태도 세는 반면,
DB 판정은 최종 저장된 양방향 row를 unordered pair로 deduplicate하기 때문이다. 논문 artifact에는
두 수치를 구분해 기록한다.

DB에 남은 36개 global Type-1 pair를 저장된 optimized pose와 constraint로 직접 대조했다.

- loop 양 끝의 optimized XY 거리: median 0.0501 m, maximum 0.1821 m
- constraint translation: median 0.0569 m, maximum 0.1843 m
- optimized graph translation residual: median 0.0129 m, maximum 0.0331 m
- optimized graph yaw residual: median 0.023°, maximum 0.171°

ID 간격은 351~1,237 node인 재방문을 포함하며, 강제로 멀리 떨어진 장소를 붙인 link는 이 검사에서
보이지 않았다. 원본 DB를 건드리지 않고 `/tmp` copy에서 `rtabmap-export --scan --poses`로
1,488 optimized pose와 687,067-point voxel cloud를 조립해 top-down으로 확인했다. 긴 주 복도는
두 벽이 일관되게 정렬됐고 직전 `global=0` map의 큰 평행 복도 중복은 크게 줄었다. 상부의 여러
분기·반복 구간에는 여전히 산란과 겹침이 있어 절대 정확도 PASS라고 부르지는 않는다.

동일한 stored optimized subset의 node 1→1681에서 raw XY gap 1.653 m가 optimized 1.077 m로
약 34.8% 감소했다. 종단 yaw 차이는 raw 7.10°에서 optimized 9.09°였고 독립 ground truth가 없으므로
이 endpoint 하나만으로 전체 정확도를 평가하지 않는다. 358 m 경로 대비 남은 약 1.08 m 종단 gap은
localization 속도·재시작 시험에서 별도로 확인할 항목이다.

판정은 다음과 같다.

- **global visual loop 기능: PASS**
- **planar/Z 안정성: PASS**
- **DB 저장/무결성: PASS**
- **최종 golden-map 동결: CANDIDATE** — 표준 PGM/YAML export 육안 검사와 frozen-DB localization
  cold-start를 통과한 뒤 동결

현재는 같은 전체 경로를 즉시 다시 촬영할 필요가 없다. 이 DB를 보존한 채 2D export와 localization
Gate로 넘어가고, 그 결과에서 실제 false relocalization이나 허용 불가한 맵 중첩이 확인될 때만
targeted remap을 수행한다.

## 21. 물리적 코너 불일치 제보와 link-type ablation 재판정 (2026-08-28, 재부팅 전후)

20절의 마지막 판정은 **철회한다.** 사용자가 실제 환경에는 두 개의 90도 코너가 있어야 하지만
export된 지도와 trajectory가 접히고 교차한다고 확인했다. 이는 독립적인 물리 ground truth이며,
낮은 optimized graph residual보다 우선한다. 낮은 residual은 optimizer가 이미 삽입된 constraint를
잘 만족했다는 뜻일 뿐, 그 constraint가 실제 같은 장소를 연결했다는 뜻은 아니다.

원본 run DB는 읽기 전용 증거로 보존했다.

```text
path=/home/unitree/.ros/rtabmap_runs/20260828_141247_planar3dof_headless/rtabmap.db
sha256=c4862d88d98ba4a14e8e725bfd6879d688778cdfa591ecd7c694cc5f343bd953
integrity=ok
```

세 개의 `/tmp` 복사본을 만들고 SQLite `Link.type`만 선택적으로 제거한 뒤, 각 복사본에 동일한
`rtabmap-export --scan --poses --voxel 0.10` 재최적화를 적용했다. 원본은 수정하지 않았다.

| 재최적화 입력 | 제거 link | path / end gap | top-down trajectory 판정 |
|---|---|---:|---|
| original all loops | 없음 | 356.075 m / 1.077 m | 상부 구간이 교차하고 두 코너가 접힘 |
| no global | Type-1만 제거 | 357.600 m / 1.240 m | 심한 접힘이 그대로 남음 |
| no proximity | Type-2만 제거 | 356.607 m / 0.986 m | raw LIO와 유사한 직교 형상, 두 90도 코너 복구 |
| no Type-1/Type-2 | 둘 다 제거 | 357.840 m / 1.949 m | raw LIO와 유사한 직교 형상 |
| raw LIO reference | 최적화 전 odom | 357.879 m / 1.653 m | 물리적으로 타당한 직교 코너 형상 |

`no global`이 실패하고 `no proximity`가 형상을 회복했으므로, 이번 왜곡의 주원인은 global
visual Type-1이 아니라 spatial proximity Type-2다. 당시 canonical 설정은 다음처럼 반복 복도에서
서로 다른 구간의 one-to-many ICP를 연쇄 승인하기 쉬운 상태였다.

```text
RGBD/ProximityBySpace=true
RGBD/ProximityAngle=180
RGBD/ProximityMaxGraphDepth=0
RGBD/ProximityPathMaxNeighbors=10
```

Logger에서도 accepted event 다수가 `Loop/Optimization_max_error_ratio=2.95~2.96`으로 rejection
기준 3.0 바로 아래에 몰렸고, 일부 `MapToOdom` 보정은 5~7.49 m에 달했다. 개별 링크를 넣은 뒤
graph가 다시 휘면서 다음 잘못된 근접 후보까지 일관돼 보이는 closure cascade가 발생한 것으로
판단한다. 이는 source/log/ablation을 결합한 인과 추론이며 독립 motion-capture ground truth는 없다.

### 21.1 적용한 최소 수정

canonical planar profile은 다음으로 변경한다.

```text
Reg/Force3DoF=true
Icp/Force4DoF=false
RGBD/NeighborLinkRefining=false
RGBD/LoopClosureIdentityGuess=true
RGBD/ProximityBySpace=false
```

Type-1 visual retrieval + 3D L2 ICP 검증은 유지한다. Type-2를 별도 진단에서 다시 켜더라도 설치된
RTAB-Map 0.21.1의 보수적 기본값인 `Angle=45`, `MaxGraphDepth=50`,
`PathMaxNeighbors=0`에서 시작한다. `Rtabmap/LoopThr=0.11`,
`Icp/CorrespondenceRatio=0.15`, `RGBD/OptimizeMaxError=3.0`은 원인 변수를 섞지 않기 위해 이번
수정에서 바꾸지 않았다. `Optimizer/Robust=false`도 유지한다. 설치된 parameter 문서상 robust
optimizer와 `RGBD/OptimizeMaxError`는 동시에 쓰는 설정이 아니다.

### 21.2 다음 합격 Gate

1. `map_headless.sh --print-config`에서 `proximity_by_space:=false`를 확인한다.
2. 실제 두 90도 코너를 포함하는 1~2분 짧은 loop를 한 번 수행한다.
3. Type-2 accepted event와 DB Type-2 link가 0인지 확인한다.
4. 올바른 Type-1이 하나 이상 생기고, 승인 직후 큰 map jump나 코너 접힘이 없는지 확인한다.
5. 통과할 때만 전체 구역을 한 번 remap한다.
6. 새 전체 map의 직선/평행 벽과 두 90도 코너를 확인한 뒤에만 PGM/YAML export와 cold-start
   localization Gate로 넘어간다.

따라서 `20260828_141247_planar3dof_headless`의 최종 판정은 다음과 같다.

- sensor/LIO/3DoF Z 안정성: PASS
- Type-1 global visual loop 기능: PASS
- DB 저장/무결성: PASS
- 물리적 map geometry: **FAIL**
- golden/localization DB 사용: **금지**

## 22. Jetson-only 전체 재처리, 처리율 감사 및 다른 에이전트 인계 (2026-08-28 18:50 KST)

이 절은 로봇 전원을 끄고 Jetson만 켠 상태에서 수행했다. 하드웨어, motor topic, 네트워크 설정,
sudo를 사용하지 않았다. 원본 DB는 SQLite immutable/read-only 입력으로 보존했고, RTAB-Map 재처리
출력은 `/tmp`에만 만들었다. 따라서 이 절은 저장된 센서 데이터에 대한 소프트웨어 검증이며 새
실시간 센서 상태나 수정 후 물리 map을 대신 증명하지 않는다.

### 22.1 현재 실행 설정과 LIVO 경계

`./map_headless.sh --print-config` 실측 출력:

```text
mapping_mode=true
gui_mode=false
graph_profile=planar3dof
graph_arg=reg_force_3dof:=true
graph_arg=icp_force_4dof:=false
graph_arg=loop_closure_identity_guess:=true
graph_arg=proximity_by_space:=false
recorder=false
```

현재 launch source와 설치본의 SHA-256은 모두
`7acb4a720aeb65b3f2db7d8df17ec2dd24640608694b4d28c74adb18841b85b0`로 같았다. 따라서 Type-2
비활성화가 source에만 있고 설치본에는 빠진 상태가 아니다.

이 프로젝트에서 `LIVO`라고 부르는 경계는 다음과 같다.

- Unitree 내장 L2가 `/utlidar/robot_odom`, `/utlidar/imu`,
  `/utlidar/cloud_deskewed`를 발행한다.
- `go2_livo_sensor_bridge.py`가 공통 clock offset을 적용하고 deskewed odom-frame cloud를
  `base_link`로 역변환해 `/livo/odom`, `/livo/imu`, `/livo/cloud`를 발행한다.
- RTAB-Map은 외부 Unitree LIO odometry를 연속 자세로 사용하고 3D L2 ICP로 loop를 검증한다.
- 전방 단안 RGB는 visual place retrieval용이다. metric visual odometry는 아니다.

마지막 robot-on 무동작 스냅샷은 cloud 65개, odom 629개, cloud/odom stamp 차이
`0.000573 s`, 최대 정지 odom step `0.0000428 m`였다. 세 전체/자격 run의 bridge clock offset은
`241.123~241.218 s`로 일관됐고, IMU auto-order는 모두 `wxyz`를 선택했다. gravity residual은
`0.35~0.73 deg`, 잘못된 `xyzw` 해석은 `114.68~144.96 deg`였다. bridge error, clock jump,
odometry lost/reset은 검출되지 않았다. 이것은 공개되지 않은 Unitree firmware 내부 알고리즘의
정확도를 증명하지는 않지만, 우리 bridge의 입력·시간·quaternion 처리에는 현재 반증이 없다는
직접 실측 증거다.

### 22.2 Type-2 OFF 전체 재처리 결과

원본은 다음 경로와 checksum으로 고정했다.

```text
/home/unitree/.ros/rtabmap_runs/20260828_141247_planar3dof_headless/rtabmap.db
sha256=c4862d88d98ba4a14e8e725bfd6879d688778cdfa591ecd7c694cc5f343bd953
integrity=ok
```

1707개 저장 node를 현재 핵심 설정으로 처음부터 다시 처리했다. 입력 DB를 수정하지 않고 새 출력
DB를 만든 명령의 핵심 파라미터는 다음과 같다.

```bash
QT_QPA_PLATFORM=offscreen rtabmap-reprocess \
  --RGBD/ProximityBySpace false \
  --RGBD/NeighborLinkRefining false \
  --RGBD/LoopClosureIdentityGuess true \
  --RGBD/ProximityAngle 45 \
  --RGBD/ProximityMaxGraphDepth 50 \
  --RGBD/ProximityPathMaxNeighbors 0 \
  --Reg/Force3DoF true \
  --Icp/Force4DoF false \
  INPUT.db /tmp/OUTPUT.db
```

| 항목 | 원본: Type-1+Type-2 | 현재 설정 재처리: Type-1 only |
|---|---:|---:|
| 처리 node | 1707 | 1707 |
| unique Type-1 | 36 | **41** |
| unique Type-2 | 159 | **0** |
| optimized pose | 1488 | 1488 |
| optimized path | 356.168 m | 356.258 m |
| optimized endpoint gap | 1.077 m | **0.711 m** |
| optimized Z span | 0.0318 m | 0.0318 m |
| raw LIO 대비 XY correction p95 | 17.092 m | **4.294 m** |
| raw LIO 대비 XY correction max | 22.043 m | **4.431 m** |
| trajectory 형상 | 상부 교차/접힘 | raw LIO와 같은 직교 코너/왕복 구조 |

Type-2를 꺼도 global visual+ICP loop는 36건에서 41건으로 유지됐다. 반복 복도에서 생성된 많은
visual 후보는 `Icp/CorrespondenceRatio=0.15`, `Icp/MaxTranslation=0.2 m`,
`RGBD/OptimizeMaxError=3`에 의해 거절됐고, 올바른 장거리 Type-1만 최종 DB에 남았다. 이는 21절의
link 삭제 ablation보다 강한 재현 증거다. 단순히 기존 링크를 지운 것이 아니라 저장 sensor data를
현재 설정으로 전체 재실행했기 때문이다.

### 22.3 입력률과 Jetson 처리 여유 실측

bridge 누적 통계와 RTAB-Map runtime line 3020개를 다시 계산했다.

| run | cloud publish | RTAB-Map mean | p95 | max | `>0.5 s` |
|---|---:|---:|---:|---:|---:|
| 짧은 자격 `124601` | 15.03 Hz | 0.141 s | 0.225 s | 0.702 s | 1 |
| 빠른 전체 `133817` | 15.24 Hz | 0.191 s | 0.277 s | 5.332 s | 4 |
| 느린 전체 `141247` | 15.30 Hz | 0.247 s | 0.397 s | 11.076 s | 5 |

느린 전체 run의 graph 성장 구간별 처리시간은 다음처럼 증가했다.

| node 구간 | mean | p95 |
|---|---:|---:|
| 1~500 | 0.162 s | 0.215 s |
| 501~1000 | 0.241 s | 0.319 s |
| 1001~1500 | 0.267 s | 0.352 s |
| 1501~1707 | 0.422 s | 0.451 s |

즉 L2 cloud 자체는 이미 약 15 Hz로 충분히 빠르며 현재 RTAB-Map keyframe rate는 2 Hz다.
3 Hz는 프레임당 `0.333 s` 예산인데 전체 map 후반 p95가 이미 `0.397~0.451 s`다. 현 상태에서
rate만 3 Hz로 올리면 callback backlog와 오래된 synchronized frame 처리 위험이 있다. 또한 빠른
run의 raw Unitree LIO endpoint gap `10.961 m`는 upstream LIO 오차이므로 RTAB-Map detection rate를
높여도 직접 고쳐지지 않는다. 더 많은 visual 후보를 제공할 가능성은 있지만 별도 live A/B 없이는
안정성 개선으로 간주하지 않는다.

저장 scan 1707개를 zlib 해제해 XYZ/I 16-byte point record로 계측했다.

| 단계 | 평균 점 수/scan | raw 대비 |
|---|---:|---:|
| bridge가 넘긴 유효 scan | 1553 | 100% |
| 현재 `Icp/VoxelSize=0.05` 근사 | 974 | 62.7% |
| 후보 `Icp/VoxelSize=0.08` 근사 | 652 | 42.0% |
| 후보 `Icp/VoxelSize=0.10` 근사 | 527 | 33.9% |

현재 RTAB-Map은 이미 ICP 직전에 5 cm voxel filtering을 한다. 따라서 bridge에서 같은 크기의
downsample을 또 넣어도 ICP 기하에는 새 정보가 생기지 않으며, DDS/메모리 절약 이득은 실제 CPU
profile로 따로 증명해야 한다.

### 22.4 8 cm voxel A/B: 기각

설정을 바꾸기 전에 같은 1707-node DB를 `Icp/VoxelSize=0.08`만 다르게 전체 재처리했다.

| 항목 | 5 cm 현재값 | 8 cm 후보 |
|---|---:|---:|
| 재처리 wall time | 약 5분 | 5분 8초 |
| unique Type-1 / Type-2 | 41 / 0 | 36 / 0 |
| endpoint gap | **0.711 m** | **82.783 m** |
| raw 대비 XY correction p95 | 4.294 m | 49.541 m |
| raw 대비 XY correction max | 4.431 m | 83.162 m |
| 후반 graph error ratio | 정상 gate 범위 | 150 이상 반복 |
| 형상 | raw와 일치 | 긴 단일 방향 발산/접힘 |

8 cm는 처리시간을 의미 있게 줄이지 못했고, accept/reject되는 Type-1 집합을 크게 바꿨다. 결과적으로
global graph가 발산했다. 공식 RTAB-Map 구현의 corridor low-complexity 보호가 PointToPoint 전환과
제약 축 투영을 수행하는 로그도 확인됐지만, 652점 수준에서는 그것만으로 잘못된 전역 결과를 막지
못했다. 따라서 **현재 5 cm를 유지하고 8/10 cm와 upstream point drop은 적용하지 않는다.**

### 22.5 공식 문서와 일치하는 해석

- [`Rtabmap/DetectionRate`](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)는
  입력 이미지를 해당 Hz로 필터링하는 값이다. L2 odometry 발행률을 높이는 값이 아니다.
- [`Icp/VoxelSize`](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)는
  ICP용 uniform sampling이고, `Mem/LaserScanVoxelSize`는 signature 생성 전에 scan 자체를
  voxel-filter한다. 후자는 기존 normal을 제거하므로 normal 재계산 설정도 함께 검증해야 한다.
- 같은 공식 소스는 corridor-like low complexity에서 PointToPoint로 바꾸고 관측 가능한 축만
  보정하는 보호 로직을 설명한다. 현재 설치본 `rtabmap-info`에도
  `PointToPlaneMinComplexity=0.02`, `LowComplexityStrategy=1`이 확인됐다.
- [RTAB-Map robust graph 공식 문서](https://github.com/introlab/rtabmap/wiki/Robust-Graph-Optimization)는
  잘못된 loop 하나도 큰 map error를 만들 수 있다고 설명한다.
- [Unitree ROS 2 공식 README](https://github.com/unitreerobotics/unitree_ros2/blob/master/README.md)는
  Go2/Foxy/CycloneDDS와 `/utlidar/cloud`는 설명하지만 firmware의
  `/utlidar/cloud_deskewed`, `/utlidar/robot_odom` 좌표 의미는 공개하지 않는다. 이 두 topic의
  의미는 우리 runtime header/pose 비교와 저장 run으로 검증한 범위를 넘어 추정하지 않는다.

### 22.6 다른 에이전트가 이어받을 때의 고정 사실

다음 항목은 같은 증거를 다시 조사할 필요가 없다.

1. `4DoF -> 3DoF`가 최신 맵 접힘의 원인이 아니다. 3DoF는 Z 발산을 해결했다.
2. 빠른 run은 raw LIO 자체의 10.961 m loop gap이 있었고, 느린 run의 raw LIO는 직교 코너와
   1.653 m gap을 유지했다. 두 run은 실패 층이 다르다.
3. 느린 run의 심한 접힘 원인은 Type-1이 아니라 old Type-2 spatial proximity cascade다.
4. 현재 source와 설치본은 Type-2 OFF이며 `map_headless.sh --print-config`도 이를 확인한다.
5. Type-2 OFF 전체 재처리는 Type-1 41건과 정상 형상을 보존했다.
6. `Icp/VoxelSize=0.08`은 오프라인 A/B에서 실패했다. 5 cm를 바꾸지 않는다.
7. 기존 `20260828_141247` DB는 원인 분석용 증거이며 golden/localization map으로 사용하지 않는다.
8. 이 절의 결과는 수정 후 실로봇 map의 물리 합격을 대신하지 않는다.

### 22.7 다음 조사 순서와 합격 기준

설정을 한꺼번에 바꾸지 않는다. 권장 순서는 다음과 같다.

1. **현재값 고정 실로봇 자격시험**: 5 cm, 2 Hz, Type-2 OFF로 실제 90도 코너 두 개와 출발점
   재방문을 포함해 1~2분 주행한다. 먼저 이 baseline을 통과시킨다.
2. **자격시험 합격 조건**: DB Type-2 0, 올바른 Type-1 1개 이상, odometry lost 0, 승인 직후 큰
   `MapToOdom` jump 없음, 두 코너와 평행 벽 유지, raw보다 optimized loop gap이 악화되지 않음.
3. **전체 golden map**: baseline 자격시험 통과 뒤 `0.2~0.3 m/s`, 회전 `15~25 deg/s`, 코너에서
   전진/회전 분리, 시작점 같은 heading으로 3~5초 정지한다.
4. **처리율 연구가 꼭 필요할 때만**: 전체 map이 먼저 안정된 뒤 짧은 동일 경로에서 2.0 Hz와
   2.5 Hz를 한 변수 A/B한다. 3 Hz부터 시작하지 않는다. node 간격, RTAB-Map p95/max,
   sync drop, Type-1 accept/reject, raw/optimized gap과 코너 형상을 모두 비교한다.
5. **Jetson 장기 처리시간 후보**: point drop보다 `Rtabmap/TimeThr` 또는 `MemoryThr`로 WM 크기를
   제한하는 A/B가 현재 병목 증거에는 더 직접적이다. 다만 LTM retrieval과 global loop 수를 바꿀
   수 있으므로 `/tmp` 전체 재처리에서 Type-1 수, endpoint gap, 최대 graph correction이 5 cm
   baseline보다 나빠지지 않을 때만 live 후보로 올린다.
6. **절대 정확도**: 최종 논문 수치는 RTAB-Map endpoint gap만 쓰지 말고 AprilTag/측정 기준점,
   PixNav waypoint 오차 또는 독립 reference trajectory와 비교한다.

다음 agent는 설정 변경보다 먼저 위 Gate를 자동 산출하는 read-only DB report를 재사용 가능하게
만드는 것이 좋다. 최소 출력은 DB checksum/integrity, raw path/gap/Z, optimized path/gap/Z,
link type별 unique count, Type-1 pair, raw 대비 optimized correction p95/max, RTAB-Map 처리시간
p95/max다. 원본 DB에서 `rtabmap-export`를 직접 실행하면 Admin cache가 갱신될 수 있으므로 반드시
복사본 또는 `/tmp` 재처리 출력에만 실행한다.
