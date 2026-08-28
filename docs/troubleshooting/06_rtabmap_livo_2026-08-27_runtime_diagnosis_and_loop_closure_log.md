# RTAB-Map LIVO `rtabmap0827` 실측 진단 및 Loop Closure 로그

> 기록 시각: 2026-08-27T15:00:22+09:00  
> 대상 Git commit: `c977dee555ad396aab1483eecccd6631737abe8c`  
> 대상 플랫폼: Unitree Go2 EDU Plus, Jetson Orin NX, Ubuntu 20.04, ROS 2 Foxy  
> 대상 센서: Go2 내장 Unitree 4D LiDAR L2, 내장 LiDAR IMU, 전방 단안 RGB 카메라  
> 안전 범위: 저장 파일과 로그의 read-only 진단. 로봇 구동 명령은 발행하지 않음.  
> 문서 상태: 이번 실행의 직접 측정값은 `실측`, 아직 재주행하지 않은 변경은 `권장/미검증`으로 표시함.

> **최종 상태 안내 (15:22 KST 재주행 반영)**: 1~13절은 첫 번째 `rtabmap0827` 주행의 원인 분석이고, 14~15절은 수정 후 `rtabmap0827_2` 재검증이다. 현재 최종 판정은 `NeighborLinkRefining=false` 적용으로 2D 형상은 개선되었고 type-2 근접 폐쇄 5개가 승인됐지만, type-1 전역 시각 폐쇄는 0개이며 3D z 발산은 미해결이라는 것이다. 한눈에 보는 최신 문제→해결 표는 [`master_plan/[2026-08-27]_RTAB-Map_LIVO_문제_원인_해결_및_재검증_총정리.md`](../master_plan/[2026-08-27]_RTAB-Map_LIVO_문제_원인_해결_및_재검증_총정리.md)를 우선 참조한다.

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
- `mapping_gui.sh`
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

현재 mapping launch는 mapping mode에서 `-d`를 사용한다. 새 map을 시작할 때는 반드시 `mapping_gui.sh` 또는 backup이 보장된 wrapper를 사용하고 launch 파일을 직접 실행하지 않는다.

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
- `mapping_headless.sh`가 사용하는 mapping mode에 logger 자동 통합
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
./mapping_planar_headless.sh
```

이 planar wrapper는 기존 `mapping_headless.sh`의 4DoF 기본값을 보존하면서 `Reg/Force3DoF=true`, `Icp/Force4DoF=false`, `Optimizer/Slam2D=true`를 함께 전달한다. DB, 전체 console, loop event, config snapshot과 SHA-256은 `/home/unitree/.ros/rtabmap_runs/<run_id>/`에 보존한다.

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

수직 변형은 첫 proximity closure 전부터 누적되어 있었으므로 5개 local closure만의 문제는 아니다. gravity-constrained 4DoF graph가 보행 중 roll/pitch를 반영하면서 긴 평면 경로를 z 방향으로 기울인 가능성이 크다. 단일층 2D navigation map이 목적이면 다음 A/B는 `Reg/Force3DoF=true`, `Icp/Force4DoF=false`, `Optimizer/Slam2D=true`가 우선 후보다. 이 3DoF 변경은 당시 결과 기록 시점에는 적용되지 않았고, 현재는 별도 planar headless profile로 준비되어 실주행 검증을 기다린다.

### 15.5 Logger 정정

이번 text log는 RTAB-Map 0.21.1 통계 키 끝의 `/`를 인식하지 못해 rejected count와 최고 visual score를 0으로 표시했다. JSONL은 원본 `statistics`를 보존했으므로 위 83건과 최고 0.844551을 복구할 수 있었다. 이후 logger를 다음처럼 수정하고 모의 시험했다.

- trailing `/`가 있는 statistics key 지원
- GUI 실행 파일명은 `gui_mapping`, headless 실행은 `headless_mapping`으로 구분
- SIGINT/SIGTERM 수신 시 `SUMMARY`를 먼저 fsync한 뒤 종료

수정 후 모의 입력에서 node 239 후보 139, score 0.844551의 `REJECTED`와 최종 `SUMMARY rejected=1` 기록을 확인했다. 실제 다음 mapping run에서 다시 검증해야 한다.

## 16. 2026-08-28 GUI 시작 중 RTAB-Map abort와 수정

11:00~11:01 KST의 `mapping_gui.sh` 두 실행은 loop closure가 단순히 0건인 주행 결과가 아니다. 두 loop log 모두 `frames=0`, 기존 DB의 mtime도 2026-08-27 그대로였고, launch log에서 `/opt/ros/foxy/lib/rtabmap_slam/rtabmap`이 각각 `exit code -6`으로 종료됐다. GUI와 static TF만 남았으므로 지도나 loop 판정에 사용할 수 없는 시작 실패다.

원인은 planar A/B를 위해 추가한 `LaunchConfiguration` 세 값이 Foxy launch의 YAML coercion을 거쳐 ROS boolean으로 전달된 것이다. 생성된 임시 YAML은 `Reg/Force3DoF: false`, `Icp/Force4DoF: true`, `Optimizer/Slam2D: false`였지만 RTAB-Map core parameter table은 이 값을 문자열로 선언한다. 실제 Apport core의 GDB backtrace도 `rclcpp::Node::declare_parameter<std::string>`에서 SIGABRT가 발생했음을 보였다. `ParameterValue(..., value_type=str)`로 세 값을 감싸 4DoF와 planar profile 모두 실제 `str`인 것을 정적 평가 시험으로 확인했다.

수정된 4DoF string parameter YAML은 동일한 Foxy/CycloneDDS 환경에서 별도 `/tmp` DB를 사용해 SLAM mode, callback setup, RGB/scan-cloud subscription까지 완료했고 12초 probe 제한시간까지 생존했다(`timeout` status 124). 이 probe는 파라미터 초기화만 검증하며 센서 처리나 실제 loop closure 증거는 아니다.

또한 GUI나 launch parent만 살아 있어도 LIVE banner가 표시되던 문제를 막기 위해, bringup은 parameter 초기화 시간을 기다린 뒤 실제 `/rtabmap` 노드가 발견될 때만 startup gate를 통과한다. 실패하면 mapping 시작 전 종료되며 motor/command 경로는 생성하지 않는다. 이 수정은 build/static/startup-probe 검증을 통과했지만 2026-08-28 physical planar 결과는 아직 없다.
