# [2026-08-27] RTAB-Map LIVO 문제·원인·해결·재검증 총정리

> 대상: Go2 EDU Plus + 내장 Unitree 4D LiDAR L2 + Jetson Orin NX + ROS 2 Foxy  
> 목적: 이번 작업에서 잘못 이해했거나 실제로 고장 났던 항목을 “문제 → 근거 → 해결 → 현재 상태” 순서로 보존한다.  
> 안전: 물리 주행은 사용자가 수동으로 수행했으며 Codex는 이동 명령을 발행하지 않았다.

## 1. 최종 결론

현재 맵핑 구성의 정확한 명칭은 다음과 같다.

> **Unitree L2 LiDAR+IMU odometry(LIO) + 3D LiDAR ICP mapping + monocular RGB visual place recognition**

전면 단안 카메라는 현재 metric visual odometry를 만들지 않는다. 카메라는 과거 장소 후보를 찾고, 후보 승인 기하는 L2 LiDAR ICP가 담당한다.

수정 후 두 번째 2D 지도는 첫 지도보다 좋아졌고 5개의 local-space proximity closure가 실제 승인됐다. 그러나 global visual loop closure는 0개이고, 3D pose graph는 z 방향으로 약 6.452 m 발산했다. 따라서 현재 판정은 다음과 같다.

- 2D 실내 map 제작: **사용 가능성 높음, 추가 planar A/B 필요**
- 3D metric map: **불합격**
- visual loop closure: **후보 검색은 동작, 전역 승인 미검증**
- mapping 중 recorder/자율제어: **의도적으로 비활성**

## 2. 문제 → 원인 → 해결 → 결과

| ID | 잘못되었던 내용/증상 | 직접 원인 또는 근거 | 적용한 해결 | 현재 결과 |
|---|---|---|---|---|
| R-01 | L2 Ethernet SDK와 Go2 내장 L2 경로를 같은 것으로 취급 | Go2는 이미 `/utlidar/*`를 DDS로 발행하며 외부 SDK path는 별도 장치 직접 연결용 | 주 mapping은 built-in `/utlidar/*`, 외부 SDK는 필요 시 `/external_l2/*`로 분리 | 해결 |
| R-02 | deskewed cloud가 `odom` 좌표인데 frame 이름만 local frame으로 변경 | 이동한 world-frame 점군을 local scan으로 오해해 ICP 중첩 붕괴 가능 | 동시각 `/utlidar/robot_odom` 역변환으로 `base_link` 점군 생성 | 해결 |
| R-03 | cloud마다 약 10,000개의 zero padding point 혼입 | Unitree built-in cloud record에 빈 점 포함 | finite 및 non-zero point만 `/livo/cloud`로 재발행 | 해결 |
| R-04 | LiDAR와 Jetson camera timestamp 기준 불일치 | 내장 L2 clock과 host ROS clock offset | odom/IMU/cloud 모두에 하나의 공통 LiDAR→host offset 적용 | 해결 |
| R-05 | IMU quaternion 순서가 잘못 해석될 가능성 | 현재 firmware DDS 배열과 ROS field mapping이 일반 xyzw 가정과 다름 | gravity residual로 `xyzw/wxyz` 자동 판정 및 정규화 | 해결; 장치에서 wxyz 선택 확인 |
| R-06 | 단안 RGB를 visual odometry라고 표현 | depth/stereo 없이 연속 metric camera pose를 제공하지 않음 | RGB 역할을 place recognition으로, metric pose를 LIO로 분리 | 문서 해결 |
| R-07 | `Reg/Strategy=1`이면 카메라가 안 쓰인다고 오해 | registration은 ICP지만 RGB visual word/hypothesis는 별도 동작 | RGB 후보 + LiDAR ICP validation 구조로 설명/로그 | 해결 |
| R-08 | 기존 LIO neighbor pose를 매 keyframe ICP로 다시 보정 | `RGBD/NeighborLinkRefining=true`; 희소 L2/긴 복도 퇴화로 오차 누적 | `RGBD/NeighborLinkRefining=false` | 2차 지도에서 2D 연속성 개선 |
| R-09 | “한 바퀴 돌면 자동 global loop”라고 기대 | 같은 위치뿐 아니라 비슷한 camera view와 충분한 LiDAR overlap이 필요 | 출발/도착에서 같은 방향·2~3초 관측, loop logger 추가 | type-2 5개; type-1은 아직 0개 |
| R-10 | 첫 map이 휘고 endpoint가 더 벌어짐 | neighbor ICP가 raw LIO를 과교정 | R-08 적용 | 첫 map 대비 개선 |
| R-11 | 두 번째 2D map이 좋아져 3D도 정확하다고 볼 위험 | 2D 투영은 좋아도 RTAB graph z range 6.452 m | 3D/2D 판정을 분리, planar 3DoF A/B 제안 | 미해결 |
| R-12 | 방사형 free-space 줄과 벽 끊김 | sparse return + `Grid/RayTracing=true` + 강한 neighbor filter | pose graph를 먼저 고정한 뒤 grid만 별도 A/B | 보류 |
| R-13 | IMU interpolation warning | synchronized callback 시각에 future IMU sample 부족 | cloud 20~30 ms buffer 후 publish 후보 | 1차 7회, 2차 6회; 미구현 |
| R-14 | camera calibration을 실제 값처럼 사용 | 현재 intrinsics/extrinsics는 프로젝트 추정값, distortion=0 가정 | checkerboard/AprilTag로 intrinsics·distortion·extrinsics 실측 | 미해결 |
| R-15 | 새 mapping이 기존 DB를 삭제할 위험 | mapping node가 `-d` 사용 | wrapper가 기존 DB를 timestamp backup한 뒤 시작 | 해결; wrapper 우회 금지 |
| R-16 | mapping에도 rosbag/도커/모터 bridge를 같이 실행 | 불필요한 저장·안전 범위 증가 | mapping mode는 recorder, Docker/VLM, `host_bridge` 미실행 | 해결 |
| R-17 | GUI가 없으면 loop 결과를 확인하기 어려움 | `/info` 통계가 화면에만 의존 | read-only headless loop logger와 JSONL/text/fsync 추가 | 해결 |
| R-18 | 첫 logger text가 rejected=0으로 표시 | RTAB-Map 0.21.1 statistics key 끝의 `/` 미처리 | key normalization, signal SUMMARY, GUI/headless label 수정 | 모의시험 PASS; 다음 실주행 확인 |
| R-19 | “RTAB-Map 50 Hz”라고 기록 | LIO pose stream과 RTAB graph update 주기를 혼동 | LIO pose와 `Rtabmap/DetectionRate=2 Hz`를 구분 | 문서 해결 |
| R-20 | `localization:=true`가 사전 map 없는 pure odometry라고 기록 | 실제 `IncrementalMemory=false`, `InitWMWithAllNodes=true`로 DB localization | mapping/localization 의미를 바로잡고 논문 역할 명시 | 문서 해결 |
| R-21 | `rtabmap-export`를 read-only라고 가정 | export가 DB의 Admin cache를 갱신 | SQLite URI `mode=ro`를 우선 사용하고 원본 hash 보존 | 해결 |

## 3. 실제 수정된 파일과 역할

| 파일 | 적용 내용 |
|---|---|
| `scratch/go2_livo_sensor_bridge.py` | clock 정렬, IMU quaternion 판정, odom→base cloud 역변환, zero point 제거, `/livo/*` 발행 |
| `src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py` | 외부 LIO 입력, LiDAR ICP + RGB place recognition, neighbor refinement 비활성, 3D grid, stats 발행 |
| `scratch/rtabmap_loop_logger.py` | global/proximity/rejected/summary event 기록, ROS publisher 없음 |
| `scratch/bringup_all_escape_nav.sh` | mapping에서 command bridge/Docker/recorder 제외, DB backup, loop logger 시작 |
| `mapping_gui.sh` | GUI mapping wrapper |
| `mapping_headless.sh` | 무디스플레이 mapping wrapper |
| `mapping_planar_headless.sh` | 3D LiDAR ICP는 유지하고 graph만 x/y/yaw로 제한; run별 DB·console·loop·config/hash 보존 |

## 4. 두 번의 실주행 비교

| 항목 | `rtabmap0827` | `rtabmap0827_2` |
|---|---:|---:|
| RTAB input nodes | 402 | 563 |
| `NeighborLinkRefining` | true | false |
| global visual closure, type 1 | 0 | 0 |
| proximity closure, type 2 | 0 | 5 |
| rejected visual hypothesis | 13 | 83 |
| 최고 visual score | 약 0.1524 | 0.844551 |
| 2D 관찰 | 벽/통로 휨 | 벽 연속성과 세부 구조 개선 |

두 번째 주행의 승인 type-2 links:

```text
287 → 276
291 → 276
292 → 275
297 → 270
303 → 265
```

이는 full-lap global closure가 아니라 최근 인접 공간 재관측을 닫은 local closure다. “카메라 루프가 성공했다”고 해석하면 안 된다.

두 번째 주행에서 출발점과 종료점이 실제 같은 위치였다는 전제하에 raw LIO endpoint gap 약 1.471 m가 RTAB graph 약 0.895 m로 줄었다. 다만 독립 ground truth가 없으므로 절대 정확도 수치가 아니라 내부 개선 지표로만 사용한다.

## 5. 아직 남은 가장 큰 문제: z 발산

두 번째 주행:

- raw LIO z range: 약 0.0212 m
- RTAB `map→base` z range: 약 6.452 m
- 최종 RTAB z: 약 -6.068 m

`Icp/Force4DoF=true`는 x/y/z/yaw correction을 허용하며 gravity는 roll/pitch 정렬을 도와도 z translation을 고정하지 않는다. 평탄한 단층 복도에서는 보행 roll/pitch와 퇴화한 벽 기하가 z 방향 graph error로 누적될 수 있다.

다음 A/B에서 한 번에 바꿀 후보는 다음 세 항목뿐이다.

```python
'Reg/Force3DoF': 'true'
'Icp/Force4DoF': 'false'
'Optimizer/Slam2D': 'true'
```

launch 기본값은 기존 4DoF 결과 보존을 위해 그대로다. 별도 `mapping_planar_headless.sh`가 이 세 launch argument만 함께 덮어쓰며, 아직 물리 주행 결과는 없다.

합격 기준:

- RTAB z range가 raw LIO와 같은 수 cm 수준에 근접
- 긴 벽의 직선성과 평행성이 두 번째 지도 이상
- raw LIO보다 endpoint gap 악화 없음
- false loop로 map이 접히지 않음
- 동일 시작 pose에서 localization 반복 분산이 사전 기준 이내

## 6. 2D SLAM처럼 쓰는 것이 맞는가

논문과 실내 navigation 목적이라면 다음 조합이 유리하다.

```text
3D L2 cloud + LIO 입력
        ↓
planar SE(2) pose graph
        ↓
2D occupancy map / planar controller
```

이는 2D LiDAR만 사용하는 전통적 2D SLAM과 다르다. 센서 입력과 ICP overlap은 3D 점군을 유지하되, 실제 로봇이 단층 바닥에서 이동한다는 prior로 graph를 x/y/yaw에 제한한다. 현재 3D z 결과보다 ICRA 실내 주행의 fixed-map localization에 더 적합할 가능성이 높다.

경사로·계단·다층 z를 논문 기여로 평가해야 할 때만 4DoF/6DoF를 별도 자격 검증한다.

## 7. 다음 mapping 실행에서 확인할 것

사용자가 GUI를 볼 수 있을 때:

```bash
cd /home/unitree/go2_ws_antarctica
./mapping_gui.sh
```

디스플레이 없이 수동 주행할 때:

```bash
cd /home/unitree/go2_ws_antarctica
./mapping_planar_headless.sh
```

기존 `mapping_headless.sh`는 4DoF 비교 기준으로 남겨 두었다. planar wrapper의 결과는 `/home/unitree/.ros/rtabmap_runs/<run_id>/`에 self-contained evidence로 저장된다.
부팅 후 Go2 DDS multicast route가 없으면 시작 시 sudo 인증을 한 번 요청한다. credential은 source나 evidence log에 저장하지 않으며, route 추가 실패 시 mapping stack은 시작되지 않는다.

두 경로 모두 현재 mapping mode에서 다음을 실행하지 않는다.

- rosbag recorder
- Docker/VLM
- `host_bridge.py`
- 자율주행 command path

종료 후 확인 순서:

1. loop JSONL의 `ACCEPTED_GLOBAL`, `ACCEPTED_PROXIMITY`, `REJECTED`, `SUMMARY`
2. DB의 link type 1/2 수
3. raw LIO와 RTAB pose의 endpoint gap 및 z range
4. PGM 벽 직선성/이중벽/방사형 ray
5. IMU interpolation warning 수
6. PGM/YAML/DB/log SHA-256

## 8. loop closure 재검증 주행 조건

1. 시작 위치에서 동일 방향으로 2~3초 정지
2. 한 바퀴 수동 주행
3. 마지막 2~3 m를 출발 때와 같은 방향/시야로 접근
4. 출발 위치에서 같은 방향으로 다시 2~3초 정지
5. global link type 1이 생겼는지 확인
6. 승인 직후 map jump, 접힘, 이중벽, z 발산이 없는지 확인

visual score가 높아도 LiDAR geometry가 부족하면 거절되는 것이 정상이다. `Rtabmap/LoopThr`를 먼저 낮추지 말고 camera 시야, L2 overlap, timestamp, correspondence distance 순으로 확인한다.

## 9. 아카이브와 상세 로그

- 오늘 지도·YAML·loop log·hash: [`2dmap/2026-08-27/MANIFEST.md`](../../2dmap/2026-08-27/MANIFEST.md)
- 전체 DB/통계/trajectory 분석: [`troubleshooting/06_rtabmap_livo_2026-08-27_runtime_diagnosis_and_loop_closure_log.md`](../troubleshooting/06_rtabmap_livo_2026-08-27_runtime_diagnosis_and_loop_closure_log.md)
- 4-Tier/ICRA 실험 계획: [`[2026-08-27]_Robot_Jetson_Docker_Server_4Tier_실측감사_및_ICRA2027_실로봇_실험프로토콜.md`](./[2026-08-27]_Robot_Jetson_Docker_Server_4Tier_실측감사_및_ICRA2027_실로봇_실험프로토콜.md)

## 10. 과거 문서를 읽을 때의 우선순위

과거 master plan의 “완벽/100%/최종 승인/50 Hz RTAB-Map/20회면 완료” 표현은 당시 계획 또는 추정이다. 현재 판단 순서는 다음으로 고정한다.

1. 오늘 보존한 DB/PGM/JSONL과 실제 runtime
2. 현재 launch/bridge/logger source
3. 이 문제 해결 총정리와 상세 troubleshooting 문서
4. 최신 원격 `paper` 브랜치의 protocol/checklist
5. 과거 master plan

실제 재검증 없이 “해결 완료”로 올리지 않는다.
