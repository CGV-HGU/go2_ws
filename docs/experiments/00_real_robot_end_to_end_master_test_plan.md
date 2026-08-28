# Go2 실로봇 전체 End-to-End 검증 및 논문 실험 마스터 계획

> 개정: 2026-08-28 KST
> 대상: Unitree Go2 EDU Plus + 내장 4D LiDAR L2 + Jetson Orin NX + Docker Jazzy + 원격 VLM server
> 범위: 센서, planar 3DoF RTAB-Map, localization, frozen PixNav, 4-Tier 통신, 안전 제어, 저속 pilot, 논문 campaign
> 현재 종합 판정: **planar/global-loop 기능 PASS, 최신 전체-map geometry FAIL(Type-2 proximity 원인), remap/localization 미검증 / physical autonomy NO-GO**

## 1. 전체 시험의 목적

이 문서는 RTAB-Map만 시험하는 계획이 아니다. 다음 실제 경로가 처음부터 끝까지 재현 가능하고 안전하게 동작하는지 검증한다.

```text
Tier 1 Go2 4D L2 + IMU + LIO + RGB
  → Tier 2 Jetson Foxy sensor bridge + planar RTAB-Map/localization
  → Tier 3 Docker Jazzy VLM agent + frozen PixNav + controller
  → Tier 4 remote Qwen VLM server
  → Tier 3 trajectory/controller
  → Tier 2 single safety gateway
  → Tier 1 Go2 locomotion
```

RTAB-Map은 전체 시스템의 첫 번째 기반 자격시험이다. 지도만 잘 만들어졌다고 ESCAPE-Nav 실로봇 시스템이 완성된 것은 아니며, frozen PixNav와 4-Tier command path가 검증되지 않으면 로봇을 자율주행시키지 않는다. S2E는 현재 paper branch의 별도 NavBench-GS 보조 실험이며 이 실로봇 backend Gate에 포함하지 않는다.

## 2. 3DoF/4DoF 운용 결정

단층 평면 복도와 PointNav 실험에는 **planar 3DoF를 고정 baseline으로 사용한다.** 3DoF는 pose graph를 `x/y/yaw`로 제한할 뿐, 4D L2의 3D cloud, LIO, IMU, 3D ICP와 3D occupancy를 끄지 않는다.

| 조건 | 선택 |
|---|---|
| 단층 평면 복도와 현재 논문 실로봇 campaign | **3DoF** |
| 보행에 따른 수 cm 상하 진동 | **3DoF** |
| 실제 경사로·고도 변화 자체가 평가 대상 | 별도 4DoF A/B |
| 평면 campaign 도중 3DoF/4DoF 혼용 | 금지 |

근거는 4DoF graph Z span 6.452 m와 3DoF run의 raw Z span 0.0335 m, sampled map Z span 0.0175 m 비교다.

## 3. 현재 시스템 준비도

| 계층 | 확인된 상태 | 남은 핵심 항목 | 판정 |
|---|---|---|---|
| Go2 센서 | 내장 L2 cloud/IMU/LIO odom, 전면 RGB 수신 | 다음 run rate/timestamp 반복 확인 | 센서 PASS |
| Jetson LIVO/RTAB-Map | 3DoF Z 안정, Type-1 기능, DB/hash 저장은 확인 | Type-2 OFF 두-코너 재자격, 전체 remap, frozen-DB localization | **MAP GEOMETRY FAIL** |
| frozen PixNav | 논문 pin·Checkpoint_A hash 일치, 정확한 VLM capture-view를 사용한 v2 CUDA 1-step replay, bounded proposal/hash-chain sink, pure fault 22/22 | post-capture history가 있는 20개 실제 clip, live localization/obstacle/controller 연결 | **FILE-ONLY PASS / LIVE 대기** |
| Docker nodes | container와 Jazzy package/executable은 존재; 현재 container는 `tail -f /dev/null`만 실행 | `e2e_node`, `controller_node`를 frozen PixNav 실구현과 file sink에 연결 | **FAIL** |
| Remote VLM | model 조회, text, 보관 RGB 1장 API 응답 | live frame, strict navigation schema, timing/provenance | PARTIAL |
| Host↔Docker bridge | pose/cmd packet 골격과 CRC 존재 | sequence/timestamp/TTL 없음, legacy raw packet 허용 | **FAIL** |
| Go2 command authority | 속도 clamp와 0.5 s watchdog 코드 존재 | `/cmd_vel`+Sport API 이중 발행, shutdown zero ACK 미검증 | **FAIL** |
| Recorder/evaluator | mapping recorder OFF | paper recorder topic 불일치, evaluator가 sample data 사용 | **FAIL** |
| 전체 자율주행 | 아직 end-to-end 실증 없음 | 아래 Gate 0~9 전부 필요 | **NO-GO** |

## 4. 전체 실로봇 Gate 계획표

이 표가 전체 시험의 authoritative 실행 순서다. 이전 Gate가 PASS가 아니면 다음 Gate로 넘어가지 않는다.

| Gate | 범위 | 핵심 시험 | 합격 기준 | 현재 |
|---:|---|---|---|---|
| 0 | 버전·안전 동결 | git/Docker/model/config hash, E-stop, 단일 operator, 통제 구역 | manifest 완성, motion path 기본 OFF, abort 절차 리허설 | file-only manifest PASS / physical 부분 |
| 1 | 실센서 preflight | L2 cloud, IMU, LIO odom, RGB, TF, clock, Jetson thermal/network | 정해진 rate 범위, timestamp 역행 0, TF 단절 0, 실제 frame 확인 | 부분 PASS |
| 2 | planar 3DoF 지도 | Type-2 OFF 두-코너 자격 후 전체 remap | Z span ≤0.05 m, Type-2=0, 올바른 Type-1, 두 90도 코너·직선·평행성 보존, DB 정상 저장 | **재매핑 필요** |
| 3 | map localization | frozen DB cold start와 같은 pose 반복, 이동 후 재지역화 | 10회 시작 성공, false relocalization 0, pose jump 기준 이내 | 대기 |
| 4 | frozen PixNav | 실제 저장 RGB history + capture-view pixel goal + Checkpoint_A 추론 | 구현 pin/model hash 고정, finite action logits, file sink만 사용 | **v2 1-step PASS / 20 clips 대기** |
| 5 | live 4-Tier 무구동 | live camera→VLM grounding→PixNav→controller audit sink | 모든 event identity 연결, stale decision 차단, 모터 topic 발행 0 | offline chain PASS / live 대기 |
| 6 | fault injection | server/VPN/Docker loss, timeout, malformed/out-of-order, stale pose/image | 모든 경우 0 command/hold, watchdog stop ≤0.5 s, stale 적용 0 | pure/file-copy 22/22 / live 대기 |
| 7 | actuator safety | 단일 gateway, E-stop, shutdown zero, 짧은 직진/회전 | 이중 authority 0, 10회 연속 safe-stop, command age/속도 clamp PASS | 대기 |
| 8 | 저속 pilot | 5 m 직선→L-corner→T-junction 순으로 단계 확대 | collision/intervention 0, localization loss 0, 모든 provenance 완전 | 대기 |
| 9 | 논문 campaign | frozen pairs, balanced order, Direct-goal vs Full | 총 50 main runs와 별도 deployment runs, 누락 artifact 0 | 대기 |

## 5. Gate별 상세 검증

### Gate 0 — configuration freeze와 안전 계약

실험 시작 전에 다음 값을 run manifest에 고정한다.

- host git HEAD와 dirty diff hash
- Docker image digest와 package build hash
- PixNav Checkpoint_A path와 SHA-256
- VLM endpoint/model ID, prompt/schema hash
- RTAB-Map DB, PGM/YAML, launch/config hash
- camera/L2 intrinsics/extrinsics version
- speed cap, command TTL, watchdog, geofence
- operator, spotter, E-stop 담당, 시작 battery band

현재처럼 checkpoint가 비어 있거나 mock backend가 default면 Gate 0부터 FAIL이다.

### Gate 1 — 실센서와 Jetson

로봇을 움직이지 않고 다음을 확인한다.

- `/utlidar/cloud_deskewed`, `/utlidar/imu`, `/utlidar/robot_odom`
- `/livo/cloud`, `/livo/imu`, `/livo/odom`
- `/camera/front/image_raw`, `/camera/front/camera_info`
- `map → odom → base_link` 및 sensor static TF
- message rate, timestamp age/reversal, dropped frame, CPU/RAM/temperature
- `eth0` Go2 path와 NetBird server path

synthetic/mock camera나 고정 흰 영상은 실센서 PASS로 인정하지 않는다.

### Gate 2 — planar 3DoF 골든 맵

세부 계획은 [`../07_real_robot_sensor_and_autonomy_verification_plan.md`](../07_real_robot_sensor_and_autonomy_verification_plan.md)를 따른다.

1. `20260828_124601_planar3dof_headless`에서 optimized graph 164 pose, DB integrity `ok`, 올바른 Type-1 2개를 확인했다.
2. 전체 run `20260828_141247_planar3dof_headless`는 1,707 nodes, Type-1 36개, Z span 0.0318 m, DB/hash 저장에는 성공했지만 실제 두 90도 코너가 접혀 geometry FAIL이다.
3. link-type ablation에서 Type-1만 제거하면 접힘이 남고 Type-2만 제거하면 직교 코너가 복구됐다. 낮은 graph residual은 물리 정확도 증거로 사용하지 않는다.
4. canonical profile에서 `RGBD/ProximityBySpace=false`를 확인하고 두 코너를 포함한 1~2분 짧은 loop를 수행한다. Type-2=0, 올바른 Type-1≥1, 코너 접힘=0이어야 한다.
5. 짧은 Gate가 통과할 때만 전체 map을 한 번 remap하고, 직선/평행 벽과 두 90도 코너를 확인한다.
6. 그 뒤 표준 PGM/YAML export와 frozen-DB localization을 통과한 DB/PGM/YAML/3D map 및 설정 hash만 동결한다.

mapping 중 recorder, Docker/VLM, host command bridge와 motor path는 OFF다.

### Gate 3 — localization과 PointNav 좌표 검증

고정된 DB를 `localization:=true`로 읽어 다음을 시험한다. 이 모드는 pure odometry가 아니라 기존 map localization이다.

1. 측량한 시작 marker 5개에서 각 2회 cold start한다.
2. 독립 reference로 시작 위치/heading 오차를 계산한다.
3. 짧게 이동한 뒤 원위치에 돌아와 pose correction jump를 측정한다.
4. global/local closure timestamp와 controller input pose를 함께 기록한다.
5. false relocalization이나 갑작스러운 map jump가 1회라도 있으면 FAIL이다.

권장 engineering 기준은 시작 오차 ≤0.25 m/5°, 순간 correction jump ≤0.30 m/10°다. 이 값은 실제 측량 정밀도에 맞춰 campaign 전에 고정하며 결과를 본 뒤 바꾸지 않는다.

#### Localization과 Nav2의 역할

`localization:=true`는 frozen DB를 이용해 `map→odom` pose를 보정할 뿐 goal planning이나 Go2 속도 제어를 수행하지 않는다. 자율주행에는 localization 외에 planner/trajectory generator, controller, collision guard와 단일 actuator gateway가 필요하다.

현재 논문의 main 비교는 같은 frozen PixNav backend를 사용하는 `Direct-goal PixNav`와 `Full ESCAPE-PixNav`이므로 **Nav2를 main 주행기에 넣지 않는다.** Full에만 Nav2를 넣거나 Direct-goal에만 다른 controller를 쓰면 비교하려는 VLM/async mechanism 외의 차이가 생긴다.

이 campaign에서 RTAB-Map localization의 역할은 다음으로 제한한다.

- 사전 등록한 start/goal의 map-frame 좌표 제공
- controller에 전달되는 pose의 재지역화와 correction 기록
- 반복 가능한 경로와 localization-loss 판정
- trajectory/loop 로그의 공통 좌표계 제공

실제 이동은 검증된 PixNav action adapter/controller와 safety gateway가 담당해야 한다. Nav2는 추후 `Classic Nav2` 선택 baseline이나 별도의 waypoint-navigation demo가 필요할 때 같은 map/localization 위에 추가하며, 현재 50-run main campaign의 필수 조건은 아니다. RTAB pose는 정책 입력 및 진단값이지 독립 ground truth는 아니다.

### Gate 4 — frozen PixNav 독립 검증

현재 paper branch의 주 backend를 공식 Pixel-Navigator `Checkpoint_A`와 논문이 고정한 연구실 구현으로 해석한다. S2E ONNX를 이 Gate의 대체 모델로 사용하지 않는다. 이 Gate는 VLM과 모터를 분리한 상태에서 검사한다.

필수 입력과 출력:

```text
capture-view 실제 Go2 RGB + goal pixel mask
  + 이후 실제 Go2 RGB history
  + frozen PixNav Checkpoint_A
  → 6-way discrete action logits
  → file-only audit sink
```

합격 조건:

- 연구실 구현 pin `6341a5d33903131ddfce74498c04e1c0ae04ec61`, mock policy 미사용
- Checkpoint_A 존재, SHA-256 `0b1faff7631962351bbbfe8cb115a3a03069f33fab499865f887ffbb5a3cabe3`
- goal RGB/mask와 history의 224×224 preprocessing 및 입력 file hash 기록
- `stop/forward/left/right/look_up/look_down` probability, distance, tracked goal에 NaN/Inf 0
- 빈/stale image, history 초과, checkpoint mismatch 시 action을 만들지 않음
- 동일 입력 replay에서 결정론 허용범위 내 재현
- 최소 20개 실제 RGB clip에서 file-only policy output이 audit를 통과
- 이 단계에서는 `/cmd_vel`과 Sport API에 어떤 motion command도 발행하지 않음

PixNav pixel은 capture-view에 고정된다. 로봇이 이동한 뒤 동일 좌표를 현재 영상에 그대로 재사용하지 않고, capture viewpoint를 복원하거나 새 관측에서 다시 grounding해야 한다. 고정 카메라의 `look_up/look_down` zero-hold와 Habitat 이산 action의 bounded proposal 변환은 file-only로 구현됐으며, live localization/controller 연결은 별도 Gate다.

2026-08-28 수정 판정: 초기 `152009/152047/152410` 실행은 VLM이 pixel을 고른 `frame_10` 대신
`frame_00`을 goal image로 사용했으므로 capture-view acceptance에서 제외한다. 수정된
`20260828_162002` v2 run은 `frame_10`을 goal과 첫 observation으로 사용해 실제 CUDA inference를
통과했다. 기존 자료에는 그 이후 frame이 없어 아직 multi-step history 증거는 아니다.

`src/escape_nav_pixnav`에는 discrete action을 0.25 m/±30° bounded macro proposal로 변환하는
file-only adapter가 추가됐다. 고정 카메라의 `look_up/look_down`은 reobserve/zero-hold이며 모든
record가 `actuation_permitted=false`다. 56개 package test, offline causal chain, pure/file-copy
fault 22/22를 통과했지만 이는 Gate 5의 live 10분 시험이나 Gate 6의 실제 watchdog stop latency를
대체하지 않는다.

### Gate 5 — live 4-Tier 무구동 폐루프

모터 대신 파일 기반 audit sink를 사용해 다음 하나의 causal chain을 남긴다.

```text
frame_id/hash + capture_pose/time
  → VLM request_id
  → response_id/complete_time
  → admit/reject reason
  → PixNav input/output hash
  → controller command sequence/age
  → sink result
```

최소 10분 연속 실행하고 다음을 만족해야 한다.

- mock/synthetic provenance 0
- live camera와 live LIO pose만 사용
- 모든 VLM completion에 apply 또는 reject reason 존재
- observation pose와 apply pose의 causal warping 검증
- sequence gap, duplicate, out-of-order 적용 0
- actuator topic과 Sport API motion request 0

### Gate 6 — fault injection

| 고장 주입 | 기대 결과 |
|---|---|
| VLM timeout/server down/NetBird loss | local controller가 0/hold, 오래된 결정 적용 금지 |
| malformed JSON/schema mismatch | reject event와 0/hold |
| delayed/out-of-order response | sequence/TTL로 reject |
| Docker/e2e/controller process kill | Jetson watchdog가 ≤0.5 s에 zero |
| odom loss/jump | trajectory 적용 중지, 재지역화 전 motion 금지 |
| camera stale/clock reversal | 새 VLM request 중지, 0/hold |
| UDP CRC/length/magic error | packet reject, legacy fallback 금지 |
| shutdown 중 network/process 종료 | actuator가 살아 있는 동안 zero command와 ACK 확인 |

각 항목을 최소 10회 반복하고 zero/hold 실패가 한 번이라도 있으면 물리 주행 NO-GO다.

### Gate 7 — 실제 actuator 안전 검증

Gate 7 전에 코드에서 다음이 해결돼야 한다.

- `/cmd_vel` 또는 Sport API 중 하나만 최종 command authority로 선택
- sequence, source timestamp, receive timestamp, TTL, CRC를 모두 검증
- 48/56-byte legacy raw packet fallback 제거
- process exit와 signal 처리에서 zero command를 먼저 보내고 확인
- local clearance guard, speed cap, geofence, operator E-stop 독립 동작

시험은 통제 구역에서 spotter와 물리 리모컨 담당자를 분리해 수행한다.

1. zero command 유지 2분
2. 0.05 m/s 짧은 전진 후 정지
3. 0.10 m/s 1 m 직진 후 정지
4. 작은 yaw 회전 후 정지
5. 통신/프로세스 종료 중 safe-stop 10회

로봇을 엎드린 상태에서 다리를 움직이는 actuation test는 관절 간섭 위험이 있으므로 기본 절차로 사용하지 않는다. 무구동 command-sink 또는 제조사 절차에 맞는 통제된 기립 상태에서 검증한다.

### Gate 8 — 저속 pilot

속도는 처음에 `v_x ≤0.15 m/s`, `|ω_z|≤0.30 rad/s`로 제한한다.

| Pilot | 경로 | 반복 | 진입 조건 |
|---|---|---:|---|
| P0 | 5 m 직선 및 정지 | 3 | actuator Gate PASS |
| P1 | 90° L-corner | 3 | P0 collision/intervention 0 |
| P2 | T-junction 한 분기 | 3 | P1 localization/control loss 0 |
| P3 | 짧은 dead-end recovery | 3 | P2 provenance 누락 0 |
| P4 | rolling obstacle/dummy | 5 | local clearance guard PASS |

pilot 도중 parameter를 수정하면 수정 전 run은 final campaign에 포함하지 않고 새 config hash로 pilot을 다시 시작한다.

### Gate 9 — 논문용 final campaign

현재 paper 기준의 main 실로봇 비교는 **Direct-goal vs Full ESCAPE-Nav** 두 방법을 paired design으로 수행한다.

```text
5 fixed start-goal pairs × 2 methods × 5 repetitions = 50 main runs
```

- 각 repetition을 두 방법의 paired block으로 묶고, pair별 선행 방법 수 차이가 최대 1이 되도록 AB/BA order를 사전 생성한다.
- map/DB, localization, PixNav checkpoint, action adapter/controller, VLM model/prompt를 모두 고정한다.
- failure, timeout, E-stop과 intervention을 삭제하지 않는다.
- Active-view recovery와 rolling/dynamic obstacle은 main paired table과 분리해 조건별 최소 5회 수행한다.
- 사람이 움직이는 dynamic obstacle은 통제·안전 절차가 확보된 뒤에만 수행한다.

기존 문서의 4개 arena × 5 methods 계획은 모든 baseline 구현이 실제로 준비됐을 때만 확장 실험으로 사용할 수 있다. 현재 Classic Nav2, Gait-only, ViNT/NoMAD의 실행 경로가 검증되지 않았으므로 main campaign의 실행 가능 계획으로 간주하지 않는다.

## 6. 전체 일정표

| 작업일 | 목표 | 예상 run | 종료 조건 |
|---|---|---:|---|
| Day 1 | Gate 2: Type-2 OFF 두-코너 자격 + 전체 remap | 짧은 1 + 전체 1 | 물리 geometry가 맞는 golden DB 생성 |
| Day 2 | Gate 3: localization 10회 + camera calibration | 10+ | frozen DB/calibration |
| Day 3 | Gate 4: capture-view/post-capture PixNav replay | ≥20 clips | hash 고정 file-only policy evidence |
| Day 4 | Gate 5~6: 4-Tier sink와 fault injection | 정상 10분 + 고장 ≥70회 | stale/motion leakage 0 |
| Day 5 | Gate 7: actuator/E-stop/stop 반복 | ≥15 | safe-stop 10/10 |
| Day 6 | Gate 8: 저속 pilot | 12~17 | collision/intervention 0 |
| Day 7~8 | Gate 9 main campaign | 50 | complete artifact 50/50 |
| Day 9 | deployment-only와 재검증 | 조건별 ≥5 | 별도 결과표 완성 |

일정은 예상치이며 Gate 실패 시 다음 날짜로 밀린다. 마감 압박을 이유로 safety Gate를 건너뛰지 않는다.

## 7. 기록과 평가

mapping 자격시험에서는 recorder를 계속 끈다. Gate 5 이후에는 기존 `record_experiment.sh` 대신 실제 topic과 provenance를 기록하는 논문용 recorder를 사용해야 한다.

현재 recorder/evaluator는 다음 이유로 final 실험에 사용할 수 없다.

- recorder가 존재하지 않는 `/rtabmap/odom`과 잘못된 `/odom`을 기록
- `/livo/odom`, localization, decision identity, command age와 intervention event가 빠짐
- `calculate_icra_metrics.py`가 rosbag을 읽지 않고 코드 내부 sample episode를 계산

각 final run의 최소 artifact:

```text
experiments/real_robot_icra2027/<campaign_id>/<pair>/<method>/<rep>/
├── run_manifest.json
├── config_snapshot/
├── exact_vlm_inputs/
├── vlm_events.jsonl
├── pixnav_events.jsonl
├── localization.csv
├── trajectory.csv
├── command_watchdog.csv
├── interventions.jsonl
├── operator_video.mp4
├── rosbag2/
├── result.json
└── SHA256SUMS
```

핵심 결과 지표는 SR, intervention/run, timeout-normalized time, recovery success, VLM latency, motion duty, decision yield다. path metric을 보고할 때는 RTAB-Map pose를 자체 ground truth로 사용하지 않고 독립 측량/overhead reference를 사용한다.

## 8. 즉시 실행 우선순위

최신 전체-map은 분석 증거로 저장됐지만 물리 geometry FAIL이다. `bringup_all_escape_nav.sh` autonomy mode는 아직 실행하지 않는다.

1. Type-2 OFF 두-코너 짧은 loop를 통과하고 전체 map을 한 번 remap
2. 새 DB에서 표준 PGM/YAML를 export하고 코너·직선·평행성·hash 확인
3. 같은 DB를 frozen 상태로 localization cold-start 10회 검증
4. post-capture history가 있는 실제 Go2 RGB clip을 최소 20개 수집해 v2 replay
5. 구현된 file-only adapter/causal ledger를 live localization·obstacle 입력과 연결
6. mock node를 실제 node로 교체하고 live 4-Tier command sink 10분 통과
7. host bridge 단일 authority/TTL/legacy 제거와 live fault injection
8. 그 후에만 저속 물리 pilot

이 순서를 모두 통과해야 “Go2–Jetson–Docker–Server 전체 시스템이 실로봇에서 동작한다”고 주장할 수 있다.
