# Robot–Jetson–Docker–Server 4-Tier 필요 요소·구현률·무구동 시험

> 실측 시각: 2026-08-28 16:44~16:47 KST  
> 기준 commit: `db9ca82`  
> 안전 범위: mapping, navigation node, command bridge, `/cmd_vel`, Sport API를 시작하지 않음  
> 퍼센트 의미: 논문 실로봇 4-Tier deployment에 필요한 항목을 Tier별 100점으로 가중한
> engineering readiness estimate. 논문 성능 수치가 아니며 코드 파일 개수로 계산하지 않음.

## 1. 현재 한 줄 결론

| 범위 | 진행률 | 현재 판정 |
|---|---:|---|
| Tier 1 Robot | **45%** | 실센서 과거 PASS, 현재 전원/연결 없음, calibration·actuator safety 미완료 |
| Tier 2 Jetson | **58%** | LIVO/planar/PixNav file-only 강점, golden map·localization·live admission 미완료 |
| Tier 3 Docker | **28%** | image/package/test 골격만 PASS, 실제 process·PixNav·live chain 없음 |
| Tier 4 Server | **66%** | model/host+Docker/실추론 PASS, live semantic schema·retry/load 미완료 |
| 네 Tier 구성요소 단순 평균 | **49%** | 구성요소 평균일 뿐 E2E 판정에 사용 금지 |
| 실제 4-Tier End-to-End Gate | **37%** | offline artifact chain까지만 PASS, live Robot→Server→safe sink 없음 |

전체 프로젝트 마스터플랜의 약 39%와 숫자가 약간 다른 이유는 이 문서가 4-Tier 통신·배치만
별도로 평가하기 때문이다. E2E에서는 가장 약한 Docker/live safety 연결이 병목이므로 Tier 평균
49%를 “자율주행 절반 완료”로 해석하지 않는다.

## 2. Tier 1 — Go2 Robot: 45/100

### 이 Tier에서 필요한 것

- 내장 4D L2 cloud, IMU, Unitree LIO odometry와 front RGB 제공
- sensor clock/QoS/rate/drop의 반복 가능한 health
- camera intrinsics와 camera↔L2↔base_link extrinsics
- 하나의 actuator command interface와 ACK/watchdog
- operator remote E-stop, 안전한 stand/stop/exit

### 점수

| 항목 | 가중치 | 확보 | 근거와 남은 것 |
|---|---:|---:|---|
| L2/IMU/LIO 실제 stream | 25 | 20 | 과거 실주행 수신 PASS; 현재 ping 실패로 반복 확인 불가 |
| front RGB RTP | 15 | 12 | 실제 JPG 저장 이력 존재; 현재 live stream 없음 |
| rate/timestamp/QoS/drop 반복 검증 | 15 | 7 | 과거 rate 측정은 있으나 reboot/장시간 반복 Gate 미통과 |
| 정식 intrinsics/extrinsics | 15 | 4 | approximate TF만 사용, 정식 calibration 미통과 |
| 단일 command/ACK/watchdog 경계 | 15 | 2 | 가능한 API는 있으나 authority 선정·safe-stop 미검증 |
| E-stop과 물리 safe-stop 반복 | 15 | 0 | 10/10 시험과 pilot 미수행 |
| **합계** | **100** | **45** | |

현재 `192.168.123.161` ping은 100% loss이고 ROS graph에 sensor node/topic이 없다. 따라서 지금
Tier 1에서 할 수 있는 시험은 reachability뿐이며, 센서 PASS를 새로 선언할 수 없다.

### 다음 완료 순서

1. Robot power-on 후 L2/IMU/LIO/RGB rate·timestamp preflight
2. camera/L2 extrinsic과 intrinsics calibration
3. mapping/localization 동안 sensor dropout 기준 동결
4. Tier 2~4 무구동 Gate 뒤 단일 actuator와 E-stop 시험

## 3. Tier 2 — Jetson Orin NX/Foxy: 58/100

### 이 Tier에서 필요한 것

- Unitree raw stream을 `/livo/*`로 정규화하는 sensor bridge
- planar 3DoF RTAB-Map mapping과 frozen-map localization
- frozen PixNav CUDA runtime
- PixNav output validation, bounded proposal와 zero-hold
- live camera/pose/VLM/PixNav causal admission
- Robot으로 가기 전 최종 local safety gateway와 recorder

### 점수

| 항목 | 가중치 | 확보 | 근거와 남은 것 |
|---|---:|---:|---|
| LIVO bridge/TF/zero-padding 처리 | 15 | 13 | 실제 sensor run PASS, 다음 전원 run 반복 필요 |
| planar 3DoF/global loop 기능 | 15 | 11 | Z 안정·Type-1 PASS, Type-2 OFF 물리 재자격 필요 |
| golden map과 artifact | 15 | 4 | export 기능은 있으나 최신 전체-map geometry FAIL |
| frozen DB localization | 15 | 0 | golden DB 부재로 cold-start 10회 미실행 |
| frozen PixNav v2 CUDA | 15 | 13 | 저장 RGB 실제 CUDA 2.751 s PASS, 20 post-capture clips 필요 |
| adapter/audit/causal/fault | 15 | 15 | 56 tests, fault 22/22, no-actuation qualification PASS |
| live admission/recorder/gateway 전단 | 10 | 2 | 계약만 있고 live sensor/process 연결 없음 |
| **합계** | **100** | **58** | |

현재 Docker/NetBird/NetworkManager service는 active, eth0에는 campus와 Go2 direct subnet이 동시에
있고 NetBird `wt0`도 존재한다. mapping, sensor bridge, PixNav worker와 command bridge process는
현재 실행 중이지 않다.

### 다음 완료 순서

1. Type-2 OFF 두-코너 자격과 전체 golden remap
2. golden DB localization cold-start 10회
3. capture 이후 history를 포함한 PixNav 20 clips
4. live event source와 file sink 10분
5. localization/obstacle/E-stop admission과 recorder

## 4. Tier 3 — Docker/Jazzy: 28/100

### 이 Tier에서 필요한 것

- 재현 가능한 image digest와 실제 시작 command
- Jetson camera/pose를 받는 명시적 protocol과 sequence/timestamp/TTL
- remote VLM async request/response identity
- paper-pinned frozen PixNav 또는 명확한 host-worker interface
- bounded proposal file sink와 supervisor
- 이후 검증된 단일 gateway로만 전달

### 점수

| 항목 | 가중치 | 확보 | 근거와 남은 것 |
|---|---:|---:|---|
| container/image/host network | 10 | 8 | Jazzy image digest 존재, host network·workspace mount 확인 |
| package/executable/contract tests | 15 | 13 | core 43, bringup 3, nodes 17 PASS·6 skip |
| 실제 navigation process | 20 | 0 | PID는 `tail -f /dev/null` 하나뿐 |
| real frozen PixNav 배치 | 15 | 0 | PixNav runtime은 Jetson host에만 있고 Docker node에 없음 |
| Host↔Docker protocol/TTL/identity | 15 | 3 | 골격/과거 UDP smoke만 존재, production contract 미완료 |
| live VLM async/causal process | 15 | 2 | package 계약은 있으나 실행 node 없음 |
| file sink/supervisor/fault runtime | 10 | 2 | mock/계약 code만 있고 live chain 미배치 |
| **합계** | **100** | **28** | |

설치된 executable 10개가 보이지만 `e2e_node`, `controller_node`, `vlm_node`는
`node_contracts.run_ros_node()`을 통해 `ros_mock_runtime`을 사용한다. 특히 controller mock runtime은
command topic publisher를 만들 수 있으므로 무구동 시험에서 이 node들을 시작하지 않는다.
컨테이너는 `network=host`, workspace와 `/dev`가 mount돼 있으므로 idle이라는 사실이 안전성의 핵심이다.
Docker가 표시하는 `Up 56 years`는 container `StartedAt`이 1970년으로 기록된 host/container clock
artifact이며 실제 uptime이나 장기 안정성 증거가 아니다.

### 다음 완료 순서

1. Docker의 역할을 VLM orchestration으로 제한할지 PixNav까지 넣을지 확정
2. `tail -f /dev/null`을 **no-actuation real process**로 교체
3. Jetson↔Docker schema에 causal ID, monotonic time, TTL, CRC 추가
4. mock backend를 acceptance에서 차단하고 실제 provenance 기록
5. 10분 live file sink와 process-kill fault 시험

## 5. Tier 4 — Remote GPU Server: 66/100

### 이 Tier에서 필요한 것

- 고정 model ID와 serving/runtime identity
- Host와 Docker 양쪽의 stable reachability
- 실제 RGB multimodal inference
- strict navigation JSON과 pixel semantic validation
- request ID, input hash, timeout/retry/idempotency
- latency/load/disconnect campaign

### 점수

| 항목 | 가중치 | 확보 | 근거와 남은 것 |
|---|---:|---:|---|
| vLLM/model serving | 20 | 20 | `qwen3.5-9b-instruct`, `/v1/models` HTTP 200 |
| Host+Docker reachability | 15 | 15 | 각각 0.0253 s, 0.0237 s |
| 실제 text inference | 15 | 15 | strict text JSON HTTP 200, 0.3965 s |
| 저장 실제 RGB multimodal | 15 | 12 | Docker→server JPG 45,835 bytes, HTTP 200, 1.7757 s |
| strict navigation semantic schema | 15 | 2 | JSON Schema 문법 강제 가능, pixel 음수/두 필드 불일치로 semantic FAIL |
| timeout/retry/idempotency | 10 | 1 | client production 정책·중복 차단 미검증 |
| load/latency/failure campaign | 10 | 1 | 단발 측정만 존재 |
| **합계** | **100** | **66** | |

일반 image prompt 응답은 Markdown code fence와 추가 필드를 포함하고 max-token에서 잘렸다. 서버의
`response_format=json_schema`를 쓰면 JSON parse는 성공했지만
`selected_image_point=[-1,-1]`, `fine_goal.point_px=[499,499]`로 semantic contract는 실패했다.
따라서 production client는 schema의 pixel 범위를 제한하고, top-level/fine pixel equality와 image
크기를 로컬에서 재검사해야 한다. 실패 시 PixNav에 넘기지 않고 zero-hold해야 한다.

### 다음 완료 순서

1. JSON Schema에 image-bound integer 제약 추가
2. response 후 동일 pixel, view ID, frame hash semantic validator 적용
3. request/response causal ID와 timeout·late response rejection
4. Host/Docker 각각 장시간 latency와 server loss 반복

## 6. 실제 4-Tier End-to-End: 37/100

| 연결 Gate | 가중치 | 확보 | 현재 |
|---|---:|---:|---|
| Robot→Jetson live sensor | 20 | 8 | 과거 실측만 있고 현재 Robot offline |
| Jetson→Docker real protocol | 20 | 5 | 공유/연결 골격만 있고 실제 worker 없음 |
| Docker→Server inference | 20 | 16 | text/image transport PASS, semantic validation 미완료 |
| 단일 live causal chain | 20 | 8 | offline VLM→PixNav→macro만 PASS |
| safe sink→single actuator | 20 | 0 | file-only까지만, physical gateway NO-GO |
| **합계** | **100** | **37** | |

## 7. 지금 로봇 없이 할 수 있는 시험

```bash
cd /home/unitree/go2_ws_antarctica

# 약 2초: 현재 process와 연결 상태만 확인
./test_4tier_no_actuation.sh status

# Jetson 56 tests + Docker 63 pass/6 skip + Server structured text inference
./test_4tier_no_actuation.sh full

# 저장 RGB 실제 PixNav CUDA와 downstream evidence
./test_pixnav_offline.sh cuda
```

`full`도 Robot sensor, navigation ROS node, controller와 command bridge를 시작하지 않는다. Tier 1이
offline이어도 다른 Tier의 독립 시험은 계속하지만 최종 출력은 Tier 1 blocked로 남긴다.

## 8. Robot 전원 후 가장 먼저 할 시험

1. Tier 1 reachability와 L2/IMU/LIO/RGB preflight
2. Type-2 OFF RTAB 두-코너 자격
3. 전체 golden map과 localization
4. `camera+localization→Docker→Server→PixNav→file sink` 10분 live chain
5. server/network/Docker kill과 stale pose/image fault
6. 위가 모두 PASS한 뒤 별도 승인으로 actuator/E-stop

현재 바로 진행해야 할 software 작업은 Tier 3의 **no-actuation real process**와 Tier 4 semantic
validator다. 두 항목은 Robot이 없어도 구현할 수 있지만, live PASS는 Robot sensor가 있어야 한다.
