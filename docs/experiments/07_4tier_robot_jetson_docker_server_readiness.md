# Robot–Jetson–Docker–Server 4-Tier 필요 요소·구현률·무구동 시험

> 실측 시각: 2026-08-28 18:03 KST
> 기준 commit: `969029a` + nearest-stamp guard source hash manifest
> 안전 범위: live RGB/L2/odom, Docker VLM, Jetson PixNav는 read-only/file-only; command bridge, `/cmd_vel`, Sport API는 시작하지 않음
> 퍼센트 의미: 논문 실로봇 4-Tier deployment에 필요한 항목을 Tier별 100점으로 가중한
> engineering readiness estimate. 논문 성능 수치가 아니며 코드 파일 개수로 계산하지 않음.

## 1. 현재 한 줄 결론

| 범위 | 진행률 | 현재 판정 |
|---|---:|---|
| Tier 1 Robot | **58%** | L2/IMU/odom/RGB live PASS, calibration·actuator safety 미완료 |
| Tier 2 Jetson | **65%** | persistent PixNav와 L2/odom P7 입구 PASS, golden map·localization 미완료 |
| Tier 3 Docker | **45%** | 실제 VLM transport process 1-cycle PASS, 상주 service·fault/soak 미완료 |
| Tier 4 Server | **84%** | live RGB strict pixel/confidence PASS, retry/load/failure campaign 미완료 |
| 네 Tier 구성요소 단순 평균 | **63%** | 구성요소 평균일 뿐 자율주행 판정에 사용 금지 |
| 실제 4-Tier End-to-End Gate | **64%** | live safe file sink/P7까지 PASS, P8 actuator와 10분 soak 없음 |

전체 프로젝트 마스터플랜의 약 45%와 숫자가 다른 이유는 이 문서가 4-Tier 통신·배치만 별도로
평가하기 때문이다. E2E 64%는 물리 자율주행 준비율이 아니다. P8 단일 actuator/E-stop, golden
localization, pilot와 논문 campaign은 여전히 NO-GO/미수행이다.

## 2. Tier 1 — Go2 Robot: 58/100

### 이 Tier에서 필요한 것

- 내장 4D L2 cloud, IMU, Unitree LIO odometry와 front RGB 제공
- sensor clock/QoS/rate/drop의 반복 가능한 health
- camera intrinsics와 camera↔L2↔base_link extrinsics
- 하나의 actuator command interface와 ACK/watchdog
- operator remote E-stop, 안전한 stand/stop/exit

### 점수

| 항목 | 가중치 | 확보 | 근거와 남은 것 |
|---|---:|---:|---|
| L2/IMU/LIO 실제 stream | 25 | 25 | 정상 기립·정지 실수신: cloud 15.49 Hz, IMU 247.03 Hz, odom 149.92 Hz |
| front RGB RTP | 15 | 15 | 실제 1280×720 capture-view+4 history를 live chain에 사용 |
| rate/timestamp/QoS/drop 반복 검증 | 15 | 12 | 6초 preflight와 P7 동시 수신 PASS; reboot/장시간 반복 pending |
| 정식 intrinsics/extrinsics | 15 | 4 | approximate TF만 사용, 정식 calibration 미통과 |
| 단일 command/ACK/watchdog 경계 | 15 | 2 | 가능한 API는 있으나 authority 선정·safe-stop 미검증 |
| E-stop과 물리 safe-stop 반복 | 15 | 0 | 10/10 시험과 pilot 미수행 |
| **합계** | **100** | **58** | |

`192.168.123.161` ping 3/3과 실제 DDS/RTP sample을 새로 확인했다. 이 판정은 센서 입력 범위이며
camera/L2 calibration, actuator ACK와 물리 safe-stop 증거는 아니다.

### 다음 완료 순서

1. Robot power-on 후 L2/IMU/LIO/RGB rate·timestamp preflight
2. camera/L2 extrinsic과 intrinsics calibration
3. mapping/localization 동안 sensor dropout 기준 동결
4. Tier 2~4 무구동 Gate 뒤 단일 actuator와 E-stop 시험

## 3. Tier 2 — Jetson Orin NX/Foxy: 65/100

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
| frozen PixNav v2 CUDA | 15 | 15 | persistent Checkpoint_A, 최신 5-frame CUDA 0.090 s, P7 source age 0.271 s |
| adapter/audit/causal/fault | 15 | 15 | 72 tests, fault 22/22, no-actuation qualification PASS |
| live admission/recorder/gateway 전단 | 10 | 7 | live L2/odom freshness·clearance P7 평가 PASS; operator/E-stop pending |
| **합계** | **100** | **65** | |

현재 Docker/NetBird/NetworkManager service는 active, eth0에는 campus와 Go2 direct subnet이 동시에
있고 NetBird `wt0`도 존재한다. mapping, sensor bridge, PixNav worker와 command bridge process는
현재 실행 중이지 않다.

### 다음 완료 순서

1. Type-2 OFF 두-코너 자격과 전체 golden remap
2. golden DB localization cold-start 10회
3. live event source와 file sink 10분 및 fault 반복
4. capture 이후 history를 포함한 PixNav 20 clips
5. operator-enable/E-stop admission과 recorder

## 4. Tier 3 — Docker/Jazzy: 45/100

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
| 실제 navigation process | 20 | 6 | ephemeral strict VLM transport process 1-cycle PASS; 상주 service는 없음 |
| real frozen PixNav 배치 | 15 | 0 | paper backend는 Jetson host persistent worker로 배치; Docker PixNav는 불필요 |
| Host↔Docker protocol/TTL/identity | 15 | 6 | mounted capture hash와 causal identity 연결; 장시간/restart pending |
| live VLM async/causal process | 15 | 8 | 실제 image request/response strict validator PASS; async queue/late reject pending |
| file sink/supervisor/fault runtime | 10 | 4 | host file sink에 실제 chain 연결; Docker supervisor/fault runtime 없음 |
| **합계** | **100** | **45** | |

설치된 executable 10개가 보이지만 `e2e_node`, `controller_node`, `vlm_node`는
`node_contracts.run_ros_node()`을 통해 `ros_mock_runtime`을 사용한다. 특히 controller mock runtime은
command topic publisher를 만들 수 있으므로 무구동 시험에서 이 node들을 시작하지 않는다.
컨테이너는 `network=host`, workspace와 `/dev`가 mount돼 있으므로 idle이라는 사실이 안전성의 핵심이다.
Docker가 표시하는 `Up 56 years`는 container `StartedAt`이 1970년으로 기록된 host/container clock
artifact이며 실제 uptime이나 장기 안정성 증거가 아니다.

### 다음 완료 순서

1. Docker 역할을 VLM orchestration으로 제한하고 PixNav는 Jetson CUDA worker로 유지
2. ephemeral client를 **no-actuation 상주 service**로 교체
3. Jetson↔Docker schema에 causal ID, monotonic time, TTL, CRC 추가
4. mock backend를 acceptance에서 차단하고 실제 provenance 기록
5. 10분 live file sink와 process-kill fault 시험

## 5. Tier 4 — Remote GPU Server: 84/100

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
| 저장 실제 RGB multimodal | 15 | 15 | live capture-view Docker→server HTTP 200, 최근 1.194 s |
| strict navigation semantic schema | 15 | 15 | image-bound pixel equality, view/action/confidence 로컬 검증 PASS |
| timeout/retry/idempotency | 10 | 3 | timeout과 causal hash는 구현; retry/late duplicate 반복 pending |
| load/latency/failure campaign | 10 | 1 | 단발 측정만 존재 |
| **합계** | **100** | **84** | |

과거 `selected_image_point`와 `fine_goal.point_px` 불일치 결과는 새 strict validator로
supersede됐다. 현재 client는 image-bound integer, top-level/fine pixel equality, view/action과
confidence를 로컬에서 재검사하고 실패 또는 confidence 0.55 미만이면 PixNav 전에 zero-hold한다.

### 다음 완료 순서

1. JSON Schema에 image-bound integer 제약 추가
2. response 후 동일 pixel, view ID, frame hash semantic validator 적용
3. request/response causal ID와 timeout·late response rejection
4. Host/Docker 각각 장시간 latency와 server loss 반복

## 6. 실제 4-Tier End-to-End: 64/100

| 연결 Gate | 가중치 | 확보 | 현재 |
|---|---:|---:|---|
| Robot→Jetson live sensor | 20 | 20 | 실제 RGB/L2/odom 동시 수신 PASS |
| Jetson→Docker real protocol | 20 | 12 | 실제 capture hash/causal identity 전달; 상주 service pending |
| Docker→Server inference | 20 | 20 | live strict multimodal inference PASS |
| 단일 live causal chain | 20 | 12 | 1-cycle P6+P7 artifact PASS; 10분/fault pending |
| safe sink→single actuator | 20 | 0 | file-only까지만, physical gateway NO-GO |
| **합계** | **100** | **64** | |

## 7. 지금 로봇 없이 할 수 있는 시험

```bash
cd /home/unitree/go2_ws_antarctica

# 약 2초: 현재 process와 연결 상태만 확인
./test_4tier_no_actuation.sh status

# Jetson 72 tests + Docker 63 pass/6 skip + Server structured text inference
./test_4tier_no_actuation.sh full

# 저장 RGB 실제 PixNav CUDA와 downstream evidence
./test_pixnav_offline.sh cuda
```

`full`도 navigation controller와 command bridge를 시작하지 않는다. 최신 live P6/P7 one-cycle은
`pixnav_live_check.py`가 별도 source/artifact manifest와 함께 수행한다.

## 8. Robot 전원 후 가장 먼저 할 시험

1. ~~Tier 1 reachability와 L2/IMU/LIO/RGB preflight~~
2. `camera→Docker→Server→PixNav→L2/odom P7→file sink` 10분 live chain
3. server/network/Docker kill과 stale camera/odom fault
4. Type-2 OFF RTAB 두-코너 자격, 전체 golden map과 localization
5. 물리 operator-enable/E-stop 입력 연결
6. 위가 모두 PASS한 뒤 별도 승인으로 P8 single actuator gateway

현재 바로 진행할 software 작업은 10분 P6 soak/fault와 Tier 3 **no-actuation 상주 service**다.
Tier 4 strict semantic validator는 완료됐고, 물리 이동은 여전히 승인 범위 밖이다.
