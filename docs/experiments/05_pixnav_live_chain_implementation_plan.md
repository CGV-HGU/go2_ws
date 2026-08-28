# PixNav → live 4-Tier 구현 계획과 현재 완료 기록

> 기준: 2026-08-28 16:30 KST  
> 현재 허용 범위: Jetson file-only/no-actuation  
> 물리 판정: controller·actuator·자율주행 **NO-GO**

## 1. 구현 경계

실제 **주행 정책 경로**는 다음과 같다. RTAB-Map이나 Nav2가 목표 경로를 생성하지 않는다.

```text
Go2 RGB capture
  → remote VLM capture-view pixel grounding
  → frozen PixNav Checkpoint_A
  → bounded macro proposal
  → obstacle/freshness/E-stop safety admission
  → single actuator gateway
```

RTAB localization과 L2 odometry는 아래의 별도 side-channel이다.

```text
RTAB map pose + L2 odometry/clearance
  → pose freshness/loss, 실제 translation, collision/stall, experiment trajectory
  → safety admission + execution evidence + evaluation log
```

RTAB pose는 PixNav pixel/action을 생성하거나 Nav2 global path를 만드는 입력이 아니다. 최신 paper
계약처럼 localization을 명시적으로 선언한 경우에만 상대 실행 증거와 graph geometry에 사용하며,
숨겨진 target geometry를 유도하지 않는다. main 비교는 동일 frozen PixNav backend를 사용하는
Direct-goal PixNav와 Full ESCAPE-PixNav이므로 Nav2는 main 주행기에 포함하지 않는다.

현재 구현은 `bounded macro proposal`까지 파일과 메모리 계약으로만 도달한다. ROS publisher,
socket, Unitree SDK와 actuator authority는 의도적으로 포함하지 않았다.

## 2. 단계별 구현 상태

| Phase | 구현물 | 합격 기준 | 현재 |
|---:|---|---|---|
| P0 | paper/reference/checkpoint pin | commit와 Checkpoint_A byte hash 일치 | **PASS** |
| P1 | capture-view v2 replay | goal frame 이후 observation만 허용, finite CUDA output | **1-step PASS** |
| P2 | 6-way→bounded proposal | 확률/hash/age/TTL 검증, vertical action zero-hold | **PASS** |
| P3 | immutable file audit | sequence와 hash-chain, tamper/actuation 차단 | **PASS** |
| P4 | offline causal linkage | VLM pixel/frame→PixNav→macro hash 연결 | **PASS** |
| P5 | pure fault harness | malformed/missing/stale/duplicate/tamper를 fail-closed | **22/22 PASS** |
| P6 | live no-actuation source | camera/localization/VLM/PixNav event를 10분 연결 | **미구현** |
| P7 | live safety admission | pose/obstacle/E-stop/clock stale에서 zero-hold | **미구현** |
| P8 | single actuator gateway | 한 authority, ACK, clamp, deadman, safe-stop 10/10 | **미구현** |
| P9 | supervised pilot | 직선→L-corner→T-junction 순차 통과 | **미수행** |

## 3. 완료된 package

`src/escape_nav_pixnav`의 역할은 다음으로 제한한다.

- action ID: `stop=0`, `forward=1`, `turn_left=2`, `turn_right=3`, `look_up=4`,
  `look_down=5`
- forward: 0.25 m proposal, 최대 0.10 m/s와 0.20 m/s²
- turn: ±30° proposal, 최대 0.25 rad/s와 0.50 rad/s²
- 선택 확률 0.55 미만, frame age 1.0 s 초과, decision age 0.5 s 초과는 zero-hold
- 고정 카메라의 vertical action은 `reobserve`, 이동 목표 0
- 모든 결과는 `actuation_permitted=false`
- event 순서:
  `frame_captured → vlm_submitted → vlm_completed → pixnav_completed → macro_audited`

검증 명령:

```bash
cd /home/unitree/go2_ws_antarctica
colcon build --packages-select escape_nav_pixnav --symlink-install
colcon test --packages-select escape_nav_pixnav --event-handlers console_direct+
colcon test-result --verbose
```

현재 결과는 56 tests, 0 errors, 0 failures다.

## 4. 다음 구현: P6 live no-actuation source

로봇 전원을 켜되 모터 command path를 시작하지 않고 다음 node/process를 만든다.

1. `capture_adapter`: `/camera/front/image_raw`를 goal/history 계약으로 저장하고, 해당 시각 RTAB
   pose는 정책 입력이 아닌 별도 execution-evidence sidecar로 같은 `causal_id`에 연결
2. `vlm_client`: request/response body hash, submit/complete monotonic timestamp 기록
3. `pixnav_worker`: capture frame을 goal로, 그 시각 이후 frame만 history로 투입
4. `proposal_sink`: 기존 adapter와 `AuditJsonlSink`만 호출
5. `live_manifest`: commit/config/model/topic/QoS/process PID와 output hash 동결

P6 acceptance:

- 10분 동안 sequence gap/duplicate/out-of-order 적용 0
- 모든 VLM completion에 apply/reject reason 존재
- capture 이전 frame이 PixNav history에 들어간 횟수 0
- `/cmd_vel`, Sport API, production command UDP 발행 0
- process kill 또는 server timeout에서 event ledger가 zero-target deadman hold 기록

P6에서 RTAB pose가 아직 없더라도 RGB→VLM→PixNav→file sink의 무구동 정책 체인은 시험할 수
있다. 다만 pose freshness/loss와 실제 이동량을 사용하는 execution-grounded 판정은 골든맵 기반
localization 또는 명시적으로 선언한 odometry source를 연결한 뒤에만 합격시킨다.

## 5. P7/P8 진입 전 금지 사항

- proposal을 시간 기반 open-loop velocity로 바로 변환
- RTAB pose가 stale/lost인데 마지막 action 재적용
- `/cmd_vel`과 Unitree Sport API를 동시에 사용
- 파일 fault 22/22를 실제 stop latency 증거로 표기
- 로봇을 엎드린 상태에서 다리 구동 시험

P7에서 localization age/jump, L2 local clearance, camera age, operator enable/E-stop을 모두 통과한
proposal만 gateway 후보가 된다. P8은 별도 물리 승인, 통제 구역, operator/spotter와 제조사 리모컨
E-stop을 준비한 뒤 진행한다.

## 6. 현재 evidence

| 범위 | 경로 |
|---|---|
| 수정 v2 CUDA | `~/.ros/pixnav_runs/20260828_162002_pixnav_file_only/` |
| macro audit | `~/.ros/pixnav_macro_runs/20260828_162023_pixnav_macro_file_only/` |
| offline causal chain | `~/.ros/pixnav_chain_runs/20260828_162122_pixnav_offline_chain/` |
| fault 22/22 | `~/.ros/pixnav_fault_runs/20260828_163454_pixnav_fault_injection/` |
| qualification manifest | `~/.ros/pixnav_qualification_runs/20260828_163514_pixnav_qualification/` |

초기 `152009/152047/152410` PixNav run은 goal/history pairing 오류가 있으므로 P1 acceptance에서
사용하지 않는다. 새 live capture는 VLM 선택 시점 이후의 history를 반드시 추가로 수집한다.
