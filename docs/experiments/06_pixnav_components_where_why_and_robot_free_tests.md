# PixNav 구현 요소별 위치·필요 이유와 로봇 없는 시험 안내

> 기준 commit: `fd63800` 이후 작업  
> 대상: Go2 EDU Plus + Jetson Orin NX, Foxy, frozen Pixel-Navigator  
> 이 문서의 모든 시험은 로봇 이동, ROS publisher, command bridge, Unitree SDK를 사용하지 않는다.

## 1. 전체에서 어디에 들어가는가

```text
[A. VLM capture-view pixel]
        │  frame/pixel/hash
        ▼
[B. frozen PixNav CUDA inference]
        │  6-way action probability
        ▼
[C. decision contract + bounded proposal]
        │  0.25 m / ±30° proposal 또는 zero-hold
        ▼
[D. append-only audit + causal ledger]
        │  file evidence only, actuation_permitted=false
        ▼
[E. future live safety admission]
        │  localization/obstacle/E-stop 검증 — 아직 미구현
        ▼
[F. future single actuator gateway]
           실제 Go2 command — 아직 NO-GO
```

현재 구현 완료 범위는 A의 저장 artifact를 입력으로 받아 D까지다. E와 F는 로봇·실센서·안전
시험이 필요하므로 이번 file-only 구현에 일부러 포함하지 않았다.

## 2. 단계별로 왜 필요했는가

### 단계 A — capture-view 입력 계약

사용 위치: VLM이 영상의 pixel goal을 고른 직후, PixNav 입력을 만들 때.

필요 이유: pixel `(u,v)`는 그것을 선택한 영상에만 의미가 있다. 최초 구현은 VLM이 `frame_10`에서
고른 `(640,600)`을 `frame_00`에 적용했다. 모델은 실행됐지만 잘못된 목표를 본 셈이다.

구현 파일:

- `pixnav_check.py`
- `--goal-frame-index`, `--history-start-index`
- report schema `go2_pixnav_file_only_v2`

현재 동작:

- goal pixel을 정확한 capture-view에 적용
- goal capture보다 앞선 frame을 history로 지정하면 차단
- source/goal/history frame 경로와 SHA-256을 분리 기록

합격 증거: `20260828_162002_pixnav_file_only/report.json`. 기존 데이터에는 `frame_10` 이후 영상이
없어 한 시점만 합격했으며, multi-step history는 새 clip이 필요하다.

### 단계 B — frozen model identity와 실제 추론

사용 위치: VLM의 local pixel goal을 Go2가 따를 6-way local action으로 바꿀 때.

필요 이유: mock policy나 다른 checkpoint가 섞이면 논문 방법을 검증한 것이 아니다. 같은 이름의
모델이라도 byte hash와 연구실 reference commit이 다르면 결과 비교가 성립하지 않는다.

고정값:

- paper commit: `126f2f024c3cbbaa091734d0557e9d6f554adbde`
- reference commit: `6341a5d33903131ddfce74498c04e1c0ae04ec61`
- Checkpoint_A SHA-256:
  `0b1faff7631962351bbbfe8cb115a3a03069f33fab499865f887ffbb5a3cabe3`
- Jetson runtime: NVIDIA PyTorch `2.0.0+nv23.05`, CUDA, Python 3.8

`pixnav_check.py`는 실제 CUDA forward를 수행하지만 결과를 파일에만 쓴다. action을 로봇 명령으로
변환하지 않는다.

### 단계 C — action contract와 bounded macro proposal

사용 위치: PixNav inference와 미래 controller 사이.

필요 이유: PixNav 출력은 `stop/forward/turn_left/turn_right/look_up/look_down`이지 Go2 속도나
trajectory가 아니다. logits를 곧바로 `/cmd_vel`로 보내면 단위, 시간, stale 입력과 고정 카메라
행동이 정의되지 않는다.

구현 파일:

- `contracts.py`: action ID, timestamp, hash, probability, 설정 자료형
- `adapter.py`: 검증과 bounded proposal 변환

현재 proposal:

| PixNav | file-only 결과 |
|---|---|
| `stop` | zero-hold |
| `forward` | 0.25 m, 최대 0.10 m/s proposal |
| `turn_left/right` | ±30°, 최대 0.25 rad/s proposal |
| `look_up/down` | 고정 카메라이므로 이동 0, reobserve 요청 |

frame age 1.0 s, decision age 0.5 s, 선택 확률 0.55, checkpoint hash, finite 여부를 검사한다.
통과한 motion proposal도 `actuation_permitted=false`이므로 실제 명령은 아니다.

### 단계 D — hash-chain file audit

사용 위치: adapter 출력 직후, controller 대신.

필요 이유: 무엇을 결정했는지 남기지 않으면 실패 원인을 policy, stale response, controller 중 어디에
돌려야 하는지 알 수 없다. JSON 한 줄을 나중에 고치는 것도 탐지해야 한다.

구현 파일:

- `audit_sink.py`: mode 0600 append-only JSONL, 이전 record hash와 sequence 검증
- `replay.py`: v2 PixNav report를 adapter와 audit sink로 재생

차단 항목: duplicate/out-of-order sequence, record tamper, `actuation_permitted=true`.

### 단계 E — offline causal linkage

사용 위치: VLM, PixNav와 macro report가 별도 프로세스·시각에 만들어진 뒤 하나의 판단인지 확인할 때.

필요 이유: 좋은 VLM 결과와 좋은 PixNav 결과가 각각 존재해도, 서로 다른 frame/pixel을 사용했다면
End-to-End 증거가 아니다.

구현 파일:

- `causal_chain.py`

검사 연결:

```text
VLM input/raw/sanitized
  → selected capture frame와 pixel
  → PixNav v2 goal frame/checkpoint
  → macro source report와 audit tail
```

필수 VLM artifact가 `SHA256SUMS`에서 빠지거나 frame/pixel/hash가 다르면 차단한다. 현재 VLM 응답은
sanitized schema였으므로 strict live schema 합격으로 확대 해석하지 않는다.

### 단계 F — 미래 live event admission 계약

사용 위치: P6 live 4-Tier no-actuation process에서 각 비동기 event가 도착할 때.

필요 이유: VLM 응답은 늦거나 순서가 바뀔 수 있다. 이전 요청의 응답을 현재 로봇 상태에 적용하면
stale action이 된다.

구현 파일:

- `event_ledger.py`

허용 순서:

```text
frame_captured
  → vlm_submitted
  → vlm_completed
  → pixnav_completed
  → macro_audited
```

중복, 역순, parent hash 불일치, 만료와 누락 stage를 차단하고 zero-target deadman hold를 기록한다.
아직 live ROS/camera/VLM process에 연결하지 않았으므로 실제 시간 지연 증거는 아니다.

### 단계 G — fault injection

사용 위치: live chain에 연결하기 전에 fail-open 경로가 없는지 확인할 때.

필요 이유: 정상 입력만 테스트하면 hash mismatch, malformed JSON, 누락 응답과 stale decision에서
이전 action이 남는 문제를 발견하지 못한다.

구현 파일:

- `fault_injection.py`

현재 임시 파일·메모리 복사본 22개 fault가 모두 fail-closed다. server/VPN/process kill과 실제
gateway stop latency는 로봇 없는 시험으로 증명할 수 없다.

### 단계 H — qualification manifest

사용 위치: 한 번의 자격시험을 종료하고 논문/실험 증거로 동결할 때.

필요 이유: model과 결과만 고정하고 adapter source/config가 바뀌면 재현할 수 없다.

구현 파일:

- `qualification.py`

기록·검사 항목: Git commit/dirty path, checkpoint byte hash, reference commit, adapter config hash,
runtime source hash, causal identity, fault 결과, no-transport import audit. 이 manifest도 physical
readiness가 아니라 Jetson file-only qualification이다.

## 3. 로봇 없이 지금 실행할 수 있는 시험

전용 진입점은 저장소 루트의 `test_pixnav_offline.sh`다.

### 3.1 빠른 시험 — GPU·서버 불필요

```bash
cd /home/unitree/go2_ws_antarctica
./test_pixnav_offline.sh quick
```

실행 내용:

- Python 문법 검사
- `escape_nav_pixnav`만 colcon build/test
- 56 unit tests
- 저장된 VLM/causal/fault/qualification `SHA256SUMS`

CPU와 로컬 파일만 사용한다. 로봇, 카메라, LiDAR, RTAB-Map, Docker, 서버, 네트워크를 쓰지 않는다.

### 3.2 현재 source로 evidence 재생성 — GPU 불필요

```bash
./test_pixnav_offline.sh evidence
```

accepted v2 PixNav report를 재사용해 causal chain, 22-fault report와 qualification manifest를 새로
만든다. 현재 source 변경이 과거 evidence를 여전히 검증하는지 확인하는 시험이다.

### 3.3 저장 RGB로 실제 CUDA 재추론 — 로봇·서버 불필요

```bash
./test_pixnav_offline.sh cuda
```

Checkpoint_A를 Jetson GPU에 적재하고 저장된 Go2 `frame_10`으로 실제 forward를 다시 수행한 뒤
macro→causal→fault→qualification을 연속 실행한다. RTAB-Map mapping process가 살아 있으면
`pixnav_check.py`가 GPU 경합 방지를 위해 중단한다.

Codex sandbox나 제한된 SSH session에서 `/dev/nvhost*` 접근이 막혀 있으면 report가
`BLOCKED_CUDA_UNAVAILABLE`이 된다. 이때 CPU로 몰래 대체하지 않으며 스크립트도
`BLOCKED_GPU_ACCESS_OR_PREREQUISITE`로 실패한다. Jetson의 일반 사용자 shell에서 다시 실행하거나
GPU device 접근 권한을 확인한다. `quick`과 `evidence`는 GPU 접근과 무관하다.

## 4. 로봇 없이는 합격시킬 수 없는 시험

| 항목 | 필요한 이유 |
|---|---|
| capture 이후 multi-step RGB history | 기존 recording이 `frame_10`에서 끝남 |
| live camera/VLM/PixNav 10분 chain | 실제 timestamp, drop, async delay가 필요 |
| RTAB localization stale/lost admission | golden DB와 실제 이동 pose가 필요 |
| L2 obstacle/clearance guard | 실제 4D L2와 robot footprint가 필요 |
| Docker/server/network loss recovery | live process와 route를 끊는 시험 필요 |
| actuator watchdog ≤0.5 s | 실제 gateway→Go2 stop 도달 시간을 측정해야 함 |
| E-stop·safe-stop 10/10과 pilot | 통제 구역, operator/spotter, 물리 승인 필요 |

따라서 로봇 없는 PASS는 P0~P5의 software/evidence 범위다. 이것을 P6 live chain이나 P8 physical
safety PASS로 승격하지 않는다.

## 5. 2026-08-28 실제 무구동 실행 결과

세 모드를 이 문서 작성 시점에 직접 실행했다.

| 모드 | 결과 | 핵심 측정 |
|---|---|---|
| `quick` | **PASS_ROBOT_FREE_QUICK** | 56/56 tests, 기존 SHA manifest 전체 OK |
| `evidence` | **PASS_ROBOT_FREE_EVIDENCE** | causal PASS, pure/file-copy fault 22/22, qualification PASS |
| `cuda` | **PASS_ROBOT_FREE_CUDA** | 저장 RGB 실제 CUDA forward 2.751 s, 이후 전체 chain PASS |

최신 `cuda` evidence:

```text
PixNav:
  ~/.ros/pixnav_runs/20260828_164126_pixnav_file_only/report.json
Macro:
  ~/.ros/pixnav_macro_runs/20260828_164137_pixnav_macro_file_only/
Causal:
  ~/.ros/pixnav_chain_runs/20260828_164137_pixnav_offline_chain/
Fault:
  ~/.ros/pixnav_fault_runs/20260828_164137_pixnav_fault_injection/ (22/22)
Qualification:
  ~/.ros/pixnav_qualification_runs/20260828_164138_pixnav_qualification/
```

제한된 sandbox 실행에서는 GPU device가 보이지 않아
`~/.ros/pixnav_runs/20260828_164025_pixnav_file_only/report.json`이
`BLOCKED_CUDA_UNAVAILABLE`을 기록했다. 같은 코드와 입력을 GPU device 접근이 허용된 Jetson
환경에서 다시 실행하자 CUDA PASS했다. 따라서 `164025`는 model/input failure가 아니라 실행
권한·device 노출 진단 기록이다.

남은 경고는 optional `torchvision.io/image.so` ABI, PyTorch TypedStorage deprecation과 attention
mask dtype 성능 경고다. 현재 image decode는 OpenCV를 사용하고 출력은 finite였으므로 이번
file-only acceptance를 막지 않았지만 production runtime 정리 항목으로 유지한다.
