# 매핑 복귀 직후 PixNav 무구동 자격시험

> 기준일: 2026-08-28 KST  
> 적용 로봇: Unitree Go2 EDU Plus + Jetson Orin NX + 내장 전면 RGB + L2 4D LiDAR  
> 안전 상태: 이 문서 전체에서 물리 구동은 **NO-GO**다. `/cmd_vel`, Unitree Sport API,
> UDP 9090/9091 command bridge를 실행하지 않는다.

## 1. 이번에 검증할 대상

현재 논문에서 주 local-navigation backend는 **S2E가 아니라 frozen Pixel-Navigator(PixNav)**다.

```text
VLM이 선택한 capture-view RGB pixel goal
  + 이후 RGB observation history
  → frozen PixNav Checkpoint_A
  → stop / forward / left / right / look_up / look_down
  → 이번 시험에서는 JSON 파일에만 기록
```

- S2E는 별도의 NavBench-GS 보조 실험이며 이번 Go2 PixNav 자격시험 대상이 아니다.
- `pixnav_s2e_check.py`와 기존 S2E evidence는 과거 분리 실험 기록으로만 남긴다.
- PixNav 출력은 Habitat식 이산 action이다. 아직 Go2 이동 명령으로 변환하지 않는다.
- RTAB-Map map/localization과 PixNav policy runtime은 서로 다른 Gate다. 둘을 각각 통과한 뒤에만
  연결을 검토한다.

## 2. 고정한 논문·코드·모델 버전

| 항목 | 고정값 | 의미 |
|---|---|---|
| 논문 저장소 branch | `CGV-HGU/s2e-vlm-async-framework:paper` | 2026-08-28 최신 논문 기준 |
| 논문 commit | `126f2f024c3cbbaa091734d0557e9d6f554adbde` | `updated paper` |
| 연구실 PixNav 구현 pin | `6341a5d33903131ddfce74498c04e1c0ae04ec61` | 논문 branch의 `reference/vlm-s2e-integration` gitlink |
| 공식 모델 | Pixel-Navigator `Checkpoint_A` | 현재 frozen local policy |
| checkpoint SHA-256 | `0b1faff7631962351bbbfe8cb115a3a03069f33fab499865f887ffbb5a3cabe3` | 연구실 `LOCAL_DATA.md`와 동일해야 함 |

로컬 준비 경로는 Git에서 제외한다.

```text
.local-data/vlm-s2e/runtime/vlm-s2e-integration-paper-pin/
.local-data/vlm-s2e/checkpoints/pixelnav_A.ckpt
```

모델 파일이 존재한다는 것만으로 PASS가 아니다. 위 hash와 구현 pin을 모두 확인한다.

2026-08-28 16:28 KST 기준 구현 pin과 217,967,433-byte checkpoint hash뿐 아니라 실제 CUDA
inference도 PASS했다. 단, 최초 세 run은 VLM이 pixel을 선택한 `frame_10`이 아니라 `frame_00`을
goal image로 잘못 사용했다. 이 결과들은 checkpoint forward/결정론 진단 증거일 뿐 capture-view
계약 합격 증거에서는 철회한다. 수정된 v2 run은 `frame_10`을 goal image이자 첫 observation으로
사용했고 모든 출력이 finite였다. ROS, socket, Unitree SDK, command publisher 호출은 0회였다.

### 2.1 Jetson runtime 선택

실측 플랫폼은 L4T `35.3.1`, JetPack 5.1.1 계열, CUDA `11.4.315`, aarch64, Ubuntu 20.04,
host Python 3.8.10이다. 현재 `sdam_go2_container`는 `arm64v8/ros:jazzy-ros-base`, Python
3.12.3이며 `torch/torchvision`이 둘 다 없다. 연구실 저장소의 검증 환경인 Python 3.9 +
PyTorch 2.8/CUDA 12.8 pin을 이 Jetson에 그대로 설치하지 않는다.

현재 Foxy/system Python을 오염시키지 않고 Git 제외 경로
`.local-data/pixnav_runtime/site-packages`에 package를 격리했다. NVIDIA가 JetPack 5.1.1용으로
배포한 Python 3.8 aarch64 wheel `torch 2.0.0+nv23.05`와 `torchvision 0.15.1`을 사용한다.
논문 pin의 `settings.py`는 Python 3.9식 `list[str]` annotation을 쓰므로 `pixnav_check.py`가
Python 3.8에서 annotation 평가만 지연한다. 참고 checkout과 policy 계산은 수정하지 않았다.

`torchvision.io/image.so`에는 optional image extension ABI 경고가 남아 있다. 현재 검사는 OpenCV로
이미지를 읽고 torchvision ResNet policy를 실제 실행하여 통과했으므로 이 경고는 현재 replay를
차단하지 않는다. 다만 production runtime에서는 torchvision image I/O 사용을 금지하거나 정확히
맞는 build로 다시 동결한다.

- NVIDIA compatibility: <https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html>
- NVIDIA install guide: <https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html>
- JetPack 5.1.1 release: <https://docs.nvidia.com/jetson/jetpack/5.1.1/release-notes/index.html>

전용 NGC/L4T container는 이미지가 1 GB를 넘을 가능성이 크므로 별도 사용자 승인 후에만 받는다.

## 3. 매핑 중에 해도 되는 일

- 공식 checkpoint 다운로드와 SHA-256 계산
- 논문 commit 및 연구실 구현 pin 확인
- `pixnav_check.py` 문법 검사와 `--preflight-only`
- 저장된 RGB evidence의 파일 개수·해시 확인
- 문서와 시험표 정리

매핑 중에는 checkpoint를 GPU/CPU에 적재하거나 inference를 실행하지 않는다. `pixnav_check.py`도
mapping process를 발견하면 replay를 `BLOCKED_MAPPING_ACTIVE`로 중단한다. ROS topic, 내장 카메라,
LiDAR, Docker, VLM 서버에는 접근하지 않는다.

## 4. 매핑 복귀 직후 순서

1. `map_headless.sh` 터미널에서 `Ctrl+C`를 한 번 누른다.
2. DB와 loop evidence 저장 요약이 출력될 때까지 기다린다.
3. 새 run에서 `accepted_global`, 종단 gap, rejection 원인을 먼저 판정한다.
4. 로봇을 엎드린 standby 상태로 둔다.
5. PixNav preflight를 실행한다.

```bash
cd /home/unitree/go2_ws_antarctica
./pixnav_check.py --preflight-only
```

preflight PASS 조건:

- 연구실 구현 HEAD가 고정 pin과 일치
- checkpoint SHA-256 일치
- `torch`, `torchvision`, `cv2`, `numpy` import 가능
- 파일 전용 interlock 활성

그 다음, 이미 저장된 실제 Go2 RGB에서 VLM 선택 당시의 capture-view를 명시해 replay한다.

```bash
./pixnav_check.py --device cuda \
  --frames-dir /home/unitree/.ros/pixnav_s2e_runs/20260828_135901_pixnav_s2e_no_actuation/frames \
  --goal-frame-index 10 \
  --history-start-index 10
```

기본 `(u,v)=(640,600)`은 앞선 실 RGB/VLM 연결 시험에서 얻은 pixel을 재사용하는 **runtime
smoke input**이다. 카메라 calibration이나 목표 의미·도달 성공을 검증한 것으로 주장하지 않는다.

2026-08-28 실측 evidence:

- `152009`, `152047`, `152410` 세 run: Checkpoint_A 실제 CUDA forward와 동일 잘못된 입력에 대한
  결정론만 확인. goal/history pairing 오류 때문에 capture-view acceptance에서는 **사용 금지**.
- 수정된 v2 CUDA replay:
  `/home/unitree/.ros/pixnav_runs/20260828_162002_pixnav_file_only/report.json`
  (`PASS_FILE_ONLY_REPLAY`, 2.889 s, goal=`frame_10`, history=`frame_10` 한 장).
- file-only macro audit:
  `/home/unitree/.ros/pixnav_macro_runs/20260828_162023_pixnav_macro_file_only/`.
  출력 `look_down`은 고정 카메라에서 실행하지 않고 `reobserve + zero hold`로 차단됐다.
- offline causal chain:
  `/home/unitree/.ros/pixnav_chain_runs/20260828_162122_pixnav_offline_chain/`.
- pure/file-copy 고장 주입 22/22:
  `/home/unitree/.ros/pixnav_fault_runs/20260828_163454_pixnav_fault_injection/`.
- 최종 Jetson file-only manifest:
  `/home/unitree/.ros/pixnav_qualification_runs/20260828_163514_pixnav_qualification/`.

기존 자료에는 `frame_10` 이후 영상이 없으므로, 여러 시점 history의 정상 추론은 아직 검증하지
못했다. 이후 clip은 VLM capture-view와 그 뒤의 관측 프레임을 함께 수집해야 한다.

## 5. 검사기가 하는 일과 하지 않는 일

`pixnav_check.py`는 다음만 수행한다.

1. 논문 commit, 연구실 구현 pin, checkpoint hash를 evidence에 기록한다.
2. 실제 Go2 저장 RGB의 파일 hash를 기록한다.
3. VLM이 pixel을 선택한 정확한 capture-view에 mask를 만들고, 같은 시각 또는 이후 RGB만
   history로 허용해 224×224로 전처리한다.
4. frozen PixNav를 실행해 각 유효 history frame의 action probability, distance, tracked goal을 기록한다.
5. NaN/Inf와 shape를 검사하고 `~/.ros/pixnav_runs/<RUN_ID>/report.json`을 남긴다.

검사기에는 ROS import, socket, SDK, command publisher가 없다. 따라서 다음을 증명하지 않는다.

- 실로봇 navigation success/SPL
- RTAB-Map localization 정확도
- pixel-to-ground projection 또는 카메라 calibration
- PixNav action의 Go2 구동 정합성
- 충돌 회피, E-stop, watchdog

`PASS_FILE_ONLY_REPLAY`는 checkpoint/runtime/input contract PASS일 뿐 End-to-End PASS가 아니다.

## 6. 실제 구동 전에 남은 Gate

논문의 PixNav 목표는 **capture-view anchored pixel**이다. 로봇이 움직인 뒤에도 같은 `(u,v)`를
현재 영상에 그대로 적용하면 안 된다. capture viewpoint를 복원하거나 새 관측으로 goal을 다시
grounding해야 한다.

추가로 다음이 필요하다.

1. ~~`forward/left/right/stop`을 bounded macro-action **proposal**로 변환하는 file-only adapter~~
2. ~~`look_up/look_down`을 고정 내장 카메라에서 `reobserve + zero hold`로 차단~~
3. live RTAB-Map localization stale/lost를 실제 event chain에서 zero-hold로 연결
4. live RGB stale, VLM timeout, PixNav timeout을 watchdog과 file sink에서 검증
5. strict VLM schema와 capture 이후 history를 최소 20개 clip으로 검증
6. 단일 actuator gateway, 장애물 guard, E-stop과 실제 stop latency 검증
7. 별도 승인 후 짧은 저속 pilot

구현된 `src/escape_nav_pixnav`는 ROS/DDS/socket/Unitree SDK를 직접 import하지 않는다. 0.25 m
translation, ±30° rotation, 속도·가속도·TTL 상한을 갖는 proposal만 hash-chain JSONL에 남기며,
항상 `actuation_permitted=false`다. `frame_captured → vlm_submitted → vlm_completed →
pixnav_completed → macro_audited` 순서를 검사하는 event ledger도 구현됐지만 아직 live process와
물리 controller에는 연결하지 않았다. 단위/패키지 시험은 56개 모두 통과했다.

논문 기준 실로봇 평가는 하나의 고정 map에서 같은 start-goal pair에 대해 Direct-goal PixNav와
전체 ESCAPE-PixNav를 각각 5회 반복하고, intervention은 failure로 기록하며 RGB·trajectory·event·
video provenance를 남기는 deployment validation이다. 이 campaign은 위 Gate 이후에만 시작한다.

## 7. 가장 짧은 진행 순서

```text
느린 golden mapping 정상 종료
  → global loop/종단 gap 판정
  → PixNav checkpoint preflight
  → 저장 RGB file-only replay
  → capture-view goal 계약 확정
  → Go2 action adapter + fail-safe 무구동 검증
  → 별도 승인 후 저속 pilot
  → Direct-goal vs ESCAPE-PixNav paired campaign
```
