# 현재 상태와 다음 실행 순서 — 4-Tier PixNav와 RTAB-Map

> 최신 측정: 2026-08-28 18:03 KST
> 실측 run: `20260828_141247_planar3dof_headless`
> 안전 범위: Docker/VLM/PixNav/L2는 read-only/file-only; recorder, 모터/`/cmd_vel`, Sport API, production bridge는 시작하지 않음

## 1. 결론

지금은 **Jetson ↔ Docker ↔ 원격 서버의 무구동 통신과 계약 시험**을 진행할 수 있다. 실제로 네트워크, 임시 양방향 UDP, text JSON 추론, 보관된 실제 Go2 카메라 이미지의 vision 추론까지 통과했다.

최신 paper branch의 실로봇 주 backend인 frozen PixNav는 Jetson CUDA에서 VLM이 실제 pixel을
고른 `frame_10`을 capture-view로 사용한 수정 v2 replay를 통과했다. 초기 11-frame 세 run은
`frame_00`을 goal로 쓴 input-pairing 오류 때문에 forward 진단 자료로만 남기고 acceptance에서는
제외한다. bounded macro proposal/file sink에 이어 실제 Go2 RGB→Docker→server VLM→Jetson
persistent PixNav→L2/odom P7 평가까지 1-cycle로 연결됐다. 다만 operator-enable, 물리 E-stop,
P8 단일 gateway와 production command path는 없으므로 이를 4-Tier 자율주행 완료로 해석하면
안 된다. S2E는 별도 NavBench-GS 보조 실험이다.

RTAB-Map 기본 운용은 **planar 3DoF로 확정한다.** 같은 층의 평평한 복도에서 3D LiDAR 입력과 3D map 생성은 유지하고, pose graph만 `x/y/yaw`로 구속한다. 4DoF는 실제 경사·고도 변화가 실험 범위일 때만 별도 DB로 비교한다.

planar 3DoF와 global visual loop의 기능 자체는 확인됐다. 그러나 최신 전체-map은 실제로 존재해야 하는 두 90도 코너가 접히고 교차해 **geometry FAIL**이다. DB 무결성과 낮은 graph residual은 "입력된 링크를 내부적으로 만족했다"는 뜻일 뿐, 링크가 실제 같은 장소를 연결했다는 보증이 아니다.

원본을 보존한 link-type ablation에서 Type-2 spatial proximity link만 제거하자 90도 코너 형태가 복구됐고, Type-1 global link만 제거한 경우에는 접힘이 남았다. 따라서 원인은 주로 기존의 공격적인 Type-2 설정(`Angle=180`, graph depth 무제한, 최대 10-neighbor)이며, canonical profile은 이제 **Type-1 visual loop 유지 + Type-2 proximity 비활성화**다. 이 DB는 원인 분석 증거로만 보존하고 localization/golden map에 사용하지 않는다.

### 1.1 최신 전체-map 재판정

| 항목 | 결과 | 판정 |
|---|---:|---|
| node / DB | 1,707 / 523,153,408 bytes, integrity `ok` | PASS |
| raw path / 평균 속도 | 358.186 m / 약 0.391 m/s | 기록 |
| optimized Z span | 0.0318 m | PASS |
| DB unique global / proximity | **36 / 159** | 기록 |
| global-link graph residual | 최대 0.0331 m / 0.171° | 내부 일관성만 확인 |
| odometry lost | 0 | PASS |
| wrapper / hash manifest | status 0 / 전체 OK | PASS |
| 동일 subset 종단 XY gap | raw 1.653 m → optimized 1.077 m | localization에서 재확인 |
| 물리적 두 90도 코너 | 접힘·교차로 불일치 | **FAIL** |
| Type-2 제거 offline ablation | 코너/직교 형상 복구 | Type-2가 주원인 |
| golden DB / localization | 사용 금지 | **FAIL** |

상세 근거는 [`troubleshooting/06_rtabmap_livo_2026-08-27_runtime_diagnosis_and_loop_closure_log.md`](troubleshooting/06_rtabmap_livo_2026-08-27_runtime_diagnosis_and_loop_closure_log.md)의 20~21절을 따른다.

### 1.2 12:46 짧은 자격 run 판정

| 항목 | 최신 결과 | 판정 |
|---|---:|---|
| node / DB | 249 nodes, 73.9 MB, integrity `ok` | PASS |
| optimized pose / 경로 | 164 poses / 41.04 m | PASS |
| optimized Z span | 0.0235 m | PASS |
| 출발–복귀 오차 | 0.0335 m | PASS |
| LiDAR proximity closure (Type-2) | logger 8회, DB unique link 9개 | 기록; canonical에서는 OFF |
| global visual closure (Type-1) | **2회: 174→61, 211→1** | **PASS** |
| 잘못된 후보 | 6회 모두 graph 삽입 전 reject | PASS |
| odometry lost / optimizer failure / NaN | 0 / 0 / 0 | PASS |
| DB 저장 / 최적화 재로딩 | saved=true / 164 poses | PASS |
| wrapper 종료 코드 | 141 (`Ctrl+C` 때 `tee`가 먼저 닫힌 logging bug) | 코드 수정, 다음 run 확인 |

마지막 `211→1`은 visual score `0.8717`, DB의 기하 보정 0.030 m이며 최적화 후 출발–복귀 오차가 3.35 cm다. `174→61`도 0.189 m 범위의 실제 재방문 링크다. 위에서 본 3D 점군/궤적에는 그래프 접힘이나 비연속 jump가 보이지 않았다. 따라서 **global loop 기능은 통과**했지만, 이 DB는 짧은 자격 코스이므로 최종 localization용 골든 맵으로 쓰지 않는다.

`map_headless.sh`는 전체 맵 전에 다음처럼 보강했다.

- `tee --ignore-interrupts`로 inner RTAB-Map cleanup 로그와 DB 종료를 끝까지 수집
- 정상적인 operator `Ctrl+C`(status 130)를 established run에서 status 0으로 기록
- Jetson에 `sqlite3` CLI가 없어도 Python 표준 sqlite3로 `database_integrity.txt` 생성

## 2. 16:46 KST 무구동 실측 결과

4-Tier의 최신 가중 평가는 Robot 58%, Jetson 65%, Docker 45%, Server 84%, 실제 cross-tier
End-to-End 64%다. 이는 live safe file sink까지의 배치 readiness이며 물리 자율주행률이 아니다.
상세 점수와 `status/full` 재검증 명령은
[`Robot–Jetson–Docker–Server 4-Tier 구현률`](./experiments/07_4tier_robot_jetson_docker_server_readiness.md)을
따른다. 이 퍼센트는 engineering readiness estimate이며 논문 성능 지표가 아니다.

| 항목 | 결과 | 증거와 해석 |
|---|---|---|
| Jetson 자원 | PASS | 15 GiB RAM 중 약 11 GiB available, NVMe 약 388 GiB available |
| host services | PASS | Docker, NetBird, NetworkManager 모두 `active` |
| Docker lifecycle | PARTIAL | `sdam_go2_container` 실행 중이나 실제 프로세스는 `tail -f /dev/null`뿐 |
| Jazzy package | PRESENT | `s2e_vlm_{bringup,core,msgs,nodes}`와 10개 node executable 존재 |
| 실제 S2E entrypoint | FAIL | `s2e-vlm-async-framework/src/vlm_s2e_async_node.py` 없음 |
| 실제 S2E checkpoint | FAIL | `/models/s2e/S2E/s2e.onnx` 없음 |
| Jetson → Docker UDP | PASS | 비제어 임시 port 19091에서 `ESCAPE_NAV_NO_ACTUATION` 수신 |
| Docker → Jetson UDP | PASS | 비제어 임시 port 19090에서 같은 payload 수신 |
| Jetson → server | PASS | `GET /v1/models` HTTP 성공 |
| Docker → server | PASS | `GET /v1/models` HTTP 성공 |
| advertised model | MEASURED | `qwen3.5-9b-instruct`, root `AxionML/Qwen3.5-9B-NVFP4` |
| text JSON contract | PASS | `status=ok`, `action=stop`, `source=docker-server-preflight` 반환 |
| archived RGB vision | PASS/제한적 | `scratch/live_camera_snapshot.jpg`에서 가까운 물체를 `office chair`로 반환하고 `action=stop` 유지 |
| frozen PixNav CUDA replay | PASS/제한적 | Checkpoint_A persistent runtime 실제 5-frame CUDA 0.106 s, finite, actuation 0; 20 clips/soak은 없음 |
| PixNav file adapter | PASS/제한적 | bounded proposal, hash-chain JSONL, causal ledger, 72 tests; 핵심 package에 ROS/socket/SDK/actuation 권한 없음 |
| offline causal/fault | PASS/제한적 | VLM→PixNav→macro identity PASS, pure/file-copy fault 22/22; live timeout·physical stop latency 증거 아님 |
| 현재 유선/서버 경로 | PASS | wlan0 disconnected, eth0 학교망+Go2 직결망, NetBird active, server HTTP 200(0.051 s) |
| S2E core tests | PASS | isolated package: 43 passed |
| bringup contract tests | PASS | isolated package: 3 passed |
| live camera acquisition | PASS | 정상 기립·정지 Go2에서 RTP→`/camera/front/image_raw` 1280×720 BGR8, 실수신 14.33 Hz |
| live camera → PixNav P6 | 1-CYCLE PASS | 실제 RGB→Docker→server VLM→persistent CUDA PixNav→file sink, 최신 P7 source age 0.271 s; 10분 soak pending |
| live L2/odom P7 | PARTIAL PASS | read-only L2/odom clearance/freshness 실평가 PASS; operator-enable/E-stop/P8 미연결로 gateway=false |
| production command path | NOT STARTED | 9090/9091 bridge와 motor sink를 실행하지 않음 |

보관 RGB 한 장의 성공은 서버가 OpenAI-compatible image payload를 받을 수 있다는 증거다. 실시간 camera timestamp, full VL-MAG schema, memory, S2E trajectory, stale response rejection 또는 navigation 품질의 증거는 아니다.

### 2.1 17:19 정상 기립·정지 실센서 preflight

사용자가 Go2를 정상 기립 상태로 두고 움직이지 않은 조건에서 command publisher, SDK client,
mapping, Docker controller를 시작하지 않고 실제 입력만 구독했다.

| 입력/변환 | 6초 실측 | 판정 |
|---|---:|---|
| Go2 mainboard | ping 3/3, 평균 0.199 ms | PASS |
| `/utlidar/cloud_deskewed` | 15.49 Hz, 11,461 records/frame | PASS |
| `/utlidar/imu` | 247.03 Hz, accel norm 9.715 m/s² | PASS |
| `/utlidar/robot_odom` | 149.92 Hz, `odom→base_link`, Z=0.3159 m | PASS |
| RTP→`/camera/front/image_raw` | 14.33 Hz, 1280×720 BGR8 | PASS |
| `go2_livo_sensor_bridge.py` | cloud rx/pub/drop=77/77/0 | PASS |
| IMU order correction | `wxyz`, gravity residual 0.12° | PASS |
| valid cloud records | 12.0% after finite/non-zero filtering | transport PASS; mapping geometry에서 재평가 |

재사용 가능한 읽기 전용 프로브는 `scratch/probe_live_sensors_no_actuation.py`다. 측정 종료 후
camera/LIVO probe, RTAB-Map, host command bridge, PixNav/S2E live process가 남아 있지 않음을 확인했다.
Foxy의 `ros2 topic info --verbose`는 최신 Unitree DDS type-hash를 XML-RPC로 표시하는 과정에서
호환 예외가 발생했지만, 각 토픽 publisher 1개와 위 실제 sample 수신은 별도로 확인됐다.

### 2.2 18:03 live PixNav + L2/odom P7 무구동 결합

`pixnav_live_check.py`를 정상 기립·정지 Go2에서 실행했고 ROS publisher, Unitree SDK client,
command UDP sender와 controller를 한 개도 만들지 않았다.

| 항목 | 실측 | 판정 |
|---|---:|---|
| Docker→remote VLM | confidence 0.95, 1.484 s, strict pixel 계약 | PASS |
| persistent PixNav | live CUDA 0.090 s | PASS |
| 최종 관측 / P7 age | 0.267 / 0.271 s (`source TTL=1.0 s`) | PASS |
| L2/odom | 1,524 valid points, nearest stamp delta 0.002 s | PASS |
| clearance | 전방 0.921 m, 회전 0.532 m | sensor gate PASS |
| PixNav output | `look_down`, probability 0.892 | fixed camera `reobserve`, 이동 0 |
| P7 최종 | operator-enable=false, E-stop clear 미연결 | gateway candidate=false, fail-closed |
| artifact | `~/.ros/pixnav_live_runs/20260828_180327_pixnav_live_no_actuation/` | nearest-stamp P7 + source manifest + SHA256 전부 OK |

따라서 현재 빠른 우선순위는 RTAB remap이 아니라 P6 10분 soak/fault와 P8 이전의 물리
operator-enable/E-stop 계약이다. RTAB 골든맵·localization은 실제 이동량/궤적 평가를 시작하기
전에 다시 합류시키면 된다.

## 3. 충전 중 할 수 있는 일

### 3.1 지금 안전하게 가능한 작업

1. ~~최신 `paper` commit과 연구실 PixNav 구현 pin 확정~~ — 완료.
2. ~~공식 PixNav Checkpoint_A 배치와 SHA-256 계약~~ — 완료.
3. ~~VLM capture-view를 정확히 사용한 v2 1-step CUDA replay~~ — 완료.
4. ~~bounded macro proposal, hash-chain sink, causal ledger, pure fault harness~~ — 72 tests와 22/22 완료.
5. ~~checkpoint/reference/adapter/evidence file-only manifest 동결~~ — 완료.
6. ~~live RGB→Docker VLM→persistent PixNav→file sink 1-cycle 연결~~ — 완료.
7. ~~L2/odom read-only clearance/freshness를 P7 입구에 연결~~ — 완료.
8. 10분 P6 soak과 live timeout/server loss/camera·odom stale/process kill을 주입한다.
9. 물리 operator-enable/E-stop 입력과 P8 single gateway를 별도 승인 후 구현한다.
10. capture 이후 history가 있는 서로 다른 실제 RGB clip을 최소 20개로 늘린다.

### 3.2 반복 가능한 읽기 전용 확인 명령

```bash
cd /home/unitree/go2_ws_antarctica

docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
docker top sdam_go2_container -eo pid,ppid,comm,args

curl --connect-timeout 3 --max-time 8 \
  http://100.96.60.15:8000/v1/models

docker exec sdam_go2_container bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /workspace/go2_ws_antarctica/s2e-vlm-async-framework/install/setup.bash
  ros2 pkg executables | grep "^s2e_vlm_" | sort
'
```

위 명령은 상태와 모델 목록을 읽을 뿐이다. `scratch/check_docker_status_dashboard.py`와 `scratch/test_all_docker_server_pipeline.py`는 synthetic fallback과 hard-coded PASS 값이 포함되어 있으므로 acceptance 도구로 사용하지 않는다.

### 3.3 충전 중 실행하지 않을 것

- `scratch/bringup_all_escape_nav.sh` autonomy mode
- `scratch/host_bridge.py` 또는 `scratch/docker_bridge.py`의 production 9090/9091 경로
- `/cmd_vel` publisher 또는 Unitree Sport motion API
- motor가 연결될 수 있는 controller launch
- mock/synthetic 결과를 real 4-Tier PASS로 승격

## 4. 충전 후 RTAB-Map 실행 순서

### Gate A — 전원 후 센서 preflight

**2026-08-28 17:19 정상 기립·정지 조건 PASS.** 아래 입력과 변환은 모두 실수신됐다. 다만
이 합격은 점군 전송/시간/프레임 파이프라인 판정이며 코너 형상과 map 정확도 판정은 Gate B에서 한다.

로봇을 움직이지 않은 상태에서 다음을 먼저 확인한다.

- Go2 ping과 DDS discovery
- `/utlidar/cloud_deskewed`, `/utlidar/imu`, `/utlidar/robot_odom` 실제 sample
- `/livo/odom`, `/livo/imu`, `/livo/cloud`의 rate, timestamp, frame
- 카메라 image/CameraInfo 수신
- 로봇 정지 상태에서 odom jump와 IMU interpolation warning 여부

### Gate B — planar 3DoF 짧은 자격 주행

4DoF 비교 구성은 raw LIO Z range 약 0.0212 m에 비해 RTAB graph Z range 약 6.452 m를 만들었다. 따라서 평면 환경의 canonical profile은 다음이다.

```text
Reg/Force3DoF=true
Icp/Force4DoF=false
RGBD/NeighborLinkRefining=false
RGBD/LoopClosureIdentityGuess=true
```

삭제된 기존 wrapper가 표시하던 `Optimizer/Slam2D`는 설치된 RTAB-Map 0.21.1에서 독립적인 canonical 설정으로 사용하지 않는다. 현재 두 진입점과 manifest에는 이 이름을 제거했으며 실제 3DoF 판정은 `Reg/Force3DoF=true`와 `Icp/Force4DoF=false`로 한다.

최신 planar 주행에서 Z 발산과 odometry loss는 없었지만, aggressive Type-2 proximity link가 지도 전체를 물리적으로 잘못 접었다. 다음 자격 조건은 proximity를 끈 상태에서 두 90도 코너가 보존되고 올바른 Type-1 global loop만 승인되는지 확인하는 것이다.

- RTAB graph z가 수 cm 수준으로 제한
- raw LIO보다 endpoint gap이 악화되지 않음
- 벽 직선성과 평행성이 map2 이상
- false closure로 지도가 접히지 않음

실행:

```bash
cd /home/unitree/go2_ws_antarctica
./map_headless.sh
```

별도 `230.0.0.0/8` privileged route는 요구하지 않는다. CycloneDDS는 `192.168.123.99/eth0`에 고정되고 RTP camera도 `multicast-iface=eth0`를 사용한다. bringup은 권한 변경 대신 `192.168.123.161`의 direct eth0 route만 읽기 전용으로 검사한다.

시작 화면에 반드시 다음이 보여야 한다.

```text
profile=planar3dof
Reg/Force3DoF=true
Icp/Force4DoF=false
Recorder=false
Docker/motor=false
```

복귀 후 `Ctrl+C`로 한 번 정상 종료한다. 실행별 증거는 아래에 묶인다.

```text
/home/unitree/.ros/rtabmap_runs/<timestamp>_planar3dof_headless/
├── run_manifest.txt
├── runtime.log
├── rtabmap.db
├── SHA256SUMS
├── git_status.txt
├── config/
└── loop_logs/
```

가장 최근 실행은 `/home/unitree/.ros/rtabmap_runs/latest` symlink로 찾는다.

### Gate C — global visual loop closure

이전 run의 기본 PnP 경로는 단안 RGB에 metric depth가 없어 Type-1을 만들지 못했다. `RGBD/LoopClosureIdentityGuess=true`로 RGB 후보를 만들고 3D LiDAR ICP가 검증하는 경로는 짧은 run에서 Type-1 2개, 최신 전체 run에서 고유 Type-1 36개를 남겼다. 다만 link 수가 많다고 정확한 것은 아니다. 이제 Type-2를 끈 짧은 코스에서 올바른 Type-1과 코너 형상을 함께 다시 확인한다.

1. 특징적인 시작 장면을 같은 방향으로 3~5초 관측한다.
2. 급회전과 빠른 보행을 피하며 짧은 loop를 돈다.
3. 출발점에 같은 접근 방향과 camera view로 돌아온다.
4. 같은 방향으로 3~5초 정지한다.
5. logger와 DB에서 type-1 global link를 확인한다.
6. 승인 직후 endpoint, 이중벽, map jump, z를 다시 확인한다.

type-1이 생성됐다는 사실만으로 합격하지 않는다. 올바른 장소를 연결했고 지도 오차를 줄였을 때만 합격이다.

### Gate D — 전체 맵과 localization 동결

최신 전체-map은 geometry FAIL로 폐기하지 않고 분석 증거로만 보존한다. Type-2 OFF 짧은 자격 run이 통과한 뒤 한 번만 전체 맵을 다시 촬영한다.

- 먼저 두 90도 코너를 포함한 1~2분 짧은 loop에서 접힘 없음과 올바른 Type-1을 확인
- 통과할 때만 주요 복도·교차로·출발 구역 전체를 재촬영
- `map_headless.sh` wrapper와 기존 DB backup 확인
- recorder, Docker, VLM, command bridge는 계속 OFF
- 종료 후 DB/PGM/YAML/loop log hash 보존
- wrapper가 status 0 또는 의도된 130으로 종료되고 DB에 optimized pose가 저장됐는지 확인
- 동일 DB로 localization 재시작을 반복해 시작 pose 분산과 map jump 확인
- 합격한 DB, PGM/YAML, launch config, calibration, commit을 한 세트로 동결

## 5. RTAB-Map 이후 4-Tier 진입 기준

RTAB-Map localization만 켠다고 자율주행이 되지는 않는다. Nav2는 planner/controller를 제공하는 한 선택지지만 최신 paper branch의 `Direct-goal PixNav vs Full ESCAPE-PixNav` main 비교에서는 같은 frozen PixNav backend를 사용하므로 필수가 아니다. RTAB-Map은 map-frame pose와 재지역화를 제공하고, 실제 이동은 검증된 PixNav action adapter/controller/safety gateway가 담당한다. S2E는 별도 NavBench-GS 보조 실험이며 현재 실로봇 backend Gate가 아니다. Nav2는 선택적인 classic baseline 또는 별도 waypoint demo로만 추가한다.

RTAB-Map 지도를 고정한 뒤 아래 순서로 진행한다.

```text
archived/live sensor replay
        → Jetson localization/camera
        → Docker VL-MAG + frozen PixNav
        → remote VLM
        → trajectory/controller
        → command sink
        → fault injection
        → verified safety gateway
        → supervised low-speed Go2
```

현재는 `Docker ↔ server`의 text/archived-image inference와 offline VLM→PixNav→macro artifact
연결까지 확인됐다. 다음 4-Tier 목표는 **live capture와 localization을 포함한 10분 no-actuation
command-sink 폐루프**이며, 로봇 주행이 아니다.

## 6. 관련 문서

- [`실로봇 전체 E2E 검증 및 논문 campaign 계획`](./experiments/00_real_robot_end_to_end_master_test_plan.md)
- [`4-Tier 실측 감사 및 ICRA 2027 실험 프로토콜`](./master_plan/[2026-08-27]_Robot_Jetson_Docker_Server_4Tier_실측감사_및_ICRA2027_실로봇_실험프로토콜.md)
- [`RTAB-Map 문제·원인·해결·재검증 총정리`](./master_plan/[2026-08-27]_RTAB-Map_LIVO_문제_원인_해결_및_재검증_총정리.md)
- [`오늘 지도와 loop log manifest`](../2dmap/2026-08-27/MANIFEST.md)
- [`프로젝트 안전·acceptance memory`](./CODEX_PROJECT_MEMORY.md)
