# 현재 상태와 다음 실행 순서 — 충전 중 Jetson–Docker–Server 점검

> 측정 시각: 2026-08-27 16:46 KST  
> 사용자 상태 전달: Go2는 충전 중이며 Jetson만 켜져 있음  
> 안전 범위: 로봇 센서/모터/`/cmd_vel`/Sport API/production bridge를 시작하지 않음

## 1. 결론

지금은 **Jetson ↔ Docker ↔ 원격 서버의 무구동 통신과 계약 시험**을 진행할 수 있다. 실제로 네트워크, 임시 양방향 UDP, text JSON 추론, 보관된 실제 Go2 카메라 이미지의 vision 추론까지 통과했다.

그러나 Docker의 실제 S2E model/checkpoint와 안전한 production command path는 없으므로, 이 결과를 4-Tier 자율주행 완료로 해석하면 안 된다.

충전 후에는 RTAB-Map을 먼저 마무리하는 방향이 맞다. 단, 현재 `mapping_gui.sh`는 아직 4DoF graph 설정이므로 곧바로 전체 최종 맵을 찍지 말고 **planar 3DoF 짧은 자격 주행 → global visual loop 검증 → 전체 맵** 순서로 진행한다.

## 2. 16:46 KST 무구동 실측 결과

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
| S2E core tests | PASS | isolated package: 43 passed |
| bringup contract tests | PASS | isolated package: 3 passed |
| live camera → VLM → S2E | NOT TESTED | 로봇 OFF이며 현재 ROS VLM node는 mock-first 구현 |
| production command path | NOT STARTED | 9090/9091 bridge와 motor sink를 실행하지 않음 |

보관 RGB 한 장의 성공은 서버가 OpenAI-compatible image payload를 받을 수 있다는 증거다. 실시간 camera timestamp, full VL-MAG schema, memory, S2E trajectory, stale response rejection 또는 navigation 품질의 증거는 아니다.

## 3. 충전 중 할 수 있는 일

### 3.1 지금 안전하게 가능한 작업

1. Docker에서 canonical S2E와 현재 local snapshot의 차이를 확정한다.
2. 실제 checkpoint/runtime 배치 위치와 SHA-256 계약을 정한다.
3. 보관된 Go2 RGB frame을 사용해 전체 VLM input/output schema를 검증한다.
4. VLM submit/complete/apply/reject identity와 image hash를 남기는 event logger를 설계한다.
5. 모터 대신 command sink를 연결해 trajectory/controller 출력을 파일로만 검증한다.
6. timeout, malformed JSON, server loss, stale response를 주입하고 항상 `stop/zero`가 되는지 확인한다.

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

로봇을 움직이지 않은 상태에서 다음을 먼저 확인한다.

- Go2 ping과 DDS discovery
- `/utlidar/cloud_deskewed`, `/utlidar/imu`, `/utlidar/robot_odom` 실제 sample
- `/livo/odom`, `/livo/imu`, `/livo/cloud`의 rate, timestamp, frame
- 카메라 image/CameraInfo 수신
- 로봇 정지 상태에서 odom jump와 IMU interpolation warning 여부

### Gate B — planar 3DoF 짧은 자격 주행

현재 launch는 다음 4DoF 값이다.

```text
Reg/Force3DoF=false
Icp/Force4DoF=true
Optimizer/Slam2D 미설정
```

이 구성은 두 번째 실주행에서 raw LIO z range 약 0.0212 m에 비해 RTAB graph z range 약 6.452 m를 만들었다. 다음 비교 profile은 아래 세 값만 바꾸는 단일-variable 묶음이다.

```text
Reg/Force3DoF=true
Icp/Force4DoF=false
Optimizer/Slam2D=true
```

아직 현재 source에는 적용하지 않았다. 별도 planar profile을 만든 뒤 짧은 동일 경로 한 바퀴로 다음을 확인한다.

- RTAB graph z가 수 cm 수준으로 제한
- raw LIO보다 endpoint gap이 악화되지 않음
- 벽 직선성과 평행성이 map2 이상
- false closure로 지도가 접히지 않음

### Gate C — global visual loop closure

1. 특징적인 시작 장면을 같은 방향으로 2~3초 관측한다.
2. 급회전과 빠른 보행을 피하며 짧은 loop를 돈다.
3. 출발점에 같은 접근 방향과 camera view로 돌아온다.
4. 같은 방향으로 2~3초 정지한다.
5. logger와 DB에서 type-1 global link를 확인한다.
6. 승인 직후 endpoint, 이중벽, map jump, z를 다시 확인한다.

type-1이 생성됐다는 사실만으로 합격하지 않는다. 올바른 장소를 연결했고 지도 오차를 줄였을 때만 합격이다.

### Gate D — 전체 맵과 localization 동결

Gate A~C 통과 후 실제 주행 구역 전체 맵을 촬영한다.

- 주요 복도·교차로·출발 구역을 재방문
- `mapping_gui.sh` wrapper의 기존 DB backup 확인
- recorder, Docker, VLM, command bridge는 계속 OFF
- 종료 후 DB/PGM/YAML/loop log hash 보존
- 동일 DB로 localization 재시작을 반복해 시작 pose 분산과 map jump 확인
- 합격한 DB, PGM/YAML, launch config, calibration, commit을 한 세트로 동결

## 5. RTAB-Map 이후 4-Tier 진입 기준

RTAB-Map 지도를 고정한 뒤 아래 순서로 진행한다.

```text
archived/live sensor replay
        → Jetson localization/camera
        → Docker VL-MAG + actual S2E
        → remote VLM
        → trajectory/controller
        → command sink
        → fault injection
        → verified safety gateway
        → supervised low-speed Go2
```

현재는 `Docker ↔ server`의 text/archived-image inference까지 확인됐다. 다음 4-Tier 목표는 **실제 S2E를 포함한 no-actuation command-sink 폐루프**이며, 로봇 주행이 아니다.

## 6. 관련 문서

- [`4-Tier 실측 감사 및 ICRA 2027 실험 프로토콜`](./master_plan/[2026-08-27]_Robot_Jetson_Docker_Server_4Tier_실측감사_및_ICRA2027_실로봇_실험프로토콜.md)
- [`RTAB-Map 문제·원인·해결·재검증 총정리`](./master_plan/[2026-08-27]_RTAB-Map_LIVO_문제_원인_해결_및_재검증_총정리.md)
- [`오늘 지도와 loop log manifest`](../2dmap/2026-08-27/MANIFEST.md)
- [`프로젝트 안전·acceptance memory`](./CODEX_PROJECT_MEMORY.md)

