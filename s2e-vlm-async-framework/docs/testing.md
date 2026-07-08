# Testing Strategy

## Goals

Testing must prove asynchronous behavior, not only node startup. A passing system demonstrates correct timestamp propagation, bounded pose lookup, latest-message caching, VLM/e2e decoupling, rotate preemption, stale-data degradation, and controller fail-closed behavior.

## Test Layers

### Unit Tests

Unit tests run with `pytest` and no ROS graph when possible.

Required unit test groups:

- Timestamp helpers: compare `header.stamp`, `source_stamp`, and `processed_stamp` semantics.
- Pose buffer: latest-pose-before-target lookup, interpolation when available, max-age rejection.
- VLM parser: valid `go`, `stop`, and `rotate`; malformed JSON; missing fields; invalid action.
- Goal compensation: relative transform from VLM pose to current pose and max distance/yaw rejection.
- Trajectory validation: exactly 10 points, finite values, `base_link` frame.
- Visualizer overlay helpers: coarse `goal_uv` parsing, fine goal/trajectory mini-map scaling, stale-label formatting, and malformed VLM display behavior.
- Controller state machine: trajectory clear on rotate, stale odometry abort, timeout, cancel.

### ROS 2 Launch Tests

Launch tests start mock nodes as separate processes and validate topic/action behavior.

Required launch tests:

- Single-PC happy path: sensor mocks -> odometry mock -> VLM mock -> e2e mock -> controller mock.
- VLM slower than e2e: e2e reuses cached VLM reasoning within TTL.
- First VLM delayed: e2e stays in `WAITING_FIRST_VLM` and publishes no trajectory.
- Pose delayed beyond max age: VLM/e2e drop or degrade instead of using stale pose.
- Rotate during trajectory following: controller clears trajectory and action owns controller.
- Debug visualization: annotated image publishes while VLM/e2e/controller debug fields update asynchronously.
- Malformed VLM string: e2e rejects it and controller does not move from that command.
- Malformed VLM visualization: visualizer labels the payload invalid without crashing or producing control output.
- Heartbeat loss: supervisor marks unhealthy and controller stops/holds if safety-critical.
- Supervisor heartbeat: `/s2e/status/supervisor_node` publishes at 1 Hz and is visible to diagnostics.
- Supervisor health: `/s2e/supervisor/health` publishes `ok_to_move=false` when critical local or remote nodes are missing.
- QoS compatibility: every required subscriber discovers its publisher and `ros2 topic info --verbose` shows compatible QoS.

### Dummy Integration and State Coverage Tests

The pure-Python dummy integration harness runs before full ROS graph tests so node interface and state-machine coverage can be checked without hardware, real model files, or host ROS installation. It creates deterministic dummy messages for every declared `NodeContract` publisher, routes the latest message to every declared subscriber, records rotate action-client/server activity, and advances each node at a different configured period for an arbitrary trial count.

Current automated coverage command:

```bash
python3 -m unittest src/s2e_vlm_nodes/test/test_dummy_integration.py -v
```

The test uses 9 trials to cover these scenario classes at least once: waiting for first VLM, normal go, stale inputs, faults, VLM stop, supervisor blocked, rotate, visualizer degraded, and odometry waiting for inputs. Passing means every documented publish/subscribe/action interface observed dummy traffic and every documented state in the static TF, sensor, odometry, VLM, e2e, controller, and debug visualizer state tables was observed. `supervisor_node` is covered with `INIT`, `ACTIVE`, `DEGRADED`, and `FAULT` dummy health states because the architecture document defines supervisor health semantics but not a separate supervisor state table.

### Real ROS Mock Graph Tests

The executable ROS integration tests run inside the ROS Docker image because the host development environment is not required to have ROS installed. These tests start the node entrypoints as separate `ros2 run s2e_vlm_nodes <node>` processes and verify real DDS topic traffic plus the `/s2e/controller/rotate` action path.

Current automated command:

```bash
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && colcon test --packages-select s2e_vlm_nodes --event-handlers console_direct+ && colcon test-result --verbose"
```

Covered sequence behavior:

- Normal async flow: independent lidar, camera, IMU, odometry, VLM, e2e, controller, supervisor, and debug visualizer processes exchange the documented ROS messages.
- Cached VLM reuse: e2e publishes multiple `Trajectory2D` messages per slower VLM reasoning message while data is fresh.
- Rotate action flow: `/s2e/controller/rotate` accepts a goal, controller publishes `ROTATING` status, feedback/result complete, and trajectory following is preempted.
- VLM-triggered rotate action flow: a VLM `rotate` reasoning string causes `vlm_node` to send the controller action goal, enter `FROZEN_ROTATING`, block e2e trajectory generation with `ROTATE_IN_PROGRESS`, and return to normal after the result.
- Stop and malformed VLM flow: e2e publishes `STOPPED_BY_VLM` and `INVALID_VLM` status on real ROS topics, and the controller publishes zero hold commands.
- Watchdog failure flow: terminating `vlm_node` causes `supervisor_node` to publish `ok_to_move=false` with `vlm_node` missing, and the controller holds zero command.
- Debug visualizer flow: camera frames produce `/s2e/debug/visualizer/image` without modifying raw camera input or publishing motion commands.

Artifact-producing visualizer runs can save annotated PNG frames and an MP4 video into the current repository through the Docker artifact mount. Compose maps `${S2E_ARTIFACT_DIR:-./artifacts}` on the host to `/artifacts` in the container, and `S2E_TEST_ARTIFACT_DIR` defaults to `/artifacts/visualizer`.

Example direct Docker command:

```bash
mkdir -p artifacts/visualizer_run
docker run --rm \
  -e S2E_TEST_ARTIFACT_DIR=/artifacts/visualizer_run \
  -e S2E_MOCK_ARTIFACT_DURATION_S=10.0 \
  -v "$PWD/artifacts:/artifacts" \
  s2e-ros-base:latest \
  bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && cd /workspace/src/s2e_vlm_nodes && python3 -m unittest test.test_ros_mock_graph.RosMockGraphTest.test_visualizer_saves_png_sequence_and_mp4_from_smooth_goal_run -v"
```

Expected files:

- `artifacts/visualizer_run/frame_0000.png`, `frame_0001.png`, ... annotated RGB frames.
- `artifacts/visualizer_run/visualizer.mp4`, encoded as H.264 / `yuv420p` when `ffmpeg` is available.
- `artifacts/visualizer_run/manifest.json` with frame count, dimensions, encoding, completion status, video codec fields, `projection_available`, `projected_trajectory_frames`, and projected trajectory point counts.

### Docker/Compose QA

Copy the example environment once per machine:

```bash
cp .env.example .env
```

For a single-PC full mock stack:

```bash
docker compose build ros-base dev-mock
docker compose --profile single_pc_mock up dev-mock
```

For a single-PC split-container rehearsal:

```bash
docker compose build ros-base onnx-runtime-base
docker compose build robot-core vlm e2e
docker compose --profile single_pc_split up robot-core vlm e2e
```

For two machines, run this on the robot computer:

```bash
docker compose build ros-base robot-core
docker compose --profile robot_side up robot-core
```

Run this on the external GPU PC:

```bash
docker compose build ros-base onnx-runtime-base
docker compose build vlm e2e
docker compose --profile external_gpu up vlm e2e
```

For three machines, keep the same robot command and split the GPU services across PC1 and PC2.

On VLM PC1:

```bash
docker compose build ros-base
docker compose build vlm
docker compose --profile vlm_only up vlm
```

On e2e PC2:

```bash
docker compose build onnx-runtime-base
docker compose build e2e
docker compose --profile e2e_only up e2e
```

Same-host three-way rehearsal uses the official role profiles together:

```bash
docker compose build ros-base onnx-runtime-base
docker compose build robot-core vlm e2e
docker compose --profile robot_side --profile vlm_only --profile e2e_only up robot-core vlm e2e
```

The shared `.env` values that must match across robot, external PC, VLM PC, and e2e PC are `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `ROS_LOCALHOST_ONLY=0`. The e2e ONNX service also uses `GPU_DEVICE_ID`, `S2E_NAV_MODEL_ZOO_DIR`, and `E2E_MODEL_PATH`. The VLM service is API-only and uses `VLM_API_URL`, `VLM_API_TIMEOUT_S`, and `VLM_API_MAX_RETRIES`. Visualizer artifacts use `S2E_ARTIFACT_DIR`, `S2E_TEST_ARTIFACT_DIR`, `S2E_MOCK_ARTIFACT_DURATION_S`, and `S2E_MOCK_ARTIFACT_SAVE_PERIOD_S`.

Set `S2E_DEBUG_MODE=1` to include a diagnostics panel in `/s2e/debug/visualizer/image` and runtime debug snapshots in `manifest.json`. Compose sets `S2E_RUNTIME_ROLE` for each service; native launches can export it manually, for example `export S2E_RUNTIME_ROLE=robot_side`.

Compose explicitly passes mock runtime parameters from `.env` into every service that runs the mock runtime. Use `S2E_MOCK_VLM_PERIOD_S`, `S2E_MOCK_E2E_PERIOD_S`, `S2E_MOCK_DEBUG_VISUALIZER_PERIOD_S`, `S2E_MOCK_CAMERA_PERIOD_S`, and related `S2E_MOCK_*` values to keep single-PC and split-container tests on the same timing cadence. This is required because Docker Compose uses `.env` for interpolation but does not automatically inject arbitrary `.env` keys into containers.

### Real-Model Smoke Tests

Download the S2E assets into the gitignored local model directory:

```bash
docker run --rm -v "$PWD:/work" -w /work python:3.12-slim \
  bash -lc "python -m pip install --no-cache-dir -q huggingface_hub && \
  hf download UCLA-VAIL/Navigation-Model-Zoo-Public --include 'S2E/*' --local-dir ./nav_model_zoo && \
  chown -R $(id -u):$(id -g) ./nav_model_zoo"
```

After building `s2e-e2e`, verify ONNXRuntime GPU providers and the mounted S2E model. The runtime path imports `onnxruntime`, not the downloaded `S2E.inference` wrapper, so PyTorch is not required in the e2e image:

```bash
docker run --rm --gpus all --entrypoint /bin/bash \
  -v "$PWD/nav_model_zoo:/models/s2e:ro" \
  s2e-e2e:latest -lc "python3 - <<'PY'
import numpy as np
from s2e_vlm_core.s2e_backend import OnnxS2ENavigator
nav = OnnxS2ENavigator('/models/s2e/S2E/s2e.onnx', device='cuda')
obs = np.random.rand(1, 11, 3, 256, 256).astype(np.float32)
traj, scores = nav.inference_trajectory(obs, goal_xy=np.array([5.0, 0.0], dtype=np.float32))
print(nav._session.get_providers())
print(traj.shape, scores.shape, traj.dtype, np.isfinite(traj).all())
PY"
```

The dummy white camera image is acceptable for integration smoke tests: S2E should load, run, return finite 10x2 points, publish a trajectory, and appear in debug visualizer artifacts. Do not treat white-image output as driving-quality validation. Use recorded or realistic RGB frame sequences for semantic trajectory quality checks.

### Manual QA

Manual QA uses ROS CLI commands after implementation.

When ROS 2 and `colcon` are unavailable on the host, use the pure-Python verification suite as the local fallback. It validates core timing, parsing, mock graph behavior, launch contracts, and Docker/Compose assets without starting a ROS graph:

```bash
python3 -m unittest discover -s src/s2e_vlm_core/test -p "test_*.py" -v
python3 -m unittest discover -s src/s2e_vlm_nodes/test -p "test_*.py" -v
python3 -m unittest src/s2e_vlm_bringup/test_launch_contracts.py -v
python3 -m unittest tests/test_docker_assets.py -v
```

Build and source:

```bash
colcon build --symlink-install
source install/setup.bash
```

Run single-PC mock graph:

```bash
ros2 launch s2e_vlm_bringup single_pc_mock.launch.py
```

Expected behavior:

- Sensor topics publish at different configured rates.
- `/tf_static` publishes `base_link -> camera`, `base_link -> lidar`, and `base_link -> imu` from `config/sensors/*.yaml`.
- `/s2e/sensors/camera/camera_info` uses camera intrinsics from `config/sensors/camera.yaml`.
- `/s2e/odometry/pose` publishes `base_link` pose.
- `/s2e/vlm/reasoning` publishes strict JSON strings at slower cadence.
- `/s2e/e2e/trajectory` publishes only after first valid VLM `go` reasoning.
- `/s2e/controller/status` reports `FOLLOWING` after valid trajectory and `ROTATING` during rotate action.
- `/s2e/debug/visualizer/image` publishes annotated images with VLM action, goal markers, camera-projected trajectory when TF is available, corrected trajectory mini-map, controller mode, and rotate progress.

Inspect topics:

```bash
ros2 topic hz /s2e/sensors/camera/image
ros2 topic hz /s2e/odometry/pose
ros2 topic echo /s2e/vlm/reasoning --once
ros2 topic echo /s2e/e2e/trajectory --once
ros2 topic echo /s2e/controller/status --once
ros2 topic echo /s2e/supervisor/health --once
ros2 topic hz /s2e/debug/visualizer/image
ros2 topic echo /tf_static --once
```

Inspect QoS compatibility:

```bash
ros2 topic info --verbose /s2e/sensors/camera/image
ros2 topic info --verbose /s2e/odometry/pose
ros2 topic info --verbose /s2e/e2e/trajectory
ros2 topic info --verbose /s2e/debug/visualizer/image
```

Expected result:

- Each topic shows at least one publisher and expected subscribers.
- Sensor topics use best-effort sensor data QoS.
- Pose, VLM reasoning, trajectory, status, debug overlay, and action-related channels use the QoS specified in [interfaces.md](interfaces.md).

Inspect rotate action:

```bash
ros2 action list
ros2 action info /s2e/controller/rotate
```

Send a mock rotate goal after implementation:

```bash
ros2 action send_goal /s2e/controller/rotate s2e_vlm_msgs/action/Rotate "{target_yaw_delta_deg: 30.0, max_yaw_rate_deg_s: 30.0, tolerance_deg: 3.0, timeout_s: 5.0}" --feedback
```

Expected result:

- Feedback shows increasing `current_yaw_delta_deg` and decreasing `remaining_deg`.
- Controller status enters `ROTATING`.
- Debug overlay shows `ROTATING`, current yaw delta, remaining yaw, and final result after completion.
- Existing trajectory is cleared.
- Result returns `success=true` in mock mode unless timeout is intentionally configured too low.

## Mock Data Defaults

Use deterministic mock publishers so tests are reproducible.

| Node | Default Rate | Mock Output |
| --- | --- | --- |
| `static_tf_node` | Static + 1 Hz heartbeat | Static sensor extrinsics from `config/sensors/*.yaml` on `/tf_static` |
| `lidar_node` | 5 Hz | Synthetic point cloud grid with moving obstacle marker |
| `camera_node` | 10 Hz | RGB image with frame counter and simple horizon/goal marker |
| `imu_node` | 100 Hz | Constant gravity and slowly changing yaw rate |
| `odometry_node` | 50 Hz | Smooth planar pose from mock motion model |
| `vlm_node` | 0.5 Hz | Alternates `go`, `stop`, and configurable `rotate` scenarios |
| `e2e_node` | 5 Hz | Deterministic 10x2 trajectory toward compensated goal |
| `controller_node` | 50 Hz | Mock `Twist` command and status, no hardware command |
| `supervisor_node` | 1 Hz heartbeat checks | Aggregated node health and degraded status when heartbeats are missed |
| `debug_visualizer_node` | Camera-driven | Annotated camera image with VLM text, coarse/fine goals, trajectory, status, and rotate progress |

## Failure Injection Scenarios

Each failure injection must have an observable safe result.

| Scenario | Injection | Expected Safe Result |
| --- | --- | --- |
| Stale pose | Pause odometry for >0.20 s | VLM/e2e degrade; controller stops if pose >0.10 s |
| Missing first VLM | Delay VLM startup | e2e publishes no trajectory |
| Malformed VLM | Publish invalid JSON | e2e rejects, no new motion |
| Malformed VLM overlay | Publish invalid JSON while camera continues | visualizer displays invalid payload and keeps publishing overlay |
| Old VLM with large pose delta | Move mock pose >1.5 m from VLM pose | e2e degrades and waits for new VLM |
| VLM heartbeat lost with cached reasoning | Stop VLM after valid `go` | supervisor health blocks motion and e2e stops producing new trajectories from cached VLM |
| Rotate while following | Send rotate goal during `FOLLOWING` | controller clears trajectory and enters `ROTATING` |
| Rotate timeout | Set low timeout | action returns failure and controller stops |
| Frame mismatch | Publish trajectory frame not `base_link` | controller rejects trajectory |
| External PC heartbeat loss | Stop VLM/e2e mocks | robot-side controller holds safe command |
| Supervisor heartbeat visible | Run any launch profile | `/s2e/status/supervisor_node` publishes at 1 Hz |
| Network partition to external PC | Block or stop external VLM/e2e heartbeats | robot-side supervisor marks external compute unhealthy and controller stops after trajectory TTL |
| Unauthorized DDS participant | Publish from non-allowlisted host in real-robot profile | deployment is rejected by network/DDS security configuration before robot commands are enabled |
| QoS mismatch | Launch a subscriber with incompatible QoS | test fails and docs require compatible QoS config |

## Multi-Machine Smoke Test

Before real hardware commands are enabled, run this smoke test with robot-side and external-PC launch files.

On both machines:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
source install/setup.bash
```

On the robot computer:

```bash
ros2 launch s2e_vlm_bringup robot_side.launch.py use_mock_hardware:=true
```

On the external PC:

```bash
ros2 launch s2e_vlm_bringup external_pc.launch.py use_mock_models:=true
```

Expected result:

- External PC can echo `/s2e/odometry/pose`.
- Robot computer can echo `/s2e/e2e/trajectory`.
- Robot computer can echo `/s2e/status/vlm_node` and `/s2e/status/e2e_node` from the external PC.
- Robot computer publishes `/s2e/status/supervisor_node` and `/s2e/supervisor/health`; health reports `ok_to_move=false` if remote heartbeats stop.
- External PC can send `/s2e/controller/rotate` action goal and receive a result.
- A debug host can subscribe to `/s2e/debug/visualizer/image` when the visualizer is enabled on either side of the split.
- If the external PC launch is stopped, robot-side controller enters safe hold after heartbeat timeout.

## Acceptance Criteria

Documentation-to-implementation acceptance criteria:

- Every node publishes status heartbeat at 1 Hz.
- `supervisor_node` publishes its own heartbeat and monitors both local and remote node heartbeats.
- `controller_node` subscribes to `/s2e/supervisor/health` and stops/holds when `ok_to_move=false`.
- Every data topic uses the stamp and frame contract in [interfaces.md](interfaces.md).
- Single-PC mock launch runs for 60 seconds without uncaught exceptions.
- VLM/e2e operate asynchronously: e2e produces multiple trajectories per single VLM reasoning when TTL is valid.
- Controller never continues stale trajectory after trajectory TTL expires.
- Rotate action preempts trajectory following and returns success/failure deterministically.
- Debug visualizer publishes overlays during normal, stop, rotate, malformed VLM, and stale-input scenarios without affecting controller behavior.
- All failure injection scenarios above are covered by automated or manual tests.
