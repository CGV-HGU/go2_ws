# S2E VLM Async Framework

Python ROS 2 framework design for an asynchronous VLM and end-to-end driving stack on Unitree Go2. The target deployment splits robot-local nodes and external compute nodes, while the first implementation runs every node on one PC over ROS 2 IPC for deterministic integration testing.

This repository contains the design documentation, implementation plan, and initial ROS 2 package skeleton under `src/`. The current implementation includes project messages/actions, pure-Python core utilities, mock graph behavior, launch profiles, tests, and Docker/Compose assets according to [docs/superpowers/plans/2026-07-06-ros2-async-vlm-e2e-framework.md](docs/superpowers/plans/2026-07-06-ros2-async-vlm-e2e-framework.md).

The first ROS 2 implementation target is ROS 2 Jazzy on Ubuntu 24.04 for CPU-only mock, robot-side, and model packages. The VLM runtime is API-only and uses the CPU ROS base. The e2e ONNX runtime uses an NVIDIA CUDA/cuDNN Ubuntu 24.04 base so it can run ROS 2 Jazzy in the same DDS domain as the CPU containers; mixing Humble GPU containers with Jazzy CPU containers produced DDS deserialization errors during split-container smoke tests.

## Target System

Robot side:

- `static_tf_node`: publishes fixed sensor extrinsics from `config/sensors/*.yaml` on `/tf_static`.
- `lidar_node`: publishes preprocessed point clouds with acquisition timestamps.
- `camera_node`: publishes preprocessed camera images with acquisition timestamps.
- `imu_node`: publishes IMU packets with acquisition timestamps.
- `odometry_node`: subscribes to lidar, camera, and IMU streams, runs an LVIO/VIO adapter, and publishes `base_link` pose.
- `controller_node`: follows e2e trajectories, owns robot motion commands, and serves rotate actions.
- `supervisor_node`: monitors local and remote node heartbeats and reports degraded health to the controller.
- `debug_visualizer_node`: optional CPU-only debug node that overlays VLM/e2e/controller state on camera images without affecting motion.

External PC side:

- `vlm_node`: consumes synchronized image and pose snapshots, publishes string reasoning, and sends rotate action goals.
- `e2e_node`: consumes image, pose, and cached VLM reasoning, then publishes ego-centric trajectory and pose-at-trajectory.

Initial single-PC mode:

- All nodes run in one ROS 2 domain with intra-process communication enabled where possible.
- Mock algorithms publish deterministic sample data to verify topic flow, timestamps, stale handling, state transitions, and rotate preemption before hardware integration.
- The debug visualizer publishes an annotated image topic so action state, coarse/fine goal points, VLM text, trajectory, and rotate progress can be inspected without RViz 3D.

## Documentation Map

- [Requirements](docs/requirements.md): authoritative system requirements, development details, deployment modes, safety rules, and acceptance criteria.
- [Architecture](docs/architecture.md): node responsibilities, time model, frame model, state machines, safety behavior, and deployment split.
- [Interfaces](docs/interfaces.md): topics, actions, message contracts, stamp semantics, frame contracts, temporary VLM string schema, QoS, and TTL defaults.
- [Sequence Diagrams](docs/sequence_diagrams.md): normal async flow, VLM/e2e cached reasoning flow, rotate action flow, and fault/degradation flow.
- [Testing](docs/testing.md): mock validation strategy, launch tests, failure cases, manual QA commands, and acceptance criteria.
- [Implementation Plan](docs/superpowers/plans/2026-07-06-ros2-async-vlm-e2e-framework.md): task-by-task plan for creating the ROS 2 packages.

## Design Decisions

The project uses ROS 2 topics for continuous streams and ROS 2 Actions for long-running controller-owned rotation. VLM reasoning remains a string payload in the first version to avoid premature custom schema churn, but the string must be strict JSON and parsing failures must degrade to safe stop/hold behavior.

Public motion interfaces use `base_link` ego-centric coordinates. The odometry algorithm may compute internally in IMU coordinates, but `odometry_node` is responsible for publishing robot pose in `base_link` relative to `odom` or `map`. The e2e node converts VLM image-domain `uv` goal points into `base_link` 2D `(x, y)` goals before model inference.

Debug visualization is topic-based, not a control dependency. `debug_visualizer_node` preserves the raw camera stream and publishes a separate annotated image on `/s2e/debug/visualizer/image`. Pixel-space VLM goals are drawn directly on the image. Metric `base_link` trajectories and fine goals are shown as a 2D mini-map unless calibrated camera projection is explicitly available.

## Reference Material Used

The gitignored `reference/` directory is treated as design reference, not as code to copy directly.

- `reference/agf/agentworks/core/last_msg_store.py`: thread-safe latest-message cache and blocking wait pattern.
- `reference/agf/agentworks/agent_host_app.py`: periodic assembly of cached world inputs into VLM/e2e payloads.
- `reference/agf/agentworks/core/agent_define.py`: topic ordering, endpoint constants, and payload sizing patterns.
- `reference/ete/python/motif_e2e/util/last_msg_store.py`: extended cache with put/pop conditions and timeout behavior.
- `reference/ete/python/motif_e2e/agent_service.py`: process supervision and log-path pattern.
- `reference/ete/README.md`: e2e model input/output shapes, TensorRT runtime expectations, and trajectory/path semantics.
- `reference/ete/python/motif_e2e/agent_e2e.py`: concrete adapter from IPC payloads into numpy/model inputs.
- `reference/ete/python/motif_e2e/datapipeline/pipeline.py`: image resize, normalize, metadata, and model tensor layout pattern.
- `reference/base/dal/coord_lib/coords_transformer.py`: explicit coordinate-system transform boundary.
- `reference/base/dal/coord_lib/utils.py`: 4x4 transform matrix and point transformation utilities.
- `reference/base/dal/geometry/transform/transform.py`: rigid transform composition and inversion pattern.
- `reference/algo/projects/team_code/lateral_controller.py`: ego/global 2D transform and route geometry helpers.
- `reference/algo/projects/team_code/longitudinal_controller.py`: PID state, braking, and curvature-aware speed limiting.
- `reference/algo/projects/team_code/controller_config.py`: initial PID configuration structure.

Some reference files are proprietary or depend on internal packages. The implementation should re-create only the required public patterns in this project with clean interfaces and tests.

## How To Run

There are four supported ways to run the current implementation:

- **Single PC, one ROS graph**: easiest path for development, integration tests, and visualizer artifacts.
- **Single PC, split containers**: runs robot-side and external-PC nodes as separate services on the same host.
- **Two machines, robot + external PC**: clone the same repo on both machines, run only each machine's role, and let ROS 2 DDS connect them.
- **Three machines, robot + VLM PC + e2e PC**: run `robot-core`, `vlm`, and `e2e` on separate hosts in the same ROS 2 DDS domain.

All modes use the same topic/action/message contracts. The robot side always owns motion authority through `controller_node` and `supervisor_node`; the external PC only runs VLM/e2e reasoning.

### Quick Start: Single PC With Docker

This is the fastest way to run the full mock stack without installing ROS 2 on the host.

```bash
cp .env.example .env
docker compose build ros-base dev-mock
docker compose --profile single_pc_mock up dev-mock
```

The `dev-mock` service launches `single_pc_mock.launch.py`, which starts static TF, lidar/camera/IMU mocks, odometry, controller, supervisor, VLM, e2e, and the debug visualizer. Visualizer artifacts are written through the artifact mount when `S2E_TEST_ARTIFACT_DIR` is set.

Example artifact run:

```bash
mkdir -p artifacts/visualizer
S2E_ARTIFACT_DIR=./artifacts \
S2E_TEST_ARTIFACT_DIR=/artifacts/visualizer \
S2E_MOCK_ARTIFACT_DURATION_S=10.0 \
S2E_DEBUG_MODE=1 \
docker compose --profile single_pc_mock up dev-mock
```

Expected files include `artifacts/visualizer/frame_0000.png`, `visualizer.mp4`, and `manifest.json`. The MP4 is encoded as H.264 with `yuv420p` when `ffmpeg` is available in the container. With `S2E_DEBUG_MODE=1`, the overlay adds a diagnostics panel and the manifest includes runtime context, supervisor health, VLM parse state, node statuses, and projection status.

Mock runtime timing and scenario knobs are configured through `.env` and are explicitly passed by Compose into each service that runs mock runtime code. For example, set `S2E_MOCK_VLM_PERIOD_S=0.5`, `S2E_MOCK_E2E_PERIOD_S=0.1`, `S2E_MOCK_DEBUG_VISUALIZER_PERIOD_S=0.1`, and `S2E_MOCK_ARTIFACT_SAVE_PERIOD_S=0.1` when you want split-container visualizer artifacts to update at the same cadence as a direct single-container debug run.

### S2E e2e Backend

The e2e service can run the deterministic mock planner or the downloaded S2E model backend. Download the public S2E assets without modifying the host Python environment:

```bash
docker run --rm -v "$PWD:/work" -w /work python:3.12-slim \
  bash -lc "python -m pip install --no-cache-dir -q huggingface_hub && \
  hf download UCLA-VAIL/Navigation-Model-Zoo-Public --include 'S2E/*' --local-dir ./nav_model_zoo && \
  chown -R $(id -u):$(id -g) ./nav_model_zoo"
```

Set these values in `.env` to use the S2E backend:

```bash
E2E_BACKEND=s2e
S2E_NAV_MODEL_ZOO_DIR=./nav_model_zoo
E2E_MODEL_PATH=/models/s2e/S2E
```

The S2E backend buffers the latest 11 RGB camera frames, converts them to `(1, 11, 3, 256, 256)` float32 `[0,1]`, uses the projected `goal_uv -> goal_xy` point-goal, and publishes the returned 10x2 trajectory through the existing `/s2e/e2e/trajectory` contract. With the current dummy white camera image this validates model loading, CUDA/ONNX execution, ROS publication, controller input, and debug artifact generation. It does not validate real driving quality; semantic validation needs recorded or realistic RGB sequences.

### Qwen VLM API Backend

`vlm_node` is the ROS-aware VLM agent/orchestrator, not the 32B model server. It collects image and pose context, checks freshness, builds the request, calls a Qwen3-VL server API, parses/refines the response, normalizes it to strict JSON, applies safety/action filters, and publishes `/s2e/vlm/reasoning` plus heartbeat status.

The Qwen3-VL 32B Thinking model should run in a separate model-serving process or service such as vLLM, SGLang, or another OpenAI-compatible endpoint. Configure the adapter with:

```bash
VLM_BACKEND=qwen_api
VLM_API_URL=http://qwen-vl-server:8000/v1/chat/completions
VLM_API_TIMEOUT_S=10.0
VLM_API_MAX_RETRIES=1
```

The mock backend remains the default until the API client implementation is enabled.

### Single PC With Native ROS 2

Use this if the host already has ROS 2 Jazzy and `colcon` installed.

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch s2e_vlm_bringup single_pc_mock.launch.py
```

Useful inspection commands:

```bash
ros2 topic list
ros2 topic echo /tf_static --once
ros2 topic echo /s2e/sensors/camera/camera_info --once
ros2 topic echo /s2e/vlm/reasoning --once
ros2 topic echo /s2e/e2e/trajectory --once
ros2 topic hz /s2e/debug/visualizer/image
ros2 action list
ros2 action info /s2e/controller/rotate
```

To manually test the rotate action:

```bash
ros2 action send_goal /s2e/controller/rotate s2e_vlm_msgs/action/Rotate \
  "{target_yaw_delta_deg: 30.0, max_yaw_rate_deg_s: 30.0, tolerance_deg: 3.0, timeout_s: 5.0}" \
  --feedback
```

The controller should enter `ROTATING`, publish yaw commands, clear the current trajectory, and return `success=true` in mock mode. Automated tests also cover the VLM-triggered path where `vlm_node` emits rotate reasoning and sends the action goal itself.

### Single PC Split Containers

This mode keeps one physical PC but runs robot-side and external-PC roles as separate containers. It is the best local rehearsal for two-machine deployment.

```bash
cp .env.example .env
docker compose build ros-base onnx-runtime-base
docker compose build robot-core vlm e2e
docker compose --profile single_pc_split up robot-core vlm e2e
```

`robot-core` is CPU-only and launches `robot_side.launch.py`. `vlm` and `e2e` reserve the configured NVIDIA GPU and run the external compute nodes. All services use host networking and the shared ROS environment from `.env`.

### Robot + External PC Split

Clone this repository on both machines and use the same branch or commit on both sides.

On both machines:

```bash
cp .env.example .env
# Edit .env if needed. At minimum, both machines must agree on:
# ROS_DOMAIN_ID=42
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ROS_LOCALHOST_ONLY=0
```

On the robot computer:

```bash
docker compose build ros-base robot-core
docker compose --profile robot_side up robot-core
```

On the external GPU PC:

```bash
docker compose build ros-base onnx-runtime-base
docker compose build vlm e2e
docker compose --profile external_gpu up vlm e2e
```

### Robot + VLM PC + e2e PC Split

Use this mode when VLM and e2e need separate GPU computers. Clone the same repository revision on all three machines and keep the shared ROS values in `.env` identical: `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `ROS_LOCALHOST_ONLY=0`.

On the robot computer:

```bash
docker compose build ros-base robot-core
docker compose --profile robot_side up robot-core
```

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

For a same-host rehearsal of the three-service boundary, run the same three official profiles together:

```bash
docker compose build ros-base onnx-runtime-base
docker compose build robot-core vlm e2e
docker compose --profile robot_side --profile vlm_only --profile e2e_only up robot-core vlm e2e
```

The e2e PC must be able to discover `/s2e/vlm/reasoning` from VLM PC1 plus robot-side `/s2e/sensors/camera/image`, `/s2e/odometry/pose`, and `/s2e/supervisor/health`. The robot computer must discover `/s2e/status/vlm_node`, `/s2e/status/e2e_node`, and `/s2e/e2e/trajectory` from the two GPU PCs.

Native ROS 2 launch is equivalent if both machines have ROS 2 Jazzy installed:

```bash
# Robot computer
source install/setup.bash
ros2 launch s2e_vlm_bringup robot_side.launch.py use_mock_hardware:=true

# External PC
source install/setup.bash
ros2 launch s2e_vlm_bringup external_pc.launch.py use_mock_models:=true
```

Visualizer placement:

```bash
# Robot-side visualizer is on by default. Disable it if the robot CPU should not render debug overlays.
ros2 launch s2e_vlm_bringup robot_side.launch.py enable_debug_visualizer:=false

# External-PC visualizer is off by default. Enable it only when the PC can see camera/debug topics.
ros2 launch s2e_vlm_bringup external_pc.launch.py enable_debug_visualizer:=true
```

Launch arguments currently declared by all launch profiles are `use_mock_hardware`, `use_mock_models`, `sensor_config_dir`, `enable_debug_visualizer`, and `namespace`. Sensor calibration overrides are read by the runtime from `S2E_SENSOR_CONFIG_DIR`, so set that environment variable or `.env` entry when mounting robot-specific calibration YAML.

Set `S2E_DEBUG_MODE=1` in `.env` when you want the visualizer artifact and overlay to include runtime diagnostics such as ROS domain/RMW, role, supervisor missing/unhealthy lists, selected node heartbeat states, VLM parse result, e2e/controller status, and projection status. Compose sets `S2E_RUNTIME_ROLE` per service so the manifest identifies whether the artifact came from `single_pc_mock`, `robot_side`, `external_pc_vlm`, or `external_pc_e2e`.

Compose also passes the documented `S2E_MOCK_*` runtime settings from `.env` into `dev-mock`, `robot-core`, `vlm`, and `e2e`. This matters in split modes because `.env` interpolation alone does not inject variables into containers; every supported mock timing/scenario/artifact knob is listed under service `environment` so robot-side and external-PC services observe the same configured cadence.

### Split-Mode Smoke Checks

Run these from either machine after both sides are up:

```bash
ros2 topic echo /s2e/odometry/pose --once
ros2 topic echo /s2e/e2e/trajectory --once
ros2 topic echo /s2e/status/vlm_node --once
ros2 topic echo /s2e/status/e2e_node --once
ros2 topic echo /s2e/supervisor/health --once
ros2 action info /s2e/controller/rotate
```

If the external PC is stopped, robot-side `supervisor_node` should mark VLM/e2e heartbeats missing and `controller_node` should hold safe zero motion after the trajectory/heartbeat timeouts.

Both machines must share `ROS_DOMAIN_ID`, compatible `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY=0`, reachable network interfaces, and synchronized clocks. Same domain ID alone is not enough if multicast, firewall, Docker networking, VPN, or mixed RMW settings block DDS discovery.

For real robot commands, `ROS_DOMAIN_ID` is not a security boundary. Robot and external GPU PC traffic must run on an isolated network or firewall allowlist, and motion-relevant topics/actions must use DDS Security/SROS2 or an equivalent authenticated transport before field deployment.

### Test Commands

Run full ROS tests in Docker:

```bash
docker build -t s2e-ros-base:latest -f docker/ros-base.Dockerfile .
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose"
```

Or run native ROS tests after building:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

On hosts without ROS 2 or `colcon`, run the pure-Python verification suite:

```bash
python3 -m unittest discover -s src/s2e_vlm_core/test -p "test_*.py" -v
python3 -m unittest discover -s src/s2e_vlm_nodes/test -p "test_*.py" -v
python3 -m unittest src/s2e_vlm_bringup/test_launch_contracts.py -v
python3 -m unittest tests/test_docker_assets.py -v
```

## Safety Defaults

- Controller is the only authority that decides active robot motion mode.
- Loss of odometry, trajectory, heartbeat, or valid VLM command degrades to stop/hold.
- In robot/PC split mode, robot-side `supervisor_node` monitors external VLM/e2e heartbeats, and `controller_node` still enforces local odometry and trajectory TTLs independently.
- Rotation preempts trajectory following and owns the controller until success, cancellation, timeout, or fault.
- Cached VLM reasoning expires by time and by pose-distance thresholds.
- Malformed VLM strings are not best-effort parsed into motion.
- Debug visualizer failures never affect robot motion authority or fail-closed behavior.
- `header.stamp` is acquisition/reference time; `processed_stamp` is node output time.
