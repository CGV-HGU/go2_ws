# Requirements and Development Details

## Purpose

This document is the consolidated requirements source for the S2E VLM async framework. It defines what the first ROS 2 system must build and verify before Unitree Go2 hardware integration. The implementation plan describes how to sequence the work. This document defines the required behavior, contracts, safety rules, deployment modes, and acceptance conditions.

The system coordinates asynchronous sensing, odometry, VLM reasoning, e2e trajectory inference, controller motion, supervision, and debug visualization for a Unitree Go2. ROS 2 is the communication layer. The first validation target is a deterministic single-PC mock graph that exercises the same topics, actions, timing rules, frame rules, and fail-closed behavior as the robot and external GPU PC split.

## Scope

The first implementation must provide a Python ROS 2 package set with these functional areas:

- ROS 2 interfaces in `s2e_vlm_msgs` for pose wrappers, trajectories, node health, system health, and rotate actions.
- Core utilities for timestamp handling, pose buffers, transforms, strict VLM JSON parsing, latest-message caches, state helpers, and adapter boundaries.
- Mock sensor, odometry, VLM, e2e, controller, supervisor, and debug visualizer nodes.
- Bringup profiles for single-PC mock validation and the robot side plus external PC split.
- Unit tests, launch tests, failure injection tests, manual QA commands, and split-machine smoke tests.
- Docker and Compose assets grouped by runtime boundary, with CPU-only robot-side images and GPU-capable model images.

The implementation must use the gitignored `reference/` directory only as design reference. Patterns such as latest-message caching, e2e IO shapes, process supervision ideas, coordinate transforms, and controller geometry may inform the project design, but proprietary code and internal packages must not be copied into this repository.

## Non-Goals

- No runtime code outside the documented ROS 2 framework is required by this document.
- No direct hardware-specific Unitree command message contract is required for mock validation. The controller must expose an adapter boundary so real Go2 command integration can replace the mock `Twist` output without changing controller authority semantics.
- No best-effort parsing of VLM natural language into motion is allowed. VLM output is a strict JSON string in version 0.
- No calibrated 3D debug visualization is required for the first version. The visualizer is a 2D image overlay with a metric mini-map unless valid camera calibration and transforms are available.
- No Kubernetes, external orchestrator, or one-container-per-node deployment is required for the initial Docker design.
- No CUDA, cuDNN, TensorRT, ONNX Runtime GPU, or NVIDIA runtime dependency may be required by robot-side controller, odometry, sensor, supervisor, or debug visualizer containers.

## Target Deployment Modes

### Single-PC Mock Development

All nodes run on one development PC in one ROS 2 domain with intra-process communication enabled where practical. This is the first implementation target. It must validate inter-process communication, message and action contracts, state transitions, timestamps, stale handling, cached VLM reuse, rotate preemption, watchdog behavior, and debug overlays before hardware integration.

The mock graph must run without Unitree Go2 hardware and without real VLM, e2e, or LVIO models. Mock publishers and adapters must be deterministic so tests are repeatable.

### Single-PC Split Containers

Robot-side, VLM, and e2e services run as separate containers on one host. This mode validates ROS 2 discovery, QoS compatibility, topic flow, rotate action flow, and heartbeat supervision across container boundaries before the deployment uses two physical machines.

Initial Compose profiles may use host networking and host IPC for lab validation. These settings are development defaults, not security controls.

The implemented Compose profile is `single_pc_split`, which starts `robot-core`, `vlm`, and `e2e`. Build `ros-base` before `robot-core` and `vlm`, and build `onnx-runtime-base` before `e2e`. Copy `.env.example` to `.env` so all services share `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `ROS_LOCALHOST_ONLY=0`.

### Unitree Go2 Robot plus External GPU PC

The deployment-ready split is:

- Robot side: sensor nodes, `odometry_node`, `controller_node`, `supervisor_node`, and optional `debug_visualizer_node`.
- External GPU PC: `vlm_node` and `e2e_node`.

The split must not change topic names, action names, message contracts, frame semantics, freshness rules, or safety behavior. Only launch files, network configuration, Docker profiles, and hardware or model adapters should differ.

The debug visualizer may run on the robot side, the external GPU PC, or a development host as a CPU-only observer when the required camera and debug topics are reachable. Its placement must not change its observer-only behavior.

Both machines must share compatible ROS 2 interface definitions, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, reachable network interfaces, and synchronized clocks. `ROS_DOMAIN_ID` is not authentication. Real robot deployments must use an isolated network or firewall allowlist, with DDS Security/SROS2 or equivalent authenticated transport for motion-relevant topics and actions.

The implemented Compose profiles are `robot_side` for `robot-core` and `external_gpu` for `vlm` plus `e2e`. Concrete commands are maintained in the README and [testing.md](testing.md); those examples are the expected operator path for mock robot/PC split validation.

### Unitree Go2 Robot plus Separate VLM and e2e PCs

The deployment may split external compute further across three machines:

- Robot side: sensor nodes, `odometry_node`, `controller_node`, `supervisor_node`, and optional `debug_visualizer_node`.
- VLM PC: `vlm_node` only.
- e2e PC: `e2e_node` only.

This split must use the same ROS 2 topics, actions, message contracts, frame semantics, freshness rules, and safety behavior as the two-machine split. The e2e PC must discover `/s2e/vlm/reasoning` from the VLM PC and robot-side camera, odometry, and supervisor health topics. The robot side must discover VLM/e2e status heartbeats and the e2e trajectory topic from the VLM and e2e PCs.

The implemented Compose profiles are `vlm_only` for the API-only `vlm` service and `e2e_only` for the GPU ONNX `e2e` service. The combined `external_gpu` profile remains supported for a single external PC that runs both services.

## Node Requirements

### Sensor Nodes

`lidar_node`, `camera_node`, and `imu_node` must publish independently at their own driver or timer rates. They must not synchronize with each other.

Required outputs:

- `lidar_node` publishes `/s2e/sensors/lidar/points` as `sensor_msgs/msg/PointCloud2`, with `header.frame_id` set to `lidar`.
- `camera_node` publishes `/s2e/sensors/camera/image` as `sensor_msgs/msg/Image`, with `header.frame_id` set to `camera`, plus optional `/s2e/sensors/camera/camera_info`.
- `imu_node` publishes `/s2e/sensors/imu` as `sensor_msgs/msg/Imu`, with `header.frame_id` set to `imu`.

Every sensor-derived message must preserve acquisition time in `header.stamp`. If a project wrapper is used, `source_stamp` is the upstream source time and `processed_stamp` is the current node's output time.

### Odometry Node

`odometry_node` subscribes to lidar, camera, and IMU streams asynchronously. The LVIO or VIO adapter owns internal synchronization, interpolation, queuing, and sensor selection. Public output must be robot pose in `base_link` relative to `odom` or `map` on `/s2e/odometry/pose`.

The odometry algorithm may compute internally in `imu` coordinates. Before publishing, `odometry_node` must apply the calibrated `imu -> base_link` transform so downstream nodes never consume IMU-frame robot pose as a public motion interface.

### VLM Node

`vlm_node` runs at VLM inference cadence, slower than the e2e node. For each inference cycle, it takes the latest image and a bounded pose at or before the image timestamp. The pose age must be no more than `0.20 s` unless interpolation provides a valid pose at the image time.

`vlm_node` publishes strict JSON strings on `/s2e/vlm/reasoning`. If the JSON action is `rotate`, the VLM node must send a `Rotate.action` goal to `/s2e/controller/rotate` and enter `FROZEN_ROTATING` until the action returns success, abort, cancel, or timeout. It must not send repeated rotate goals while one is active.

For real VLM operation, `vlm_node` is the ROS-aware agentic orchestrator and API client. It may assemble context, prompts, state summaries, and safety constraints locally, but Qwen3-VL 32B Thinking model loading and generation must run in a separate model-serving process or service. The node must keep publishing heartbeat/degraded status when the model server times out, rejects a request, or returns malformed output.

### E2E Node

`e2e_node` runs independently from VLM and may run faster. It consumes the latest image, bounded pose, cached VLM reasoning, and `/s2e/supervisor/health`. It must treat `vlm_node` in `unhealthy_nodes` or `missing_critical_nodes`, and any `ok_to_move=false` state, as cache invalidation and publish no new trajectory.

If no valid VLM reasoning has arrived, `e2e_node` must stay in `WAITING_FIRST_VLM` and publish no trajectory. For VLM action `stop`, it must publish motion-blocking status and no new trajectory. For invalid or stale VLM reasoning, it must publish degraded or invalid status and no new trajectory. For valid `go`, it must convert `goal_uv` to a `base_link` 2D goal, compensate that goal from the VLM pose to the current e2e pose, run the e2e adapter, and publish `/s2e/e2e/trajectory`.

The S2E e2e backend must buffer 11 RGB frames before inference. Until that context is available, it must publish a waiting/degraded status such as `WAITING_IMAGE_CONTEXT` and must not publish trajectory. The model input is `(1, 11, 3, 256, 256)` float32 in `[0,1]`, built from camera frames in acquisition order. Runtime inference must load `s2e.onnx` directly through ONNXRuntime GPU and must not require the downloaded Python `S2E.inference` wrapper or PyTorch. The backend must reject non-finite or incorrectly shaped model output before publishing.

The version 0 mock `goal_uv` conversion must be deterministic and config-driven. Real adapters must replace it with calibrated camera geometry, depth, ground-plane projection, or model-native preprocessing. If required calibration or depth is unavailable, e2e must not invent a metric goal. It must publish degraded status and no trajectory.

### Controller Node

`controller_node` is the only motion authority. It subscribes to `/s2e/e2e/trajectory`, `/s2e/odometry/pose`, `/s2e/e2e/status`, and `/s2e/supervisor/health`, and it serves `/s2e/controller/rotate`.

During trajectory following, the controller transforms the stored trajectory from `pose_at_trajectory` to the current pose and generates robot commands through PID or hardware adapters. In mock mode, `/s2e/controller/command` may be `geometry_msgs/msg/Twist`, where linear `x` is forward velocity and angular `z` is yaw rate. Real Go2 command integration must keep the same controller authority semantics.

Rotation is controller-exclusive. Accepting a rotate action must clear stored trajectory state, stop trajectory following, record start yaw from fresh odometry, and command rotation until the signed yaw delta reaches the requested target within tolerance. If odometry becomes stale, the controller must abort or stop and hold. Rotate completion is reported through the action result and controller status, not through a separate completion topic.

### Supervisor Node

`supervisor_node` is the robot-side system health publisher and heartbeat monitor. It publishes `/s2e/supervisor/health` and its own status heartbeat. In split deployment, it must monitor local robot node heartbeats plus remote `/s2e/status/vlm_node` and `/s2e/status/e2e_node` heartbeats from the external PC.

The supervisor marks missing safety-critical heartbeats unhealthy after `3` missed heartbeat periods. With the default heartbeat rate of `1 Hz`, that means health becomes degraded after three missed seconds. The controller must subscribe to `/s2e/supervisor/health` and treat `ok_to_move=false` as motion-blocking.

### Static TF Node

`static_tf_node` runs on the robot/sensor side. It parses `s2e_vlm_bringup/config/sensors/*.yaml`, publishes fixed sensor transforms on ROS `/tf_static`, and publishes `/s2e/status/static_tf_node`. Missing or malformed sensor calibration is a node fault.

### Debug Visualizer Node

`debug_visualizer_node` is an optional CPU-only observer. It subscribes to camera images, optional camera info, VLM reasoning, e2e trajectory, e2e status, controller status, enumerated node status topics, and supervisor health. It publishes annotated images on `/s2e/debug/visualizer/image`.

The visualizer must never publish robot commands, influence controller state, acknowledge rotate actions, or make safety decisions. If debug inputs are stale, malformed, or unavailable, it must keep publishing the raw image with degraded overlay labels when camera frames are available.

## Interface Requirements

All project topics and actions use the `/s2e` prefix. These names are required.

Robot-local topics:

- `/s2e/sensors/lidar/points`
- `/s2e/sensors/camera/image`
- `/s2e/sensors/camera/camera_info`
- `/s2e/sensors/imu`
- `/s2e/odometry/pose`
- `/s2e/controller/command`
- `/s2e/controller/status`
- `/s2e/supervisor/health`
- `/s2e/debug/visualizer/image`

ROS standard graph topics used by the system:

- `/tf_static`

External-compute topics:

- `/s2e/vlm/reasoning`
- `/s2e/e2e/trajectory`
- `/s2e/e2e/status`

Action:

- `/s2e/controller/rotate`

Status topics:

- `/s2e/status/static_tf_node`
- `/s2e/status/lidar_node`
- `/s2e/status/camera_node`
- `/s2e/status/imu_node`
- `/s2e/status/odometry_node`
- `/s2e/status/vlm_node`
- `/s2e/status/e2e_node`
- `/s2e/status/controller_node`
- `/s2e/status/supervisor_node`
- `/s2e/status/debug_visualizer_node`

### `NodeStatus`

`s2e_vlm_msgs/msg/NodeStatus` is required for node heartbeats and may also be used for `/s2e/controller/status` and `/s2e/e2e/status` in version 0.

```text
std_msgs/Header header
string node_name
string state
string active_mode
bool is_healthy
bool is_motion_critical
float32 last_input_age_s
float32 last_output_age_s
string error_code
string message
```

`header.stamp` is status publish time. `state` must match the documented node states or `UNKNOWN` during startup. `active_mode` carries mode details such as `FOLLOWING`, `ROTATING`, `STOPPED_BY_VLM`, `DEGRADED`, `INVALID_VLM`, `VLM_STALE`, or `SUPERVISOR_BLOCKED`. `is_motion_critical` is true for odometry, controller, supervisor, and external VLM/e2e heartbeats in robot/PC split mode. Debug visualizer health is non-critical.

### `StampedPose`

`s2e_vlm_msgs/msg/StampedPose` is required when `/s2e/odometry/pose` cannot use `nav_msgs/msg/Odometry` with complete pose and twist semantics.

```text
std_msgs/Header header
builtin_interfaces/Time source_stamp
builtin_interfaces/Time processed_stamp
string child_frame_id
geometry_msgs/Pose pose
float32 confidence
string status
```

`header.stamp` is the pose reference time. `header.frame_id` is `odom` or `map`. `child_frame_id` must be `base_link`. The pose represents `base_link` in the parent frame, never IMU-frame pose. `source_stamp` preserves the upstream sensor or odometry reference time, and `processed_stamp` is the publish time for latency and watchdog checks.

### `SystemHealth`

`s2e_vlm_msgs/msg/SystemHealth` is required on `/s2e/supervisor/health`.

```text
std_msgs/Header header
bool ok_to_move
string overall_state
string[] unhealthy_nodes
string[] missing_critical_nodes
string reason
```

`ok_to_move=false` is motion-blocking. The controller must stop or hold regardless of the current trajectory. `e2e_node` must invalidate cached VLM reasoning and publish no new trajectory when `vlm_node` is listed in `unhealthy_nodes` or `missing_critical_nodes`, or when `ok_to_move=false`. Missing critical nodes include odometry, controller self-health, supervisor self-health, and remote VLM/e2e heartbeats in robot/PC split mode. Debug-only node failure must not set `ok_to_move=false`.

### `Trajectory2D`

`s2e_vlm_msgs/msg/Trajectory2D` is the ROS contract for `/s2e/e2e/trajectory`.

```text
std_msgs/Header header
builtin_interfaces/Time source_stamp
builtin_interfaces/Time processed_stamp
geometry_msgs/PoseStamped pose_at_trajectory
geometry_msgs/Point32[10] points
geometry_msgs/Point32 goal_point_base_link
bool has_goal_point
string source_vlm_json
string status
```

`header.stamp` and `source_stamp` are the image timestamp used for e2e inference. `header.frame_id` must be `base_link`. `pose_at_trajectory` is the `base_link` pose in `odom` or `map` at trajectory start. `points` must contain exactly 10 finite ego-centric 2D points in `base_link`; only `x` and `y` are used, and `z` is `0.0`.

The reference e2e runtime may produce `trajectory/path` as 6x2. The ROS contract still requires exactly 10 finite `base_link` points. The e2e adapter must reconcile reference output with the ROS contract by resampling, padding, or interpolation, and it must mark the method in `status`.

### `Rotate.action`

`s2e_vlm_msgs/action/Rotate.action` is required for `/s2e/controller/rotate`.

```text
float32 target_yaw_delta_deg
float32 max_yaw_rate_deg_s
float32 tolerance_deg
float32 timeout_s
---
bool success
string result_code
float32 final_yaw_delta_deg
string message
---
float32 current_yaw_delta_deg
float32 remaining_deg
string controller_state
```

The controller must reject goals when it is in `FAULT`, odometry is stale, or another rotate action is active. On accept, it clears trajectory state and enters `ROTATING`. Failure result codes include `TIMEOUT`, `ODOM_STALE`, `CANCELED`, and `COMMAND_ERROR`.

Recommended rotate defaults are `3 deg` tolerance, `0.30 s` settle time, feedback at `10 Hz`, and timeout `max(3.0, abs(target_deg) / 30.0 + 2.0)` seconds in mock control.

### VLM Strict JSON String

`/s2e/vlm/reasoning` uses `std_msgs/msg/String` in version 0. The payload must be strict JSON.

```json
{
  "schema_version": 0,
  "stamp": {"sec": 0, "nanosec": 0},
  "frame_id": "camera",
  "action": "go",
  "goal_uv": {"u": 640.0, "v": 360.0},
  "rotate_deg": 0.0,
  "pose": {
    "frame_id": "odom",
    "child_frame_id": "base_link",
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  },
  "reasoning": "safe concise natural-language explanation"
}
```

Allowed actions are `stop`, `go`, and `rotate`. `goal_uv` is required for `go`. `rotate_deg` must be non-zero for `rotate`. `pose` is the VLM pose snapshot at `stamp`. Invalid JSON, missing fields, unknown schema versions, or unknown actions must be treated as no valid motion command. Consumers must not use best-effort string matching to create motion.

## Async, Time, and Freshness Requirements

Sensor, VLM, e2e, controller, supervisor, and visualizer loops run independently. The system must prove asynchronous behavior in tests rather than only proving startup.

Time fields have fixed meanings:

- `header.stamp`: acquisition or reference time for the represented data.
- `source_stamp`: original upstream acquisition or reference time when a message wraps or transforms another message.
- `processed_stamp`: time the current node produced the output.

Pose lookup policy is latest pose at or before the target timestamp, with max-age rejection. If interpolation is available and a pose after the target timestamp exists, consumers may interpolate instead of using latest-before. If no bounded pose exists, the node must drop output, publish degraded status, or stop according to its role.

Required defaults:

- Pose sync max age for VLM and e2e image synchronization: `0.20 s`.
- Controller odometry max age: `0.10 s`.
- VLM reasoning TTL: `8.0 s`.
- E2E trajectory TTL: `0.50 s`.
- Heartbeat rate: `1 Hz`.
- Missed heartbeat threshold: `3` periods.
- Cached VLM compensation translation bound: `1.5 m`.
- Cached VLM compensation yaw bound: `30 deg`.

E2E may reuse cached VLM reasoning only while the VLM TTL is valid, `/s2e/supervisor/health` does not report blocked system health or list `vlm_node` as unhealthy or missing, and compensation from the VLM pose to the current pose stays within both translation and yaw bounds. If either bound is exceeded, e2e must not keep warping the old goal. It must degrade and wait for fresh VLM reasoning.

## Frame and Coordinate Requirements

Public motion interfaces use `base_link` ego-centric coordinates. Initial public frames are `map`, `odom`, `base_link`, `camera`, `lidar`, and `imu`.

Frame rules:

- Odometry publishes `base_link` pose relative to `odom` or `map`.
- E2E input goals after preprocessing are `base_link` 2D points, with `x+` forward and `y+` left.
- E2E trajectories are 10x2 ego-centric `base_link` points at `pose_at_trajectory`.
- Controller transforms stored trajectories using relative pose between `pose_at_trajectory` and the latest current pose.
- Image-domain VLM `goal_uv` is pixel-space `(u, v)` and must not be treated as metric until e2e preprocessing converts it.
- Trajectory frames other than `base_link` are safety errors. The controller must reject them.

ROS 2 `tf2` is the runtime frame authority. Fixed extrinsics such as `base_link -> camera`, `base_link -> lidar`, and `base_link -> imu` are published with static transform broadcasters. Dynamic `odom -> base_link` transforms are published by odometry. Nodes must look up transforms at message timestamp when possible. Falling back to latest transforms is allowed only in mocks and must be marked degraded.

Sensor calibration is managed as one YAML file per sensor under `s2e_vlm_bringup/config/sensors/`. Each sensor YAML owns its extrinsic, and camera YAML also owns intrinsics used for `CameraInfo`. Nodes that need calibration must use the shared sensor config parser instead of duplicating hard-coded extrinsic or intrinsic values.

## Safety and Watchdog Requirements

The system must fail closed. Loss of odometry, valid trajectory, heartbeat, valid VLM command, supervisor health, or required transforms must lead to stop or hold behavior. The controller remains the final motion-safety authority even when the supervisor is present.

Required safety behavior:

- Controller is the only node that decides active robot motion mode.
- Debug visualizer failures never affect robot motion authority.
- Malformed VLM strings never produce motion.
- `STOPPED_BY_VLM`, `INVALID_VLM`, `VLM_STALE`, and `SUPERVISOR_BLOCKED` are motion-blocking for the controller.
- Rotate preempts trajectory following and owns the controller until success, cancel, timeout, or fault.
- If external VLM/e2e services disappear in split mode, robot-side `supervisor_node` marks health degraded and `controller_node` also stops independently when trajectory or odometry TTL expires.
- The robot side must be able to command zero velocity or hold without receiving any new message from the external PC.

Watchdog ownership is layered:

- `supervisor_node` monitors global node health and publishes `/s2e/supervisor/health`.
- `controller_node` enforces odometry staleness, trajectory TTL, rotate timeout, command adapter faults, and supervisor blocks.
- `vlm_node` and `e2e_node` enforce data freshness before producing reasoning or trajectories.
- `odometry_node` enforces sensor freshness before publishing pose.

## Debug Visualizer Requirements

The debug visualizer must publish `/s2e/debug/visualizer/image` as an annotated camera image while preserving the raw `/s2e/sensors/camera/image` stream unchanged. It is a control-independent observer for development, testing, and field debugging.

Required subscriptions:

- `/s2e/sensors/camera/image`
- `/s2e/sensors/camera/camera_info`
- `/s2e/vlm/reasoning`
- `/s2e/e2e/trajectory`
- `/s2e/e2e/status`
- `/s2e/controller/status`
- Enumerated node status topics listed in this document
- `/s2e/supervisor/health`

Required overlay content:

- Latest VLM action and concise reasoning summary.
- Coarse VLM `goal_uv` drawn in image coordinates.
- Fine e2e `goal_point_base_link` and 10-point trajectory drawn in a metric mini-map by default.
- Optional projection of `base_link` points into the image only when valid `CameraInfo` and `tf2` transforms exist for the image timestamp. In mock mode the saved visualizer manifest records whether projection was available and how many frames contained projected trajectory points.
- Controller and e2e state, rotate progress, remaining yaw, last rotate result, cache age, and data-age warnings.
- Clear labels for malformed VLM JSON, stale inputs, missing calibration, and unavailable optional data.

The visualizer must keep publishing overlays for normal, stop, rotate, malformed VLM, stale-input, and degraded-supervisor scenarios when camera frames are available.

## Docker and GPU Requirements

Docker images are grouped by dependency and deployment boundary, not by individual ROS 2 node.

Required image groups:

- `s2e-ros-base`: ROS 2, shared project packages, generated messages/actions, common launch/config utilities.
- `s2e-dev-mock`: extends the ROS base for single-PC mock testing.
- `s2e-robot`: CPU-only robot-side runtime for sensors, odometry, controller, supervisor, and optional debug visualizer.
- `s2e-onnx-runtime-base`: pinned e2e runtime base with CUDA, cuDNN, ROS 2 Jazzy, and ONNXRuntime GPU support, without PyTorch.
- `s2e-gpu-inference-base`: optional heavyweight GPU base with PyTorch CUDA wheels for checkpoint experiments, training-adjacent smoke tests, or future model runtimes that cannot use ONNX directly.
- `s2e-vlm`: API-only VLM node runtime extending the CPU ROS base; it does not request GPU access or mount local model weights.
- `s2e-e2e`: S2E ONNX e2e node runtime extending `s2e-onnx-runtime-base`, with GPU access and mounted `nav_model_zoo` assets.

The first ROS 2 implementation target is ROS 2 Jazzy on Ubuntu 24.04 for CPU-only mock, robot-side, API-only VLM, and GPU e2e packages. GPU model containers must use a CUDA/cuDNN Ubuntu 24.04 base when they directly participate in the same ROS 2 DDS domain as the CPU containers. A Humble-on-Ubuntu-22.04 GPU base is not accepted for direct DDS participation because split-container smoke tests produced `sequence size exceeds remaining buffer` deserialization logs when mixed with Jazzy CPU containers.

Only services that load local GPU models may request GPU devices. The current VLM service calls an external Qwen API and must not request a GPU. The e2e ONNX service must run with NVIDIA Container Toolkit on the host and explicit GPU reservations when Compose supports them. Inference-only services should use `NVIDIA_DRIVER_CAPABILITIES=compute,utility` unless a service has a documented need for graphics, video, or display capability.

Runtime images should include only the libraries needed by the active backend. VLM is API-only and uses the CPU ROS base. S2E e2e is ONNX-only and uses ONNXRuntime GPU without PyTorch. PyTorch CUDA wheels belong in the optional heavyweight `gpu-inference-base` or a future training/development image, not in the default runtime path. CUDA devel images are reserved for training, custom CUDA extension builds, or TensorRT/plugin compilation.

Compose profiles must include `single_pc_mock`, `single_pc_split`, `robot_side`, `external_gpu`, `vlm_only`, and `e2e_only`. ROS 2 launch files remain the source of truth for which nodes run together. Compose selects runtime environment, machine boundary, volumes, GPU access, and network settings.

## Testing and Acceptance Requirements

Testing must prove timestamp propagation, bounded pose lookup, latest-message caching, VLM/e2e decoupling, rotate preemption, stale-data degradation, watchdog behavior, QoS compatibility, debug visualization, and fail-closed controller behavior.

Required unit test groups:

- Timestamp helpers for `header.stamp`, `source_stamp`, and `processed_stamp` semantics.
- Pose buffer latest-before lookup, interpolation, max-age rejection, empty buffers, and out-of-order inserts.
- VLM parser cases for valid `go`, `stop`, `rotate`, malformed JSON, missing fields, invalid actions, and schema mismatch.
- Goal compensation bounds for translation and yaw.
- Trajectory validation for exactly 10 finite `base_link` points.
- Visualizer overlay helpers for `goal_uv`, mini-map scaling, stale labels, and malformed VLM display.
- Controller state machine cases for rotate preemption, stale odometry abort, timeout, and cancel.

Required launch and integration tests:

- Single-PC happy path from sensor mocks through controller mock.
- VLM slower than e2e, with cached VLM reuse within TTL.
- First VLM delayed, with e2e staying in `WAITING_FIRST_VLM` and publishing no trajectory.
- Pose delayed beyond `0.20 s`, with VLM/e2e degrading instead of using stale pose.
- Controller stopping when odometry exceeds `0.10 s` age.
- Rotate during trajectory following, with trajectory cleared and action-owned rotation.
- Malformed VLM string rejected by e2e and displayed by visualizer without creating motion.
- Heartbeat loss causing supervisor degradation and controller stop/hold when safety-critical.
- QoS compatibility for every required cross-node topic.
- Debug visualizer overlay publication during normal, stop, rotate, malformed VLM, stale-input, and degraded states.

Required split-deployment smoke checks:

- External PC can echo `/s2e/odometry/pose`.
- Robot side can echo `/s2e/e2e/trajectory`.
- Robot side can echo `/s2e/status/vlm_node` and `/s2e/status/e2e_node` from the external PC.
- Robot side publishes `/s2e/status/supervisor_node` and `/s2e/supervisor/health`.
- External PC can send a goal to `/s2e/controller/rotate` and receive a result.
- Stopping external VLM/e2e services causes robot-side safe hold after heartbeat and trajectory TTL handling.

Implementation acceptance criteria:

- Every node publishes heartbeat status at `1 Hz` on its enumerated status topic.
- `/s2e/supervisor/health` publishes `ok_to_move=false` for missing critical local or remote nodes.
- `controller_node` subscribes to `/s2e/supervisor/health` and stops or holds when `ok_to_move=false`.
- All sensor-derived data follows the stamp and frame contracts.
- Single-PC mock launch runs for `60 s` without uncaught exceptions.
- E2E produces multiple trajectories per valid VLM reasoning when TTL and compensation bounds permit reuse.
- Controller never continues a trajectory after the `0.50 s` trajectory TTL expires.
- Rotate action preempts following and returns deterministic success or failure.
- Debug visualizer publishes annotated images without affecting controller behavior.
- All required failure injection scenarios have observable safe results.

## Implementation-Readiness Checklist

- The implementation uses ROS 2 topics for continuous streams and ROS 2 Actions for controller-owned rotation.
- All topic and action names match this document and `docs/interfaces.md`.
- `StampedPose`, `Trajectory2D`, `NodeStatus`, `SystemHealth`, and `Rotate.action` match the documented contracts before dependent nodes are written.
- VLM output is strict JSON in `std_msgs/msg/String`, and invalid payloads are motion-blocking for consumers.
- Pose sync max age, controller odometry max age, VLM TTL, trajectory TTL, heartbeat rate, missed heartbeat threshold, and cached VLM compensation bounds use the documented defaults.
- Public motion coordinates are `base_link`, while image goals remain pixel-space until e2e conversion.
- The e2e adapter reconciles reference 6x2 output with the required 10-point ROS trajectory contract.
- Controller is the only motion authority and can stop or hold without external PC messages.
- Robot/PC split keeps safety on the robot side through supervisor health plus controller-local TTL checks.
- Debug visualizer is an observer only and publishes `/s2e/debug/visualizer/image` without changing the raw camera stream.
- Docker images are grouped by runtime boundary, with CPU-only robot-side services and GPU-capable VLM/e2e services.
- Tests cover normal async flow, cached VLM reuse, rotate flow, stop flow, malformed VLM, heartbeat loss, QoS compatibility, and split deployment smoke behavior.
