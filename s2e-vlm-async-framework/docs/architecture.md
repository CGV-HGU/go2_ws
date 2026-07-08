# Architecture

## Purpose

This system coordinates asynchronous perception, reasoning, planning, and control for a Unitree Go2 using Python ROS 2. Sensor and control loops run independently at their natural rates. Slower VLM reasoning is cached and reused only within explicit freshness and pose-compensation bounds.

## Deployment Model

The first implementation runs all nodes on one PC in one ROS 2 domain. This validates inter-process communication, topic contracts, state machines, and mock algorithms before hardware deployment.

The first implementation target is ROS 2 Jazzy on Ubuntu 24.04 for CPU-only mock, robot-side, API-only VLM, and GPU e2e packages. The VLM node calls an external Qwen API and stays on the CPU ROS base. The e2e ONNX container uses an NVIDIA CUDA/cuDNN Ubuntu 24.04 base so direct ROS 2 DDS traffic stays on one ROS distro. A Humble-on-Ubuntu-22.04 GPU base was tested and rejected for direct DDS participation because it produced `sequence size exceeds remaining buffer` deserialization logs when mixed with Jazzy CPU containers.

The deployment-ready split is:

- Robot computer: sensor nodes, odometry node, controller node, supervisor node, optional debug visualizer.
- External PC: VLM node, e2e node.

The split must not change topic names, message contracts, or frame semantics. Only launch files, ROS domain/network configuration, and hardware adapters should change.

Implemented launch profiles preserve those boundaries:

- `single_pc_mock.launch.py`: runs `static_tf_node`, `lidar_node`, `camera_node`, `imu_node`, `odometry_node`, `controller_node`, `supervisor_node`, `vlm_node`, `e2e_node`, and enables `debug_visualizer_node` by default.
- `robot_side.launch.py`: runs the robot-side nodes and enables `debug_visualizer_node` by default; it can be disabled with `enable_debug_visualizer:=false` when robot CPU budget is tight.
- `external_pc.launch.py`: runs `vlm_node` and `e2e_node`; `debug_visualizer_node` is disabled by default and can be enabled when the external PC can see the camera/debug topics.

All launch profiles declare `use_mock_hardware`, `use_mock_models`, `sensor_config_dir`, `enable_debug_visualizer`, and `namespace` for operator consistency. The current mock runtime reads calibration override paths from the `S2E_SENSOR_CONFIG_DIR` environment variable.

Multi-machine deployments must explicitly configure `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, network reachability, and clock synchronization. DDS discovery failures are deployment errors, not node-logic errors.

## Node Responsibilities

### Sensor Nodes

Each sensor is an independent publisher with its own timer or driver callback.

- `lidar_node` publishes cleaned point cloud data.
- `camera_node` publishes cleaned image data.
- `imu_node` publishes IMU packets.

Each output includes acquisition time in `header.stamp`, sensor frame in `header.frame_id`, and node output time in `processed_stamp` when using custom wrapper messages. Sensor nodes must not synchronize with each other.

### Odometry Node

`odometry_node` subscribes to lidar, camera, and IMU streams asynchronously. The LVIO/VIO algorithm adapter decides how to synchronize, interpolate, or queue inputs internally. Public output is always robot pose in `base_link` relative to `odom` or `map`. If twist is available, the implementation should publish `nav_msgs/Odometry`; if only pose is available, `StampedPose` or `PoseStamped` is acceptable.

The odometry algorithm may use IMU coordinates internally. Before publishing, `odometry_node` applies the calibrated `imu -> base_link` transform so downstream nodes do not need to know LVIO internals.

### VLM Node

`vlm_node` runs at VLM inference cadence. After each inference cycle finishes, it takes the latest image and the latest bounded pose at or before the image timestamp. The selected pose must satisfy the max pose age bound in [interfaces.md](interfaces.md).

Output is a strict JSON string on `/s2e/vlm/reasoning`. The string contains action, image-domain goal point, pose snapshot, and reasoning text. If the action is `rotate`, VLM sends a ROS 2 Action goal to the controller and freezes inference until the action result returns success, abort, cancel, or timeout.

### E2E Node

`e2e_node` runs independently from VLM and usually faster. It consumes image and pose snapshots using the same latest-pose-before-image policy. It also consumes the latest cached VLM reasoning and `/s2e/supervisor/health` so cached reasoning is invalidated when the supervisor reports blocked system health or lists `vlm_node` as unhealthy or missing.

If no VLM reasoning has ever arrived, e2e waits in `WAITING_FIRST_VLM`. If cached reasoning exists and is valid, e2e parses action and goal point. For `go`, it converts image-domain `uv` to a `base_link` 2D goal, compensates that goal from the VLM pose to the current e2e pose, runs the e2e model, and publishes a 10x2 ego-centric trajectory. For `stop` or invalid reasoning, it publishes no new trajectory and publishes `STOPPED_BY_VLM` or `INVALID_VLM` status so the controller can stop immediately instead of waiting only for trajectory TTL.

Version 0 mock conversion from `goal_uv` to `goal_point_base_link` uses a documented deterministic image-plane-to-ground-plane mapping from config: normalized image offset maps to a bounded forward/lateral goal in `base_link`. Real adapters must replace this with calibrated camera geometry, depth, ground-plane projection, or model-native preprocessing. If required calibration/depth is unavailable, e2e must not invent a metric goal; it publishes degraded status and no trajectory.

The real e2e adapter boundary must reconcile reference-model IO with the ROS contract. The reference e2e runtime may consume multi-camera inputs, calibration matrices, navigation target points, traffic-light/status fields, and produce a 6x2 trajectory/path. The ROS contract remains `Trajectory2D` with exactly 10 points for controller simplicity; adapters must resample, pad, or interpolate real model outputs into 10 finite `base_link` points and mark the method in `status`.

### Controller Node

`controller_node` owns robot motion. It subscribes to e2e trajectory, current pose, e2e status, and supervisor health. During normal following, it transforms the stored trajectory from `pose_at_trajectory` to the latest current pose and generates robot commands through PID adapters. `STOPPED_BY_VLM`, `INVALID_VLM`, or degraded supervisor health clears or blocks trajectory following.

Rotation is controller-exclusive. Accepting a rotate action clears stored trajectory state, stops trajectory following, records start yaw from odometry, and commands rotation until the yaw delta reaches the requested angle within tolerance. When complete, the action returns success and the controller publishes mode/status. If odometry is stale, the action aborts and the controller stays stopped.

### Debug Visualizer Node

`debug_visualizer_node` is an optional observer for development, testing, and field debugging. It subscribes to the camera image, optional camera info, VLM reasoning string, e2e trajectory, e2e status, controller status, and node status streams. It publishes an annotated `sensor_msgs/Image` on `/s2e/debug/visualizer/image` and a normal heartbeat on `/s2e/status/debug_visualizer_node`.

The node must never publish robot commands, influence controller state, acknowledge rotate actions, or make safety decisions. If visualization inputs are stale, malformed, or unavailable, it keeps rendering the raw image with degraded overlay text instead of crashing or blocking the control graph.

Overlay content should include the latest VLM action and reasoning summary, coarse VLM `goal_uv`, fine e2e `base_link` goal, 10-point trajectory, controller/e2e state, rotate progress, remaining yaw, last rotate result, and data-age warnings. The raw camera topic remains unchanged.

## Time Model

The system uses three time concepts.

| Field | Meaning | Owner |
| --- | --- | --- |
| `header.stamp` | Acquisition or reference time for the data in `header.frame_id` | Original data producer |
| `source_stamp` | Original upstream acquisition/reference time when wrapping or transforming data | Current publisher copies from input |
| `processed_stamp` | Time the current node produced this output | Current publisher |

Rules:

- `header.stamp` is never publish time unless the message represents a command/status created at publish time.
- `processed_stamp` is used for latency and watchdogs, not for sensor fusion.
- Every output derived from input data must preserve source time explicitly.
- Pose synchronization policy is “lookup or interpolate pose at target timestamp using the latest pose at or before the target, with max age bound.” If no bounded pose exists, the node drops or degrades the output.

## Frame Model

Initial public frames:

- `map`: optional global frame when SLAM/global localization is available.
- `odom`: continuous local odometry frame.
- `base_link`: robot body frame used by public pose, trajectory, and controller interfaces.
- `camera`: camera optical or camera body frame, defined by calibration.
- `lidar`: lidar frame, defined by calibration.
- `imu`: IMU frame, defined by calibration.

Public contracts:

- Odometry publishes `base_link` pose relative to `odom` or `map`.
- E2E input goal after preprocessing is `base_link` ego-centric 2D: `x+` forward, `y+` left, `z` dropped.
- E2E trajectory is 10x2 in `base_link` ego-centric 2D at `pose_at_trajectory`.
- Controller transforms stored trajectory using relative pose between `pose_at_trajectory` and current pose.
- Image-domain VLM goal is `(u, v)` in pixel coordinates and must not be interpreted as metric until the e2e front-end converts it.
- Visualizer draws image-domain goals directly in pixels. It draws `base_link` goals and trajectories in a 2D mini-map by default, and projects them into the camera image only when valid `CameraInfo` and `tf2` transforms are available for the image timestamp.

ROS 2 `tf2` is the runtime frame authority. Fixed extrinsics such as `base_link -> camera`, `base_link -> lidar`, and `base_link -> imu` are published with static transform broadcasters. Dynamic transforms such as `odom -> base_link` are published by odometry. Nodes must look up transforms at the message timestamp when possible; falling back to the latest transform is only allowed in mocks and must be marked degraded.

Sensor calibration files live in `s2e_vlm_bringup/config/sensors/`, one YAML file per sensor. `camera.yaml` contains both extrinsic and intrinsic calibration; `lidar.yaml` and `imu.yaml` contain extrinsics. `s2e_vlm_core.sensor_config` is the shared parser used by sensor publishers, static TF publication, and debug visualization fallback metadata so calibrated values can be replaced without changing node logic.

The reference transform design is inspired by `reference/base/dal/coord_lib/coords_transformer.py`, `reference/base/dal/coord_lib/utils.py`, and `reference/base/dal/geometry/transform/transform.py`, but implementation should use ROS 2 `tf2` conventions and project-owned data classes.

## State Machines

### Static TF Node

| State | Meaning | Exit |
| --- | --- | --- |
| `INIT` | Sensor config parser and static transform broadcaster created | Sensor configs load successfully |
| `ACTIVE` | Publishing fixed sensor transforms on `/tf_static` and heartbeat status | Config reload or broadcaster failure |
| `FAULT` | Sensor config missing, malformed, or static transform publication failed | Config fixed and node restarted |

### Sensor Nodes

| State | Meaning | Exit |
| --- | --- | --- |
| `INIT` | Parameters and publishers created | Driver/mock configured |
| `ACTIVE` | Publishing data | Driver failure or shutdown |
| `STALE_INPUT` | Driver produced no data within expected period | Data resumes or fault threshold exceeded |
| `FAULT` | Repeated driver failure or invalid data | Manual restart or supervisor restart |

### Odometry Node

| State | Meaning | Exit |
| --- | --- | --- |
| `INIT` | Algorithm adapter loading | Inputs available |
| `WAITING_INPUTS` | Waiting for minimum sensor set | Required inputs arrive |
| `ACTIVE` | Publishing pose | Input stale or algorithm error |
| `DEGRADED` | Publishing lower-confidence or no pose due to partial inputs | Inputs recover or fault threshold exceeded |
| `FAULT` | Algorithm unavailable or transform missing | Restart or operator action |

### VLM Node

| State | Meaning | Exit |
| --- | --- | --- |
| `INIT` | Model/mock loading | Model ready |
| `WAITING_SYNC` | Waiting for image and bounded pose | Sync available |
| `ACTIVE` | Running VLM inference | Rotate requested, stale input, or fault |
| `FROZEN_ROTATING` | Rotate action in progress | Action success, abort, cancel, or timeout |
| `STALE_INPUT` | Image or pose too old | Fresh bounded sync available |
| `FAULT` | Model error, malformed internal output, action client failure | Restart or operator action |

### E2E Node

| State | Meaning | Exit |
| --- | --- | --- |
| `INIT` | Model/mock loading | Model ready |
| `WAITING_FIRST_VLM` | No usable VLM reasoning has arrived | Valid reasoning arrives |
| `ACTIVE` | Running e2e inference for `go` | Stop, stale, malformed input, or fault |
| `STOPPED_BY_VLM` | Latest valid VLM action is `stop` | Valid `go` arrives |
| `DEGRADED` | Cached VLM is stale or pose compensation exceeds bounds | Fresh VLM reasoning arrives |
| `FAULT` | Model error, frame mismatch, invalid trajectory | Restart or operator action |

### Controller Node

| State | Meaning | Exit |
| --- | --- | --- |
| `INIT` | Robot command adapter loading | Adapter ready |
| `WAITING_TRAJECTORY` | No valid trajectory and pose-at-trajectory | Valid trajectory arrives |
| `FOLLOWING` | PID trajectory following | Rotate action, stale input, stop, or fault |
| `ROTATING` | Rotate action owns controller | Action success, cancel, timeout, or abort |
| `STOPPING` | Commanding zero velocity/hold | Safe stop confirmed |
| `DEGRADED` | Missing non-critical data; holding safe command | Data recovers or fault threshold exceeded |
| `FAULT` | Odometry lost, command adapter error, frame mismatch | Restart or operator action |

### Debug Visualizer Node

| State | Meaning | Exit |
| --- | --- | --- |
| `INIT` | Publishers, subscribers, and OpenCV bridge are loading | Subscriptions ready |
| `ACTIVE` | Annotating camera frames with latest debug state | Input stale, overlay error, or shutdown |
| `DEGRADED` | Rendering raw image with partial or stale overlay data | Required debug inputs recover |
| `FAULT` | Image conversion or publisher failure prevents output | Restart or operator action |

## Stale Data and TTL Policy

Default TTL values for mock implementation:

| Data | TTL | Degradation |
| --- | --- | --- |
| Camera image | 2x configured camera period | Drop image-dependent inference |
| Lidar point cloud | 2x configured lidar period | Odometry degraded or waits |
| IMU packet | 3x configured IMU period | Odometry degraded or fault |
| Pose | 0.20 s max age for image sync, 0.10 s for controller | Drop inference or stop controller |
| VLM reasoning | 8.0 s and max compensation distance | E2E enters `DEGRADED` or `STOPPED_BY_VLM` |
| E2E trajectory | 0.50 s | Controller stops following |
| Debug overlay inputs | Render latest values with visible age labels | Visualizer marks stale overlay fields |
| Heartbeat | 3 missed periods | Supervisor marks node unhealthy |

Pose compensation bounds for cached VLM reasoning:

- Max translation since VLM pose: 1.5 m.
- Max yaw change since VLM pose: 30 deg.
- If either bound is exceeded, e2e must not warp the goal indefinitely. It should stop/degrade and wait for new VLM reasoning.

## Rotate Action Semantics

Rotation uses ROS 2 Action, not a bare `rotate_done` topic. The VLM node is the action client. The controller node is the action server.

Action lifecycle:

1. VLM publishes reasoning string containing `action=rotate` and `rotate_deg`.
2. VLM sends `/s2e/controller/rotate` action goal with the requested signed angle.
3. Controller accepts the goal only if odometry is fresh and no higher-priority fault exists.
4. Controller clears stored trajectory and enters `ROTATING`.
5. Controller records start yaw from current pose.
6. Controller commands yaw velocity until yaw delta reaches target within tolerance.
7. Controller requires tolerance to be held for a short settle time.
8. Controller returns success, aborted, canceled, or timeout.
9. VLM leaves `FROZEN_ROTATING` after result.

Recommended defaults:

- Angle tolerance: 3 deg.
- Settle time: 0.30 s.
- Timeout: `max(3.0, abs(target_deg) / 30.0 + 2.0)` seconds for mock control.
- Feedback rate: 10 Hz.

## Watchdog and Supervision

Every node publishes heartbeat/status at 1 Hz on `/s2e/status/<node_name>`, including `supervisor_node` itself. Status uses the `NodeStatus` contract in [interfaces.md](interfaces.md): node state, active mode, input/output ages, health flag, motion-critical flag, error code, and message. In robot/PC split mode the robot-side `supervisor_node` is the authoritative safety monitor. It must receive local robot node heartbeats and remote `/s2e/status/vlm_node` and `/s2e/status/e2e_node` heartbeats over ROS 2, tolerate network partition, and publish `/s2e/supervisor/health` for the controller and e2e cache-invalidation logic. The debug visualizer heartbeat is non-critical: its failure is reported but must not trigger robot motion changes.

Watchdog ownership is layered:

- `supervisor_node`: global node-health watchdog for local robot nodes and remote external-compute nodes.
- `controller_node`: final motion-safety watchdog for odometry staleness, trajectory TTL, rotate timeout, and command adapter faults.
- `vlm_node` and `e2e_node`: data-freshness watchdogs for image/pose/VLM-cache TTL before producing reasoning or trajectories.
- `odometry_node`: sensor-freshness watchdog for lidar/camera/IMU availability before publishing pose.

Robot/PC split remains safe only if the robot side does not depend solely on remote processes for stopping. If the external PC or network disappears, `supervisor_node` marks VLM/e2e unhealthy after missed heartbeats and publishes degraded system health. `controller_node` subscribes to that health topic and also independently stops when trajectory TTL or odometry TTL expires. The controller must be able to hold zero command without receiving any new message from the external PC.

VLM heartbeat loss invalidates cached VLM reasoning for safety. E2E may retain the old string for debug display, but it must not keep producing new trajectories from cached VLM reasoning after `supervisor_node` reports VLM unhealthy or after the VLM TTL expires, whichever happens first.

Production hardware drivers, odometry, and controller nodes should be candidates for ROS 2 lifecycle nodes because they benefit from explicit configure, activate, deactivate, cleanup, and error transitions. Mock nodes may remain ordinary `rclpy` nodes to keep tests simple.

The reference process service pattern in `reference/ete/python/motif_e2e/agent_service.py` shows useful logging and process control ideas, but implementation should prefer ROS 2 launch lifecycle and diagnostics for node health.

## Reference Influence

The design borrows patterns from `reference/` without depending on internal packages:

- Latest-message caching: `reference/agf/agentworks/core/last_msg_store.py` and `reference/ete/python/motif_e2e/util/last_msg_store.py`.
- Periodic cached payload assembly: `reference/agf/agentworks/agent_host_app.py`.
- IPC/TCP and latest-only stream concepts: `reference/base/comm/comm_node.py`, `reference/base/comm/comm_base_sub.py`, and `reference/test/comm/test_pub_sub_ipc.py`. ROS 2 replaces this transport layer, but the freshness principle remains.
- E2E IO shape expectations and model adapter patterns: `reference/ete/README.md`, `reference/ete/python/motif_e2e/agent_e2e.py`, and `reference/ete/python/motif_e2e/motife2e.py`.
- Image preprocessing and calibration flow: `reference/ete/python/motif_e2e/datapipeline/data_preprocessor.py` and `reference/ete/python/motif_e2e/datapipeline/pipeline.py`.
- Coordinate transform boundaries: `reference/base/dal/coord_lib/coords_transformer.py`, `reference/base/dal/coord_lib/utils.py`, `reference/base/dal/frame/keyframe.py`, and `reference/base/dal/geometry/transform/transform.py`.
- Sensor/parser data structure ideas: `reference/base/dal/sensors/camera/data.py`, `reference/base/dal/sensors/lidar/data.py`, `reference/base/dal/parser/camera_data_parser.py`, and `reference/base/dal/parser/lidar_data_parser.py`.
- Controller transforms and PID patterns: `reference/algo/projects/team_code/lateral_controller.py`, `longitudinal_controller.py`, `controller_config.py`, `nav_planner.py`, and `agent_demo_controller.py`.

## Design Caveats

- VLM string schema is intentionally temporary. It must still be strict JSON in version 0.
- Hardware-specific Unitree command messages are not part of the first mock implementation. The controller uses an adapter boundary so real Go2 commands can replace mock commands later.
- Exact sensor extrinsics are not known yet. Static transform publishers and config files must be structured so calibrated transforms can be inserted without changing node logic.
- The debug visualizer is a 2D image-overlay tool, not a replacement for RViz, tf tree inspection, or calibrated 3D visualization.
