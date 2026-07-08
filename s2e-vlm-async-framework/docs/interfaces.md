# Interfaces

## Naming

All project topics and actions use `/s2e` prefix.

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

Actions:

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

## Stamp Contract

Every message carrying sensor-derived data follows this contract:

- `header.stamp`: acquisition/reference time of the represented data.
- `header.frame_id`: coordinate frame of the represented data.
- `source_stamp`: original upstream acquisition/reference time if the message is a derived wrapper.
- `processed_stamp`: time the publishing node produced the output.

For standard ROS messages that do not contain `source_stamp` or `processed_stamp`, the implementation should create project wrapper messages in `s2e_vlm_msgs`. Until custom wrappers exist, docs and logs must treat `header.stamp` as source time and measure publish latency externally.

## Topic Contracts

### `/tf_static`

Recommended type: `tf2_msgs/msg/TFMessage` published through `tf2_ros.StaticTransformBroadcaster`.

Contract:

- Publisher: `static_tf_node` on robot/sensor side.
- Required transforms: `base_link -> camera`, `base_link -> lidar`, and `base_link -> imu`.
- Source config: `s2e_vlm_bringup/config/sensors/*.yaml`, parsed through `s2e_vlm_core.sensor_config`.
- QoS: ROS 2 static TF defaults, transient local and reliable.

### `/s2e/sensors/lidar/points`

Recommended type: `sensor_msgs/msg/PointCloud2` for version 0.

Contract:

- `header.stamp`: lidar acquisition time.
- `header.frame_id`: `lidar`.
- QoS: sensor data, best effort, volatile, depth 5.
- Point fields: implementation-specific, but mock publishes `x`, `y`, `z`, `intensity`.

### `/s2e/sensors/camera/image`

Recommended type: `sensor_msgs/msg/Image` with optional `sensor_msgs/msg/CameraInfo` on `/s2e/sensors/camera/camera_info`.

Contract:

- `header.stamp`: image acquisition time.
- `header.frame_id`: `camera`.
- QoS: sensor data, best effort, volatile, depth 5.
- Mock encoding: `rgb8`.

### `/s2e/sensors/imu`

Recommended type: `sensor_msgs/msg/Imu`.

Contract:

- `header.stamp`: IMU acquisition time.
- `header.frame_id`: `imu`.
- QoS: sensor data, best effort, volatile, depth 50.

### `/s2e/odometry/pose`

Recommended type for version 0: `nav_msgs/msg/Odometry` when twist is available; otherwise the project `s2e_vlm_msgs/msg/StampedPose`.

Project custom wrapper: `s2e_vlm_msgs/msg/StampedPose`.

Contract:

- `header.stamp`: pose reference time.
- `header.frame_id`: `odom` or `map`, the parent frame.
- `child_frame_id`: `base_link` when using `Odometry` or `StampedPose`.
- Pose represents `base_link` in the parent frame.
- Public pose is never IMU-frame pose.
- QoS: reliable, volatile, depth 50.

`StampedPose` fields:

```text
std_msgs/Header header
builtin_interfaces/Time source_stamp
builtin_interfaces/Time processed_stamp
string child_frame_id
geometry_msgs/Pose pose
float32 confidence
string status
```

If using `nav_msgs/Odometry`, pose is expressed in `header.frame_id` and twist is expressed in `child_frame_id`. Implementations must not leave `child_frame_id` empty.

### `/s2e/vlm/reasoning`

Version 0 type: `std_msgs/msg/String`.

Contract:

- Payload is strict JSON.
- Invalid JSON, missing required fields, unknown schema version, or invalid action is treated as `NO_COMMAND` by consumers.
- Consumers must not use best-effort string matching to create motion.
- QoS: reliable, transient local may be used for late joiners, depth 5.

Required JSON schema version 0:

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

Allowed actions:

- `stop`: e2e does not produce a new trajectory; controller stops/holds if no valid trajectory mode remains.
- `go`: e2e converts `goal_uv` to `base_link` 2D and runs inference.
- `rotate`: VLM sends rotate action goal using `rotate_deg` and freezes until action result.

Field rules:

- `stamp` is the synchronized image/pose reference time used by VLM.
- `goal_uv` is required for `go`.
- `rotate_deg` is required and non-zero for `rotate`.
- `pose` is the VLM pose snapshot at `stamp`.

### `/s2e/e2e/trajectory`

Recommended future type: `s2e_vlm_msgs/msg/Trajectory2D`.

Version 0 may use `std_msgs/msg/Float32MultiArray` only for mock smoke tests, but docs and tests should target the custom message.

Contract:

- `header.stamp`: input image timestamp used for e2e inference.
- `header.frame_id`: `base_link`.
- `source_stamp`: same as input image timestamp.
- `processed_stamp`: e2e output time.
- `pose_at_trajectory`: `base_link` pose in `odom` or `map` at the trajectory start.
- `points`: exactly 10 ego-centric 2D points.
- `goal_point_base_link`: fine e2e goal point after VLM `goal_uv` conversion and pose compensation.
- `has_goal_point`: false when trajectory was produced without a usable explicit goal.
- QoS: reliable, volatile, depth 10.

Proposed message:

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

Only `x` and `y` of `Point32` are used; `z` is always `0.0`.

### `/s2e/controller/command`

Version 0 mock type: `geometry_msgs/msg/Twist`.

Contract:

- Linear `x` is forward velocity command.
- Angular `z` is yaw rate command.
- During real Unitree Go2 integration, this topic is replaced or bridged by a robot command adapter. Controller authority semantics do not change.

### `/s2e/status/<node_name>`

Type: `s2e_vlm_msgs/msg/NodeStatus`.

Required topics:

- `/s2e/status/lidar_node`
- `/s2e/status/camera_node`
- `/s2e/status/imu_node`
- `/s2e/status/odometry_node`
- `/s2e/status/vlm_node`
- `/s2e/status/e2e_node`
- `/s2e/status/controller_node`
- `/s2e/status/supervisor_node`
- `/s2e/status/debug_visualizer_node`

Message:

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

Contract:

- `header.stamp`: status publish time.
- `state`: one of the node states documented in [architecture.md](architecture.md), or `UNKNOWN` during startup.
- `active_mode`: node-specific mode such as `FOLLOWING`, `ROTATING`, `STOPPED_BY_VLM`, `DEGRADED`, or empty when not applicable.
- `is_healthy=false` means the publisher knows it is degraded or faulted.
- `is_motion_critical=true` for odometry, controller, supervisor, and external VLM/e2e heartbeats in robot/PC split mode.
- QoS: reliable, volatile, depth 5.

### `/s2e/supervisor/health`

Type: `s2e_vlm_msgs/msg/SystemHealth`.

Message:

```text
std_msgs/Header header
bool ok_to_move
string overall_state
string[] unhealthy_nodes
string[] missing_critical_nodes
string reason
```

Contract:

- Publisher: robot-side `supervisor_node`.
- Subscribers: `controller_node`, `e2e_node`; optional debug visualizer.
- `ok_to_move=false` is motion-blocking and controller must stop/hold regardless of current trajectory freshness.
- `e2e_node` must invalidate cached VLM reasoning and publish no new trajectory when `vlm_node` is listed in `unhealthy_nodes` or `missing_critical_nodes`, or when `ok_to_move=false`.
- Missing critical nodes include odometry, controller self-health, supervisor self-health, and remote VLM/e2e heartbeats in robot/PC split mode.
- Debug-only node failures, including `debug_visualizer_node`, must not set `ok_to_move=false`.
- QoS: reliable, volatile, depth 5.

### `/s2e/controller/status` and `/s2e/e2e/status`

Version 0 type: `s2e_vlm_msgs/msg/NodeStatus`. These topics use the same base fields as `/s2e/status/<node_name>` and add mode-specific meaning through `active_mode`, `error_code`, and `message`.

Required status concepts:

- Node state and active mode.
- Last input age and last output age.
- Error code and human-readable message.
- Fresh/reused/degraded/invalid output status encoded in `active_mode` or `error_code`.
- Controller rotate fields when applicable: active rotate goal, current yaw delta, remaining yaw, last rotate result code, and whether the last rotate action completed successfully.
- E2E debug fields when applicable: source VLM stamp, cached VLM age, coarse `goal_uv`, fine `goal_point_base_link`, and whether the trajectory reused cached reasoning.
- `STOPPED_BY_VLM`, `INVALID_VLM`, `VLM_STALE`, or `SUPERVISOR_BLOCKED` are motion-blocking states for the controller.

### `/s2e/debug/visualizer/image`

Recommended type: `sensor_msgs/msg/Image`.

Contract:

- Publisher: `debug_visualizer_node`.
- `header.stamp`: copied from the source camera image used for the overlay.
- `header.frame_id`: copied from the source camera image.
- Encoding: `rgb8` for mock mode; implementation may use `bgr8` internally with OpenCV but the published encoding must be documented and stable.
- QoS: sensor data, best effort, volatile, depth 2.
- The raw `/s2e/sensors/camera/image` topic is never modified.
- Missing debug inputs are shown as stale or unavailable text on the overlay; missing debug inputs must not prevent republishing the annotated image.

Required visualizer subscriptions:

| Input | Purpose | QoS |
| --- | --- | --- |
| `/s2e/sensors/camera/image` | Base image for overlay | Sensor data QoS |
| `/s2e/sensors/camera/camera_info` | Optional projection calibration | Reliable or sensor-compatible |
| `/s2e/vlm/reasoning` | VLM action, reasoning text, coarse `goal_uv`, rotate request | Reliable, match publisher |
| `/s2e/e2e/trajectory` | Trajectory and fine `base_link` goal | Reliable, match publisher |
| `/s2e/e2e/status` | E2E state and cache/debug metadata | Reliable, match publisher |
| `/s2e/controller/status` | Controller mode, rotate progress, remaining yaw, last rotate result | Reliable, match publisher |
| Explicit `/s2e/status/<node_name>` topics | Node health overlays | Reliable, match publisher |
| `/s2e/supervisor/health` | Overall motion permission and missing critical nodes | Reliable, match publisher |

Overlay rules:

- Draw VLM `goal_uv` as the coarse image-space goal when present.
- Draw `goal_point_base_link` and trajectory points in a metric mini-map by default.
- Project `base_link` points into the image only when camera calibration and transforms are valid for the image timestamp.
- Render malformed VLM JSON as invalid debug text only; visualizer parsing must not create motion commands.
- Show rotate completion from controller status or action result state, not from an independent `rotate_done` topic.
- When `S2E_DEBUG_MODE=1`, render a compact diagnostics panel with runtime role, ROS domain/RMW, supervisor health, missing/unhealthy nodes, selected heartbeat ages, VLM parse state, e2e/controller mode, and projection status. The same debug snapshot should be persisted in the visualizer artifact manifest when artifact saving is enabled.

## Rotate Action Contract

Recommended action: `s2e_vlm_msgs/action/Rotate.action`.

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

Server behavior:

- Reject if controller is in `FAULT`.
- Reject if odometry is stale.
- Reject if another rotate action is active.
- On accept, clear trajectory and enter `ROTATING`.
- Return `success=false` with `result_code=TIMEOUT`, `ODOM_STALE`, `CANCELED`, or `COMMAND_ERROR` when appropriate.

Client behavior:

- VLM enters `FROZEN_ROTATING` after accepted goal.
- VLM exits freeze on result, cancellation, abort, or timeout.
- VLM must not repeatedly send the same rotate goal while one is active.

## QoS Defaults

| Stream | Reliability | Durability | Depth | Reason |
| --- | --- | --- | --- | --- |
| Lidar/camera/IMU | Best effort | Volatile | 5-50 | Prefer latest sensor data over backlog |
| Pose | Reliable | Volatile | 50 | Pose is safety-critical and used for interpolation |
| VLM reasoning | Reliable | Transient local or volatile | 5 | Late e2e joiner may need latest reasoning |
| E2E trajectory | Reliable | Volatile | 10 | Controller should receive each trajectory but not stale backlog |
| Controller command | Reliable | Volatile | 1 | Only latest command matters |
| Status/heartbeat | Reliable | Volatile | 5 | Health monitoring |
| Supervisor health | Reliable | Volatile | 5 | Motion permission must reach controller |
| Debug visualizer image | Best effort | Volatile | 2 | Human debug stream should not backlog or affect control |
| Rotate action | Reliable | Action default | Action default | Long-running goal/result semantics |

During testing, run `ros2 topic info --verbose <topic>` for every cross-node topic. A QoS mismatch that prevents communication is a test failure.

## Latest-Pose-Before-Image Lookup

Consumers maintain a bounded pose buffer ordered by `header.stamp`.

Algorithm:

1. Receive target image timestamp `t_img`.
2. Find pose samples `p_i` where `p_i.header.stamp <= t_img`.
3. Select the latest `p_i`.
4. Reject if `t_img - p_i.header.stamp > max_pose_age`.
5. If interpolation is available and a pose after `t_img` exists, interpolate instead of selecting latest-before.
6. Return pose with metadata `fresh` or `degraded`.

Default `max_pose_age`:

- VLM/e2e image sync: `0.20 s`.
- Controller control loop: `0.10 s`.

## Frame Mismatch Behavior

Frame mismatch is a safety error, not a warning.

- If trajectory frame is not `base_link`, controller rejects the trajectory.
- If pose parent frame changes unexpectedly between `odom` and `map`, consumers clear cached transforms and wait for coherent data.
- If required static transform is missing, the node enters `FAULT` or `DEGRADED` depending on whether motion could be commanded.

## Multi-Machine ROS 2 Contract

Robot and external PC launch profiles must document and export the same ROS domain and compatible middleware.

Required environment variables:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

Docker Compose reads these values from `.env` or `.env.example` and applies them to `robot-core`, `vlm`, and `e2e` services. Native two-machine runs must export matching values before launching either side.

Additional deployment requirements:

- Both machines must have synchronized clocks through NTP, PTP, or chrony.
- Firewalls must allow DDS discovery and data traffic.
- Docker containers must use networking that permits DDS discovery or explicit peers.
- All machines must source compatible workspaces with the same `s2e_vlm_msgs` interface definitions.
- Robot-side `supervisor_node` is the authoritative safety monitor for split deployment. It must receive `/s2e/status/vlm_node` and `/s2e/status/e2e_node` from the external PC and report degraded health when those heartbeats disappear.
- `controller_node` must enforce local odometry and trajectory TTLs even if supervisor messages are delayed or unavailable.
- `ROS_DOMAIN_ID` is not a security control. Real robot deployments must run on an isolated network or firewall allowlist and use DDS Security/SROS2 or equivalent authenticated transport for motion-relevant topics and actions.
- A split deployment must be tested by running `ros2 topic list`, `ros2 topic info --verbose`, and a rotate action goal across machines before connecting real robot commands.
