# ROS2 Async VLM/E2E Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python ROS 2 package set that runs mock sensor, odometry, VLM, e2e, controller, supervisor, and debug visualizer nodes with strict async timing, frame, state, TTL, watchdog, rotate-action, and overlay-debug contracts.

**Architecture:** The implementation creates ROS 2 interface, core utility, node, algorithm-adapter, bringup, and test packages. Mock algorithms are implemented first so the full graph can be verified on one PC before Unitree Go2 hardware or real VLM/e2e/LVIO models are integrated.

**Tech Stack:** ROS 2 Jazzy or newer unless the project pins another distro, Python 3.10+, `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `diagnostic_msgs`, `tf2_ros`, `cv_bridge`, OpenCV, optional `message_filters`, `launch`, `launch_testing`, `pytest`, `numpy`.

---

## Planned Package Layout

- Create `s2e_vlm_msgs`: custom messages/actions for timestamps, trajectory, status, and rotate.
- Create `s2e_vlm_core`: pure-Python utilities for time, pose buffers, JSON parsing, state machines, transforms, caches, and adapters.
- Create `s2e_vlm_nodes`: ROS 2 Python nodes for sensors, odometry, VLM, e2e, controller, supervisor, and debug visualization.
- Create `s2e_vlm_bringup`: launch files for single-PC mock mode and future robot/PC split mode.
- Create `tests`: unit tests and launch tests.

## Task 1: Create ROS 2 Workspace Package Skeleton

**Files:**

- Create: `src/s2e_vlm_msgs/package.xml`
- Create: `src/s2e_vlm_msgs/CMakeLists.txt`
- Create: `src/s2e_vlm_core/package.xml`
- Create: `src/s2e_vlm_core/setup.py`
- Create: `src/s2e_vlm_core/s2e_vlm_core/__init__.py`
- Create: `src/s2e_vlm_nodes/package.xml`
- Create: `src/s2e_vlm_nodes/setup.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/__init__.py`
- Create: `src/s2e_vlm_bringup/package.xml`
- Create: `src/s2e_vlm_bringup/setup.py`
- Create: `src/s2e_vlm_bringup/launch/single_pc_mock.launch.py`

- [ ] **Step 1: Create package directories**

Run:

```bash
mkdir -p src/s2e_vlm_msgs/msg src/s2e_vlm_msgs/action
mkdir -p src/s2e_vlm_core/s2e_vlm_core
mkdir -p src/s2e_vlm_nodes/s2e_vlm_nodes
mkdir -p src/s2e_vlm_bringup/launch
```

Expected: directories exist.

- [ ] **Step 2: Add minimal package metadata**

Write package metadata with `ament_python` for Python packages and `rosidl_default_generators` for `s2e_vlm_msgs`.

- [ ] **Step 3: Build skeleton**

Run:

```bash
colcon build --symlink-install
```

Expected: build exits 0.

## Task 2: Define Message and Action Interfaces

**Files:**

- Create: `src/s2e_vlm_msgs/msg/StampedPose.msg`
- Create: `src/s2e_vlm_msgs/msg/Trajectory2D.msg`
- Create: `src/s2e_vlm_msgs/msg/NodeStatus.msg`
- Create: `src/s2e_vlm_msgs/msg/SystemHealth.msg`
- Create: `src/s2e_vlm_msgs/action/Rotate.action`
- Modify: `src/s2e_vlm_msgs/CMakeLists.txt`

- [ ] **Step 1: Define messages exactly from docs/interfaces.md**

Include `source_stamp`, `processed_stamp`, frame fields, concrete `NodeStatus`, `SystemHealth`, and 10-point trajectory representation with `goal_point_base_link` and `has_goal_point`.

- [ ] **Step 2: Define rotate action**

Use the action contract in [docs/interfaces.md](../../interfaces.md): goal target angle, max yaw rate, tolerance, timeout; result success/result code/final delta; feedback current delta/remaining/controller state.

- [ ] **Step 3: Build and inspect interfaces**

Run:

```bash
colcon build --symlink-install --packages-select s2e_vlm_msgs
source install/setup.bash
ros2 interface show s2e_vlm_msgs/action/Rotate
```

Expected: action definition prints with goal/result/feedback sections.

## Task 3: Implement Core Time and Pose Utilities

**Files:**

- Create: `src/s2e_vlm_core/s2e_vlm_core/time_utils.py`
- Create: `src/s2e_vlm_core/s2e_vlm_core/pose_buffer.py`
- Create: `src/s2e_vlm_core/s2e_vlm_core/transforms_2d.py`
- Test: `src/s2e_vlm_core/test/test_pose_buffer.py`
- Test: `src/s2e_vlm_core/test/test_transforms_2d.py`

- [ ] **Step 1: Write tests for latest-pose-before-target**

Cover exact hit, latest-before hit, max-age rejection, empty buffer, and out-of-order insert sorting.

- [ ] **Step 2: Implement pose buffer**

Store bounded pose samples by timestamp and return lookup metadata: pose, age, fresh/degraded flag, and rejection reason.

- [ ] **Step 3: Implement 2D relative transform helpers**

Implement `relative_pose_2d(a, b)`, `transform_point_2d(point, relative)`, yaw wrapping, and translation/yaw bound checks.

- [ ] **Step 4: Run tests**

Run:

```bash
pytest src/s2e_vlm_core/test/test_pose_buffer.py src/s2e_vlm_core/test/test_transforms_2d.py -v
```

Expected: all tests pass.

## Task 4: Implement VLM String Parser and Cache

**Files:**

- Create: `src/s2e_vlm_core/s2e_vlm_core/vlm_schema.py`
- Create: `src/s2e_vlm_core/s2e_vlm_core/latest_store.py`
- Test: `src/s2e_vlm_core/test/test_vlm_schema.py`
- Test: `src/s2e_vlm_core/test/test_latest_store.py`

- [ ] **Step 1: Write parser tests**

Cover valid `go`, `stop`, `rotate`, malformed JSON, missing `goal_uv`, unknown action, and schema version mismatch.

- [ ] **Step 2: Implement strict parser**

Return typed parse result with action enum, stamps, pose snapshot, goal, rotate angle, reasoning, and validity status. Invalid payloads return `NO_COMMAND` with reason.

- [ ] **Step 3: Implement latest store**

Use `threading.Lock` and `threading.Condition` pattern inspired by `reference/agf/agentworks/core/last_msg_store.py`, with timeout-aware `get`, `put`, `pop`, and `wait_for`.

- [ ] **Step 4: Run tests**

Run:

```bash
pytest src/s2e_vlm_core/test/test_vlm_schema.py src/s2e_vlm_core/test/test_latest_store.py -v
```

Expected: all tests pass.

## Task 5: Implement Mock Algorithm Adapters

**Files:**

- Create: `src/s2e_vlm_core/s2e_vlm_core/algorithms.py`
- Create: `src/s2e_vlm_core/s2e_vlm_core/mock_algorithms.py`
- Test: `src/s2e_vlm_core/test/test_mock_algorithms.py`

- [ ] **Step 1: Define adapter protocols**

Define interfaces for sensor mocks, odometry estimator, VLM reasoner, e2e planner, and controller command generator.

- [ ] **Step 2: Implement deterministic mocks**

Use rates and output shapes in [docs/testing.md](../../testing.md). E2E mock returns exactly 10 points. VLM mock can be configured for `go`, `stop`, malformed, and `rotate` scenarios. Document the mock `goal_uv` to `goal_point_base_link` mapping in config and keep it replaceable by calibrated real adapters.

- [ ] **Step 3: Run tests**

Run:

```bash
pytest src/s2e_vlm_core/test/test_mock_algorithms.py -v
```

Expected: deterministic outputs match expected stamps, shapes, and actions.

## Task 6: Implement ROS 2 Nodes

**Files:**

- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/lidar_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/camera_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/imu_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/odometry_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/vlm_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/e2e_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/controller_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/supervisor_node.py`
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/debug_visualizer_node.py`
- Modify: `src/s2e_vlm_nodes/setup.py`

- [ ] **Step 1: Implement sensor nodes**

Publish mock `PointCloud2`, `Image`, and `Imu` with independent timers and sensor-data QoS.

- [ ] **Step 2: Implement odometry node**

Subscribe to sensor streams, feed mock odometry adapter, and publish `StampedPose` with public `base_link` pose.

- [ ] **Step 3: Implement VLM node**

Maintain pose buffer, sync image to bounded pose, publish strict JSON reasoning, and use rotate action client.

- [ ] **Step 4: Implement E2E node**

Maintain pose buffer and latest VLM cache, enforce TTL, supervisor health, and compensation bounds, publish `Trajectory2D` only for valid `go`, and publish motion-blocking status for `stop`, stale VLM, invalid VLM, or missing calibration.

- [ ] **Step 5: Implement controller node**

Maintain state machine, follow latest valid trajectory, serve rotate action, clear trajectory on rotate, abort on stale odometry, subscribe to `/s2e/e2e/status` and `/s2e/supervisor/health`, stop/hold on motion-blocking status, and publish mock `Twist` commands.

- [ ] **Step 6: Implement supervisor node**

Publish `/s2e/status/supervisor_node`, track `NodeStatus` heartbeats from local and remote nodes, and publish `/s2e/supervisor/health`. In robot/PC split mode, the robot-side supervisor monitors remote `/s2e/status/vlm_node` and `/s2e/status/e2e_node` as safety-critical external-compute heartbeats and sets `ok_to_move=false` when they disappear.

- [ ] **Step 7: Implement debug visualizer node**

Subscribe to camera image, optional camera info, VLM reasoning, e2e trajectory, e2e status, controller status, and node status streams. Use `cv_bridge` and OpenCV to publish `/s2e/debug/visualizer/image` with action text, coarse `goal_uv`, fine `goal_point_base_link`, trajectory mini-map, controller/e2e state, rotate progress, and stale-input labels. The visualizer must never publish commands or affect control state.

- [ ] **Step 8: Build nodes**

Run:

```bash
colcon build --symlink-install --packages-select s2e_vlm_core s2e_vlm_nodes
```

Expected: build exits 0.

## Task 7: Implement Bringup Launch Files

**Files:**

- Create: `src/s2e_vlm_bringup/launch/single_pc_mock.launch.py`
- Create: `src/s2e_vlm_bringup/launch/robot_side.launch.py`
- Create: `src/s2e_vlm_bringup/launch/external_pc.launch.py`
- Create: `src/s2e_vlm_bringup/config/mock_rates.yaml`
- Create: `src/s2e_vlm_bringup/config/ttl.yaml`
- Create: `src/s2e_vlm_bringup/config/static_frames.yaml`

- [ ] **Step 1: Implement single-PC launch**

Launch all mock nodes, including the debug visualizer by default, in one ROS 2 domain with screen output.

- [ ] **Step 2: Implement split launch files**

Robot-side launch starts sensor, odometry, controller, supervisor, and optional debug visualizer. External-PC launch starts VLM and e2e. The visualizer may run on any host that can receive the camera image and debug topics.

- [ ] **Step 3: Run single-PC launch manually**

Run:

```bash
source install/setup.bash
ros2 launch s2e_vlm_bringup single_pc_mock.launch.py
```

Expected: all nodes start and publish heartbeat.

- [ ] **Step 4: Verify QoS compatibility**

Run:

```bash
ros2 topic info --verbose /s2e/sensors/camera/image
ros2 topic info --verbose /s2e/odometry/pose
ros2 topic info --verbose /s2e/e2e/trajectory
ros2 topic info --verbose /s2e/debug/visualizer/image
```

Expected: publishers and subscribers are discovered with compatible QoS.

## Task 8: Implement Launch Tests and Failure Injection

**Files:**

- Create: `src/s2e_vlm_nodes/test/test_single_pc_launch.py`
- Create: `src/s2e_vlm_nodes/test/test_rotate_action.py`
- Create: `src/s2e_vlm_nodes/test/test_failure_modes.py`
- Create: `src/s2e_vlm_nodes/test/test_debug_visualizer.py`

- [ ] **Step 1: Test happy path graph**

Use `launch_testing` to start `single_pc_mock.launch.py`, wait for topics, assert VLM reasoning, e2e trajectory, and debug visualizer image arrive.

- [ ] **Step 2: Test rotate action**

Send rotate goal, assert controller status enters `ROTATING`, action returns success in mock mode, and trajectory is cleared.

- [ ] **Step 3: Test failure modes**

Inject malformed VLM string, stale pose, frame mismatch, and heartbeat loss. Assert safe degradation or stop/hold.

- [ ] **Step 4: Test debug visualizer behavior**

Feed valid, stale, and malformed debug inputs. Assert annotated images continue publishing, stale labels are visible through testable overlay metadata or mock hooks, and no command topics are published by the visualizer.

- [ ] **Step 5: Run full tests**

Run:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all tests pass.

## Task 9: Update Documentation After Implementation

**Files:**

- Modify: `README.md`
- Modify: `docs/architecture.md`
- Modify: `docs/interfaces.md`
- Modify: `docs/testing.md`
- Modify: `docs/sequence_diagrams.md`

- [ ] **Step 1: Replace planned commands with verified commands**

Run every command listed in `README.md` and `docs/testing.md`, then update expected outputs with observed names and topic types.

- [ ] **Step 2: Add hardware integration notes**

Document which adapter files must be replaced for Unitree Go2 sensors and command bridge.

- [ ] **Step 3: Run final verification**

Run:

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build and tests pass.

## Task 10: Add Docker and Split-Deployment Assets

**Files:**

- Create: `docker/ros-base.Dockerfile`
- Create: `docker/dev-mock.Dockerfile`
- Create: `docker/robot.Dockerfile`
- Create: `docker/gpu-inference-base.Dockerfile`
- Create: `docker/vlm.Dockerfile`
- Create: `docker/e2e.Dockerfile`
- Create: `compose.yaml`
- Create: `.env.example`

- [ ] **Step 1: Pin compatibility matrix**

Document ROS 2 Jazzy/Ubuntu 24.04 for CPU packages and the separate Ubuntu 22.04/CUDA 11.8/TensorRT path for GPU runtime containers if required.

- [ ] **Step 2: Implement Compose profiles**

Create `single_pc_mock`, `single_pc_split`, `robot_side`, and `external_gpu` profiles with explicit GPU reservations only for VLM/e2e services.

- [ ] **Step 3: Add split-deployment smoke checks**

Verify DDS discovery, `/s2e/status/vlm_node`, `/s2e/status/e2e_node`, `/s2e/supervisor/health`, trajectory flow, and rotate action across containers or machines before hardware commands are enabled.

## Self-Review

- Spec coverage: The plan covers package skeleton, interfaces, time/frame utilities, VLM string parsing, latest cache, mock algorithms, all nodes including debug visualization, launch files, failure tests, and documentation updates.
- Placeholder scan: No task depends on undefined future decisions for the mock implementation. Hardware-specific Unitree adapters are intentionally behind adapter boundaries and documented as later replacements.
- Type consistency: The plan consistently uses `StampedPose`, `Trajectory2D`, `NodeStatus`, `SystemHealth`, and `Rotate.action` as defined in [docs/interfaces.md](../../interfaces.md).
