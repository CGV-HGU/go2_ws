# Coordinate, Extrinsic, and Visualizer Projection Design

## Context

The single-PC ROS 2 mock graph already exchanges real pub/sub/action traffic, generates smooth VLM image goals, converts those goals into `base_link` trajectory targets, and saves visualizer PNG/MP4 artifacts. The remaining gap is frame correctness: the mock preprocessing maps image-right to positive `base_link.y`, the visualizer mini-map draws positive `y` to the right, and sensor extrinsics are documented but not actually published or consumed.

This design closes that gap while keeping the mock graph lightweight and testable.

## Goals

- Use the documented robot convention: `base_link` has `x+` forward, `y+` left, and `z+` up.
- Use the documented camera optical convention: `camera` has `x+` image-right, `y+` image-down, and `z+` forward.
- Store sensor calibration in one config directory with one YAML file per sensor.
- Parse extrinsics and camera intrinsics through one shared parser used by all relevant nodes.
- Publish concrete static extrinsics for `base_link -> camera`, `base_link -> lidar`, and `base_link -> imu` on ROS `/tf_static`.
- Make the camera node publish `CameraInfo` from the camera YAML intrinsics.
- Make the visualizer project `base_link` trajectory points into the camera image using parsed/published `CameraInfo.K` and the static transform.
- Preserve the mini-map as a fallback/debug view, but fix its handedness so `y+` appears left when `x+` points upward.
- Keep debug visualization non-critical: TF/projection failures should never stop the graph or controller.

## Non-Goals

- Do not add real Unitree Go2 calibration values yet. The mock extrinsics are deterministic placeholders that can be replaced later.
- Do not introduce a custom calibration service or database; YAML files are enough for the mock and first robot integration stage.
- Do not change public motion message frames; trajectories remain ego-centric in `base_link`.
- Do not make visualizer output a motion dependency.

## Recommended Approach

Use real ROS static TF, not a visualizer-local shortcut.

The mock graph will add a `static_tf_node` executable that reads sensor YAML files through a shared parser and publishes fixed transforms with `tf2_ros.StaticTransformBroadcaster`. This satisfies the documented `/tf_static` requirement and lets future robot/PC split deployments use the same interface. Sensor nodes will also use the same parser for frame names and calibration data. The camera node will publish `CameraInfo` from `camera.yaml`; the visualizer will use `tf2_ros.Buffer` and `TransformListener` to lookup `camera <- base_link`, then project trajectory points into the image with the received `CameraInfo` intrinsics.

Rejected alternatives:

- Direct YAML-only visualizer projection: simpler, but it bypasses ROS TF and would not validate the interface the real graph needs. The visualizer may use the shared parser only for fallback metadata, not as the authoritative transform path.
- Launching only `tf2_ros static_transform_publisher` CLI actions: acceptable for deployment, but harder to unit-test consistently in this Python subprocess graph and does not centralize mock extrinsic parsing.

## Sensor Configuration

Sensor calibration will live under `src/s2e_vlm_bringup/config/sensors/` with one YAML file per physical sensor:

- `camera.yaml`
- `lidar.yaml`
- `imu.yaml`

Each YAML file will include:

- `parent_frame`
- `child_frame`
- `translation_m`: `[x, y, z]`
- an explicit rotation, preferably `rotation_matrix_row_major` or `rotation_quaternion_xyzw`

Camera YAML will also include intrinsic calibration:

- `image_width`
- `image_height`
- `distortion_model`
- `camera_matrix_row_major`
- `distortion_coefficients`
- optional `rectification_matrix_row_major`
- optional `projection_matrix_row_major`

The config should avoid RPY as the authoritative representation for the camera optical frame because Euler convention mistakes are common. If RPY is accepted later for human convenience, tests must still verify the resulting transform matrix.

Initial mock values:

- `base_link -> camera`: translation `[0.25, 0.0, 0.35]`, rotation from base axes to optical axes such that base `x+` maps to camera `z+`, base `y+` maps to camera `x-`, and base `z+` maps to camera `y-`. Expressed as child camera axes in parent `base_link`, the row-major rotation matrix is `[0, 0, 1, -1, 0, 0, 0, -1, 0]`. Camera intrinsics default to the current `640x480` mock image with `fx=640`, `fy=480`, `cx=320`, and `cy=240`.
- `base_link -> lidar`: identity rotation with a small forward/up offset.
- `base_link -> imu`: identity rotation near the body center.

The sensor config parser will live in `s2e_vlm_core` so `s2e_vlm_nodes` can import it without depending on bringup internals. It will first try to load installed files from `share/s2e_vlm_bringup/config/sensors`. For local fallback tests, it may also load the source-tree config path when available.

The parser will expose typed results for:

- sensor name
- parent/child frames
- translation
- quaternion converted from matrix or explicit quaternion
- optional camera intrinsic fields

All nodes that need sensor calibration must use this parser instead of hard-coded calibration literals. In the first implementation this means `static_tf_node`, `camera_node`, `lidar_node`, `imu_node`, and `debug_visualizer_node` where applicable.

## Coordinate Fixes

`image_goal_to_base_link()` will keep the existing bounded heuristic, but lateral sign changes:

- Image center maps to `(forward, 0)`.
- Image-right maps to negative `base_link.y`.
- Image-left maps to positive `base_link.y`.
- Image-top still maps farther forward than image-center.

The controller already turns based on `goal_point_base_link.y`; after this fix, right-side image goals produce negative lateral targets and the command sign will reflect the corrected convention.

The visualizer mini-map will map metric points as:

- pixel `y` decreases as `base_link.x` increases.
- pixel `x` decreases as `base_link.y` increases.

Labels will make the convention explicit: `+x forward`, `+y left`, `-y right`.

## Visualizer Projection Flow

The debug visualizer will keep the latest camera image, latest `CameraInfo`, latest VLM JSON, latest trajectory, and parsed sensor config metadata. For each overlay cycle:

1. Draw text and VLM `goal_uv` marker as today.
2. Draw the corrected mini-map if a trajectory exists.
3. If `CameraInfo` and `camera <- base_link` transform are available, transform each trajectory point `(x, y, 0)` into the camera optical frame.
4. Reject points behind the camera or outside finite projection bounds.
5. Project valid points with `u = fx * X / Z + cx`, `v = fy * Y / Z + cy`.
6. Draw the projected polyline and endpoint on the camera image.
7. If projection is unavailable, draw `projection unavailable` and continue publishing.

Because the mock trajectory lies on the ground plane and the camera is above the ground, some points may fall out of frame depending on mock calibration and goal. This is acceptable if at least normal smooth-goal artifact runs produce visible projected segments and tests verify the projection path.

## Tests

Add or update tests at three levels:

- Core unit tests: verify image-right maps to negative `base_link.y`, image-left maps positive, center remains zero lateral, and forward bounds remain unchanged.
- Parser tests: verify the parser loads per-sensor YAML files, converts camera rotation to the expected quaternion/axis mapping, returns camera intrinsics, and rejects malformed calibration shapes.
- ROS graph tests: verify `/tf_static` publishes transforms for `camera`, `lidar`, and `imu`; verify `CameraInfo.K` matches `camera.yaml`; verify the debug visualizer receives camera info and publishes overlays while TF is present.
- Artifact/projection test: in the smooth-goal run, assert generated manifest records projection availability and at least one projected trajectory segment. Keep PNG/MP4 checks.

Fallback behavior should also be covered: if TF lookup fails, visualizer still publishes a debug image and marks projection unavailable instead of failing.

## Verification Commands

After implementation:

```bash
python3 -m unittest discover -s src/s2e_vlm_core/test -p 'test_*.py' -v
python3 -m unittest discover -s src/s2e_vlm_nodes/test -p 'test_*.py' -v
python3 -m unittest src/s2e_vlm_bringup/test_launch_contracts.py -v
python3 -m unittest tests/test_docker_assets.py -v
python3 -m compileall -q src tests
docker compose build ros-base
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose"
```

Regenerate a 10-second visualizer artifact under `artifacts/` and confirm the manifest reports `projection_available=true` and nonzero projected trajectory frames.

## Acceptance Criteria

- Real ROS mock graph includes a running static TF publisher node.
- Sensor config lives in `config/sensors/*.yaml`, one file per sensor.
- All calibration consumers use the shared parser instead of duplicating hard-coded extrinsic/intrinsic values.
- `/tf_static` exposes `base_link -> camera`, `base_link -> lidar`, and `base_link -> imu`.
- `/s2e/sensors/camera/camera_info` uses intrinsics from `camera.yaml`.
- E2E preprocessing follows the corrected `base_link` handedness.
- Visualizer overlays both the VLM image goal and camera-projected trajectory when TF and intrinsics are available.
- Mini-map convention matches `x+` forward/up and `y+` left.
- Existing pub/sub/action tests, dummy integration coverage, Docker tests, and artifact generation still pass.
