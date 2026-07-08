# Coordinate Extrinsic Visualizer Projection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add per-sensor calibration YAML, shared parsing, real ROS static TF, corrected coordinate handedness, and camera-image trajectory projection for the mock graph.

**Architecture:** Calibration lives under `s2e_vlm_bringup/config/sensors/`, one YAML per sensor. `s2e_vlm_core.sensor_config` parses and validates those files so sensor nodes, static TF, and visualizer share one calibration path. `s2e_vlm_nodes.ros_mock_runtime` consumes parsed calibration, publishes `/tf_static`, publishes camera intrinsics from YAML, and projects `base_link` trajectories into camera image overlays.

**Tech Stack:** Python 3, ROS 2 Jazzy, `rclpy`, `tf2_ros`, `geometry_msgs`, `sensor_msgs`, `tf2_msgs`, OpenCV, NumPy, PyYAML, `unittest`, Docker/colcon.

---

## File Structure

- Create `src/s2e_vlm_core/s2e_vlm_core/sensor_config.py`: shared parser, dataclasses, config discovery, matrix/quaternion conversion, validation.
- Create `src/s2e_vlm_core/test/test_sensor_config.py`: parser unit tests using temporary per-sensor YAML files.
- Modify `src/s2e_vlm_core/package.xml`: declare `python3-yaml` and `ament_index_python` runtime dependencies.
- Create `src/s2e_vlm_bringup/config/sensors/camera.yaml`: camera extrinsic and 640x480 intrinsic.
- Create `src/s2e_vlm_bringup/config/sensors/lidar.yaml`: lidar extrinsic.
- Create `src/s2e_vlm_bringup/config/sensors/imu.yaml`: IMU extrinsic.
- Modify `src/s2e_vlm_bringup/setup.py`: install nested `config/sensors/*.yaml` files.
- Modify `src/s2e_vlm_nodes/s2e_vlm_nodes/static_tf_node.py`: new entrypoint module with `NodeContract`.
- Modify `src/s2e_vlm_nodes/setup.py`: add `static_tf_node` console script.
- Modify `src/s2e_vlm_nodes/package.xml`: add `tf2_ros`, `tf2_msgs`, and `python3-yaml` dependencies.
- Modify `src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py`: add `StaticTfMockNode`, use parsed sensor config in camera/lidar/imu, fix mini-map, add TF projection and manifest counters.
- Modify `src/s2e_vlm_bringup/launch/single_pc_mock.launch.py`: launch `static_tf_node` with the graph.
- Modify `src/s2e_vlm_nodes/test/test_ros_mock_graph.py`: include static TF, verify camera info from YAML, verify projection manifest.
- Modify `src/s2e_vlm_core/s2e_vlm_core/mock_algorithms.py`: fix image-right to `base_link.y-`.
- Modify `src/s2e_vlm_core/test/test_mock_algorithms.py`: update sign expectations.
- Modify docs if verification commands or config layout references need to reflect `config/sensors/*.yaml`.

---

### Task 1: Shared Sensor Config Parser

**Files:**
- Create: `src/s2e_vlm_core/s2e_vlm_core/sensor_config.py`
- Create: `src/s2e_vlm_core/test/test_sensor_config.py`
- Modify: `src/s2e_vlm_core/package.xml`

- [ ] **Step 1: Write failing parser tests**

Add `src/s2e_vlm_core/test/test_sensor_config.py`:

```python
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s2e_vlm_core.sensor_config import SensorConfigError, load_all_sensor_configs, load_sensor_config


CAMERA_YAML = """
sensor_name: camera
parent_frame: base_link
child_frame: camera
translation_m: [0.25, 0.0, 0.35]
rotation_matrix_row_major: [0, 0, 1, -1, 0, 0, 0, -1, 0]
intrinsic:
  image_width: 640
  image_height: 480
  distortion_model: plumb_bob
  camera_matrix_row_major: [640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0]
  distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
"""


class SensorConfigTest(unittest.TestCase):
    def test_loads_camera_extrinsic_and_intrinsic(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir)
            (path / "camera.yaml").write_text(CAMERA_YAML, encoding="utf-8")

            config = load_sensor_config("camera", config_dir=path)

        self.assertEqual(config.sensor_name, "camera")
        self.assertEqual(config.parent_frame, "base_link")
        self.assertEqual(config.child_frame, "camera")
        self.assertEqual(config.translation_m, (0.25, 0.0, 0.35))
        self.assertIsNotNone(config.intrinsic)
        assert config.intrinsic is not None
        self.assertEqual(config.intrinsic.image_width, 640)
        self.assertEqual(config.intrinsic.image_height, 480)
        self.assertEqual(config.intrinsic.camera_matrix_row_major, (640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0))
        qx, qy, qz, qw = config.rotation_quaternion_xyzw
        self.assertTrue(all(math.isfinite(value) for value in (qx, qy, qz, qw)))
        self.assertAlmostEqual(math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw), 1.0, places=6)

    def test_load_all_sensor_configs(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir)
            (path / "camera.yaml").write_text(CAMERA_YAML, encoding="utf-8")
            (path / "lidar.yaml").write_text(
                "sensor_name: lidar\nparent_frame: base_link\nchild_frame: lidar\ntranslation_m: [0.1, 0.0, 0.2]\nrotation_quaternion_xyzw: [0, 0, 0, 1]\n",
                encoding="utf-8",
            )

            configs = load_all_sensor_configs(config_dir=path)

        self.assertEqual(sorted(configs), ["camera", "lidar"])

    def test_rejects_bad_matrix_shape(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir)
            (path / "camera.yaml").write_text(
                "sensor_name: camera\nparent_frame: base_link\nchild_frame: camera\ntranslation_m: [0, 0, 0]\nrotation_matrix_row_major: [1, 0, 0]\n",
                encoding="utf-8",
            )

            with self.assertRaises(SensorConfigError):
                load_sensor_config("camera", config_dir=path)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run parser tests and verify RED**

Run:

```bash
python3 -m unittest src/s2e_vlm_core/test/test_sensor_config.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 's2e_vlm_core.sensor_config'`.

- [ ] **Step 3: Implement the parser**

Create `src/s2e_vlm_core/s2e_vlm_core/sensor_config.py`:

```python
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class SensorConfigError(ValueError):
    pass


@dataclass(frozen=True)
class CameraIntrinsic:
    image_width: int
    image_height: int
    distortion_model: str
    camera_matrix_row_major: tuple[float, ...]
    distortion_coefficients: tuple[float, ...]
    rectification_matrix_row_major: tuple[float, ...]
    projection_matrix_row_major: tuple[float, ...]


@dataclass(frozen=True)
class SensorConfig:
    sensor_name: str
    parent_frame: str
    child_frame: str
    translation_m: tuple[float, float, float]
    rotation_quaternion_xyzw: tuple[float, float, float, float]
    rotation_matrix_row_major: tuple[float, ...]
    intrinsic: CameraIntrinsic | None = None


def default_sensor_config_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("s2e_vlm_bringup")) / "config" / "sensors"
    except Exception:
        return Path(__file__).resolve().parents[2] / "s2e_vlm_bringup" / "config" / "sensors"


def load_all_sensor_configs(config_dir: str | Path | None = None) -> dict[str, SensorConfig]:
    directory = Path(config_dir) if config_dir is not None else default_sensor_config_dir()
    if not directory.is_dir():
        raise SensorConfigError(f"sensor config directory does not exist: {directory}")
    configs: dict[str, SensorConfig] = {}
    for path in sorted(directory.glob("*.yaml")):
        config = _load_sensor_config_path(path)
        configs[config.sensor_name] = config
    if not configs:
        raise SensorConfigError(f"no sensor YAML files found in {directory}")
    return configs


def load_sensor_config(sensor_name: str, config_dir: str | Path | None = None) -> SensorConfig:
    directory = Path(config_dir) if config_dir is not None else default_sensor_config_dir()
    return _load_sensor_config_path(directory / f"{sensor_name}.yaml")


def _load_sensor_config_path(path: Path) -> SensorConfig:
    if not path.is_file():
        raise SensorConfigError(f"sensor config file does not exist: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SensorConfigError(f"sensor config must be a mapping: {path}")
    sensor_name = _required_str(data, "sensor_name")
    parent_frame = _required_str(data, "parent_frame")
    child_frame = _required_str(data, "child_frame")
    translation = _float_tuple(data.get("translation_m"), 3, "translation_m")
    matrix = _rotation_matrix(data)
    quaternion = _rotation_quaternion(data, matrix)
    intrinsic = _camera_intrinsic(data.get("intrinsic"))
    return SensorConfig(sensor_name, parent_frame, child_frame, translation, quaternion, matrix, intrinsic)


def _required_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        raise SensorConfigError(f"{key} must be a non-empty string")
    return value


def _float_tuple(value: Any, length: int, key: str) -> tuple[float, ...]:
    if not isinstance(value, list | tuple) or len(value) != length:
        raise SensorConfigError(f"{key} must contain {length} numeric values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise SensorConfigError(f"{key} must contain finite values")
    return result


def _rotation_matrix(data: dict[str, Any]) -> tuple[float, ...]:
    if "rotation_matrix_row_major" in data:
        return _float_tuple(data["rotation_matrix_row_major"], 9, "rotation_matrix_row_major")
    quaternion = _float_tuple(data.get("rotation_quaternion_xyzw"), 4, "rotation_quaternion_xyzw")
    return _quaternion_to_matrix(quaternion)


def _rotation_quaternion(data: dict[str, Any], matrix: tuple[float, ...]) -> tuple[float, float, float, float]:
    if "rotation_quaternion_xyzw" in data:
        qx, qy, qz, qw = _float_tuple(data["rotation_quaternion_xyzw"], 4, "rotation_quaternion_xyzw")
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            raise SensorConfigError("rotation_quaternion_xyzw must not be zero length")
        return qx / norm, qy / norm, qz / norm, qw / norm
    return _matrix_to_quaternion(matrix)


def _camera_intrinsic(value: Any) -> CameraIntrinsic | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise SensorConfigError("intrinsic must be a mapping")
    k = _float_tuple(value.get("camera_matrix_row_major"), 9, "camera_matrix_row_major")
    d = tuple(float(item) for item in value.get("distortion_coefficients", []))
    r = _float_tuple(value.get("rectification_matrix_row_major", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]), 9, "rectification_matrix_row_major")
    p = _float_tuple(value.get("projection_matrix_row_major", [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]), 12, "projection_matrix_row_major")
    return CameraIntrinsic(
        image_width=int(value.get("image_width", 0)),
        image_height=int(value.get("image_height", 0)),
        distortion_model=str(value.get("distortion_model", "plumb_bob")),
        camera_matrix_row_major=k,
        distortion_coefficients=d,
        rectification_matrix_row_major=r,
        projection_matrix_row_major=p,
    )


def _matrix_to_quaternion(matrix: tuple[float, ...]) -> tuple[float, float, float, float]:
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = matrix
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


def _quaternion_to_matrix(quaternion: tuple[float, ...]) -> tuple[float, ...]:
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise SensorConfigError("rotation_quaternion_xyzw must not be zero length")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy - qz * qw),
        2.0 * (qx * qz + qy * qw),
        2.0 * (qx * qy + qz * qw),
        1.0 - 2.0 * (qx * qx + qz * qz),
        2.0 * (qy * qz - qx * qw),
        2.0 * (qx * qz - qy * qw),
        2.0 * (qy * qz + qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
```

- [ ] **Step 4: Add core dependency**

Modify `src/s2e_vlm_core/package.xml` to include:

```xml
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
```

- [ ] **Step 5: Run parser tests and verify GREEN**

Run:

```bash
python3 -m unittest src/s2e_vlm_core/test/test_sensor_config.py -v
```

Expected: PASS.

- [ ] **Step 6: Commit parser work**

```bash
GIT_MASTER=1 git add src/s2e_vlm_core/s2e_vlm_core/sensor_config.py src/s2e_vlm_core/test/test_sensor_config.py src/s2e_vlm_core/package.xml
GIT_MASTER=1 git commit -m "Add sensor config parser" -m "Ultraworked with [Sisyphus](https://github.com/code-yeongyu/oh-my-openagent)" -m "Co-authored-by: Sisyphus <clio-agent@sisyphuslabs.ai>"
```

---

### Task 2: Per-Sensor YAML Configs and Packaging

**Files:**
- Create: `src/s2e_vlm_bringup/config/sensors/camera.yaml`
- Create: `src/s2e_vlm_bringup/config/sensors/lidar.yaml`
- Create: `src/s2e_vlm_bringup/config/sensors/imu.yaml`
- Modify: `src/s2e_vlm_bringup/setup.py`
- Modify: `tests/test_docker_assets.py` if it checks config packaging

- [ ] **Step 1: Write YAML files**

Create `src/s2e_vlm_bringup/config/sensors/camera.yaml`:

```yaml
sensor_name: camera
parent_frame: base_link
child_frame: camera
translation_m: [0.25, 0.0, 0.35]
rotation_matrix_row_major: [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]
intrinsic:
  image_width: 640
  image_height: 480
  distortion_model: plumb_bob
  camera_matrix_row_major: [640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0]
  distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
  rectification_matrix_row_major: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  projection_matrix_row_major: [640.0, 0.0, 320.0, 0.0, 0.0, 480.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

Create `src/s2e_vlm_bringup/config/sensors/lidar.yaml`:

```yaml
sensor_name: lidar
parent_frame: base_link
child_frame: lidar
translation_m: [0.15, 0.0, 0.22]
rotation_quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
```

Create `src/s2e_vlm_bringup/config/sensors/imu.yaml`:

```yaml
sensor_name: imu
parent_frame: base_link
child_frame: imu
translation_m: [0.0, 0.0, 0.12]
rotation_quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
```

- [ ] **Step 2: Install nested config files**

Modify `src/s2e_vlm_bringup/setup.py`:

```python
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config/sensors", glob("config/sensors/*.yaml")),
```

- [ ] **Step 3: Add packaging test if missing**

In `tests/test_docker_assets.py`, add an assertion that `src/s2e_vlm_bringup/setup.py` contains `config/sensors/*.yaml` and that all three sensor YAML paths exist.

- [ ] **Step 4: Run host packaging/static tests**

Run:

```bash
python3 -m unittest tests/test_docker_assets.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit sensor configs**

```bash
GIT_MASTER=1 git add src/s2e_vlm_bringup/config/sensors src/s2e_vlm_bringup/setup.py tests/test_docker_assets.py
GIT_MASTER=1 git commit -m "Add per-sensor calibration configs" -m "Ultraworked with [Sisyphus](https://github.com/code-yeongyu/oh-my-openagent)" -m "Co-authored-by: Sisyphus <clio-agent@sisyphuslabs.ai>"
```

---

### Task 3: Static TF Node and Sensor Node Calibration Consumption

**Files:**
- Create: `src/s2e_vlm_nodes/s2e_vlm_nodes/static_tf_node.py`
- Modify: `src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py`
- Modify: `src/s2e_vlm_nodes/setup.py`
- Modify: `src/s2e_vlm_nodes/package.xml`
- Modify: `src/s2e_vlm_bringup/launch/single_pc_mock.launch.py`
- Modify: `src/s2e_vlm_nodes/test/test_ros_mock_graph.py`

- [ ] **Step 1: Write failing ROS test for TF and camera intrinsics**

In `src/s2e_vlm_nodes/test/test_ros_mock_graph.py`:

```python
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage
```

Add `"static_tf_node"` to `NODE_EXECUTABLES`.

Subscribe in `test_single_pc_mock_graph_exchanges_real_ros_topics_and_rotate_action`:

```python
self._subscribe("/tf_static", TFMessage)
self._subscribe("/s2e/sensors/camera/camera_info", CameraInfo, sensor_qos=True)
```

Add required topics:

```python
"/tf_static",
"/s2e/sensors/camera/camera_info",
```

After startup, assert:

```python
tf_children = {
    transform.child_frame_id
    for message in self.messages["/tf_static"]
    for transform in message.transforms
}
self.assertTrue({"camera", "lidar", "imu"}.issubset(tf_children))
camera_info = self.messages["/s2e/sensors/camera/camera_info"][-1]
self.assertEqual(camera_info.header.frame_id, "camera")
self.assertEqual((camera_info.width, camera_info.height), (640, 480))
self.assertEqual(list(camera_info.k), [640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0])
```

- [ ] **Step 2: Run ROS graph test and verify RED in Docker**

Run inside ROS container:

```bash
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 -m unittest src/s2e_vlm_nodes/test/test_ros_mock_graph.py -v"
```

Expected: FAIL because `static_tf_node` does not exist and `/tf_static` is not published.

- [ ] **Step 3: Add static TF node contract**

Create `src/s2e_vlm_nodes/s2e_vlm_nodes/static_tf_node.py`:

```python
from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="static_tf_node",
    publishes=("/tf_static", "/s2e/status/static_tf_node"),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
```

- [ ] **Step 4: Add runtime implementation**

In `src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py`, import parser and TF broadcaster:

```python
from s2e_vlm_core.sensor_config import SensorConfig, SensorConfigError, load_all_sensor_configs, load_sensor_config
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
```

Add helper:

```python
def _load_sensor_config_or_default(sensor_name: str) -> SensorConfig | None:
    try:
        return load_sensor_config(sensor_name)
    except SensorConfigError:
        return None
```

Add class before `LidarMockNode`:

```python
class StaticTfMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.transforms = self._make_transforms()
        self.create_timer(1.0, self.publish_transforms)
        self.publish_transforms()

    def _make_transforms(self) -> list[TransformStamped]:
        configs = load_all_sensor_configs()
        transforms: list[TransformStamped] = []
        for config in configs.values():
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = config.parent_frame
            message.child_frame_id = config.child_frame
            message.transform.translation.x = config.translation_m[0]
            message.transform.translation.y = config.translation_m[1]
            message.transform.translation.z = config.translation_m[2]
            qx, qy, qz, qw = config.rotation_quaternion_xyzw
            message.transform.rotation.x = qx
            message.transform.rotation.y = qy
            message.transform.rotation.z = qz
            message.transform.rotation.w = qw
            transforms.append(message)
        return transforms

    def publish_transforms(self) -> None:
        for transform in self.transforms:
            transform.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.transforms)
        self.set_status("ACTIVE", "ACTIVE", message=f"published {len(self.transforms)} static transforms")
```

Register factory:

```python
"static_tf_node": StaticTfMockNode,
```

- [ ] **Step 5: Use parser in sensor publishers**

In `CameraMockNode.__init__`, load config:

```python
self.sensor_config = _load_sensor_config_or_default("camera")
self.camera_intrinsic = self.sensor_config.intrinsic if self.sensor_config is not None else None
self.width = self.camera_intrinsic.image_width if self.camera_intrinsic is not None else int(_env_float("S2E_MOCK_CAMERA_WIDTH", 640.0))
self.height = self.camera_intrinsic.image_height if self.camera_intrinsic is not None else int(_env_float("S2E_MOCK_CAMERA_HEIGHT", 480.0))
self.frame_id = self.sensor_config.child_frame if self.sensor_config is not None else "camera"
```

In `publish_image()`, use `self.frame_id` and intrinsic fields:

```python
image.header.frame_id = self.frame_id
info.header.frame_id = self.frame_id
if self.camera_intrinsic is not None:
    info.k = list(self.camera_intrinsic.camera_matrix_row_major)
    info.d = list(self.camera_intrinsic.distortion_coefficients)
    info.r = list(self.camera_intrinsic.rectification_matrix_row_major)
    info.p = list(self.camera_intrinsic.projection_matrix_row_major)
    info.distortion_model = self.camera_intrinsic.distortion_model
else:
    info.k = [float(self.width), 0.0, self.width / 2.0, 0.0, float(self.height), self.height / 2.0, 0.0, 0.0, 1.0]
```

In `LidarMockNode` and `ImuMockNode`, load config and set `message.header.frame_id` from `child_frame` instead of hard-coded literals.

- [ ] **Step 6: Add entrypoint, package deps, and launch node**

In `src/s2e_vlm_nodes/setup.py`, add:

```python
"static_tf_node = s2e_vlm_nodes.static_tf_node:main",
```

In `src/s2e_vlm_nodes/package.xml`, add:

```xml
  <depend>tf2_msgs</depend>
  <depend>tf2_ros</depend>
  <exec_depend>python3-yaml</exec_depend>
```

In `src/s2e_vlm_bringup/launch/single_pc_mock.launch.py`, add `"static_tf_node"` to `NODES` before sensor nodes.

- [ ] **Step 7: Run ROS graph test and verify GREEN**

Run:

```bash
docker compose build ros-base
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 -m unittest src/s2e_vlm_nodes/test/test_ros_mock_graph.py -v"
```

Expected: PASS for TF and camera info assertions.

- [ ] **Step 8: Commit static TF work**

```bash
GIT_MASTER=1 git add src/s2e_vlm_nodes/s2e_vlm_nodes/static_tf_node.py src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py src/s2e_vlm_nodes/setup.py src/s2e_vlm_nodes/package.xml src/s2e_vlm_bringup/launch/single_pc_mock.launch.py src/s2e_vlm_nodes/test/test_ros_mock_graph.py
GIT_MASTER=1 git commit -m "Add static sensor transforms" -m "Ultraworked with [Sisyphus](https://github.com/code-yeongyu/oh-my-openagent)" -m "Co-authored-by: Sisyphus <clio-agent@sisyphuslabs.ai>"
```

---

### Task 4: Coordinate Sign and Mini-Map Convention

**Files:**
- Modify: `src/s2e_vlm_core/s2e_vlm_core/mock_algorithms.py`
- Modify: `src/s2e_vlm_core/test/test_mock_algorithms.py`
- Modify: `src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py`

- [ ] **Step 1: Update sign tests first**

In `test_image_goal_to_base_link_is_bounded_and_config_driven`, add left and expect right negative:

```python
left = image_goal_to_base_link((0.0, 240.0), image_size=(640, 480), max_forward_m=4.0, max_lateral_m=2.0)
self.assertEqual(left, (2.5, 2.0))
self.assertEqual(right, (2.5, -2.0))
```

- [ ] **Step 2: Run test and verify RED**

```bash
python3 -m unittest src/s2e_vlm_core/test/test_mock_algorithms.py -v
```

Expected: FAIL because right currently maps to `+2.0`.

- [ ] **Step 3: Fix image-goal lateral sign**

In `src/s2e_vlm_core/s2e_vlm_core/mock_algorithms.py`:

```python
lateral = -max_lateral_m * normalized_u
```

- [ ] **Step 4: Fix mini-map sign and labels**

In `_draw_trajectory_minimap()`:

```python
px = int(origin[0] - point.y * 35.0)
py = int(origin[1] - point.x * 25.0)
```

Add labels near axes:

```python
cv2.putText(frame, "+x", (origin[0] + 4, top + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)
cv2.putText(frame, "+y left", (left + 12, origin[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)
cv2.putText(frame, "-y right", (right - 70, origin[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)
```

- [ ] **Step 5: Run core tests**

```bash
python3 -m unittest src/s2e_vlm_core/test/test_mock_algorithms.py -v
```

Expected: PASS.

- [ ] **Step 6: Commit coordinate fix**

```bash
GIT_MASTER=1 git add src/s2e_vlm_core/s2e_vlm_core/mock_algorithms.py src/s2e_vlm_core/test/test_mock_algorithms.py src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py
GIT_MASTER=1 git commit -m "Fix base link handedness" -m "Ultraworked with [Sisyphus](https://github.com/code-yeongyu/oh-my-openagent)" -m "Co-authored-by: Sisyphus <clio-agent@sisyphuslabs.ai>"
```

---

### Task 5: Visualizer Camera Projection and Artifact Manifest

**Files:**
- Modify: `src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py`
- Modify: `src/s2e_vlm_nodes/test/test_ros_mock_graph.py`

- [ ] **Step 1: Write failing artifact projection assertions**

In `test_visualizer_saves_png_sequence_and_mp4_from_smooth_goal_run`, after reading manifest:

```python
import json
manifest = json.loads((artifact_dir / "manifest.json").read_text(encoding="utf-8"))
self.assertTrue(manifest["projection_available"])
self.assertGreater(manifest["projected_trajectory_frames"], 0)
self.assertGreaterEqual(manifest["last_projected_point_count"], 2)
```

Expected RED before implementation: keys missing.

- [ ] **Step 2: Add TF listener and camera info storage**

In `DebugVisualizerMockNode.__init__`:

```python
self.last_camera_info: CameraInfo | None = None
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
self.projection_available = False
self.projected_trajectory_frames = 0
self.last_projected_point_count = 0
```

Change camera info subscription:

```python
self.create_subscription(CameraInfo, "/s2e/sensors/camera/camera_info", self._on_camera_info, SENSOR_QOS)
```

Add callback:

```python
def _on_camera_info(self, message: CameraInfo) -> None:
    self.last_camera_info = message
```

- [ ] **Step 3: Add projection helpers**

Add methods on `DebugVisualizerMockNode`:

```python
def _lookup_camera_from_base(self) -> TransformStamped | None:
    if self.last_camera_info is None:
        return None
    try:
        return self.tf_buffer.lookup_transform(self.last_camera_info.header.frame_id, "base_link", rclpy.time.Time())
    except TransformException:
        return None


def _project_base_point(self, transform: TransformStamped, point: Point32) -> tuple[int, int] | None:
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    tz = transform.transform.translation.z
    q = transform.transform.rotation
    # Convert quaternion to rotation matrix for source base_link -> target camera.
    x, y, z, w = q.x, q.y, q.z, q.w
    m00 = 1.0 - 2.0 * (y * y + z * z)
    m01 = 2.0 * (x * y - z * w)
    m02 = 2.0 * (x * z + y * w)
    m10 = 2.0 * (x * y + z * w)
    m11 = 1.0 - 2.0 * (x * x + z * z)
    m12 = 2.0 * (y * z - x * w)
    m20 = 2.0 * (x * z - y * w)
    m21 = 2.0 * (y * z + x * w)
    m22 = 1.0 - 2.0 * (x * x + y * y)
    cx = m00 * point.x + m01 * point.y + m02 * point.z + tx
    cy = m10 * point.x + m11 * point.y + m12 * point.z + ty
    cz = m20 * point.x + m21 * point.y + m22 * point.z + tz
    if self.last_camera_info is None or cz <= 1e-6:
        return None
    k = self.last_camera_info.k
    u = k[0] * cx / cz + k[2]
    v = k[4] * cy / cz + k[5]
    if not math.isfinite(u) or not math.isfinite(v):
        return None
    if u < 0 or u >= self.last_camera_info.width or v < 0 or v >= self.last_camera_info.height:
        return None
    return int(round(u)), int(round(v))


def _draw_projected_trajectory(self, frame, trajectory: Trajectory2D) -> int:
    transform = self._lookup_camera_from_base()
    if transform is None:
        cv2.putText(frame, "projection unavailable", (16, frame.shape[0] - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 180), 2, cv2.LINE_AA)
        self.projection_available = False
        self.last_projected_point_count = 0
        return 0
    points = [projected for point in trajectory.points if (projected := self._project_base_point(transform, point)) is not None]
    self.projection_available = len(points) >= 2
    self.last_projected_point_count = len(points)
    if len(points) >= 2:
        for before, after in zip(points, points[1:]):
            cv2.line(frame, before, after, (0, 180, 255), 3, cv2.LINE_AA)
        cv2.circle(frame, points[-1], 6, (0, 120, 255), -1, cv2.LINE_AA)
        self.projected_trajectory_frames += 1
    return len(points)
```

- [ ] **Step 4: Call projection in overlay**

In `_draw_overlay()`:

```python
if self.last_trajectory is not None:
    self._draw_projected_trajectory(frame, self.last_trajectory)
    self._draw_trajectory_minimap(frame, self.last_trajectory)
```

- [ ] **Step 5: Write projection manifest counters**

In `_write_manifest()` add:

```python
"projection_available": self.projection_available,
"projected_trajectory_frames": self.projected_trajectory_frames,
"last_projected_point_count": self.last_projected_point_count,
```

- [ ] **Step 6: Run artifact test in Docker**

```bash
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 -m unittest src/s2e_vlm_nodes/test/test_ros_mock_graph.py::RosMockGraphTest.test_visualizer_saves_png_sequence_and_mp4_from_smooth_goal_run -v"
```

Expected: PASS. If `unittest` does not accept `::`, run the whole file:

```bash
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 -m unittest src/s2e_vlm_nodes/test/test_ros_mock_graph.py -v"
```

- [ ] **Step 7: Commit visualizer projection**

```bash
GIT_MASTER=1 git add src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py src/s2e_vlm_nodes/test/test_ros_mock_graph.py
GIT_MASTER=1 git commit -m "Project trajectories onto camera image" -m "Ultraworked with [Sisyphus](https://github.com/code-yeongyu/oh-my-openagent)" -m "Co-authored-by: Sisyphus <clio-agent@sisyphuslabs.ai>"
```

---

### Task 6: Full Verification and Artifact Regeneration

**Files:**
- Modify: `docs/testing.md` only if commands or artifact manifest fields changed.
- No source edits unless verification reveals a regression caused by these tasks.

- [ ] **Step 1: Run host fallback tests**

```bash
python3 -m unittest discover -s src/s2e_vlm_core/test -p 'test_*.py' -v
python3 -m unittest discover -s src/s2e_vlm_nodes/test -p 'test_*.py' -v
python3 -m unittest src/s2e_vlm_bringup/test_launch_contracts.py -v
python3 -m unittest tests/test_docker_assets.py -v
python3 -m compileall -q src tests
```

Expected: PASS or ROS-specific tests skipped on hosts without ROS 2.

- [ ] **Step 2: Run LSP diagnostics on changed Python files**

Check:

```text
src/s2e_vlm_core/s2e_vlm_core/sensor_config.py
src/s2e_vlm_core/s2e_vlm_core/mock_algorithms.py
src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py
src/s2e_vlm_nodes/s2e_vlm_nodes/static_tf_node.py
src/s2e_vlm_core/test/test_sensor_config.py
src/s2e_vlm_core/test/test_mock_algorithms.py
src/s2e_vlm_nodes/test/test_ros_mock_graph.py
```

Expected: no errors caused by this work.

- [ ] **Step 3: Run Docker ROS build/test**

```bash
docker compose build ros-base
docker run --rm s2e-ros-base:latest bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose"
```

Expected: all packages pass.

- [ ] **Step 4: Regenerate a repo-visible visualizer artifact**

Use the existing compose path with repo-mounted `artifacts/`. Run the same artifact command documented in `docs/testing.md`, with `S2E_MOCK_ARTIFACT_DURATION_S=10.0` and `S2E_TEST_ARTIFACT_DIR=/artifacts/visualizer`.

Expected: `artifacts/.../manifest.json` reports:

```json
{
  "format": "png",
  "projection_available": true,
  "projected_trajectory_frames": 1
}
```

`projected_trajectory_frames` should be greater than zero, not exactly one.

- [ ] **Step 5: Commit docs or verification updates if any**

If `docs/testing.md` changed:

```bash
GIT_MASTER=1 git add docs/testing.md
GIT_MASTER=1 git commit -m "Document projected visualizer artifacts" -m "Ultraworked with [Sisyphus](https://github.com/code-yeongyu/oh-my-openagent)" -m "Co-authored-by: Sisyphus <clio-agent@sisyphuslabs.ai>"
```

- [ ] **Step 6: Final status check**

```bash
GIT_MASTER=1 git status
GIT_MASTER=1 git log --oneline -10
```

Expected: only intentional untracked runtime artifacts remain ignored under `artifacts/`; implementation/source changes are committed or explicitly reported.

---

## Self-Review Checklist

- Spec coverage: per-sensor YAML, shared parser, static TF, camera intrinsic publication, corrected `base_link` sign, camera projection, mini-map fallback, manifest evidence, and Docker verification are covered.
- Placeholder scan: no task says to add unspecified tests or generic error handling; each task names files, code shape, commands, and expected results.
- Type consistency: parser returns `SensorConfig`/`CameraIntrinsic`; ROS runtime uses `SensorConfig`, `CameraInfo`, `TransformStamped`, and `Trajectory2D` consistently.
- Scope check: no real Unitree calibration, no calibration service, no controller behavior changes beyond corrected signed trajectory input.
