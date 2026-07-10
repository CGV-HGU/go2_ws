# pyright: reportMissingImports=false

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
        self.assertEqual(config.rotation_matrix_row_major, (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0))
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

    def test_rejects_missing_camera_matrix_when_intrinsic_is_present(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir)
            (path / "camera.yaml").write_text(
                "sensor_name: camera\nparent_frame: base_link\nchild_frame: camera\ntranslation_m: [0, 0, 0]\nrotation_quaternion_xyzw: [0, 0, 0, 1]\nintrinsic:\n  image_width: 640\n  image_height: 480\n",
                encoding="utf-8",
            )

            with self.assertRaises(SensorConfigError):
                load_sensor_config("camera", config_dir=path)


if __name__ == "__main__":
    unittest.main()
