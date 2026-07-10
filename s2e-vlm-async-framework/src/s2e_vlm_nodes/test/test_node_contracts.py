import importlib
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "s2e_vlm_core"))
sys.path.insert(0, str(ROOT / "s2e_vlm_nodes"))


EXPECTED_CONTRACTS = {
    "static_tf_node": {"publishes": ["/tf_static", "/s2e/status/static_tf_node"]},
    "lidar_node": {"publishes": ["/s2e/sensors/lidar/points", "/s2e/status/lidar_node"]},
    "camera_node": {"publishes": ["/s2e/sensors/camera/image", "/s2e/sensors/camera/camera_info", "/s2e/status/camera_node"]},
    "imu_node": {"publishes": ["/s2e/sensors/imu", "/s2e/status/imu_node"]},
    "odometry_node": {"publishes": ["/s2e/odometry/pose", "/s2e/status/odometry_node"]},
    "vlm_node": {"publishes": ["/s2e/vlm/reasoning", "/s2e/status/vlm_node"], "actions": ["/s2e/controller/rotate"]},
    "e2e_node": {"publishes": ["/s2e/e2e/trajectory", "/s2e/e2e/status", "/s2e/status/e2e_node"]},
    "controller_node": {"publishes": ["/s2e/controller/command", "/s2e/controller/status", "/s2e/status/controller_node"], "actions": ["/s2e/controller/rotate"]},
    "supervisor_node": {"publishes": ["/s2e/supervisor/health", "/s2e/status/supervisor_node"]},
    "debug_visualizer_node": {"publishes": ["/s2e/debug/visualizer/image", "/s2e/status/debug_visualizer_node"]},
}


class NodeContractTest(unittest.TestCase):
    def test_node_modules_declare_documented_topics_and_actions(self):
        for module_name, expected in EXPECTED_CONTRACTS.items():
            with self.subTest(module_name=module_name):
                module = importlib.import_module(f"s2e_vlm_nodes.{module_name}")
                contract = module.NODE_CONTRACT
                for topic in expected.get("publishes", []):
                    self.assertIn(topic, contract.publishes)
                for action in expected.get("actions", []):
                    self.assertIn(action, contract.actions)

    def test_visualizer_declares_no_command_publishers(self):
        module = importlib.import_module("s2e_vlm_nodes.debug_visualizer_node")

        self.assertNotIn("/s2e/controller/command", module.NODE_CONTRACT.publishes)

    def test_supervisor_monitors_static_tf_status(self):
        module = importlib.import_module("s2e_vlm_nodes.supervisor_node")

        self.assertIn("/s2e/status/static_tf_node", module.NODE_CONTRACT.subscribes)


if __name__ == "__main__":
    unittest.main()
