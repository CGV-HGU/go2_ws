import importlib.util
import unittest
from pathlib import Path

LAUNCH_DIR = Path(__file__).resolve().parent / "launch"


class LaunchContractTest(unittest.TestCase):
    def _load_launch(self, file_name):
        spec = importlib.util.spec_from_file_location(file_name, LAUNCH_DIR / file_name)
        assert spec is not None
        assert spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.generate_launch_description()

    def _node_names(self, description):
        if isinstance(description, dict):
            return description["nodes"]
        arguments = self._launch_arguments(description)
        names = []
        for action in description.entities:
            if action.__class__.__name__ != "Node":
                continue
            name = action._Node__node_name
            if name == "debug_visualizer_node" and arguments.get("enable_debug_visualizer") != "true":
                continue
            names.append(name)
        return names

    def _launch_arguments(self, description):
        if isinstance(description, dict):
            return description.get("arguments", {})
        return {
            action.name: action.default_value[0].text
            for action in description.entities
            if action.__class__.__name__ == "DeclareLaunchArgument"
        }

    def test_single_pc_mock_launch_lists_all_nodes_in_fallback_mode(self):
        description = self._load_launch("single_pc_mock.launch.py")

        self.assertEqual(
            self._node_names(description),
            [
                "static_tf_node",
                "lidar_node",
                "camera_node",
                "imu_node",
                "odometry_node",
                "controller_node",
                "supervisor_node",
                "vlm_node",
                "e2e_node",
                "debug_visualizer_node",
            ],
        )

    def test_split_launches_keep_robot_and_external_boundaries(self):
        robot = self._load_launch("robot_side.launch.py")
        external = self._load_launch("external_pc.launch.py")
        robot_nodes = self._node_names(robot)
        external_nodes = self._node_names(external)

        self.assertIn("controller_node", robot_nodes)
        self.assertIn("supervisor_node", robot_nodes)
        self.assertIn("static_tf_node", robot_nodes)
        self.assertNotIn("vlm_node", robot_nodes)
        self.assertEqual(external_nodes, ["vlm_node", "e2e_node"])

    def test_launch_profiles_declare_operational_split_arguments(self):
        expected = {
            "use_mock_hardware",
            "use_mock_models",
            "sensor_config_dir",
            "enable_debug_visualizer",
            "namespace",
        }

        for launch_file in ["single_pc_mock.launch.py", "robot_side.launch.py", "external_pc.launch.py"]:
            with self.subTest(launch_file=launch_file):
                arguments = self._launch_arguments(self._load_launch(launch_file))
                self.assertTrue(expected.issubset(arguments))

        self.assertEqual(self._launch_arguments(self._load_launch("single_pc_mock.launch.py"))["enable_debug_visualizer"], "true")
        self.assertEqual(self._launch_arguments(self._load_launch("robot_side.launch.py"))["enable_debug_visualizer"], "true")
        self.assertEqual(self._launch_arguments(self._load_launch("external_pc.launch.py"))["enable_debug_visualizer"], "false")


if __name__ == "__main__":
    unittest.main()
