# pyright: reportMissingImports=false, reportAttributeAccessIssue=false, reportOptionalMemberAccess=false, reportPossiblyUnboundVariable=false

import os
import json
import random
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Any

from s2e_vlm_nodes.debug_visualizer_node import NODE_CONTRACT as DEBUG_VISUALIZER_CONTRACT

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from s2e_vlm_msgs.action import Rotate
    from s2e_vlm_msgs.msg import NodeStatus, StampedPose, SystemHealth, Trajectory2D
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except ImportError:
    rclpy = None


NODE_EXECUTABLES = (
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
)


class NodeContractTest(unittest.TestCase):
    def test_debug_visualizer_subscribes_to_every_status_topic_it_renders(self):
        expected_status_topics = {
            "/s2e/status/static_tf_node",
            "/s2e/status/odometry_node",
            "/s2e/status/controller_node",
            "/s2e/status/vlm_node",
            "/s2e/status/e2e_node",
        }
        self.assertTrue(expected_status_topics.issubset(DEBUG_VISUALIZER_CONTRACT.subscribes))


class RosMockGraphTest(unittest.TestCase):
    def setUp(self):
        if rclpy is None:
            self.skipTest("ROS 2 rclpy is unavailable on this host")
        self.env = os.environ.copy()
        self.env["ROS_DOMAIN_ID"] = str(random.randint(150, 220))
        self.env["S2E_MOCK_VLM_SCENARIOS"] = "go,go,rotate,go,stop,malformed,go"
        self.env["S2E_MOCK_VLM_PERIOD_S"] = "0.5"
        self.env["S2E_MOCK_E2E_PERIOD_S"] = "0.1"
        self.env["S2E_MOCK_HEARTBEAT_TIMEOUT_S"] = "1.2"
        self.env["PYTHONUNBUFFERED"] = "1"
        os.environ["ROS_DOMAIN_ID"] = self.env["ROS_DOMAIN_ID"]
        rclpy.init(args=None)
        self.node = rclpy.create_node(f"ros_mock_graph_probe_{os.getpid()}")
        self.processes: dict[str, subprocess.Popen[str]] = {}
        self.expected_stopped: set[str] = set()
        self.messages: dict[str, list[Any]] = defaultdict(list)
        self._subscriptions = []

    def tearDown(self):
        if rclpy is not None and hasattr(self, "node"):
            self.node.destroy_node()
            rclpy.shutdown()
        for process in self.processes.values():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 3.0
        for process in self.processes.values():
            if process.poll() is None:
                remaining = max(0.1, deadline - time.monotonic())
                try:
                    process.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=1.0)

    def _start_graph(self) -> None:
        for executable in NODE_EXECUTABLES:
            self.processes[executable] = subprocess.Popen(
                ["ros2", "run", "s2e_vlm_nodes", executable],
                env=self.env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )

    def _stop_node(self, executable: str) -> None:
        process = self.processes[executable]
        self.expected_stopped.add(executable)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)

    def _subscribe(self, topic: str, msg_type: type, *, sensor_qos: bool = False) -> None:
        qos = qos_profile_sensor_data if sensor_qos else 10
        self._subscriptions.append(
            self.node.create_subscription(msg_type, topic, lambda msg, name=topic: self.messages[name].append(msg), qos)
        )

    def _spin_until(self, predicate, *, timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for name, process in self.processes.items():
                if name in self.expected_stopped:
                    continue
                if process.poll() not in (None, 0):
                    try:
                        output, _ = process.communicate(timeout=0.1)
                    except subprocess.TimeoutExpired:
                        output = "<process exited but stdout pipe did not close>"
                    self.fail(f"{name} exited early with {process.returncode}: {output}")
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail(f"Timed out waiting for {description}; received counts={dict((k, len(v)) for k, v in self.messages.items())}")

    def test_single_pc_mock_graph_exchanges_real_ros_topics_and_rotate_action(self):
        self._subscribe("/s2e/sensors/lidar/points", PointCloud2, sensor_qos=True)
        self._subscribe("/s2e/sensors/camera/image", Image, sensor_qos=True)
        self._subscribe("/s2e/sensors/camera/camera_info", CameraInfo, sensor_qos=True)
        self._subscribe("/s2e/sensors/imu", Imu, sensor_qos=True)
        self._subscribe("/tf_static", TFMessage)
        self._subscribe("/s2e/odometry/pose", StampedPose)
        self._subscribe("/s2e/vlm/reasoning", String)
        self._subscribe("/s2e/e2e/trajectory", Trajectory2D)
        self._subscribe("/s2e/e2e/status", NodeStatus)
        self._subscribe("/s2e/controller/status", NodeStatus)
        self._subscribe("/s2e/controller/command", Twist)
        self._subscribe("/s2e/supervisor/health", SystemHealth)
        self._subscribe("/s2e/debug/visualizer/image", Image, sensor_qos=True)
        for executable in NODE_EXECUTABLES:
            self._subscribe(f"/s2e/status/{executable}", NodeStatus)

        self._start_graph()

        required_topics = (
            "/s2e/sensors/lidar/points",
            "/s2e/sensors/camera/image",
            "/s2e/sensors/camera/camera_info",
            "/s2e/sensors/imu",
            "/tf_static",
            "/s2e/odometry/pose",
            "/s2e/vlm/reasoning",
            "/s2e/e2e/trajectory",
            "/s2e/e2e/status",
            "/s2e/controller/status",
            "/s2e/controller/command",
            "/s2e/supervisor/health",
            "/s2e/debug/visualizer/image",
        )
        self._spin_until(
            lambda: all(self.messages[topic] for topic in required_topics),
            timeout_s=8.0,
            description="all documented single-PC data topics",
        )
        self._spin_until(
            lambda: all(self.messages[f"/s2e/status/{name}"] for name in NODE_EXECUTABLES),
            timeout_s=4.0,
            description="all node heartbeat status topics",
        )
        self._spin_until(
            lambda: len(self.messages["/s2e/e2e/trajectory"]) > len(self.messages["/s2e/vlm/reasoning"]),
            timeout_s=4.0,
            description="e2e publishes multiple trajectories from cached VLM reasoning",
        )
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
        trajectory = self.messages["/s2e/e2e/trajectory"][-1]
        self.assertEqual(trajectory.header.frame_id, "base_link")
        self.assertEqual(len(trajectory.points), 10)
        self.assertTrue(all(point.z == 0.0 for point in trajectory.points))

        action_client = ActionClient(self.node, Rotate, "/s2e/controller/rotate")
        self.assertTrue(action_client.wait_for_server(timeout_sec=5.0))
        goal = Rotate.Goal()
        goal.target_yaw_delta_deg = 30.0
        goal.max_yaw_rate_deg_s = 30.0
        goal.tolerance_deg = 3.0
        goal.timeout_s = 5.0
        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
        self.assertTrue(result_future.result().result.success)
        self.assertTrue(
            any(status.active_mode == "ROTATING" for status in self.messages["/s2e/controller/status"]),
            "controller status should expose ROTATING during action-owned rotation",
        )

    def test_vlm_rotate_reasoning_sends_controller_action_goal(self):
        from s2e_vlm_core.vlm_schema import VlmAction, parse_vlm_reasoning

        self.env["S2E_MOCK_VLM_SCENARIOS"] = "rotate"
        self.env["S2E_MOCK_VLM_PERIOD_S"] = "0.25"
        self.env["S2E_MOCK_E2E_PERIOD_S"] = "0.05"
        self.env["S2E_MOCK_CONTROLLER_PERIOD_S"] = "0.02"
        self._subscribe("/s2e/vlm/reasoning", String)
        self._subscribe("/s2e/e2e/status", NodeStatus)
        self._subscribe("/s2e/controller/status", NodeStatus)
        self._subscribe("/s2e/controller/command", Twist)
        self._subscribe("/s2e/status/vlm_node", NodeStatus)
        self._start_graph()

        self._spin_until(
            lambda: any(
                (parsed := parse_vlm_reasoning(payload.data)).valid and parsed.action == VlmAction.ROTATE
                for payload in self.messages["/s2e/vlm/reasoning"]
            ),
            timeout_s=8.0,
            description="VLM publishes rotate reasoning",
        )
        self._spin_until(
            lambda: any(status.active_mode == "FROZEN_ROTATING" for status in self.messages["/s2e/status/vlm_node"]),
            timeout_s=4.0,
            description="VLM freezes while its rotate action goal is active",
        )
        self._spin_until(
            lambda: any(status.active_mode == "ROTATING" for status in self.messages["/s2e/controller/status"]),
            timeout_s=4.0,
            description="controller enters ROTATING from VLM action client",
        )
        self._spin_until(
            lambda: any(abs(command.angular.z) > 1e-6 for command in self.messages["/s2e/controller/command"]),
            timeout_s=4.0,
            description="controller publishes yaw command during VLM-triggered rotate",
        )
        self._spin_until(
            lambda: any(status.active_mode == "ROTATE_IN_PROGRESS" for status in self.messages["/s2e/e2e/status"]),
            timeout_s=4.0,
            description="e2e holds trajectory generation while rotate owns controller",
        )

    def test_stop_and_malformed_vlm_sequences_are_observable_on_real_ros_topics(self):
        self._subscribe("/s2e/vlm/reasoning", String)
        self._subscribe("/s2e/e2e/status", NodeStatus)
        self._subscribe("/s2e/controller/command", Twist)
        self._start_graph()

        self._spin_until(
            lambda: any(status.active_mode == "STOPPED_BY_VLM" for status in self.messages["/s2e/e2e/status"])
            and any(status.active_mode == "INVALID_VLM" for status in self.messages["/s2e/e2e/status"]),
            timeout_s=8.0,
            description="stop and malformed VLM states on e2e status",
        )
        self.assertTrue(
            any(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6 for command in self.messages["/s2e/controller/command"]),
            "controller should publish zero hold commands during stop or invalid VLM states",
        )

    def test_vlm_heartbeat_loss_blocks_motion_on_real_ros_topics(self):
        self.env["S2E_MOCK_VLM_SCENARIOS"] = "go"
        self._subscribe("/s2e/e2e/status", NodeStatus)
        self._subscribe("/s2e/controller/command", Twist)
        self._subscribe("/s2e/supervisor/health", SystemHealth)
        self._start_graph()

        self._spin_until(
            lambda: any(health.ok_to_move for health in self.messages["/s2e/supervisor/health"]),
            timeout_s=8.0,
            description="initial healthy supervisor state before heartbeat loss",
        )

        self._stop_node("vlm_node")
        self._spin_until(
            lambda: any(not health.ok_to_move and "vlm_node" in health.missing_critical_nodes for health in self.messages["/s2e/supervisor/health"]),
            timeout_s=6.0,
            description="supervisor health blocks motion after vlm heartbeat loss",
        )
        self.assertTrue(
            any(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6 for command in self.messages["/s2e/controller/command"]),
            "controller should publish zero hold commands when health is blocked",
        )

    def test_missing_camera_intrinsic_blocks_metric_goal_publication(self):
        with tempfile.TemporaryDirectory() as config_root:
            source_dir = Path(__file__).resolve().parents[2] / "s2e_vlm_bringup" / "config" / "sensors"
            config_dir = Path(config_root)
            shutil.copy(source_dir / "lidar.yaml", config_dir / "lidar.yaml")
            shutil.copy(source_dir / "imu.yaml", config_dir / "imu.yaml")
            camera_config = (source_dir / "camera.yaml").read_text(encoding="utf-8").split("intrinsic:", 1)[0]
            (config_dir / "camera.yaml").write_text(camera_config, encoding="utf-8")
            self.env["S2E_SENSOR_CONFIG_DIR"] = str(config_dir)
            self.env["S2E_MOCK_VLM_SCENARIOS"] = "go"
            self._subscribe("/s2e/e2e/status", NodeStatus)
            self._subscribe("/s2e/e2e/trajectory", Trajectory2D)
            self._start_graph()

            self._spin_until(
                lambda: any(status.active_mode == "CALIBRATION_UNAVAILABLE" for status in self.messages["/s2e/e2e/status"]),
                timeout_s=8.0,
                description="e2e blocks metric goal publication without camera intrinsic calibration",
            )

            self.assertEqual(len(self.messages["/s2e/e2e/trajectory"]), 0)

    def test_visualizer_saves_png_sequence_and_mp4_from_smooth_goal_run(self):
        from s2e_vlm_core.vlm_schema import parse_vlm_reasoning

        with tempfile.TemporaryDirectory() as artifact_root:
            artifact_dir = Path(artifact_root) / "visualizer"
            self.env["S2E_TEST_ARTIFACT_DIR"] = str(artifact_dir)
            self.env["S2E_MOCK_CAMERA_WIDTH"] = "640"
            self.env["S2E_MOCK_CAMERA_HEIGHT"] = "480"
            self.env["S2E_MOCK_CAMERA_MODE"] = "white"
            self.env["S2E_MOCK_VLM_SCENARIOS"] = "go"
            self.env["S2E_MOCK_VLM_PERIOD_S"] = "0.5"
            self.env["S2E_MOCK_E2E_PERIOD_S"] = "0.1"
            self.env["S2E_MOCK_DEBUG_VISUALIZER_PERIOD_S"] = "0.1"
            self.env["S2E_MOCK_ARTIFACT_DURATION_S"] = "10.0"
            self.env["S2E_DEBUG_MODE"] = "1"
            self.env["S2E_RUNTIME_ROLE"] = "single_pc_mock"
            self._subscribe("/s2e/sensors/camera/image", Image, sensor_qos=True)
            self._subscribe("/s2e/debug/visualizer/image", Image, sensor_qos=True)
            self._subscribe("/s2e/vlm/reasoning", String)
            self._subscribe("/s2e/e2e/trajectory", Trajectory2D)
            self._subscribe("/s2e/e2e/status", NodeStatus)
            self._subscribe("/s2e/controller/status", NodeStatus)
            self._start_graph()

            self._spin_until(
                lambda: self.messages["/s2e/sensors/camera/image"]
                and self.messages["/s2e/debug/visualizer/image"]
                and self.messages["/s2e/vlm/reasoning"]
                and self.messages["/s2e/e2e/trajectory"]
                and any(status.active_mode == "FOLLOWING" for status in self.messages["/s2e/controller/status"]),
                timeout_s=8.0,
                description="smooth goal visualizer graph startup",
            )
            end = time.monotonic() + 10.5
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)

            raw_image = self.messages["/s2e/sensors/camera/image"][-1]
            debug_image = self.messages["/s2e/debug/visualizer/image"][-1]
            self.assertEqual((raw_image.width, raw_image.height, raw_image.encoding), (640, 480, "rgb8"))
            self.assertEqual(len(raw_image.data), 640 * 480 * 3)
            self.assertTrue(all(value == 255 for value in bytes(raw_image.data[:4096])))
            self.assertEqual((debug_image.width, debug_image.height, debug_image.encoding), (640, 480, "rgb8"))
            self.assertNotEqual(bytes(debug_image.data), bytes(raw_image.data))

            goals = []
            for payload in self.messages["/s2e/vlm/reasoning"]:
                parsed = parse_vlm_reasoning(payload.data)
                if parsed.valid and parsed.goal_uv is not None:
                    goals.append(parsed.goal_uv)
            self.assertGreaterEqual(len(goals), 5)
            self.assertGreater(len(set(goals)), 1)
            for u, v in goals:
                self.assertGreaterEqual(u, 0.0)
                self.assertLess(u, 640.0)
                self.assertGreaterEqual(v, 0.0)
                self.assertLess(v, 480.0)
            self.assertFalse(
                any(status.active_mode == "CALIBRATION_UNAVAILABLE" for status in self.messages["/s2e/e2e/status"]),
                "default smooth visualizer run should not drive VLM goals above the projectable ground region",
            )

            trajectory = self.messages["/s2e/e2e/trajectory"][-1]
            self.assertEqual(trajectory.header.frame_id, "base_link")
            self.assertTrue(trajectory.has_goal_point)
            self.assertEqual(len(trajectory.points), 10)
            self.assertAlmostEqual(trajectory.points[-1].x, trajectory.goal_point_base_link.x, places=5)
            self.assertAlmostEqual(trajectory.points[-1].y, trajectory.goal_point_base_link.y, places=5)
            self.assertIn("preprocessed_goal_base_link", trajectory.status)

            png_files = sorted(artifact_dir.glob("frame_*.png"))
            self.assertGreaterEqual(len(png_files), 20)
            self.assertTrue((artifact_dir / "visualizer.mp4").is_file())
            self.assertTrue((artifact_dir / "manifest.json").is_file())
            manifest = json.loads((artifact_dir / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["video_codec"], "h264")
            self.assertEqual(manifest["video_pix_fmt"], "yuv420p")
            self.assertTrue(manifest["debug_mode"])
            self.assertEqual(manifest["runtime_context"]["role"], "single_pc_mock")
            self.assertEqual(manifest["runtime_context"]["ros_domain_id"], self.env["ROS_DOMAIN_ID"])
            self.assertIn("vlm_node", manifest["last_node_statuses"])
            self.assertIn("controller_node", manifest["last_node_statuses"])
            self.assertIn("ok_to_move", manifest["last_supervisor_health"])
            self.assertIn("action", manifest["last_vlm_parse"])
            self.assertIn("active_mode", manifest["last_e2e_status"])
            self.assertIn("active_mode", manifest["last_controller_status"])
            self.assertIn("status", manifest["last_projection_status"])
            ffprobe = shutil.which("ffprobe")
            if ffprobe is not None:
                video_probe = subprocess.run(
                    [
                        ffprobe,
                        "-v",
                        "error",
                        "-select_streams",
                        "v:0",
                        "-show_entries",
                        "stream=codec_name,pix_fmt,nb_frames",
                        "-of",
                        "json",
                        str(artifact_dir / "visualizer.mp4"),
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=10.0,
                )
                stream = json.loads(video_probe.stdout)["streams"][0]
                self.assertEqual(stream["codec_name"], "h264")
                self.assertEqual(stream["pix_fmt"], "yuv420p")
                self.assertGreaterEqual(int(stream["nb_frames"]), 20)
            self.assertTrue(manifest["projection_available"])
            self.assertGreater(manifest["projected_trajectory_frames"], 0)
            self.assertEqual(manifest["last_projected_total_point_count"], 10)
            self.assertGreater(manifest["last_projected_point_count"], 0)
            self.assertLessEqual(manifest["last_projected_point_count"], manifest["last_projected_total_point_count"])
            self.assertGreater(manifest["last_projected_visible_point_count"], 0)
            self.assertGreaterEqual(manifest["last_projected_clipped_point_count"], 0)
            self.assertLessEqual(manifest["last_projected_visible_point_count"], manifest["last_projected_total_point_count"])


if __name__ == "__main__":
    unittest.main()
