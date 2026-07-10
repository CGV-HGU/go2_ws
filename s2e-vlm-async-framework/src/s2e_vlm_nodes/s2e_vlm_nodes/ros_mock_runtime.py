from __future__ import annotations

# pyright: reportMissingImports=false, reportAttributeAccessIssue=false

import math
import os
import json
import shutil
import struct
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PoseStamped, TransformStamped, Twist
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from s2e_vlm_core.mock_algorithms import MockE2EPlanner, MockOdometryEstimator, MockVlmReasoner, camera_image_to_base_link_ground
from s2e_vlm_core.pose_buffer import Pose2D
from s2e_vlm_core.s2e_backend import S2EFrameContext, S2EPlanner, ros_rgb8_to_chw_float
from s2e_vlm_core.sensor_config import SensorConfig, SensorConfigError, load_all_sensor_configs, load_sensor_config
from s2e_vlm_core.vlm_schema import VlmAction, parse_vlm_reasoning
from nav_msgs.msg import Odometry
from s2e_vlm_msgs.action import Rotate
from s2e_vlm_msgs.msg import NodeStatus, StampedPose, SystemHealth, Trajectory2D
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from .node_contracts import NodeContract


SENSOR_QOS = qos_profile_sensor_data
RELIABLE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
STATUS_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _time_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _pose2d_from_stamped_pose(message: StampedPose) -> Pose2D:
    qz = message.pose.orientation.z
    qw = message.pose.orientation.w
    yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    return Pose2D(message.pose.position.x, message.pose.position.y, yaw)


def _load_sensor_config_or_default(sensor_name: str) -> SensorConfig | None:
    try:
        return load_sensor_config(sensor_name)
    except SensorConfigError:
        return None


def _node_status(
    node: Node,
    *,
    node_name: str,
    state: str,
    active_mode: str,
    healthy: bool,
    motion_critical: bool,
    last_input_age_s: float = 0.0,
    last_output_age_s: float = 0.0,
    error_code: str = "",
    message: str = "",
) -> NodeStatus:
    status = NodeStatus()
    status.header.stamp = node.get_clock().now().to_msg()
    status.node_name = node_name
    status.state = state
    status.active_mode = active_mode
    status.is_healthy = healthy
    status.is_motion_critical = motion_critical
    status.last_input_age_s = float(last_input_age_s)
    status.last_output_age_s = float(last_output_age_s)
    status.error_code = error_code
    status.message = message
    return status


class BaseMockNode(Node):
    def __init__(self, contract: NodeContract, *, motion_critical: bool = False) -> None:
        super().__init__(contract.node_name)
        self.contract = contract
        self.motion_critical = motion_critical or contract.motion_authority
        self.state = "INIT"
        self.active_mode = "INIT"
        self.healthy = True
        self.error_code = ""
        self.status_message = "mock node starting"
        
        self.declare_parameter("use_mock_hardware", True)
        self.use_mock_hardware = self.get_parameter("use_mock_hardware").value
        
        self.status_publisher = self.create_publisher(NodeStatus, f"/s2e/status/{contract.node_name}", STATUS_QOS)
        self.create_timer(1.0, self.publish_heartbeat)

    def make_status(self) -> NodeStatus:
        return _node_status(
            self,
            node_name=self.contract.node_name,
            state=self.state,
            active_mode=self.active_mode,
            healthy=self.healthy,
            motion_critical=self.motion_critical,
            error_code=self.error_code,
            message=self.status_message,
        )

    def publish_heartbeat(self) -> None:
        self.status_publisher.publish(self.make_status())

    def set_status(self, state: str, active_mode: str, *, healthy: bool = True, error_code: str = "", message: str = "") -> None:
        self.state = state
        self.active_mode = active_mode
        self.healthy = healthy
        self.error_code = error_code
        self.status_message = message


class StaticTfMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.transforms = self._make_transforms()
        self.create_timer(1.0, self.publish_transforms)
        self.publish_transforms()

    def _make_transforms(self) -> list[TransformStamped]:
        transforms: list[TransformStamped] = []
        try:
            configs = load_all_sensor_configs()
        except SensorConfigError as exc:
            self.set_status("FAULT", "FAULT", healthy=False, error_code="SENSOR_CONFIG", message=str(exc))
            return transforms
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
        if not self.transforms:
            self.publish_heartbeat()
            return
        stamp = self.get_clock().now().to_msg()
        for transform in self.transforms:
            transform.header.stamp = stamp
        self.broadcaster.sendTransform(self.transforms)
        self.set_status("ACTIVE", "ACTIVE", message=f"published {len(self.transforms)} static transforms")
        self.publish_heartbeat()


class LidarMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract)
        self.sensor_config = _load_sensor_config_or_default("lidar")
        self.frame_id = self.sensor_config.child_frame if self.sensor_config is not None else "lidar"
        self.publisher = self.create_publisher(PointCloud2, "/s2e/sensors/lidar/points", SENSOR_QOS)
        self.create_timer(_env_float("S2E_MOCK_LIDAR_PERIOD_S", 0.2), self.publish_points)
        self.set_status("ACTIVE", "ACTIVE", message="mock lidar publishing")

    def publish_points(self) -> None:
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.height = 1
        message.width = 1
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = 16
        message.data = struct.pack("ffff", 1.0, 0.0, 0.0, 1.0)
        message.is_dense = True
        self.publisher.publish(message)


class CameraMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract)
        self.image_publisher = self.create_publisher(Image, "/s2e/sensors/camera/image", SENSOR_QOS)
        self.info_publisher = self.create_publisher(CameraInfo, "/s2e/sensors/camera/camera_info", SENSOR_QOS)
        self.frame_index = 0
        self.sensor_config = _load_sensor_config_or_default("camera")
        self.camera_intrinsic = self.sensor_config.intrinsic if self.sensor_config is not None else None
        self.width = self.camera_intrinsic.image_width if self.camera_intrinsic is not None else int(_env_float("S2E_MOCK_CAMERA_WIDTH", 640.0))
        self.height = self.camera_intrinsic.image_height if self.camera_intrinsic is not None else int(_env_float("S2E_MOCK_CAMERA_HEIGHT", 480.0))
        self.frame_id = self.sensor_config.child_frame if self.sensor_config is not None else "camera"
        self.mode = os.environ.get("S2E_MOCK_CAMERA_MODE", "white")
        self.create_timer(_env_float("S2E_MOCK_CAMERA_PERIOD_S", 0.1), self.publish_image)
        self.set_status("ACTIVE", "ACTIVE", message="mock camera publishing")

    def publish_image(self) -> None:
        stamp = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = self.height
        image.width = self.width
        image.encoding = "rgb8"
        image.is_bigendian = False
        image.step = image.width * 3
        if self.mode == "white":
            image.data = bytes([255, 255, 255]) * image.height * image.width
        else:
            pixel = self.frame_index % 255
            image.data = bytes([pixel, 32, 128] * image.height * image.width)
        self.image_publisher.publish(image)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.height = image.height
        info.width = image.width
        if self.camera_intrinsic is not None:
            info.distortion_model = self.camera_intrinsic.distortion_model
            info.d = list(self.camera_intrinsic.distortion_coefficients)
            info.k = list(self.camera_intrinsic.camera_matrix_row_major)
            info.r = list(self.camera_intrinsic.rectification_matrix_row_major)
            info.p = list(self.camera_intrinsic.projection_matrix_row_major)
        else:
            info.k = [float(self.width), 0.0, self.width / 2.0, 0.0, float(self.height), self.height / 2.0, 0.0, 0.0, 1.0]
        self.info_publisher.publish(info)
        self.frame_index += 1


class ImuMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract)
        self.sensor_config = _load_sensor_config_or_default("imu")
        self.frame_id = self.sensor_config.child_frame if self.sensor_config is not None else "imu"
        self.publisher = self.create_publisher(Imu, "/s2e/sensors/imu", SENSOR_QOS)
        self.create_timer(_env_float("S2E_MOCK_IMU_PERIOD_S", 0.01), self.publish_imu)
        self.set_status("ACTIVE", "ACTIVE", message="mock imu publishing")

    def publish_imu(self) -> None:
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.orientation.w = 1.0
        message.angular_velocity.z = 0.05
        message.linear_acceleration.z = 9.81
        self.publisher.publish(message)


class OdometryMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract, motion_critical=True)
        self.publisher = self.create_publisher(StampedPose, "/s2e/odometry/pose", RELIABLE_QOS)
        self.estimator = MockOdometryEstimator()
        self.last_lidar = None
        self.last_camera = None
        self.last_imu = None
        
        if self.use_mock_hardware:
            self.create_subscription(PointCloud2, "/s2e/sensors/lidar/points", lambda msg: setattr(self, "last_lidar", msg), SENSOR_QOS)
            self.create_subscription(Image, "/s2e/sensors/camera/image", lambda msg: setattr(self, "last_camera", msg), SENSOR_QOS)
            self.create_subscription(Imu, "/s2e/sensors/imu", lambda msg: setattr(self, "last_imu", msg), SENSOR_QOS)
            self.create_timer(_env_float("S2E_MOCK_ODOMETRY_PERIOD_S", 0.02), self.publish_pose)
            self.set_status("WAITING_INPUTS", "WAITING_INPUTS", message="waiting for sensor streams")
        else:
            # Real hardware mode: subscribe to actual LIO outputs
            self.declare_parameter("lio_pose_topic", "/utlidar/robot_pose")
            self.declare_parameter("lio_odom_topic", "/odom")
            lio_pose_topic = self.get_parameter("lio_pose_topic").value
            lio_odom_topic = self.get_parameter("lio_odom_topic").value

            self.create_subscription(PoseStamped, lio_pose_topic, self._on_real_pose, RELIABLE_QOS)
            self.create_subscription(Odometry, lio_odom_topic, self._on_real_odom, RELIABLE_QOS)
            self.set_status("ACTIVE", "ACTIVE", message="monitoring real LIO topics")

    def _on_real_pose(self, msg: PoseStamped) -> None:
        stamped_pose = StampedPose()
        stamped_pose.header = msg.header
        stamped_pose.header.frame_id = "odom"
        stamped_pose.child_frame_id = "base_link"
        stamped_pose.source_stamp = msg.header.stamp
        stamped_pose.processed_stamp = self.get_clock().now().to_msg()
        stamped_pose.pose = msg.pose
        stamped_pose.confidence = 1.0
        stamped_pose.status = "ACTIVE"
        self.publisher.publish(stamped_pose)
        self.set_status("ACTIVE", "ACTIVE", message="publishing base_link pose from real PoseStamped")

    def _on_real_odom(self, msg: Odometry) -> None:
        stamped_pose = StampedPose()
        stamped_pose.header = msg.header
        stamped_pose.header.frame_id = "odom"
        stamped_pose.child_frame_id = "base_link"
        stamped_pose.source_stamp = msg.header.stamp
        stamped_pose.processed_stamp = self.get_clock().now().to_msg()
        stamped_pose.pose = msg.pose.pose
        stamped_pose.confidence = 1.0
        stamped_pose.status = "ACTIVE"
        self.publisher.publish(stamped_pose)
        self.set_status("ACTIVE", "ACTIVE", message="publishing base_link pose from real Odometry")

    def publish_pose(self) -> None:
        if self.last_lidar is None or self.last_camera is None or self.last_imu is None:
            self.set_status("WAITING_INPUTS", "WAITING_INPUTS", message="waiting for sensor streams")
            return
        stamp_msg = self.get_clock().now().to_msg()
        stamp = _time_to_float(stamp_msg)
        pose = self.estimator.estimate(stamp)
        qz, qw = _yaw_to_quaternion(pose.pose.yaw)
        message = StampedPose()
        message.header.stamp = stamp_msg
        message.header.frame_id = pose.frame_id
        message.source_stamp = stamp_msg
        message.processed_stamp = self.get_clock().now().to_msg()
        message.child_frame_id = pose.child_frame_id
        message.pose.position.x = pose.pose.x
        message.pose.position.y = pose.pose.y
        message.pose.position.z = 0.0
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        message.confidence = pose.confidence
        message.status = pose.status
        self.publisher.publish(message)
        self.set_status("ACTIVE", "ACTIVE", message="mock odometry publishing base_link pose")


class VlmMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract, motion_critical=True)
        self.vlm_backend = os.environ.get("VLM_BACKEND", "mock").strip().lower()
        self.vlm_api_url = os.environ.get("VLM_API_URL", "")
        self.vlm_api_timeout_s = _env_float("VLM_API_TIMEOUT_S", 10.0)
        self.vlm_api_max_retries = int(_env_float("VLM_API_MAX_RETRIES", 1.0))
        scenarios = [item.strip() for item in os.environ.get("S2E_MOCK_VLM_SCENARIOS", "go").split(",") if item.strip()]
        self.reasoner = MockVlmReasoner(scenarios)
        self.publisher = self.create_publisher(String, "/s2e/vlm/reasoning", RELIABLE_QOS)
        self.rotate_client = ActionClient(self, Rotate, "/s2e/controller/rotate")
        self.last_image: Image | None = None
        self.last_pose: StampedPose | None = None
        self.rotate_active = False
        self.create_subscription(Image, "/s2e/sensors/camera/image", self._on_image, SENSOR_QOS)
        self.create_subscription(StampedPose, "/s2e/odometry/pose", self._on_pose, RELIABLE_QOS)
        self.create_timer(_env_float("S2E_MOCK_VLM_PERIOD_S", 2.0), self.publish_reasoning)
        self.set_status("WAITING_SYNC", "WAITING_SYNC", message="waiting for image and pose")

    def _on_image(self, message: Image) -> None:
        self.last_image = message

    def _on_pose(self, message: StampedPose) -> None:
        self.last_pose = message

    def publish_reasoning(self) -> None:
        if self.rotate_active:
            self.set_status("FROZEN_ROTATING", "FROZEN_ROTATING", message="rotate action in progress")
            return
        if self.last_image is None or self.last_pose is None:
            self.set_status("WAITING_SYNC", "WAITING_SYNC", message="waiting for image and pose")
            return
        image_stamp = _time_to_float(self.last_image.header.stamp)
        pose_stamp = _time_to_float(self.last_pose.header.stamp)
        if image_stamp - pose_stamp > 0.20:
            self.set_status("STALE_INPUT", "STALE_INPUT", healthy=False, error_code="POSE_STALE", message="pose too old for image")
            return
        payload = self._reason(image_stamp)
        if payload is None:
            return
        message = String()
        message.data = payload
        self.publisher.publish(message)
        parsed = parse_vlm_reasoning(payload)
        if parsed.valid and parsed.action == VlmAction.ROTATE:
            self._send_rotate_goal(parsed.rotate_deg)
        else:
            self.set_status("ACTIVE", "ACTIVE", healthy=parsed.valid, error_code="" if parsed.valid else parsed.reason, message="mock VLM reasoning published")

    def _reason(self, image_stamp: float) -> str | None:
        pose = _pose2d_from_stamped_pose(self.last_pose)
        if self.vlm_backend == "qwen_api":
            return self._call_qwen_api(image_stamp, pose)
        return self.reasoner.reason(image_stamp, pose)

    def _call_qwen_api(self, image_stamp: float, pose: Pose2D) -> str | None:
        if not self.vlm_api_url:
            self.set_status("DEGRADED", "API_UNAVAILABLE", healthy=False, error_code="API_UNAVAILABLE", message="VLM_API_URL is not configured")
            return None
        request_payload = {
            "model": os.environ.get("VLM_API_MODEL", "qwen3-vl-thinking"),
            "messages": [
                {
                    "role": "user",
                    "content": json.dumps(
                        {
                            "task": "Return strict S2E VLM JSON only.",
                            "image_stamp": image_stamp,
                            "pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
                        }
                    ),
                }
            ],
            "temperature": 0.0,
        }
        body = json.dumps(request_payload).encode("utf-8")
        last_error = ""
        for _attempt in range(max(1, self.vlm_api_max_retries + 1)):
            request = urllib.request.Request(self.vlm_api_url, data=body, headers={"Content-Type": "application/json"}, method="POST")
            try:
                with urllib.request.urlopen(request, timeout=self.vlm_api_timeout_s) as response:
                    response_payload = json.loads(response.read().decode("utf-8"))
                content = self._extract_qwen_content(response_payload)
                parsed = parse_vlm_reasoning(content)
                if not parsed.valid:
                    self.set_status("DEGRADED", "INVALID_API_RESPONSE", healthy=False, error_code=parsed.reason, message="Qwen API returned malformed VLM JSON")
                    return None
                return content
            except urllib.error.URLError as exc:
                last_error = str(exc)
            except TimeoutError as exc:
                last_error = str(exc)
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                last_error = str(exc)
                break
        self.set_status("DEGRADED", "API_TIMEOUT", healthy=False, error_code="API_TIMEOUT", message=last_error or "Qwen API request failed")
        return None

    def _extract_qwen_content(self, response_payload: dict) -> str:
        choices = response_payload.get("choices")
        if isinstance(choices, list) and choices:
            message = choices[0].get("message", {})
            content = message.get("content", "")
            if isinstance(content, str):
                return content
        content = response_payload.get("content")
        if isinstance(content, str):
            return content
        raise ValueError("Qwen API response does not contain text content")

    def _send_rotate_goal(self, rotate_deg: float) -> None:
        if not self.rotate_client.wait_for_server(timeout_sec=0.0):
            self.set_status("FAULT", "ACTION_UNAVAILABLE", healthy=False, error_code="ACTION_SERVER_MISSING", message="rotate action server unavailable")
            return
        goal = Rotate.Goal()
        goal.target_yaw_delta_deg = float(rotate_deg)
        goal.max_yaw_rate_deg_s = 30.0
        goal.tolerance_deg = 3.0
        goal.timeout_s = max(3.0, abs(float(rotate_deg)) / 30.0 + 2.0)
        self.rotate_active = True
        self.set_status("FROZEN_ROTATING", "FROZEN_ROTATING", message="rotate goal sent")
        self.publish_heartbeat()
        future = self.rotate_client.send_goal_async(goal)
        future.add_done_callback(self._on_rotate_goal_response)

    def _on_rotate_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.rotate_active = False
            self.set_status("DEGRADED", "ROTATE_REJECTED", healthy=False, error_code="ROTATE_REJECTED", message="rotate goal rejected")
            self.publish_heartbeat()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotate_result)

    def _on_rotate_result(self, future) -> None:
        result = future.result().result
        self.rotate_active = False
        self.set_status("ACTIVE", "ACTIVE" if result.success else "DEGRADED", healthy=result.success, error_code="" if result.success else result.result_code, message=result.message)
        self.publish_heartbeat()


class E2EMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract, motion_critical=True)
        self.e2e_backend = os.environ.get("E2E_BACKEND", "mock").strip().lower()
        self.image_context = S2EFrameContext()
        self.image_context_error = ""
        self.planner = self._make_planner()
        self.trajectory_publisher = self.create_publisher(Trajectory2D, "/s2e/e2e/trajectory", RELIABLE_QOS)
        self.e2e_status_publisher = self.create_publisher(NodeStatus, "/s2e/e2e/status", STATUS_QOS)
        self.last_image: Image | None = None
        self.last_pose: StampedPose | None = None
        self.last_vlm: String | None = None
        self.camera_sensor_config = _load_sensor_config_or_default("camera")
        self.ground_z_m = _env_float("S2E_MOCK_GROUND_Z_M", 0.0)
        self.max_goal_distance_m = _env_float("S2E_MOCK_MAX_GOAL_DISTANCE_M", 8.0)
        self.health = SystemHealth(ok_to_move=True, overall_state="OK", unhealthy_nodes=[], missing_critical_nodes=[], reason="")
        self.create_subscription(Image, "/s2e/sensors/camera/image", self._on_image, SENSOR_QOS)
        self.create_subscription(StampedPose, "/s2e/odometry/pose", self._on_pose, RELIABLE_QOS)
        self.create_subscription(String, "/s2e/vlm/reasoning", self._on_vlm, RELIABLE_QOS)
        self.create_subscription(SystemHealth, "/s2e/supervisor/health", self._on_health, RELIABLE_QOS)
        self.create_timer(_env_float("S2E_MOCK_E2E_PERIOD_S", 0.2), self.run_cycle)
        self.set_status("WAITING_FIRST_VLM", "WAITING_FIRST_VLM", message="waiting for VLM reasoning")

    def _make_planner(self):
        if self.e2e_backend == "s2e":
            return S2EPlanner(os.environ.get("E2E_MODEL_PATH", "/models/s2e/S2E"))
        return MockE2EPlanner()

    def _on_image(self, message: Image) -> None:
        self.last_image = message
        if self.e2e_backend != "s2e":
            return
        if message.encoding != "rgb8":
            self.image_context_error = f"unsupported image encoding {message.encoding}"
            return
        try:
            frame = ros_rgb8_to_chw_float(bytes(message.data), width=int(message.width), height=int(message.height))
        except ValueError as exc:
            self.image_context_error = str(exc)
            return
        self.image_context.append(frame)
        self.image_context_error = ""

    def _on_pose(self, message: StampedPose) -> None:
        self.last_pose = message

    def _on_vlm(self, message: String) -> None:
        self.last_vlm = message

    def _on_health(self, message: SystemHealth) -> None:
        self.health = message

    def publish_e2e_status(self, state: str, active_mode: str, *, healthy: bool, error_code: str = "", message: str = "") -> None:
        self.set_status(state, active_mode, healthy=healthy, error_code=error_code, message=message)
        status = self.make_status()
        self.e2e_status_publisher.publish(status)
        self.status_publisher.publish(status)

    def run_cycle(self) -> None:
        if self.last_image is None or self.last_pose is None:
            self.publish_e2e_status("WAITING_FIRST_VLM", "WAITING_INPUTS", healthy=True, message="waiting for image and pose")
            return
        if self.last_vlm is None:
            self.publish_e2e_status("WAITING_FIRST_VLM", "WAITING_FIRST_VLM", healthy=True, message="waiting for VLM reasoning")
            return
        parsed = parse_vlm_reasoning(self.last_vlm.data)
        if not parsed.valid:
            self.publish_e2e_status("DEGRADED", "INVALID_VLM", healthy=False, error_code=parsed.reason, message="invalid VLM reasoning")
            return
        if parsed.stamp is None or _time_to_float(self.get_clock().now().to_msg()) - parsed.stamp > _env_float("S2E_MOCK_VLM_TTL_S", 8.0):
            self.publish_e2e_status("DEGRADED", "VLM_STALE", healthy=False, error_code="VLM_STALE", message="cached VLM reasoning expired")
            return
        if parsed.action == VlmAction.STOP:
            self.publish_e2e_status("STOPPED_BY_VLM", "STOPPED_BY_VLM", healthy=True, message="VLM requested stop")
            return
        if parsed.action == VlmAction.ROTATE:
            self.publish_e2e_status("DEGRADED", "ROTATE_IN_PROGRESS", healthy=True, message="rotate action owns controller")
            return
        if parsed.action != VlmAction.GO or parsed.goal_uv is None:
            self.publish_e2e_status("DEGRADED", "INVALID_VLM", healthy=False, error_code="NO_GOAL", message="VLM did not provide a trajectory goal")
            return
        calibration_available = self.camera_sensor_config is not None and self.camera_sensor_config.intrinsic is not None
        goal_point = self._goal_uv_to_base_link(parsed.goal_uv)
        if goal_point is None:
            active_mode = "GOAL_UNPROJECTABLE" if calibration_available else "CALIBRATION_UNAVAILABLE"
            error_code = "GOAL_UNPROJECTABLE" if calibration_available else "CAMERA_CALIBRATION_UNAVAILABLE"
            message = "goal_uv does not intersect bounded base_link ground plane" if calibration_available else "camera intrinsic/extrinsic calibration cannot project goal_uv to base_link ground"
            self.publish_e2e_status(
                "DEGRADED",
                active_mode,
                healthy=False,
                error_code=error_code,
                message=message,
            )
            return
        if not self.health.ok_to_move or "vlm_node" in self.health.unhealthy_nodes or "vlm_node" in self.health.missing_critical_nodes:
            self.publish_e2e_status("DEGRADED", "SUPERVISOR_BLOCKED", healthy=False, error_code="SUPERVISOR_BLOCKED", message="supervisor blocked VLM cache")
            return
        projection_status = "camera_config"
        plan = self._plan_trajectory(goal_point)
        if plan is None:
            return
        trajectory = Trajectory2D()
        trajectory.header.stamp = self.last_image.header.stamp
        trajectory.header.frame_id = plan.frame_id
        trajectory.source_stamp = self.last_image.header.stamp
        trajectory.processed_stamp = self.get_clock().now().to_msg()
        trajectory.pose_at_trajectory = PoseStamped()
        trajectory.pose_at_trajectory.header = self.last_pose.header
        trajectory.pose_at_trajectory.pose = self.last_pose.pose
        trajectory.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in plan.points]
        trajectory.goal_point_base_link = Point32(x=float(plan.goal_point_base_link[0]), y=float(plan.goal_point_base_link[1]), z=0.0)
        trajectory.has_goal_point = plan.has_goal_point
        trajectory.source_vlm_json = self.last_vlm.data
        trajectory.status = f"{plan.status};preprocessed_goal_base_link=({goal_point[0]:.3f},{goal_point[1]:.3f});image_to_base_link_projection={projection_status}"
        self.trajectory_publisher.publish(trajectory)
        self.publish_e2e_status("ACTIVE", "ACTIVE", healthy=True, message=f"trajectory fresh ({plan.status})")

    def _plan_trajectory(self, goal_point: tuple[float, float]):
        current_pose = _pose2d_from_stamped_pose(self.last_pose)
        if self.e2e_backend != "s2e":
            return self.planner.plan(goal_point, current_pose)
        if self.image_context_error:
            self.publish_e2e_status("DEGRADED", "IMAGE_CONTEXT_INVALID", healthy=False, error_code="IMAGE_CONTEXT_INVALID", message=self.image_context_error)
            return None
        obs = self.image_context.batch()
        if obs is None:
            self.publish_e2e_status(
                "WAITING_FIRST_VLM",
                "WAITING_IMAGE_CONTEXT",
                healthy=True,
                message=f"waiting for 11 RGB frames, have {self.image_context.frame_count}",
            )
            return None
        try:
            return self.planner.plan(obs, goal_point, current_pose)
        except Exception as exc:
            self.publish_e2e_status("DEGRADED", "S2E_BACKEND_ERROR", healthy=False, error_code="S2E_BACKEND_ERROR", message=str(exc))
            return None

    def _goal_uv_to_base_link(self, goal_uv: tuple[float, float]) -> tuple[float, float] | None:
        if self.camera_sensor_config is None or self.camera_sensor_config.intrinsic is None:
            return None
        return camera_image_to_base_link_ground(
            goal_uv,
            camera_matrix_row_major=self.camera_sensor_config.intrinsic.camera_matrix_row_major,
            base_from_camera_translation_m=self.camera_sensor_config.translation_m,
            base_from_camera_rotation_matrix_row_major=self.camera_sensor_config.rotation_matrix_row_major,
            ground_z_m=self.ground_z_m,
            max_ground_distance_m=self.max_goal_distance_m,
        )


class ControllerMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract, motion_critical=True)
        self.command_publisher = self.create_publisher(Twist, "/s2e/controller/command", RELIABLE_QOS)
        self.controller_status_publisher = self.create_publisher(NodeStatus, "/s2e/controller/status", STATUS_QOS)
        self.last_pose: StampedPose | None = None
        self.last_pose_monotonic: float | None = None
        self.current_trajectory: Trajectory2D | None = None
        self.trajectory_received_monotonic: float | None = None
        self.health = SystemHealth(ok_to_move=True, overall_state="OK", unhealthy_nodes=[], missing_critical_nodes=[], reason="")
        self.motion_blocked = False
        
        # Real controller parameters
        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 1.2)
        self.declare_parameter("kd_angular", 0.05)
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("lookahead_index", 3)
        self.declare_parameter("kp_rotate", 0.8)
        self.declare_parameter("tolerance_deg", 3.0)
        
        self.kp_linear = self.get_parameter("kp_linear").value
        self.kp_angular = self.get_parameter("kp_angular").value
        self.kd_angular = self.get_parameter("kd_angular").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.lookahead_index = self.get_parameter("lookahead_index").value
        self.kp_rotate = self.get_parameter("kp_rotate").value
        self.tolerance_deg = self.get_parameter("tolerance_deg").value
        
        self.last_yaw_error = 0.0

        self.create_subscription(Trajectory2D, "/s2e/e2e/trajectory", self._on_trajectory, RELIABLE_QOS)
        self.create_subscription(StampedPose, "/s2e/odometry/pose", self._on_pose, RELIABLE_QOS)
        self.create_subscription(NodeStatus, "/s2e/e2e/status", self._on_e2e_status, STATUS_QOS)
        self.create_subscription(SystemHealth, "/s2e/supervisor/health", self._on_health, RELIABLE_QOS)
        self.rotate_server = ActionServer(self, Rotate, "/s2e/controller/rotate", execute_callback=self._execute_rotate)
        
        period = 0.05 if self.use_mock_hardware else 0.02 # 50Hz for real control
        self.create_timer(period, self.control_cycle)
        self.set_status("ACTIVE", "WAITING_TRAJECTORY", message="waiting for trajectory")

    def publish_controller_status(self) -> None:
        status = self.make_status()
        self.controller_status_publisher.publish(status)
        self.status_publisher.publish(status)

    def _on_trajectory(self, message: Trajectory2D) -> None:
        if message.header.frame_id != "base_link":
            self.current_trajectory = None
            self.trajectory_received_monotonic = None
            self.motion_blocked = True
            self.set_status("FAULT", "FAULT", healthy=False, error_code="FRAME_MISMATCH", message="trajectory frame must be base_link")
            self.publish_controller_status()
            return
        self.current_trajectory = message
        self.trajectory_received_monotonic = time.monotonic()
        self.motion_blocked = False

    def _on_pose(self, message: StampedPose) -> None:
        self.last_pose = message
        self.last_pose_monotonic = time.monotonic()

    def _on_e2e_status(self, message: NodeStatus) -> None:
        if message.active_mode in {"STOPPED_BY_VLM", "INVALID_VLM", "VLM_STALE", "SUPERVISOR_BLOCKED", "CALIBRATION_UNAVAILABLE", "GOAL_UNPROJECTABLE"}:
            self.current_trajectory = None
            self.trajectory_received_monotonic = None
            self.motion_blocked = True
            self.set_status("ACTIVE", "STOPPING", message=message.active_mode)

    def _on_health(self, message: SystemHealth) -> None:
        self.health = message
        if not message.ok_to_move:
            self.current_trajectory = None
            self.trajectory_received_monotonic = None
            self.motion_blocked = True
            self.set_status("DEGRADED", "STOPPING", healthy=False, error_code="SUPERVISOR_BLOCKED", message=message.reason)

    def control_cycle(self) -> None:
        command = Twist()
        now = time.monotonic()
        
        # In real hardware, we allow up to 0.2s of odom age bound to avoid sudden stops on network jitter
        odom_timeout = 0.10 if self.use_mock_hardware else 0.20
        if self.last_pose_monotonic is not None and now - self.last_pose_monotonic > odom_timeout:
            self.current_trajectory = None
            self.set_status("FAULT", "FAULT", healthy=False, error_code="ODOM_STALE", message="odometry exceeded controller age bound")
        elif self.motion_blocked or not self.health.ok_to_move:
            self.set_status(self.state if self.state in {"DEGRADED", "FAULT"} else "ACTIVE", "STOPPING", healthy=self.health.ok_to_move, error_code="" if self.health.ok_to_move else "SUPERVISOR_BLOCKED", message="holding zero command")
        elif self.current_trajectory is not None and self.trajectory_received_monotonic is not None:
            # Trajectory TTL check
            ttl = _env_float("S2E_MOCK_TRAJECTORY_TTL_S", 0.50) if self.use_mock_hardware else 1.0 # 1.0s limit for real execution
            if now - self.trajectory_received_monotonic <= ttl:
                if self.use_mock_hardware:
                    # Mock trajectory tracking
                    command.linear.x = 0.2
                    command.angular.z = max(-0.5, min(0.5, float(self.current_trajectory.goal_point_base_link.y) * 0.25))
                    self.set_status("ACTIVE", "FOLLOWING", message="following mock trajectory")
                else:
                    # Real trajectory tracking using PD/PID control
                    points = self.current_trajectory.points
                    idx = min(self.lookahead_index, len(points) - 1)
                    if idx >= 0:
                        # Extract lookahead point in base_link (robot local frame)
                        pt = points[idx]
                        dx = float(pt.x)
                        dy = float(pt.y)
                        
                        # Heading and distance errors
                        heading_error = math.atan2(dy, dx)
                        distance_error = math.hypot(dx, dy)
                        
                        # PD angular control
                        d_heading = heading_error - self.last_yaw_error
                        self.last_yaw_error = heading_error
                        
                        w_cmd = self.kp_angular * heading_error + self.kd_angular * d_heading
                        
                        # Proportional linear control
                        v_cmd = self.kp_linear * distance_error
                        
                        # Saturate velocities
                        command.linear.x = max(0.0, min(self.max_linear_speed, v_cmd))
                        command.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, w_cmd))
                        
                        self.set_status("ACTIVE", "FOLLOWING", message=f"following real trajectory: v={command.linear.x:.2f}, w={command.angular.z:.2f}")
                    else:
                        self.set_status("ACTIVE", "WAITING_TRAJECTORY", error_code="TRAJECTORY_INVALID", message="trajectory points empty")
            else:
                self.current_trajectory = None
                self.trajectory_received_monotonic = None
                self.set_status("ACTIVE", "WAITING_TRAJECTORY", error_code="TRAJECTORY_STALE", message="trajectory TTL expired")
        else:
            self.set_status("ACTIVE", "WAITING_TRAJECTORY", message="waiting for trajectory")
        self.command_publisher.publish(command)
        self.publish_controller_status()

    def _execute_rotate(self, goal_handle):
        result = Rotate.Result()
        
        odom_timeout = 0.10 if self.use_mock_hardware else 0.20
        if self.last_pose_monotonic is None or time.monotonic() - self.last_pose_monotonic > odom_timeout:
            goal_handle.abort()
            result.success = False
            result.result_code = "ODOM_STALE"
            result.final_yaw_delta_deg = 0.0
            result.message = "odometry stale"
            return result
            
        self.current_trajectory = None
        self.trajectory_received_monotonic = None
        self.motion_blocked = False
        target_deg = float(goal_handle.request.target_yaw_delta_deg)
        sign = 1.0 if target_deg >= 0.0 else -1.0
        
        self.set_status("ACTIVE", "ROTATING", message="rotate action active")
        
        if self.use_mock_hardware:
            # Mock rotate logic
            steps = 5
            for index in range(steps):
                current = target_deg * float(index + 1) / float(steps)
                feedback = Rotate.Feedback()
                feedback.current_yaw_delta_deg = current
                feedback.remaining_deg = target_deg - current
                feedback.controller_state = "ROTATING"
                goal_handle.publish_feedback(feedback)
                command = Twist()
                command.angular.z = sign * math.radians(max(1.0, float(goal_handle.request.max_yaw_rate_deg_s)))
                self.command_publisher.publish(command)
                self.publish_controller_status()
                time.sleep(0.05)
            final_deg = target_deg
        else:
            # Real closed-loop rotate logic using odometry pose feedback
            start_pose = _pose2d_from_stamped_pose(self.last_pose)
            start_yaw = start_pose.yaw
            target_yaw = start_yaw + math.radians(target_deg)
            target_yaw = (target_yaw + math.pi) % (2.0 * math.pi) - math.pi
            
            timeout = float(goal_handle.request.timeout_s)
            tolerance = float(goal_handle.request.tolerance_deg)
            w_limit = math.radians(float(goal_handle.request.max_yaw_rate_deg_s))
            
            start_time = time.monotonic()
            
            while time.monotonic() - start_time < timeout:
                if time.monotonic() - self.last_pose_monotonic > 0.20:
                    goal_handle.abort()
                    result.success = False
                    result.result_code = "ODOM_STALE"
                    result.final_yaw_delta_deg = 0.0
                    result.message = "odometry lost during rotation"
                    return result
                
                current_pose = _pose2d_from_stamped_pose(self.last_pose)
                current_yaw = current_pose.yaw
                
                # Remaining angle
                yaw_error = target_yaw - current_yaw
                yaw_error = (yaw_error + math.pi) % (2.0 * math.pi) - math.pi
                yaw_error_deg = math.degrees(yaw_error)
                
                if abs(yaw_error_deg) < tolerance:
                    break
                
                # P-control on yaw velocity
                w_cmd = self.kp_rotate * yaw_error
                w_cmd = max(-w_limit, min(w_limit, w_cmd))
                
                command = Twist()
                command.angular.z = w_cmd
                self.command_publisher.publish(command)
                
                # Publish action feedback
                feedback = Rotate.Feedback()
                delta_yaw = current_yaw - start_yaw
                delta_yaw = (delta_yaw + math.pi) % (2.0 * math.pi) - math.pi
                feedback.current_yaw_delta_deg = math.degrees(delta_yaw)
                feedback.remaining_deg = yaw_error_deg
                feedback.controller_state = "ROTATING"
                goal_handle.publish_feedback(feedback)
                
                self.publish_controller_status()
                time.sleep(0.02)
                
            final_pose = _pose2d_from_stamped_pose(self.last_pose)
            final_delta = final_pose.yaw - start_yaw
            final_delta = (final_delta + math.pi) % (2.0 * math.pi) - math.pi
            final_deg = math.degrees(final_delta)
            
        self.command_publisher.publish(Twist())
        result.success = True
        result.result_code = "SUCCESS"
        result.final_yaw_delta_deg = final_deg
        result.message = "rotate complete"
        goal_handle.succeed()
        self.set_status("ACTIVE", "WAITING_TRAJECTORY", message="rotate complete")
        self.publish_controller_status()
        return result


class SupervisorMockNode(BaseMockNode):
    CRITICAL_NODES = frozenset({"static_tf_node", "odometry_node", "controller_node", "vlm_node", "e2e_node"})

    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract, motion_critical=True)
        self.health_publisher = self.create_publisher(SystemHealth, "/s2e/supervisor/health", STATUS_QOS)
        self.status_by_node: dict[str, tuple[NodeStatus, float]] = {}
        self.started_monotonic = time.monotonic()
        for topic in contract.subscribes:
            self.create_subscription(NodeStatus, topic, self._on_status, STATUS_QOS)
        self.create_timer(_env_float("S2E_MOCK_SUPERVISOR_PERIOD_S", 0.2), self.publish_health)
        self.set_status("ACTIVE", "ACTIVE", message="supervisor monitoring heartbeats")

    def _on_status(self, message: NodeStatus) -> None:
        self.status_by_node[message.node_name] = (message, time.monotonic())

    def publish_health(self) -> None:
        now = time.monotonic()
        threshold = _env_float("S2E_MOCK_HEARTBEAT_TIMEOUT_S", 3.0)
        startup_grace_elapsed = now - self.started_monotonic > threshold
        missing = sorted(
            node
            for node in self.CRITICAL_NODES
            if (node not in self.status_by_node and startup_grace_elapsed)
            or (node in self.status_by_node and now - self.status_by_node[node][1] > threshold)
        )
        unhealthy = sorted(node for node, (status, seen_at) in self.status_by_node.items() if node in self.CRITICAL_NODES and (now - seen_at > threshold or not status.is_healthy))
        health = SystemHealth()
        health.header.stamp = self.get_clock().now().to_msg()
        health.ok_to_move = not missing and not unhealthy
        health.overall_state = "OK" if health.ok_to_move else "DEGRADED"
        health.unhealthy_nodes = unhealthy
        health.missing_critical_nodes = missing
        health.reason = "" if health.ok_to_move else "missing or unhealthy critical heartbeats"
        self.health_publisher.publish(health)
        self.set_status("ACTIVE" if health.ok_to_move else "DEGRADED", "ACTIVE" if health.ok_to_move else "DEGRADED", healthy=health.ok_to_move, error_code="" if health.ok_to_move else "MISSING_CRITICAL", message=health.reason)
        self.publish_heartbeat()


class DebugVisualizerMockNode(BaseMockNode):
    def __init__(self, contract: NodeContract) -> None:
        super().__init__(contract)
        self.publisher = self.create_publisher(Image, "/s2e/debug/visualizer/image", SENSOR_QOS)
        self.last_image: Image | None = None
        self.last_camera_info: CameraInfo | None = None
        self.last_vlm: String | None = None
        self.last_trajectory: Trajectory2D | None = None
        self.last_e2e_status: NodeStatus | None = None
        self.last_controller_status: NodeStatus | None = None
        self.last_health: SystemHealth | None = None
        self.last_node_statuses: dict[str, NodeStatus] = {}
        self.last_node_status_seen_monotonic: dict[str, float] = {}
        self.camera_sensor_config = _load_sensor_config_or_default("camera")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.debug_mode = os.environ.get("S2E_DEBUG_MODE", "0").lower() in {"1", "true", "yes", "on"}
        self.runtime_role = os.environ.get("S2E_RUNTIME_ROLE", "")
        self.artifact_dir = Path(os.environ["S2E_TEST_ARTIFACT_DIR"]) if os.environ.get("S2E_TEST_ARTIFACT_DIR") else None
        self.artifact_duration_s = _env_float("S2E_MOCK_ARTIFACT_DURATION_S", 10.0)
        self.artifact_save_period_s = _env_float("S2E_MOCK_ARTIFACT_SAVE_PERIOD_S", 0.1)
        self.artifact_started_monotonic: float | None = None
        self.last_artifact_save_monotonic = 0.0
        self.artifact_frame_count = 0
        self.video_writer = None
        self.artifacts_complete = False
        self.video_codec = ""
        self.video_pix_fmt = ""
        self.projection_available = False
        self.projected_trajectory_frames = 0
        self.last_projected_point_count = 0
        self.last_projected_total_point_count = 0
        self.last_projected_visible_point_count = 0
        self.last_projected_clipped_point_count = 0
        self.last_projection_status = "waiting"
        self.ground_z_m = _env_float("S2E_MOCK_GROUND_Z_M", 0.0)
        if self.artifact_dir is not None:
            self.artifact_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, "/s2e/sensors/camera/image", self._on_image, SENSOR_QOS)
        self.create_subscription(CameraInfo, "/s2e/sensors/camera/camera_info", self._on_camera_info, SENSOR_QOS)
        self.create_subscription(String, "/s2e/vlm/reasoning", self._on_vlm, RELIABLE_QOS)
        self.create_subscription(Trajectory2D, "/s2e/e2e/trajectory", self._on_trajectory, RELIABLE_QOS)
        self.create_subscription(NodeStatus, "/s2e/e2e/status", self._on_e2e_status, STATUS_QOS)
        self.create_subscription(NodeStatus, "/s2e/controller/status", self._on_controller_status, STATUS_QOS)
        self.create_subscription(SystemHealth, "/s2e/supervisor/health", self._on_health, STATUS_QOS)
        for topic in contract.subscribes:
            if topic.startswith("/s2e/status/"):
                self.create_subscription(NodeStatus, topic, self._on_node_status, STATUS_QOS)
        self.create_timer(_env_float("S2E_MOCK_DEBUG_VISUALIZER_PERIOD_S", 0.2), self.publish_overlay)
        self.set_status("DEGRADED", "DEGRADED", message="waiting for camera image")

    def _on_image(self, message: Image) -> None:
        self.last_image = message

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.last_camera_info = message

    def _on_vlm(self, message: String) -> None:
        self.last_vlm = message

    def _on_trajectory(self, message: Trajectory2D) -> None:
        self.last_trajectory = message

    def _on_e2e_status(self, message: NodeStatus) -> None:
        self.last_e2e_status = message
        if message.active_mode in {"STOPPED_BY_VLM", "INVALID_VLM", "VLM_STALE", "SUPERVISOR_BLOCKED", "CALIBRATION_UNAVAILABLE", "GOAL_UNPROJECTABLE"}:
            self.last_trajectory = None

    def _on_controller_status(self, message: NodeStatus) -> None:
        self.last_controller_status = message

    def _on_health(self, message: SystemHealth) -> None:
        self.last_health = message

    def _on_node_status(self, message: NodeStatus) -> None:
        self.last_node_statuses[message.node_name] = message
        self.last_node_status_seen_monotonic[message.node_name] = time.monotonic()

    def publish_overlay(self) -> None:
        if self.last_image is None:
            self.set_status("DEGRADED", "DEGRADED", message="waiting for camera image")
            return
        frame = self._draw_overlay()
        overlay = Image()
        overlay.header = self.last_image.header
        overlay.height = self.last_image.height
        overlay.width = self.last_image.width
        overlay.encoding = self.last_image.encoding
        overlay.is_bigendian = self.last_image.is_bigendian
        overlay.step = self.last_image.step
        overlay.data = frame.tobytes()
        self.publisher.publish(overlay)
        self._save_artifact_frame(frame)
        self.set_status("ACTIVE", "ACTIVE", message="mock debug overlay published")

    def _draw_overlay(self):
        assert self.last_image is not None
        height = int(self.last_image.height)
        width = int(self.last_image.width)
        frame = np.frombuffer(bytes(self.last_image.data), dtype=np.uint8).reshape((height, width, 3)).copy()
        parsed = parse_vlm_reasoning(self.last_vlm.data) if self.last_vlm is not None else None
        y = 28
        cv2.putText(frame, "S2E VLM Mock Debug", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
        if parsed is not None and parsed.valid:
            cv2.putText(frame, f"VLM: {parsed.action.value}", (16, y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 0), 2, cv2.LINE_AA)
            if parsed.goal_uv is not None:
                u, v = parsed.goal_uv
                point = (int(round(u)), int(round(v)))
                cv2.circle(frame, point, 11, (255, 0, 0), 2, cv2.LINE_AA)
                cv2.drawMarker(frame, point, (255, 0, 0), cv2.MARKER_CROSS, 26, 2, cv2.LINE_AA)
                cv2.putText(frame, f"goal_uv=({u:.0f},{v:.0f})", (16, y + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
        elif parsed is not None:
            cv2.putText(frame, f"VLM INVALID: {parsed.reason}", (16, y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "VLM: waiting", (16, y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 2, cv2.LINE_AA)

        if self.last_controller_status is not None:
            cv2.putText(frame, f"CTRL: {self.last_controller_status.active_mode}", (16, y + 84), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 160), 2, cv2.LINE_AA)
        if self.last_e2e_status is not None:
            cv2.putText(frame, f"E2E: {self.last_e2e_status.active_mode}", (16, y + 112), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 160), 2, cv2.LINE_AA)
        if self.last_health is not None:
            cv2.putText(frame, f"health ok={self.last_health.ok_to_move}", (16, y + 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 160), 2, cv2.LINE_AA)
        if self.last_trajectory is not None:
            self._draw_projected_trajectory(frame, self.last_trajectory)
            self._draw_trajectory_minimap(frame, self.last_trajectory)
        if self.debug_mode:
            self._draw_debug_panel(frame)
        return frame

    def _draw_debug_panel(self, frame) -> None:
        height, width, _ = frame.shape
        panel_w = 270
        left = max(0, width - panel_w - 8)
        top = 8
        bottom = min(height - 160, 210)
        cv2.rectangle(frame, (left, top), (width - 8, bottom), (255, 255, 255), -1)
        cv2.rectangle(frame, (left, top), (width - 8, bottom), (0, 0, 0), 1)
        y = top + 18
        role = self.runtime_role or "unknown"
        lines = [
            f"DEBUG role={role}",
            f"domain={os.environ.get('ROS_DOMAIN_ID', '')} rmw={os.environ.get('RMW_IMPLEMENTATION', '')}",
        ]
        if self.last_health is not None:
            missing = ",".join(self.last_health.missing_critical_nodes) or "-"
            unhealthy = ",".join(self.last_health.unhealthy_nodes) or "-"
            lines.extend([
                f"health={self.last_health.overall_state} ok={self.last_health.ok_to_move}",
                f"missing={missing[:34]}",
                f"unhealthy={unhealthy[:32]}",
            ])
        if self.last_vlm is not None:
            parsed = parse_vlm_reasoning(self.last_vlm.data)
            action = parsed.action.value if parsed.valid and parsed.action is not None else f"invalid:{parsed.reason}"
            lines.append(f"vlm={action}")
        if self.last_e2e_status is not None:
            lines.append(f"e2e={self.last_e2e_status.active_mode} {self.last_e2e_status.error_code}"[:42])
        if self.last_controller_status is not None:
            lines.append(f"ctrl={self.last_controller_status.active_mode} {self.last_controller_status.error_code}"[:42])
        lines.append(f"projection={self.last_projection_status}")
        for name in ("static_tf_node", "odometry_node", "controller_node", "vlm_node", "e2e_node"):
            status = self.last_node_statuses.get(name)
            if status is None:
                lines.append(f"{name}: missing")
                continue
            seen_at = self.last_node_status_seen_monotonic.get(name, time.monotonic())
            age = time.monotonic() - seen_at
            lines.append(f"{name}: {status.active_mode} {age:.1f}s"[:42])
        for line in lines[:11]:
            cv2.putText(frame, line, (left + 6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 0), 1, cv2.LINE_AA)
            y += 17

    def _lookup_camera_from_base(self) -> TransformStamped | None:
        camera_frame = self.last_camera_info.header.frame_id if self.last_camera_info is not None else None
        if not camera_frame and self.camera_sensor_config is not None:
            camera_frame = self.camera_sensor_config.child_frame
        if not camera_frame:
            return None
        try:
            return self.tf_buffer.lookup_transform(camera_frame, "base_link", Time())
        except TransformException:
            return None

    def _project_base_point(self, transform: TransformStamped, point: Point32) -> tuple[int, int, bool] | None:
        if self.last_camera_info is None:
            return None
        q = transform.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        m00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        m01 = 2.0 * (qx * qy - qz * qw)
        m02 = 2.0 * (qx * qz + qy * qw)
        m10 = 2.0 * (qx * qy + qz * qw)
        m11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        m12 = 2.0 * (qy * qz - qx * qw)
        m20 = 2.0 * (qx * qz - qy * qw)
        m21 = 2.0 * (qy * qz + qx * qw)
        m22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        point_z = self.ground_z_m
        camera_x = m00 * point.x + m01 * point.y + m02 * point_z + tx
        camera_y = m10 * point.x + m11 * point.y + m12 * point_z + ty
        camera_z = m20 * point.x + m21 * point.y + m22 * point_z + tz
        if camera_z <= 1e-9:
            return None
        k = self.last_camera_info.k
        u = k[0] * camera_x / camera_z + k[2]
        v = k[4] * camera_y / camera_z + k[5]
        if not math.isfinite(u) or not math.isfinite(v):
            return None
        max_u = max(0.0, float(self.last_camera_info.width) - 1.0)
        max_v = max(0.0, float(self.last_camera_info.height) - 1.0)
        inside = 0.0 <= u <= max_u and 0.0 <= v <= max_v
        if not inside:
            return None
        return int(round(u)), int(round(v)), True

    def _trajectory_color(self, index: int, total: int) -> tuple[int, int, int]:
        palette = (
            (0, 80, 255),
            (0, 140, 255),
            (0, 200, 255),
            (0, 220, 160),
            (40, 210, 80),
            (140, 200, 40),
            (220, 170, 40),
            (255, 110, 40),
            (255, 60, 120),
            (180, 60, 255),
        )
        if total <= 1:
            return palette[0]
        palette_index = round(index * (len(palette) - 1) / (total - 1))
        return palette[int(max(0, min(len(palette) - 1, palette_index)))]

    def _draw_projected_trajectory(self, frame, trajectory: Trajectory2D) -> None:
        transform = self._lookup_camera_from_base()
        if transform is None:
            self.projection_available = False
            self.last_projected_point_count = 0
            self.last_projected_total_point_count = 0
            self.last_projected_visible_point_count = 0
            self.last_projected_clipped_point_count = 0
            self.last_projection_status = "unavailable:no_tf_or_camera_info"
            cv2.putText(frame, "projection unavailable", (16, frame.shape[0] - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 180), 2, cv2.LINE_AA)
            return
        points = []
        visible_count = 0
        total_count = len(trajectory.points)
        for index, point in enumerate(trajectory.points):
            projected = self._project_base_point(transform, point)
            if projected is not None:
                u, v, inside = projected
                points.append((u, v, index, self._trajectory_color(index, total_count)))
                if inside:
                    visible_count += 1
        self.last_projected_point_count = len(points)
        self.last_projected_total_point_count = total_count
        self.last_projected_visible_point_count = visible_count
        self.last_projected_clipped_point_count = len(points) - visible_count
        self.projection_available = len(points) >= 2
        if len(points) < 2:
            self.last_projection_status = f"unavailable:visible_points={len(points)}/{total_count}"
            cv2.putText(frame, "projection unavailable", (16, frame.shape[0] - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 180), 2, cv2.LINE_AA)
            return
        for before, after in zip(points, points[1:]):
            cv2.line(frame, (before[0], before[1]), (after[0], after[1]), after[3], 3, cv2.LINE_AA)
        for u, v, index, color in points:
            cv2.circle(frame, (u, v), 4, color, -1, cv2.LINE_AA)
            if index in {0, len(trajectory.points) - 1}:
                cv2.putText(frame, str(index + 1), (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        cv2.circle(frame, (points[-1][0], points[-1][1]), 7, points[-1][3], 2, cv2.LINE_AA)
        cv2.putText(frame, "projected base_link trajectory", (16, frame.shape[0] - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 120), 2, cv2.LINE_AA)
        self.last_projection_status = f"projected:{visible_count}/{total_count}"
        self.projected_trajectory_frames += 1

    def _draw_trajectory_minimap(self, frame, trajectory: Trajectory2D) -> None:
        height, width, _ = frame.shape
        map_w = 180
        map_h = 130
        left = width - map_w - 18
        top = height - map_h - 18
        right = left + map_w
        bottom = top + map_h
        cv2.rectangle(frame, (left, top), (right, bottom), (245, 245, 245), -1)
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 0), 2)
        origin = (left + map_w // 2, bottom - 18)
        cv2.line(frame, (origin[0], top + 10), origin, (160, 160, 160), 1, cv2.LINE_AA)
        cv2.line(frame, (left + 10, origin[1]), (right - 10, origin[1]), (160, 160, 160), 1, cv2.LINE_AA)
        cv2.putText(frame, "+x", (origin[0] + 4, top + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, "+y left", (left + 12, origin[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, "-y right", (right - 70, origin[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)
        points = []
        total_count = len(trajectory.points)
        for index, point in enumerate(trajectory.points):
            px = int(origin[0] - point.y * 35.0)
            py = int(origin[1] - point.x * 25.0)
            points.append((max(left + 4, min(right - 4, px)), max(top + 4, min(bottom - 4, py)), index, self._trajectory_color(index, total_count)))
        for before, after in zip(points, points[1:]):
            cv2.line(frame, (before[0], before[1]), (after[0], after[1]), after[3], 2, cv2.LINE_AA)
        for px, py, index, color in points:
            cv2.circle(frame, (px, py), 4, color, -1, cv2.LINE_AA)
            if index in {0, len(trajectory.points) - 1}:
                cv2.putText(frame, str(index + 1), (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1, cv2.LINE_AA)
        goal = trajectory.goal_point_base_link
        cv2.putText(frame, f"base_link goal x={goal.x:.1f} y={goal.y:.1f}", (left + 8, top + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

    def _save_artifact_frame(self, frame) -> None:
        if self.artifact_dir is None or self.artifacts_complete:
            return
        now = time.monotonic()
        if self.artifact_started_monotonic is None:
            self.artifact_started_monotonic = now
        if now - self.last_artifact_save_monotonic < self.artifact_save_period_s:
            return
        elapsed = now - self.artifact_started_monotonic
        if elapsed > self.artifact_duration_s:
            self._finalize_artifacts(frame)
            return
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        png_path = self.artifact_dir / f"frame_{self.artifact_frame_count:04d}.png"
        cv2.imwrite(str(png_path), bgr)
        self.artifact_frame_count += 1
        self.last_artifact_save_monotonic = now
        self._write_manifest(complete=False)

    def _finalize_artifacts(self, frame) -> None:
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        self._encode_mp4_from_png_sequence()
        self.artifacts_complete = True
        self._write_manifest(complete=True)

    def _encode_mp4_from_png_sequence(self) -> None:
        if self.artifact_dir is None or self.artifact_frame_count == 0:
            return
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            return
        fps = max(1.0, 1.0 / max(self.artifact_save_period_s, 1e-3))
        video_path = self.artifact_dir / "visualizer.mp4"
        pattern = self.artifact_dir / "frame_%04d.png"
        command = [
            ffmpeg,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-framerate",
            f"{fps:.3f}",
            "-i",
            str(pattern),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(video_path),
        ]
        try:
            subprocess.run(command, check=True, timeout=30.0)
        except (subprocess.SubprocessError, OSError):
            self.video_codec = ""
            self.video_pix_fmt = ""
            return
        self.video_codec = "h264"
        self.video_pix_fmt = "yuv420p"

    def _write_manifest(self, *, complete: bool) -> None:
        if self.artifact_dir is None:
            return
        manifest = {
            "complete": complete,
            "debug_mode": self.debug_mode,
            "runtime_context": self._runtime_context_snapshot(),
            "frame_count": self.artifact_frame_count,
            "format": "png",
            "video": "visualizer.mp4",
            "video_codec": self.video_codec,
            "video_pix_fmt": self.video_pix_fmt,
            "width": int(self.last_image.width) if self.last_image is not None else 0,
            "height": int(self.last_image.height) if self.last_image is not None else 0,
            "encoding": self.last_image.encoding if self.last_image is not None else "",
            "projection_available": self.projection_available,
            "projected_trajectory_frames": self.projected_trajectory_frames,
            "last_projected_point_count": self.last_projected_point_count,
            "last_projected_total_point_count": self.last_projected_total_point_count,
            "last_projected_visible_point_count": self.last_projected_visible_point_count,
            "last_projected_clipped_point_count": self.last_projected_clipped_point_count,
            "last_projection_status": {"status": self.last_projection_status},
            "last_supervisor_health": self._health_snapshot(self.last_health),
            "last_node_statuses": {name: self._node_status_snapshot(status) for name, status in sorted(self.last_node_statuses.items())},
            "last_vlm_parse": self._vlm_snapshot(),
            "last_e2e_status": self._node_status_snapshot(self.last_e2e_status),
            "last_controller_status": self._node_status_snapshot(self.last_controller_status),
        }
        (self.artifact_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")

    def _runtime_context_snapshot(self) -> dict[str, str]:
        return {
            "role": self.runtime_role,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
            "namespace": os.environ.get("ROS_NAMESPACE", ""),
        }

    def _node_status_snapshot(self, status: NodeStatus | None) -> dict[str, object]:
        if status is None:
            return {}
        seen_at = self.last_node_status_seen_monotonic.get(status.node_name)
        return {
            "node_name": status.node_name,
            "state": status.state,
            "active_mode": status.active_mode,
            "is_healthy": bool(status.is_healthy),
            "is_motion_critical": bool(status.is_motion_critical),
            "last_input_age_s": float(status.last_input_age_s),
            "last_output_age_s": float(status.last_output_age_s),
            "error_code": status.error_code,
            "message": status.message,
            "heartbeat_age_s": None if seen_at is None else round(time.monotonic() - seen_at, 3),
        }

    def _health_snapshot(self, health: SystemHealth | None) -> dict[str, object]:
        if health is None:
            return {}
        return {
            "ok_to_move": bool(health.ok_to_move),
            "overall_state": health.overall_state,
            "unhealthy_nodes": list(health.unhealthy_nodes),
            "missing_critical_nodes": list(health.missing_critical_nodes),
            "reason": health.reason,
        }

    def _vlm_snapshot(self) -> dict[str, object]:
        if self.last_vlm is None:
            return {}
        parsed = parse_vlm_reasoning(self.last_vlm.data)
        return {
            "valid": bool(parsed.valid),
            "reason": parsed.reason,
            "action": parsed.action.value if parsed.valid and parsed.action is not None else "",
            "stamp": parsed.stamp,
            "goal_uv": list(parsed.goal_uv) if parsed.goal_uv is not None else None,
            "rotate_deg": parsed.rotate_deg,
        }


def _factory(contract: NodeContract) -> Callable[[NodeContract], Node]:
    factories: dict[str, Callable[[NodeContract], Node]] = {
        "static_tf_node": StaticTfMockNode,
        "lidar_node": LidarMockNode,
        "camera_node": CameraMockNode,
        "imu_node": ImuMockNode,
        "odometry_node": OdometryMockNode,
        "vlm_node": VlmMockNode,
        "e2e_node": E2EMockNode,
        "controller_node": ControllerMockNode,
        "supervisor_node": SupervisorMockNode,
        "debug_visualizer_node": DebugVisualizerMockNode,
    }
    return factories[contract.node_name]


def run_mock_ros_node(contract: NodeContract, args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = _factory(contract)(contract)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
