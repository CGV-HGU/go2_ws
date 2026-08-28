#!/usr/bin/env python3
"""Read-only Unitree L2/odometry collector for P7 safety evidence.

Only ROS subscriptions are created.  The module has no publisher, service
client, Unitree SDK client, command socket, or actuator endpoint.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def _stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("ODOM_QUATERNION_INVALID")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _xyz_from_cloud(message: PointCloud2) -> np.ndarray:
    offsets = {field.name: int(field.offset) for field in message.fields}
    if not all(name in offsets for name in ("x", "y", "z")):
        raise ValueError("POINT_CLOUD_XYZ_FIELDS_MISSING")
    byte_order = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [f"{byte_order}f4", f"{byte_order}f4", f"{byte_order}f4"],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": int(message.point_step),
        }
    )
    count = int(message.width) * int(message.height)
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    return np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float64, copy=False
    )


class LiveL2OdomCollector(Node):
    """Hold the most recent raw cloud and odometry without republishing them."""

    def __init__(self) -> None:
        super().__init__("pixnav_live_l2_odom_guard_no_actuation")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._lock = threading.Lock()
        self._cloud: Optional[PointCloud2] = None
        self._cloud_received_ns: Optional[int] = None
        self._odom_samples: deque[tuple[int, int, Odometry]] = deque(maxlen=400)
        self._odom_history: deque[tuple[int, float, float, float]] = deque(maxlen=300)
        self._counts = {"cloud": 0, "odom": 0}
        self.create_subscription(
            PointCloud2,
            "/utlidar/cloud_deskewed",
            self._on_cloud,
            qos,
        )
        self.create_subscription(
            Odometry,
            "/utlidar/robot_odom",
            self._on_odom,
            qos,
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        with self._lock:
            self._cloud = message
            self._cloud_received_ns = time.monotonic_ns()
            self._counts["cloud"] += 1

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        yaw = _yaw_from_quaternion(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        received_ns = time.monotonic_ns()
        with self._lock:
            self._odom_samples.append((_stamp_ns(message), received_ns, message))
            self._odom_history.append(
                (received_ns, float(pose.position.x), float(pose.position.y), yaw)
            )
            self._counts["odom"] += 1

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            cloud = self._cloud
            cloud_received_ns = self._cloud_received_ns
            odom_samples = list(self._odom_samples)
            history = list(self._odom_history)
            counts = dict(self._counts)
        odom = None
        odom_received_ns = None
        if cloud is not None and odom_samples:
            cloud_stamp_ns = _stamp_ns(cloud)
            _, odom_received_ns, odom = min(
                odom_samples,
                key=lambda sample: abs(sample[0] - cloud_stamp_ns),
            )
        base = {
            "schema_version": "go2_l2_odom_safety_snapshot_v1",
            "status": "BLOCKED_SENSOR_INPUT_MISSING",
            "cloud_topic": "/utlidar/cloud_deskewed",
            "odom_topic": "/utlidar/robot_odom",
            "cloud_count": counts["cloud"],
            "odom_count": counts["odom"],
            "cloud_received_monotonic_ns": cloud_received_ns,
            "odom_received_monotonic_ns": odom_received_ns,
            "ros_subscribers_created": 2,
            "ros_publishers_created": 0,
            "unitree_sdk_clients_created": 0,
            "udp_command_senders_created": 0,
            "actuation_calls": 0,
            "frame_interpretation": "deskewed_xyz_in_odom_transformed_to_base_with_robot_odom",
        }
        if cloud is None or odom is None:
            return base
        try:
            points_odom = _xyz_from_cloud(cloud)
            pose = odom.pose.pose
            translation = np.asarray(
                [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
            )
            rotation = _rotation_matrix(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
            finite = np.all(np.isfinite(points_odom), axis=1)
            nonzero = np.linalg.norm(points_odom, axis=1) > 0.05
            points_odom = points_odom[finite & nonzero]
            points_base = (points_odom - translation) @ rotation
            height = (points_base[:, 2] >= -0.28) & (points_base[:, 2] <= 0.50)
            front = (
                height
                & (points_base[:, 0] >= 0.15)
                & (points_base[:, 0] <= 2.0)
                & (np.abs(points_base[:, 1]) <= 0.30)
            )
            radius = np.hypot(points_base[:, 0], points_base[:, 1])
            rotation_zone = height & (radius >= 0.20) & (radius <= 1.5)
            front_clearance = float(np.min(points_base[front, 0])) if np.any(front) else None
            rotation_clearance = float(np.min(radius[rotation_zone])) if np.any(rotation_zone) else None

            max_step_m = 0.0
            max_yaw_step_deg = 0.0
            for previous, current in zip(history, history[1:]):
                max_step_m = max(
                    max_step_m,
                    math.hypot(current[1] - previous[1], current[2] - previous[2]),
                )
                yaw_delta = math.atan2(
                    math.sin(current[3] - previous[3]),
                    math.cos(current[3] - previous[3]),
                )
                max_yaw_step_deg = max(max_yaw_step_deg, abs(math.degrees(yaw_delta)))

            cloud_stamp_ns = _stamp_ns(cloud)
            odom_stamp_ns = _stamp_ns(odom)
            base.update(
                {
                    "status": "PASS_LIVE_L2_ODOM_SNAPSHOT",
                    "cloud_frame_id": cloud.header.frame_id,
                    "odom_frame_id": odom.header.frame_id,
                    "odom_child_frame_id": odom.child_frame_id,
                    "cloud_source_stamp_ns": cloud_stamp_ns,
                    "odom_source_stamp_ns": odom_stamp_ns,
                    "cloud_odom_stamp_delta_s": abs(cloud_stamp_ns - odom_stamp_ns) / 1e9,
                    "cloud_records": int(cloud.width) * int(cloud.height),
                    "valid_cloud_points": int(points_base.shape[0]),
                    "front_corridor_points": int(np.count_nonzero(front)),
                    "rotation_zone_points": int(np.count_nonzero(rotation_zone)),
                    "front_clearance_m": (
                        round(front_clearance, 6) if front_clearance is not None else None
                    ),
                    "rotation_clearance_m": (
                        round(rotation_clearance, 6) if rotation_clearance is not None else None
                    ),
                    "max_odom_step_m": round(max_step_m, 9),
                    "max_odom_yaw_step_deg": round(max_yaw_step_deg, 9),
                    "odom_pose": {
                        "x": float(pose.position.x),
                        "y": float(pose.position.y),
                        "z": float(pose.position.z),
                    },
                    "odom_twist": {
                        "vx": float(odom.twist.twist.linear.x),
                        "vy": float(odom.twist.twist.linear.y),
                        "wz": float(odom.twist.twist.angular.z),
                    },
                }
            )
        except Exception as error:
            base.update(
                {
                    "status": "BLOCKED_SENSOR_SNAPSHOT_DECODE",
                    "error_type": type(error).__name__,
                    "error": str(error)[:500],
                }
            )
        return base


class BackgroundLiveL2OdomCollector:
    """Spin the read-only subscriptions while VLM/PixNav work is in flight."""

    def __init__(self) -> None:
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self.node = LiveL2OdomCollector()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, name="pixnav-l2-odom-read-only")
        self._thread.daemon = True

    def _spin(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def start(self) -> None:
        self._thread.start()

    def snapshot(self) -> dict[str, Any]:
        return self.node.snapshot()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()


def _write_snapshot_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, allow_nan=False, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only live L2/odom JSON snapshot writer")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--interval", type=float, default=0.05)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be positive and finite")
    if not math.isfinite(args.interval) or not 0.02 <= args.interval <= 1.0:
        raise SystemExit("--interval must be finite and in [0.02, 1.0]")
    collector = BackgroundLiveL2OdomCollector()
    collector.start()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            value = collector.snapshot()
            value["snapshot_created_monotonic_ns"] = time.monotonic_ns()
            _write_snapshot_atomic(args.output.expanduser().resolve(), value)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        collector.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
