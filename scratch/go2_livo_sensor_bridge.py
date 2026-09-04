#!/usr/bin/env python3
"""Prepare the Go2 built-in LiDAR/IMU odometry stream for RTAB-Map.

Inputs are the built-in Unitree DDS topics.  They share the LiDAR clock, which
is different from the Jetson clock used by the camera.  This node applies one
common clock offset to all three streams, preserving their relative timing.

The built-in deskewed cloud is expressed in ``odom`` and contains zero-padded
records.  RTAB-Map expects each scan in a robot/sensor-local frame.  For every
scan this node looks up the matching Unitree LiDAR odometry pose, transforms
the XYZ values from ``odom`` back to ``base_link``, and removes invalid/padded
points.  It never publishes motion commands.

Published topics:
  /livo/odom   nav_msgs/Odometry      (odom -> base_link)
  /livo/imu    sensor_msgs/Imu        (utlidar_imu)
  /livo/cloud  sensor_msgs/PointCloud2 (base_link)
"""

import copy
import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _rotation_matrix(x: float, y: float, z: float, w: float) -> Optional[np.ndarray]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-8:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _quat_inv(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )


def _gravity_alignment_deg(quaternion: Tuple[float, float, float, float], acceleration: np.ndarray) -> float:
    rotation = _rotation_matrix(*quaternion)
    accel_norm = float(np.linalg.norm(acceleration))
    if rotation is None or accel_norm < 1.0e-6:
        return 180.0
    predicted_up_in_sensor = rotation.T @ np.array([0.0, 0.0, 1.0])
    measured_up_in_sensor = acceleration / accel_norm
    cosine = float(np.clip(np.dot(predicted_up_in_sensor, measured_up_in_sensor), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


class Go2LivoSensorBridge(Node):
    def __init__(self) -> None:
        super().__init__('go2_livo_sensor_bridge')

        self.declare_parameter('cloud_mode', 'deskewed')
        self.declare_parameter('max_odom_cloud_dt', 0.10)
        self.declare_parameter('imu_quaternion_order', 'auto')
        cloud_mode = str(self.get_parameter('cloud_mode').value).lower()
        self.max_odom_cloud_dt_ns = int(
            float(self.get_parameter('max_odom_cloud_dt').value) * 1_000_000_000
        )
        if cloud_mode not in ('deskewed', 'base'):
            raise ValueError("cloud_mode must be 'deskewed' or 'base'")
        self.cloud_mode = cloud_mode
        self.imu_order_mode = str(self.get_parameter('imu_quaternion_order').value).lower()
        if self.imu_order_mode not in ('auto', 'xyzw', 'wxyz'):
            raise ValueError("imu_quaternion_order must be 'auto', 'xyzw' or 'wxyz'")
        self.detected_imu_order: Optional[str] = None

        self.declare_parameter('zero_origin', True)
        self.zero_origin = bool(self.get_parameter('zero_origin').value)
        self.origin_p: Optional[np.ndarray] = None
        self.origin_q: Optional[np.ndarray] = None
        self.origin_rot: Optional[np.ndarray] = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.odom_pub = self.create_publisher(Odometry, '/livo/odom', sensor_qos)
        self.rtabmap_odom_pub = self.create_publisher(Odometry, '/rtabmap/odom', sensor_qos)
        self.imu_pub = self.create_publisher(Imu, '/livo/imu', sensor_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/livo/cloud', cloud_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.clock_offset_ns: Optional[int] = None
        self.last_output_stamp = {'odom': -1, 'imu': -1, 'cloud': -1}
        self.odom_history: Deque[Tuple[int, np.ndarray, np.ndarray, Odometry]] = deque(maxlen=400)
        self.cloud_received = 0
        self.cloud_published = 0
        self.cloud_dropped_no_pose = 0
        self.input_points = 0
        self.output_points = 0

        self.create_subscription(Odometry, '/utlidar/robot_odom', self._odom_callback, sensor_qos)
        self.create_subscription(Imu, '/utlidar/imu', self._imu_callback, sensor_qos)
        cloud_topic = '/utlidar/cloud_deskewed' if cloud_mode == 'deskewed' else '/utlidar/cloud_base'
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_callback, cloud_qos)
        self.create_timer(5.0, self._report_stats)

        self.get_logger().info(
            'Go2 LIVO bridge active: %s -> /livo/cloud (%s), shared LiDAR clock alignment, no actuation'
            % (cloud_topic, 'odom-to-base transform' if cloud_mode == 'deskewed' else 'base-frame passthrough')
        )

    def _aligned_stamp(self, source_ns: int, stream: str):
        now_ns = self.get_clock().now().nanoseconds
        if source_ns <= 0:
            aligned_ns = now_ns
        else:
            observed_offset = now_ns - source_ns
            if self.clock_offset_ns is None:
                self.clock_offset_ns = observed_offset
                self.get_logger().info('LiDAR-to-host clock offset initialized: %.3f s' % (observed_offset / 1e9))
            elif abs(observed_offset - self.clock_offset_ns) > 2_000_000_000:
                self.get_logger().warn('LiDAR clock jump detected; reinitializing the common clock offset')
                self.clock_offset_ns = observed_offset
            elif observed_offset < self.clock_offset_ns:
                # ``receipt - source`` is the clock offset plus nonnegative
                # transport/callback delay.  The first callback after startup
                # can therefore overestimate the offset and stamp all later
                # samples in the future.  Track the lower envelope across the
                # shared LiDAR streams; the per-stream monotonic clamp below
                # protects consumers while a smaller offset settles.
                self.clock_offset_ns = observed_offset
            aligned_ns = source_ns + self.clock_offset_ns

        # ROS consumers reject time that moves backwards. This clamp is normally
        # inactive; it only protects each stream across a source clock reset.
        aligned_ns = max(aligned_ns, self.last_output_stamp[stream] + 1)
        self.last_output_stamp[stream] = aligned_ns
        return rclpy.time.Time(nanoseconds=aligned_ns).to_msg()

    def _odom_callback(self, msg: Odometry) -> None:
        source_ns = _stamp_to_ns(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rotation = _rotation_matrix(q.x, q.y, q.z, q.w)
        if rotation is None:
            return
        translation = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.odom_history.append((source_ns, translation, rotation, msg))

        p_raw = np.array([p.x, p.y, p.z], dtype=np.float64)
        q_raw = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

        if self.zero_origin:
            if self.origin_p is None:
                self.origin_p = p_raw.copy()
                self.origin_q = q_raw.copy()
                self.origin_rot = rotation.copy()
                self.get_logger().info(
                    'Zero-origin calibrated: origin_p=(%.3f, %.3f, %.3f)' % (p.x, p.y, p.z)
                )

            # Transform translation and orientation relative to origin (SE3 relative pose)
            p_rel = self.origin_rot.T @ (p_raw - self.origin_p)
            q_rel = _quat_mult(_quat_inv(self.origin_q), q_raw)
            q_norm = math.sqrt(float(np.sum(q_rel * q_rel)))
            if q_norm > 1e-8:
                q_rel = q_rel / q_norm
            pub_px, pub_py, pub_pz = float(p_rel[0]), float(p_rel[1]), float(p_rel[2])
            pub_qx, pub_qy, pub_qz, pub_qw = float(q_rel[0]), float(q_rel[1]), float(q_rel[2]), float(q_rel[3])
        else:
            pub_px, pub_py, pub_pz = p.x, p.y, p.z
            pub_qx, pub_qy, pub_qz, pub_qw = q.x, q.y, q.z, q.w

        out = copy.deepcopy(msg)
        out.header.stamp = self._aligned_stamp(source_ns, 'odom')
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = pub_px
        out.pose.pose.position.y = pub_py
        out.pose.pose.position.z = pub_pz
        out.pose.pose.orientation.x = pub_qx
        out.pose.pose.orientation.y = pub_qy
        out.pose.pose.orientation.z = pub_qz
        out.pose.pose.orientation.w = pub_qw
        self.odom_pub.publish(out)
        self.rtabmap_odom_pub.publish(out)

        transform = TransformStamped()
        transform.header = copy.deepcopy(out.header)
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = pub_px
        transform.transform.translation.y = pub_py
        transform.transform.translation.z = pub_pz
        transform.transform.rotation.x = pub_qx
        transform.transform.rotation.y = pub_qy
        transform.transform.rotation.z = pub_qz
        transform.transform.rotation.w = pub_qw
        self.tf_broadcaster.sendTransform(transform)

    def _imu_callback(self, msg: Imu) -> None:
        source_ns = _stamp_to_ns(msg.header.stamp)
        out = copy.deepcopy(msg)
        out.header.stamp = self._aligned_stamp(source_ns, 'imu')
        out.header.frame_id = 'utlidar_imu'

        raw = msg.orientation
        candidate_xyzw = (raw.x, raw.y, raw.z, raw.w)
        # Current Go2 built-in DDS firmware places its w,x,y,z array into the
        # ROS x,y,z,w fields without reordering. This is separate from the
        # external unilidar_sdk2 wrapper, whose documented order is x,y,z,w.
        candidate_wxyz = (raw.y, raw.z, raw.w, raw.x)
        if self.detected_imu_order is None:
            if self.imu_order_mode == 'auto':
                acceleration = np.array(
                    [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                    dtype=np.float64,
                )
                xyzw_error = _gravity_alignment_deg(candidate_xyzw, acceleration)
                wxyz_error = _gravity_alignment_deg(candidate_wxyz, acceleration)
                self.detected_imu_order = 'wxyz' if wxyz_error < xyzw_error else 'xyzw'
                self.get_logger().info(
                    'IMU quaternion order auto-detected as %s (gravity residuals: xyzw=%.2f deg, wxyz=%.2f deg)'
                    % (self.detected_imu_order, xyzw_error, wxyz_error)
                )
            else:
                self.detected_imu_order = self.imu_order_mode

        selected = candidate_wxyz if self.detected_imu_order == 'wxyz' else candidate_xyzw
        norm = math.sqrt(sum(value * value for value in selected))
        if norm > 1.0e-8:
            out.orientation.x = selected[0] / norm
            out.orientation.y = selected[1] / norm
            out.orientation.z = selected[2] / norm
            out.orientation.w = selected[3] / norm
        self.imu_pub.publish(out)

    def _nearest_odom(self, stamp_ns: int):
        if not self.odom_history:
            return None
        nearest = min(self.odom_history, key=lambda item: abs(item[0] - stamp_ns))
        if abs(nearest[0] - stamp_ns) > self.max_odom_cloud_dt_ns:
            return None
        return nearest

    @staticmethod
    def _xyz_dtype(msg: PointCloud2) -> np.dtype:
        offsets = {field.name: field.offset for field in msg.fields}
        if not all(name in offsets for name in ('x', 'y', 'z')):
            raise ValueError('PointCloud2 is missing x/y/z fields')
        byte_order = '>' if msg.is_bigendian else '<'
        return np.dtype(
            {
                'names': ['x', 'y', 'z'],
                'formats': [byte_order + 'f4'] * 3,
                'offsets': [offsets['x'], offsets['y'], offsets['z']],
                'itemsize': msg.point_step,
            }
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self.cloud_received += 1
        source_ns = _stamp_to_ns(msg.header.stamp)
        point_count = int(msg.width) * int(msg.height)
        self.input_points += point_count
        if point_count == 0 or msg.point_step <= 0:
            return
        if msg.row_step != msg.width * msg.point_step:
            self.get_logger().warn('Dropping PointCloud2 with unsupported row padding')
            return

        try:
            dtype = self._xyz_dtype(msg)
            points = np.frombuffer(msg.data, dtype=dtype, count=point_count).copy()
        except (TypeError, ValueError) as exc:
            self.get_logger().error('Cannot decode PointCloud2: %s' % exc)
            return

        xyz = np.column_stack((points['x'], points['y'], points['z'])).astype(np.float64, copy=False)
        valid = np.isfinite(xyz).all(axis=1) & (np.einsum('ij,ij->i', xyz, xyz) > 1.0e-10)

        if self.cloud_mode == 'deskewed':
            pose = self._nearest_odom(source_ns)
            if pose is None:
                self.cloud_dropped_no_pose += 1
                return
            _, translation, rotation, _ = pose
            # Unitree publishes the motion-deskewed XYZ values in odom. R maps
            # base -> odom, so row-vector inversion is (p_odom - t) @ R.
            xyz[valid] = (xyz[valid] - translation) @ rotation

        points = points[valid]
        xyz = xyz[valid].astype(np.float32)
        points['x'], points['y'], points['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        out = copy.deepcopy(msg)
        out.header.stamp = self._aligned_stamp(source_ns, 'cloud')
        out.header.frame_id = 'base_link'
        out.height = 1
        out.width = int(points.size)
        out.row_step = out.width * out.point_step
        out.data = points.tobytes()
        out.is_dense = True
        self.cloud_pub.publish(out)
        self.cloud_published += 1
        self.output_points += out.width

    def _report_stats(self) -> None:
        if self.cloud_received == 0:
            return
        keep = 100.0 * self.output_points / max(1, self.input_points)
        self.get_logger().info(
            'clouds rx/pub/drop=%d/%d/%d, valid points %.1f%%'
            % (self.cloud_received, self.cloud_published, self.cloud_dropped_no_pose, keep)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2LivoSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
