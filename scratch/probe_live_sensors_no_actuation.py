#!/usr/bin/env python3
"""Read-only live sensor probe for the Go2 built-in DDS streams.

This node creates subscriptions only. It never creates a command publisher,
SDK client, service client, or motor/control endpoint.
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2


class LiveSensorProbe(Node):
    def __init__(self) -> None:
        super().__init__('probe_live_sensors_no_actuation')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.counts = {'cloud': 0, 'imu': 0, 'odom': 0, 'camera': 0}
        self.cloud_meta = None
        self.imu_meta = None
        self.odom_meta = None
        self.camera_meta = None

        self.create_subscription(
            PointCloud2, '/utlidar/cloud_deskewed', self._cloud, qos
        )
        self.create_subscription(Imu, '/utlidar/imu', self._imu, qos)
        self.create_subscription(
            Odometry, '/utlidar/robot_odom', self._odom, qos
        )
        self.create_subscription(Image, '/camera/front/image_raw', self._camera, qos)

    def _cloud(self, msg: PointCloud2) -> None:
        self.counts['cloud'] += 1
        self.cloud_meta = (
            msg.header.frame_id,
            int(msg.width) * int(msg.height),
            int(msg.point_step),
        )

    def _imu(self, msg: Imu) -> None:
        self.counts['imu'] += 1
        a = msg.linear_acceleration
        accel_norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self.imu_meta = (msg.header.frame_id, accel_norm)

    def _odom(self, msg: Odometry) -> None:
        self.counts['odom'] += 1
        p = msg.pose.pose.position
        self.odom_meta = (
            msg.header.frame_id,
            msg.child_frame_id,
            p.x,
            p.y,
            p.z,
        )

    def _camera(self, msg: Image) -> None:
        self.counts['camera'] += 1
        self.camera_meta = (
            msg.header.frame_id,
            int(msg.width),
            int(msg.height),
            msg.encoding,
            len(msg.data),
        )

def main() -> int:
    duration = 6.0
    rclpy.init()
    node = LiveSensorProbe()
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        elapsed = time.monotonic() - started
        print('LIVE_SENSOR_PROBE_NO_ACTUATION')
        for name in ('cloud', 'imu', 'odom', 'camera'):
            count = node.counts[name]
            print(f'{name}_count={count} {name}_hz={count / elapsed:.2f}')
        print(f'cloud_meta={node.cloud_meta}')
        print(f'imu_meta={node.imu_meta}')
        print(f'odom_meta={node.odom_meta}')
        print(f'camera_meta={node.camera_meta}')
        passed = all(count > 0 for count in node.counts.values())
        print('PASS_ALL_RAW_LIVO_STREAMS_LIVE' if passed else 'FAIL_ONE_OR_MORE_RAW_LIVO_STREAMS')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
