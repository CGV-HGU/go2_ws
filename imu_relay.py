#!/usr/bin/env python3
"""
IMU Relay: Republishes /camera/imu -> /imu/data_raw
This is needed because imu_filter_madgwick uses message_filters::Subscriber which
ignores --ros-args -r remappings in ROS 2 Foxy. The node hardcodes the subscriber
to 'imu/data_raw' and the warning message shows '/imu/data_raw' but remapping
is ignored internally.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')

        # Use best_effort QoS to match camera driver publisher
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publish with RELIABLE for imu_filter_madgwick
        filter_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=100
        )

        self.sub = self.create_subscription(
            Imu,
            '/camera/imu',
            self.callback,
            camera_qos
        )
        self.pub = self.create_publisher(Imu, '/imu/data_raw', filter_qos)
        self.count = 0
        self.get_logger().info('IMU Relay started: /camera/imu -> /imu/data_raw')

    def callback(self, msg: Imu):
        relayed_msg = Imu()
        relayed_msg.header = msg.header
        relayed_msg.header.frame_id = 'camera_link'

        # RealSense optical frame (X right, Y down, Z forward) -> ROS body frame (X forward, Y left, Z up)
        relayed_msg.linear_acceleration.x = msg.linear_acceleration.z
        relayed_msg.linear_acceleration.y = -msg.linear_acceleration.x
        relayed_msg.linear_acceleration.z = -msg.linear_acceleration.y

        relayed_msg.angular_velocity.x = msg.angular_velocity.z
        relayed_msg.angular_velocity.y = -msg.angular_velocity.x
        relayed_msg.angular_velocity.z = -msg.angular_velocity.y

        self.pub.publish(relayed_msg)
        self.count += 1
        if self.count == 1:
            self.get_logger().info('🚀 [FIRST IMU RECEIVED] Converted & relayed 1st IMU message to camera_link body frame!')
        elif self.count % 100 == 0:
            self.get_logger().info(f'🔄 [RELAYING] Total IMU messages relayed: {self.count}')


def main():
    rclpy.init()
    node = ImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
