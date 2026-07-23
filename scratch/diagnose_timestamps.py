#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, CameraInfo
from tf2_msgs.msg import TFMessage
import time

class Ros2Diagnostics(Node):
    def __init__(self):
        super().__init__('diagnose_timestamps_node')
        
        # QoS profiles
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.create_subscription(Image, '/camera/color/image_raw', self.cb_color, qos_reliable)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.cb_depth, qos_reliable)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, qos_reliable)
        self.create_subscription(TFMessage, '/tf_static', self.cb_tf_static, qos_reliable)
        
        self.get_logger().info("=========================================")
        self.get_logger().info("ROS 2 Timestamp Diagnostics Node Started")
        self.get_logger().info("=========================================")

    def get_time_diff(self, header):
        now_sec = time.time()
        msg_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
        return now_sec - msg_sec

    def cb_color(self, msg):
        diff = self.get_time_diff(msg.header)
        self.get_logger().info(f"[RGB Image] Frame ID: {msg.header.frame_id} | Stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} | Offset from System Clock: {diff:.6f}s")

    def cb_depth(self, msg):
        diff = self.get_time_diff(msg.header)
        self.get_logger().info(f"[Depth Image] Frame ID: {msg.header.frame_id} | Stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} | Offset from System Clock: {diff:.6f}s")

    def cb_imu(self, msg):
        diff = self.get_time_diff(msg.header)
        self.get_logger().info(f"[IMU Filtered] Frame ID: {msg.header.frame_id} | Stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} | Offset from System Clock: {diff:.6f}s")

    def cb_tf_static(self, msg):
        for transform in msg.transforms:
            self.get_logger().info(f"[TF Static] {transform.header.frame_id} -> {transform.child_frame_id} | Stamp: {transform.header.stamp.sec}.{transform.header.stamp.nanosec:09d}")

def main():
    rclpy.init()
    node = Ros2Diagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
