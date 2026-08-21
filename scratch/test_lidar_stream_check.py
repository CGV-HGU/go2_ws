#!/usr/bin/env python3
"""
========================================================================================
📡 Unitree Go2 4D LiDAR L2 (UTLiDAR) Live Stream Diagnostic & Activation Checker
========================================================================================
Checks real-time reception of Unitree 4D LiDAR L2 PointCloud2 packets on CycloneDDS.
========================================================================================
"""

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

class LidarStreamChecker(Node):
    def __init__(self):
        super().__init__('lidar_stream_checker')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.count_utlidar = 0
        self.count_rt_utlidar = 0
        self.count_unilidar = 0
        self.last_points = 0
        
        self.create_subscription(PointCloud2, '/utlidar/cloud', self.cb_utlidar, qos)
        self.create_subscription(PointCloud2, 'rt/utlidar/cloud', self.cb_rt_utlidar, qos)
        self.create_subscription(PointCloud2, '/unilidar/cloud', self.cb_unilidar, qos)
        
        self.timer = self.create_timer(1.0, self.report)
        print("🔍 [LIDAR CHECKER] Listening for Unitree 4D LiDAR L2 PointCloud2 on eth0...")

    def cb_utlidar(self, msg):
        self.count_utlidar += 1
        self.last_points = msg.width * msg.height

    def cb_rt_utlidar(self, msg):
        self.count_rt_utlidar += 1
        self.last_points = msg.width * msg.height

    def cb_unilidar(self, msg):
        self.count_unilidar += 1
        self.last_points = msg.width * msg.height

    def report(self):
        total = self.count_utlidar + self.count_rt_utlidar + self.count_unilidar
        if total > 0:
            print(f"🟢 [LIVE] 4D LiDAR L2 Streaming Active! Rate: {total} Hz | Points/frame: {self.last_points}")
        else:
            print("⚪ [STANDBY] 4D LiDAR L2 is 0 Hz (Lidar streaming switch in Unitree App is OFF).")
        self.count_utlidar = 0
        self.count_rt_utlidar = 0
        self.count_unilidar = 0

def main():
    rclpy.init()
    node = LidarStreamChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
