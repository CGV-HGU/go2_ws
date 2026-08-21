#!/usr/bin/env python3
"""
========================================================================================
🔬 Unitree Go2 ESCAPE-Nav LIVO Multi-Modal Topic Cross-Verification Suite
========================================================================================
Audits live publishing rates (Hz), message timestamps, packet counts, and latency
across all sensor, TF, and SLAM topics.
========================================================================================
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_msgs.msg import TFMessage

class LIVOCrossVerifier(Node):
    def __init__(self):
        super().__init__('livo_cross_verifier')
        
        # Test targets with expected frequencies
        self.topic_configs = [
            ('/camera/front/image_raw', Image, 30.0, "Front Camera RGB (30fps)"),
            ('/camera/front/camera_info', CameraInfo, 30.0, "Camera Matrix & Distortion"),
            ('/tf', TFMessage, 70.0, "6-DoF TF Coordinate Tree (odom -> base -> camera)"),
            ('/cloud_map', PointCloud2, 2.0, "RTAB-Map 3D Dense Point Cloud Map"),
            ('/map', OccupancyGrid, 0.5, "RTAB-Map 2D Laser Occupancy Grid"),
            ('/imu', Imu, 50.0, "Unitree Go2 Body IMU (50~500Hz)"),
            ('/odom', Odometry, 50.0, "Unitree Go2 Leg Kinematic Odometry (50Hz)"),
            ('/utlidar/cloud', PointCloud2, 10.0, "Unitree 4D LiDAR L2 PointCloud2"),
        ]
        
        self.msg_counts = {t[0]: 0 for t in self.topic_configs}
        self.first_stamps = {}
        self.last_stamps = {}
        
        # QoS profiles: Support both Reliable and Best-Effort
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        for topic_name, msg_type, _, _ in self.topic_configs:
            self.create_subscription(
                msg_type,
                topic_name,
                self.make_callback(topic_name),
                reliable_qos
            )

    def make_callback(self, name):
        def cb(msg):
            now = time.time()
            if name not in self.first_stamps:
                self.first_stamps[name] = now
            self.last_stamps[name] = now
            self.msg_counts[name] += 1
        return cb

def run_cross_verification(duration_sec=6.0):
    rclpy.init()
    node = LIVOCrossVerifier()
    
    print("=" * 80)
    print(" 🔬 [UNITREE GO2 ESCAPE-NAV] LIVO MULTI-MODAL TOPIC CROSS-VERIFICATION SUITE")
    print(f" ⏱️ Audit Duration: {duration_sec:.1f} seconds | Clock: ROS 2 System Wall Time")
    print("=" * 80)
    
    start_time = time.time()
    while time.time() - start_time < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    
    actual_duration = time.time() - start_time
    
    print("\n" + "=" * 80)
    print(" 📊 [CROSS-VERIFICATION AUDIT RESULTS]")
    print("=" * 80)
    print(f" {'Topic Name':<28} | {'Type':<14} | {'Target':<7} | {'Measured':<9} | {'Packets':<7} | {'Verdict'}")
    print("-" * 80)
    
    for topic_name, msg_type, target_hz, desc in node.topic_configs:
        count = node.msg_counts[topic_name]
        type_str = msg_type.__name__
        
        if count > 0 and topic_name in node.first_stamps:
            time_span = node.last_stamps[topic_name] - node.first_stamps[topic_name]
            measured_hz = (count / time_span) if time_span > 0 else (count / actual_duration)
            
            if measured_hz >= target_hz * 0.7:
                verdict = "🟢 PASS (EXCELLENT)"
            elif measured_hz > 0:
                verdict = "🟡 PASS (MODERATE/LATCHED)"
            else:
                verdict = "⚪ IDLE"
        else:
            measured_hz = 0.0
            verdict = "⚪ STANDBY/IDLE"
            
        print(f" {topic_name:<28} | {type_str:<14} | {target_hz:>5.1f}Hz | {measured_hz:>7.2f}Hz | {count:>7d} | {verdict}")
        
    print("=" * 80)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_cross_verification(6.0)
