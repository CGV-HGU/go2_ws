#!/usr/bin/env python3
"""
Unitree Go2 Real-Time LiDAR & IMU Topic Frequency Benchmarker
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState, SportModeState

class TopicHzAuditor(Node):
    def __init__(self):
        super().__init__('topic_hz_auditor')
        
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.counts = {
            'lowstate (Go2 MCU LowState @ BestEffort)': 0,
            'sportmodestate (Go2 MCU SportState @ BestEffort)': 0,
            'utlidar_cloud (Go2 4D LiDAR Raw Cloud @ BestEffort)': 0,
            'pointcloud (Go2 Driver LiDAR @ Reliable)': 0,
            'imu (Standard Body IMU sensor_msgs/Imu @ Reliable)': 0,
            'odom (Kinematic Odometry nav_msgs/Odometry @ SensorData)': 0,
        }
        
        self.create_subscription(LowState, '/lowstate', lambda m: self.increment('lowstate (Go2 MCU LowState @ BestEffort)'), sensor_qos)
        self.create_subscription(SportModeState, '/sportmodestate', lambda m: self.increment('sportmodestate (Go2 MCU SportState @ BestEffort)'), sensor_qos)
        self.create_subscription(PointCloud2, '/utlidar/cloud', lambda m: self.increment('utlidar_cloud (Go2 4D LiDAR Raw Cloud @ BestEffort)'), sensor_qos)
        self.create_subscription(PointCloud2, '/pointcloud', lambda m: self.increment('pointcloud (Go2 Driver LiDAR @ Reliable)'), reliable_qos)
        self.create_subscription(Imu, '/imu', lambda m: self.increment('imu (Standard Body IMU sensor_msgs/Imu @ Reliable)'), reliable_qos)
        self.create_subscription(Odometry, '/odom', lambda m: self.increment('odom (Kinematic Odometry nav_msgs/Odometry @ SensorData)'), sensor_qos)
        
    def increment(self, name):
        self.counts[name] += 1

def main():
    rclpy.init()
    node = TopicHzAuditor()
    print("=" * 80)
    print(" 📡 [Hz Benchmark] Listening to LiDAR & IMU ROS 2 topics for 5.0 seconds...")
    print("=" * 80)
    
    start_t = time.time()
    while time.time() - start_t < 5.0:
        rclpy.spin_once(node, timeout_sec=0.01)
        
    elapsed = time.time() - start_t
    print("\n" + "=" * 80)
    print(f" 📊 [Hz Benchmark Results] Measured over {elapsed:.2f} seconds:")
    print("=" * 80)
    for name, count in node.counts.items():
        hz = count / elapsed
        status = "🟢 LIVE" if hz > 0 else "⚪ OFF/0Hz"
        print(f"  • {name:<65} : {hz:6.1f} Hz ({count:4d} pkts) [{status}]")
    print("=" * 80)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
