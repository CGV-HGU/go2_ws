#!/usr/bin/env python3
# ==============================================================================
# Best-Effort QoS Compatible Topic Hz Meter for Unitree Go2
# Solves ROS 2 Foxy 'ros2 topic hz' QoS incompatibility with Sensor Data
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu, Image
import time
import sys

class TopicHzMeter(Node):
    def __init__(self, topic_name, msg_type):
        super().__init__('topic_hz_meter')
        self.topic_name = topic_name
        self.timestamps = []
        
        # SensorDataQoS (Best Effort, Volatile, Depth 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.create_subscription(msg_type, topic_name, self.callback, qos)
        print(f"📡 [Hz Meter] Subscribing to '{topic_name}' with Best-Effort QoS...")

    def callback(self, msg):
        now = time.time()
        self.timestamps.append(now)
        if len(self.timestamps) > 100:
            self.timestamps.pop(0)
            
        if len(self.timestamps) >= 2:
            dt = self.timestamps[-1] - self.timestamps[-2]
            window = self.timestamps[-1] - self.timestamps[0]
            hz = (len(self.timestamps) - 1) / window if window > 0 else 0.0
            print(f"average rate: {hz:6.3f} Hz | dt: {dt:6.4f}s | count: {len(self.timestamps)}")

def main():
    rclpy.init()
    topic = sys.argv[1] if len(sys.argv) > 1 else '/utlidar/cloud'
    
    msg_type = PointCloud2
    if 'imu' in topic.lower():
        msg_type = Imu
    elif 'image' in topic.lower() or 'camera' in topic.lower():
        msg_type = Image
        
    meter = TopicHzMeter(topic, msg_type)
    try:
        rclpy.spin(meter)
    except KeyboardInterrupt:
        pass
    finally:
        meter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
