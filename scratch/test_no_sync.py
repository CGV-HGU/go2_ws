#!/usr/bin/env python3
import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class StreamTester(Node):
    def __init__(self):
        super().__init__('stream_tester')
        
        # We will check with the standard QoS (Reliable for color/imu, matching what we saw)
        matching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.color_count = 0
        self.depth_count = 0
        self.imu_count = 0

        self.sub_color = self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, matching_qos)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, matching_qos)
        self.sub_imu = self.create_subscription(Imu, '/camera/imu', self.imu_cb, matching_qos)
        self.get_logger().info("StreamTester listening for streams (no sync)...")

    def color_cb(self, msg):
        self.color_count += 1
        if self.color_count % 10 == 0 or self.color_count < 5:
            self.get_logger().info(f"[STREAM] Received {self.color_count} images on /camera/color/image_raw")

    def depth_cb(self, msg):
        self.depth_count += 1
        if self.depth_count % 10 == 0 or self.depth_count < 5:
            self.get_logger().info(f"[STREAM] Received {self.depth_count} images on /camera/depth/image_rect_raw")

    def imu_cb(self, msg):
        self.imu_count += 1
        if self.imu_count % 50 == 0 or self.imu_count < 10:
            self.get_logger().info(f"[STREAM] Received {self.imu_count} messages on /camera/imu")

def main():
    ros_base = "/opt/ros/foxy"
    dds_ws = "/home/unitree/cyclonedds_ws"
    ws_root = "/home/unitree/go2_ws"

    processes = []
    bash_prefix = f"source {ros_base}/setup.bash && source {dds_ws}/install/setup.bash && source {ws_root}/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://{ws_root}/cyclonedds.xml && export ROS_DOMAIN_ID=0 && export LD_PRELOAD=/usr/local/lib/librealsense2.so && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && "

    print("1. Launching camera node with enable_sync:=false, align_depth:=false...")
    cam_cmd = bash_prefix + (
        f"ros2 launch realsense2_camera rs_launch.py "
        f"initial_reset:=true "
        f"align_depth:=false "
        f"enable_sync:=false "
        f"depth_module.profile:=640x480x15 "
        f"rgb_camera.profile:=640x480x15 "
        f"enable_accel:=true "
        f"enable_gyro:=true "
        f"unite_imu_method:=1"
    )
    cam_proc = subprocess.Popen(["bash", "-c", cam_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    processes.append(("camera", cam_proc))

    print("Waiting 12 seconds for camera to initialize...")
    time.sleep(12)

    print("\n--- Starting Active Stream Test (No Sync) ---")
    rclpy.init()
    tester = StreamTester()
    
    # Spin for 10 seconds to collect data
    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        
    print(f"\n--- Stream Results ---")
    print(f"Total Color Images: {tester.color_count}")
    print(f"Total Depth Images: {tester.depth_count}")
    print(f"Total IMU Messages: {tester.imu_count}")
    print(f"----------------------\n")
    
    tester.destroy_node()
    rclpy.shutdown()

    print("Terminating camera node...")
    for name, p in processes:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()

if __name__ == '__main__':
    main()
