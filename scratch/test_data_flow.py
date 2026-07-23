#!/usr/bin/env python3
import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class FlowTester(Node):
    def __init__(self):
        super().__init__('flow_tester')
        
        # QoS profiles matching the publishers
        # /camera/imu is published with RELIABLE, Volatile (according to our diagnosis)
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # /imu/data_raw is published with RELIABLE, Volatile
        raw_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # /imu/data is published by madgwick (usually reliable)
        filter_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.cam_count = 0
        self.raw_count = 0
        self.filter_count = 0

        self.sub_cam = self.create_subscription(Imu, '/camera/imu', self.cam_cb, camera_qos)
        self.sub_raw = self.create_subscription(Imu, '/imu/data_raw', self.raw_cb, raw_qos)
        self.sub_filter = self.create_subscription(Imu, '/imu/data', self.filter_cb, filter_qos)
        
        self.get_logger().info("Subscriptions initialized. Listening for messages...")

    def cam_cb(self, msg):
        self.cam_count += 1
        if self.cam_count % 50 == 0:
            self.get_logger().info(f"[FLOW] Received {self.cam_count} messages on /camera/imu")

    def raw_cb(self, msg):
        self.raw_count += 1
        if self.raw_count % 50 == 0:
            self.get_logger().info(f"[FLOW] Received {self.raw_count} messages on /imu/data_raw")

    def filter_cb(self, msg):
        self.filter_count += 1
        if self.filter_count % 50 == 0:
            self.get_logger().info(f"[FLOW] Received {self.filter_count} messages on /imu/data")

def main():
    # Setup paths
    ros_base = "/opt/ros/foxy"
    dds_ws = "/home/unitree/cyclonedds_ws"
    ws_root = "/home/unitree/go2_ws"
    
    # Reset camera
    print("Resetting RealSense camera...")
    subprocess.run([f"{ros_base}/bin/rs-enumerate-devices", "-r"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("Waiting 12 seconds for USB controller to stabilize...")
    time.sleep(12)

    # Start processes
    processes = []
    bash_prefix = f"source {ros_base}/setup.bash && source {dds_ws}/install/setup.bash && source {ws_root}/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://{ws_root}/cyclonedds.xml && export ROS_DOMAIN_ID=0 && export LD_PRELOAD=/usr/local/lib/librealsense2.so && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && "

    print("1. Launching camera node...")
    cam_cmd = bash_prefix + (
        f"ros2 launch realsense2_camera rs_launch.py "
        f"initial_reset:=true "
        f"align_depth:=true "
        f"enable_sync:=true "
        f"depth_module.profile:=640x480x15 "
        f"rgb_camera.profile:=640x480x15 "
        f"enable_accel:=true "
        f"enable_gyro:=true "
        f"unite_imu_method:=1"
    )
    cam_proc = subprocess.Popen(["bash", "-c", cam_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    processes.append(("camera", cam_proc))

    print("Waiting 10 seconds for camera to initialize...")
    time.sleep(10)

    print("2. Launching IMU relay...")
    relay_cmd = bash_prefix + f"python3 {ws_root}/imu_relay.py"
    relay_proc = subprocess.Popen(["bash", "-c", relay_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    processes.append(("relay", relay_proc))
    time.sleep(3)

    print("3. Launching Madgwick filter...")
    filter_cmd = bash_prefix + (
        f"ros2 run imu_filter_madgwick imu_filter_madgwick_node "
        f"--ros-args "
        f"-p use_mag:=false "
        f"-p publish_tf:=false "
        f"-r /imu/data:=/imu/data"
    )
    filter_proc = subprocess.Popen(["bash", "-c", filter_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    processes.append(("filter", filter_proc))
    time.sleep(5)

    print("\n--- Starting Active Data Flow Test ---")
    rclpy.init()
    tester = FlowTester()
    
    # Spin for 10 seconds to collect data
    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        
    print(f"\n--- Data Flow Results ---")
    print(f"Total /camera/imu messages: {tester.cam_count}")
    print(f"Total /imu/data_raw messages: {tester.raw_count}")
    print(f"Total /imu/data messages: {tester.filter_count}")
    print(f"-------------------------\n")
    
    tester.destroy_node()
    rclpy.shutdown()

    print("Terminating processes...")
    for name, p in processes:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()

if __name__ == '__main__':
    main()
