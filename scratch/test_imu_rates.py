#!/usr/bin/env python3
import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class IMUTester(Node):
    def __init__(self):
        super().__init__('imu_tester')
        
        matching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.imu_count = 0
        self.sub_imu = self.create_subscription(Imu, '/camera/imu', self.imu_cb, matching_qos)
        self.get_logger().info("IMUTester listening for /camera/imu...")

    def imu_cb(self, msg):
        self.imu_count += 1
        if self.imu_count % 50 == 0 or self.imu_count < 10:
            self.get_logger().info(f"[STREAM] Received {self.imu_count} messages on /camera/imu")

def test_rates(accel_rate, gyro_rate):
    ros_base = "/opt/ros/foxy"
    dds_ws = "/home/unitree/cyclonedds_ws"
    ws_root = "/home/unitree/go2_ws"

    print(f"\n==================================================")
    print(f"Testing Accel: {accel_rate} Hz, Gyro: {gyro_rate} Hz")
    print(f"==================================================")

    # Reset camera
    print("1. Performing camera reset...")
    subprocess.run([f"{ros_base}/bin/rs-enumerate-devices", "-r"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(12)

    processes = []
    bash_prefix = f"source {ros_base}/setup.bash && source {dds_ws}/install/setup.bash && source {ws_root}/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://{ws_root}/cyclonedds.xml && export ROS_DOMAIN_ID=0 && export LD_PRELOAD=/usr/local/lib/librealsense2.so && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && "

    # Launch camera node with the test rates
    print("2. Launching camera node...")
    cam_cmd = bash_prefix + (
        f"ros2 launch realsense2_camera rs_launch.py "
        f"initial_reset:=false "
        f"align_depth:=true "
        f"enable_sync:=true "
        f"depth_module.profile:=640x480x15 "
        f"rgb_camera.profile:=640x480x15 "
        f"enable_accel:=true "
        f"enable_gyro:=true "
        f"unite_imu_method:=1 "
        f"accel_fps:={accel_rate} "
        f"gyro_fps:={gyro_rate}"
    )
    cam_proc = subprocess.Popen(["bash", "-c", cam_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    processes.append(("camera", cam_proc))

    time.sleep(8)

    print("3. Starting data collection...")
    rclpy.init()
    tester = IMUTester()
    
    start_time = time.time()
    while time.time() - start_time < 8.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        
    print(f"Result -> Total IMU Messages: {tester.imu_count}")
    
    tester.destroy_node()
    rclpy.shutdown()

    print("Terminating camera node...")
    for name, p in processes:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()

    # Capture logs to check for "force pause"
    cam_output = []
    for line in cam_proc.stdout:
        cam_output.append(line)
    
    has_force_pause = any("force pause" in line for line in cam_output)
    print(f"Result -> Got force pause warning: {has_force_pause}")
    if has_force_pause:
        print("Mismatched or unsupported rate warning printed.")

    return tester.imu_count > 0

def main():
    # Test 200 Hz Accel & 200 Hz Gyro
    success = test_rates(200, 200)
    if success:
        print("\nSUCCESS! Accel 200 Hz, Gyro 200 Hz worked!")
        return

    # If that fails, test 400 Hz Accel & 400 Hz Gyro
    success = test_rates(400, 400)
    if success:
        print("\nSUCCESS! Accel 400 Hz, Gyro 400 Hz worked!")
        return

    # Test 100 Hz Accel & 200 Hz Gyro but with unite_imu_method:=2 (linear interpolation)
    print("\nTesting unite_imu_method:=2 (Linear Interpolation) with 100 Hz Accel, 200 Hz Gyro")
    ros_base = "/opt/ros/foxy"
    dds_ws = "/home/unitree/cyclonedds_ws"
    ws_root = "/home/unitree/go2_ws"
    subprocess.run([f"{ros_base}/bin/rs-enumerate-devices", "-r"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(12)
    
    processes = []
    bash_prefix = f"source {ros_base}/setup.bash && source {dds_ws}/install/setup.bash && source {ws_root}/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://{ws_root}/cyclonedds.xml && export ROS_DOMAIN_ID=0 && export LD_PRELOAD=/usr/local/lib/librealsense2.so && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && "
    cam_cmd = bash_prefix + (
        f"ros2 launch realsense2_camera rs_launch.py "
        f"initial_reset:=false "
        f"align_depth:=true "
        f"enable_sync:=true "
        f"depth_module.profile:=640x480x15 "
        f"rgb_camera.profile:=640x480x15 "
        f"enable_accel:=true "
        f"enable_gyro:=true "
        f"unite_imu_method:=2"
    )
    cam_proc = subprocess.Popen(["bash", "-c", cam_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    processes.append(("camera", cam_proc))
    time.sleep(8)
    rclpy.init()
    tester = IMUTester()
    start_time = time.time()
    while time.time() - start_time < 8.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
    print(f"Result (Method 2) -> Total IMU Messages: {tester.imu_count}")
    tester.destroy_node()
    rclpy.shutdown()
    for name, p in processes:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()

if __name__ == '__main__':
    main()
