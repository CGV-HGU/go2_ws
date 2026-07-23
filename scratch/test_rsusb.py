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
        
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.cam_count = 0
        self.sub_cam = self.create_subscription(Imu, '/camera/imu', self.cam_cb, camera_qos)
        self.get_logger().info("FlowTester initialized. Listening for /camera/imu...")

    def cam_cb(self, msg):
        self.cam_count += 1
        if self.cam_count % 50 == 0 or self.cam_count < 10:
            self.get_logger().info(f"[FLOW] Received {self.cam_count} messages on /camera/imu")

def run_sudo(cmd_list):
    password = "admin"
    full_cmd = ["sudo", "-S"] + cmd_list
    proc = subprocess.Popen(full_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = proc.communicate(input=password + "\n")
    return proc.returncode

def main():
    ros_base = "/opt/ros/foxy"
    dds_ws = "/home/unitree/cyclonedds_ws"
    ws_root = "/home/unitree/go2_ws"

    print("1. Unbinding usbhid from interface 2-2:1.5...")
    run_sudo(["sh", "-c", "echo '2-2:1.5' > /sys/bus/usb/drivers/usbhid/unbind"])
    time.sleep(2)

    # Start processes (WITHOUT CYCLONEDDS_URI to test local flow)
    processes = []
    bash_prefix = f"source {ros_base}/setup.bash && source {dds_ws}/install/setup.bash && source {ws_root}/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=0 && export LD_PRELOAD=/usr/local/lib/librealsense2.so && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && "

    print("2. Launching camera node with initial_reset:=false, unite_imu_method:=1...")
    cam_cmd = bash_prefix + (
        f"ros2 launch realsense2_camera rs_launch.py "
        f"initial_reset:=false "
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

    print("\n--- Starting Active Data Flow Test ---")
    rclpy.init()
    tester = FlowTester()
    
    # Spin for 10 seconds to collect data
    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        
    print(f"\n--- Data Flow Results ---")
    print(f"Total /camera/imu messages: {tester.cam_count}")
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

    # Print camera node output
    print("\n--- Camera node logs (last 30 lines) ---")
    cam_output = []
    for line in cam_proc.stdout:
        cam_output.append(line)
    print("".join(cam_output[-30:]))

if __name__ == '__main__':
    main()
