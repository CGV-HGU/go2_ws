#!/usr/bin/env python3
import subprocess
import time
import os

def main():
    ros_base = "/opt/ros/foxy"
    dds_ws = "/home/unitree/cyclonedds_ws"
    ws_root = "/home/unitree/go2_ws"

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

    print("Waiting 12 seconds for camera to initialize...")
    time.sleep(12)

    print("\n--- Running ros2 topic echo /camera/imu ---")
    echo_cmd = bash_prefix + "timeout 5 ros2 topic echo /camera/imu"
    res = subprocess.run(["bash", "-c", echo_cmd], capture_output=True, text=True)
    
    print(f"Exit Code: {res.returncode}")
    if res.stdout:
        print("Stdout:\n", res.stdout)
    if res.stderr:
        print("Stderr:\n", res.stderr)
    print("--------------------------------------------------\n")

    print("Terminating camera node...")
    for name, p in processes:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()

if __name__ == '__main__':
    main()
