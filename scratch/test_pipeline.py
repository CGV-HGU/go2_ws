#!/usr/bin/env python3
import subprocess
import time
import os
import sys

# Define environment variables based on INIT_ENV in run_map.sh
env = os.environ.copy()
# Clear existing ROS paths to prevent pollution
for key in ['AMENT_PREFIX_PATH', 'ROS_PREFIX_PATH', 'ROS_DISTRO', 'PYTHONPATH', 'LD_LIBRARY_PATH']:
    if key in env:
        del env[key]

# Setup new environment
ros_base = "/opt/ros/foxy"
dds_ws = "/home/unitree/cyclonedds_ws"
ws_root = "/home/unitree/go2_ws"

# Source ROS 2 base and workspaces by running a shell snippet and parsing output, or we can use a wrapper shell command.
# To keep it simple, we can run bash command strings directly.

# Reset camera
print("Resetting RealSense camera...")
subprocess.run([f"{ros_base}/bin/rs-enumerate-devices", "-r"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print("Waiting 12 seconds for USB controller to stabilize...")
time.sleep(12)

# Start processes
processes = []

# Build bash launch command prefix
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

# Give camera some time to reset and start up
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

print("\n--- Running diagnosis ---")
diag_cmd = bash_prefix + f"python3 {ws_root}/scratch/diagnose_topics.py"
subprocess.run(["bash", "-c", diag_cmd])
print("-------------------------\n")

print("Terminating processes...")
for name, p in processes:
    print(f"Stopping {name}...")
    p.terminate()
    try:
        p.wait(timeout=3)
    except subprocess.TimeoutExpired:
        print(f"Force-killing {name}...")
        p.kill()

# Print stdout logs from the processes to check for errors
print("\n--- Camera logs (last 30 lines) ---")
cam_output = []
for line in cam_proc.stdout:
    cam_output.append(line)
print("".join(cam_output[-30:]))

print("\n--- Relay logs ---")
relay_output = []
for line in relay_proc.stdout:
    relay_output.append(line)
print("".join(relay_output))

print("\n--- Filter logs ---")
filter_output = []
for line in filter_proc.stdout:
    filter_output.append(line)
print("".join(filter_output))
