#!/bin/bash

# ====================================================================================
# [D435i + RTAB-Map SLAM - Clean Termination Script]
# ====================================================================================

echo "🛑 Stopping all SLAM, Camera, IMU Relay, and ROS 2 nodes..."

# Kill specific python relay script
pkill -9 -f imu_relay.py 2>/dev/null || true

# Kill RealSense camera node
pkill -9 -f realsense2_camera 2>/dev/null || true

# Kill IMU Filter
pkill -9 -f imu_filter_madgwick 2>/dev/null || true

# Kill RTAB-Map SLAM
pkill -9 -f rtabmap 2>/dev/null || true

# Kill static transform publishers
pkill -9 -f static_transform_publisher 2>/dev/null || true

# Kill any remaining ROS 2 launch processes
pkill -9 -f ros2 2>/dev/null || true

# Reset ROS 2 daemon to clear DDS cache
source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true
ros2 daemon stop >/dev/null 2>&1 || true

echo "✅ All SLAM and ROS 2 processes have been completely terminated."
