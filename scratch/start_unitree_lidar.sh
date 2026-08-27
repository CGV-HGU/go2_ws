#!/bin/bash
# ==============================================================================
# Unitree external L2 LiDAR SDK bringup (not the Go2 built-in DDS LiDAR)
# Uses separate topic names so it cannot collide with /utlidar/* publishers.
# ==============================================================================

set -e

source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source /home/unitree/go2_ws_antarctica/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/unitree/go2_ws_antarctica/cyclonedds.xml"
export ROS_DOMAIN_ID=0

# Ensure 192.168.1.2/24 alias exists for LiDAR UDP communication
echo admin | sudo -S ip addr add 192.168.1.2/24 dev eth0 2>/dev/null || true

echo "🚀 [EXTERNAL L2] Launching Unitree unilidar_sdk2 v2.x driver..."
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node \
    --ros-args \
    -p initialize_type:=2 \
    -p lidar_ip:="192.168.1.62" \
    -p local_ip:="192.168.1.2" \
    -p lidar_port:=6101 \
    -p local_port:=6201 \
    -p work_mode:=0 \
    -p use_system_timestamp:=true \
    -p cloud_scan_num:=18 \
    -p range_min:=0.0 \
    -p range_max:=50.0 \
    -p cloud_frame:="unilidar_lidar" \
    -p cloud_topic:="/external_l2/cloud" \
    -p imu_frame:="unilidar_imu" \
    -p imu_topic:="/external_l2/imu"
