#!/bin/bash

# ====================================================================================
# [D435i + RTAB-Map SLAM - Final Optimized & Isolated Version]
# ====================================================================================

WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"
DDS_WS="/home/unitree/cyclonedds_ws" 

# Strict Isolation Environment: Sourcing Base -> CycloneDDS -> New Workspace
# This prevents pollution from other workspaces and ensures binary compatibility
INIT_ENV="unset AMENT_PREFIX_PATH ROS_PREFIX_PATH ROS_DISTRO PYTHONPATH LD_LIBRARY_PATH; \
source $ROS_BASE/setup.bash; \
source $DDS_WS/install/setup.bash; \
source $WS_ROOT/install/setup.bash; \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
export CYCLONEDDS_URI=file://$WS_ROOT/cyclonedds.xml; \
export ROS_DOMAIN_ID=0; \
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;"

echo "🟡 [1/3] Resetting RealSense & Launching Camera..."
$ROS_BASE/bin/rs-enumerate-devices -r > /dev/null 2>&1
sleep 2

gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch realsense2_camera rs_launch.py \
    align_depth:=true \
    enable_sync:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    enable_accel:=false \
    enable_gyro:=false; exec bash"

sleep 5

echo "🟢 [2/3] Launching RTAB-Map (Optimized Sync)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start --Vis/MinInliers 5 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true' \
    frame_id:=camera_link \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    odom_approx_sync:=true \
    approx_sync_max_interval:=0.1 \
    wait_for_transform:=1.5 \
    qos:=2 \
    rviz:=true; exec bash"

echo "🟡 [3/3] Publishing Static TF (base_link -> camera_link)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link; exec bash"

echo "✅ [SUCCESS] SLAM Booted with full environment shielding."
