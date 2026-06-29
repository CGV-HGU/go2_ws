#!/bin/bash

# ====================================================================================
# [D435i Camera-Only Visual Odometry (VIO) Mapping - Standalone Mode]
# - Runs offline/standalone without launching go2_bringup
# ====================================================================================

WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"
DDS_WS="/home/unitree/cyclonedds_ws" 

# RealSense Resolution & FPS Configuration (USB 2.0 Fallback Optimization)
# For USB 3.0/3.2: Use "640x480x30"
# For USB 2.0 fallback: Use "640x480x15" or "424x240x15" (prevents bandwidth starvation)
CAMERA_PROFILE="640x480x15"

# Strict Isolation Environment
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
    depth_module.profile:=$CAMERA_PROFILE \
    rgb_camera.profile:=$CAMERA_PROFILE \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=1; exec bash"

sleep 5

echo "🟡 [1.5/3] Launching IMU Filter (Madgwick)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -p use_mag:=false \
    -p publish_tf:=false \
    -p world_frame:=enu \
    -p remove_gravity_vector:=true \
    -r /imu/data_raw:=/camera/imu \
    -r /imu/data:=/imu/data; exec bash"

sleep 3

echo "🟢 [2/3] Launching RTAB-Map in Visual Odometry (VIO) Mapping Mode..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start --Vis/MinInliers 5 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true' \
    frame_id:=base_link \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    odom_approx_sync:=true \
    approx_sync_max_interval:=0.1 \
    wait_for_transform:=1.5 \
    subscribe_imu:=true \
    imu_topic:=/imu/data \
    qos:=2 \
    rviz:=true; exec bash"

echo "🟡 [3/3] Publishing Static TF (base_link -> camera_link)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link; exec bash"

echo "✅ [SUCCESS] SLAM Booted in Camera-Only VO Mode. (Static TF enabled)"
