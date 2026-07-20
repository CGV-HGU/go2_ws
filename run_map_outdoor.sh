#!/bin/bash

# ====================================================================================
# [Go2 Onboard 4D LiDAR L1 + IMU Odometry Mapping Mode]
# - Uses the robot's stable leg/lidar-inertial odometry (/odom)
# - Bypasses the camera's visual odometry entirely (highly stable under USB 2.0)
# ====================================================================================

WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"
DDS_WS="/home/unitree/cyclonedds_ws" 

# RealSense Resolution & FPS Configuration (USB 2.0 Fallback Optimization)
# For USB 3.0/3.2: Use "640x480x30"
# For USB 2.0 fallback: Use "640x480x15" or "424x240x15" (prevents bandwidth starvation)
CAMERA_PROFILE="424x240x30"

# Strict Isolation Environment: Sourcing Base -> CycloneDDS -> New Workspace
# This prevents pollution from other workspaces and ensures binary compatibility
# Dynamic DDS Interface Selection
if ip addr | grep -q '192.168.123.'; then
    DDS_CONFIG="export CYCLONEDDS_URI=file://$WS_ROOT/cyclonedds.xml;"
    echo "🔗 Robot network (192.168.123.xx) detected. Applying CycloneDDS profile."
else
    DDS_CONFIG=""
    echo "🔌 Robot network NOT detected. Running CycloneDDS in local default mode."
fi

INIT_ENV="unset AMENT_PREFIX_PATH ROS_PREFIX_PATH ROS_DISTRO PYTHONPATH LD_LIBRARY_PATH; \
source $ROS_BASE/setup.bash; \
source $DDS_WS/install/setup.bash; \
source $WS_ROOT/install/setup.bash; \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
$DDS_CONFIG \
export ROS_DOMAIN_ID=0; \
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;"

echo "⚪️ [1/3] Launching Go2 Bringup (URDF + Driver)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch go2_bringup go2.launch.py; exec bash"
sleep 5

echo "🟡 [2/3] Resetting RealSense & Launching Camera..."
/usr/local/bin/rs-enumerate-devices -r > /dev/null 2>&1
echo "⏳ Waiting 25 seconds for Jetson USB controller to stabilize..."
sleep 25

gnome-terminal --tab -- bash -c "export LD_PRELOAD=/usr/local/lib/librealsense2.so; $INIT_ENV \
ros2 launch realsense2_camera rs_launch.py \
    initial_reset:=false \
    align_depth.enable:=true \
    enable_color:=true \
    enable_sync:=false \
    enable_sync.enable:=false \
    depth_module.profile:=$CAMERA_PROFILE \
    rgb_camera.profile:=$CAMERA_PROFILE \
    enable_accel:=false \
    enable_gyro:=false \
    publish_tf:=true; exec bash"
sleep 5

echo "🟢 [3/3] Launching RTAB-Map in LiDAR-Odom Mapping Mode..."
# Note: visual_odometry:=false disables camera VO, and odom_frame_id:=odom reads the robot's /odom topic.
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start --Vis/MinInliers 5 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true' \
    frame_id:=base_link \
    odom_frame_id:=odom \
    visual_odometry:=false \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    wait_for_transform:=1.5 \
    qos:=2 \
    rviz:=true; exec bash"

echo "✅ [SUCCESS] SLAM Booted in LiDAR-Odom Mode. (URDF TF used, static TF disabled)"
