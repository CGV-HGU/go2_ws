#!/bin/bash

# ====================================================================================
# [D435i + RTAB-Map SLAM - Final Optimized Version]
# ====================================================================================

WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"
DDS_WS="/home/unitree/cyclonedds_ws" 

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
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH; \
export PYTHONUNBUFFERED=1;"

echo "⚪️ [1/6] Launching Go2 Bringup (URDF + Driver)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch go2_bringup go2.launch.py; exec bash"

sleep 3

echo "🟡 [2/6] Launching Camera Node..."
gnome-terminal --tab -- bash -c "export LD_PRELOAD=/usr/local/lib/librealsense2.so; $INIT_ENV \
ros2 launch realsense2_camera rs_launch.py \
    initial_reset:=false \
    align_depth.enable:=true \
    enable_color:=true \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_sync:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=2 \
    accel_qos:=SYSTEM_DEFAULT \
    gyro_qos:=SYSTEM_DEFAULT \
    publish_tf:=true \
    global_time_enabled:=false \
    hold_back_imu_for_frames:=false; exec bash"

sleep 10

echo "🟡 [3/6] Launching IMU Relay..."
gnome-terminal --tab -- bash -c "$INIT_ENV python3 $WS_ROOT/imu_relay.py; exec bash"

sleep 2

echo "🟡 [4/6] Launching IMU Filter (Madgwick)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -p gain:=0.02 \
    -p zeta:=0.0 \
    -p use_mag:=false \
    -p publish_tf:=false \
    -p world_frame:=\"enu\" \
    -p orientation_stddev:=0.01 \
    -r /imu/data:=/imu/data; exec bash"

sleep 6

echo "🟢 [5/6] Launching RTAB-Map (Optimized Sync + VIO)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start --Vis/MinInliers 10 --Rtabmap/MinVisInliers 10 --Rtabmap/DetectionRate 2.0 --Grid/RangeMin 0.3 --Grid/RangeMax 3.0 --Grid/MaxGroundHeight 0.1 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true --Optimizer/GravitySigma 0.3' \
    odom_args:='--ros-args -p Odom/Strategy:=0' \
    frame_id:=base_link \
    visual_odometry:=true \
    odom_topic:=/odom \
    odom_frame_id:=odom \
    odom_info_topic:=/odom_info \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    odom_approx_sync:=true \
    approx_sync_max_interval:=1.0 \
    wait_for_transform:=1.5 \
    qos:=1 \
    qos_image:=1 \
    qos_camera_info:=1 \
    qos_imu:=1 \
    qos_odom:=1 \
    wait_imu_to_init:=true \
    imu_topic:=/imu/data \
    rtabmap_viz:=true \
    rviz:=true; exec bash"

echo "🟡 [6/6] Publishing Static TF..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link camera_imu_optical_frame & \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_gyro_optical_frame camera_imu_optical_frame; exec bash"

echo "✅ [SUCCESS] SLAM Booted with full environment shielding (VIO enabled)."
