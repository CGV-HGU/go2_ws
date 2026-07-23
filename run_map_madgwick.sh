#!/bin/bash

# ====================================================================================
# [D435i + RTAB-Map SLAM - Go2 Tuned Madgwick Version]
# ====================================================================================

WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"
DDS_WS="/home/unitree/cyclonedds_ws" 

# Strict Isolation Environment: Sourcing Base -> CycloneDDS -> New Workspace
INIT_ENV="unset AMENT_PREFIX_PATH ROS_PREFIX_PATH ROS_DISTRO PYTHONPATH LD_LIBRARY_PATH; \
source $ROS_BASE/setup.bash; \
source $DDS_WS/install/setup.bash; \
source $WS_ROOT/install/setup.bash; \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
export CYCLONEDDS_URI=file://$WS_ROOT/cyclonedds.xml; \
export ROS_DOMAIN_ID=0; \
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;"

echo "🟡 [1/4] Resetting RealSense & Launching Camera..."
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
    depth_module.profile:=424x240x30 \
    rgb_camera.profile:=424x240x30 \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=2 \
    publish_tf:=true \
    global_time_enabled:=false \
    hold_back_imu_for_frames:=true; exec bash"

sleep 10

echo "🟡 [2/4] Launching IMU Filter (Madgwick - Tuned for Go2)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -p use_mag:=false \
    -p publish_tf:=false \
    -p gain:=0.02 \
    -p zeta:=0.001 \
    -p do_bias_estimation:=true \
    -r /imu/data_raw:=/camera/imu \
    -r /imu/data:=/imu/data; exec bash"

sleep 3

echo "🟢 [3/4] Launching RTAB-Map (Optimized Sync + VIO)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start --Vis/MinInliers 5 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true --Optimizer/GravitySigma 0.3' \
    frame_id:=camera_link \
    visual_odometry:=true \
    odom_topic:=/odom \
    odom_info_topic:=/odom_info \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    odom_approx_sync:=true \
    approx_sync_max_interval:=1.0 \
    wait_for_transform:=1.5 \
    qos:=1 \
    wait_imu_to_init:=true \
    imu_topic:=/imu/data \
    rviz:=true; exec bash"

echo "🟡 [4/4] Publishing Static TF..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link & \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_gyro_optical_frame camera_imu_optical_frame; exec bash"

echo "✅ [SUCCESS] SLAM Booted with Go2-Tuned Madgwick Filter."
