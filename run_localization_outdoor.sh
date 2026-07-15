#!/bin/bash

# ====================================================================================
# [Go2 Onboard 4D LiDAR L1 + IMU Odometry Localization & Nav2 Mode]
# - Uses the robot's stable leg/lidar-inertial odometry (/odom)
# - Bypasses the camera's visual odometry entirely (highly stable under USB 2.0)
# ====================================================================================

WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"
DDS_WS="/home/unitree/cyclonedds_ws" 

# Verify RTAB-Map DB file exists
RTABMAP_DB_PATH="/home/unitree/.ros/rtabmap.db"
if [ ! -f "$RTABMAP_DB_PATH" ]; then
    echo "❌ [ERROR] RTABMAP DB file not found at $RTABMAP_DB_PATH!"
    echo "Please build the map first using run_map_lidar.sh before running localization."
    exit 1
fi

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

echo "⚪️ [1/7] Launching Go2 Bringup (URDF + Driver)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch go2_bringup go2.launch.py; exec bash"
sleep 5

echo "🟡 [2/7] Resetting RealSense & Launching Camera + DepthToLaserScan..."
$ROS_BASE/bin/rs-enumerate-devices -r > /dev/null 2>&1
sleep 2

gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch realsense2_camera rs_launch.py \
    initial_reset:=true \
    align_depth:=true \
    enable_sync:=true \
    depth_module.profile:=$CAMERA_PROFILE \
    rgb_camera.profile:=$CAMERA_PROFILE \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=1 & \
sleep 5 && \
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
    --ros-args \
    -r depth:=/camera/aligned_depth_to_color/image_raw \
    -r depth_camera_info:=/camera/aligned_depth_to_color/camera_info \
    -r scan:=/scan \
    -p output_frame:=camera_link \
    -p scan_height:=10 \
    -p range_min:=0.3 \
    -p range_max:=5.0; exec bash"
sleep 5

echo "🟢 [3/7] Launching RTAB-Map in LiDAR-Odom Localization Mode..."
# Sets visual_odometry:=false to disable camera VO, and odom_frame_id:=odom to read /odom topic.
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true' \
    localization:=true \
    frame_id:=base_link \
    odom_frame_id:=odom \
    visual_odometry:=false \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    wait_for_transform:=1.5 \
    qos:=2 \
    rviz:=false; exec bash"
sleep 5

echo "🟢 [4/7] Launching Map Server & Lifecyle Manager..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/unitree/.ros/rtabmap.yaml & \
sleep 2 && \
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p node_names:=[map_server]; exec bash"
sleep 3

echo "🟢 [5/7] Launching Navigation2 (DWB/MPPI Local Planner)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch go2_nav navigation2.launch.py \
    use_sim_time:=false \
    params_file:=$WS_ROOT/nav2_params.yaml; exec bash"
sleep 3

echo "🔵 [6/7] Launching RViz Visualization..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run rviz2 rviz2 -d $WS_ROOT/src/go2_robot/go2_rviz/rviz/nav2.rviz; exec bash"

echo "✅ [SUCCESS] LiDAR-Odom Localization & Nav2 Stack Started."
