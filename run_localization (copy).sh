#!/bin/bash

# ====================================================================================
# [D435i + RTAB-Map Localization + Nav2 - Go2 Final Optimized Version]
# Fully Synchronized with nav2_params.yaml and base_link frame
# ====================================================================================

# 1. Environment & Paths
WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"
DDS_WS="/home/unitree/cyclonedds_ws" 
PARAMS_FILE="$WS_ROOT/nav2_params.yaml"
# RTAB-Map Database & Map
RTABMAP_DB_PATH="/home/unitree/.ros/rtabmap.db"
MAP_YAML="/home/unitree/.ros/rtabmap.yaml"
MAP_PGM="/home/unitree/.ros/rtabmap.pgm"

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
    align_depth:=true \
    enable_sync:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    enable_accel:=false \
    enable_gyro:=false & \
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

echo "🟢 [3/7] Launching RTAB-Map in Localization Mode..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    localization:=true \
    rtabmap_args:='--Mem/IncrementalMemory false --Vis/MinInliers 5 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true' \
    frame_id:=base_link \
    odom_frame_id:=odom \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    wait_for_transform:=1.5 \
    database_path:=${RTABMAP_DB_PATH} \
    qos:=2 \
    rviz:=false; exec bash"

sleep 10

echo "🟠 [4/7] Launching Map Server (using .ros/rtabmap.yaml)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run nav2_map_server map_server --ros-args --params-file $PARAMS_FILE & \
sleep 2 && \
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    --params-file $PARAMS_FILE \
    -r __node:=lifecycle_manager_localization; exec bash"

sleep 5

echo "🔵 [5/7] Launching Navigation2 (Nav2 Stack)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    autostart:=true \
    params_file:=$PARAMS_FILE; exec bash"

sleep 5

echo "🟣 [7/7] Launching RViz (Nav2 Optimized)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch nav2_bringup rviz_launch.py; exec bash"

echo "✅ [SUCCESS] Go2 Localization & Nav2 Stack fully booted."
echo "💡 TIP: In RViz, use '2D Nav Goal' to set your destination once the map appears."
