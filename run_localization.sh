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

# Ensure directory exists and copy map if available in workspace
mkdir -p $(dirname "$MAP_PGM")
if [ -f "$WS_ROOT/rtabmap.pgm" ]; then
    echo "📋 Syncing map files from workspace..."
    cp "$WS_ROOT/rtabmap.pgm" "$MAP_PGM"
    # Also copy yaml if it exists in workspace
    if [ -f "$WS_ROOT/rtabmap.yaml" ]; then
        cp "$WS_ROOT/rtabmap.yaml" "$MAP_YAML"
    fi
fi

# If yaml is still missing but pgm exists, generate the metadata (required by Nav2)
if [ ! -f "$MAP_YAML" ] && [ -f "$MAP_PGM" ]; then
    echo "📝 Generating metadata for your map: $MAP_YAML"
    cat <<EOF > "$MAP_YAML"
image: rtabmap.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF
fi

echo "⚪️ [1/7] Launching Go2 Bringup (URDF + Driver)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch go2_bringup go2.launch.py; exec bash"

sleep 5

echo "🟡 [2/7] Resetting RealSense & Launching Camera + DepthToLaserScan..."
$ROS_BASE/bin/rs-enumerate-devices -r > /dev/null 2>&1
echo "⏳ Waiting 15 seconds for Jetson USB controller to stabilize..."
sleep 15

gnome-terminal --tab -- bash -c "export LD_PRELOAD=/usr/local/lib/librealsense2.so; $INIT_ENV \
ros2 launch realsense2_camera rs_launch.py \
    align_depth:=true \
    enable_sync:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=2 & \
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

echo "🟡 [6/7] Publishing Static TFs..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link & \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_gyro_optical_frame camera_imu_optical_frame; exec bash"

sleep 5

echo "🟡 [2.5/7] Launching IMU Relay (/camera/imu -> /imu/data_raw)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH; \
python3 $WS_ROOT/imu_relay.py; exec bash"

sleep 3

echo "🟡 [2.7/7] Launching IMU Filter (Madgwick)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -p use_mag:=false \
    -p publish_tf:=false \
    -p world_frame:=enu \
    -r /imu/data:=/rtabmap/imu \
    -r imu/data:=/rtabmap/imu; exec bash"

sleep 3


echo "🟢 [3/7] Launching RTAB-Map in Localization Mode (with VIO)..."
gnome-terminal --tab -- bash -c "$INIT_ENV \
ros2 launch rtabmap_launch rtabmap.launch.py \
    localization:=true \
    rtabmap_args:='--Mem/IncrementalMemory false --Vis/MinInliers 5 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true --Optimizer/GravitySigma 0.3' \
    frame_id:=base_link \
    odom_frame_id:=odom \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    wait_for_transform:=1.5 \
    database_path:=${RTABMAP_DB_PATH} \
    map_topic:=/rtabmap/map \
    qos:=1 \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu \
    rviz:=true; exec bash"

sleep 10

echo "🟠 [4/7] Launching Map Server & Localization Manager..."
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
    map_subscribe_transient_local:=true \
    params_file:=$PARAMS_FILE; exec bash"

sleep 5

# echo "🟣 [7/7] Launching RViz (Nav2 Optimized)..."
# gnome-terminal --tab -- bash -c "$INIT_ENV \
# ros2 launch nav2_bringup rviz_launch.py; exec bash"

echo "✅ [SUCCESS] Go2 Localization & Nav2 Stack fully booted."
echo "💡 TIP: In RViz, use '2D Nav Goal' to set your destination once the map appears."
