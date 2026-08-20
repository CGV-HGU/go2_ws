#!/bin/bash
# ==============================================================================
# 🚀 Unitree Go2 ESCAPE-Nav 1-Click RTAB-Map LIVO All-in-One Master Bringup
# ==============================================================================
# Launches:
# 1. Go2 Front Ultra-Wide Camera & CameraInfo Publisher (/camera/front/image_raw @ 30fps)
# 2. Unitree 4D L1/L2 LiDAR Driver Node (/utlidar/cloud @ 15Hz)
# 3. Native Sensor Node (/imu @ 50Hz, /odom @ 50Hz)
# 4. RTAB-Map LIVO Odometry & SLAM (50Hz /rtabmap/odom)
#
# Usage:
#   bash scratch/start_rtabmap_livo.sh          # Pure Odometry Mode (Default for S2E)
#   bash scratch/start_rtabmap_livo.sh mapping  # 3D Mapping Mode (Saves ~/.ros/rtabmap.db)
# ==============================================================================

set -e

MODE=${1:-"odom"}
if [ "$MODE" == "mapping" ]; then
    LOC_PARAM="localization:=false"
    echo "🗺️  [MODE] 3D Mapping Mode Activated (localization:=false)"
else
    LOC_PARAM="localization:=true"
    echo "⚡ [MODE] Pure Odometry Mode Activated (localization:=true)"
fi

# 1. Environment & DDS Setup
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source /home/unitree/go2_ws_antarctica/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/unitree/go2_ws_antarctica/cyclonedds.xml"
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/home/unitree/opencv_build/opencv/build/lib:/usr/local/lib:$LD_LIBRARY_PATH

# 2. Network Interface & Multicast Setup
echo "🌐 Configuring Network & Multicast Interfaces..."
echo admin | sudo -S ip addr add 192.168.1.2/24 dev eth0 2>/dev/null || true
echo admin | sudo -S ip route add 230.0.0.0/8 dev eth0 2>/dev/null || true

# 3. Clean existing sensor nodes
pkill -f go2_front_camera_publisher.py 2>/dev/null || true
pkill -f unitree_lidar_ros2_node 2>/dev/null || true
pkill -f go2_native_sensor_node.py 2>/dev/null || true

# Cleanup trap on Exit
cleanup() {
    echo ""
    echo "🛑 [SHUTDOWN] Stopping all LIVO sensor and driver nodes..."
    kill $CAM_PID $LIDAR_PID $IMU_PID 2>/dev/null || true
    pkill -f rtabmap 2>/dev/null || true
    echo "✅ All LIVO nodes terminated safely."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "========================================================================"
echo " 📷 [1/3] Launching Front Camera & CameraInfo Publisher (30 fps)..."
echo "========================================================================"
python3 /home/unitree/go2_ws_antarctica/scratch/go2_front_camera_publisher.py &
CAM_PID=$!
sleep 1

echo "========================================================================"
echo " 📡 [2/3] Launching Unitree 4D LiDAR Driver Node (/utlidar/cloud @ 15Hz)..."
echo "========================================================================"
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node \
    --ros-args \
    -p initialize_type:=2 \
    -p lidar_ip:="192.168.1.62" \
    -p local_ip:="192.168.1.2" \
    -p lidar_port:=6101 \
    -p local_port:=6201 \
    -p cloud_frame:="unilidar_lidar" \
    -p cloud_topic:="/utlidar/cloud" \
    -p imu_frame:="unilidar_imu" \
    -p imu_topic:="/utlidar/imu" &
LIDAR_PID=$!
sleep 1

echo "========================================================================"
echo " 🧭 [3/3] Launching Native Sensor Node (/imu @ 50Hz, /odom @ 50Hz)..."
echo "========================================================================"
python3 /home/unitree/go2_ws_antarctica/scratch/go2_native_sensor_node.py &
IMU_PID=$!
sleep 2

echo "========================================================================"
echo " 🚀 [LIVO MASTER] Launching RTAB-Map LIVO 50Hz Odometry ($LOC_PARAM)..."
echo "========================================================================"
ros2 launch rtabmap_launch go2_rtabmap.launch.py $LOC_PARAM scan_cloud_topic:=/utlidar/cloud
