#!/bin/bash
# ==============================================================================
# 🚀 Unitree Go2 ESCAPE-Nav Full Sensor Bringup Script
# Launches:
# 1. Go2 Built-in Ultra-Wide RGB Front Camera (/camera/front/image_raw @ 30fps)
# 2. Go2 Robot Driver + L2 LiDAR + 50Hz Odom + 500Hz IMU (/odom, /imu, /pointcloud)
# ==============================================================================

source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
source /home/unitree/go2_ws_new/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/unitree/go2_ws_antarctica/cyclonedds.xml"
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/home/unitree/opencv_build/opencv/build/lib:/usr/local/lib:$LD_LIBRARY_PATH

# 1. 멀티캐스트 라우팅 보장
echo admin | sudo -S ip route add 230.0.0.0/8 dev eth0 2>/dev/null || true

# 2. 이전 잔여 노드 정리
pkill -f go2_front_camera_publisher.py 2>/dev/null || true

echo "========================================================================"
echo " 📷 [1/2] Launching Go2 Built-in Front Camera Publisher (30 fps)..."
echo "========================================================================"
python3 /home/unitree/go2_ws_antarctica/scratch/go2_front_camera_publisher.py &
CAM_PID=$!

echo "========================================================================"
echo " 🤖 [2/2] Launching Go2 Robot Driver & LiDAR (50Hz Odom / 500Hz IMU)..."
echo "========================================================================"
ros2 launch go2_bringup go2.launch.py lidar:=True

# 종료 시 카메라 노드도 함께 종료
kill $CAM_PID 2>/dev/null || true
