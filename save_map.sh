#!/bin/bash

# ====================================================================================
# [RTAB-Map 2D Map Exporter (Foxy)]
# - Saves the current 2D occupancy grid map to a yaml/pgm file in ~/.ros/
# ====================================================================================

WS_ROOT="/home/unitree/go2_ws"
ROS_BASE="/opt/ros/foxy"

INIT_ENV="source $ROS_BASE/setup.bash; source $WS_ROOT/install/setup.bash;"

echo "💾 Saving current RTAB-Map 2D occupancy grid map to /home/unitree/.ros/rtabmap..."

# Execute the map saver CLI from nav2_map_server
eval "$INIT_ENV ros2 run nav2_map_server map_saver_cli -f /home/unitree/.ros/rtabmap"

echo "✅ [SUCCESS] Map saved successfully! Files created:"
ls -la /home/unitree/.ros/rtabmap.*
