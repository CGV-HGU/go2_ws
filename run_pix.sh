#!/bin/bash
# ==============================================================================
# Canonical 1-Click Launcher for Baseline: Direct Goal / Pure PixNav
# ==============================================================================
# Usage:
#   ./run_pix.sh             # Interactive selection of candidate goals [1-5]
#   ./run_pix.sh 1           # Direct run to Goal #1
#   ./run_pix.sh 2           # Direct run to Goal #2
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
RTABMAP_DB="/home/unitree/.ros/rtabmap.db"
GOAL_ARG="${1:-}"

if [ ! -f "$RTABMAP_DB" ]; then
    echo "❌ Error: Map database not found: $RTABMAP_DB" >&2
    echo "   Please record a map first using ./run_map" >&2
    exit 1
fi

PIDS=()

cleanup() {
    local exit_status=$?
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "🛑 Shutting down PixNav Baseline Stack..."
    for pid in "${PIDS[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f go2_autonomous_navigator.py 2>/dev/null || true
    pkill -f go2_front_camera_publisher.py 2>/dev/null || true
    pkill -f go2_livo_sensor_bridge.py 2>/dev/null || true
    pkill -f '[r]os2 launch rtabmap_launch go2_rtabmap\.launch\.py' 2>/dev/null || true
    pkill -x rtabmap 2>/dev/null || true
    pkill -x rtabmap_viz 2>/dev/null || true
    echo "✅ Robot safely halted and stack closed. 🐕"
    exit "$exit_status"
}

trap cleanup SIGINT SIGTERM EXIT

# Environment Setup
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash 2>/dev/null || true
source /home/unitree/backup/legacy_workspaces/go2_analysis/go2_ws/install/setup.bash 2>/dev/null || true
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$WORKSPACE_DIR/cyclonedds.xml"
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/home/unitree/opencv_build/opencv/build/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}

# 1. Clean up background nodes
pkill -9 -f go2_autonomous_navigator 2>/dev/null || true
pkill -9 -f go2_front_camera 2>/dev/null || true
pkill -9 -f go2_livo_sensor_bridge 2>/dev/null || true
pkill -9 -f '[r]os2 launch rtabmap_launch go2_rtabmap\.launch\.py' 2>/dev/null || true
pkill -9 -x rtabmap 2>/dev/null || true
pkill -9 -x rtabmap_viz 2>/dev/null || true
sleep 1

# 2. Start Front Camera Publisher
echo "📷 [1/3] Starting Front Camera Publisher (30fps)..."
python3 "$WORKSPACE_DIR/scratch/go2_front_camera_publisher.py" >/dev/null 2>&1 &
PIDS+=($!)
sleep 1

# 3. Start LIVO Sensor Bridge
echo "🛰️ [2/3] Starting Unitree LIO Bridge (/livo/*)..."
python3 "$WORKSPACE_DIR/scratch/go2_livo_sensor_bridge.py" \
    --ros-args -p cloud_mode:=deskewed -p imu_quaternion_order:=auto >/dev/null 2>&1 &
PIDS+=($!)
sleep 1

# 4. Launch RTAB-Map in Localization Mode (Headless background)
echo "🗺️ [3/3] Launching RTAB-Map Localization Engine (localization:=true)..."
ros2 launch rtabmap_launch go2_rtabmap.launch.py \
    localization:=true \
    rtabmap_viz:=false \
    reg_force_3dof:=true \
    icp_force_4dof:=false \
    loop_closure_identity_guess:=true \
    proximity_by_space:=false >/dev/null 2>&1 &
PIDS+=($!)
sleep 3

# 5. Launch Goal-Directed PixNav Controller
if [ -n "$GOAL_ARG" ]; then
    exec python3 "$WORKSPACE_DIR/scratch/go2_autonomous_navigator.py" --mode pixnav --goal "$GOAL_ARG"
else
    exec python3 "$WORKSPACE_DIR/scratch/go2_autonomous_navigator.py" --mode pixnav
fi
