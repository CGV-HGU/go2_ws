#!/bin/bash
# ==============================================================================
# Canonical 1-Click Localization Launcher for Unitree Go2 Planar 3DoF SLAM
# ==============================================================================
# Localizes the robot in real-time against ~/.ros/rtabmap.db without modifying it.
# Automatically brings up front camera, Unitree LIO bridge, and RTAB-Map in localization mode.
# ==============================================================================

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
RTABMAP_DB="/home/unitree/.ros/rtabmap.db"
GUI_MODE=false
GUI_ARG="rtabmap_viz:=false"

usage() {
    cat <<'USAGE_EOF'
Usage:
  ./run_local.sh            Start planar 3DoF localization (auto-detects display)
  ./run_local.sh --gui      Force GUI with 3D rtabmap_viz visualizer
  ./run_local.sh --headless Force headless localization (SSH/tmux)
  ./run_local.sh --help     Show this help
USAGE_EOF
}

find_display() {
    if [ -n "${DISPLAY:-}" ] && xdpyinfo >/dev/null 2>&1; then
        return 0
    fi
    for display_candidate in :1 :0 :1001; do
        if DISPLAY="$display_candidate" xdpyinfo >/dev/null 2>&1; then
            export DISPLAY="$display_candidate"
            return 0
        fi
    done
    return 1
}

# Parse Arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)
            GUI_MODE=true
            GUI_ARG="rtabmap_viz:=true"
            shift
            ;;
        --headless)
            GUI_MODE=false
            GUI_ARG="rtabmap_viz:=false"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: unsupported option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ "$GUI_MODE" = false ] && [ "$GUI_ARG" = "rtabmap_viz:=false" ]; then
    if find_display; then
        GUI_MODE=true
        GUI_ARG="rtabmap_viz:=true"
    fi
fi

if [ ! -f "$RTABMAP_DB" ]; then
    echo "❌ Error: Target map database not found: $RTABMAP_DB" >&2
    echo "   Please record a map first using ./run_map.sh" >&2
    exit 1
fi

echo "========================================================================"
echo " 🐕 [Unitree Go2] Planar 3DoF Localization Mode"
echo " Map DB  : $RTABMAP_DB ($(du -h "$RTABMAP_DB" | cut -f1))"
echo " Mode    : Read-only localization (localization:=true)"
echo " Display : $([ "$GUI_MODE" = true ] && echo "GUI (rtabmap_viz active on $DISPLAY)" || echo "Headless (Console only)")"
echo "========================================================================"

PIDS=()

cleanup() {
    local exit_status=$?
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "🛑 Shutting down localization stack..."
    for pid in "${PIDS[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f go2_front_camera_publisher.py 2>/dev/null || true
    pkill -f go2_livo_sensor_bridge.py 2>/dev/null || true
    pkill -f '[r]os2 launch rtabmap_launch go2_rtabmap\.launch\.py' 2>/dev/null || true
    pkill -x rtabmap 2>/dev/null || true
    pkill -x rtabmap_viz 2>/dev/null || true
    echo "✅ Localization terminated safely. Bye! 🐕"
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
pkill -9 -f go2_front_camera 2>/dev/null || true
pkill -9 -f go2_livo_sensor_bridge 2>/dev/null || true
pkill -9 -f '[r]os2 launch rtabmap_launch go2_rtabmap\.launch\.py' 2>/dev/null || true
pkill -9 -x rtabmap 2>/dev/null || true
pkill -9 -x rtabmap_viz 2>/dev/null || true
sleep 1

# 2. Start Front Camera Publisher
echo "📷 [1/3] Starting Front Camera Publisher (30fps)..."
python3 "$WORKSPACE_DIR/scratch/go2_front_camera_publisher.py" &
PIDS+=($!)
sleep 1

# 3. Start LIVO Sensor Bridge
echo "🛰️ [2/3] Starting Unitree LIO Bridge (/livo/*)..."
python3 "$WORKSPACE_DIR/scratch/go2_livo_sensor_bridge.py" \
    --ros-args -p cloud_mode:=deskewed -p imu_quaternion_order:=auto &
PIDS+=($!)
sleep 1

# 4. Launch RTAB-Map in Localization Mode
echo "🗺️ [3/3] Launching RTAB-Map Localization (localization:=true)..."
ros2 launch rtabmap_launch go2_rtabmap.launch.py \
    localization:=true \
    $GUI_ARG \
    reg_force_3dof:=true \
    icp_force_4dof:=false \
    loop_closure_identity_guess:=true \
    proximity_by_space:=false &
PIDS+=($!)

echo ""
echo "🚀 [READY] Robot is now localizing against $RTABMAP_DB!"
echo "   • Pose Topic : /rtabmap/localization_pose"
echo "   • Frame TF   : map -> odom -> base_link"
echo "   • Press Ctrl+C to stop."
echo ""

wait
