#!/bin/bash
# ==============================================================================
# Canonical 1-Click Localization & Interactive Goal Manager for Unitree Go2
# ==============================================================================
# Features:
#   1. Real-time (X, Y, Z, Yaw) localization HUD
#   2. Automatic CSV & TXT log saving to ~/.ros/localization_runs/latest/
#   3. Press [ENTER] anytime to record Goal #1, Goal #2, etc.
#   4. Press 'd' / 'del' to delete last goal (with confirmation)
#   5. 100% SSH & Headless compatible
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
RTABMAP_DB="/home/unitree/.ros/rtabmap.db"
GUI_MODE=false
GUI_ARG="rtabmap_viz:=false"

usage() {
    cat <<'USAGE_EOF'
Usage:
  ./run_localization.sh                     Start localization HUD & goal recorder (SSH/GUI auto-detect)
  ./run_localization.sh --start-goal <ID>   Seed localization starting at registered waypoint ID (e.g. 1)
  ./run_localization.sh --start-origin      Seed localization starting at map origin (0,0,0)
  ./run_localization.sh --initial-pose "..." Seed localization with custom pose ("x y z roll pitch yaw")
  ./run_localization.sh --auto-reloc        Auto-relocalize against recorded map keyframes
  ./run_localization.sh --gui               Force GUI with 3D rtabmap_viz visualizer
  ./run_localization.sh --headless          Force headless localization (SSH/tmux)
  ./run_localization.sh --help              Show this help
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

EXPLICIT_GUI=false
EXPLICIT_HEADLESS=false
START_AT_ORIGIN=true
INITIAL_POSE=""
START_GOAL=""
PASSTHROUGH_ARGS=()

# Parse Arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)
            EXPLICIT_GUI=true
            shift
            ;;
        --headless)
            EXPLICIT_HEADLESS=true
            shift
            ;;
        --start-goal)
            START_GOAL="$2"
            START_AT_ORIGIN=false
            PASSTHROUGH_ARGS+=("--start-goal" "$2")
            shift 2
            ;;
        --start-origin)
            START_AT_ORIGIN=true
            PASSTHROUGH_ARGS+=("--start-origin")
            shift
            ;;
        --initial-pose)
            INITIAL_POSE="$2"
            START_AT_ORIGIN=false
            PASSTHROUGH_ARGS+=("--initial-pose" "$2")
            shift 2
            ;;
        --auto-reloc)
            START_AT_ORIGIN=false
            PASSTHROUGH_ARGS+=("--auto-reloc")
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

if [ -n "$START_GOAL" ] && [ -z "$INITIAL_POSE" ]; then
    INITIAL_POSE=$(python3 -c "import sys; sys.path.insert(0, '$WORKSPACE_DIR/scratch'); import map_relocalizer, math; wps = {w['id']: w for w in map_relocalizer.load_registered_waypoints()}; w = wps.get($START_GOAL); sys.stdout.write(f\"{w['x_m']} {w['y_m']} {w['z_m']} 0 0 {math.radians(w['yaw_deg']):.4f}\" if w else '')" 2>/dev/null || true)
fi

if [ "$EXPLICIT_GUI" = true ]; then
    if find_display; then
        GUI_MODE=true
        GUI_ARG="rtabmap_viz:=true"
    else
        echo "⚠️ No X display found. Falling back to headless localization." >&2
        GUI_MODE=false
        GUI_ARG="rtabmap_viz:=false"
    fi
elif [ "$EXPLICIT_HEADLESS" = true ]; then
    GUI_MODE=false
    GUI_ARG="rtabmap_viz:=false"
else
    # Default for robot operation: headless over SSH unless DISPLAY is explicitly forwarded
    if [ -n "${DISPLAY:-}" ] && xdpyinfo >/dev/null 2>&1; then
        GUI_MODE=true
        GUI_ARG="rtabmap_viz:=true"
    else
        GUI_MODE=false
        GUI_ARG="rtabmap_viz:=false"
    fi
fi

if [ ! -f "$RTABMAP_DB" ]; then
    echo "❌ Error: Target map database not found: $RTABMAP_DB" >&2
    echo "   Please record a map first using ./run_mapping.sh" >&2
    exit 1
fi

echo "========================================================================"
echo " 🐕 [Unitree Go2] Planar 3DoF Real-Time Localization & Goal Manager"
echo " Map DB  : $RTABMAP_DB ($(du -h "$RTABMAP_DB" | cut -f1))"
echo " Mode    : Read-only localization (localization:=true)"
echo " Display : $([ "$GUI_MODE" = true ] && echo "GUI (rtabmap_viz active on $DISPLAY)" || echo "Headless (Console only over SSH)")"
echo "========================================================================"

PIDS=()
SESSION_START_TIME=$(date +%s)
SESSION_START_STR=$(date '+%Y-%m-%d %H:%M:%S')

cleanup() {
    local exit_status=$?
    trap - SIGINT SIGTERM EXIT
    local session_end_time=$(date +%s)
    local session_duration=$((session_end_time - SESSION_START_TIME))
    local session_end_str=$(date '+%Y-%m-%d %H:%M:%S')
    echo ""
    echo "🛑 Shutting down localization stack..."
    for pid in "${PIDS[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f go2_localization_and_goal_recorder.py 2>/dev/null || true
    pkill -f go2_front_camera_publisher.py 2>/dev/null || true
    pkill -f go2_livo_sensor_bridge.py 2>/dev/null || true
    pkill -f '[r]os2 launch rtabmap_launch go2_rtabmap\.launch\.py' 2>/dev/null || true
    pkill -x rtabmap 2>/dev/null || true
    pkill -x rtabmap_viz 2>/dev/null || true
    echo ""
    echo "========================================================================"
    echo "⏱️ [LOCALIZATION SESSION TIME LOG]"
    echo " • Session Start : $SESSION_START_STR"
    echo " • Session End   : $session_end_str"
    echo " • Total Elapsed : ${session_duration}s ($((session_duration / 60))m $((session_duration % 60))s)"
    echo " • Active Goals  : $WORKSPACE_DIR/config/navigation_goals.json"
    echo "========================================================================"
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
pkill -9 -f go2_localization_and_goal_recorder 2>/dev/null || true
pkill -9 -f go2_front_camera 2>/dev/null || true
pkill -9 -f go2_livo_sensor_bridge 2>/dev/null || true
pkill -9 -f '[r]os2 launch rtabmap_launch go2_rtabmap\.launch\.py' 2>/dev/null || true
pkill -9 -x rtabmap 2>/dev/null || true
pkill -9 -x rtabmap_viz 2>/dev/null || true
sleep 1

# 2. Start Front Camera Publisher
echo "📷 [1/4] Starting Front Camera Publisher (30fps)..."
python3 "$WORKSPACE_DIR/scratch/go2_front_camera_publisher.py" >/dev/null 2>&1 &
PIDS+=($!)
sleep 1

# 3. Start LIVO Sensor Bridge
echo "🛰️ [2/4] Starting Unitree LIO Bridge (/livo/*)..."
python3 "$WORKSPACE_DIR/scratch/go2_livo_sensor_bridge.py" \
    --ros-args -p cloud_mode:=deskewed -p imu_quaternion_order:=auto >/dev/null 2>&1 &
PIDS+=($!)
sleep 1

# 4. Launch RTAB-Map in Localization Mode (Headless background)
echo "🗺️ [3/4] Launching RTAB-Map Localization Engine (localization:=true)..."
INITIAL_POSE_ARGS=()
if [ -n "$INITIAL_POSE" ]; then
    INITIAL_POSE_ARGS=("initial_pose:=$INITIAL_POSE")
fi

ros2 launch rtabmap_launch go2_rtabmap.launch.py \
    localization:=true \
    $GUI_ARG \
    reg_force_3dof:=true \
    icp_force_4dof:=false \
    loop_closure_identity_guess:=false \
    proximity_by_space:=false \
    start_at_origin:="$START_AT_ORIGIN" \
    range_max:=25.0 \
    "${INITIAL_POSE_ARGS[@]}" >/home/unitree/.ros/rtabmap_launch.log 2>&1 &
RTABMAP_PID=$!
PIDS+=($RTABMAP_PID)
sleep 3

if ! kill -0 "$RTABMAP_PID" 2>/dev/null; then
    echo "❌ Error: RTAB-Map failed to launch! Output from /home/unitree/.ros/rtabmap_launch.log:" >&2
    tail -n 25 /home/unitree/.ros/rtabmap_launch.log >&2
    exit 1
fi

# 5. Start Unified Real-Time HUD & Interactive Goal Recorder (Front Console)
echo "🎯 [4/4] Starting Real-Time Localization HUD & Goal Manager..."
python3 "$WORKSPACE_DIR/scratch/go2_localization_and_goal_recorder.py" "${PASSTHROUGH_ARGS[@]}"
