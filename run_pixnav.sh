#!/bin/bash
# ==============================================================================
# Canonical 1-Click Launcher for Baseline: Direct Goal / Pure PixNav
# ==============================================================================
# Usage:
#   ./run_pixnav.sh                     # Interactive selection of candidate goals [1-5]
#   ./run_pixnav.sh 1                   # Direct run to Goal #1
#   ./run_pixnav.sh 1 --start-origin    # Start at map origin (0,0,0) and run to Goal #1
#   ./run_pixnav.sh 2 --start-goal 1    # Start at Goal #1 and run to Goal #2
#   ./run_pixnav.sh 1 --auto-reloc      # Auto-relocalize against recorded map keyframes
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
RTABMAP_DB="/home/unitree/.ros/rtabmap.db"

GOAL_ARG=""
START_GOAL=""
START_AT_ORIGIN=true
INITIAL_POSE=""
PASSTHROUGH_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
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
        -*)
            echo "Error: unsupported option: $1" >&2
            exit 2
            ;;
        *)
            if [ -z "$GOAL_ARG" ]; then
                GOAL_ARG="$1"
            else
                PASSTHROUGH_ARGS+=("$1")
            fi
            shift
            ;;
    esac
done

if [ -z "$GOAL_ARG" ]; then
    GOAL_ARG="1"
fi

if [ -n "$START_GOAL" ] && [ -z "$INITIAL_POSE" ]; then
    INITIAL_POSE=$(python3 -c "import sys; sys.path.insert(0, '$WORKSPACE_DIR/scratch'); import map_relocalizer, math; wps = {w['id']: w for w in map_relocalizer.load_registered_waypoints()}; w = wps.get($START_GOAL); sys.stdout.write(f\"{w['x_m']} {w['y_m']} {w['z_m']} 0 0 {math.radians(w['yaw_deg']):.4f}\" if w else '')" 2>/dev/null || true)
fi

if [ ! -f "$RTABMAP_DB" ]; then
    echo "❌ Error: Map database not found: $RTABMAP_DB" >&2
    echo "   Please record a map first using ./run_mapping.sh" >&2
    exit 1
fi

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
    echo ""
    echo "========================================================================"
    echo "⏱️ [PIXNAV SESSION TIME LOG]"
    echo " • Session Start : $SESSION_START_STR"
    echo " • Session End   : $session_end_str"
    echo " • Total Elapsed : ${session_duration}s ($((session_duration / 60))m $((session_duration % 60))s)"
    echo "========================================================================"
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
INITIAL_POSE_ARGS=()
if [ -n "$INITIAL_POSE" ]; then
    INITIAL_POSE_ARGS=("initial_pose:=$INITIAL_POSE")
fi

ros2 launch rtabmap_launch go2_rtabmap.launch.py \
    localization:=true \
    rtabmap_viz:=false \
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

# 5. Launch Goal-Directed PixNav Controller
python3 "$WORKSPACE_DIR/scratch/go2_autonomous_navigator.py" --mode pixnav --goal "$GOAL_ARG" "${PASSTHROUGH_ARGS[@]}"
