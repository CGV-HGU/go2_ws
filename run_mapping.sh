#!/bin/bash
# ==============================================================================
# Canonical 1-Click Mapping Launcher for Unitree Go2 Planar 3DoF SLAM
# ==============================================================================
# Works 100% over SSH (Headless) and on Desktop (GUI).
# Auto-detects display; falls back to clean console mapping when running over SSH.
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
GUI_MODE=false
GUI_ARG="rtabmap_viz:=false"

usage() {
    cat <<'USAGE_EOF'
Usage:
  ./run_mapping.sh            Start planar 3DoF mapping (auto-detects SSH/GUI)
  ./run_mapping.sh --headless Force headless mapping (SSH/tmux)
  ./run_mapping.sh --gui      Force GUI with 3D rtabmap_viz visualizer
  ./run_mapping.sh --view [DB] View a safe temporary copy of a saved DB
  ./run_mapping.sh --help     Show this help
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
            if find_display; then
                GUI_MODE=true
                GUI_ARG="rtabmap_viz:=true"
            else
                echo "⚠️ No X display found. Falling back to headless mapping over SSH." >&2
                GUI_MODE=false
                GUI_ARG="rtabmap_viz:=false"
            fi
            shift
            ;;
        --headless)
            GUI_MODE=false
            GUI_ARG="rtabmap_viz:=false"
            shift
            ;;
        --view)
            db_path="${2:-/home/unitree/.ros/rtabmap.db}"
            if [ ! -f "$db_path" ]; then
                echo "Error: map database not found: $db_path" >&2
                exit 1
            fi
            if ! find_display; then
                echo "Error: Cannot open 3D viewer over headless SSH without X display." >&2
                exit 1
            fi
            viewer_dir="$(mktemp -d /tmp/rtabmap_view.XXXXXX)"
            viewer_db="$viewer_dir/$(basename "$db_path")"
            cp -aL "$db_path" "$viewer_db"
            cleanup_view() { rm -rf -- "$viewer_dir"; }
            trap cleanup_view EXIT INT TERM
            echo "Opening temporary viewer copy: $viewer_db"
            rtabmap-databaseViewer "$viewer_db" || true
            exit 0
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

# Backup old database safely before starting new mapping
RUN_ID="$(date +%Y%m%d_%H%M%S)_planar3dof_$([ "$GUI_MODE" = true ] && echo "gui" || echo "headless")"
RUN_DIR="/home/unitree/.ros/rtabmap_runs/$RUN_ID"
mkdir -p "$RUN_DIR" "/home/unitree/.ros/rtabmap_backups"

if [ -f "/home/unitree/.ros/rtabmap.db" ]; then
    backup_file="/home/unitree/.ros/rtabmap_backups/rtabmap_$(date +%Y%m%d_%H%M%S).db"
    cp -f "/home/unitree/.ros/rtabmap.db" "$backup_file"
    rm -f "/home/unitree/.ros/rtabmap.db"
fi

echo "========================================================================"
echo " 🐕 [Unitree Go2] Planar 3DoF SLAM Mapping"
echo " Run ID  : $RUN_ID"
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
    echo "🛑 Stopping mapping stack and saving database..."
    for pid in "${PIDS[@]:-}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f rtabmap_loop_logger.py 2>/dev/null || true
    pkill -f go2_front_camera_publisher.py 2>/dev/null || true
    pkill -f go2_livo_sensor_bridge.py 2>/dev/null || true
    pkill -f '[r]os2 launch rtabmap_launch go2_rtabmap\.launch\.py' 2>/dev/null || true
    pkill -x rtabmap 2>/dev/null || true
    pkill -x rtabmap_viz 2>/dev/null || true

    sleep 2
    if [ -f "/home/unitree/.ros/rtabmap.db" ]; then
        cp -f "/home/unitree/.ros/rtabmap.db" "$RUN_DIR/rtabmap.db"
        echo "💾 Saved Map DB: $RUN_DIR/rtabmap.db ($(du -h "$RUN_DIR/rtabmap.db" | cut -f1))"
        echo "🗺️ Auto-extracting 2D Golden Map..."
        python3 "$WORKSPACE_DIR/scratch/extract_final_golden_map.py" "/home/unitree/.ros/rtabmap.db" "$WORKSPACE_DIR/2dmap" || true
    fi
    echo ""
    echo "========================================================================"
    echo "⏱️ [MAPPING SESSION TIME LOG]"
    echo " • Session Start : $SESSION_START_STR"
    echo " • Session End   : $session_end_str"
    echo " • Total Elapsed : ${session_duration}s ($((session_duration / 60))m $((session_duration % 60))s)"
    echo " • Saved Map Run : $RUN_DIR"
    echo "========================================================================"
    echo "✅ Mapping stack terminated safely. 🗺️"
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
pkill -9 -f rtabmap_loop_logger 2>/dev/null || true
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

# 4. Start RTAB-Map SLAM Engine (Mapping mode)
echo "🗺️ [3/4] Launching RTAB-Map SLAM Engine (localization:=false)..."
ros2 launch rtabmap_launch go2_rtabmap.launch.py \
    localization:=false \
    $GUI_ARG \
    reg_force_3dof:=true \
    icp_force_4dof:=false \
    loop_closure_identity_guess:=false \
    proximity_by_space:=false >/dev/null 2>&1 &
PIDS+=($!)
sleep 3

# 5. Start Live Loop-Event Logger (Front Console)
echo "🔍 [4/4] Starting Real-Time Loop Closure Logger..."
python3 "$WORKSPACE_DIR/scratch/rtabmap_loop_logger.py"
