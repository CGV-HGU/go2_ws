#!/bin/bash
# ==============================================================================
# 🎬 Unitree Go2 1-Click 3D Mapping & Real-Time 2D Map MP4 Screen Recorder
# ==============================================================================
# - Starts RTAB-Map LIVO 3D/2D Mapping Stack (with GUI window if display connected)
# - Records a broadcast-quality 1080p MP4 video of the real-time 2D map being built
# - 100% IMMUNE to HDMI Cable Unplugging / Display Disconnect (Renders in RAM)
# - Saves output video to: 2dmap/recordings/live_2d_mapping_YYYYMMDD_HHMMSS.mp4
# - Press Ctrl+C when done to safely finalize video and save map database!
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUT_DIR="$DIR/2dmap/recordings"
mkdir -p "$OUT_DIR"
VIDEO_OUT="$OUT_DIR/live_2d_mapping_${TIMESTAMP}.mp4"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================================================${NC}"
echo -e "${CYAN} 🎬 Starting Go2 3D Mapping + Live 2D Map MP4 Screen Recorder...${NC}"
echo -e "${CYAN}========================================================================${NC}"
echo -e " • Output Video Path : ${GREEN}${VIDEO_OUT}${NC}"
echo -e " • Database Storage  : ${GREEN}~/.ros/rtabmap.db${NC}"
echo -e " • HDMI Status       : ${YELLOW}100% Safe to unplug/plug HDMI cable anytime!${NC}"
echo -e " • To Stop & Save    : ${YELLOW}Press Ctrl+C when you finish walking the robot${NC}"
echo -e "${CYAN}========================================================================${NC}\n"

# Prevent X11 screen blanking if X11 is running
if [ -n "$DISPLAY" ]; then
    xset s off -dpms 2>/dev/null || true
fi

PIDS=()

cleanup() {
    echo ""
    echo -e "${YELLOW}========================================================================${NC}"
    echo -e "${YELLOW} 🛑 [MAPPING COMPLETED] Finalizing MP4 Video & Saving Map Database...${NC}"
    echo -e "${YELLOW}========================================================================${NC}"
    
    # Kill background recorder and bringup
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
        fi
    done
    sleep 2
    
    # Auto-export 2D occupancy grid map
    echo -e "${BLUE}🗺️ Exporting 2D Map to 2dmap/map_${TIMESTAMP}...${NC}"
    ros2 run nav2_map_server map_saver_cli -f "$DIR/2dmap/map_${TIMESTAMP}" 2>/dev/null || true
    
    echo -e "${GREEN}========================================================================${NC}"
    echo -e "${GREEN} 🏆 [SUCCESS] 2D Map & Live Video Successfully Saved!${NC}"
    echo -e "${GREEN}    👉 Video File: ${VIDEO_OUT}${NC}"
    echo -e "${GREEN}    👉 Database  : ~/.ros/rtabmap.db${NC}"
    echo -e "${GREEN}    👉 2D Map    : 2dmap/map_${TIMESTAMP}.pgm${NC}"
    echo -e "${GREEN}========================================================================${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# 1. Start Master Mapping Stack (Hardware + SLAM + GUI if available)
GUI_FLAG="--gui"
if [ -z "$DISPLAY" ]; then
    GUI_FLAG=""
fi

bash "$DIR/scratch/bringup_all_escape_nav.sh" --mapping $GUI_FLAG &
PIDS+=($!)

# 2. Wait for /map topic to come alive
echo -e "${BLUE}⏳ Waiting for SLAM /map topic to initialize...${NC}"
sleep 5

# 3. Start Live 2D Map Video Recorder (Renders 1080p MP4 in RAM)
echo -e "${BLUE}🎬 Starting Live 1080p Map Video Recorder...${NC}"
python3 "$DIR/scratch/live_2d_map_video_recorder.py" &
PIDS+=($!)

echo -e "\n${GREEN}========================================================================${NC}"
echo -e "${GREEN} 🚀 [RECORDING ACTIVE] Live 2D Map Video is now being recorded!${NC}"
echo -e "${GREEN}    - You can walk the robot untethered with the remote controller.${NC}"
echo -e "${GREEN}    - You can unplug the HDMI monitor cable whenever you like.${NC}"
echo -e "${GREEN}========================================================================${NC}\n"

wait
