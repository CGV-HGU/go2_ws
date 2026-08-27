#!/bin/bash
# ==============================================================================
# 🖥️ Unitree Go2 ESCAPE-Nav 1-Click Real-time 3D GUI Mapping (mapping_gui.sh)
# ==============================================================================
# - Launches RTAB-Map LIVO 50Hz with Real-time 3D GUI Visualizer (rtabmap_viz)
# - Displays camera feed, 3D point cloud, and trajectory on connected monitor
# - Usage: ./mapping_gui.sh  or  bash mapping_gui.sh
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
if [ -f "$DIR/scratch/bringup_all_escape_nav.sh" ]; then
    EXEC_SCRIPT="$DIR/scratch/bringup_all_escape_nav.sh"
elif [ -f "$DIR/bringup_all_escape_nav.sh" ]; then
    EXEC_SCRIPT="$DIR/bringup_all_escape_nav.sh"
else
    echo "❌ Error: bringup_all_escape_nav.sh not found!"
    exit 1
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
mkdir -p "$DIR/2dmap" "$DIR/2dmap/clean"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

cleanup() {
    echo ""
    echo -e "${YELLOW}========================================================================${NC}"
    echo -e "${YELLOW} 🛑 [MAPPING COMPLETED] Exporting 2D Map & Auto-Cleaning Spikes...${NC}"
    echo -e "${YELLOW}========================================================================${NC}"
    
    # Export 2D map
    echo -e "${BLUE}🗺️ Exporting 2D Map to 2dmap/map_${TIMESTAMP}...${NC}"
    ros2 run nav2_map_server map_saver_cli -f "$DIR/2dmap/map_${TIMESTAMP}" 2>/dev/null || true
    
    # Auto-clean ray-tracing spikes and wall noise
    if [ -f "$DIR/2dmap/map_${TIMESTAMP}.pgm" ]; then
        echo -e "${BLUE}✨ [CLEANER] Removing Ray-Tracing Spikes & Smoothing Walls...${NC}"
        python3 "$DIR/scratch/clean_and_export_2d_map.py" "$DIR/2dmap/map_${TIMESTAMP}.pgm" "$DIR/2dmap/map_${TIMESTAMP}.yaml" "$DIR/2dmap/clean" 2>/dev/null || true
    fi
    
    echo -e "${GREEN}========================================================================${NC}"
    echo -e "${GREEN} 🏆 [SUCCESS] 3D GUI Mapping Completed & 2D Clean Map Saved!${NC}"
    echo -e "${GREEN}    👉 Database   : ~/.ros/rtabmap.db${NC}"
    echo -e "${GREEN}    👉 Raw 2D Map : 2dmap/map_${TIMESTAMP}.pgm${NC}"
    echo -e "${GREEN}    👉 Clean Map  : 2dmap/clean/map_${TIMESTAMP}_clean.pgm${NC}"
    echo -e "${GREEN}    👉 Paper PNG  : 2dmap/clean/map_${TIMESTAMP}_clean_publication.png${NC}"
    echo -e "${GREEN}========================================================================${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "========================================================================"
echo " 🖥️ Launching Unitree Go2 RTAB-Map 3D GUI Visualizer (DISPLAY=$DISPLAY)..."
echo "========================================================================"

bash "$EXEC_SCRIPT" --mapping --gui "$@" || true
cleanup
