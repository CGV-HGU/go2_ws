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

export DISPLAY="${DISPLAY:-:0}"

echo "========================================================================"
echo " 🖥️ Launching Unitree Go2 RTAB-Map 3D GUI Visualizer (DISPLAY=$DISPLAY)..."
echo "========================================================================"

bash "$EXEC_SCRIPT" --mapping --gui "$@"
