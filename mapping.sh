#!/bin/bash
# ==============================================================================
# 🗺️ Unitree Go2 ESCAPE-Nav 1-Click 3D Mapping Runner (mapping.sh)
# ==============================================================================
# - RTAB-Map LIVO 50Hz Offline 3D Mapping Launcher
# - Automatically executes master bringup in --mapping mode
# - Post-mapping: Run 'python3 scratch/inspect_rtabmap_db.py' to inspect DB
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

bash "$EXEC_SCRIPT" --mapping "$@"
