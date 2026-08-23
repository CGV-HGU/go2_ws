#!/bin/bash
# ==============================================================================
# 🐕 Unitree Go2 1-Click Headless 3D Mapping (No Monitor / No HDMI Needed)
# ==============================================================================
# - Runs 100% headless in background / SSH
# - Saves real-time 3D LIVO map directly into ~/.ros/rtabmap.db
# - Safe to walk the robot untethered with remote controller
# - Press Ctrl+C when done to safely save map database
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
EXEC_SCRIPT="$DIR/scratch/bringup_all_escape_nav.sh"

echo "========================================================================"
echo " 🗺️ Starting Headless 3D LIVO Mapping (Saving to ~/.ros/rtabmap.db)..."
echo " 👉 Walk the robot untethered with controller. Press Ctrl+C when done."
echo "========================================================================"

bash "$EXEC_SCRIPT" --mapping "$@"
