#!/bin/bash
# ==============================================================================
# 🐕 Unitree Go2 1-Click Headless 3D Mapping (No Monitor / No HDMI Needed)
# ==============================================================================
# - Runs without X/HDMI and is suitable for a persistent SSH/tmux session
# - Saves real-time 3D LIVO map directly into ~/.ros/rtabmap.db
# - Logs accepted/rejected RTAB-Map loop events under
#   ~/.ros/rtabmap_loop_logs/loop_events_*.{jsonl,log}
# - Does not start rosbag, Docker/VLM, host command bridge or autonomous motion
# - Press Ctrl+C when done so RTAB-Map and the logger close cleanly
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
EXEC_SCRIPT="$DIR/scratch/bringup_all_escape_nav.sh"

echo "========================================================================"
echo " 🗺️ Starting Headless 3D LIVO Mapping (Saving to ~/.ros/rtabmap.db)..."
echo " 🧭 Loop logs: ~/.ros/rtabmap_loop_logs/loop_events_*.{jsonl,log}"
echo " 👉 Keep this shell alive (tmux recommended) and press Ctrl+C when done."
echo "========================================================================"

bash "$EXEC_SCRIPT" --mapping "$@"
