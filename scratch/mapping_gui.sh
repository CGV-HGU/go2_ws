#!/bin/bash
# ==============================================================================
# Unitree Go2 LIO + RGB RTAB-Map real-time 3D GUI mapping
# ==============================================================================
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
if [ -z "${DISPLAY:-}" ]; then
    for DISPLAY_CANDIDATE in :1 :0 :1001; do
        if DISPLAY="$DISPLAY_CANDIDATE" xdpyinfo >/dev/null 2>&1; then
            export DISPLAY="$DISPLAY_CANDIDATE"
            break
        fi
    done
fi
if [ -z "${DISPLAY:-}" ]; then
    echo "Error: no accessible X display found. Run this from the Jetson desktop terminal."
    exit 1
fi
exec bash "$DIR/bringup_all_escape_nav.sh" --mapping --gui
