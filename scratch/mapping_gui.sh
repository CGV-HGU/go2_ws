#!/bin/bash
# ==============================================================================
# 🖥️ Unitree Go2 ESCAPE-Nav 1-Click Real-time 3D GUI Mapping (scratch/mapping_gui.sh)
# ==============================================================================
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export DISPLAY="${DISPLAY:-:0}"
bash "$DIR/bringup_all_escape_nav.sh" --mapping --gui "$@"
