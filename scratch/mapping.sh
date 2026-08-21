#!/bin/bash
# ==============================================================================
# 🗺️ Unitree Go2 ESCAPE-Nav 1-Click 3D Mapping Runner (scratch/mapping.sh)
# ==============================================================================
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
bash "$DIR/bringup_all_escape_nav.sh" --mapping "$@"
