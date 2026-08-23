#!/bin/bash
# ==============================================================================
# 🚀 Unitree Go2 1-Click Headless Autonomous Navigation (ESCAPE-Nav S2E)
# ==============================================================================
# - Runs 100% headless full autonomy stack (Host LIVO 50Hz + Docker S2E)
# - No monitor / No GUI overhead for maximum Jetson Orin NX performance
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
EXEC_SCRIPT="$DIR/scratch/bringup_all_escape_nav.sh"

echo "========================================================================"
echo " 🚀 Starting Headless ESCAPE-Nav Autonomous Navigation..."
echo " 👉 Press Ctrl+C at any time to trigger E-STOP and safely halt robot."
echo "========================================================================"

bash "$EXEC_SCRIPT" "$@"
