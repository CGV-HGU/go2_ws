#!/bin/bash
# ==============================================================================
# 🗺️ 1-Click 3D Map Inspector (Open saved rtabmap.db in 3D Viewer)
# ==============================================================================
export DISPLAY="${DISPLAY:-:0}"
DB_PATH="${1:-$HOME/.ros/rtabmap.db}"

if [ ! -f "$DB_PATH" ]; then
    echo "❌ Error: Map database not found at $DB_PATH"
    exit 1
fi

echo "========================================================================"
echo " 🗺️ Opening RTAB-Map 3D Database Viewer ($DB_PATH)..."
echo "========================================================================"

rtabmap-databaseViewer "$DB_PATH"
