#!/bin/bash
# Compatibility wrapper for the Go2 built-in LiDAR/IMU RTAB-Map stack.
#
# This path must not launch the external unilidar_sdk2 node: the Go2 already
# publishes /utlidar/* over DDS, and a second publisher on those names creates
# mixed clocks, frames and point formats.
#
# Usage:
#   bash scratch/start_rtabmap_livo.sh           # existing-map localization
#   bash scratch/start_rtabmap_livo.sh mapping   # backup DB, then create new map
#   bash scratch/start_rtabmap_livo.sh mapping gui

set -e

LAUNCH_ARGS=()
if [ "${1:-localization}" = "mapping" ]; then
    LAUNCH_ARGS+=(--mapping)
fi
if [ "${2:-}" = "gui" ] || [ "${1:-}" = "gui" ]; then
    LAUNCH_ARGS+=(--gui)
fi

exec bash /home/unitree/go2_ws_antarctica/scratch/bringup_all_escape_nav.sh "${LAUNCH_ARGS[@]}"
