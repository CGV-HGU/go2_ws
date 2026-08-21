#!/bin/bash
# ==============================================================================
# 🐕 Unitree Go2 Safe Lab Stepping Trot Verification Runner (run_test.sh)
# ==============================================================================
# - Pre-flight refresh of CycloneDDS participants
# - Directly executes safe active stepping verification sequence
# - Real-time physical displacement telemetry & safety stop on Ctrl+C
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

# 1. Pre-flight Cleanup & Refresh
ros2 daemon stop >/dev/null 2>&1 || true
pkill -f go2_driver 2>/dev/null || true

# 2. Environment Setup
source /opt/ros/foxy/setup.bash 2>/dev/null || true
source /home/unitree/cyclonedds_ws/install/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$DIR/cyclonedds.xml"
export ROS_DOMAIN_ID=0

# 3. Execute Stepping Test
python3 scratch/test_lab_micro_motion.py "$@"

