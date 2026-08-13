#!/usr/bin/env bash
# ==============================================================================
# ICRA 2026 Go2 Real-Robot Physical Trial ROS 2 Bag Logger Script
# ==============================================================================
# Usage: ./record_experiment.sh <scenario_name> <model_name> <trial_id>
# Example: ./record_experiment.sh Indoor_Corridor Ours_Async Trial1

SCENARIO=${1:-"Indoor_Corridor"}
MODEL=${2:-"Ours_Async"}
TRIAL=${3:-"Trial1"}

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="bag_${SCENARIO}_${MODEL}_${TRIAL}_${TIMESTAMP}"
OUTPUT_DIR="experiments_bags/${SCENARIO}"

mkdir -p "${OUTPUT_DIR}"

echo "========================================================================"
echo " 🎬 Recording Real-Robot Trial: ${BAG_NAME}"
echo " Scenario: ${SCENARIO} | Model: ${MODEL} | Trial: ${TRIAL}"
echo " Saving to: ${OUTPUT_DIR}/${BAG_NAME}"
echo " Press Ctrl+C when the trial is finished."
echo "========================================================================"

ros2 bag record -o "${OUTPUT_DIR}/${BAG_NAME}" \
    /rtabmap/odom \
    /cmd_vel \
    /camera/front/image_raw/compressed \
    /utlidar/cloud_deskewed \
    /tf \
    /tf_static
