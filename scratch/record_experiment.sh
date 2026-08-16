#!/usr/bin/env bash
# ==============================================================================
# ICRA 2026 Go2 Real-Robot Physical 5-Set PointNav Logger Script
# ==============================================================================
# Usage: ./record_experiment.sh <set_name> <model_name> <trial_id>
# Example: ./record_experiment.sh Set1_Straight_5m Ours_Async Trial1
# Example: ./record_experiment.sh Set4_Deadlock_Corner S2E_Only Trial1

SET_NAME=${1:-"Set1_Straight_5m"}
MODEL=${2:-"Ours_Async"}
TRIAL=${3:-"Trial1"}

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="bag_${SET_NAME}_${MODEL}_${TRIAL}_${TIMESTAMP}"
OUTPUT_DIR="experiments_bags/${SET_NAME}"

mkdir -p "${OUTPUT_DIR}"

echo "========================================================================"
echo " 🎬 Recording Real-Robot PointNav Trial: ${BAG_NAME}"
echo " Set: ${SET_NAME} | Model: ${MODEL} | Trial: ${TRIAL}"
echo " Saving to: ${OUTPUT_DIR}/${BAG_NAME}"
echo " Press Ctrl+C when the robot reaches the goal."
echo "========================================================================"

# I/O 최적화: 대용량 raw pointcloud를 배제하고 실시간 오도메트리, 제어 속도, 압축 비전, TF만 선별 기록
ros2 bag record -o "${OUTPUT_DIR}/${BAG_NAME}" \
    /rtabmap/odom \
    /cmd_vel \
    /camera/front/image_raw/compressed \
    /tf \
    /tf_static
