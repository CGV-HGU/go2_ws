#!/usr/bin/env bash
# ==============================================================================
# ICRA 2026 Go2 Real-Robot Physical Indoor PointNav Logger Script
# ==============================================================================
# Usage: ./record_experiment.sh <scenario_name> <model_name> <trial_id>
# Example: ./record_experiment.sh Straight_Corridor Ours_Async Trial1
# Example: ./record_experiment.sh Corner_90Deg Ours_Async Trial1
# Example: ./record_experiment.sh TJunction_15m Ours_Async Trial1
# Example: ./record_experiment.sh Dynamic_Obstacle Ours_Async Trial1

SCENARIO=${1:-"Straight_Corridor"}
MODEL=${2:-"Ours_Async"}
TRIAL=${3:-"Trial1"}

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="bag_${SCENARIO}_${MODEL}_${TRIAL}_${TIMESTAMP}"
OUTPUT_DIR="experiments_bags/${SCENARIO}"

mkdir -p "${OUTPUT_DIR}"

echo "========================================================================"
echo " 🎬 Recording Real-Robot PointNav Trial: ${BAG_NAME}"
echo " Scenario: ${SCENARIO} | Model: ${MODEL} | Trial: ${TRIAL}"
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
