#!/usr/bin/env bash
# ==============================================================================
# ICRA 2026 ESCAPE-Nav Go2 Real-Robot Table VIII Experiment Logger Script
# ==============================================================================
# Usage: bash record_experiment.sh <scenario_name> <model_name> <trial_id>
# Example Scenarios (Table VIII Core & Deployment):
#   - Dead_end_room
#   - Blocked_goal_direction
#   - Repeated_corridor
#   - Active_view_recovery
#   - Dynamic_obstacle
#
# Example Models:
#   - Direct_goal (Baseline)
#   - Full_ESCAPE_Nav (Ours)

SCENARIO=${1:-"Dead_end_room"}
MODEL=${2:-"Full_ESCAPE_Nav"}
TRIAL=${3:-"Trial1"}

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="bag_${SCENARIO}_${MODEL}_${TRIAL}_${TIMESTAMP}"
OUTPUT_DIR="experiments_bags/${SCENARIO}"

mkdir -p "${OUTPUT_DIR}"

echo "========================================================================"
echo " 🎬 [ESCAPE-Nav Table VIII] Recording Real-Robot Trial"
echo " Scenario : ${SCENARIO}"
echo " Model    : ${MODEL}"
echo " Trial    : ${TRIAL}"
echo " Buffer   : 100MB Queue Buffer (--max-cache-size 104857600)"
echo " Saving to: ${OUTPUT_DIR}/${BAG_NAME}"
echo " Press Ctrl+C when the episode completes (or times out)."
echo "========================================================================"

# I/O 최적화: 대용량 raw pointcloud 제외, 50Hz Odom, 제어 속도, 압축 비전, TF만 선별 기록
# 100MB 캐시 큐 버퍼를 지정하여 Jetson Flash 스토리지 쓰기 I/O 지연 방지
ros2 bag record -o "${OUTPUT_DIR}/${BAG_NAME}" \
    --max-cache-size 104857600 \
    /rtabmap/odom \
    /odom \
    /cmd_vel \
    /camera/front/image_raw \
    /tf \
    /tf_static
