#!/usr/bin/env bash
# ==============================================================================
# ICRA 2026 ESCAPE-Nav Go2 Real-Robot Table VIII Experiment Logger Script
# ==============================================================================
# Usage: bash record_experiment.sh <scenario_name> <model_name> <trial_id> [--with-image]
#
# Storage Optimization:
#   - Default (Lightweight): Records ONLY 50Hz Odom, CmdVel, TF, Collision (~3MB / trial)
#   - --with-image         : Records raw camera frame (Caution: ~80MB/s storage load)
# ==============================================================================

SCENARIO=${1:-"Dead_end_room"}
MODEL=${2:-"Full_ESCAPE_Nav"}
TRIAL=${3:-"Trial1"}
WITH_IMAGE=false

if [[ "$*" == *"--with-image"* ]]; then
    WITH_IMAGE=true
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="bag_${SCENARIO}_${MODEL}_${TRIAL}_${TIMESTAMP}"
OUTPUT_DIR="experiments_bags/${SCENARIO}"

mkdir -p "${OUTPUT_DIR}"

echo "========================================================================"
echo " 🎬 [ESCAPE-Nav Table VIII] Recording Real-Robot Trial (Host OS)"
echo " Scenario : ${SCENARIO}"
echo " Model    : ${MODEL}"
echo " Trial    : ${TRIAL}"
echo " Mode     : $([ "$WITH_IMAGE" = true ] && echo "📸 RAW IMAGES INCLUDED (Heavy)" || echo "⚡ LIGHTWEIGHT NUMERIC (Safe <3MB/trial)")"
echo " Buffer   : 100MB Queue Buffer (--max-cache-size 104857600)"
echo " Saving to: ${OUTPUT_DIR}/${BAG_NAME}"
echo " Press Ctrl+C when the episode completes (or times out)."
echo "========================================================================"

# 기본 기록 토픽 (초경량 수치 데이터: 20회 주행 총합 < 60MB)
TOPICS=(
    /rtabmap/odom
    /odom
    /cmd_vel
    /robot/collision_detected
    /robot/obstacle_status
    /tf
    /tf_static
)

# 옵션: 고화질 비디오 필요 시에만 추가
if [ "$WITH_IMAGE" = true ]; then
    TOPICS+=(/camera/front/image_raw)
fi

# I/O 최적화: 100MB 큐 버퍼로 NVMe/Flash 쓰기 지연 방지
ros2 bag record -o "${OUTPUT_DIR}/${BAG_NAME}" \
    --max-cache-size 104857600 \
    "${TOPICS[@]}"
