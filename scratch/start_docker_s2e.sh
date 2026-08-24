#!/bin/bash
# ==============================================================================
# 🚀 1-Click Unitree Go2 Docker Autonomy Stack Launcher (S2E + VLM + UDP Bridge)
# ==============================================================================
set -e

CONTAINER_NAME="sdam_go2_container"

echo "============================================================================"
echo " 🐳 [Docker Stack] Starting S2E VLM Autonomy Stack in '${CONTAINER_NAME}'"
echo "============================================================================"

# 1. Ensure container is running
STATUS=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || echo "false")
if [ "$STATUS" != "true" ]; then
    echo "▶️  Starting Docker Container: $CONTAINER_NAME ..."
    docker start "$CONTAINER_NAME"
    sleep 1
fi

echo "🟢 Docker Container is ACTIVE."

# 2. Launch Docker Autonomy Nodes
echo "🚀 Launching Docker Bridge & S2E Navigation Stack..."
docker exec -it "$CONTAINER_NAME" bash -ic "
source /opt/ros/jazzy/setup.bash
source /workspace/go2_ws_antarctica/s2e-vlm-async-framework/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0
export QWEN_BASE_URL=\"http://100.96.60.15:8000/v1\"
export QWEN_MODEL=\"qwen3.8-27b-instruct\"
export PYTHONPATH=\"/workspace/go2_ws_antarctica/qwen_nav_memory_framework_v3/qwen_nav_memory_framework:\$PYTHONPATH\"

echo '▶️  Starting Docker Bridge in background (UDP 127.0.0.1:9091 ➔ 9090)...'
python3 /workspace/go2_ws_antarctica/scratch/docker_bridge.py &
BRIDGE_PID=\$!

echo '▶️  Starting S2E VLM Async Navigation Policy Node...'
python3 /workspace/go2_ws_antarctica/s2e-vlm-async-framework/src/vlm_s2e_async_node.py

kill \$BRIDGE_PID 2>/dev/null || true
"
