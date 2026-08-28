#!/usr/bin/env bash
# Read-only/no-actuation audit for Robot -> Jetson -> Docker -> Server.
# It never starts mapping, ROS navigation nodes, command bridges or motor APIs.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-status}"
ROBOT_IP="${GO2_MAINBOARD_IP:-192.168.123.161}"
CONTAINER="${ESCAPE_DOCKER_CONTAINER:-sdam_go2_container}"
SERVER_BASE="${QWEN_BASE_URL:-http://100.96.60.15:8000/v1}"
MODEL="${QWEN_MODEL:-qwen3.5-9b-instruct}"

usage() {
    echo "Usage: ./test_4tier_no_actuation.sh [status|full]"
    echo "  status : services, Robot reachability, ROS graph, Docker process, server model endpoint"
    echo "  full   : status + Jetson PixNav tests + Docker package tests + text-only server inference"
}

if [[ "$MODE" != "status" && "$MODE" != "full" ]]; then
    usage
    exit 2
fi

cd "$ROOT_DIR"

echo "========================================================================"
echo " ESCAPE-Nav 4-Tier no-actuation audit: $MODE"
echo " ROS launch=false  Command bridge=false  SDK=false  Actuation=false"
echo "========================================================================"

echo "[Tier 1/4] Go2 reachability only"
if ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1; then
    echo "TIER1_REACHABLE_NO_SENSOR_QUALIFICATION: $ROBOT_IP"
else
    echo "TIER1_BLOCKED_ROBOT_OFFLINE_OR_UNREACHABLE: $ROBOT_IP"
fi

echo "[Tier 2/4] Jetson host state"
echo "git=$(git rev-parse --short HEAD)"
systemctl is-active docker netbird NetworkManager
if pgrep -af 'map_headless|rtabmap|icp_odometry|go2_livo_sensor_bridge|host_bridge|docker_bridge' \
    | grep -v 'pgrep -af' >/dev/null; then
    echo "BLOCKED_UNEXPECTED_MAPPING_OR_BRIDGE_PROCESS" >&2
    pgrep -af 'map_headless|rtabmap|icp_odometry|go2_livo_sensor_bridge|host_bridge|docker_bridge' >&2
    exit 3
fi
echo "TIER2_NO_MAPPING_OR_COMMAND_BRIDGE_PROCESS"

echo "[Tier 3/4] Docker process state"
docker ps --filter "name=^/${CONTAINER}$" --format 'container={{.Names}} image={{.Image}} status={{.Status}}'
docker top "$CONTAINER" -eo pid,ppid,comm,args
docker exec "$CONTAINER" bash -lc '
  source /opt/ros/jazzy/setup.bash
  if [ -f /workspace/go2_ws_antarctica/s2e-vlm-async-framework/install/setup.bash ]; then
    source /workspace/go2_ws_antarctica/s2e-vlm-async-framework/install/setup.bash
  fi
  printf "executables="
  ros2 pkg executables | grep -c "^s2e_vlm_" || true
  printf "live_nodes="
  ros2 node list | grep -c "^/s2e/" || true
'

echo "[Tier 4/4] Server reachability from Host and Docker"
curl --silent --show-error --fail --connect-timeout 3 --max-time 8 \
    "$SERVER_BASE/models" >/dev/null
docker exec "$CONTAINER" curl --silent --show-error --fail --connect-timeout 3 --max-time 8 \
    "$SERVER_BASE/models" >/dev/null
echo "TIER4_MODEL_ENDPOINT_REACHABLE_FROM_HOST_AND_DOCKER"

if [[ "$MODE" == "status" ]]; then
    echo "STATUS_AUDIT_COMPLETE_NO_ACTUATION"
    exit 0
fi

echo "[Full 1/3] Jetson file-only PixNav regression"
./test_pixnav_offline.sh quick

echo "[Full 2/3] Docker package-isolated tests"
docker exec "$CONTAINER" bash -lc '
  set -e
  cd /workspace/go2_ws_antarctica/s2e-vlm-async-framework
  python3 -m pytest -q src/s2e_vlm_core/test
  python3 -m pytest -q src/s2e_vlm_bringup/test_launch_contracts.py
  python3 -m pytest -q src/s2e_vlm_nodes/test
'
echo "TIER3_CONTRACT_TESTS_PASS_NOT_LIVE_NAVIGATION"

echo "[Full 3/3] Text-only structured server inference"
server_response="$(
    curl --silent --show-error --fail --connect-timeout 3 --max-time 30 \
        "$SERVER_BASE/chat/completions" \
        -H 'Content-Type: application/json' \
        --data "{\"model\":\"$MODEL\",\"temperature\":0,\"max_tokens\":80,\"response_format\":{\"type\":\"json_schema\",\"json_schema\":{\"name\":\"no_actuation_status\",\"strict\":true,\"schema\":{\"type\":\"object\",\"properties\":{\"schema_version\":{\"type\":\"string\",\"const\":\"escape_nav_server_probe_v1\"},\"action\":{\"type\":\"string\",\"const\":\"stop\"}},\"required\":[\"schema_version\",\"action\"],\"additionalProperties\":false}}},\"messages\":[{\"role\":\"user\",\"content\":\"Return the required no-actuation status object.\"}]}"
)"
python3 -c '
import json
import sys
outer = json.load(sys.stdin)
inner = json.loads(outer["choices"][0]["message"]["content"])
assert inner == {"schema_version": "escape_nav_server_probe_v1", "action": "stop"}
print("TIER4_STRUCTURED_TEXT_INFERENCE_PASS")
' <<<"$server_response"

echo "------------------------------------------------------------------------"
echo "PASS_4TIER_AVAILABLE_NO_ACTUATION_TESTS"
echo "Tier 1 remains blocked unless the Robot is powered and real sensor topics are qualified."
echo "Tier 3 package tests are contract/mock evidence; the container remains idle."
echo "No physical command path was started."
