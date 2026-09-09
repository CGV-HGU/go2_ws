#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2 ESCAPE-Nav (Docker Architecture) One-Click Runner
# Author: DeepMind Antigravity & Unitree Antarctica Team
# ==============================================================================
set -e

REPO_DIR="/home/unitree/s2e-vlm-async-framework-minimal"
OPERATOR_SCRIPT="$REPO_DIR/scripts/run_escape_operator.py"
GO2_ROBOT_IP="192.168.123.161"
VLM_URL="http://server-02.cgv:8000/v1/models"

# ANSI Colors
CYAN="\033[0;36m"
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
BOLD="\033[1m"
NC="\033[0m"

echo -e "${BOLD}${CYAN}========================================================================${NC}"
echo -e "${BOLD}${CYAN} 🚀 [ESCAPE-Nav] Unitree Go2 Docker Autonomy Stack Launcher${NC}"
echo -e "${BOLD}${CYAN}========================================================================${NC}"

# 1. Preflight Check: Robot Body Reachability
echo -ne "${CYAN}🛰️  [1/4] Checking Go2 Robot Body Connection (${GO2_ROBOT_IP})... ${NC}"
if ping -c 1 -W 1 "$GO2_ROBOT_IP" >/dev/null 2>&1; then
    echo -e "${GREEN}[OK] Connected.${NC}"
else
    echo -e "${YELLOW}[WARNING] Body Unreachable!${NC}"
    echo -e "${YELLOW}   👉 Please turn ON the Go2 robot power button and ensure Ethernet is connected.${NC}"
    echo -ne "   Press [ENTER] to continue anyway, or Ctrl+C to abort (auto-continue in 5s): "
    read -t 5 -r || true
    echo ""
fi

# 2. Preflight Check: External VLM Server
echo -ne "${CYAN}🧠 [2/4] Checking VLM Server Connectivity (${VLM_URL})... ${NC}"
if curl -s -m 2 "$VLM_URL" >/dev/null 2>&1; then
    echo -e "${GREEN}[OK] Qwen3.5-9B Online.${NC}"
else
    echo -e "${RED}[WARNING] VLM Server Unreachable!${NC}"
    echo -e "   Please verify that http://server-02.cgv:8000 is running."
fi

# 3. Docker Containers Bringup
echo -e "${CYAN}🐳 [3/4] Ensuring Docker Containers are active...${NC}"
CONTAINERS=(
    "escape-robot-minimal-camera-1"
    "escape-robot-minimal-pixnav-1"
    "escape-robot-minimal-unitree-command-1"
    "escape-robot-minimal-escape-1"
)

# Start any stopped containers
for c in "${CONTAINERS[@]}"; do
    if docker ps -a --format '{{.Names}}' | grep -Eq "^${c}\$"; then
        STATUS=$(docker inspect -f '{{.State.Status}}' "$c" 2>/dev/null || echo "unknown")
        if [ "$STATUS" != "running" ]; then
            echo -e "   • Starting container: ${BOLD}$c${NC} (was $STATUS)..."
            docker start "$c" >/dev/null
        else
            echo -e "   • Container ${BOLD}$c${NC} is already ${GREEN}running${NC}."
        fi
    else
        echo -e "${YELLOW}   • Container $c not found. Running compose up...${NC}"
        docker compose --env-file "$REPO_DIR/config/robot-full.env" \
          -f "$REPO_DIR/compose.robot-minimal.yaml" \
          -f "$REPO_DIR/compose.robot-full.yaml" \
          up -d --no-recreate
        break
    fi
done

# 4. Sync latest operator script into escape container
echo -e "${CYAN}📦 [4/4] Syncing operator script into escape container...${NC}"
docker cp "$OPERATOR_SCRIPT" escape-robot-minimal-escape-1:/workspace-minimal/run_escape_operator.py

# Emergency Stop Guard Cleanup on Shell Exit
cleanup() {
    local exit_status=$?
    echo ""
    echo -e "${CYAN}🛑 [STOP GUARD] Ensuring robot motion is disabled...${NC}"
    docker exec escape-robot-minimal-escape-1 bash -lc \
      "source /opt/ros/jazzy/setup.bash && source /opt/s2e-robot-minimal/setup.bash && python3 -c \"
import rclpy
from std_srvs.srv import SetBool
rclpy.init()
n = rclpy.create_node('emergency_stop_guard')
cli = n.create_client(SetBool, '/s2e/robot/set_motion_enabled')
if cli.wait_for_service(timeout_sec=1.5):
    fut = cli.call_async(SetBool.Request(data=False))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=2.0)
\" 2>/dev/null || true"
    echo -e "${GREEN}✅ Robot safe stop enforced. Exiting.${NC}"
    exit "$exit_status"
}
trap cleanup EXIT INT TERM

# 5. Execute Autonomous Mission inside container
echo -e "\n${BOLD}${GREEN}========================================================================${NC}"
echo -e "${BOLD}${GREEN} 🐕 Launching ESCAPE-Nav Autonomous Driving Pipeline...${NC}"
echo -e "${BOLD}${GREEN}========================================================================${NC}\n"

docker exec -it escape-robot-minimal-escape-1 bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /opt/s2e-robot-minimal/setup.bash && exec python3 -u /workspace-minimal/run_escape_operator.py \"$@\""

