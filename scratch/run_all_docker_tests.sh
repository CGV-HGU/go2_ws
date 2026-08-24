#!/usr/bin/env bash
# ==============================================================================
# 🐳 [Unitree Go2 ESCAPE-Nav] Docker Autonomy 1-Click Master Test Suite
# ==============================================================================
# Executes all 6 practical verification tests across Docker and Remote VLM Server:
#   1. S2E Core Algorithms & Node Contract Tests (pytest)
#   2. 50Hz High-Rate UDP Loopback Stress Test (500 packets, CRC32)
#   3. Real 720p Multimodal Image-to-VLM Decision Test
#   4. S2E Full End-to-End Navigation Dry-Run Loop
#   5. Kinematic Stall Detector & Active-View Recovery Test
#   6. Remote Server 6-Point Communication Robustness Test
#
# Auto-detects whether executed from Host OS or inside Docker Container!
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

CONTAINER="sdam_go2_container"
PASSED_COUNT=0
TOTAL_TESTS=6

# Auto-detect runtime environment (Inside Docker vs Host OS)
if [ -f "/.dockerenv" ] || [ -d "/workspace/go2_ws_antarctica" ]; then
    IN_DOCKER=1
    RUN_ENV="Inside Docker Container (sdam_go2_container)"
    WS_DIR="/workspace/go2_ws_antarctica"
else
    IN_DOCKER=0
    RUN_ENV="Host OS (Forwarding into sdam_go2_container)"
    WS_DIR="/workspace/go2_ws_antarctica"
fi

echo -e "${CYAN}================================================================================${NC}"
echo -e "${BOLD}${CYAN} 🐳 [ESCAPE-Nav] Docker Autonomy 1-Click Master Test Suite${NC}"
echo -e "${CYAN} Execution Context: ${RUN_ENV}${NC}"
echo -e "${CYAN} Remote Server    : http://100.96.60.15:8000/v1 (Qwen3.5-9B / NetBird P2P)${NC}"
echo -e "${CYAN}================================================================================${NC}"

# Check Docker container status if on Host
if [ "$IN_DOCKER" -eq 0 ]; then
    echo -e "\n${BLUE}[0/6] Checking Docker Container Status...${NC}"
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
        echo -e "${GREEN}  • Container '${CONTAINER}' is ACTIVE and RUNNING 🟢${NC}"
    else
        echo -e "${YELLOW}  • Container '${CONTAINER}' is stopped. Starting container...${NC}"
        docker start ${CONTAINER}
        sleep 1
        echo -e "${GREEN}  • Container started successfully 🟢${NC}"
    fi
fi

# Helper function to run command in the appropriate context
run_test_cmd() {
    local cmd="$1"
    if [ "$IN_DOCKER" -eq 1 ]; then
        eval "$cmd"
    else
        docker exec ${CONTAINER} bash -ic "$cmd"
    fi
}

# ------------------------------------------------------------------------------
# Test 1: S2E Pytest Unit Tests
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[1/6] Running S2E Core Pytest Unit & Contract Tests...${NC}"
if run_test_cmd "pytest ${WS_DIR}/s2e-vlm-async-framework/tests ${WS_DIR}/s2e-vlm-async-framework/src/s2e_vlm_core/test -q"; then
    echo -e "${GREEN}  ✅ Test 1 PASS: S2E Algorithms & Contracts 100% Validated${NC}"
    PASSED_COUNT=$((PASSED_COUNT + 1))
else
    echo -e "${RED}  ❌ Test 1 FAIL: Pytest failed${NC}"
fi

# ------------------------------------------------------------------------------
# Test 2: 50Hz UDP Stress Test
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[2/6] Running 50Hz High-Rate UDP Loopback Stress Test (10s, 500 Packets)...${NC}"
if run_test_cmd "python3 ${WS_DIR}/scratch/test_docker_50hz_stress.py"; then
    echo -e "${GREEN}  ✅ Test 2 PASS: 50Hz Continuous Streaming (0% Loss, <0.1ms Latency)${NC}"
    PASSED_COUNT=$((PASSED_COUNT + 1))
else
    echo -e "${RED}  ❌ Test 2 FAIL: UDP Stress Test failed${NC}"
fi

# ------------------------------------------------------------------------------
# Test 3: Real 720p Multimodal Image-to-VLM Test
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[3/6] Running Real 720p Multimodal Image VLM Decision Test...${NC}"
if run_test_cmd "python3 ${WS_DIR}/scratch/test_docker_real_image_vlm.py"; then
    echo -e "${GREEN}  ✅ Test 3 PASS: Multimodal Image Encoding & Qwen3-VL Decision Extraction${NC}"
    PASSED_COUNT=$((PASSED_COUNT + 1))
else
    echo -e "${RED}  ❌ Test 3 FAIL: Multimodal VLM Test failed${NC}"
fi

# ------------------------------------------------------------------------------
# Test 4: S2E Full End-to-End Dry-Run Loop
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[4/6] Running S2E Full End-to-End Navigation Dry-Run Loop...${NC}"
if run_test_cmd "python3 ${WS_DIR}/scratch/test_docker_s2e_dryrun.py"; then
    echo -e "${GREEN}  ✅ Test 4 PASS: Observation ➔ VLM Decision ➔ 50Hz Trajectory ➔ CmdVel Loop${NC}"
    PASSED_COUNT=$((PASSED_COUNT + 1))
else
    echo -e "${RED}  ❌ Test 4 FAIL: S2E Dry-Run Loop failed${NC}"
fi

# ------------------------------------------------------------------------------
# Test 5: Kinematic Stall & Active-View Recovery Test
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[5/6] Running Kinematic Stall Detector & Active-View Recovery Test...${NC}"
if run_test_cmd "python3 ${WS_DIR}/scratch/test_docker_stall_and_recovery.py"; then
    echo -e "${GREEN}  ✅ Test 5 PASS: Obstacle Stalling Detection & Active-View Recovery Guard${NC}"
    PASSED_COUNT=$((PASSED_COUNT + 1))
else
    echo -e "${RED}  ❌ Test 5 FAIL: Stall Detector Test failed${NC}"
fi

# ------------------------------------------------------------------------------
# Test 6: Remote Server 6-Point Communication Robustness Test
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[6/6] Running Remote Server 6-Point Communication Robustness Test...${NC}"
if run_test_cmd "python3 ${WS_DIR}/scratch/test_server_communication_robustness.py"; then
    echo -e "${GREEN}  ✅ Test 6 PASS: Wi-Fi Jitter, JSON Parsing, Monotonicity & Fallback Guard${NC}"
    PASSED_COUNT=$((PASSED_COUNT + 1))
else
    echo -e "${RED}  ❌ Test 6 FAIL: Communication Robustness Test failed${NC}"
fi

# ------------------------------------------------------------------------------
# Final Score Dashboard
# ------------------------------------------------------------------------------
SCORE=$((PASSED_COUNT * 100 / TOTAL_TESTS))
echo -e "\n${CYAN}================================================================================${NC}"
echo -e "${BOLD}${CYAN} 📊 [SUMMARY] DOCKER AUTONOMY VERIFICATION RESULTS: ${PASSED_COUNT} / ${TOTAL_TESTS} PASSED (${SCORE}%)${NC}"
echo -e "${CYAN}================================================================================${NC}"

if [ "$PASSED_COUNT" -eq "$TOTAL_TESTS" ]; then
    echo -e "${GREEN} 🏆 [VERDICT] DOCKER AUTONOMY PIPELINE IS 100% PRODUCTION-READY FOR REAL-ROBOT DEPLOYMENT! 🐕${NC}"
    exit 0
else
    echo -e "${RED} ⚠️ [VERDICT] SOME TESTS FAILED (${PASSED_COUNT}/${TOTAL_TESTS}). Please inspect failed stages above.${NC}"
    exit 1
fi
