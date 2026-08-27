#!/bin/bash
# ==============================================================================
# Unitree Go2 LIO + RGB RTAB-Map / ESCAPE-Nav bringup
# ==============================================================================
# Orchestrates:
#   Tier 1: Go2 Mainboard Hardware (192.168.123.161)
#   Tier 2: Jetson Host OS (camera, Unitree LIO bridge, RTAB-Map)
#   Tier 3: Docker Sandbox (sdam_go2_container S2E Async Policy Node)
#   Tier 4: Remote GPU VLM Server (100.96.60.15:8000 Qwen3-VL)
#
# Usage:
#   bash scratch/bringup_all_escape_nav.sh             # Online Autonomy Mode (localization:=true)
#   bash scratch/bringup_all_escape_nav.sh --mapping   # New 3D map (backs up DB, then uses -d)
#   bash scratch/bringup_all_escape_nav.sh --mapping --planar
#                                                     # Planar x/y/yaw graph with 3D LiDAR ICP
#   bash scratch/bringup_all_escape_nav.sh --record <Scenario> <Model> <Trial>
# ==============================================================================

set -e

MODE_ARG="localization:=true"
MAPPING_MODE=false
RECORD_MODE=false
GUI_MODE=false
GUI_ARG="rtabmap_viz:=false"
PRINT_CONFIG=false
GRAPH_PROFILE="4dof"
GRAPH_ARGS=(
    "reg_force_3dof:=false"
    "icp_force_4dof:=true"
    "optimizer_slam_2d:=false"
)
SCENARIO="Dead_end_room"
MODEL="Full_ESCAPE_Nav"
TRIAL="Trial1"

while [[ $# -gt 0 ]]; do
    case $1 in
        --mapping)
            MAPPING_MODE=true
            MODE_ARG="localization:=false"
            shift
            ;;
        --gui)
            GUI_MODE=true
            GUI_ARG="rtabmap_viz:=true"
            export DISPLAY="${DISPLAY:-:0}"
            shift
            ;;
        --planar)
            GRAPH_PROFILE="planar3dof"
            GRAPH_ARGS=(
                "reg_force_3dof:=true"
                "icp_force_4dof:=false"
                "optimizer_slam_2d:=true"
            )
            shift
            ;;
        --print-config)
            PRINT_CONFIG=true
            shift
            ;;
        --record)
            RECORD_MODE=true
            SCENARIO=${2:-"Dead_end_room"}
            MODEL=${3:-"Full_ESCAPE_Nav"}
            TRIAL=${4:-"Trial1"}
            shift 4 2>/dev/null || shift
            ;;
        *)
            shift
            ;;
    esac
done

if [ "$GRAPH_PROFILE" = "planar3dof" ] && [ "$MAPPING_MODE" = false ]; then
    echo "Error: --planar is accepted only together with --mapping."
    exit 2
fi

if [ "$PRINT_CONFIG" = true ]; then
    echo "mapping_mode=$MAPPING_MODE"
    echo "gui_mode=$GUI_MODE"
    echo "graph_profile=$GRAPH_PROFILE"
    printf 'graph_arg=%s\n' "${GRAPH_ARGS[@]}"
    echo "recorder=$RECORD_MODE"
    echo "run_dir=${RTABMAP_RUN_DIR:-default}"
    exit 0
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

PIDS=()

cleanup() {
    local cleanup_status=$?
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo -e "${YELLOW}========================================================================${NC}"
    if [ "$MAPPING_MODE" = true ]; then
        echo -e "${YELLOW} 🛑 [MAPPING SHUTDOWN] Stopping sensor and mapping processes...${NC}"
    else
        echo -e "${YELLOW} 🛑 [ESCAPE-Nav E-STOP] Shutting down all processes safely...${NC}"
    fi
    echo -e "${YELLOW}========================================================================${NC}"
    
    # 1. 도커 내부 S2E 노드 정지 (mapping mode never starts it)
    echo -e "${CYAN}[1/4] Stopping Docker S2E Autonomy Node...${NC}"
    if [ "$MAPPING_MODE" = false ]; then
        docker exec sdam_go2_container pkill -f vlm_s2e_async_node.py 2>/dev/null || true
    fi
    
    # 2. 백그라운드 프로세스 정리
    echo -e "${CYAN}[2/4] Killing background host processes...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null || true
        fi
    done
    # Give the read-only loop logger time to fsync its SUMMARY before the
    # fallback process-name cleanup below.
    for pid in "${PIDS[@]}"; do
        for _ in {1..20}; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.05
        done
    done
    pkill -f go2_front_camera_publisher.py 2>/dev/null || true
    pkill -f unitree_lidar_ros2_node 2>/dev/null || true
    pkill -f go2_native_sensor_node.py 2>/dev/null || true
    pkill -f go2_livo_sensor_bridge.py 2>/dev/null || true
    pkill -f host_bridge.py 2>/dev/null || true
    pkill -f go2_rtabmap.launch.py 2>/dev/null || true
    pkill -f rtabmap 2>/dev/null || true
    
    # 3. Mapping never creates a command path. Online autonomy is not an
    # accepted physical mode, but preserve its historical zero-packet cleanup.
    if [ "$MAPPING_MODE" = false ]; then
        echo -e "${CYAN}[3/4] Sending zero velocity safety command...${NC}"
        python3 -c "
import socket, struct, zlib
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
raw = struct.pack('6d', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
crc = zlib.crc32(raw) & 0xFFFF
packet = struct.pack('!IH', 0x53324501, crc) + raw
sock.sendto(packet, ('127.0.0.1', 9090))
" 2>/dev/null || true
    else
        echo -e "${CYAN}[3/4] Mapping mode: no command socket or motor packet was created.${NC}"
    fi

    echo -e "${GREEN}[4/4] All systems safely terminated. Bye! 🐕${NC}"
    exit "$cleanup_status"
}

trap cleanup SIGINT SIGTERM EXIT

echo -e "${CYAN}========================================================================${NC}"
echo -e "${CYAN} 🐕 [Unitree Go2 ESCAPE-Nav] 4-Tier Master Bringup System${NC}"
echo -e "${CYAN} Mode    : $([ "$MAPPING_MODE" = true ] && echo "🗺️ 3D MAPPING" || echo "🚀 ONLINE AUTONOMOUS NAVIGATION")${NC}"
echo -e "${CYAN} Graph   : ${GRAPH_PROFILE} (${GRAPH_ARGS[*]})${NC}"
echo -e "${CYAN} Host    : Jetson Orin NX (Ubuntu 20.04 Foxy / CUDA 11.4)${NC}"
echo -e "${CYAN} Docker  : sdam_go2_container (Ubuntu 24.04 Jazzy ARM64)${NC}"
echo -e "${CYAN} Server  : RTX Pro 6000 (100.96.60.15:8000 Qwen3-VL)${NC}"
echo -e "${CYAN}========================================================================${NC}"

# ------------------------------------------------------------------------------
# Phase 1: 3초 사전 헬스체크 (Pre-Flight Diagnostics)
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}🔍 [Phase 1/4] Running Pre-Flight Health Checks...${NC}"

echo -n "  • Checking Go2 Mainboard (192.168.123.161)... "
if ping -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    echo -e "${GREEN}ONLINE (0.2ms)${NC}"
else
    echo -e "${RED}OFFLINE! Please turn on robot battery.${NC}"
    exit 1
fi

if [ "$MAPPING_MODE" = false ]; then
    echo -n "  • Checking Remote VLM Server (100.96.60.15)... "
    if ping -c 1 -W 2 100.96.60.15 >/dev/null 2>&1; then
        echo -e "${GREEN}ONLINE (14ms VPN Direct)${NC}"
    else
        echo -e "${YELLOW}WARNING (VPN Unreachable - Running in Offline/Fallback Mode)${NC}"
    fi

    echo -n "  • Checking Docker Container (sdam_go2_container)... "
    if docker ps --format '{{.Names}}' | grep -q "^sdam_go2_container$"; then
        echo -e "${GREEN}RUNNING${NC}"
    else
        echo -e "${YELLOW}STARTING CONTAINER...${NC}"
        docker start sdam_go2_container >/dev/null 2>&1 || true
    fi
fi

# ------------------------------------------------------------------------------
# Phase 2: Host OS 센서 및 RTAB-Map LIVO 가동
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}📷 [Phase 2/4] Launching Host Sensor & RTAB-Map LIVO Stack...${NC}"

source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash 2>/dev/null || true
source /home/unitree/backup/legacy_workspaces/go2_analysis/go2_ws/install/setup.bash 2>/dev/null || true
source /home/unitree/go2_ws_antarctica/install/setup.bash 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/unitree/go2_ws_antarctica/cyclonedds.xml"
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH=/home/unitree/opencv_build/opencv/build/lib:/usr/local/lib:$LD_LIBRARY_PATH

# Clean up any stale background nodes from previous runs
pkill -9 -f unitree_lidar 2>/dev/null || true
pkill -9 -f go2_native_sensor 2>/dev/null || true
pkill -9 -f go2_livo_sensor_bridge 2>/dev/null || true
pkill -9 -f go2_front_camera 2>/dev/null || true
pkill -9 -f rtabmap_loop_logger 2>/dev/null || true
pkill -9 -f host_bridge 2>/dev/null || true
pkill -9 -f rtabmap 2>/dev/null || true
sleep 1

# Built-in Go2 topics arrive over CycloneDDS. The external L2 SDK and its
# 192.168.1.2/UDP 6201 setup are intentionally not started on this path. Never
# store or pipe a credential here: use an existing route or non-interactive
# sudo configured by the operator.
if ! ip route show 230.0.0.0/8 2>/dev/null | grep -q 'dev eth0'; then
    sudo -n ip route add 230.0.0.0/8 dev eth0 2>/dev/null || true
fi
if ! ip route show 230.0.0.0/8 2>/dev/null | grep -q 'dev eth0' && [ -t 0 ]; then
    echo -e "${YELLOW}  • Go2 DDS multicast route is missing; sudo authentication is required once.${NC}"
    sudo ip route add 230.0.0.0/8 dev eth0 || true
fi
if ! ip route show 230.0.0.0/8 2>/dev/null | grep -q 'dev eth0'; then
    echo -e "${RED}ERROR: multicast route 230.0.0.0/8 via eth0 is missing.${NC}"
    echo "Run 'sudo ip route add 230.0.0.0/8 dev eth0' in a terminal; no credential is stored by this script."
    exit 1
fi
echo "  • Go2 DDS multicast route ready: $(ip route show 230.0.0.0/8)"

# 1. 전면 카메라 퍼블리셔 (30fps + CameraInfo)
echo "  • [1/4] Starting Front Camera & CameraInfo Publisher (30fps)..."
python3 /home/unitree/go2_ws_antarctica/scratch/go2_front_camera_publisher.py &
PIDS+=($!)
sleep 1

# 2. Built-in Unitree LiDAR odometry + IMU + deskewed cloud bridge.
echo "  • [2/4] Starting Unitree LIO time/frame bridge (/livo/*, no actuation)..."
python3 /home/unitree/go2_ws_antarctica/scratch/go2_livo_sensor_bridge.py \
    --ros-args -p cloud_mode:=deskewed -p imu_quaternion_order:=auto &
PIDS+=($!)
sleep 1

# 3. The command-capable Docker bridge is not needed during mapping.
if [ "$MAPPING_MODE" = false ]; then
    echo "  • [3/4] Starting Host-to-Docker UDP Socket Bridge..."
    python3 /home/unitree/go2_ws_antarctica/scratch/host_bridge.py &
    PIDS+=($!)
    sleep 1
fi

# Mapping evidence logger. RTAB-Map publishes /info only while it has a
# subscriber, so start this before the mapping node. It is read-only with
# respect to ROS and writes only closure events/heartbeats to ~/.ros.
LOOP_LOG_DIR="${RTABMAP_RUN_DIR:-/home/unitree/.ros/rtabmap_loop_logs}"
if [ -n "${RTABMAP_RUN_DIR:-}" ]; then
    LOOP_LOG_DIR="${RTABMAP_RUN_DIR}/loop_logs"
fi
if [ "$MAPPING_MODE" = true ]; then
    LOOP_RUN_LABEL="headless_${GRAPH_PROFILE}"
    if [ "$GUI_MODE" = true ]; then
        LOOP_RUN_LABEL="gui_${GRAPH_PROFILE}"
    fi
    echo "  • [3/4] Starting RTAB-Map loop event logger (global/proximity/rejected)..."
    python3 /home/unitree/go2_ws_antarctica/scratch/rtabmap_loop_logger.py \
        --ros-args \
        -p info_topic:=/info \
        -p output_dir:="$LOOP_LOG_DIR" \
        -p run_label:="$LOOP_RUN_LABEL" &
    PIDS+=($!)
    sleep 1
fi

# Mapping mode intentionally creates a new DB. Preserve the current DB first.
RTABMAP_DB="/home/unitree/.ros/rtabmap.db"
if [ "$MAPPING_MODE" = true ] && [ -f "$RTABMAP_DB" ]; then
    BACKUP_DIR="/home/unitree/.ros/rtabmap_backups"
    mkdir -p "$BACKUP_DIR"
    BACKUP_PATH="$BACKUP_DIR/rtabmap_$(date +%Y%m%d_%H%M%S).db"
    cp -a "$RTABMAP_DB" "$BACKUP_PATH"
    echo "  • Existing RTAB-Map DB backed up to: $BACKUP_PATH"
fi

# 4. RTAB-Map consumes external Unitree LIO odometry; it does not publish VO.
echo "  • [MASTER] Starting RTAB-Map LIO + visual-place mapping (${MODE_ARG}, ${GUI_ARG})..."
ros2 launch rtabmap_launch go2_rtabmap.launch.py \
    ${MODE_ARG} ${GUI_ARG} "${GRAPH_ARGS[@]}" &
PIDS+=($!)
sleep 3

# ------------------------------------------------------------------------------
# Phase 3: 도커 샌드박스 S2E 비동기 자율주행 가동
# ------------------------------------------------------------------------------
if [ "$MAPPING_MODE" = false ]; then
    echo -e "\n${BLUE}🧠 [Phase 3/4] Launching Docker S2E Autonomous Navigation Node...${NC}"
    docker exec -d sdam_go2_container bash -c "
        source /opt/ros/jazzy/setup.bash
        source /workspace/go2_ws_antarctica/s2e-vlm-async-framework/install/setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID=0
        python3 /workspace/go2_ws_antarctica/scratch/docker_bridge.py &
        python3 /workspace/go2_ws_antarctica/s2e-vlm-async-framework/src/vlm_s2e_async_node.py
    "
    echo -e "${GREEN}  • Docker S2E Policy Node Active (Closed-loop 50Hz)!${NC}"
fi

# ------------------------------------------------------------------------------
# Phase 4: Rosbag 녹화 (옵션) 및 실시간 모니터링 대시보드
# ------------------------------------------------------------------------------
if [ "$RECORD_MODE" = true ]; then
    echo -e "\n${BLUE}🎬 [Phase 4/4] Starting 100MB Queue Rosbag Logger (${SCENARIO} / ${TRIAL})...${NC}"
    bash /home/unitree/go2_ws_antarctica/scratch/record_experiment.sh "${SCENARIO}" "${MODEL}" "${TRIAL}" &
    PIDS+=($!)
fi

echo -e "\n${GREEN}========================================================================${NC}"
if [ "$MAPPING_MODE" = true ]; then
    echo -e "${GREEN} ✅ [MAPPING STACK LIVE] Sensors, LIO and RTAB-Map are active.${NC}"
else
    echo -e "${GREEN} ✅ [ALL SYSTEMS LIVE] Unitree Go2 ESCAPE-Nav is now fully operational!${NC}"
fi
echo -e "${GREEN}    - Unitree LIO input  : /livo/odom + /livo/imu + /livo/cloud${NC}"
echo -e "${GREEN}    - RTAB-Map update    : 2Hz, LiDAR ICP + RGB place recognition${NC}"
echo -e "${GREEN}    - Front RGB Camera   : /camera/front/image_raw${NC}"
if [ "$MAPPING_MODE" = false ]; then
    echo -e "${GREEN}    - Docker S2E Autonomy: Active${NC}"
else
    echo -e "${GREEN}    - Recorder/Actuation : Disabled in mapping mode${NC}"
    echo -e "${GREEN}    - Loop event logs     : ${LOOP_LOG_DIR}/loop_events_*${NC}"
fi
echo -e "${GREEN}========================================================================${NC}"
if [ "$MAPPING_MODE" = true ]; then
    echo -e "${YELLOW}👉 Press Ctrl+C once to save logs and stop mapping (no motor command is sent).${NC}"
else
    echo -e "${YELLOW}👉 Press Ctrl+C at any time to safely stop the robot and exit.${NC}"
fi
echo ""

# 메인 프로세스 유지
wait
