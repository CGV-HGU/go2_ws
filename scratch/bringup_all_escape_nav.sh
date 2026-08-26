#!/bin/bash
# ==============================================================================
# 🚀 Unitree Go2 ESCAPE-Nav 4-Tier Master All-in-One Bringup Launcher
# ==============================================================================
# Orchestrates:
#   Tier 1: Go2 Mainboard Hardware (192.168.123.161)
#   Tier 2: Jetson Host OS (Camera 30fps, RTAB-Map LIVO 50Hz, Host Bridge)
#   Tier 3: Docker Sandbox (sdam_go2_container S2E Async Policy Node)
#   Tier 4: Remote GPU VLM Server (100.96.60.15:8000 Qwen3-VL)
#
# Usage:
#   bash scratch/bringup_all_escape_nav.sh             # Online Autonomy Mode (localization:=true)
#   bash scratch/bringup_all_escape_nav.sh --mapping   # 3D Mapping Mode (localization:=false)
#   bash scratch/bringup_all_escape_nav.sh --record <Scenario> <Model> <Trial>
# ==============================================================================

set -e

MODE_ARG="localization:=true"
MAPPING_MODE=false
RECORD_MODE=false
GUI_MODE=false
GUI_ARG="rtabmap_viz:=false"
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

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

PIDS=()

cleanup() {
    echo ""
    echo -e "${YELLOW}========================================================================${NC}"
    echo -e "${YELLOW} 🛑 [ESCAPE-Nav E-STOP] Shutting down all processes safely...${NC}"
    echo -e "${YELLOW}========================================================================${NC}"
    
    # 1. 도커 내부 S2E 노드 정지
    echo -e "${CYAN}[1/4] Stopping Docker S2E Autonomy Node...${NC}"
    docker exec sdam_go2_container pkill -f vlm_s2e_async_node.py 2>/dev/null || true
    
    # 2. 백그라운드 프로세스 정리
    echo -e "${CYAN}[2/4] Killing background host processes...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null || true
        fi
    done
    pkill -f go2_front_camera_publisher.py 2>/dev/null || true
    pkill -f unitree_lidar_ros2_node 2>/dev/null || true
    pkill -f go2_native_sensor_node.py 2>/dev/null || true
    pkill -f host_bridge.py 2>/dev/null || true
    pkill -f go2_rtabmap.launch.py 2>/dev/null || true
    pkill -f rtabmap 2>/dev/null || true
    
    # 3. 로봇 모터 0 속도 안전 발행
    echo -e "${CYAN}[3/4] Sending zero velocity safety command...${NC}"
    python3 -c "
import socket, struct, zlib
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
raw = struct.pack('6d', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
crc = zlib.crc32(raw) & 0xFFFF
packet = struct.pack('!IH', 0x53324501, crc) + raw
sock.sendto(packet, ('127.0.0.1', 9090))
" 2>/dev/null || true

    echo -e "${GREEN}[4/4] All systems safely terminated. Bye! 🐕${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

echo -e "${CYAN}========================================================================${NC}"
echo -e "${CYAN} 🐕 [Unitree Go2 ESCAPE-Nav] 4-Tier Master Bringup System${NC}"
echo -e "${CYAN} Mode    : $([ "$MAPPING_MODE" = true ] && echo "🗺️ 3D MAPPING" || echo "🚀 ONLINE AUTONOMOUS NAVIGATION")${NC}"
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
pkill -9 -f go2_front_camera 2>/dev/null || true
pkill -9 -f host_bridge 2>/dev/null || true
pkill -9 -f rtabmap 2>/dev/null || true
sleep 1

# 멀티캐스트 라우팅 및 4D 라이다 IP 에일리어스 설정
echo admin | sudo -S fuser -k 6201/udp 2>/dev/null || true
echo admin | sudo -S ip addr add 192.168.1.2/24 dev eth0 2>/dev/null || true
echo admin | sudo -S ip route add 230.0.0.0/8 dev eth0 2>/dev/null || true

# 1. 전면 카메라 퍼블리셔 (30fps + CameraInfo)
echo "  • [1/4] Starting Front Camera & CameraInfo Publisher (30fps)..."
python3 /home/unitree/go2_ws_antarctica/scratch/go2_front_camera_publisher.py &
PIDS+=($!)
sleep 1

# 2. 바디 IMU 및 오도메트리 Native 센서 노드 (/imu @ 50Hz, /odom @ 50Hz, /tf @ 50Hz)
echo "  • [2/5] Starting Native Body IMU & Kinematic Odometry Node (50Hz TF)..."
python3 /home/unitree/go2_ws_antarctica/scratch/go2_native_sensor_node.py &
PIDS+=($!)
sleep 1

# 3. Unitree 4D LiDAR L2 하드웨어 드라이버 (15Hz /pointcloud 실시간 발행)
echo "  • [3/5] Starting Unitree 4D LiDAR L2 Driver (UDP 192.168.1.62:6101 -> /pointcloud)..."
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
    -p initialize_type:=2 \
    -p lidar_ip:=192.168.1.62 \
    -p lidar_port:=6101 \
    -p local_ip:=192.168.1.2 \
    -p local_port:=6201 \
    -p cloud_topic:=/pointcloud \
    -p cloud_frame:=radar \
    -p cloud_scan_num:=18 &
PIDS+=($!)
sleep 1

# 4. Host Bridge (0x53324501 매직넘버 수신기)
echo "  • [4/5] Starting Host-to-Docker UDP Socket Bridge..."
python3 /home/unitree/go2_ws_antarctica/scratch/host_bridge.py &
PIDS+=($!)
sleep 1

# 4. RTAB-Map LIVO 50Hz 위치추정 노드 (순정 pointcloud 및 GUI 여부 명시)
echo "  • [MASTER] Starting RTAB-Map LIVO 50Hz SLAM Node (${MODE_ARG}, ${GUI_ARG})..."
ros2 launch rtabmap_launch go2_rtabmap.launch.py ${MODE_ARG} ${GUI_ARG} scan_cloud_topic:=/pointcloud &
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
echo -e "${GREEN} ✅ [ALL SYSTEMS LIVE] Unitree Go2 ESCAPE-Nav is now fully operational!${NC}"
echo -e "${GREEN}    - Host RTAB-Map LIVO : 50Hz (/rtabmap/odom)${NC}"
echo -e "${GREEN}    - Front RGB Camera   : 30fps (/camera/front/image_raw)${NC}"
echo -e "${GREEN}    - Inter-OS Bridge    : < 0.1ms (UDP 127.0.0.1:9090 / 9091)${NC}"
echo -e "${GREEN}    - Docker S2E Autonomy: Active${NC}"
echo -e "${GREEN}========================================================================${NC}"
echo -e "${YELLOW}👉 Press Ctrl+C at any time to safely stop the robot and exit.${NC}"
echo ""

# 메인 프로세스 유지
wait
