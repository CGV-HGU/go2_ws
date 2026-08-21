#!/bin/bash
# ==============================================================================
# 🐕 Unitree Go2 Safe Lab Micro-Motion Verification Runner (scratch/run_test.sh)
# ==============================================================================
# 1. Launches official Go2 Robot Driver (go2_driver) in background
# 2. Runs safe micro-motion sequence (Forward 15cm -> Backward 15cm)
# 3. Cleanly stops all nodes on completion or Ctrl+C
# ==============================================================================

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." >/dev/null 2>&1 && pwd )"
cd "$DIR"

# 1. Environment Setup
source /opt/ros/foxy/setup.bash 2>/dev/null || true
source /home/unitree/cyclonedds_ws/install/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$DIR/cyclonedds.xml"
export ROS_DOMAIN_ID=0

echo "========================================================================"
echo " 🐕 Unitree Go2 Safe Lab Micro-Motion Verification (run_test.sh)"
echo "========================================================================"
echo " ⚠️  [안전 확인 사항]"
echo "   1. 로봇이 기립(Stand-Up) 상태인지 확인해 주세요."
echo "   2. 로봇 앞/뒤 30cm 이내에 장애물이 없는지 확인해 주세요."
echo "   3. 무선 조종기를 손에 쥐고 이상 시 즉시 'L2 + B'를 누를 준비를 해주세요."
echo "========================================================================"
echo ""

DRIVER_PID=""
cleanup() {
    echo ""
    echo "🛑 [STOP] Cleaning up driver and sending safety stop..."
    # 0 속도 안전 명령 발행
    python3 -c "
import rclpy
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node('emergency_stop_pub')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
cmd = Twist()
for _ in range(5):
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.01)
rclpy.shutdown()
" 2>/dev/null || true

    if [ -n "$DRIVER_PID" ]; then
        kill "$DRIVER_PID" 2>/dev/null || true
    fi
    pkill -f go2_driver 2>/dev/null || true
    echo "✅ Cleaned up safely."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 2. Launch Go2 Driver if not already running
if ! pgrep -f "go2_driver" >/dev/null; then
    echo "🤖 [1/2] Launching Go2 Robot Driver (go2_driver)..."
    ros2 launch go2_driver go2_driver.launch.py >/dev/null 2>&1 &
    DRIVER_PID=$!
    sleep 2
    echo "✅ Go2 Driver active!"
else
    echo "✅ Go2 Driver is already running."
fi

# 3. Execute Micro-Motion Test
echo "🚀 [2/2] Running Micro-Motion sequence..."
python3 scratch/test_lab_micro_motion.py
