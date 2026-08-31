#!/usr/bin/env bash
# ========================================================================================
# 🖥️ [ESCAPE-Nav] Unitree Go2 Real-Time Live VLM Trajectory Display Launcher
# ========================================================================================
# Usage:
#   ./run_live_vlm_display.sh
#
# Shortcuts in GUI Window:
#   [q] or [ESC] : Exit safely and print session statistics
#   [f]          : Toggle Fullscreen / Windowed mode
#   [s]          : Capture instantaneous snapshot PNG
# ========================================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

echo "================================================================================"
echo " 🚀 [ESCAPE-Nav] Starting Real-Time Live VLM Trajectory Monitor"
echo "================================================================================"

# 1. Pre-flight Check: Go2 Robot Base
echo -n "[1/4] Checking Go2 Robot Base Connection (192.168.123.161)... "
if ping -c 1 -W 1 192.168.123.161 > /dev/null 2>&1; then
    echo "🟢 ONLINE"
else
    echo "⚠️ WARNING: Robot base 192.168.123.161 not reachable. Please check power."
fi

# 2. Pre-flight Check: Remote VLM Server
echo -n "[2/4] Checking Remote Qwen3.5-9B VLM Server (100.96.60.15:8000)... "
if curl -s --connect-timeout 2 http://100.96.60.15:8000/v1/models | grep -q "qwen" 2>/dev/null; then
    echo "🟢 CONNECTED"
else
    echo "⚠️ WARNING: Remote VLM server (100.96.60.15:8000) not responding. Check NetBird VPN."
fi

# 3. Pre-flight Check: Kernel Multicast Route
echo -n "[3/4] Checking Multicast Route (230.0.0.0/8)... "
if ip route | grep -q "230.0.0.0"; then
    echo "🟢 CONFIGURED"
else
    echo "ℹ️ Adding route 230.0.0.0/8 dev eth0..."
    sudo ip route add 230.0.0.0/8 dev eth0 2>/dev/null || true
fi

# 4. Display Configuration
export DISPLAY="${DISPLAY:-:0}"
echo "[4/4] Active Display Target: DISPLAY=${DISPLAY}"
echo "--------------------------------------------------------------------------------"
echo "🖥️ Starting GUI Window on Display..."
echo "🌐 Live Web UI Stream also available at: http://localhost:8888"
echo "⌨️ Keybindings: [q/ESC] Exit | [f] Fullscreen | [s] Save Snapshot"
echo "================================================================================"

# Launch Infinite Real-Time VLM GUI Engine
python3 "${SCRIPT_DIR}/scratch/run_infinite_live_vlm_gui.py"
