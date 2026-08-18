#!/usr/bin/env bash
# ==============================================================================
# ESCAPE-Nav Network Latency Diagnostic Tool
# ==============================================================================
# Diagnoses latency between Host, Remote Server, and Jetson.
# Usage: bash check_network_latency.sh

VPN_IP="100.96.204.119"
LOCAL_IP="192.168.123.99"
ROBOT_CONTROLLER_IP="192.168.123.161"

echo "========================================================================"
echo " 📡 [ESCAPE-Nav] Network Latency & Connectivity Diagnostics"
echo "========================================================================"

echo -n "[1/3] Checking NetBird VPN (${VPN_IP})... "
if ping -c 3 -W 1 "${VPN_IP}" > /dev/null 2>&1; then
    RTT=$(ping -c 3 "${VPN_IP}" | tail -1 | awk '{print $4}' | cut -d '/' -f 2)
    echo "🟢 OK (RTT: ${RTT} ms)"
else
    echo "🔴 FAILED (VPN Unreachable)"
fi

echo -n "[2/3] Checking Local LAN IP (${LOCAL_IP})... "
if ping -c 3 -W 1 "${LOCAL_IP}" > /dev/null 2>&1; then
    RTT=$(ping -c 3 "${LOCAL_IP}" | tail -1 | awk '{print $4}' | cut -d '/' -f 2)
    echo "🟢 OK (RTT: ${RTT} ms)"
else
    echo "🟡 Local LAN not on same subnet (Normal if on VPN)"
fi

echo -n "[3/3] Checking Go2 Motion Controller (${ROBOT_CONTROLLER_IP})... "
if ping -c 3 -W 1 "${ROBOT_CONTROLLER_IP}" > /dev/null 2>&1; then
    RTT=$(ping -c 3 "${ROBOT_CONTROLLER_IP}" | tail -1 | awk '{print $4}' | cut -d '/' -f 2)
    echo "🟢 OK (RTT: ${RTT} ms)"
else
    echo "🔴 FAILED (Check robot power / ethernet)"
fi

echo "========================================================================"
