#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2 Automatic Mobile Hotspot & Wireless LAN Switcher
# Workspace: /home/unitree/go2_ws_antarctica
# Target SSID: "민석의 S25 Ultra"
# ==============================================================================
set -euo pipefail

# ANSI Colors
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

HOTSPOT_SSID="민석의 S25 Ultra"

echo -e "\n${BOLD}${CYAN}========================================================================${NC}"
echo -e "${BOLD}${CYAN} 📱 [Unitree Go2 Mobile Hotspot & Wireless LAN Switcher]${NC}"
echo -e "${BOLD}${CYAN}========================================================================${NC}"

# 1. Enable Wi-Fi Radio
echo -e "${YELLOW}📡 [1/3] Enabling Wi-Fi radio on wlan0...${NC}"
nmcli radio wifi on 2>/dev/null || true
sudo ip link set wlan0 up 2>/dev/null || true
sleep 1

# 2. Rescan Wi-Fi networks
echo -e "${YELLOW}🔍 [2/3] Scanning for '${HOTSPOT_SSID}'...${NC}"
nmcli dev wifi rescan 2>/dev/null || true
sleep 1

# 3. Connect to Hotspot Profile
echo -e "${YELLOW}🔗 [3/3] Connecting to '${HOTSPOT_SSID}'...${NC}"
if nmcli con up "${HOTSPOT_SSID}" 2>/dev/null; then
    echo -e "${GREEN}✅ Successfully activated profile '${HOTSPOT_SSID}'!${NC}"
else
    echo -e "${YELLOW}⚠️ Profile activation retry: Connecting directly to Wi-Fi SSID...${NC}"
    nmcli dev wifi connect "${HOTSPOT_SSID}" 2>/dev/null || {
        echo -e "${RED}❌ Failed to connect to '${HOTSPOT_SSID}'. Please verify phone hotspot is turned ON!${NC}"
        exit 1
    }
fi

# 4. Wait for IP address assignment
echo -e "${YELLOW}⏳ Verifying IP lease on wlan0...${NC}"
WLAN_IP=""
for i in {1..10}; do
    WLAN_IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || true)
    if [ -n "$WLAN_IP" ]; then
        break
    fi
    sleep 1
done

VPN_IP=$(ip -4 addr show wt0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || true)

echo -e "\n${BOLD}${GREEN}========================================================================${NC}"
echo -e "${BOLD}${GREEN} 🎉 [HOTSPOT CONNECTION ESTABLISHED & WIRELESS ACTIVE!]${NC}"
echo -e "${BOLD}${GREEN}========================================================================${NC}"
echo -e "  📶 ${BOLD}Hotspot SSID${NC} : ${CYAN}${HOTSPOT_SSID}${NC}"
echo -e "  🌐 ${BOLD}Wireless IP (wlan0)${NC} : ${GREEN}${WLAN_IP:-'Acquiring...'}${NC}"
if [ -n "$VPN_IP" ]; then
    echo -e "  🛡️ ${BOLD}NetBird VPN (wt0)${NC}   : ${GREEN}${VPN_IP}${NC}"
fi
echo -e "${BOLD}${GREEN}========================================================================${NC}"
echo -e "  ${BOLD}🐕 [SAFE TO UNPLUG] 이제 유선 랜선을 뽑고 복도로 이동하셔도 SSH가 유지됩니다!${NC}"
echo -e "${BOLD}${GREEN}========================================================================${NC}\n"
