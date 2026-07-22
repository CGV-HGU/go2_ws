#!/bin/bash
# ============================================================
# 로봇 자체 통신망 설정 적용 (자동 포트 감지)
# IP: 192.168.123.99 / 255.255.255.0
# ============================================================

# 랜선이 연결된 이더넷 포트 및 NetworkManager 연결 이름 자동 탐색
DEV_INFO=$(nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device | grep ":ethernet:connected" | head -n1)

if [ -n "$DEV_INFO" ]; then
    IFACE=$(echo "$DEV_INFO" | cut -d: -f1)
    CON_NAME=$(echo "$DEV_INFO" | cut -d: -f4)
else
    IFACE=$(nmcli -t -f DEVICE,TYPE device | grep ":ethernet" | head -n1 | cut -d: -f1)
    CON_NAME=$(nmcli -t -f NAME,DEVICE connection show | grep ":$IFACE" | cut -d: -f1 | head -n1)
fi

[ -z "$CON_NAME" ] && CON_NAME=$(nmcli -t -f NAME,TYPE connection show | grep ":802-3-ethernet" | head -n1 | cut -d: -f1)
[ -z "$IFACE" ] && IFACE="eth0"
[ -z "$CON_NAME" ] && CON_NAME="Wired connection 1"

PASS="admin"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🤖 [ROBOT] 로봇 통신망 설정 적용 중... ($IFACE / $CON_NAME)"

# NetworkManager 프로필 수정 및 재시작
echo "$PASS" | sudo -S nmcli connection modify "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "192.168.123.99/24" \
    ipv4.gateway "" \
    ipv4.dns ""

echo "$PASS" | sudo -S nmcli connection up "$CON_NAME"

# 물리 인터페이스에 강제 IP 주입 (NM 연결 지연 우회)
echo "$PASS" | sudo -S ip addr flush dev "$IFACE"
echo "$PASS" | sudo -S ip addr add 192.168.123.99/24 dev "$IFACE"
echo "$PASS" | sudo -S ip link set "$IFACE" up

# CycloneDDS 네트워크 인터페이스 설정 동기화
DDS_XML="$SCRIPT_DIR/cyclonedds.xml"
if [ -f "$DDS_XML" ]; then
    sed -i -E "s|<NetworkInterfaceAddress>.*</NetworkInterfaceAddress>|<NetworkInterfaceAddress>$IFACE</NetworkInterfaceAddress>|g" "$DDS_XML"
    echo "🤖 CycloneDDS 인터페이스가 '$IFACE'로 동기화되었습니다."
fi

echo ""
echo "✅ 완료 - 로봇 통신망 설정 강제 적용 ($IFACE):"
ip addr show "$IFACE" | grep "inet "

