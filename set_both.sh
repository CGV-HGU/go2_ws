#!/bin/bash
# ====================================================================================
# [Multi-IP / IP Aliasing - Internet + Robot Dual Active Mode]
# ====================================================================================

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

echo "⚡ [DUAL] 인터넷(외부망) + 로봇 통신망 동시 활성화 시작..."
echo "🔍 자동 감지된 포트: $IFACE (프로필: $CON_NAME)"

# 1. NetworkManager 프로필에 인터넷 IP + 로봇 IP 동시 등록 (우분투 설정창에도 표기됨)
echo "$PASS" | sudo -S nmcli connection modify "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "203.252.107.219/25, 192.168.123.99/24" \
    ipv4.gateway "203.252.107.129" \
    ipv4.dns "8.8.8.8"

echo "$PASS" | sudo -S nmcli connection up "$CON_NAME"

# 2. 커널 물리 인터페이스에도 보조 IP 보장 주입
echo "$PASS" | sudo -S ip addr add 192.168.123.99/24 dev "$IFACE" 2>/dev/null || true

# 3. CycloneDDS 네트워크 인터페이스 설정 동기화
DDS_XML="$SCRIPT_DIR/cyclonedds.xml"
if [ -f "$DDS_XML" ]; then
    sed -i -E "s|<NetworkInterfaceAddress>.*</NetworkInterfaceAddress>|<NetworkInterfaceAddress>$IFACE</NetworkInterfaceAddress>|g" "$DDS_XML"
    echo "🤖 CycloneDDS 인터페이스가 '$IFACE'로 동기화되었습니다."
fi

echo ""
echo "✅ [SUCCESS] 동시 통신 설정 완료!"
echo "--------------------------------------------------------"
ip addr show "$IFACE" | grep "inet "
echo "--------------------------------------------------------"
echo "🌐 인터넷 게이트웨이: $(ip route show default | grep "$IFACE" | awk '{print $3}')"
echo "🤖 로봇 통신 주소 (DDS): 192.168.123.99"
echo "💡 인터넷과 로봇 통신이 모두 활성화되었습니다!"

