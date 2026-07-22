#!/bin/bash
# ============================================================
# 학교 유선랜 (인터넷) 설정 적용 (자동 포트 감지)
# IP: 203.252.107.219 / 255.255.255.128 / GW: 203.252.107.129
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

echo "🌐 [LAN] 학교 인터넷 설정 적용 중... ($IFACE / $CON_NAME)"

# NetworkManager 프로필 수정 및 재시작
echo "$PASS" | sudo -S nmcli connection modify "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "203.252.107.219/25" \
    ipv4.gateway "203.252.107.129" \
    ipv4.dns "8.8.8.8"

echo "$PASS" | sudo -S nmcli connection up "$CON_NAME"

# 물리 인터페이스에 강제 IP 주입 (NM 연결 지연 우회)
echo "$PASS" | sudo -S ip addr flush dev "$IFACE"
echo "$PASS" | sudo -S ip addr add 203.252.107.219/25 dev "$IFACE"
echo "$PASS" | sudo -S ip route add default via 203.252.107.129 dev "$IFACE" || true
echo "$PASS" | sudo -S ip link set "$IFACE" up

echo ""
echo "✅ 완료 - 학교 인터넷 설정 강제 적용 ($IFACE):"
ip addr show "$IFACE" | grep "inet "
echo "게이트웨이: $(ip route show default | grep "$IFACE" | awk '{print $3}')"

