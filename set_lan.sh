#!/bin/bash
# ============================================================
# 학교 유선랜 (인터넷) 설정 적용 (물리 강제 부여 버전)
# IP: 203.252.107.219 / 255.255.255.128 / GW: 203.252.107.129
# ============================================================

CON_NAME="Wired connection 1"
IFACE="eth0"
PASS="admin"

echo "🌐 [LAN] 학교 인터넷 설정 적용 중..."

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
echo "✅ 완료 - 학교 인터넷 설정 강제 적용:"
ip addr show "$IFACE" | grep "inet "
echo "게이트웨이: $(ip route show default | grep "$IFACE" | awk '{print $3}')"
