#!/bin/bash
# ============================================================
# 로봇 자체 통신망 설정 적용 (물리 강제 부여 버전)
# IP: 192.168.123.99 / 255.255.255.0
# ============================================================

CON_NAME="Wired connection 1"
IFACE="eth0"
PASS="admin"

echo "🤖 [ROBOT] 로봇 통신망 설정 적용 중..."

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

echo ""
echo "✅ 완료 - 로봇 통신망 설정 강제 적용:"
ip addr show "$IFACE" | grep "inet "
