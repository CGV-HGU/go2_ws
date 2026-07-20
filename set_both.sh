#!/bin/bash
# ====================================================================================
# [Multi-IP / IP Aliasing - Internet + Robot Dual Active Mode]
# ====================================================================================

CON_NAME="Wired connection 1"
IFACE="eth0"
PASS="admin"

echo "⚡ [DUAL] 인터넷(외부망) + 로봇 통신망 동시 활성화 시작..."

# 1. 기존 IP들 초기화 및 학교 유선랜 기본 IP 설정 (인터넷 가능하게)
echo "$PASS" | sudo -S nmcli connection modify "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "203.252.107.219/25" \
    ipv4.gateway "203.252.107.129" \
    ipv4.dns "8.8.8.8"

echo "$PASS" | sudo -S nmcli connection up "$CON_NAME"

# 2. 로봇 내부망용 보조 IP (192.168.123.99)를 eth0에 추가로 한 장 더 얹기 (멀티 IP)
echo "$PASS" | sudo -S ip addr add 192.168.123.99/24 dev "$IFACE"

echo ""
echo "✅ [SUCCESS] 동시 통신 설정 완료!"
echo "--------------------------------------------------------"
ip addr show "$IFACE" | grep "inet "
echo "--------------------------------------------------------"
echo "🌐 인터넷 게이트웨이: $(ip route show default | grep "$IFACE" | awk '{print $3}')"
echo "🤖 로봇 통신 주소 (DDS): 192.168.123.99"
echo "💡 이제 저랑 실시간으로 대화하면서 동시에 로봇 SLAM을 구동할 수 있습니다!"
