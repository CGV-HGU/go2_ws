# 🌐 Jetson 이더넷 네트워크 설정 (`set_*.sh`) 분석 및 개선 가이드

본 문서는 Unitree Go2 로봇 PC(Jetson)의 네트워크 설정 스크립트([set_robot.sh](file:///C:/Users/hmkan/go2_ws/set_robot.sh), [set_lan.sh](file:///C:/Users/hmkan/go2_ws/set_lan.sh), [set_both.sh](file:///C:/Users/hmkan/go2_ws/set_both.sh))에 대한 정밀 분석 결과 및 **실제 현장 구동 시 발생할 수 있는 5가지 주요 문제점과 개선안**을 정리한 가이드입니다.

---

## 📌 1. 기존 네트워크 스크립트 역할 개요

| 스크립트명 | 용도 및 설정 IP | 작동 방식 |
| :--- | :--- | :--- |
| **`set_robot.sh`** | **로봇 전용 통신망** (`192.168.123.99/24`) | 로봇 SLAM 및 자율주행 주행 시 외부 인터페이스 차단 |
| **`set_lan.sh`** | **학교 유선 인터넷망** (`203.252.107.219/25`, GW: `203.252.107.129`) | 원격 코드 패키지 설치 및 깃 허브 업데이트용 |
| **`set_both.sh`** | **이중 IP 활성화 (Dual Mode)** | 인터넷 접속 및 로봇 DDS 통신 동시 사용 |

---

## ⚠️ 2. 현장 구동 시 발생 가능한 5가지 주요 문제점

### 1️⃣ NetworkManager 커넥션 이름 하드코딩 문제 🔴 (가장 흔한 오류)
- **현상**: `CON_NAME="Wired connection 1"` 하드코딩
- **문제점**: Ubuntu의 언어 설정이 한글일 경우 커넥션 명칭이 `"유선 연결 1"`로 바뀌거나, OS 재설치/네트워크 드라이버에 따라 `"eth0"`, `"netplan-eth0"` 등으로 변경됩니다.
- **결과**: `nmcli connection modify` 실행 시 `Error: Unknown connection 'Wired connection 1'` 이 발생하며 스크립트 전체가 실패합니다.

### 2️⃣ `set_both.sh`에서 로봇 통신 IP(192.168.123.99) 자동 증발 문제 🟠
- **현상**: `sudo ip addr add 192.168.123.99/24 dev eth0`
- **문제점**: `ip addr add`는 커널 레벨의 임시 주입 방식입니다. NetworkManager가 주기적으로 상태를 재점검하거나 유선 랜선이 재연결되면 **임시 추가된 로봇 IP가 자동으로 삭제**됩니다.
- **결과**: 주행 중 로봇 통신(DDS)이 끊겨 SLAM 노드가 다운될 수 있습니다.

### 3️⃣ SSH 원격 접속 중 세션 절단(Disconnect) 위험 🟠
- **현상**: `sudo ip addr flush dev eth0`
- **문제점**: SSH를 통해 유선 이더넷(`eth0`)으로 로봇 PC에 접속해 있는 상태에서 `ip addr flush`가 실행되면 **인터페이스 IP가 순간적으로 완전히 제거되어 SSH 접속이 강제 종료**됩니다.

### 4️⃣ Sudo 비밀번호 하드코딩 및 보안 약점 🟡
- **현상**: `PASS="admin"` 하드코딩 및 `echo "$PASS" | sudo -S`
- **문제점**: 비밀번호가 평문으로 저장되어 보안에 취약하며, 로봇 계정 비밀번호가 변경되거나 다른 계정인 경우 모든 network 설정 명령이 거부됩니다.

### 5️⃣ 학교 유선랜 고정 IP 환경 의존성 🟡
- **현상**: `203.252.107.219/25` (GW: `203.252.107.129`)
- **문제점**: 학교 고정 IP 환경이 아닌 외부 연구실, 공유기(DHCP), 홈 네트워크 환경으로 이동 시 인터넷 연결이 동작하지 않습니다.

---

## 🛠️ 3. 개선된 네트워크 설정 스크립트 (권장안)

아래 개선안은 **eth0에 바인딩된 실제 커넥션 이름을 동적으로 자동 감지**하고, **Root 권한 체크 및 nmcli 정석 멀티 IP 등록 방식**을 적용하여 안정성을 극대화한 스크립트입니다.

### 🟢 1) `set_both.sh` (인터넷 + 로봇 동시 접속 개선안)
```bash
#!/bin/bash
# ====================================================================================
# [Multi-IP Dual Active Mode - Dynamic Connection Name Detection]
# ====================================================================================

IFACE="eth0"

# Root 권한 확인
if [ "$EUID" -ne 0 ]; then
  echo "❌ Error: sudo 권한으로 실행해야 합니다. (예: sudo ./set_both.sh)"
  exit 1
fi

# eth0에 연결된 활성화된 NetworkManager 커넥션 이름 자동 추출 (한글/영문 대응)
CON_NAME=$(nmcli -t -f NAME,DEVICE connection show --active | grep ":$IFACE" | cut -d: -f1)

if [ -z "$CON_NAME" ]; then
    CON_NAME=$(nmcli -t -f NAME,DEVICE connection show | grep ":$IFACE" | head -n1 | cut -d: -f1)
fi

if [ -z "$CON_NAME" ]; then
    echo "❌ Error: $IFACE 인터페이스에 할당된 NetworkManager 프로필을 찾을 수 없습니다."
    exit 1
fi

echo "⚡ [DUAL] 감지된 프로필 '$CON_NAME' ($IFACE)에 인터넷 + 로봇 통신망 동시 적용 중..."

# nmcli 정석 방식을 통한 멀티 IP 설정 (재시작되어도 유지됨)
nmcli connection modify "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "203.252.107.219/25, 192.168.123.99/24" \
    ipv4.gateway "203.252.107.129" \
    ipv4.dns "8.8.8.8"

nmcli connection up "$CON_NAME"

echo ""
echo "✅ [SUCCESS] 동시 통신 설정 완료!"
echo "--------------------------------------------------------"
ip addr show "$IFACE" | grep "inet "
echo "--------------------------------------------------------"
```

---

### 🤖 2) `set_robot.sh` (로봇 주행 전용 통신망 개선안)
```bash
#!/bin/bash
# ====================================================================================
# [Robot Dedicated Network - 192.168.123.99/24]
# ====================================================================================

IFACE="eth0"

if [ "$EUID" -ne 0 ]; then
  echo "❌ Error: sudo 권한으로 실행해야 합니다. (예: sudo ./set_robot.sh)"
  exit 1
fi

CON_NAME=$(nmcli -t -f NAME,DEVICE connection show --active | grep ":$IFACE" | cut -d: -f1)
if [ -z "$CON_NAME" ]; then
    CON_NAME=$(nmcli -t -f NAME,DEVICE connection show | grep ":$IFACE" | head -n1 | cut -d: -f1)
fi

echo "🤖 [ROBOT] 감지된 프로필 '$CON_NAME' ($IFACE)에 로봇 전용 통신망 설정 적용 중..."

nmcli connection modify "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "192.168.123.99/24" \
    ipv4.gateway "" \
    ipv4.dns ""

nmcli connection up "$CON_NAME"

echo ""
echo "✅ 완료 - 로봇 통신망 적용 완료:"
ip addr show "$IFACE" | grep "inet "
```

---

## 💡 4. 체크리스트 및 운용 팁

1. **스크립트 실행 방식**:
   - 비밀번호 하드코딩 제거 후 **`sudo ./set_both.sh`** 와 같이 실행합니다.
2. **IP 확인 명령어**:
   - `ip addr show eth0` 실행 시 `inet 203.252.107.219`와 `inet 192.168.123.99`가 동시에 보이면 정상적으로 동시 통신 모드가 동작하는 상태입니다.
3. **DDS 통신 확인**:
   - 로봇 통신망 적용 후 `ros2 topic list`로 로봇 토픽이 수신되는지 확인합니다.
