# ⚡ [Part 1] 하드웨어 & 네트워크 사전 준비 (Pre-Flight & Network Setup)

> **문서 버전**: v1.0.0 (ICRA 2026 Standard)  
> **대상 플랫폼**: Unitree Go2 EDU (Jetson Orin NX 16GB + 4D LiDAR L2)  
> **상위 허브**: [`system_bringup/README.md`](README.md)

---

## 1. 🤖 하드웨어 부팅 순서

### 1.1 배터리 및 본체 전원 인가
1. **배터리 전원 버튼**:
   * **1회 짧게 누른 후, 즉시 1회 길게(3초간)** 누릅니다.
   * 배터리 잔량 LED(4칸)가 차례로 켜진 후 Go2 로봇이 기상 시퀀스를 시작합니다.
2. **자세 확인**:
   * 로봇이 서서히 일어서며 표준 스탠드 자세(Stand Mode)를 안정적으로 유지하는지 육안 확인합니다.

### 1.2 온보드 컴퓨터(Jetson Orin NX 16GB) 접속
* **유선 이더넷 연결 시 (책상 위 작업)**:
  ```bash
  ssh unitree@192.168.123.18
  # Password: 123
  ```
* **무선 NetBird VPN 연결 시 (원격/복도 주행)**:
  ```bash
  ssh unitree@<GO2_NETBIRD_IP>
  ```

---

## 2. 📶 모바일 핫스팟 및 NetBird VPN 원터치 전환

복도 자율주행을 위해 유선 랜선을 분리하기 전, 반드시 무선 네트워크 스위칭 스크립트를 실행합니다.

```bash
cd /home/unitree/go2_ws_antarctica
./connect_hotspot.sh
```

### 2.1 스크립트 내부 자동 수행 작업
1. `wlan0` 무선 인터페이스 우선순위(Metric) 승격
2. 지정된 모바일 핫스팟 SSID로 자동 연결
3. 유선 이더넷(`eth0`) 게이트웨이 충돌 방지 라우팅 설정
4. NetBird VPN(`wt0`) 인터페이스 활성화 검증

### 2.2 사전 점검 체크리스트 (Self-Check)

터미널에서 다음 3가지 연결 상태를 반드시 확인하십시오:

```bash
# 1. 무선 인터페이스 IP 확인
ip addr show wlan0

# 2. NetBird VPN 인터페이스(wt0) 확인
ip addr show wt0

# 3. 외부 GPU VLM 서버(100.96.60.15:8000) 핑 및 API 응답 점검
ping -c 3 100.96.60.15
curl -s http://100.96.60.15:8000/v1/models | grep qwen
```

> [!IMPORTANT]
> `ping`과 `curl` 응답이 모두 정상이어야 **제안 기법(ESCAPE-Nav)**이 외부 GPU 서버와 지연 없이 통신할 수 있습니다. 만약 응답이 없다면 모바일 핫스팟의 데이터 연결 상태와 NetBird 데몬 상태(`sudo netbird status`)를 점검하십시오.
