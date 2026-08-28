# 🛠️ [Guide 00] Phase 0: 연구실 도착 직후 5분 퀵 점검 및 4-Tier 통신 헬스체크 상세 가이드

> **작성 일자**: 2026년 8월 27일 (목요일) 21:35 KST  
> **실행 대상**: **연구실 도착 직후 첫 5분 (09:00 ~ 09:05 KST)**  
> **담당 주체**: 현장 엔지니어 (Jetson Host & Go2 Hardware Lead)  
> **문서 목적**: 로봇 센서 및 자율주행 모드를 켜기 전, 배터리, 통신, DDS, 원격 GPU 서버 및 도커 샌드박스의 물리적/네트워크 무결성을 3분 만에 전수 검증하여 당일 실패 요인을 100% 사전에 차단함.

---

## 📋 1. Phase 0 목표 및 사전 준비물

1. **사전 준비물**:
   - Unitree Go2 EDU Plus 본체 (충전기에서 분리)
   - Unitree Go2 무선 조종기 (배터리 완충 확인)
   - 조작용 노트북 (동일 Wi-Fi 또는 NetBird VPN 연결)
2. **Phase 0 완료 기준**:
   - 배터리 $\ge 90\%$, 온보드 젯슨 RAM 여유 $\ge 10\text{ GiB}$, NVMe $\ge 300\text{ GiB}$.
   - Go2 MCU 핑 $< 1.0\text{ms}$, 원격 GPU 서버 핑 $< 20\text{ms}$.
   - DDS 멀티캐스트 라우팅(`230.0.0.0/8`) 정상 등록.

---

## 💻 2. 단계별 상세 실행 절차 및 콘솔 명령어

### [Step 0-1] 온보드 젯슨 SSH 접속 및 Git 저장소 최신화
```bash
# 1. 젯슨 온보드 접속 (로컬 IP 또는 NetBird VPN IP)
ssh unitree@192.168.123.99
# 또는 NetBird IP: ssh unitree@100.96.x.x

# 2. 작업 디렉토리 이동 및 최신 변경사항 풀
cd /home/unitree/go2_ws_antarctica
git pull --rebase origin antarctica

# 3. 최신 커밋 해시 확인 (b94582f 이상이어야 함)
git log -1 --oneline
```

---

### [Step 0-2] 하드웨어 자원 및 전력/발열 퀵 진단
```bash
# 메모리 및 디스크 용량 점검
free -h
df -h /home/unitree

# 젯슨 전력 모드 및 테그라 통계 확인
sudo nvpmodel -q
tegrastats --interval 1000  # 3초 확인 후 Ctrl+C
```
* **정상 판정 기준**:
  - `Mem: available` $\ge 10\text{GiB}$
  - `/dev/nvme0n1p1` 용량 $\ge 50\text{GiB}$ 이상 여유
  - CPU/GPU 온도 $\le 55^\circ\text{C}$

---

### [Step 0-3] 4-Tier 네트워크 통신 레이턴시 3초 전수 점검
```bash
# 1. Tier 1 (Go2 메인보드 MCU) 핑 점검
ping -c 3 192.168.123.161

# 2. Tier 4 (원격 GPU VLM 서버) NetBird VPN 핑 점검
ping -c 3 100.96.60.15

# 3. 원격 Qwen3.5-9B VLM 모델 서빙 HTTP 상태 확인
curl -s --connect-timeout 3 http://100.96.60.15:8000/v1/models | grep -o '"id":"[^"]*"'
```
* **정상 콘솔 출력 예시**:
  ```text
  --- 192.168.123.161 ping statistics ---
  3 packets transmitted, 3 received, 0% packet loss, time 2004ms
  rtt min/avg/max/mdev = 0.180/0.210/0.245/0.027 ms

  --- 100.96.60.15 ping statistics ---
  3 packets transmitted, 3 received, 0% packet loss, time 2003ms
  rtt min/avg/max/mdev = 13.520/14.210/15.100/0.650 ms

  "id":"qwen3.5-9b-instruct"
  ```

---

### [Step 0-4] Go2 direct route 및 도커 샌드박스 상태 점검
```bash
# 1. Go2가 eth0와 192.168.123.99 source로 직접 연결되는지 확인
ip -4 route get 192.168.123.161

# 2. 도커 sdam_go2_container 실행 상태 확인
docker ps --filter "name=sdam_go2_container" --format "table {{.ID}}\t{{.Names}}\t{{.Status}}"
```
* **정상 콘솔 출력 예시**:
  ```text
  192.168.123.161 dev eth0 src 192.168.123.99
  CONTAINER ID   NAMES                STATUS
  8a1b2c3d4e5f   sdam_go2_container   Up 2 hours
  ```

---

## 🚨 3. Phase 0 트러블슈팅 가이드

| 증상 | 원인 | 즉각 조치 방법 |
| :--- | :--- | :--- |
| **`100.96.60.15` Ping 실패** | NetBird VPN 데몬 다운 | `sudo systemctl restart netbird` 실행 후 5초 뒤 재확인 |
| **`192.168.123.161` Ping 실패** | 이더넷 케이블 접촉 불량 | Go2 내부 이더넷 잭 재체결 및 `sudo systemctl restart NetworkManager` |
| **도커 컨테이너가 Exited 상태** | 도커 데몬 재시작 필요 | `docker start sdam_go2_container` 실행 |
| **VLM 서버 HTTP 500/Connection Refused** | 서버 측 vLLM/SGLang 미기동 | 원격 서버 관리자에게 서빙 프로세스 가동 요청 |

---

## ✅ Phase 0 통과 확인 후 다음 액션
모든 항목이 정상(PASS)으로 확인되면 즉시 **[Phase 1: 평면 3DoF 맵핑 및 골든 맵 영구 동결](01_phase1_planar_3dof_golden_mapping_and_freeze_guide.md)** 단계로 진입합니다.
