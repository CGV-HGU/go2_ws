# 📊 [Docker Status Dashboard] 온보드 도커 자율주행 상시 상태 점검표

> **최종 검증 일시**: 2026-08-21 (실시간 자동 갱신 지원)  
> **대상 컨테이너**: `sdam_go2_container` (Ubuntu 24.04 LTS / ROS 2 Jazzy ARM64 / CPU Mode)  
> **자동 점검 스크립트**: [`scratch/check_docker_status_dashboard.py`](file:///home/unitree/go2_ws_antarctica/scratch/check_docker_status_dashboard.py)  
> **종합 판정**: 🟢 **8/8 ALL PASS (Production-Ready 100% 준비 완료)**

---

## 📌 1. 도커 8대 서브시스템 상시 점검 대시보드 (Live Dashboard)

| 번호 | 점검 서브시스템 | 실측 상태 / 세부 스펙 | 판정 | 기술적 검증 근거 |
| :---: | :--- | :--- | :---: | :--- |
| **1** | **컨테이너 런타임 환경** | `arm64v8/ros:jazzy-ros-base` / Noble 24.04 | 🟢 **PASS** | `/etc/bash.bashrc` 자동 로드 및 `unless-stopped` 적용 |
| **2** | **S2E 단위/계약 테스트** | `59 passed in 0.36s` (`pytest`) | 🟢 **PASS** | SE(2) 변환, PoseBuffer, LatestStore 59개 전원 통과 |
| **3** | **VLM 원격 REST API** | `100.96.60.15:8000` / `qwen3.8-27b-instruct` | 🟢 **PASS** | NetBird VPN RTT 32.7ms 및 `/v1/models` 정상 응답 |
| **4** | **멀티모달 시각 추론** | 1280x720 RGB 이미지 ➔ 서브골 `[640, 600]` | 🟢 **PASS** | 실제 시각 이미지 기반 바닥 픽셀 목표점 산출 성공 |
| **5** | **이종 UDP 소켓 브릿지** | Magic `0x53324501` + CRC32 (지연 0.14ms) | 🟢 **PASS** | Port 9091(Pose 62B) / Port 9090(CmdVel 54B) 통과 |
| **6** | **S2E 풀루프 드라이런** | PoseBuffer ➔ VLM 결정 ➔ $v_x=0.30\text{ m/s}$ | 🟢 **PASS** | 엔드투엔드 비동기 제어 루프 무결성 검증 |
| **7** | **정체감지 & 능동회복** | Kinematic Stall ➔ $v_x=0$ 차단 ➔ $w_z=0.40$ 선회 | 🟢 **PASS** | 물리적 벽면 충돌 방어 및 360° 선회 탈출 가드 |
| **8** | **8대 노드 런치 그래프** | `s2e_vlm_bringup` 8개 프로세스 준비 완료 | 🟢 **PASS** | `robot_side.launch.py` 및 Supervisor 안전 락 |

---

## 🚀 2. 상시 1-Click 실시간 점검 방법 (언제든 3초 만에 대시보드 갱신)

터미널에서 언제든 아래 **단 1줄의 명령어**를 실행하면 8대 영역을 실시간으로 3초 만에 전수 재점검하고 대시보드를 출력합니다:

```bash
# [Host 터미널에서 실행]
python3 /home/unitree/go2_ws_antarctica/scratch/check_docker_status_dashboard.py
```

---

## 🛠️ 3. 개별 세부 진단 스크립트 모음 (Deep Diagnostics)

특정 컴포넌트만 정밀하게 단독 테스트하고 싶을 때 사용하는 도커 전용 진단 스크립트입니다:

| 스크립트 파일 | 실행 명령어 | 점검 목적 |
| :--- | :--- | :--- |
| **VLM 서버 연결 진단** | `python3 scratch/test_vlm_server_connection.py` | Qwen3.8-27B 텍스트 추론 및 NetBird RTT 측정 |
| **멀티모달 시각 추론** | `docker exec sdam_go2_container python3 /workspace/go2_ws_antarctica/scratch/test_docker_real_image_vlm.py` | 720p 사진 기반 픽셀 목표점 산출 |
| **50Hz 스트레스 부하** | `python3 scratch/test_docker_50hz_stress.py` | 500개 패킷 0% 손실 및 0.1ms 지연 측정 |
| **S2E 풀루프 드라이런** | `docker exec sdam_go2_container python3 /workspace/go2_ws_antarctica/scratch/test_docker_s2e_dryrun.py` | PoseBuffer + VLM + 속도 생성 전수 테스트 |
| **충돌/정체 방어 가드** | `docker exec sdam_go2_container python3 /workspace/go2_ws_antarctica/scratch/test_docker_stall_and_recovery.py` | Kinematic Stall 시 $v_x=0$ 차단 및 회복 선회 |
