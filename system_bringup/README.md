# 🚀 Unitree Go2 ESCAPE-Nav & PixNav 시스템 브링업 마스터 허브

> **문서 버전**: v1.0.0 (ICRA 2026 Production Standard)  
> **대상 플랫폼**: Unitree Go2 EDU (Jetson Orin NX 16GB + 4D LiDAR L2 + Front RGB Camera)  
> **주요 독자**: 연구팀원, 로봇 운영자, 벤치마크 평가자  
> **기준 브랜치**: `antarctica`

---

## 📌 마스터 가이드 모듈 구성

본 `system_bringup/` 디렉터리는 실물 사족로봇 Go2의 부팅부터 벤치마크 평가, 그리고 딥러닝 정책 분석까지 **역할별로 특화된 4대 독립 모듈**로 체계화되어 있습니다:

| 모듈 번호 | 문서명 | 세부 내용 및 핵심 목적 |
|:---:|---|---|
| **Part 1** | [⚡ `01_preflight_and_network_setup.md`](01_preflight_and_network_setup.md) | **하드웨어 부팅 & 네트워크 전환**<br>• Go2 본체 전원 인가 및 Stand Mode 확인<br>• `./connect_hotspot.sh` 원터치 무선랜 및 NetBird VPN(`wt0`) 전환<br>• 외부 VLM GPU 서버(`100.96.60.15:8000`) 핑/API 헬스체크 |
| **Part 2** | [📋 `02_five_step_operational_sop.md`](02_five_step_operational_sop.md) | **엔드투엔드 5단계 운영 SOP**<br>• 1단계: 3D/2D 복도 매핑 (`./run_mapping.sh`)<br>• 2단계: 2D 골든 맵 추출 (`extract_final_golden_map.py`)<br>• 3단계: 위치추정 HUD & 엔터키 골 등록 (`./run_localization.sh`)<br>• 4단계: 자율주행 실행 (`./run_escape_nav.sh` vs `./run_pixnav.sh`)<br>• 5단계: 결과 CSV 및 대시보드 그래프 확인 |
| **Part 3** | [🧠 `03_pixnav_policy_and_slam_cheat_key_synthesis.md`](03_pixnav_policy_and_slam_cheat_key_synthesis.md) | **PixNav 실체 분석 & SLAM 치트키 논쟁**<br>• Checkpoint_A 4채널 ResNet-18 + 트랜스포머 디코더 구조<br>• 3D 좌표의 2D 픽셀 $u$ 투영 공식 및 $v=360$ 중앙 고정 이유<br>• 등 뒤($180^\circ$) $u=160$ 억지 클램핑과 25° 회전 가드의 실체<br>• 100% 유클리드 거리 도착 판정 (비전 개입 0%)<br>• "SLAM 치트키" vs ESCAPE-Nav 비교 및 ICRA 논문 기여도 |
| **Part 4** | [🛠️ `04_failsafes_and_troubleshooting.md`](04_failsafes_and_troubleshooting.md) | **안전 가드 & 현장 트러블슈팅**<br>• 대칭 복도에서의 20m 텔레포트 오인 락온(Perceptual Aliasing) 방지<br>• `/localization_pose` 토픽 네임스페이스 점검 가이드<br>• 25° 강제 회전 헛바퀴 대처법 & 비정상 골 좌표 초기화<br>• 키보드/조종기/명령어 비상 정지(E-Stop) 프로토콜 |

---

## 🏗️ 시스템 아키텍처 개요

```text
                                [Unitree Go2 EDU 실물 로봇]
                                             │
      ┌──────────────────────────────────────┼──────────────────────────────────────┐
      ▼                                      ▼                                      ▼
[4D 라이다 L2 & IMU]                  [전면 광각 RGB 카메라]                 [Jetson Orin NX 16GB]
  • /livo/cloud (10Hz 점군)            • /camera/front/image_raw (30fps)     • RTAB-Map 3DoF SLAM (1Hz)
  • /livo/odom  (50Hz 오도메트리)       • H.264 RTP 멀티캐스트 수신            • Checkpoint_A 온보드 CUDA (54ms)
      │                                      │                                      │
      └──────────────────────────────────────┼──────────────────────────────────────┘
                                             │
                                             ▼
                        [통합 자율주행 제어기 (go2_autonomous_navigator.py)]
                         ├── [Proposed: ESCAPE-Nav] ──▶ 외부 GPU 서버 (Qwen-VL 9B)
                         └── [Baseline: Direct PixNav] ──▶ 온보드 PyTorch CUDA 직접 실행
```

---

## 🎯 4대 표준 실행 스크립트 퀵 레퍼런스

```bash
cd /home/unitree/go2_ws_antarctica

# 1. 3D/2D 복도 매핑
./run_mapping.sh          # 헤드리스 매핑 (GUI 시각화 필요 시 --view 옵션 추가)

# 2. 실시간 위치추정 HUD & 대화형 골 등록
./run_localization.sh     # 5초 웜업 후 [ENTER] 누르면 현재 위치 및 카메라 사진을 골로 자동 저장

# 3. 제안 기법: Full ESCAPE-Nav 자율주행
./run_escape_nav.sh 1     # Goal 1번으로 Qwen-VL 비전 서보잉 주행

# 4. 베이스라인: Direct PixNav 온보드 CUDA 자율주행
./run_pixnav.sh 1         # Goal 1번으로 Checkpoint_A 신경망 고속 주행 (~54ms)
```

---

## 🛑 비상 조치 (Emergency Stop)

* **키보드 즉시 정지**: 실행 중인 터미널에서 **`Ctrl + C`**를 누르면 안전 트랩이 발동하여 로봇 선속도/각속도를 $0.0$으로 리셋하고 모터를 안전하게 정지시킵니다.
* **조종기 강제 락**: Unitree 무선 조종기의 비상 정지 버튼(L2+B)을 눌러 모터 하드웨어 락을 겁니다.
* **프로세스 긴급 사살**:
  ```bash
  pkill -9 -f go2_autonomous_navigator
  pkill -9 -f rtabmap
  ```
