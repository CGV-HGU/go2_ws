# 🛠️ Scratch Scripts & Runtime Nodes Guide (`scratch/`)

이 디렉터리는 Unitree Go2의 **핵심 구동 노드, 센서 브릿지, 진단 도구 및 자율주행 엔진 스크립트**를 포함합니다.

---

## 🚀 1. 핵심 런타임 노드 (Core Runtime Nodes)

| 스크립트명 | 역할 및 기능 | 실행 명령 / 연동 |
|---|---|---|
| **`go2_autonomous_navigator.py`** | **통합 자율주행 메인 제어기**<br>• 10Hz 제어 루프, 30° 헤딩 정렬 회전, 점프 거부 가드<br>• `--mode ours`: Qwen-VL 비동기 시각 서보잉 주행<br>• `--mode pixnav`: 온보드 CUDA Checkpoint_A 신경망 주행 | `./run_escape_nav.sh`<br>`./run_pixnav.sh` |
| **`go2_localization_and_goal_recorder.py`** | **실시간 위치추정 HUD & 대화형 골 매니저**<br>• 5초 웜업 안정성 검증, 실시간 X, Y, Z, Yaw 출력<br>• 맵 경계선 근접 알림, 카메라 스냅샷 자동 저장<br>• 중복(60cm 이내) 입력 방지 필터 | `./run_localization.sh`<br>`(run_local.sh)` |
| **`go2_front_camera_publisher.py`** | **전면 초광각 RGB 카메라 퍼블리셔**<br>• H.264 RTP 멀티캐스트(`230.1.1.1:1720`) 수신 후 `/camera/front/image_raw` 토픽 30fps 퍼블리시 | `run_*.sh` 자동 구동 |
| **`go2_livo_sensor_bridge.py`** | **Unitree LIO 센서 융합 브릿지**<br>• Deskewed 4D 라이다 점군 역변환 및 바디 프레임 정합<br>• 제로 패딩 포인트 필터링 및 RTAB-Map 입력 정합 | `run_*.sh` 자동 구동 |
| **`host_bridge.py`** | **호스트 ↔ 도커 초저지연 UDP 소켓 브릿지**<br>• 0.1ms 미만 포즈 송신(포트 9091) 및 속도 명령 수신(포트 9090)<br>• CRC32 무결성 검증 | S2E 프레임워크 연동 |
| **`extract_final_golden_map.py`** | **RTAB-Map DB 골든 맵 자동 추출기**<br>• 126개 노드 전수 병합, 벽체 모폴로지 클로징<br>• `2d.png`, `2d_metadata.json`, `0833.pgm` 일괄 생성 | 맵 갱신 시 실행 |

---

## 🧪 2. 진단 및 벤치마크 도구 (Diagnostics & Benchmark)

* **`calculate_icra_metrics.py`**: ICRA 2026 Table VIII 정량 지표 계산기
* **`check_network_latency.sh`**: VPN 및 로봇 통신 RTT 지연 측정
* **`tools/pixnav_check.py` / `pixnav_live_check.py`**: PixNav CUDA 런타임 무결성 검증
* **`launchers/`**: 헤드리스/CI용 배치 런처 스크립트 모음

---

## 📦 3. 아카이브 (Legacy)

* **`legacy/`**: `record_goal_waypoints.py` 등 이전 세대 구버전 프로토타입 보관
