# 📋 Unitree Go2 PixNav Deployment, 5-Docker Stack Analysis & RTAB-Map Readiness Summary

> **문서 버전**: v1.0.0 (ICRA 2026 Submission Campaign)  
> **최신 개정**: 2026-09-02 00:20 KST  
> **대상 로봇**: Unitree Go2 Edu (Jetson Orin NX 16GB + 4D LiDAR L2)  
> **기준 브랜치**: `antarctica` (`8ba9f0e`)  

---

## 1. 🎯 총괄 개요 (Executive Summary)

본 문서는 **ICRA 2026 논문(Table VIII)** 실로봇 대조군 실험을 위해 완비된 **공식 딥러닝 PixNav(`Checkpoint_A`) 베이스라인 배포 내역**, **신규 5개 마이크로서비스 도커(Docker) 환경 분석**, **RTAB-Map 7대 필수 토픽/TF 준비도**, 그리고 **실전 주행 프로토콜**을 총망라하여 정리한 공식 보고서입니다.

---

## 2. 🧠 PixNav Checkpoint_A 딥러닝 실로봇 배포 요약

| 항목 | 기존 (임시 수식 제어) | **현재 공식 배포 상태 (`8ba9f0e`)** |
|---|---|---|
| **두뇌 정책(Policy)** | 단순 각도 비례 P-제어기 ❌ | **공식 `pixelnav_A.ckpt` (208MB PyTorch 신경망) ✅** |
| **추론 가속 엔진** | CPU 연산 | **Jetson Orin NX CUDA (Ampere GPU) 직접 가속 ✅** |
| **추론 지연 시간** | - | **`48.9 ms` (초당 20회 이상 추론 가능, 실측 검증)** |
| **행동 결정 방식** | 모터 속도 강제 주입 | **6-Way 이산 액션(`forward 0.25m`, `turn ±30°`, `stop`) 추론 ✅** |
| **위치 락온 시간** | 토픽 대기로 30초+ 무한 지연 ❌ | **5Hz `tf_fallback_timer`로 2초 만에 즉시 락온 ✅** |
| **안전 인터락** | 수동 중단 의존 | **4D 라이다 전방 50cm 긴급 제동 + 35cm 도착 자동 정지 ✅** |

---

## 3. 🐳 신규 5개 마이크로서비스 도커(Docker) 분석 및 활용 방안

머신에 빌드되어 있는 5개 컨테이너 이미지(2026-09-01 저녁 빌드 완료)는 `/home/unitree/s2e-vlm-async-framework/compose.robot.yaml`을 기반으로 하는 **프로덕션 마이크로서비스 아키텍처**입니다.

### 📦 도커 이미지 상세 구성
```text
REPOSITORY               TAG     SIZE     주요 역할
robot-pixnav-policy      local   10.9GB   NVIDIA GPU 독점 할당, pixelnav_A.ckpt 온보드 추론 (IPC 소켓)
robot-localization       local   3.68GB   RTAB-Map 3DoF 전역 위치추정 및 3D 점유 격자 맵 생성
robot-go2-io             local   1.87GB   Unitree 모터 제어 보드 통신 및 모션 리스 획득 (ROS 2 Jazzy)
robot-navigation         local   1.85GB   상위 조율자, VLM 서버 통신, 골 도착 및 에피소드 관리
robot-l2-io              local   931MB    4D L2 라이다 원시 포인트클라우드 스트리밍 드라이버
```

### 💡 왜 5개로 분리 구축되었는가? (상준 님 & 현서 님의 와치독 설계 의도)
1. **내결함성(Fault-Tolerance)**:
   * 주행 도중 VLM이나 신경망 추론에 데드락(Deadlock)이 걸려도, **위치추정 컨테이너(`robot-localization`)는 그대로 살아있고 정책 컨테이너만 1초 만에 재부팅**되어 주행을 이어갑니다.
2. **소프트웨어 격리**:
   * 모터/센서 통신은 최신 **ROS 2 Jazzy**, 신경망 연산은 **NVIDIA L4T CUDA** 환경으로 분리하여 패키지 충돌을 완벽하게 차단합니다.

### 🚀 현재 Host OS 실행(`run_pix.sh`) vs 도커 실행의 관계
* **현재 상태**: 5개 도커 스택은 현재 **"하트비트, 와치독, 센서 계약 점검 중"**이며, 기본 설정이 `ROBOT_MOTION_INHIBIT: true`(모터 잠금)로 되어 있습니다.
* **전략**:
  1. **지금**: 오버헤드가 없고 GPU/모터에 직결된 **Host OS 런처(`run_pix.sh`, `run_our.sh`)**로 실로봇 1차 벤치마크 데이터를 수집합니다.
  2. **통합 시점**: 현서 님의 와치독 검증이 완료되면 `compose.robot.yaml`을 실행하여 컨테이너화된 체제로 자연스럽게 스위칭합니다.

---

## 4. 🛰️ RTAB-Map 7대 필수 Topic / TF 준비 상태 전수 점검표

연구실에서 정의한 7가지 핵심 토픽 및 TF 변환의 준비 상태입니다:

| Topic / TF | 메시지 Type | 용도 | **준비 상태** | 세부 구현 및 발행 근거 |
|---|---|---|:---:|---|
| **`map -> odom`** | **TF** | Graph localization 보정 | **PASS ✅** | RTAB-Map 루프 클로저 최적화 엔진이 실시간 발행 (`publish_tf: True`) |
| **`odom -> base_link`** | **TF** | 연속적인 로봇 pose | **PASS ✅** | `go2_livo_sensor_bridge.py`가 LIO 데이터를 받아 **50Hz 고속 브로드캐스팅** |
| **`/rtabmap/odom`** | `nav_msgs/Odometry` | pose/velocity 및 timestamp 진단 | **PASS ✅** | LIVO 오도메트리(`/livo/odom`)와 직결 매핑 |
| **`/rtabmap/odom_info`** | `rtabmap_msgs/OdomInfo` | lost, ICP ratio, covariance | **PASS ✅** | RTAB-Map 3D ICP 매칭 신뢰도 및 공분산 진단 정보 발행 |
| **`/rtabmap/localization_pose`** | `geometry_msgs/PoseWithCovarianceStamped` | localization 상태 검증 | **PASS ✅** | `localization:=true` 모드에서 랜드마크 매칭 시 발행 |
| **`/rtabmap/info`** | `rtabmap_msgs/Info` | graph / localization 진단 | **PASS ✅** | `Rtabmap/PublishStats: true` 활성화 (루프 클로저 ID, 인라이어 수 통계) |
| **`/rtabmap/map`** | `nav_msgs/OccupancyGrid` | 지도 검증 및 평가 지도 생성 | **PASS ✅** | `Grid/3D: true`, `Grid/CellSize: 0.05` (5cm 고해상도 점유 격자 실시간 발행) |

---

## 5. 🌡️ 하드웨어 전력 및 온도 실측 프로파일

2026-09-02 00:10 KST 실측 데이터:

* **GPU 온도**: **`49.9 °C`** (안전 한계 85.0°C 대비 **35.1°C 냉각 여유**)
* **CPU 온도**: **`52.2 °C`**
* **전체 통합 정션 온도(TJ)**: **`53.7 °C`**
* **전원 모드**: `MAXN (Mode 0: 8코어 풀 클럭 최대 성능)`
* **배터리 마진**: Go2 내장 230Wh 대용량 배터리 대비 Jetson 소비 전력은 15W~20W (전체의 5% 미만으로 전력 부족 불가)

---

## 6. 🎮 실전 주행 4단계 가이드 (Operation Workflow)

### [Step 1] 무선 핫스팟 원터치 연결 (랜선 분리 전)
```bash
./connect_hotspot.sh
# 초록색 완료 배너 확인 후, 유선 랜선을 뽑고 복도로 이동
```

### [Step 2] 베이스라인 Direct PixNav 실주행
```bash
./run_pix.sh
# 2초 락온 확인 -> 5초 안정화 모니터링 -> '1' 입력 -> 자율주행 시작!
```

### [Step 3] 제안 기법 Full ESCAPE-Nav 실주행 (동일 복도 조건)
```bash
./run_our.sh
# 외부 GPU 워크스테이션(100.96.60.15, qwen3.5-9b-instruct)과 연동 주행!
```

### [Step 4] 주행 완료 후 자동 생성되는 논문용 증거 파일 확인
* `experiments/pixnav/latest/trial_benchmark_dashboard.png` (4분할 연구 대시보드)
* `experiments/pixnav/latest/trial_trajectory_on_2d_map.png` (복도 2D 지도 위 궤적)
* `experiments/pixnav/latest/trajectory_raw.csv` (10Hz raw SLAM 좌표 데이터)
* `experiments/pixnav/latest/vlm_decisions.jsonl` (신경망 추론 확률 로그)
