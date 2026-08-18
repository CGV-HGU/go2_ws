# 📑 [07] ESCAPE-Nav 실물 로봇 실내 실증 주행 마스터 계획서 검증 및 종합 팩트체크 보고서

> **문서 소유자**: **민석 (Minseok - Hardware, Sensor & Deployment Lead)**  
> **논문 공식 명칭**: **`ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation` (ICRA 2026)**  
> **문서 목적**: [`docs/05_real_robot_indoor_testing_protocol.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/05_real_robot_indoor_testing_protocol.md) 마스터 계획서의 **이종 ROS 2 통신 구조, Jetson Orin NX 16GB 자원 배분, No-Prior-Metric-Map 학술 정합성, Table VIII 5대 시나리오 및 정량 수식($T^\dagger$, DRS, FBR)에 대한 종합 팩트체크 및 4대 실무 개선 권고 보고서**입니다.

---

## 📌 목차 (Table of Contents)
1. [하이브리드 아키텍처 및 이종 ROS 2 통신 구조 검증](#1-하이브리드-아키텍처-및-이종-ros-2-통신-구조-검증)
2. [온보드 컴퓨팅 자원 및 메모리 할당 타당성 평가](#2-온보드-컴퓨팅-자원-및-메모리-할당-타당성-평가)
3. [학술적 프레이밍과 SLAM/LIVO 역할의 논리적 정합성 검증](#3-학술적-프레이밍과-slamlivo-역할의-논리적-정합성-검증)
4. [ICRA 2026 Table VIII 5대 주행 시나리오 및 실행 매뉴얼 팩트체크](#4-icra-2026-table-viii-5대-주행-시나리오-및-실행-매뉴얼-팩트체크)
5. [정량적 평가 지표 및 수식 수립의 학술적 엄밀성 검증](#5-정량적-평가-지표-및-수식-수립의-학술적-엄밀성-검증)
6. [결론 및 현장 실증 개선 권고 사항](#6-결론-및-현장-실증-개선-권고-사항)

---

## 🏗️ 1. 하이브리드 아키텍처 및 이종 ROS 2 통신 구조 검증

문서 `05_real_robot_indoor_testing_protocol.md`(이하 05번 마스터 계획서)에서 제안된 하이브리드 아키텍처는 호스트 운영체제인 **Ubuntu 20.04(ROS 2 Foxy)**와 도커 컨테이너 환경인 **Ubuntu 24.04(ROS 2 Jazzy)**를 이원화하여 연동하는 제어 구조를 명시하고 있습니다. 이러한 이종 ROS 2 배포판 간의 분산 통신 환경에서 직면하는 핵심 기술적 병목과 이에 대한 UDP 소켓 브릿지 설계의 타당성은 미들웨어 호환성 관점에서 명확하게 증명됩니다.

ROS 2 Foxy와 ROS 2 Jazzy는 기본 데이터 분배 서비스(DDS) 미들웨어의 내부 버전과 메시지 직렬화 패러다임에서 상당한 기술적 격차를 보입니다. Foxy 배포판은 FastDDS 2.0.x 계열을 탑재한 반면 Jazzy 배포판은 FastDDS 2.14.0 이상을 채택하고 있어, 동일한 DDS Domain ID를 공유하더라도 노드 간 자동 발견(Participant Discovery) 실패나 메시지 역직렬화(Deserialization) 오류가 발생할 가능성이 매우 높습니다.

05번 마스터 계획서는 이러한 RMW(ROS Middleware) 계층의 구조적 비호환성과 네트워크 탐색 오버헤드를 완벽히 우회하기 위하여 복잡한 ROS 2 토픽 브릿지 대신 **루프백 네트워크 기반의 초저지연 UDP 소켓 브릿지(`host_bridge.py` 및 `docker_bridge.py`)를 적용**하였습니다.

| 시스템 구분 | 호스트 OS 계층 (Host OS) | 도커 컨테이너 계층 (Docker Container) | 원격 GPU 서버 계층 (Remote Server) |
| :--- | :--- | :--- | :--- |
| **운영체제 및 환경** | Ubuntu 20.04 / ROS 2 Foxy | Ubuntu 24.04 / ROS 2 Jazzy | Ubuntu 22.04 LTS / CUDA 12.x |
| **주요 담당 역할** | RTAB-Map LIVO ($50\text{Hz}$ 오도메트리),<br/>관절 제어 (`SportClient.Move`) | S2E 비동기 궤적 생성 노드 ($10\text{Hz}$) | Qwen3-VL 32B VLM<br/>($1\sim2\text{Hz}$ 비주얼 메모리 그래프) |
| **데이터 패킷 규격** | 48-byte CmdVel 수신 | 56-byte Pose 수신 | 비주얼 메모리 그래프 프레임 전송 |
| **통신 프로토콜** | Local UDP Socket (`127.0.0.1:9090`) | Local UDP Socket (`127.0.0.1:9091`) | High-speed Async Network / VPN |
| **목표 전송 지연** | $\le 0.1\text{ms}$ | $\le 0.1\text{ms}$ | $100\sim500\text{ms}$ (비동기 허용) |

이러한 로컬 UDP 루프백 통신 구성은 ROS 2 Discovery Protocol의 패킷 교환을 생략하게 만듦으로써 지연 시간을 $0.1\text{ms}$ 수준으로 단축시킵니다. 또한 56-byte 포즈 데이터와 48-byte 속도 명령어를 C-struct 형태의 바이너리 버퍼로 이진화하여 바인딩함으로써 Python 3.8 환경과 Python 3.12 환경 간의 라이브러리 파편화 문제를 원천 차단합니다. 

결과적으로 호스트 제어기의 $50\text{Hz}$ 제어 주기와 S2E 노드의 $10\text{Hz}$ 궤적 생성 주기가 DDS 파이프라인의 차단(Blocking) 현상 없이 안정적으로 상호작용할 수 있도록 보장하므로, **계획서의 하이브리드 아키텍처 설계는 기술적으로 매우 타당**합니다.

---

## 💾 2. 온보드 컴퓨팅 자원 및 메모리 할당 타당성 평가

NVIDIA Jetson Orin NX (16GB) 온보드 모듈의 통합 LPDDR5 메모리 환경에서 구동되는 프로세스별 자원 점유율과 연산 부하를 검증한 결과, 계획서의 자원 배분 전략은 하드웨어 제약 조건과 완벽히 부합함이 확인됩니다.

Jetson Orin NX 16GB 장치는 CPU와 GPU가 물리적 메모리를 경계 없이 공유하는 통합 메모리(UMA) 구조를 가집니다. 시스템 가동 시 예상되는 메인 메모리 점유 현황은 아래 표와 같이 구체적으로 정리됩니다.

| 프로세스 및 시스템 요소 | 예상 메모리 점유량 | 비고 및 시스템 영향 |
| :--- | :---: | :--- |
| **JetPack 5.1.x OS 및 호스트 커널** | $\sim 1.2\text{GB}$ | 기본 시스템 드라이버 및 백그라운드 프로세스 |
| **ROS 2 Foxy 미들웨어 & DDS 버퍼** | $\sim 0.3\text{GB}$ | RGB, LiDAR, IMU 입력 토픽 센서 버퍼 |
| **RTAB-Map LIVO ($50\text{Hz}$ 융합)** | $\sim 1.0\text{GB}$ | 점군 전처리, 키프레임 DB, 3D Pose 적분 |
| **S2E 궤적 생성 노드 ($10\text{Hz}$)** | $\sim 1.5\text{GB}$ | PyTorch 기반 온보드 경량 추론 연산 |
| **최종 여유 메모리 (Buffer)** | **$\sim 12.0\text{GB}$** | **시스템 크래시 없는 실시간 안정 구동 범위 확보** |

01번 시스템 아키텍처 문서 및 05번 마스터 계획서 분석에 따르면, 32B 파라미터 규모의 시각-언어 모델인 Qwen3-VL 32B는 INT4 수준으로 양자화하더라도 최소 $16\sim18\text{GB}$의 VRAM/RAM 공간을 요구합니다. 만약 이 대형 모델을 Jetson Orin NX 온보드 환경에 올릴 경우, 이미 점유된 $2.5\text{GB}$의 필수 프로세스 메모리와 합쳐져 가용 메모리 한계를 즉시 초과하게 되며, 결국 리눅스 커널의 OOM(Out of Memory) 킬러에 의해 자율주행 프로세스가 강제 종료됩니다.

05번 마스터 계획서는 무거운 32B VLM 추론을 원격 GPU 서버(RTX Pro 6000)에 오프라인 및 비동기 방식으로 위임하고, 온보드 Jetson에는 $50\text{Hz}$ RTAB-Map LIVO 실시간 오도메트리와 $10\text{Hz}$ S2E 궤적 생성 노드, $50\text{Hz}$ 3-DOF 제어기만을 전담 배치하였습니다. 이는 **하드웨어 한계를 엄밀하게 반영하여 실시간 제어 안정성을 담보한 현실적이고 우수한 자원 배분 전략**으로 판단됩니다.

---

## 🎯 3. 학술적 프레이밍과 SLAM/LIVO 역할의 논리적 정합성 검증

ESCAPE-Nav 논문의 정체성은 사전 구축된 정밀 3D 지오메트리 맵 없이 비주얼 메모리 그래프만으로 목표점을 찾아가는 **"No-Prior-Metric-Map"** 항법에 기반합니다. 이에 반해 05번 마스터 계획서의 [3단계] 절차에 "복도 1바퀴 수동 주행 및 3D 점군 지도 저장(`~/.ros/rtabmap.db`)" 과정이 포함되어 있어 학술적 프레이밍과의 논리적 충돌 여부를 명확히 판별할 필요가 있습니다.

팩트체크 분석 결과, 01번 가이드 문서에 명시된 역할 분리 기준에 의하여 모순 없이 학술적 정합성이 유지되고 있음이 입증됩니다. 온라인 자율 탐색 과정에서 로봇은 사전 맵 기반의 전역 경로 계획을 전혀 수행하지 않으며, **RTAB-Map LIVO는 단순히 $50\text{Hz}$ 실시간 3D 위치 추적기(Pure Odometry Logger)로만 동작**합니다.

저장된 3D 지도 데이터베이스(`rtabmap.db`)는 주행이 완주된 이후, 오프라인 상에서 이동 경로의 정규화 완주 시간($T^\dagger$)을 산출하기 위한 **이론적 최단 경로 거리($l_i$) 기준선(Ground Truth)을 계산하는 목적으로만 제한적으로 활용**됩니다.

다만 실증 주행 시 RTAB-Map LIVO 노드가 기존 저장된 지도 기반의 위치 재인식(Global Relocalization) 신호를 S2E 노드로 잘못 전달하지 않도록, `go2_rtabmap.launch.py` 실행 시 순수 오도메트리 전용 파라미터가 명확히 적용되어 있는지 확인하는 항목을 실행 체크리스트에 보완할 필요가 있습니다.

---

## 📊 4. ICRA 2026 Table VIII 5대 주행 시나리오 및 실행 매뉴얼 팩트체크

05번 마스터 계획서 4절에 기재된 실내 Table VIII 5대 주행 시나리오 및 5절의 1-Click Rosbag 자동 로깅 체계는 ICRA 2026 논문 초안의 정량적 평가 양식을 완벽히 준수하고 있습니다.

| 시나리오 명칭 | 검증 분류 | 핵심 평가 목적 및 동작 메커니즘 | 목표 반복 횟수 |
| :--- | :---: | :--- | :---: |
| **1. Dead-end room** | Core | 막다른 공간 진입 시 $360^\circ$ Active Sweep을 거쳐 후방 $180^\circ$ 출구 탈출 | 5회 |
| **2. Blocked goal direction** | Core | 목표 방향 전방 장애물 차단 시 측면 우회로 탐색 및 능동 선회 | 5회 |
| **3. Repeated corridor** | Core | 반복 복도 구조에서 Directional Memory로 과거 실패 에지 재진입 억제 | 5회 |
| **4. Active-view recovery** | Deployment | 주행 정체(Stagnation) 감지 시 능동 Yaw 회전으로 신규 브랜치 확장 | 5회 |
| **5. Dynamic obstacle** | Deployment | $1.2\text{m/s}$ 이동 보행자 접근 시 실시간 비동기 재계획 및 감속 우회 | 5회 |

총 20회의 수집 데이터를 수집하기 위해 마련된 현장 4단계 온보드 실행 매뉴얼은 실무 적용성이 매우 높습니다:
1. **1단계**: `ros-foxy-rtabmap-ros` 바이너리를 활용하여 패키지 빌드 시간에 발생하는 불확실성을 최소화.
2. **2단계**: `ros2 topic hz /rtabmap/odom` 명령을 통해 $50\text{Hz}$ 출력을 사전에 확인하도록 설계.
3. **3단계**: 수동 주행 시 속도를 $0.2 \sim 0.3\text{m/s}$로 제한하는 권장 사항은 센서 데이터의 모션 블러를 방지하고 루프 클로저(Loop Closure) 정밀도를 극대화.
4. **4단계**: 고속 센서 데이터 수집 과정에서 발생할 수 있는 스토리지 I/O 병목을 예방하기 위해 `record_experiment.sh` 스크립트로 오도메트리 및 주요 제어 토픽만을 선택적으로 로깅.

---

## 📐 5. 정량적 평가 지표 및 수식 수립의 학술적 엄밀성 검증

05번 마스터 계획서와 06번 총괄보고서에 정의된 정량적 평가 지표들은 단순한 완주 성공 여부 측정을 넘어, 비동기 VLM 항법의 인과 정합성을 평가할 수 있도록 수학적으로 엄밀하게 구성되어 있습니다.

### ① 정규화 완주 시간 (Normalized Completion Time, $T^\dagger$)
$$T_i^\dagger = S_i \min(T_i, T_{\max}) + (1 - S_i) T_{\max}$$
* 주행 평가 시 단순 평균 시간 지표를 도입할 경우, 출발 직후 벽에 충돌하여 실험이 조기 종료된 실패 사례가 전체 평균 시간을 낮추어 성능이 우수한 것으로 왜곡되는 심각한 파라독스가 발생합니다. 
* 위 공식은 성공 여부를 나타내는 이진 지표 $S_i \in \{0, 1\}$를 적용하여 **주행 실패 시 타임아웃 페널티 $T_{\max}$를 강제로 부여함으로써 이러한 평가 오류를 완전히 차단**합니다.

### ② 방향성 복구 점수 (Directional Recovery Score, DRS)
$$\text{DRS} = \frac{N_{\text{escaped and resumed}}}{N_{\text{true detected}}}$$
* 이 지표는 로봇이 막다른 복도나 장애물에 의해 정체 상태($N_{\text{true detected}}$)에 빠졌음을 스스로 정확히 진단하고, 능동적 시야 확장 및 비동기 재계획을 거쳐 정상 주행 궤적으로 탈출($N_{\text{escaped and resumed}}$)한 비율을 측정함으로써 **시각-언어 재계획 알고리즘의 실질적 복구 능력을 정량화**합니다.

### ③ 실패 에지 재진입률 (Failed-Branch Re-entry Rate, FBR)
$$\text{FBR} = \frac{N_{\text{failed edge reentry}}}{N_{\text{opportunity}}}$$
* 연구동 내 반복되는 직각 코너 및 복도(Repeated corridor) 시나리오에서, 이전에 진입했다가 실패했던 경험 경로($N_{\text{opportunity}}$)에 로봇이 다시 접근했을 때 **경험적 가중치 그래프(Experience-Shaped Graph)가 비인과적 재진입($N_{\text{failed edge reentry}}$)을 효과적으로 억제하는지 판별**하는 핵심 기준이 됩니다.

---

## 💡 6. 결론 및 현장 실증 4대 개선 권고 사항

05번 마스터 계획서(`05_real_robot_indoor_testing_protocol.md`)에 대한 종합 검증 결과, 본 계획서는 소프트웨어 아키텍처, 온보드 메모리 제약, 학술적 역할 정의, 실증 시나리오 및 수학적 평가 지표 간의 연계성이 매우 뛰어난 고품질 마스터 문서로 최종 승인되었습니다.

실증 데이터의 안정적인 수집과 현장 예외 상황 방지를 위해 다음과 같은 **4대 실무 개선 항목**을 즉시 시스템에 반영합니다:

1. **UDP 소켓 패킷 무결성 보장**:
   * 비신뢰성 전송인 UDP 프로토콜의 특성상 순간적인 무선 패킷 손실이 발생할 수 있으므로, `host_bridge.py`와 `docker_bridge.py` 간 전송 구조체에 **4-byte 헤더 매직 넘버(`0x53324501`)와 체크섬 로직**을 포함시켜 잘못된 데이터 주입을 방지합니다.
2. **RTAB-Map 순수 오도메트리 파라미터 명시**:
   * 4단계 자율주행 실행 시 기존 저장 지도가 위치 추정에 개입하지 못하도록, 런치 파일 실행 파라미터에 `localization:=true` (`Mem/IncrementalMemory: false`) 옵션이 명확히 적용되도록 분리합니다.
3. **NetBird VPN 지연 모니터링 체계 구축**:
   * 와이파이 간섭으로 인한 VPN RTT(Round Trip Time) 급증에 대비하여, 원격 서버 연동 상태를 실시간 모니터링하고 필요 시 로컬 AP 망 Direct IP 커넥션으로 즉시 전환할 수 있는 `scratch/check_network_latency.sh` 진단 도구를 제공합니다.
4. **Rosbag 저장 스토리지 I/O 버퍼 설정**:
   * 센서 데이터의 고속 저장 시 Jetson Flash 스토리지 병목을 줄이기 위해, `scratch/record_experiment.sh` 스크립트에 `--max-cache-size 104857600` ($100\text{MB}$) 전용 쓰기 큐 버퍼 크기를 지정합니다.
