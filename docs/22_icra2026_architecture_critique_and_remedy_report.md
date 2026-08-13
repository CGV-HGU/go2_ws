# 🎓 [VL-MAG / ICRA 2026] Unitree Go2 기반 온보드 자율주행 배포 전략 정밀 검증 및 아키텍처 비판 보고서

> **문서 소유자**: **민석 (Minseok)**  
> **공유 대상**: 상준 (리더), 현서, 건민, 현우 및 ICRA 2026 연구 팀 전체  
> **문서 목적**: Unitree Go2 EDU 온보드 컴퓨팅(Jetson Orin NX 16GB) 배포 전략의 **6대 핵심 구조적 결함(No-SLAM 프레이밍 충돌, VLM 메모리 폭발, UDP QoS 부재, 3-DOF 횡속도 차단, 통계적 엄밀성 미달, Rosbag 디스크 I/O 병목)**을 정밀 검증하고, 이를 극복하기 위한 학술적·기술적 개선 대안(Strategic Remedies)을 수립한 정밀 보고서입니다.

---

## 📌 목차 (Table of Contents)
1. [개요 및 학술적 개념의 근본적 모순점 (No-SLAM vs RTAB-Map LIVO)](#1-개요-및-학술적-개념의-근본적-모순점)
2. [온보드 하드웨어 제약 및 VLM 구동 물리적 불가능성 팩트체크](#2-온보드-하드웨어-제약-및-vlm-구동-물리적-불가능성-팩트체크)
3. [시스템 통신 아키텍처 및 이중 OS 브릿지 취약점 분석](#3-시스템-통신-아키텍처-및-이중-os-브릿지-취약점-분석)
4. [센서 처리 파이프라인 및 사족보행 제어기 알고리즘 비판](#4-센서-처리-파이프라인-및-사족보행-제어기-알고리즘-비판)
5. [ICRA 2026 평가지표 및 실험 로깅 시스템 비판](#5-icra-2026-평가지표-및-실험-로깅-시스템-비판)
6. [종합 결론 및 시스템 재설계 로드맵 (Actionable Remedies)](#6-종합-결론-및-시스템-재설계-로드맵)

---

## 1. 🔍 1. 개요 및 학술적 개념의 근본적 모순점

### 1.1 No-SLAM Driven 연구 프레이밍과 RTAB-Map LIVO 시스템의 명백한 논리 충돌
본 연구의 핵심 정체성은 "No-SLAM driven 자율주행"을 실물 로봇 온보드 환경에 배포하는 것으로 명시되어 있다. 그러나 메인 아키텍처 명세에 따르면, 호스트 OS 상에서 RTAB-Map LIVO(`go2_rtabmap.launch.py`)를 $50\text{Hz}$ 주기로 구동하여 실시간 위치 추정(Odometry) 및 3D 점군 지도 작성(Mapping)을 상시 수행하도록 설정되어 있다.

RTAB-Map(Real-Time Appearance-Based Mapping)은 대표적인 그래피컬 루프 클로저 기반 3D Visual-LiDAR SLAM 프레임워크이다. 지도 작성 없이 비주얼 메모리와 비동기 프레임워크에 기반하여 주행한다는 "No-SLAM" 연구에서 고성능 SLAM 시스템인 RTAB-Map LIVO를 $50\text{Hz}$로 동시 가동하는 것은 논리적으로 상충한다. 심사위원(Reviewer) 시각에서 이러한 구성은 주행 성공률의 향상이 비주얼 메모리 기반 VLM 정책 때문인지, 혹은 고성능 SLAM 오도메트리를 제공하는 RTAB-Map 때문인지 인과관계를 명확히 분리할 수 없게 만드는 치명적인 실험 설계 오류로 평가된다.

### 1.2 ICRA 2026 기여도(Contribution) 훼손 가능성
VL-MAG(Vision-Language Memory-Action Graph) 연구는 전통적인 기하학적 맵을 사전에 구축하거나 유지하지 않고, 비주얼 임베딩 그래프만으로 자율주행하는 능력을 기여점으로 제시해야 한다. 실제 배포 환경에서 RTAB-Map LIVO를 위치 추정기로 강제 바인딩함으로써 "No-SLAM" 프레이밍의 학술적 신선도(Novelty)를 훼손하고 "기존 3D SLAM 위에서 구동되는 단순 포즈 선택기"로 격하될 위험이 존재한다.

---

## 2. ⚡ 2. 온보드 하드웨어 제약 및 VLM 구동 물리적 불가능성 팩트체크

### 2.1 Jetson Orin NX 16GB 메모리 병목 및 Qwen3-VL 32B 탑재 불가능성
Jetson Orin NX 16GB의 LPDDR5 통합 메모리(Unified Memory) 스펙과 Qwen3-VL 32B 소요 자원 정밀 비교:

| 평가 항목 | Jetson Orin NX 16GB 하드웨어 스펙 | Qwen3-VL 32B 최소 소요 자원 | 온보드 구동 가능 여부 및 검증 결과 |
| :--- | :--- | :--- | :--- |
| **통합 메모리 (RAM/VRAM)** | 16 GB (LPDDR5 Unified) | • FP16: ~64~73 GB<br/>• INT8: ~32~36 GB<br/>• INT4: ~16~18 GB | 🔴 **실행 불가능 (Immediate OOM)**<br/>호스트 OS 점유(~2.5GB) 제외 가용 가상화 메모리(13.5GB) 초과로 가중치 로딩 즉시 OOM 크래시. |
| **메모리 대역폭** | 102.4 GB/s | 패스당 16~64GB 대역폭 점유 | 🔴 **전체 시스템 병목**<br/>VLM 추론 시 대역폭 독점으로 RTAB-Map 및 DDS 제어 패킷 드롭 발생. |
| **구동 모드 설정** | Docker CPU Mode | CPU 추론 시 서버급 Multi-socket 필요 | 🔴 **실시간성 파탄**<br/>8코어 ARM CPU 추론 시 프레임당 수십 초 소요되어 10Hz 제어 불가능. |

---

### 2.2 온보드 실증을 위한 경량 VLM (Qwen3-VL 2B / 4B) 대체안

온보드 실증을 위해 **Qwen3-VL 32B 모델은 Pro 6000 원격 서빙 서버 전용**으로 사용하고, **Jetson Orin NX 온보드 보드에는 Qwen3-VL 2B/4B 경량 모델**을 다운스케일링하여 탑재합니다:

| 대안 모델 규격 | 파라미터 및 양자화 | 소요 VRAM (KV 캐시 포함) | Jetson Orin NX 추론 속도 (GPU 가속) | 10Hz 실시간 제어 적합성 |
| :--- | :---: | :---: | :---: | :---: |
| **Qwen3-VL 2B** | GGUF / UD-IQ2_M | ~1.4 ~ 2.0 GB | **TensorRT-LLM / llama.cpp 적용 시 15~25 tps** | 🟢 **적합 ($10\text{Hz}$ 비동기 갱신 가능)** |
| **Qwen3-VL 4B** | INT4 AWQ / W4A16 | ~3.5 ~ 4.5 GB | **vLLM / TensorRT-LLM 적용 시 8~12 tps** | 🟡 **조건부 가능 ($5\sim 10\text{Hz}$ 갱신)** |
| **Qwen3-VL 8B** | INT4 AWQ | ~6.5 ~ 8.0 GB | vLLM 적용 시 2~4 tps | 🔴 부적합 (지연시간 > 250ms 발생) |

---

## 3. 🌐 3. 시스템 통신 아키텍처 및 이중 OS 브릿지 취약점 분석

### 3.1 Custom UDP 브릿지의 연산 오버헤드 및 패킷 손실 위험
* `scratch/host_bridge.py` $\leftrightarrow$ `docker_bridge.py` UDP 소켓 사용 시 QoS(Quality of Service) 부재로 패킷 순서 보장 및 흐름 제어가 불가능함.
* 젯슨 CPU 부하 급증 시 `/cmd_vel` 패킷 유실 발생 가능성 존재.

---

## 4. 🐕 4. 센서 처리 파이프라인 및 사족보행 제어기 알고리즘 비판

### 4.1 Unitree L2 LiDAR 제원 대비 50Hz RTAB-Map 연산 오버헤드
* L2 LiDAR 스캔 출력 주기는 $10\text{Hz} \sim 15\text{Hz}$ 수준임. $50\text{Hz}$ 업스케일링은 동일 프레임 중복 매칭 연산으로 메모리 대역폭(102.4 GB/s)을 헛되이 소모시킴.

### 4.2 횡속도($v_y=0.0$) 차단 해제 및 3-DOF 홀로노믹 기동성 복원
* 사족보행 로봇 특유의 홀로노믹 전방향 이동 우위를 살리기 위해 `pd_controller.py`에 $v_y$ 횡이동 댐핑 통로를 개방하여 좁은 복도 및 L자 코너에서 발끝 슬립(Foot-end Slip) 및 충돌률 최소화.

---

## 5. 📊 5. ICRA 2026 평가지표 및 실험 로깅 시스템 비판

### 5.1 95% Wilson Score Confidence Interval 및 Mann-Whitney U-test 도입
* 이분법적 성공률(SR %)에 대해 단순 표준편차 표기를 대신하여 **95% Wilson Score Confidence Interval**을 명시하고, SOTA(NoMAD) 대비 **Mann-Whitney U-test p-value** 산출 로직을 `scratch/calculate_icra_metrics.py`에 구현 완료.

### 5.2 Rosbag 디스크 쓰기 병목 해소
* `scratch/record_experiment.sh`에서 대용량 미가공 포인트클라우드(`/utlidar/cloud_deskewed`) 배제하고, 압축 비전 토픽(`/camera/front/compressed`), `/rtabmap/odom`, `/cmd_vel`, `/tf`만 선별 수집하여 NVMe I/O Wait 지터 방지.

---

## 🛠️ 6. 종합 결론 및 시스템 재설계 로드맵 (Actionable Remedies)

| 평가 항목 | 기존 antarctica 브랜치 설정 | 기술적 문제점 및 위험 요인 | **전략적 개선 대안 (Strategic Remedy)** |
| :--- | :--- | :--- | :--- |
| **연구 프레임워크** | RTAB-Map LIVO $50\text{Hz}$ 활성화 + "No-SLAM" 프레이밍 | 학술적 정체성 모순 및 논문 기여도 격하 | RTAB-Map은 Ground-Truth 오도메트리 수집용으로 분리하고, 주행 정책은 Pure Go2 VIO/Leg Odom과 비주얼 그래프 메모리로만 구동. |
| **VLM 모델 구동** | Qwen3-VL 32B + Docker CPU Mode | OOM 크래시 발생 및 연산 지연시간 파탄 | **온보드(Jetson)**: Qwen3-VL 2B/4B 다운스케일링 (TensorRT-LLM 15~25 tps)<br/>**서버(Pro 6000)**: Qwen3-VL 32B 원격 서빙 유지 |
| **통신 브릿지** | Custom UDP Socket (127.0.0.1:5005) | 패킷 유실 위험, QoS 부재, 파이썬 GIL 병목 | **RoboStack / Conda 가상환경** 도입 또는 CycloneDDS Zero-copy 인터페이스 활용 |
| **로봇 제어기** | $v_y = 0.0$ 강제 차단 | 사족보행 전방향 기동성 상실, 충돌률 및 발 슬립 증가 | **3-DOF 전방향 제어기 통로 개방 ($v_x, v_y, w_z$)** (`pd_controller.py` 반영 완료) |
| **실험 정량 평가** | $\text{Mean} \pm \text{SD}$ 단설 표 산출 | 이분법적 데이터 통계 엄밀성 미달 | **95% Wilson Score CI 표기 & Mann-Whitney U-test p-value** 산출 (`calculate_icra_metrics.py` 반영 완료) |
