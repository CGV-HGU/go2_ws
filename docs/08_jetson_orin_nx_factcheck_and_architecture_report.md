# 📑 Unitree Go2 기반 Jetson Orin NX 온보드 자율주행 시스템 기술 검증 및 최적 아키텍처 수립 보고서

> **본 보고서는** 사족보행 로봇 Unitree Go2의 내장 온보드 컴퓨터인 **NVIDIA Jetson Orin NX 16GB** 상에서 비전-나비게이션 트랜스포머(ViNT, NoMAD 등) 및 S2E-VLM 자율주행 모델을 단독(All-in-One) 구동할 때 발생하는 물리적 커널 레이어, OS, ROS 2 미들웨어 스택의 병목 현상을 정밀 검증하고, 최적의 시스템 아키텍처 수립 방향을 다룹니다.

---

## 1. 📌 서론 및 연구 배경

외부 컴퓨팅 서버의 조력 없이 Unitree Go2의 내장 컴퓨팅 자원인 **Jetson Orin NX 16GB** 단독으로 임베디드 자율주행 및 비전-나비게이션 트랜스포머(ViNT, NoMAD 등) 모델을 구동하는 올인원(All-in-One) 자율주행 시스템 구축 시도는 물리적 커널 레이어, 임베디드 운영체제(OS), ROS 2 미들웨어 스택에 걸친 복합적인 하드웨어 및 소프트웨어 병목을 수반합니다.

* **호스트 OS 제약**: 제조사인 Unitree는 Go2 Pro 및 EDU 모델의 동적 보행 안정성과 관절 모터 제어 드라이버의 실시간성을 보장하기 위해 호스트 OS를 **Ubuntu 20.04 LTS 기반 JetPack 5.1.x (L4T R35)** 패키지로 고정하여 제공합니다.
* **최신 알고리즘 요구사항**: 최신 자율주행 연구 스택은 Python 3.12, ROS 2 Jazzy Jalisco 배포판, CUDA 12.x 이상을 요구함에 따라 호스트 OS와 자율주행 소프트웨어 간의 **삼각 버전 갈등(Triangular Version Conflict)**이 발생합니다.
* **가상화(Docker) 우회 시도와 한계**: 버전 격리를 위해 도커 컨테이너 우회가 제시되었으나, 임베디드 SoC(System on Chip) 특유의 GPU 드라이버 커널 종속성으로 인해 연산 마비 현상이 보고되고 있습니다.

---

## 2. 📊 하드웨어·소프트웨어 스택 팩트체크 (Fact-Check)

엔비디아 개발자 문서, ROS 2 공식 배포 규격, 사족보행 로봇 운용 실증 데이터를 기반으로 기존 주장을 정밀 검증한 결과입니다.

| 검증 대상 항목 | 기존 주장 내용 | 정밀 팩트체크 결과 | 기술적 근거 및 세부 원인 분석 |
| :--- | :--- | :--- | :--- |
| **호스트 OS 제약 및 워런티** | Ubuntu 20.04/JetPack 5 고정. 업그레이드 시 모터 커널 파손 및 워런티 소멸 | 🟡 **부분적 사실 (Partially True)** | 제조사 정식 기술 지원 및 워런티는 JetPack 5에 귀속됨. 그러나 NVMe 전체 재플래싱을 통해 JetPack 6.2.2(Ubuntu 22.04, L4T R36.5)로 전환하고 CycloneDDS 통신을 재구성한 성공 사례가 있어, 플래싱 자체가 불가능한 것은 아님. |
| **CUDA 드라이버 불일치** | JetPack 5 호스트 위에서 Ubuntu 24.04/CUDA 12.x 도커 구동 시 GPU 연산 완전히 마비 | 🔴 **사실 (True)** | Tegra L4T 아키텍처 특성상 컨테이너 내부 CUDA 사용자 공간 라이브러리는 호스트 커널 드라이버(`nvgpu.ko`, JetPack 5 = CUDA 11.4) 한계를 초과할 수 없음. 실행 시 `CUDA driver version is insufficient` 오류 발생. |
| **ARM64 도커 이미지 부재** | ROS 2 Jazzy 공식 이미지가 x86_64 전용이며, ARM64 구동 시 `Exec format error` 크래시 발생 | 🟢 **거짓 (False / Misconception)** | Open Source Robotics Foundation(OSRF)은 ROS 2 Jazzy(`ros:jazzy`, `osrf/ros:jazzy-desktop`)의 `arm64v8/aarch64` 이미지를 공식 배포 중임. 크래시는 빌드 시 `--platform linux/arm64` 미지정으로 인한 아키텍처 혼선이 원인. |
| **디바이스(`/dev`) 접근 제약** | 도커 가상화 내 센서 포트 접근 실패 시 드라이버 노드가 다운됨 | 🔴 **사실 (True)** | 격리 공간 특성상 시리얼, USB, CSI 카메라, GPIO 접근을 위한 `--device` 바인딩, `privileged` 권한, `udev` 그룹 매핑이 누락되면 I/O 연결이 원천 차단됨. |
| **ROS 2 Jazzy 필수 요구성** | ViNT, NoMAD 등 최신 AI 알고리즘 구동을 위해 ROS 2 Jazzy 필수 요구 | 🟢 **거짓 (False)** | ViNT 및 NoMAD는 PyTorch 기반 신경망 모델로, ROS 2 프레임워크 자체와 직접적 결합도가 없음. ONNX 포맷 및 TensorRT 엔진으로 추출 시 특정 ROS 2 버전에 종속되지 않고 단독 추론 가능. |

---

## 3. 🔬 심층 기술 분석 및 병목 메커니즘

### 3.1 Tegra L4T 커널 구조와 CUDA 드라이버 불일치 메커니즘
x86 데스크톱 환경에서는 호스트에 최신 그래픽 드라이버를 설치하고 NVIDIA Container Toolkit을 통해 컨테이너 내부에서 유연한 CUDA 버전을 구동할 수 있습니다. 

그러나 Jetson 플랫폼은 CPU와 GPU가 동일한 LPDDR 메모리를 공유하는 **통합 메모리 아키텍처(UMA)** 기반 System-on-Chip 구조입니다.

```
[ Container Space (Ubuntu 24.04) ]
   └── User-space CUDA 12.x Toolkit & Libraries
            │  ❌ ABI Mismatch Breakdown! (CUDA Error 35)
            ▼
[ Host Kernel Space (JetPack 5.1.x / L4T R35) ]
   └── nvgpu.ko Kernel Driver (CUDA 11.4 Hardware Interface API Only)
```

1. **JetPack 5.1.x (L4T R35) 커널 한계**: 호스트 OS(`nvgpu.ko`)는 물리적으로 **CUDA 11.4 드라이버 API 세트**까지만 지원합니다.
2. **ABI(Application Binary Interface) 불일치**: 컨테이너 내부에 CUDA 12.x/13.x 사용자 공간 라이브러리를 설치하더라도 호스트 커널 드라이버 호출 단계에서 ABI 불일치가 발생합니다.
3. **결과**: `CUDA Error 35 (CUDA_ERROR_INSUFFICIENT_DRIVER)`가 반환되며 CUDA Context 생성이 거부되고, GPU 가속이 차단되어 CPU 폴백(Fallback) 또는 프로세스 다운 현상이 발생합니다.

### 3.2 도커 가상화와 ARM64 아키텍처 호환성의 실상
* OSRF는 Ubuntu 24.04 (Noble Numbat) 기반의 `arm64v8` 공식 ROS 2 Jazzy 이미지를 Docker Hub에 공개 및 유지보수하고 있습니다.
* 현장에서 발생하는 `Exec format error`의 실제 원인은:
  1. x86_64 빌드 PC에서 `--platform linux/arm64` 옵션 없이 교차 빌드 시 바이너리 아키텍처 혼선 발생.
  2. 컨테이너 내부 CUDA 라이브러리가 호스트 L4T 커널 초기화 중 크래시를 일으키는 현상을 아키텍처 오류로误진단함.

### 3.3 Unitree Go2 호스트 OS 업그레이드와 제조사 보증
* **JetPack 6.2.2 (Ubuntu 22.04 LTS, L4T R36.5) 업그레이드 가능 여부**: 타겟 재플래싱(NVMe SSD 전체 플래싱)을 통한 업그레이드 자체는 수술적으로 가능합니다.
* **실무적 리스크**:
  > [!CAUTION]
  > Unitree 본사의 **공식 워런티 및 기술 지원이 즉시 파기**되며, 기존 모션 제어 드라이버 및 `unitree_ros2` 통신 파이프라인, CycloneDDS 파라미터, 센서 바인딩 노드를 처음부터 재구축해야 하는 막대한 공수가 발생합니다.

---

## 4. ⚡ 온보드 컴퓨팅 병목 (Reality Check)

### 4.1 DDS 통신 미들웨어 및 TF 트리 지연
* 도커 실행 시 `--net=host` 옵션으로 네트워크 가상화를 우회하더라도, 호스트(ROS 2 Foxy)와 컨테이너(ROS 2 Jazzy) 간 메시지 패킷 정의나 IDL 직렬화 규격 차이가 발생하면 디코딩 오버헤드가 누적됩니다.
* 사족보행 제어기는 최소 10Hz 이상의 제어 루프 유지를 요구하므로, 통신 지연 시 제어권 상실 또는 비상 정지 리스크가 극대화됩니다.

### 4.2 Jetson Orin NX 16GB 메모리 및 GPU 대역폭 한계
* **메모리 대역폭**: 102.4 GB/s UMA 구조
* **자원 경쟁**: LiDAR SLAM(RTAB-Map), 카메라 버퍼링, RViz 시각화, 디퓨전 내비게이션(NoMAD) 동시 실행 시 **Out-Of-Memory(OOM) 크래시**가 유발됩니다.
* TensorRT 최적화 없이 PyTorch 원본 모델 직접 추론 시 지연 시간이 100ms를 초과하여 실시간 충돌 회피가 불가합니다.

---

## 5. 🛠️ 최적 아키텍처 구현 및 단계별 우회 전략 비교

| 평가 기준 | **전략 A: Pure Python + ONNX/TensorRT (최우선 추천)** | **전략 B: JetPack 6.2.2 호스트 재플래싱** | **전략 C: L4T Base 컨테이너 내 소스 빌드** |
| :--- | :--- | :--- | :--- |
| **기반 환경** | **JetPack 5.1.x 순정 호스트 OS 유지** | 호스트 전체를 Ubuntu 22.04로 재플래싱 | JetPack 5 호스트 + `nvcr.io/nvidia/l4t-base` 컨테이너 |
| **ROS 미들웨어** | **ROS 2 Foxy (네이티브) 또는 단독 노드** | ROS 2 Humble (네이티브) | 컨테이너 내부 ROS 2 Jazzy (소스 빌드) |
| **AI 추론 엔진** | **ONNX Runtime / TensorRT (CUDA 11.4 가속)** | PyTorch / TensorRT (CUDA 12.6 가속) | PyTorch / CUDA 11.4 제한적 가속 |
| **제조사 워런티** | 🟢 **100% 보존** | 🔴 **소멸 (Void)** | 🟢 **100% 보존** |
| **추론 지연시간** | ⚡ **초저지연 (Sub-30ms)** | 🟡 **저지연 (30~50ms)** | 🔴 **가상화 오버헤드로 고지연** |
| **구현 난이도** | 🟢 **보통 (모델 경량화/ONNX 변환)** | 🔴 **높음 (전체 재플래싱/센서 재설정)** | 🔴 **극상 (의존성 패키지 수동 컴파일)** |

---

## 6. 🚀 세부 실행 전략

### 🟢 전략 A: Pure Python 및 ONNX Runtime / TensorRT 탈중앙화 파이프라인 (최우선 권장안)
ROS 2 Jazzy 가상화 환경을 도커로 무리하게 구축하는 대신, 최신 AI 모델을 호스트 시스템 호환 포맷으로 경량화하여 단독 프로세스로 구동하는 전략입니다.

```
[ ViNT / NoMAD PyTorch Checkpoint ]
                │
                ▼ (ONNX Export & Quantization)
[ ONNX Engine (CUDA 11.4 / TensorRT EP) ]  <-- JetPack 5.1.x Native Host
                │
                ▼ (Sub-30ms Inference Output: Waypoint 10x2)
[ unitree_sdk2_python Direct DDS Motion Command ]
                │
                ▼
[ Unitree Go2 SportClient.Move(vx, vy, yaw) ]
```

1. ViNT/NoMAD 모델 체크포인트를 ONNX 포맷으로 내보낸 후, 호스트 OS(JetPack 5.1.x)의 CUDA 11.4 및 TensorRT Execution Provider와 호환되는 `onnxruntime-gpu` 라이브러리로 추론을 수행합니다.
2. 추론된 2D 웨이포인트 결과를 `unitree_sdk2_python` 라이브러리를 통해 직접 Go2 로봇 제어기로 전달합니다.
3. **장점**: 순정 호스트 OS 유지로 워런티가 보전되며, 도커 가상화 오버헤드가 제거되어 추론~제어 지연시간을 **30ms 이내**로 단축합니다.

---

### 🟡 전략 B: JetPack 6.2.2 호스트 재플래싱 및 ROS 2 Humble 단일화 (장기 표준안)
자율주행 연구 스택이 ROS 2 표준 노드 구조 및 Isaac ROS 가속 파이프라인과 강하게 결합되어 있어 호스트 개편이 불가피할 때 적용합니다.

1. Go2 온보드 Jetson Orin NX를 복구 모드(APX)로 전환 후 NVMe SSD 전체를 **JetPack 6.2.2 (Ubuntu 22.04 LTS, L4T R36.5)**로 재설치합니다.
2. 호스트 OS 상에 **ROS 2 Humble LTS**를 네이티브 구성합니다.
3. **장점**: 최신 CUDA 12.6 드라이버 스택과 Isaac ROS 3.x 가속 모듈(Visual SLAM, nvblox 3D Mapping 등)을 네이티브 활용 가능합니다. (단, 워런티 소멸 사전 승인 필요)

---

### 🔴 전략 C: NVIDIA L4T Base 컨테이너 상 Custom ROS 2 Jazzy 소스 빌드 (제한적 대안)
호스트 OS(JetPack 5)와 워런티를 보존하면서 컨테이너 내부에서 ROS 2 Jazzy 환경을 꼭 써야할 때의 수동 우회 전략입니다.

1. `nvcr.io/nvidia/l4t-base:r35.4.1` 베이스 이미지 기반으로 Dockerfile을 작성합니다.
2. 컨테이너 내부 Python 3.12 스택 빌드 후 ROS 2 Jazzy 소스코드를 수동 클론하여 `colcon build` 수행.
3. **단점**: 컴파일 시 의존성 패키지 충돌이 극심하며 빌드 시간이 과도하게 소요되어 현장 유지보수가 매우 까다롭습니다.

---

## 7. 🎯 결론 및 종합 권고 (Actionable Recommendations)

1. **도커 크래시의 본질**: 도커 컨테이너를 통한 Ubuntu 24.04 / ROS 2 Jazzy 구동 시의 장애 원인은 ARM64 아키텍처 미지원이 아니라, **Tegra UMA 아키텍처 상 JetPack 5 호스트 커널 드라이버(`nvgpu.ko`)와 컨테이너 내 CUDA 12.x 사용자 라이브러리 간 ABI 불일치**입니다.
2. **최우선 실행안**: 단기 개발 타당성과 연산 가속을 위한 최선의 선택은 **전략 A (Pure Python + ONNX Runtime / TensorRT)**입니다. 호스트 OS를 건드리지 않고 Sub-30ms 추론 및 100% 워런티 보전이 가능합니다.
3. **장기 연구 생태계안**: Isaac ROS 및 ROS 2 네이티브 파이프라인 정립이 필수라면, 워런티 파기 승인 후 **전략 B (JetPack 6.2.2 & ROS 2 Humble 재플래싱)**를 채택하여 CUDA 12.6 네이티브 성능을 확보해야 합니다.
