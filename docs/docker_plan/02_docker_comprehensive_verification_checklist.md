# 🐳 [Docker Plan 02] 도커(Docker) 샌드박스 5대 핵심 검증 항목 및 정밀 점검 매뉴얼

> **문서 소유자**: **민석 (Minseok - Hardware, Sensor & Deployment Lead)**  
> **논문 공식 명칭**: **`ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation` (ICRA 2026)**  
> **컨테이너 환경**: `sdam_go2_container` (Ubuntu 24.04 LTS / ROS 2 Jazzy ARM64 / Python 3.12)  
> **문서 목적**: 실물 로봇 실주행 전, 도커 격리 환경 내부에서 사전에 전수 검증해야 하는 **5대 핵심 영역(S2E 알고리즘, 단위 테스트, 공유 메모리/권한, VLM 의사결정 파이프라인, 비동기 폐루프 드라이런)의 체크리스트와 1줄 실행 명령어**를 제공합니다.

---

## 📌 목차 (Table of Contents)
1. [도커 5대 핵심 점검 매트릭스](#1-도커-5대-핵심-점검-매트릭스)
2. [점검 1: S2E 코어 및 노드 단위 테스트 (Unit Tests)](#점검-1-s2e-코어-및-노드-단위-테스트-unit-tests)
3. [점검 2: 50Hz 고주파 UDP 스트리밍 스트레스 테스트](#점검-2-50hz-고주파-udp-스트리밍-스트레스-테스트)
4. [점검 3: 720p 멀티모달 이미지 VLM 실시간 추론 테스트](#점검-3-720p-멀티모달-이미지-vlm-실시간-추론-테스트)
5. [점검 4: S2E 비동기 자율주행 풀 드라이런 (Full Dry-Run)](#점검-4-s2e-비동기-자율주행-풀-드라이런-full-dry-run)
6. [점검 5: 도커 공유 메모리 및 시스템 권한 상태 확인](#점검-5-도커-공유-메모리-및-시스템-권한-상태-확인)

---

## 📊 1. 도커 5대 핵심 점검 매트릭스

| 점검 번호 | 점검 항목 | 실행 스크립트 / 명령어 | 검증 목표 및 성공 기준 | 상태 |
| :---: | :--- | :--- | :--- | :---: |
| **1** | **S2E 코어 단위 테스트** | `python3 -m unittest discover` | SE(2) 궤적 보간, transforms_2d, vlm_schema 100% PASS | 🟢 **PASS** |
| **2** | **50Hz UDP 스트레스** | `python3 test_docker_50hz_stress.py` | 500개 패킷 0% 유실, < 0.1ms 지연, 매직헤더 0x53324501 | 🟢 **PASS** |
| **3** | **멀티모달 VLM 추론** | `python3 test_docker_real_image_vlm.py` | 720p RGB 이미지 인코딩 ➔ Qwen3-VL Action/Goal JSON 추출 | 🟢 **PASS** |
| **4** | **S2E 풀 드라이런** | `python3 test_docker_s2e_dryrun.py` | 관측 ➔ VLM 의사결정 ➔ 50Hz 궤적 ➔ 속도 명령 폐루프 확인 | 🟢 **PASS** |
| **5** | **도커 리소스/권한** | `df -h /dev/shm`, `docker inspect` | `/dev/shm` 여유 공간 및 포트 9090/9091 네트워크 바인딩 | 🟢 **PASS** |

---

## 🧪 2. 현장 1줄 실행 명령어 (One-Liner Runbook)

### [점검 1] 도커 내부 S2E 단위 테스트 전수 실행
```bash
docker exec sdam_go2_container bash -c "
    python3 -m unittest discover -s /workspace/go2_ws_antarctica/s2e-vlm-async-framework/src/s2e_vlm_core/test
    python3 -m unittest discover -s /workspace/go2_ws_antarctica/s2e-vlm-async-framework/src/s2e_vlm_nodes/test
"
```

### [점검 2] 50Hz 고주파 UDP 스트리밍 스트레스 테스트 (10초)
```bash
python3 /home/unitree/go2_ws_antarctica/scratch/test_docker_50hz_stress.py
```

### [점검 3] 720p 멀티모달 이미지 기반 VLM 원격 추론 테스트
```bash
docker exec sdam_go2_container python3 /workspace/go2_ws_antarctica/scratch/test_docker_real_image_vlm.py
```

### [점검 4] S2E 비동기 자율주행 풀 드라이런 (End-to-End Dry-Run)
```bash
docker exec sdam_go2_container python3 /workspace/go2_ws_antarctica/scratch/test_docker_s2e_dryrun.py
```

### [점검 5] 도커 공유 메모리 및 시스템 권한 점검
```bash
docker exec sdam_go2_container df -h /dev/shm
```
