# 📑 [04] ESCAPE-Nav PointNav 미팅 분석 및 플랫폼별 이원화 전략 가이드

> **문서 소유자**: **민석 (Minseok)**  
> **논문 공식 명칭**: **`ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation` (ICRA 2026)**  
> **문서 근거**: 상준 님의 최신 13페이지 논문 초안(`ICRA_논문 초안.pdf`) 및 팀 미팅 의사결정  
> **문서 목적**: ESCAPE-Nav 비동기 시스템 구조, Habitat(PixNav) vs 실물로봇/NavBench(S2E) 이원화 전략, PointNav 2중 안전 장치(Stop Guard) 및 민석 님 전담 실물 로봇 주행 파이프라인 총괄 명세입니다.

---

## 📌 목차 (Table of Contents)
1. [ICRA_point nav.pdf 9대 핵심 내용 요약](#1-icra_point-navpdf-9대-핵심-내용-요약)
2. [전략 이원화: Habitat(PixNav) vs 실물 로봇/NavBench-GS(S2E)](#2-전략-이원화-habitatpixnav-vs-실물-로봇navbench-gss2e)
3. [PointNav 2중 Stop Guard 및 View Ranking 안전 메커니즘](#3-pointnav-2중-stop-guard-및-view-ranking-안전-메커니즘)
4. [민석 님 전담 실물 로봇 4대 코스 주행 프로세스](#4-민석-님-전담-실물-로봇-4대-코스-주행-프로세스)

---

## 🔍 1. ICRA_point nav.pdf 9대 핵심 내용 요약

1. **PointNav 관측**: `[distance, bearing]` 상대 좌표 센서를 수신하여 VLM 비동기 메모리 그래프가 목표 뷰를 랭킹하고 S2E 궤적을 비동기로 출력함.
2. **Habitat 시뮬레이션 내 S2E 한계 발견**: 공터(`ep0`)에서는 잘 되나 좁은 복도(`ep1`)에서 문틀 찝힘 후 탈출 불가 현상 확인 ➔ 시뮬레이션과 실로봇 전략을 이원화하기로 결정.
3. **플랫폼별 역할 분리**:
   * **Habitat 3D 시뮬레이션 (현서/현우)**: `VLM + PixNav (Async)`로 주행 성공률 극대화 입증.
   * **NavBench-GS & 실물 Go2 (건민/민석)**: `VLM + S2E (Async)`로 4족 보행의 연속 궤적성, 90도 코너링, GPS/Pose 노이즈 환경에서의 강건성 입증.

---

## ⚖️ 2. 전략 이원화: Habitat(PixNav) vs 실물 로봇/NavBench-GS(S2E)

| 비교 항목 | **Habitat 3D 시뮬레이션** | **실물 로봇 Unitree Go2 & NavBench-GS** |
| :--- | :--- | :--- |
| **주요 실행기 (Executor)** | **`VLM + PixNav (Async)`** | **`VLM + S2E (Async)`** |
| **선택 배경** | 좁은 복도 문틀 찝힘 회피 및 PointNav 성공률 입증 | 4족 보행 고유의 부드러운 궤적 연속성 및 노이즈 강건성 입증 |
| **담당 연구원** | 현서 님, 현우 님 | **민석 님**, 건민 님 |

---

## 🛡️ 3. PointNav 2중 Stop Guard 및 View Ranking 안전 메커니즘

1. **Stop Guard 1 (강제 정지)**: 로봇이 `goal radius` (0.2m ~ 1.0m) 이내에 진입하면 VLM 응답과 무관하게 **강제로 `stop` 명령 인가**.
2. **Stop Guard 2 (오정지 반려)**: 로봇이 아직 `goal radius` 바깥인데 VLM이 `stop`을 내면 **`stop`을 무시하고 관측 요청(Observation Request)으로 되돌려 주행 속행**.
3. **S2E Candidate View Ranking**: 후보 뷰 랭킹 시 Object Semantic보다 **`목표점 방위각(Bearing)`을 최우선**으로 사용하여 S2E 궤적 산출.

---

## 🐕 4. 민석 님 전담 실물 로봇 4대 코스 주행 프로세스

```text
[1] Offline Stage (사전 맵핑):
    • RTAB-Map LIVO로 실내 복도 수동 주행 1회 ➔ 실내 3D 점군 지도 1장 생성.
    • 지도 상에서 4대 시나리오(직선, 90도 직각 코너, T자 갈림길, 동적 장애물) 목표점 좌표 마킹.

[2] Online Stage (실시간 4대 코스 주행):
    • 로봇 부팅 ➔ 50Hz VLIO 위치 정합 ➔ PointNav 태스크 인가 ➔ 목표 도달 ➔ Rosbag 자동 로깅 ➔ 자동 채점.
```
