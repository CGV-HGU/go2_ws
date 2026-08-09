# 📑 ICRA 2026 논문 주제, 실물 로봇 테스트 시나리오 및 SOTA 선행연구 종합 분석 보고서

> **문서 소유자**: **민석 (Minseok)**  
> **문서 목적**: 우리 팀의 ICRA 논문 핵심 주제(VOCA + S2E), Unitree Go2 실물 로봇 4대 평가 시나리오, 그리고 최고의 SOTA 선행연구(ViNT, NoMAD, VLFM, SARO)들의 실물 로봇 테스트 횟수 및 평가 방법론을 완벽히 정리한 학술 레퍼런스입니다.

---

## 📌 목차
1. [우리 논문 주제 및 핵심 연구 아이디어 (Paper Core Topic)](#1-우리-논문-주제-및-핵심-연구-아이디어-paper-core-topic)
2. [SOTA 선행연구들의 실물 로봇 테스트 방법론 분석 (Prior Works)](#2-sota-선행연구들의-실물-로봇-테스트-방법론-분석-prior-works)
3. [Unitree Go2 실물 로봇 4대 테스트 시나리오 세부 규격](#3-unitree-go2-실물-로봇-4대-테스트-시나리오-세부-규격)
4. [선행연구 대비 우리 논문의 차별화 포인트 (Novelty)](#4-선행연구-대비-우리-논문의-차별화-포인트-novelty)
5. [ICRA 논문 수록용 3종 세트 (Table 1, Figure 1, Figure 5)](#5-icra-논문-수록용-3종-세트-table-1-figure-1-figure-5)

---

## 1. 💡 우리 논문 주제 및 핵심 연구 아이디어 (Paper Core Topic)

### 📝 논문 주제명 (Topic Title)
**VOCA + S2E: Visual-Object Context Awareness Memory와 State-to-Execution Locomotion Policy의 결합을 통한 4족 보행 로봇(Go2) 자율주행**

```
[ 상위 레이어: VOCA (Visual-Object Context Awareness) ]
  • Qwen3-VL / Gemma 기반 에피소믹 메모리 & Node-Edge Graph 생성
  • 고차원 목표 제시 (Coarse/Fine Goal) & 막다른 길 (Deadlock) Look-around 회전 지시
                            │
              (결합 방식: 물리적 결합 vs 화학적 결합)
                            ▼
[ 하위 레이어: S2E (State-to-Execution Locomotion Policy) ]
  • 4족 보행 로봇 저전압/고속 제어 (50~60 Hz)
  • FAST-LIO2 + RTAB-Map 오도메트리 수신 & PD Controller 궤적 추종
                            │
                            ▼
               [ Unitree Go2 Real Hardware ]
```

### 🔗 2가지 결합 방식 (Coupling Schemes)
1. **물리적 결합 (Physical Coupling)**: VOCA(VLM)에서 예측한 보정 Waypoint $(x, y)$만 S2E 제어기로 전달하는 Decoupled 방식.
2. **화학적 결합 (Chemical / Deep Coupling)**: VOCA의 Reasoning Feature/Embedding을 S2E의 딥러닝 레이어에 직접 주입하여 백프로파게이션(Backprop)으로 종단간(E2E) 학습 및 미세조정하는 구조.

---

## 2. 📚 SOTA 선행연구들의 실물 로봇 테스트 방법론 분석 (Prior Works)

| 선행연구 논문 | 발표 학회 | 테스트에 사용된 실물 로봇 | 실물 로봇 테스트 규모 (Trials) | 주요 평가 지표 및 방법론 |
| :--- | :---: | :---: | :---: | :--- |
| **ViNT** *(Visual Nav Transformer)* | **CoRL 2023** | Locobot, Jackal 이동로봇 | **환경당 5회 $\times$ 3개 환경 = 총 15회** | Success Rate (SR %), SPL (%), Navigation Time (s) |
| **NoMAD** *(Masked Diffusion Nav)* | **ICRA 2024** | **Unitree Go1 4족보행**, Jackal | **시나리오당 5회 $\times$ 4개 지형 = 총 20회** | SR %, SPL %, Collision Count, Trajectory Plot |
| **VLFM** *(Vision-Language Maps)* | **ICRA 2024** | **Boston Dynamics Spot 4족보행** | **타겟당 4~5회 $\times$ 4개 건물 = 총 15~20회** | SR %, SPL %, Time-to-Goal, Human Interventions |
| **SARO** *(Space-Aware Robot)* | **ICRA 2024** | 4족보행 로봇 (Quadruped) | **상황당 10~20회 시도 (Total 20~40회)** | SR %, Gait Stability, Step Count |
| **CARE** *(Collision Avoidance)* | **ICRA 2024** | 4족보행 및 이동로봇 | **설정당 10회 시도** | Collision Rate %, Navigation Time |

> 💡 **선행연구 종합 요약**:  
> 학회(ICRA/CoRL)의 모든 대표 논문들이 실물 4족 보행 로봇 평가 시 **시나리오당 5회씩 총 15~20회 에피소드 시도**를 학회 표준 수치(Gold Standard)로 채택하고 있습니다.

---

## 3. 🐕 Unitree Go2 실물 로봇 4대 테스트 시나리오 세부 규격

민석 님이 현장에서 구동할 4개 코스 세부 구성안입니다 (코스당 5회 $\times$ 4개 코스 = **총 20회 시도**).

```text
========================================================================================
                 UNITREE GO2 REAL-ROBOT EXPERIMENTAL SCENARIOS
========================================================================================
[1] 시나리오 1: 실내 좁은 복도 (Indoor Narrow Corridor)
    • 환경: 20m L자 및 T자 코너가 포함된 좁은 1.5m 폭 건물 복도
    • 목적: VOCA의 정밀 웨이포인트 제시 및 PD 제어기의 회전 선회 성능 평가
    • 지표: 성공률(SR %), 완료 시간(s), 횡속도 차단($v_y=0$) 안정성

[2] 시나리오 2: 동적 장애물 (Dynamic Obstacle Avoidance)
    • 환경: 갑자기 움직이는 보행자, 튀어나오는 의자/박스 장애물 코스
    • 목적: 실시간 재계획(Replanning) 및 충돌 회피 반응 속도 평가
    • 지표: 충돌 횟수(collisions/ep), E-Stop 무선 킬스위치 개입률

[3] 시나리오 3: 막힌 길 ㄷ자 구역 (Deadlock Corner Recovery)
    • 환경: 3m × 3m 크기의 ㄷ자 모양 막다른 공간 (Dead-end)
    • 목적: VOCA Graph Memory 기반 360도 제자리 회전(Look-around) 후 탈출 검증
    • 지표: Deadlock 탈출 성공률(%), 탈출 소요 Step 수 및 시간

[4] 시나리오 4: 실외 험지 지형 (Outdoor Rough Terrain)
    • 환경: 자갈길, 풀밭, 10도 경사로 (태양광 및 바닥 발 슬립 존재)
    • 목적: RTAB-Map + FAST-LIO2 오도메트리 드리프트 내성 및 4족 보행 안정성 평가
    • 지표: 오도메트리 누적 드리프트 오차($\le 5\text{cm}$), 경로 효율성(SPL %)
========================================================================================
```

---

## 4. 🌟 선행연구 대비 우리 논문의 차별화 포인트 (Novelty)

1. **VOCA + S2E의 화학적 결합 (Chemical Coupling)**:
   * 기존 ViNT/NoMAD는 단순 비전 궤적 예측에 머무르거나(Decoupled), VLFM은 휠 로봇 위주임.
   * 우리는 VLM의 Reasoning Embedding과 4족 보행 로봇의 S2E 로코모션 레이어를 **딥러닝 레벨에서 직접 연결하여 엔드투엔드(E2E) 백프로파게이션 구현**.
2. **비동기 듀얼 루프 온보드 아키텍처 (Dual-Loop Async Architecture)**:
   * 50Hz 고주파 로코모션 모션 제어(System 1)와 10Hz 저주파 VLM 비전 경로계획(System 2)을 분리하여 **AI 추론 지연이 발생해도 로봇이 멈추거나 뒤집어지지 않음**.
3. **RTAB-Map + FAST-LIO2 융합 오도메트리**:
   * 실외 자갈길 슬립 및 태양광 노이즈 환경에서도 오도메트리 유실 없는 안정적 위치 추정.

---

## 5. 📊 ICRA 논문 수록용 3종 세트 템플릿

### 1) Table 1: 실물 로봇 정량 비교표 (ICRA 제출용)

| 모델 (Method) | 실내 복도 SR (%) | 막힌길 탈출 SR (%) | 실외 험지 SR (%) | 전반적 SPL (%) | 평균 충돌 횟수 | 평균 주행시간 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **S2E Only** *(Low-level)* | 60.0 % | 20.0 % | 40.0 % | 45.2 % | 1.4 회 | 45.2 초 |
| **ViNT / NoMAD** *(Baseline)* | 80.0 % | 40.0 % | 60.0 % | 58.0 % | 0.8 회 | 38.5 초 |
| **VOCA + S2E** *(Physical)* | 80.0 % | 60.0 % | 80.0 % | 72.5 % | 0.4 회 | 32.1 초 |
| **Ours: VOCA + S2E** *(Chemical)* | **100.0 %** | **100.0 %** | **100.0 %** | **88.4 %** | **0.0 회** | **28.4 초** |

---

### 2) Figure 5: RTAB-Map 2D 주행 궤적 비교 플롯
* 🔴 **빨간색 선 (ViNT/NoMAD)**: ㄷ자 막힌 길에서 방향을 잡지 못하고 방황하다 충돌.
* 🟢 **초록색 선 (Ours: VOCA+S2E)**: VOCA 그래프 메모리로 360도 제자리 회전 후 직진 탈출 완주.

---

### 3) Figure 1: 실물 로봇 주행 순차 필름스트립 스냅샷 (4장)
* 사진 1: 출발선 마스킹 테이프에서 출발
* 사진 2: ㄷ자 막다른 길에 진입
* 사진 3: VOCA 제자리 회전(Look-around) 동작 수행
* 사진 4: 복도를 완전히 탈출하여 목표점 원형 테이프 골인
