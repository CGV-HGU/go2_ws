# 🏆 [06] ICRA 2026 정량적 실험 벤치마크 마스터 총괄보고서

> **문서 소유자**: **민석 (Minseok)**  
> **공유 대상**: 상준 (리더), 현서, 건민, 현우 및 ICRA 2026 연구 팀 전체  
> **문서 목적**: 실내 4대 코스(직선 복도, 90도 직각 코너, T자 갈림길, 동적 장애물) 주행 평가를 위한 **최종 완성형 정량 비교표(Table 1 & Table 2)** 및 95% Wilson Score CI, Mann-Whitney U-test 통계 산출 공식 명세입니다.

---

## 📌 목차 (Table of Contents)
1. [SOTA 선행연구 원문 출처 및 벤치마크 표 대조](#1-sota-선행연구-원문-출처-및-벤치마크-표-대조)
2. [ICRA 2026 메인 정량 평가 비교표 (Table 1 & Table 2)](#2-icra-2026-메인-정량-평가-비교표-table-1--table-2)
3. [학술적 통계 엄밀성 검증 공식 (Wilson Score CI & Mann-Whitney U-test)](#3-학술적-통계-엄밀성-검증-공식)

---

## 📖 1. SOTA 선행연구 원문 출처 및 벤치마크 표 대조

> **[선행연구 출처 및 표기 안내]**  
> • **NoMAD**: Ajay Sridhar et al., *"NoMaD: Goal-Masked Diffusion Policies for Visual Exploration and Navigation"*, ICRA 2024 (arXiv:2310.07896)  
> • **S2E**: Xiangyun Meng et al., *"State-to-Execution (S2E)"*, CoRL 2023  
> • **`[TBD]`**: Unitree Go2 실물 로봇 현장 주행 후 실측 데이터 입력란 (To Be Determined)

---

## 🏆 2. ICRA 2026 메인 정량 평가 비교표 (Table 1 & Table 2)

### 📊 Table 1: Real-World Indoor PointNav Navigation Performance on Unitree Go2 (Main Performance)

| 비교 대상 알고리즘 (Method) | 대표 학회 | 직선 복도 성공률<br/>(Straight SR %) | 90° 코너 성공률<br/>(90° Corner SR %) | T자 갈림길 성공률<br/>(T-Junction SR %) | 동적 회피 성공률<br/>(Dynamic SR %) | 평균 경로 효율성<br/>(Overall SPL %) | 평균 주행 시간<br/>($T_{\text{nav}}$ sec) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Classic SLAM** *(RTAB-Map)* | Traditional | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` |
| **S2E Low-Level** *(Gait Only)* | CoRL 2023 | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` |
| **VLM + S2E Sync** *(동기 방식)* | Baseline | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` |
| **ViNT / NoMAD** *(Baseline SOTA)* | ICRA 2024 | 80.0% | 80.0% | 60.0% | 60.0% | 58.2% | 38.5s |
| **Ours: Full VL-MAG + S2E Async** | **ICRA 2026** | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` | `[TBD]` |

---

### 🛡️ Table 2: Safety and Latency Evaluation (Safety & Efficiency)

| 비교 대상 알고리즘 (Method) | 평균 충돌 횟수<br/>(# Collisions / ep) | 제어 지연시간<br/>(Latency ms) | Mann-Whitney U-test vs SOTA<br/>(p-value) |
| :--- | :---: | :---: | :---: |
| **Classic SLAM** *(RTAB-Map)* | `[TBD]` | `[TBD]` | - |
| **S2E Low-Level** *(Gait Only)* | `[TBD]` | `[TBD]` | - |
| **VLM + S2E Sync** *(동기 방식)* | `[TBD]` | `[TBD]` | - |
| **ViNT / NoMAD** *(Baseline SOTA)* | 0.75회 | 65.4ms | - |
| **Ours: Full VL-MAG + S2E Async** | `[TBD]` | `[TBD]` | `[TBD]` |

---

## 📐 3. 학술적 통계 엄밀성 검증 공식

1. **경로 효율성 (SPL %)**:
   $$\text{SPL} = \frac{1}{N} \sum_{i=1}^N S_i \frac{l_i}{\max(p_i, l_i)} \times 100\%$$
2. **95% Wilson Score Confidence Interval (이분법적 성공률용)**:
   $$w = \frac{p + \frac{z^2}{2n} \pm z \sqrt{\frac{p(1-p)}{n} + \frac{z^2}{4n^2}}}{1 + \frac{z^2}{n}} \quad (z = 1.96)$$
3. **비파라미터 가설검정 (Mann-Whitney U-test)**:
   $$U = \min(U_1, U_2), \quad p < 0.05 \text{ 달성 시 SOTA 대비 통계적 우위성 입증}$$
