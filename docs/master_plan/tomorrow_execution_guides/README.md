# 📂 [Tomorrow Execution Guides Hub] 내일 연구실 4대 Phase별 상세 실행 가이드 허브

> **작성 일자**: 2026년 8월 27일 (목요일) 21:40 KST  
> **허브 목적**: 내일 연구실 현장에서 진행될 **[Phase 0 ➔ Phase 1 ➔ Phase 2 ➔ Phase 3 ➔ Phase 4]의 각 단계별 세부 실행 가이드 및 트러블슈팅 매뉴얼을 1:1로 직결 제공하는 전용 허브**입니다.

> **2026-08-28 실행 상태**: Phase 2~4의 기존 원클릭 명령은 아직 검증되지 않아 그대로 실행하면 안 된다. 실제 순서는 [`../../experiments/00_real_robot_end_to_end_master_test_plan.md`](../../experiments/00_real_robot_end_to_end_master_test_plan.md)의 Gate 0~9를 따르며, main campaign은 `Direct-goal`과 `Full ESCAPE-Nav`의 총 50회 paired test다.

---

## 🧭 4대 Phase별 상세 가이드 문서 목록

| 단계 (Phase) | 가이드 문서명 | 핵심 실행 내용 및 목표 | 링크 |
| :---: | :--- | :--- | :---: |
| **Phase 0** | **`00_phase0_arrival_preflight_and_network_healthcheck_guide.md`** | • 연구실 도착 직후 5분 퀵 점검<br/>• 배터리, 젯슨 자원, 4-Tier 핑(Go2/GPU서버) 무결성 검증 | 👉 **[Phase 0 가이드 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/tomorrow_execution_guides/00_phase0_arrival_preflight_and_network_healthcheck_guide.md)** |
| **Phase 1** | **`01_phase1_planar_3dof_golden_mapping_and_freeze_guide.md`** | • 평면 3DoF 맵핑 실측 (`./mapping_planar_headless.sh`)<br/>• Z축 변동 $< 5\text{cm}$ 수렴 확인 & 골든 맵 영구 동결 | 👉 **[Phase 1 가이드 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/tomorrow_execution_guides/01_phase1_planar_3dof_golden_mapping_and_freeze_guide.md)** |
| **Phase 2** | **`02_phase2_docker_s2e_zero_actuation_dryrun_and_safety_guide.md`** | • 도커 S2E 무구동 가상 폐루프 검증<br/>• Qwen3.5-9B 서브골 수신 ➔ 50Hz 궤적 파일 로깅 & 0속도 인터록 | 👉 **[Phase 2 가이드 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/tomorrow_execution_guides/02_phase2_docker_s2e_zero_actuation_dryrun_and_safety_guide.md)** |
| **Phase 3** | **`03_phase3_180m_corridor_5_scenarios_autonomous_driving_guide.md`** | **보류**: Gate 0~8 통과 뒤 승인된 5개 pair로 재작성 필요 | [Phase 3 초안](03_phase3_180m_corridor_5_scenarios_autonomous_driving_guide.md) |
| **Phase 4** | **`04_phase4_paper_table_latex_auto_scoring_and_dataset_export_guide.md`** | **BLOCKED**: 실제 artifact importer 구현 전 sample evaluator 사용 금지 | [Phase 4 상태/계약](04_phase4_paper_table_latex_auto_scoring_and_dataset_export_guide.md) |

---

## 🔗 상위 마스터 문서 바로가기
* **내일 연구실 종합 마스터 SOP**: [`[2026-08-28_내일_연구실_현장_실물실증_완전정복_1-Click_체크리스트_및_SOP].md`](../%5B2026-08-28_%EB%82%B4%EC%9D%BC_%EC%97%B0%EA%B5%AC%EC%8B%A4_%ED%98%84%EC%9E%A5_%EC%8B%A4%EB%AC%BC%EC%8B%A4%EC%A6%9D_%EC%99%84%EC%A0%84%EC%A0%95%EB%B3%B5_1-Click_%EC%B2%B4%ED%81%AC%EB%A6%AC%EC%8A%A4%ED%8A%B8_%EB%B0%8F_SOP%5D.md)
* **마스터 플랜 메인 인덱스**: [`docs/master_plan/README.md`](../README.md)
