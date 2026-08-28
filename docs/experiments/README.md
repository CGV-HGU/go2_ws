# 실로봇 End-to-End 실험 계획 허브

> 실측 개정: 2026-08-28 KST
> 현재 판정: planar/global-loop 기능 PASS, 최신 전체-map geometry FAIL(Type-2 proximity가 주원인) / PixNav v2 file-only·adapter·offline fault PASS / remap·localization·live chain 미검증 / physical autonomy NO-GO
> 판정 원칙: 실제 소스·checkpoint·runtime artifact가 과거 문서의 “완성/Final” 표현보다 우선함

## 문서 우선순위

| 우선순위 | 문서 | 용도 | 현재 상태 |
|---:|---|---|---|
| 1 | [실로봇 전체 E2E 마스터 계획](00_real_robot_end_to_end_master_test_plan.md) | 센서→3DoF map→localization→frozen PixNav→4-Tier→안전→pilot→50-run campaign | **Authoritative / Active** |
| 2 | [매핑 복귀 직후 PixNav 무구동 자격시험](04_post_mapping_pixnav_zero_actuation_qualification.md) | 논문 고정 PixNav 구현+Checkpoint_A→실 RGB replay→파일 evidence | **Authoritative / Active** |
| 2.5 | [PixNav→live 4-Tier 구현 계획](05_pixnav_live_chain_implementation_plan.md) | 완료된 file-only 계층과 다음 live/safety 구현 Gate | **Implementation active** |
| 2.6 | [PixNav 구성요소별 위치·필요 이유와 무구동 시험](06_pixnav_components_where_why_and_robot_free_tests.md) | 파일별 역할, 단계별 필요성, quick/evidence/CUDA 실행법 | **Operator guide** |
| 3 | [4-Tier 실측 감사 및 ICRA 2027 프로토콜](../master_plan/[2026-08-27]_Robot_Jetson_Docker_Server_4Tier_실측감사_및_ICRA2027_실로봇_실험프로토콜.md) | 현재 구현 준비도와 paired campaign 원칙 | **Authoritative audit** |
| 4 | [RTAB-Map 실물 검증계획](../07_real_robot_sensor_and_autonomy_verification_plan.md) | 센서·map·global loop 세부 기준 | **Active** |
| 5 | [실로봇 정량 테이블 규격](01_table1_table2_quantitative_experiment_master_protocol.md) | Direct-goal vs Full, 25회/method, main/deployment/safety table | **Schema active / 값 미측정** |
| 6 | [4개 arena 초안](02_four_arenas_physical_setup_and_evaluation_guide.md) | 확장 deployment 후보와 측량 양식 | **Draft / 좌표 미검증** |
| 7 | [비교 방법 정의](03_five_baselines_implementation_and_execution_guide.md) | main 두 방법의 공정성 계약과 optional baseline 준비도 | **Definition active / 실행 NO-GO** |
| 8 | [지표·통계·테이블 생성 명세](04_statistical_metrics_and_auto_scoring_pipeline.md) | artifact schema, paired 통계, importer acceptance | **Schema active / importer 미구현** |

## 전체 실행 순서

```text
버전·안전 동결
  → 실센서 preflight
  → planar 3DoF golden map
  → map localization
  → frozen PixNav capture-view/post-capture real-RGB file-only replay
  → live 4-Tier command sink
  → fault injection
  → actuator/E-stop
  → 저속 pilot
  → 50-run paired campaign
```

각 단계의 합격 기준과 현재 상태는 [실로봇 전체 E2E 마스터 계획](00_real_robot_end_to_end_master_test_plan.md)을 따른다.

## 현재 금지 사항

- `bringup_all_escape_nav.sh` autonomy mode로 실제 로봇 이동
- mock `e2e_node/controller_node` 결과를 실로봇 PASS로 기록
- checkpoint hash가 없는 PixNav 결과 사용
- S2E 보조 실험을 현재 PixNav 실로봇 backend 검증으로 대체 표기
- sample episode가 내장된 `calculate_icra_metrics.py` 출력을 논문 수치로 사용
- `/cmd_vel`과 Sport API를 동시에 command authority로 사용
- 사람을 동적 장애물로 투입하는 시험을 안전 Gate보다 먼저 수행

mapping 동안 recorder는 OFF를 유지한다. 논문 campaign에서는 실제 topic과 provenance가 수정된 전용 recorder가 준비된 뒤에만 기록을 시작한다.
