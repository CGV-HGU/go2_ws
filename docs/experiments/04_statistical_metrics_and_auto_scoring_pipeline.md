# 실로봇 지표·통계·테이블 생성 파이프라인 명세

> 개정: 2026-08-28 KST
> 현재 상태: 계산 규격만 확정 / 실제 artifact importer 미구현
> 주의: `scratch/calculate_icra_metrics.py`는 sample episode 기반이므로 논문 결과 생성에 사용하지 않음
> table schema: [`01_table1_table2_quantitative_experiment_master_protocol.md`](01_table1_table2_quantitative_experiment_master_protocol.md)

## 1. Main table 지표

| 지표 | 정의 | 실패 run 처리 |
|---|---|---|
| SR | success run / 전체 run | 실패를 denominator에 포함 |
| Intv. | intervention count/run | intervention run은 success=0 |
| Time `T†` | timeout-normalized completion time | 공통 `T_max` 부여 |
| Rec. | successful recovery / triggered recovery | event identity가 연결된 경우만 계산 |
| Lat. | run별 VLM dispatch→parse completion 평균 | timeout도 별도 event로 보존 |
| Duty | active base motion / wall time | 실패 run도 포함 |
| Yield | applied / completed VLM decisions | rejected completion도 denominator에 포함 |

### 1.1 Success와 시간

```text
success = within_goal_radius
          AND within_timeout
          AND interventions == 0
          AND collisions == 0
          AND stop_reason == "goal_reached"

T† = success·min(wall_time, T_max) + (1-success)·T_max
```

main campaign goal radius는 1.0 m로 고정한다.

### 1.2 Duty와 Yield

```text
Duty  = active_motion_duration / episode_wall_time
Yield = applied_decisions / completed_decisions
```

active-motion threshold, command source, monotonic clock와 decision state machine은 campaign manifest에 기록한다.

## 2. 필수 입력 artifact

테이블 생성기는 rosbag 하나만 읽어서는 안 된다. 다음 파일을 함께 검증해야 한다.

```text
run_manifest.json
result.json
localization.csv
trajectory.csv
vlm_events.jsonl
s2e_events.jsonl
command_watchdog.csv
interventions.jsonl
SHA256SUMS
```

최소 `result.json` schema:

```json
{
  "schema_version": "real_robot_result_v1",
  "campaign_id": "...",
  "pair_id": "P1",
  "paired_block_id": "P1-R1",
  "method_order": "A_first|B_first",
  "method": "Direct-goal|Full ESCAPE-Nav",
  "repetition": 1,
  "success": false,
  "stop_reason": "goal_reached|timeout|collision|e_stop|intervention|system_fault",
  "within_goal_radius": false,
  "goal_distance_m": 0.0,
  "goal_reference": "surveyed_floor_marker+operator_video",
  "goal_radius_m": 1.0,
  "wall_time_s": 0.0,
  "timeout_s": 0.0,
  "interventions": 0,
  "collisions": 0,
  "recoveries_triggered": 0,
  "recoveries_successful": 0,
  "active_motion_time_s": 0.0,
  "completed_decisions": 0,
  "applied_decisions": 0,
  "complete": false
}
```

`complete=true`는 required artifact와 hash가 모두 존재하고 cross-check를 통과했을 때 importer만 설정한다.

## 3. Import validation

run별로 다음을 검증한다.

1. campaign/pair/paired-block/method/order/repetition이 preregistration과 일치
2. 동일 run identity 중복 없음
3. config/model/map/code hash가 frozen manifest와 일치
4. monotonic event timestamp 역행 없음
5. VLM submit/completion/apply/reject identity 연결
6. completed decision 수와 apply+reject 수 일치
7. trajectory와 command가 같은 source decision을 참조
8. intervention/collision/E-stop event와 stop reason 일치
9. wall time, active time, latency가 음수/NaN/Inf가 아님
10. bag/video/log/hash 누락 없음

하나라도 실패하면 해당 run을 자동으로 삭제하지 않고 `complete=false`와 이유를 exclusion audit에 기록한다.

## 4. 통계

### 4.1 방법별 요약

- SR: `k/n`, percentage, Wilson 95% interval
- count 지표: total과 mean/run을 함께 표기
- 연속 지표: mean±SD를 기본으로 하고 median[IQR]을 보조 제공
- recovery와 yield: numerator/denominator를 반드시 보존

### 4.2 방법 간 비교

main design은 같은 5개 pair에서 두 방법을 5개 paired block으로 반복하는 paired/block design이다. 각 block의 AB/BA 선행 순서는 사전 생성하며 홀수 반복 때문에 생기는 불균형은 pair별 최대 1회로 제한한다.

- pair를 상위 resampling unit, pair 내부 paired block을 하위 단위로 한 hierarchical paired bootstrap difference와 95% interval을 우선한다.
- 명확히 matched된 run slot이 있을 때 연속형 지표에 Wilcoxon signed-rank를 보조로 사용할 수 있다.
- binary paired outcome에 정당한 1:1 matching이 있을 때만 McNemar exact test를 고려한다.
- 독립표본 Mann–Whitney U-test를 “SOTA 대비 필수 p-value”로 사용하지 않는다.
- 문헌에 보고된 ViNT/NoMAD 숫자와 이 campaign raw runs 사이의 p-value를 계산하지 않는다.

작은 실로봇 표본에서는 raw denominator와 effect interval이 p-value보다 우선한다.

## 5. 보조 진단 지표

다음은 main table과 분리한다.

- collision, near-miss, minimum clearance
- path length와 SPL
- failed-edge re-entry/opportunity
- localization loss와 correction jump
- RTAB Type-1/Type-2 closure timestamp
- command age, watchdog stop latency
- Jetson thermal/power와 packet loss

SPL은 독립 reference의 shortest path와 실제 path가 모두 신뢰할 수 있을 때만 계산한다. 정책이 사용하는 RTAB-Map pose를 독립 ground truth로 재사용하지 않는다.

## 6. 출력 파일

실제 importer가 완성되면 다음을 생성한다.

```text
campaign_validation_report.json
run_level_results.csv
pair_level_results.csv
table_real_robot_quantitative.tex
table_real_robot_deployment.tex
table_real_robot_safety.tex
exclusions.csv
SHA256SUMS
```

LaTeX 표의 모든 셀은 run-level CSV에서 재생성 가능해야 한다. 수기로 숫자를 넣지 않는다.

## 7. 현재 evaluator 처리

현재 `scratch/calculate_icra_metrics.py`는 다음 이유로 acceptance 도구가 아니다.

- main 함수가 4개의 hard-coded sample episode를 생성
- rosbag/result/event artifact를 읽지 않음
- campaign completeness와 hash를 확인하지 않음
- pilot/final campaign을 구분하지 않음

따라서 실제 importer 구현 전까지 이 스크립트 출력은 demo로만 취급하고 논문 표·그래프·통계에 사용하지 않는다.
