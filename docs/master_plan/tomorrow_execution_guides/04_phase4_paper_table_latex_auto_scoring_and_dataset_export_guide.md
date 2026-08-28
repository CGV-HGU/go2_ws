# Phase 4: 실로봇 논문 테이블 생성 및 데이터 export 가이드

> 실측 개정: 2026-08-28 KST
> 현재 상태: **BLOCKED — 실제 artifact importer 미구현**
> table 규격: [`../../experiments/01_table1_table2_quantitative_experiment_master_protocol.md`](../../experiments/01_table1_table2_quantitative_experiment_master_protocol.md)
> 지표/import 규격: [`../../experiments/04_statistical_metrics_and_auto_scoring_pipeline.md`](../../experiments/04_statistical_metrics_and_auto_scoring_pipeline.md)

## 1. 현재 실행하면 안 되는 명령

```bash
python3 scratch/calculate_icra_metrics.py
```

현재 스크립트는 rosbag이나 run artifact를 읽지 않고 코드 내부의 4개 sample episode를 계산한다. 출력되는 SR, Time, Duty, Yield와 LaTeX는 실측 결과가 아니므로 논문·보고서·발표자료에 사용하지 않는다.

## 2. 확정된 main table

```text
5 fixed pairs × 2 methods × 5 repetitions
= 25 runs/method
= 50 main runs
```

비교 방법은 `Direct-goal`과 `Full ESCAPE-Nav`다. main table 열은 다음과 같다.

| Method | SR | Intv. | Time `T†` | Rec. | Lat. | Duty | Yield |
|---|---|---|---|---|---|---|---|
| Direct-goal | -- | -- | -- | -- | -- | -- | -- |
| Full ESCAPE-Nav | -- | -- | -- | -- | -- | -- | -- |

모든 값은 final campaign artifact가 준비될 때까지 `--`로 유지한다.

## 3. Table 생성 전 필수 입력

각 50개 run에는 다음이 있어야 한다.

```text
run_manifest.json
result.json
localization.csv
trajectory.csv
vlm_events.jsonl
s2e_events.jsonl
command_watchdog.csv
interventions.jsonl
operator_video.mp4
rosbag2/
SHA256SUMS
```

pilot, tuning, incomplete run을 final campaign 디렉터리와 섞지 않는다.

## 4. 향후 실제 생성 절차

artifact importer가 구현되면 다음 순서로 실행한다.

1. preregistered pair/method/repetition과 50개 run identity 대조
2. config/model/map/code hash freeze 확인
3. event timestamp, decision identity, stop reason 교차검증
4. incomplete/excluded run을 삭제하지 않고 audit log 생성
5. run-level과 pair-level CSV 생성
6. main/deployment/safety LaTeX table 생성
7. CSV에서 LaTeX의 모든 셀을 재계산해 round-trip 검증
8. 최종 artifact SHA-256 생성

예정 출력:

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

## 5. 합격 체크리스트

- [ ] Gate 0~8 통과와 final config freeze
- [ ] main run 50/50 complete
- [ ] failure, timeout, collision, E-stop, intervention 누락 0
- [ ] SR은 `k/25`와 percentage 동시 표기
- [ ] 실패 run의 Time은 `T_max`로 처리
- [ ] recovery와 yield numerator/denominator 보존
- [ ] pair/paired-block hierarchical bootstrap interval 생성
- [ ] pilot 수치가 main table에 없음
- [ ] 문헌 baseline 숫자가 로컬 실측 행에 없음
- [ ] 수기 입력 없이 CSV에서 LaTeX 재생성 가능

Git commit과 push는 데이터 소유자가 결과를 검토한 뒤 별도로 수행한다. 이 가이드는 자동 commit/push를 실행하지 않는다.
