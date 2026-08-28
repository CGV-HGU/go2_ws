# Go2 실로봇 정량 테이블 규격 및 campaign 프로토콜

> 개정: 2026-08-28 KST
> 파일명은 과거 링크 호환을 위해 유지함
> 현재 상태: table schema 확정 / 실측값 없음 / recorder·importer 미완료
> 전체 Gate: [`00_real_robot_end_to_end_master_test_plan.md`](00_real_robot_end_to_end_master_test_plan.md)

## 1. 기존 초안에서 교정한 내용

| 기존 항목 | 문제 | 교정 |
|---|---|---|
| 4 arena × 5 methods를 main table로 사용 | 실제 구현되지 않은 baseline이 포함되고 현재 paired paper protocol과 불일치 | main은 Direct-goal vs Full ESCAPE-Nav 두 방법만 사용 |
| ViNT/NoMAD에 80/80/60/60, SPL 58.2, latency 65.4 ms 입력 | 이 Go2와 이 코스에서 측정한 값이 아니며 출처·조건이 동일하지 않음 | 모두 제거. 재현 실측하지 않은 baseline 숫자는 표에 넣지 않음 |
| `T_nav`를 성공 run 평균으로만 계산 | 실패/timeout을 제외해 성능을 과대평가 | 실패에 공통 `T_max`를 부여하는 `T†` 사용 |
| success radius 0.8 m와 1.0 m 혼용 | trial 판정이 campaign 중 바뀔 수 있음 | main campaign은 1.0 m로 사전 고정 |
| 동적 장애물에서 “정지하면 실패” | 안전 정지가 오히려 실패로 처리됨 | 접촉 없이 goal 도달을 success로 판정하고 연속성은 Duty로 측정 |
| Mann–Whitney p-value 열을 main table에 강제 | paired design과 작은 실로봇 표본에 부적절 | raw denominator와 pair-aware interval을 우선, 검정은 보조 분석 |
| 1-Click evaluator가 실측 표를 만든다고 설명 | 현재 evaluator는 코드 내부 sample episode를 사용 | 실제 artifact importer가 완성될 때까지 자동 표 생성 금지 |

## 2. Main real-robot table

실로봇 main 비교는 같은 고정 map과 같은 start–goal pair에서 두 방법을 비교한다.

- `Direct-goal`: active observation/directional memory를 우회하는 동일 frozen backend
- `Full ESCAPE-Nav`: 최종 고정 시스템
- `P=5` fixed pairs
- pair마다 method별 5회
- `n=25/method`, 총 50 main runs
- 각 repetition은 두 방법을 묶은 paired block으로 정의
- AB/BA 선행 방법 수 차이가 1 이하가 되도록 사전 생성한 counterbalanced order 사용

### 2.1 논문용 빈 LaTeX template

아래 값은 실제 artifact import 전까지 `--`로 둔다. 임의 예상값이나 pilot 결과를 채우지 않는다.

```latex
\begin{table}[t]
  \centering
  \caption{Go2 paired navigation in one fixed map
  ($P=5$, five repetitions per pair and method, $n=25$/method).
  Time is timeout-normalized $T^\dagger$; Rec. is successful/triggered recovery.}
  \label{tab:real_robot_quantitative}
  \scriptsize
  \setlength{\tabcolsep}{1.35pt}
  \begin{tabular}{@{}lccccccc@{}}
    \toprule
    Method & SR $\uparrow$ & Intv. $\downarrow$ & Time $\downarrow$
      & Rec. $\uparrow$ & Lat. (s) $\downarrow$
      & Duty $\uparrow$ & Yield $\uparrow$ \\
    \midrule
    Direct-goal & -- & -- & -- & -- & -- & -- & -- \\
    \textbf{Full ESCAPE-Nav} & -- & -- & -- & -- & -- & -- & -- \\
    \bottomrule
  \end{tabular}
\end{table}
```

### 2.2 셀 형식

| 열 | 셀 형식 | 계산 단위 |
|---|---|---|
| SR | `k/25 (xx.x%)` | 성공 run / 전체 run |
| Intv. | `mean ± SD` | intervention count/run |
| Time | `mean ± SD` seconds | 모든 25 run의 `T†` |
| Rec. | `successful/triggered` | recovery event 합계 |
| Lat. | `mean ± SD` seconds | run별 VLM dispatch→response parse 평균을 25 run에서 요약 |
| Duty | `mean ± SD %` | active base motion / episode wall time |
| Yield | `applied/completed (xx.x%)` | admission된 decision / completed decision |

Direct-goal에도 VLM completion/admission 개념이 있다면 같은 정의로 Yield를 계산한다. 해당 메커니즘이 구조적으로 없을 때만 `N/A`를 쓰고, 0으로 기록하지 않는다.

## 3. 지표 정의

### 3.1 Success

다음 조건을 모두 만족해야 success다.

- 사전 측량한 물리 goal 중심의 1.0 m 원 안에 base 중심이 도달
- `T_max` 이내 도달
- 사람 또는 safety intervention 없음
- 충돌 또는 E-stop 종료 없음

intervention이 발생한 run은 intervention count에 포함하고 success는 0으로 처리한다.

goal 원은 campaign 전에 바닥 표식/측량 좌표로 고정하고 operator video로 판정한다. 정책 입력에 쓰인 RTAB-Map pose를 독립 success ground truth로 재사용하지 않는다. 연속된 한 번의 수동 takeover는 intervention 1건으로 세며, autonomy를 다시 넘긴 뒤 발생한 새 takeover만 추가 event로 센다.

### 3.2 Timeout-normalized completion time

```text
T† = S·min(T, T_max) + (1-S)·T_max
```

실패, 충돌, E-stop, timeout run을 삭제하거나 성공 run 평균에서 제외하지 않는다.

### 3.3 Recovery

trigger와 success event가 명시적으로 연결된 경우에만 `successful/triggered`로 센다. RTAB-Map Type-1/Type-2 loop closure는 VL-MAG recovery가 아니므로 Recovery 열에 포함하지 않는다.

### 3.4 Latency, Duty, Yield

- Latency: request dispatch부터 response parsing 완료까지. network RTT만 쓰지 않는다. 각 run에서 먼저 평균을 구한 뒤 method의 25개 run 평균±SD를 계산하고 request-level p50/p95는 보조 진단으로 둔다.
- Duty: `|v|`가 사전 정의된 motion threshold보다 큰 시간 / episode wall time.
- Yield: completed VLM response 중 sequence/TTL/pose-delta/schema admission을 통과해 실제 backend에 적용된 비율.

threshold와 clock source는 campaign 전에 manifest에 고정한다.

## 4. 고정 start–goal pair

아래는 경로 특성이고 좌표는 golden map과 독립 측량 후 별도 YAML에 사전 등록한다.

| Pair | 경로 특성 | 핵심 실패 모드 |
|---|---|---|
| P1 | 직선 복도 | tracking, latency, stop-and-go |
| P2 | 90° L-turn | camera FOV, corner cutting |
| P3 | T-junction/blocked bearing | branch selection |
| P4 | 반복 문·유사 복도 | failed branch re-entry |
| P5 | 다중 코너 장거리 | stale decision, localization stability |

모든 pair에 대해 다음을 사전 고정한다.

- start map coordinate와 yaw
- goal coordinate와 1.0 m radius
- `T_max`
- shortest path reference와 측정 방법
- 장애물 배치, 조명 구간, battery band
- paired block ID와 AB/BA order. 5회가 홀수이므로 pair별 선행 방법 수 차이는 최대 1로 제한하고 전체 order를 결과 확인 전에 동결

## 5. Main table과 분리할 deployment table

Active-view recovery와 dynamic obstacle은 main paired superiority 결과에 섞지 않는다.

```latex
\begin{table}[t]
  \centering
  \caption{Go2 deployment-only stress tests. These trials are not pooled
  with the paired main comparison.}
  \label{tab:real_robot_deployment}
  \scriptsize
  \begin{tabular}{@{}lccccc@{}}
    \toprule
    Condition & Runs & Success & Intv. & Collisions & Min clear. (m) \\
    \midrule
    Active-view recovery & -- & -- & -- & -- & -- \\
    Rolling obstacle/dummy & -- & -- & -- & -- & -- \\
    Approved dynamic trial & -- & -- & -- & -- & -- \\
    \bottomrule
  \end{tabular}
\end{table}
```

- 조건별 최소 5회 수행한다.
- 먼저 rolling obstacle/dummy를 사용한다.
- 사람 dynamic trial은 안전·통제 절차가 승인된 경우에만 수행한다.
- “멈추지 않음”을 success 조건으로 사용하지 않고 Duty/Time으로 보고한다.

## 6. 보조 안전·시스템 테이블

다음 표는 main 성능표와 분리해 supplement 또는 engineering report에 둔다.

| Method | Runs | Collisions | Near-misses | E-stops | Min clearance | Watchdog stop p95 | Localization losses |
|---|---:|---:|---:|---:|---:|---:|---:|
| Direct-goal | -- | -- | -- | -- | -- | -- | -- |
| Full ESCAPE-Nav | -- | -- | -- | -- | -- | -- | -- |

충돌과 near-miss는 서로 다른 event이며 합치지 않는다. minimum clearance에는 sensor/source와 유효 범위를 함께 기록한다.

## 7. 통계 보고 원칙

- SR은 반드시 `k/n`과 percentage를 함께 표기한다.
- Wilson interval은 method별 binary rate의 보조 interval로 사용할 수 있다.
- 방법 차이는 pair를 상위 resampling unit으로 하고 pair 내부 paired repetition을 하위 단위로 한 hierarchical paired bootstrap interval을 우선한다.
- 연속형 matched run-slot 분석이 정당할 때 Wilcoxon signed-rank를 보조로 사용할 수 있다.
- 임의의 ViNT/NoMAD 문헌 숫자와 이 Go2 결과를 직접 p-value로 비교하지 않는다.
- `n=25/method`에서 유의하지 않은 차이를 우월성으로 표현하지 않는다.
- exclusion은 결과 확인 전에 정의한 hardware/system fault에만 허용하고 exclusion log를 공개한다.

## 8. 테이블 채우기 전 필수 조건

- Gate 0~8 통과와 final configuration freeze
- complete run artifact 50/50
- `result.json`과 원시 event log의 일치
- 실제 artifact importer와 schema validation PASS
- failure/intervention/timeout 누락 0
- exact input, config, model, DB와 code hash 보존
- pilot run과 final campaign run 완전 분리

현재는 위 조건을 충족하지 않았으므로 모든 결과 셀은 비어 있어야 한다.
