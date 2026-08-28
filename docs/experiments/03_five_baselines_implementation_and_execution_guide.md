# 실로봇 비교 방법 정의 및 구현 준비도

> 개정: 2026-08-28 KST
> 파일명은 과거 링크 호환을 위해 유지함
> 현재 main 비교: Direct-goal PixNav vs Full ESCAPE-PixNav
> 물리 실행: Gate 0~8 통과 전 금지

## 1. Main paired comparison

main 실로봇 표에는 구현·안전·logging이 동일한 두 방법만 넣는다.

| 항목 | Direct-goal PixNav | Full ESCAPE-PixNav |
|---|---|---|
| 공통 frozen backend | 같은 PixNav Checkpoint_A, action adapter, localization, VLM endpoint | 동일 |
| coarse goal | 직접 goal pursuit | VL-MAG가 선택한 local goal |
| adaptive observation | OFF | ON |
| causal admission/warping | 논문 정의에 따라 고정된 baseline behavior | ON |
| directional outcome memory | OFF | ON |
| speed/safety limits | 동일 | 동일 |
| map/DB와 calibration | 동일 | 동일 |
| run 수 | 5 pairs × 5 reps = 25 | 5 pairs × 5 reps = 25 |

두 main 방법 모두 Nav2 planner/controller를 사용하지 않는다. Nav2를 한 방법에만 추가하면 low-level navigation stack이 달라져 async VLM mechanism 비교가 아니게 된다. RTAB-Map localization은 두 방법에 동일하게 map-frame pose를 제공한다.

두 방법의 차이는 논문이 비교하려는 mechanism으로 제한한다. 한쪽에만 다른 checkpoint, 속도 제한, obstacle API 또는 map을 적용하면 campaign을 다시 시작한다.

## 2. 구현 acceptance

각 method는 final campaign 전에 다음을 만족해야 한다.

- 존재하는 package executable로 시작
- mock backend OFF와 provenance 표시
- 실제 Go2 camera/LIO 입력
- 실제 PixNav Checkpoint_A SHA-256 고정
- 모든 VLM submit/complete/apply/reject identity 기록
- 같은 controller와 단일 Go2 command gateway 사용
- timeout/server loss/odom loss에서 동일한 fail-closed safety 적용
- command sink, fault injection, actuator safe-stop Gate PASS

현재 이 조건은 두 method 모두 충족하지 않았으므로 실행 명령을 제공하지 않는다.

## 3. 과거 5개 baseline 후보의 상태

| 후보 | 로컬 구현 확인 | 모델/checkpoint | 실물 command path | main table 사용 |
|---|---|---|---|---|
| Classic Nav2/SLAM | 검증된 Nav2 planner launch 없음 | 해당 없음 | 미검증 | **NO** |
| S2E gait-only | 실제 정책 node 미확인 | 실제 S2E 없음 | 미검증 | **NO** |
| VLM+S2E Sync | method switch/실행 artifact 없음 | 실제 S2E 없음 | 미검증 | **NO** |
| ViNT/NoMAD | source 후보만 존재 | Go2용 checkpoint/config 미검증 | 없음 | **NO** |
| Full ESCAPE-PixNav | mock graph와 설계 골격 존재 | 공식 checkpoint hash 및 Jetson CUDA 11-frame file-only replay PASS | adapter/통합/안전 미검증 | main 후보, 현재 NO-GO |

과거 문서에 있던 ViNT/NoMAD의 `80/80/60/60`, `SPL 58.2%`, `38.5s`, `0.75 collision`, `65.4ms` 값은 이 Go2, 이 map, 이 goal과 같은 조건에서 측정된 값이 아니므로 삭제했다.

## 4. Optional extension baseline을 추가하는 조건

Classic Nav2, Sync, ViNT/NoMAD 등을 확장 표에 추가하려면 각 방법에 대해 다음을 별도로 통과해야 한다.

1. 정확한 upstream commit과 license 기록
2. checkpoint와 preprocessing hash 고정
3. Go2 sensor/actuator adapter 검증
4. 같은 localization, success radius, timeout, speed/safety 사용
5. pilot 최소 3회에서 collision/intervention 0
6. main과 동일한 pair/order/repetition protocol
7. 실제 artifact importer로 complete run 생성

문헌 숫자를 가져와 로컬 실측 행처럼 넣지 않는다. 재현하지 못한 방법은 관련 연구 설명에만 두고 quantitative row에서 제외한다.

## 5. Method manifest

각 run은 최소한 다음 method identity를 기록한다.

```json
{
  "method": "Direct-goal PixNav|Full ESCAPE-PixNav",
  "method_version": "...",
  "enabled_mechanisms": {
    "adaptive_observation": false,
    "causal_warping": false,
    "directional_memory": false
  },
  "pixnav_checkpoint_sha256": "...",
  "vlm_model": "...",
  "prompt_schema_sha256": "...",
  "controller_config_sha256": "...",
  "rtabmap_db_sha256": "..."
}
```

표의 method 이름만 다르고 실제 enabled mechanism이 같거나, 반대로 method 이름은 같지만 config hash가 다르면 importer가 run을 거부해야 한다.
