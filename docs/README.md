# ESCAPE-Nav 문서 시작점

> 최종 상태 갱신: 2026-08-30 KST
> 판정 기준: 현재 런타임 → 현재 소스/설정 → canonical repository → 실제 경로를 통과한 시험 → 과거 문서 순으로 신뢰한다.

## 지금 가장 먼저 볼 문서

| 우선순위 | 문서 | 용도 |
|---:|---|---|
| **⭐ 0** | [`시스템 브링업 & 운영 SOP 마스터 허브`](../system_bringup/README.md) | **부팅·핫스팟·5단계 SOP·PixNav 분석 및 트러블슈팅 총괄** |
| 1 | [`실로봇 전체 E2E 마스터 계획`](./experiments/00_real_robot_end_to_end_master_test_plan.md) | 센서→3DoF map→localization→PixelNav/S2E→4-Tier→안전→pilot→논문 campaign |
| 2 | [`00_CURRENT_STATUS_AND_NEXT_STEPS.md`](./00_CURRENT_STATUS_AND_NEXT_STEPS.md) | RTAB-Map 최신 실측과 바로 다음 작업 |
| 3 | [`4-Tier 최신 구현률`](./experiments/07_4tier_robot_jetson_docker_server_readiness.md) | Tier별 준비도, Gate 완료도, 8월 30일 Jetson-only 검증 |
| 4 | [`RTAB-Map fact-first 검증 계획`](./experiments/08_rtabmap_livo_fact_first_validation_plan.md) | 현재값 고정→짧은 loop→원인별 단일변수 A/B |
| 5 | [`4-Tier 실측 감사 및 ICRA 2027 실험 프로토콜`](./master_plan/[2026-08-27]_Robot_Jetson_Docker_Server_4Tier_실측감사_및_ICRA2027_실로봇_실험프로토콜.md) | paired campaign 원칙 |
| 6 | [`CODEX_PROJECT_MEMORY.md`](./CODEX_PROJECT_MEMORY.md) | 장기 프로젝트 사실·안전·acceptance 기준 |

## 현재 한눈에 보기

| 구간 | 현재 상태 | 다음 게이트 |
|---|---|---|
| Go2/L2 | 과거 실제 cloud/IMU/LIO/RGB 수신 PASS | 각 실험일 전원 후 rate/timestamp preflight |
| RTAB-Map | 최신 short loop: Z 0.0235 m, Type-1 2개, start/end 0.0335 m; 반복성 1/3 | short loop 2회 추가→golden map→localization |
| Jetson | PixNav/P7/P8-A no-actuation 86 tests PASS; RTAB 경로 존재 | golden map/localization, live 10분, P8-B |
| Jetson↔Docker | 임시 비제어 포트 양방향 UDP PASS | production bridge 대신 command sink 시험 |
| Docker/PixelNav | Docker node는 mock; frozen PixNav는 Jetson Checkpoint_A로 live 1-cycle PASS | 실제 clip 20개와 10분 상주 service |
| Docker↔Server | model 조회, text JSON, 보관 RGB vision 요청 PASS | live frame + 전체 navigation schema + provenance |
| 물리 자율주행 | **NO-GO** | P8-B 단일 dispatcher·physical E-stop·safe-stop 10/10 |

## 현재 mapping 명령

```bash
cd /home/unitree/go2_ws_antarctica

./run_mapping.sh          # 3D/2D 복도 매핑 (헤드리스 기본)
./run_mapping.sh --view   # Jetson 데스크톱 GUI 점군 뷰어 동시 활성화
```

사용자용 최신 브링업 및 자율주행 파이프라인은 [`../system_bringup/README.md`](../system_bringup/README.md)에 집대성되어 있습니다.

## 폴더 지도

- [`experiments/`](./experiments/): 실로봇 전체 E2E Gate와 논문 campaign 계획
- [`master_plan/`](./master_plan/): 현재 총괄 계획과 과거 계획 아카이브
- [`troubleshooting/`](./troubleshooting/): 문제 증거, 원인, 해결 및 재검증
- [`jetson_plan/`](./jetson_plan/): Foxy host·DDS·RTAB-Map·host bridge 관련 문서
- [`docker_plan/`](./docker_plan/): Jazzy/S2E/container 검증 문서
- [`docker/`](./docker/): Docker 아키텍처와 시각화 자료
- [`guides/`](./guides/): DB와 지도 확인 도구

## 문서 상태 표시를 읽는 법

- **MEASURED**: 표시된 시각에 실제 명령·데이터로 확인
- **PARTIAL**: 일부 실제 경로만 확인, 전체 폐루프 주장은 불가
- **PROPOSED**: 아직 소스 적용 또는 실주행 검증 전
- **HISTORICAL**: 과거 계획·추정. 실행 지침으로 사용하지 않음
- **NO-GO**: 물리 자율주행 금지

과거 문서의 `완벽`, `100%`, `ALL PASS`, `50 Hz RTAB-Map`, `Production-ready` 문구는 현재 acceptance 근거가 아니다. 위 우선 문서와 실제 소스·런타임이 항상 우선한다.
