# ESCAPE-Nav 문서 시작점

> 최종 상태 갱신: 2026-08-27 16:46 KST  
> 판정 기준: 현재 런타임 → 현재 소스/설정 → canonical repository → 실제 경로를 통과한 시험 → 과거 문서 순으로 신뢰한다.

## 지금 가장 먼저 볼 문서

| 우선순위 | 문서 | 용도 |
|---:|---|---|
| 1 | [`00_CURRENT_STATUS_AND_NEXT_STEPS.md`](./00_CURRENT_STATUS_AND_NEXT_STEPS.md) | 충전 중 가능한 무구동 작업과 충전 후 RTAB-Map 실행 순서 |
| 2 | [`4-Tier 실측 감사 및 ICRA 2027 실험 프로토콜`](./master_plan/[2026-08-27]_Robot_Jetson_Docker_Server_4Tier_실측감사_및_ICRA2027_실로봇_실험프로토콜.md) | Robot–Jetson–Docker–Server 전체 구조와 실험 게이트 |
| 3 | [`RTAB-Map 문제·원인·해결·재검증 총정리`](./master_plan/[2026-08-27]_RTAB-Map_LIVO_문제_원인_해결_및_재검증_총정리.md) | LIO/ICP/visual loop 문제와 다음 planar A/B |
| 4 | [`RTAB-Map 런타임 진단 및 루프 로그`](./troubleshooting/06_rtabmap_livo_2026-08-27_runtime_diagnosis_and_loop_closure_log.md) | DB·trajectory·loop closure 상세 근거 |
| 5 | [`CODEX_PROJECT_MEMORY.md`](./CODEX_PROJECT_MEMORY.md) | 장기 프로젝트 사실·안전·acceptance 기준 |

## 현재 한눈에 보기

| 구간 | 현재 상태 | 다음 게이트 |
|---|---|---|
| Go2/L2 | 충전 중·로봇 OFF(사용자 전달) | 전원 후 DDS cloud/IMU/odom preflight |
| RTAB-Map | 2D map 개선, type-2 closure 5개 | planar 3DoF 짧은 A/B와 type-1 global visual closure |
| Jetson | Docker·NetBird·NetworkManager active, 자원 여유 | 현 상태 유지 |
| Jetson↔Docker | 임시 비제어 포트 양방향 UDP PASS | production bridge 대신 command sink 시험 |
| Docker | 컨테이너 실행 중, PID 1은 idle `tail` | 실제 S2E runtime/checkpoint 배치 |
| Docker↔Server | model 조회, text JSON, 보관 RGB vision 요청 PASS | live frame + 전체 navigation schema + provenance |
| 물리 자율주행 | **NO-GO** | watchdog·단일 command authority·fault injection 완료 |

## 폴더 지도

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

