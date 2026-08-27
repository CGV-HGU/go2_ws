# 2026-08-27 RTAB-Map 2D 지도 및 루프 폐쇄 증거

이 디렉터리는 2026-08-27 실주행에서 추출한 두 개의 2D occupancy map과 두 번째 주행의 RTAB-Map 루프/근접 폐쇄 로그를 보존한다. 주행은 사용자가 직접 수행했으며, Codex는 로봇 이동 명령을 발행하지 않았다.

## 보존 파일

| 파일 | SHA-256 |
|---|---|
| `rtabmap0827.pgm` | `27e00f1d36755de6f165a32969f090fe4ee9c97a0bbe673da2cf0437a59afe17` |
| `rtabmap0827.yaml` | `9cbe953f74f8f9912b33527a1dcd62aa3cea1b2b6c1dae5e8ceaf568b5f008b7` |
| `rtabmap0827_2.pgm` | `97e589e6140345afc2337029836e24766eaea809c4c490fa8dbd49abaee675d4` |
| `rtabmap0827_2.yaml` | `cc9320d7f491c7acd12716713c271279c839c0599e531853bac74a9d5c07b709` |
| `loop_logs/loop_events_20260827_151630_headless_mapping.jsonl` | `85acb0dc5621f3d48cea7ef5332c115cace48fe6fae231ba0bb4ea529f1e080f` |
| `loop_logs/loop_events_20260827_151630_headless_mapping.log` | `e6e01c352ff59531ffd897328b6ac1dbdca953c0d285973630ab0e5110cb3d3d` |

두 YAML의 `image` 항목은 같은 디렉터리의 PGM을 상대 경로로 참조한다. 해상도는 모두 `0.05 m/pixel`이다.

## 지도별 관찰 결과

### `rtabmap0827.*` — 첫 번째 주행

- RTAB-Map 노드 402개
- 루프/근접 폐쇄 0개
- 당시 활성화되어 있던 neighbor-link ICP 보정의 영향으로 벽과 통로가 휘는 현상이 관찰됨
- PGM: 544 × 450 cells
- occupied 2,296 / free 69,947 / unknown 172,557 cells

### `rtabmap0827_2.*` — 두 번째 주행

- RTAB-Map 노드 563개
- `RGBD/NeighborLinkRefining=false`로 재실험
- type-2 공간 근접 폐쇄 5개, type-1 전역 시각 루프 폐쇄 0개
- 거절된 시각 후보 83개, 최대 시각 점수 0.844551
- 첫 지도보다 벽의 연속성과 2D 형상이 개선됨
- PGM: 532 × 504 cells
- occupied 3,262 / free 73,237 / unknown 191,629 cells

## 해석 시 주의사항

두 번째 지도의 2D 결과는 개선되었지만, 이를 정확한 3D SLAM 결과로 해석하면 안 된다. 같은 주행에서 원시 LIO 궤적의 z 범위는 약 0.0212 m였으나 RTAB-Map `map→base` 궤적의 z 범위는 약 6.452 m였다. 즉 현재 구성에는 3D pose-graph의 z 방향 발산 문제가 남아 있다.

루프 폐쇄, DB 조회, z 발산 및 다음 실험 후보에 관한 상세 근거는 [런타임 진단 및 루프 폐쇄 기록](../../docs/troubleshooting/06_rtabmap_livo_2026-08-27_runtime_diagnosis_and_loop_closure_log.md)을 참조한다.
