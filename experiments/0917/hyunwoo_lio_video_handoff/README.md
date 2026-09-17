# 현우님 전달용 — Goal 4 영상·지도·궤적

ESCAPE-Nav 성공 회차 **009**와 Direct Goal PixelNav 비교 회차 **005**를 같은 Goal 4 기준으로 모았습니다. 기존 LIO/오도메트리 주행 자료입니다. VIO 실험 결과가 아닙니다.

**외부에서 로봇을 촬영한 휴대폰 영상은 별도로 받아야 합니다.** 이 폴더의 영상은 로봇 전방 카메라 영상입니다. 이번 폴더 재정리와 PixelNav 영상 추가 작업은 외부 전송·Git push하지 않았습니다.

## 폴더 구성

```text
hyunwoo_lio_video_handoff/
├── README.md                         ← 먼저 읽기
├── 01_videos/
│   ├── escape_nav/                   ← ESCAPE 영상 3개
│   └── direct_goal_pixnav/            ← 원본 bag에서 추출한 PixelNav 영상 2개
├── 02_topview/                       ← 비교 그림 PNG·SVG·PDF, 궤적 없는 지도
├── 03_trajectories/                  ← 두 방법의 궤적 CSV, PixelNav 영상 프레임 시각
├── 04_vlm_decisions/                 ← ESCAPE 판단 시각·원문, 실제 요청 이미지 21장
└── 05_metadata/                      ← 회차 결과·출처·동기화·재생 확인 기록
```

## 영상 제작에 바로 사용할 파일

| 용도 | 파일 |
|---|---|
| 요청하신 ESCAPE RGB+VLM 4배속 | [escape_goal4_009_rgb_vlm_4x.mp4](01_videos/escape_nav/escape_goal4_009_rgb_vlm_4x.mp4) — 1920×1080, 76초 |
| ESCAPE RGB+VLM 원속도 | [escape_goal4_009_rgb_vlm_1x.mp4](01_videos/escape_nav/escape_goal4_009_rgb_vlm_1x.mp4) — 1920×1080, 303.87초 |
| ESCAPE RGB 중심 원속도 | [escape_goal4_009_1x.mp4](01_videos/escape_nav/escape_goal4_009_1x.mp4) — 1280×778, RGB 720p+정보 영역 |
| PixelNav 원속도 편집용 | [pixnav_goal4_005_rgb_1x.mp4](01_videos/direct_goal_pixnav/pixnav_goal4_005_rgb_1x.mp4) — 1280×720, 60.47초, 자막 없음 |
| PixelNav 4배속 확인용 | [pixnav_goal4_005_rgb_4x.mp4](01_videos/direct_goal_pixnav/pixnav_goal4_005_rgb_4x.mp4) — 1280×720, 15.27초, 회차·배속 표시 |
| 완성 top view 비교 그림 | [PNG](02_topview/fig6_goal4_ours_vs_pixnav_run005.png) · [편집 가능한 SVG](02_topview/fig6_goal4_ours_vs_pixnav_run005.svg) · [PDF](02_topview/fig6_goal4_ours_vs_pixnav_run005.pdf) |
| 궤적을 시간에 따라 그릴 때 | [ESCAPE CSV](03_trajectories/ours_trajectory.csv) · [PixelNav CSV](03_trajectories/pixnav_trajectory.csv) · [좌표·시각 설명](03_trajectories/README.md) |
| VLM 판단을 재편집할 때 | [판단 타임라인](04_vlm_decisions/decision-timeline.json) · [표시 방법](04_vlm_decisions/README.md) |

모든 MP4는 15fps입니다. PixelNav 영상은 원본 RGB를 확대·크롭하지 않고 추출했습니다. 1배속·4배속 모두 주행 중 정지·회전·대기 구간을 따로 제거하지 않았습니다. Direct Goal PixelNav는 VLM을 사용하지 않으므로 해당 영상에 VLM 판단은 없습니다.

## 영상·궤적 맞추기

각 방법의 **영상 0초 = 해당 회차 목표 발송 시각 = CSV의 `t_s=0`**입니다. 두 회차의 실제 촬영 시각은 다르므로 UTC를 서로 맞추면 안 됩니다. 4배속 영상의 재생 시각에 4를 곱하면 해당 회차의 원속도 시각입니다. 마지막에는 프레임 단위의 짧은 패딩이 있을 수 있습니다.

ESCAPE는 약 302.35초, PixelNav는 약 58.90초에 제어가 종료됐습니다. 두 영상에는 종료 뒤 약 1.5초도 포함되어 있습니다. CSV는 종료 전 궤적이므로 영상 끝까지 궤적을 그릴 때 마지막 위치를 유지하면 됩니다. 카메라 bag 수신 시각을 사용했으며, 실제 노출 시각과 하드웨어 동기화된 외부 촬영 시각은 아닙니다.

두 방법 모두 목적지는 **Goal 4**입니다. `005`는 PixelNav 회차 번호이며 Goal 5가 아닙니다. 지도에는 Goal 4 하나와 1m 축척을 표시했습니다. top view는 기록된 지도 위 궤적 그림이며, 외부에서 촬영한 top view 동영상은 아닙니다.

## 결과와 해석 범위

| 방법 / 회차 | 로그상 종료 | 최종 목표 거리 | 주행 시간 | 10Hz odom 경로 길이 |
|---|---|---:|---:|---:|
| ESCAPE `full-goal-4-009` | GOAL_DISTANCE_REACHED | 0.984m | 302.35초 | 14.97m |
| PixelNav `direct_goal-goal-4-005` | PIXNAV_TERMINAL_BEFORE_GOAL | 8.791m | 58.90초 | 3.73m |

선택된 두 회차의 정성 비교입니다. 전체 평균 성능이나 독립 실측 GT가 아닙니다. 두 회차의 현장 개입·접촉·진로 방해 여부는 자료상 미확인이고 실행 이미지 버전도 다릅니다. 그림을 위해 y축을 뒤집거나 궤적을 평활화하지 않았습니다. SPL은 확정하지 않았습니다. 상세 회차 근거는 [metadata.json](05_metadata/metadata.json)에 있습니다.

## 파일 확인과 전달

MP4 5개의 전체 디코딩 확인 기록, 궤적 행 수와 시각 검사, VLM 이미지 연결 확인은 [verification.json](05_metadata/verification.json)에 있습니다. 원본 위치는 [file-locations.json](05_metadata/file-locations.json)에 남겼습니다. 새로운 해시는 계산하지 않았습니다.

이 폴더 전체를 전달하면 문서·영상·그림을 열 수 있습니다. **MP4는 저장소의 Git 제외 대상이므로 Git push만으로는 영상이 전달되지 않습니다.** 전체 자료가 들어 있는 `hyunwoo_lio_goal4_handoff_20260917.zip`을 별도로 전달하세요. Jetson의 Downloads 폴더에 준비했습니다. 원본 bag은 용량 때문에 포함하지 않았고 추출 경로를 기록했습니다. Jetson 내부에서는 일부 파일이 원본과 저장공간을 공유하므로, 수정할 때는 다른 이름으로 복사하여 편집하세요.
