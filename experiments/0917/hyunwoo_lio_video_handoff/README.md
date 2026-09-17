# 현우님 전달용: 기존 LIO/오도메트리 Goal 4 자료

**기존 오도메트리 주행 자료이며 VIO 성공 자료가 아닙니다.** 전방 카메라 영상과
지도 위 궤적을 모았습니다. 로봇 몸체를 외부에서 촬영한 휴대폰 영상은 포함돼 있지 않습니다.
파일을 외부로 전송하거나 push하지는 않았습니다.

## 바로 사용할 파일

| 파일 | 내용 |
|---|---|
| `escape_goal4_009_rgb_vlm_4x.mp4` | 요청하신 파일. 1920×1080, 15fps, 76초, 약 19.7MB |
| `escape_goal4_009_rgb_vlm_1x.mp4` | 같은 RGB+VLM 판단 영상의 원속도 편집본, 303.867초 |
| `escape_goal4_009_1x.mp4` | RGB 중심 원속도 영상, 1280×778(720p+정보 영역) |
| `fig6_goal4_ours_vs_pixnav_run005.png/.svg/.pdf` | Goal4만 표시한 top view 지도와 두 방법의 궤적. 1m 축척. SVG 편집 가능 |
| `ours_trajectory.csv` | ESCAPE 009의 시각·XY·yaw 5,359개 표본 |
| `pixnav_trajectory.csv` | Direct Goal PixelNav 005의 시각·XY·yaw 1,028개 표본 |
| `decision-timeline.json` | ESCAPE VLM 판단 21건의 촬영·수신·적용 시각과 판단 원문 |
| `decision_01_request.jpg`~`decision_21_request.jpg` | 판단 기록이 참조하는 실제 관측 영상 21장. 전달용 재편집 자료에 추가 |
| `metadata.json`, `file-locations.json` | 그림의 좌표·회차 근거, 각 파일의 원본 위치 |

두 방법의 목적지는 **Goal4**입니다. 005는 PixelNav 회차 번호이며 Goal5가 아닙니다.
CSV `t_s`는 해당 회차 목표 발송 이후 초, `utc`는 UTC, XY는 표시 시작점·방향 기준 m입니다.
ESCAPE 영상 0초도 목표 발송에 맞춰져 있습니다. 4배속의 재생 1초는 실제 약 4초이며
정지/관측 구간을 별도로 삭제하지 않았습니다. 그림은 시간별 궤적 영상 제작에 쓸 수 있는
정적 지도이고, 외부 카메라로 촬영한 top view 동영상은 아닙니다.

| 방법/회차 | 로그상 종료 | 최종 목표 거리 | 주행 시간 | 10Hz odom 경로 길이 |
|---|---|---:|---:|---:|
| ESCAPE `full-goal-4-009` | GOAL_DISTANCE_REACHED | 0.984m | 302.35초 | 14.97m |
| Direct Goal PixelNav `direct_goal-goal-4-005` | PIXNAV_TERMINAL_BEFORE_GOAL | 8.791m | 58.90초 | 3.73m |

선택된 두 회차의 정성 비교이며 전체 평균 성능이나 실측 GT가 아닙니다.
두 회차의 현장 개입·접촉 여부는 해당 자료상 미확인입니다. 그림을 위해 y축을 뒤집거나
경로를 평활화하지 않았습니다. 세부 제한은 아래 원본 설명에 있습니다.

## PixelNav 영상 원본

PixelNav 005의 완성 MP4는 이 묶음에 없습니다. 원본 bag metadata에서 RGB와
CameraInfo 각 1,096개를 확인했습니다. 이 bag을 영상으로 내보낼 수 있습니다.

`/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-recaptured5-20260914/episodes/direct_goal-goal-4-005/native-inputs`

영상 토픽: `/robot_nav/sensors/front_camera/image_raw`.
목표 발송·종료 경계는 같은 회차의 runtime 기록과 아래 재계산 자료를 사용합니다.
bag 전체를 중복 복사하거나 PixelNav MP4를 새로 렌더링하지는 않았습니다.

## 상세 설명과 원본

- `experiments/0916/goal4_video/vlm_annotated/README.md`: VLM 판단·영상 동기화 방식.
- `2dmap/0913/paper/goal4_run005/README.md`: 그림과 좌표 변환 근거.
- `experiments/0914/night_log_review/episodes/original5/full-goal-4-009`.
- `experiments/0914/night_log_review/episodes/recaptured5/direct_goal-goal-4-005`.

파일은 가능한 경우 원본과 저장공간을 공유하는 하드링크로 모았습니다. 전송하면
일반 파일로 복사됩니다. 원본을 덮어쓰지 말고 편집본은 별도 이름으로 저장하세요.
추가 해시 계산은 하지 않았습니다.

## 전달 전 재확인

2026-09-17에 MP4 3개를 모두 끝까지 디코딩해 오류 없음을 확인했습니다.
Top view PNG를 직접 확인했고 SVG에 외부 이미지 의존성이 없음을 확인했습니다.
두 CSV는 각각 5,359/1,028행이며 숫자 누락·비정상 값·시간 역전이 없습니다.
VLM 판단 21건이 참조하는 이미지도 모두 폴더에 포함하고 파일 읽기를 확인했습니다.
PixelNav 완성 MP4와 외부 촬영 영상은 포함되지 않았다는 범위는 그대로입니다.
