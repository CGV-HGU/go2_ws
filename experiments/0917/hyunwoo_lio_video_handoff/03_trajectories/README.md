# 궤적과 영상 동기화

- `ours_trajectory.csv`: ESCAPE Goal4 / 009, 5,359개 위치 표본.
- `pixnav_trajectory.csv`: Direct Goal PixelNav Goal4 / 005, 1,028개 위치 표본.
- `pixnav_frame_timeline.csv`: 새 PixelNav 원속도 영상 907프레임과 원본 카메라 시각 대응.

| 열 | 의미 |
|---|---|
| `t_s` | 해당 회차 목표 발송 이후 초. 영상 0초와 같은 기준 |
| `utc` | 표본 기록 UTC |
| `source_stamp_s`, `receipt_age_s` | 원본 센서 시각(Unix초), 수신 지연 |
| `x_m`, `y_m` | 표시한 출발점·방향 기준 XY(m), 시작 전방이 +X, 왼쪽이 +Y |
| `z_m` | 기록된 원본 높이. XY처럼 시작점 높이를 빼서 0으로 만든 값은 아님 |
| `yaw_rad` | 시작 방향 기준 방향각(rad), XY와 동일한 회전 기준 |
| `source`, `frame`, `raw_frame` | 기록 출처와 좌표계 표시 |

목표는 `(10.613861, -3.886406)m`입니다. 원본 내보내기 순서를 보존했으며 궤적을 반전하거나 평활화하지 않았습니다. 4배속 영상에는 `t_s / 4` 위치에 표본을 표시하세요. 마지막 표본 이후 영상이 끝날 때까지 마지막 위치를 유지할 수 있습니다.

PixelNav 프레임 표의 `source_receipt_unix_ns`는 bag 수신 시각, `source_header_ns`는 영상 헤더 시각입니다. 15fps 출력은 각 시점까지 도착한 직전 영상을 유지합니다. 영상 보간이나 중간 정지 삭제를 하지 않았습니다. 이 표는 외부 카메라와의 하드웨어 시간 동기화 증거가 아닙니다.
