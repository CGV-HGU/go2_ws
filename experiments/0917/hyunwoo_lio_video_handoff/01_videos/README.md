# 영상 선택 안내

두 하위 폴더는 같은 Goal 4로 주행한 서로 다른 회차입니다. 모든 영상은 15fps이며, 정지·회전·대기 구간을 임의로 삭제하지 않았습니다.

| 방법 | 파일 | 용도 | 해상도 / 길이 |
|---|---|---|---|
| ESCAPE 009 | [RGB + VLM · 4배속](escape_nav/escape_goal4_009_rgb_vlm_4x.mp4) | 요청하신 영상, 빠른 확인 | 1920×1080 / 76.00초 |
| ESCAPE 009 | [RGB + VLM · 원속도](escape_nav/escape_goal4_009_rgb_vlm_1x.mp4) | 편집 및 원래 동작 시간 확인 | 1920×1080 / 303.87초 |
| ESCAPE 009 | [RGB 중심 · 원속도](escape_nav/escape_goal4_009_1x.mp4) | RGB와 하단 정보 영역 | 1280×778 / 303.87초 |
| PixelNav 005 | [RGB · 원속도](direct_goal_pixnav/pixnav_goal4_005_rgb_1x.mp4) | 원본 bag에서 추출한 자막 없는 편집용 영상 | 1280×720 / 60.47초 |
| PixelNav 005 | [RGB · 4배속](direct_goal_pixnav/pixnav_goal4_005_rgb_4x.mp4) | 회차·배속 표시가 있는 확인용 영상 | 1280×720 / 15.27초 |

Direct Goal PixelNav는 VLM을 사용하지 않으므로 VLM 판단 영상이 없습니다. 외부 촬영 영상은 별도로 받아야 합니다.

[전체 자료 안내](../00_먼저읽기.md) · [영상과 궤적 시각 맞추기](../03_trajectories/README.md)
