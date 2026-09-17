# ESCAPE VLM 판단 재편집 자료

`decision-timeline.json`과 `decision_01_request.jpg`부터 `decision_21_request.jpg`까지 함께 사용하세요. JSON의 `image`는 이 폴더 안의 파일명입니다.

- `observation_t`: 해당 관측 영상의 시각(목표 발송 이후 초).
- `available_t`: 판단이 수신된 시각. 이때부터 응답을 표시합니다.
- `pixel`: 선택한 과거 관측 영상 안의 픽셀입니다. 현재 RGB 위에 그대로 표시하면 안 됩니다.
- `reason`: 실제 VLM 판단 이유 원문입니다.
- `events`: received / queue / admit / discard 등 실제 처리 시각입니다.

비동기 처리 때문에 최근 수신된 판단과 현재 실행 중인 판단은 다를 수 있습니다. 영상의 표시 문구는 모델 출력과 명령 기록이며, 독립적으로 확인된 실제 로봇 이동이나 안전성 보장은 아닙니다.

Direct Goal PixelNav 005는 VLM을 사용하지 않으므로 해당 방법의 VLM 타임라인은 없습니다.
