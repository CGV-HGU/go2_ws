# Goal 4 / PixNav 005 그림 근거

상준님과 사용자가 요청한 **005 회차**를 Ours Goal 4 성공 회차와 함께 표시했다. 목적지를 Goal 5로 바꾸거나 궤적의 y축을 반전한 그림이 아니다.

| 방법 | 회차 | 로그상 종료 | 최종 목표 거리 | 경로 길이¹ | 시간 |
|---|---|---|---:|---:|---:|
| ESCAPE-Nav | `original5/full-goal-4-009` | `GOAL_DISTANCE_REACHED` | 0.984m | 14.97m | 302.35s |
| Direct-goal PixNav | `recaptured5/direct_goal-goal-4-005` | `PIXNAV_TERMINAL_BEFORE_GOAL` | 8.791m | 3.73m | 58.90s |

¹ 기존 분석의 10Hz odom 경로 길이. 그림에는 원본 내보내기 좌표를 모두 사용했다.

005는 전진 후 우측으로 꺾여 `(2.103, −1.685)m` 부근에서 종료했다. 모델의 전진 12회·우회전 5회·좌회전 3회가 실행된 기록이며, 종료는 목표 도달 전 모델 STOP이다. 선택 이유는 사용자가 요청한 정성 비교 사례이며 전체 성능의 평균이나 대표성에 따른 선택은 아니다. 전체 다섯 후보는 `../../goal4_comparison_paper/pixnav_candidates.png`에 보존되어 있다.

## 목표가 전달됐다는 말의 범위

두 회차의 Goal 4 좌표는 `(10.613861, −3.886406)m`로 같고 지도 DB 해시도 같다. 005에서는 `goal_published=true`, `apply_status=admit`, 투영 픽셀 `(433,175)`가 기록되어 있다. 이는 저장된 목표 좌표와 그 픽셀 투영이 전달됐다는 뜻이다. 실제 목표의 가시성·가림 여부나 카메라 보정의 절대 정확성까지 증명하지 않는다. 정책은 metric 좌표 그대로가 아니라 목표 영상·마스크와 관측 이력을 입력받는다.

## 그림과 검증

- Ours 5,359점, PixNav 005 1,028점의 원본 이벤트를 다시 계산해 그림 CSV와 대조했다. 원점·방향 변환 후 좌표와 시각의 최대 차이는 0이다.
- 기존 지도 자유 영역 마스크와 시작점 좌표 변환을 유지했다. 영역 메우기, 벽 이동, 궤적 반전·평활화·축약을 하지 않았다.
- 축척 선은 지도 좌표에서 정확히 1m, 원 지도에서 20픽셀이다. 가로·세로 축척도 같다.
- 축척 글자·선의 실제 렌더링 범위와 여유 공간을 지도 마스크와 대조하여 **겹친 흰 영역 0셀**을 검사한다. 흰 배경 상자를 추가하지 않았다.
- SVG에는 편집 가능한 텍스트와 벡터 경로를 사용한다. 목표, 궤적, 종료점, 축척의 요소 ID를 명시했다.
- `validation.json`, `metadata.json`, `input_provenance.json`, `SHA256SUMS`에 검증과 출처를 남긴다.

두 회차의 현장 개입·접촉·방해 여부는 자료상 미확인이고 navigation 이미지 버전도 다르다. 이 그림은 시작점에 정렬한 odom의 정성 사례다. 새 ICP 정합이나 외부 Ground Truth가 아니며, 최단 통행 경로가 검증되지 않아 SPL은 표시하지 않는다.

제안 캡션:

> Real-robot Goal 4 example. ESCAPE-Nav reaches the odometry-based 1m success region. The selected Direct-goal PixNav run (005) turns early and terminates with 8.79m remaining. Marked-start odometry trajectories are overlaid on the recorded map for spatial context.

재생성: 저장소 루트에서 `python3 2dmap/0913/paper/goal4_run005/build_figure.py`. 로봇 본체·ROS·GPU 없이 기존 증거 파일만 읽어 그림을 생성한다. 생성 파일은 `paper/` 내부로 제한된다.
