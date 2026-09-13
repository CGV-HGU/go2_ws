# 2번 목표 주행 거리·궤적

에피소드: `robot-pointnav-120b82e5d467-1`. Full 비동기, 고정 시작점/오도메트리 방식.

- 시작 시 목표까지 직선 거리: 3.115 m
- 누적 이동 거리: 3.103 m (10 Hz 보간)
- 시작점→정지 요청 시점 직선 변위: 2.140 m
- 시스템 도착 판정 잔여 거리: 0.988 m
- 골 발행→정지 요청 시간: 88.87 s
- 현장 관찰: 직접 개입·접촉·진로 방해 없음. 목표에서 육안상 약 1 m, 정확한 실측 없음.

`trajectory.png` / `trajectory.pdf`: 시작 방향을 +X, 왼쪽을 +Y로 표시한 궤적과 거리 추이.
`episode-trajectory-10hz.csv`: 주행 구간 좌표·방향·누적 거리·목표 잔여 거리.
`../odom-trajectory.csv`: 시작 전/정지 후 일부를 포함한 원본 좌표 CSV.
`../raw-episode.tar.gz`: 원시 에피소드 로그·정책 입력·설정 묶음. `../raw-manifest.json`에 파일별 SHA-256 기록.

오도메트리 추정치이며 실측 궤적이나 Ground Truth가 아니다. 누적 거리는 골 발행부터 직접 StopMove 요청까지이며 정지 후 안정화 구간은 제외한다. 5/10/20 Hz 집계는 각각 3.017/3.103/3.166 m이다. 제자리 몸체 움직임과 오도메트리 잡음도 포함할 수 있다. 시작-목표 직선 거리를 SPL 최단 거리로 간주하지 않는다.

재생성: `python3 .local-data/fixed-start-goals-20260913/export_goal2_trajectory.py`
