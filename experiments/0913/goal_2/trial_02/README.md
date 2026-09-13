# 2번 목표 주행 거리·궤적

에피소드: `robot-pointnav-c8a1b7d40c83-1`. Full 비동기, 고정 시작점/오도메트리 방식.

- 시작 시 목표까지 직선 거리: 3.115 m
- 누적 이동 거리: 2.551 m (10 Hz 보간)
- 시작점→정지 요청 시점 직선 변위: 2.159 m
- 종료 시 시스템 잔여 거리: 0.961 m
- 골 발행→정지 요청 시간: 47.54 s
- 현장 관찰: 아까 보다 좀 오차가 있어보이는데? 좀 1m보다는 살짝 먼감도 있는데 대충은 맞는것 같아. 개입접촉 진로방해는 없었어.

`trajectory.png` / `trajectory.pdf`: 시작 방향을 +X, 왼쪽을 +Y로 표시한 궤적과 거리 추이.
`episode-trajectory-10hz.csv`: 주행 구간 좌표·방향·누적 거리·목표 잔여 거리.
`../odom-trajectory.csv`: 시작 전/정지 후 일부를 포함한 원본 좌표 CSV.
`../raw-episode.tar.gz`: 원시 에피소드 로그·정책 입력·설정 묶음. `../raw-manifest.json`에 파일별 SHA-256 기록.

오도메트리 추정치이며 실측 궤적이나 Ground Truth가 아니다. 누적 거리는 골 발행부터 직접 StopMove 요청까지이며 정지 후 안정화 구간은 제외한다. 5/10/20 Hz 집계는 각각 2.486/2.551/2.574 m이다. 제자리 몸체 움직임과 오도메트리 잡음도 포함할 수 있다. 시작-목표 직선 거리를 SPL 최단 거리로 간주하지 않는다.

재생성: `python3 .local-data/fixed-start-goals-20260913/export_episode_trajectory.py --trial /home/unitree/s2e-vlm-async-framework-minimal/.local-data/fixed-start-goals-20260913/prepared/goal-2-002 --archive /home/unitree/s2e-vlm-async-framework-minimal/.local-data/fixed-start-goals-20260913/archives/goal-2-002-reviewed`
