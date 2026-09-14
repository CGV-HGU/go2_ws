# Jetson 후속 검토: 에피소드 제한 360초와 시뮬레이터 1800초

> 후속: 2026-09-14 사용자 요청으로 [1800초 적용과 기록 경로 검증](episode_timeout_1800/README.md)을 완료했다. 아래는 변경 전 보존 메모다.

사용자 요청: 현재 주행 설정은 변경하지 않고, 이후 Jetson만 구동하는 시간에 검토한다.

## 전달받은 실험 조건

사용자가 공유한 연구팀 대화에서 이상준 연구원은 시뮬레이터 실험의 전체 제한 시간이 1800초였다고 설명했다. 이는 사용자 제공 정보이며, 정확한 시뮬레이터 실행 설정/버전과는 후속 대조가 필요하다. 현재 실로봇 기록용 실행기는 전체 에피소드 제한 360초를 사용했다. 360초를 시뮬레이터와 동일한 값이라고 취급하면 안 된다.

## 구분해야 할 실제 종료

- `recording-five-goals-long5-20260913/episodes/full-goal-5-001`: 전체 `TRIAL_TIME_LIMIT`, 약 360초, 오도메트리 잔여 거리 1.513m. 목표까지 더 갈 기회를 시간 제한이 종료한 사례다. 1800초였으면 성공했을 것이라는 주장은 아직 검증되지 않았다.
- `recording-five-goals-recaptured5-20260914/episodes/full-goal-5-001`: `CONTROLLER_INHIBITED:MOTION_TIMEOUT`, 잔여 7.690m. 관측 회전 약 73도를 요청하고 2.25초에 회전 명령을 0으로 낮춘 뒤 안정화 대기 중 약 20.30초에 종료했다. 전체 에피소드 제한을 늘리는 것만으로 이 원인이 해결되지는 않는다.
- `recording-five-goals-recaptured5-20260914/episodes/direct_goal-goal-4-002`: `PIXNAV_TERMINAL_BEFORE_GOAL`, 약 35.40초, 잔여 7.463m. 정책 21번째 출력이 STOP이었다. 전체 시간 제한이나 500-step 한도 때문이 아니며, 비교를 위해 STOP 의미를 바꾸거나 임의 재시작해서는 안 된다. 사용자가 개입·접촉·진로 방해 모두 없음을 확인했다.

## Jetson에서 검토할 사항

1. 시뮬레이터의 실제 전체 timeout 설정과 단위, 종료 조건을 실행 설정/코드로 확인한다. 연구팀이 전달한 1800초 조건에 맞춰 실로봇 설정 변경안을 마련한다.
2. 실행기·목표 주입기·준비 스크립트·검증기·모니터의 제한, 출력 문구, manifest를 함께 확인한다. 현재 `robot_pointgoal_trial.py`에는 900초 상한도 있어 숫자 한 곳만 바꿔서는 1800초로 실행되지 않는다.
3. Ours와 Direct PixelNav에 같은 전체 에피소드 한도를 적용한다. 기존 360초 결과는 원본 조건을 유지하고, 1800초 결과와 구분해 기록한다. Direct의 500-step 한도가 1800초 전에 종료를 유발하는지도 점검한다. 정책 STOP 종료는 시간 제한과 별도로 기록한다.
4. 회전 후 안정화 실패는 별도 검토한다. 0.35초 동안 0.25도 이내의 각도 안정 조건과 실제 오도메트리/IMU의 잔여 진동을 재현하고, 물리적 불안정과 과도한 판정 조건을 구분한다. 관측 회전 완료의 의미와 종료/정지 경로를 보존한다.
5. 30분 최대 주행을 가정한 저장공간, 영상·센서 기록량, 로그 종료/보관 시간, 정책 세션 한도, Ctrl+C 및 통신 단절 종료 경로를 검사한다. 제한 증가로 기존 기록을 삭제하지 않는다.
6. Jetson에서 가능한 검증과 실제 로봇에서 필요한 검증을 구분한다. 기존 영상/로그 재생만으로 1800초 목표 도달을 보장할 수 없다.

소스 저장소: `/home/unitree/s2e-vlm-async-framework-minimal`
현재 캠페인: `/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-recaptured5-20260914`
회전 분석: 위 캠페인의 `quick-results/full-goal-5-001-motion-timeout/analysis.json`
PixelNav 평가: 위 캠페인의 `quick-results/direct-goal-4-002-evaluation/episode.reviewed.json`

이 메모 저장 시 런타임 설정·제어 코드·기존 회차 원본은 변경하지 않았다.
