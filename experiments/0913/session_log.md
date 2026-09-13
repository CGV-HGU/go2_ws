# 고정 시작점 반복 주행 기록 (2026-09-13)

Full 비동기, look 동작 0.1m 전진 대체, 도착 반경 1m, 최종 방향 제어 없음. 6회 동안 주행 설정 변경 없음. 실패 회차 포함.

| 회차 | 좌표상 결과 | 시간(s) | 누적 오도메트리(m, 10Hz) | 종료 시 목표 잔여(m) |
|---|---|---:|---:|---:|
| goal-1-001 | CONTROLLER_INHIBITED:MOTION_TIMEOUT | 56.66 | 0.564 | 4.777 |
| goal-1-002 | GOAL_DISTANCE_REACHED | 259.91 | 6.923 | 0.999 |
| goal-1-003 | GOAL_DISTANCE_REACHED | 195.35 | 5.979 | 1.000 |
| goal-2-001 | GOAL_DISTANCE_REACHED | 88.87 | 3.103 | 0.988 |
| goal-2-002 | GOAL_DISTANCE_REACHED | 47.54 | 2.551 | 0.961 |
| goal-2-003 | GOAL_DISTANCE_REACHED | 50.30 | 2.610 | 0.958 |

좌표상 도착 5회 / 초기 관측 회전 시간 초과 1회. 이것을 실측 SR로 간주하지 않는다. 현장 보고는 대부분 약 1m이며, goal-2-002는 1m보다 조금 멀어 보였다는 보고다. goal-1-002는 저장 방향과 반대로 정지했으며 오도메트리 방향 차이는 약 172도다. 최종 방향은 도착 조건에 포함되지 않는다.

원시 에피소드: prepared/goal-*/full5m 및 navigation.log, sidecar.log, pixnav.log. 현장 보고: 각 회차 field-report.json (미응답 회차는 없음). 거리와 좌표: archives/goal-*/episode.json, odom-trajectory.csv, trajectory-view/. 전체 색인: episode-index.json / episodes.csv.

Raw RGB/점군/IMU/odom/TF rosbag은 ../prepared-daylight-map-20260913/native-inputs-fixed-start 및 native-inputs-goal-* 에 각각 보존했다. 각 회차 종료 후 rosbag을 정상 종료했다.

후속 작업: 6회 반복을 마쳐 ../prepared-full-area-map-20260913 에서 새 전체 매핑 시작. 현재 자동주행 프로세스는 종료 상태. 사용자가 전체 경로를 수동 매핑하고 done을 알리면 DB를 보존하고 다음 목표 수집으로 진행한다.

충전 중 분석할 항목: 관측 회전의 반동 및 실패/통과 변동, 관측에 소요되는 정지 시간, 비동기 후보 수용·대기, look 대체 동작과 실제 진전, 고정 시작점 오도메트리 도착 판정과 현장 위치 오차.
