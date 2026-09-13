# 새 매핑·목표 5개·기록용 에피소드 준비 — 2026-09-13

요청 범위는 새 지도에서 실제 목표 1~5번을 저장하고 ESCAPE Full 에피소드들을 기록하는 것이다. 기존 두 목표 비교 준비를 이 작업의 완료로 간주하지 않는다. **현재 소프트웨어 준비와 파일 기반 검증은 완료했으며, 실제 지도·목표는 현장에서 새로 생성해야 한다.** 이번 준비 중 카메라·SLAM·명령 서비스를 시작하거나 로봇을 움직이지 않았다.

## 현장 실행의 기준

- 앞서 사용자가 선택하고 짧은 목표를 주행한 `fixed_start_odometry` 방식이다. 매 회차 표시한 같은 시작 위치·방향으로 돌아온다.
- RTAB-Map으로 새 지도를 남긴 뒤 SLAM만 정상 종료하여 DB를 고정한다. 오도메트리 입력과 기록기는 계속 켜서 수동 이동 중 목표 5개를 저장한다. 이 과정을 ICP 전역 로컬라이제이션 성공으로 설명하지 않는다.
- 다섯 목표가 모두 같은 원점·부팅 세션에서 캡처되고 새 지도에 묶여야 실행 묶음을 확정한다. 출발·목표 원본 스냅샷과 해시도 저장한다.
- 도착 반경 1m, 최종 yaw 미요청, Full 비동기, 룩→10cm 전진, 일반 전진 25cm, 에피소드 제한 360초를 유지한다. Direct는 이후 같은 확정 목표를 사용한다.
- 이전 지도·목표를 자동으로 재사용하거나 시험용 좌표를 실험 목표로 만들지 않는다.

## 준비한 경로

실제 새 실험 루트:
`/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-20260913`

현재 상태는 `awaiting_mapping`이다. 지도와 실제 목표 좌표는 아직 없다. 다음 항목만 생성했다.

| 위치 | 역할 |
|---|---|
| `camera.compose.json`, `camera-config/` | 매핑 전에 필요한 카메라 단독 서비스, 고정 이미지·보정 파일 |
| `mapping/` | 새 RTAB-Map 세션의 설정·원본 센서 기록 준비 |
| `goals/` | 실제 시작점과 목표 1~5번 캡처용 빈 디렉터리 |
| `episodes/` | Full/Direct·목표 번호·시도 번호별 원본 기록 |
| `archives/` | 주행 후 궤적 CSV, 에피소드 요약, 원본 파일 해시 목록 |
| `campaign.json` | 5개 목표 요구사항과 모든 시도 상태 |
| `candidate-validation.json` | 고정 이미지와 확인한 코드의 해시·검증 근거 |

현재 포인터 `.local-data/current-prepared-robot-trials.json`도 이 새 실험을 가리킨다. 기존 두 목표의 준비 폴더는 과거 자료다.

최종 이미지: `escape-navigation:five-goal-recording-20260913`
`sha256:a92b920649fabdc2ba7b894f9e75a68454596cc997d70825d3214349a305cf14`.
직전 후보와 주행 런타임은 같고, launch에서 기록용 VLM 요청 보존을 명시적으로 켤 수 있도록 추가했다. 설치된 launch 해시:
`013e4a71e91696a69734dd1fb21ba8e7a62094804ad3eedea461b61598ac0d9c`.

## 남기는 기록과 누락 확인

| 기록 | 보관 내용 / 확인 |
|---|---|
| 지도 | 정상 종료한 RTAB-Map DB를 SQLite backup으로 고정, DB 검사·해시, 매핑 설정·센서 bag |
| 시작점·5개 목표 | 좌표·방향·시각·원점 ID·오도메트리 발행자·원본 스냅샷·지도/목표 해시 |
| 에피소드 | 실험 ID, 목표 ID, 시도 번호, 실제 발행 목표, 출발 위치·방향, 종료 상태·정지 확인 |
| 궤적 | 시간표시 오도메트리, 제어 위치, 목표 상대 거리·방향, 이후 10Hz/5Hz/20Hz 경로 계산 가능한 원본 |
| 센서 | RGB·CameraInfo·주행 점군·odom·IMU MCAP, 발행되는 TF/TF_static·RTAB-Map 정보 |
| 몸체 | IMU 자세·각속도, 관절 위치·속도, 발 힘, 배터리, Sport 상태, 수신되는 리모컨 입력 |
| PixelNav | 실제 입력 텐서·영상, 정책 출력, 실행 명령·결과, 룩 대체 횟수, 비동기 전환 trace |
| VLM | 모델·요청 원문·입력 영상 payload·응답·호출 시간·오류·episode ID, 의사결정/관측 trace |
| 현장 보고 | 직접 개입·접촉·방해·실제 종료 위치·측정 방법·사용자 관찰 원문; 미확인값은 null |
| 기록 검사 | 출발 전 실제 기록의 신선도·RGB 파일·원본 기록기 구독, 종료 후 파일·토픽·시간 구간·2초 초과 기록 공백·정책 입력 누락 |

이전에는 VLM 호출 설정이 요청 payload를 생략했고, 기록기 실행만으로 파일 수신을 확인하지 못했다. 기록용 실행에서는 두 부분을 보완했다. Native bag은 corrected navigation cloud를 포함한다. 구독 확인은 실제 수신 완료를 의미하지 않으므로 종료 후 토픽 메시지 수와 bag 시간 범위도 확인한다.

검사 결과는 `recording-preflight.json`, `recording-integrity.json`으로 남긴다. 기록이 불완전하면 `RECORDING_INCOMPLETE`를 남기며 실패·중단 회차를 지우지 않는다. 파일이 존재한다는 이유만으로 완전한 기록으로 간주하지 않는다. 이 검사는 모든 DDS 패킷의 무손실 전달을 증명하지는 않는다.

`RECORDERS_CLOSED`는 명령·원본 기록·관측 기록 프로세스가 종료된 뒤에만 만든다. 보관 스크립트는 이 표시 없이 진행하지 않는다. 모든 원본은 회차 폴더에 즉시 저장되므로, 큰 bag의 전체 해시 계산과 정리 작업은 실주행 후 충전 시간으로 미룰 수 있다.

## 운영 순서

아래는 다음 담당자를 위한 명령 순서다. 준비 작업으로 이 명령들을 실행하지 않았다. 매핑·목표 발행은 현장 사용자의 해당 작업 요청과 현재 준비 확인 후 수행한다.

1. 본체 부팅 후 카메라·센서·정지 경로를 확인한다. 다른 카메라 발행자가 없는지 확인하고 실험 루트의 `camera.compose.json`으로 카메라만 시작한다. 카메라는 목표 수집과 에피소드 사이에 유지한다.
2. `scripts/robot_map_session.py start-mapping --run-dir <root>/mapping`으로 새 매핑을 시작한다.
3. 사용자 `done` 후 `freeze-fixed-start`로 DB를 고정한다. 이 명령은 SLAM만 멈추고 목표 수집용 입력·기록기를 유지한다.
4. 표시한 시작점·방향에서 `scripts/fixed_start_goals.py origin --goal-dir <root>/goals --run-dir <root>/mapping --start-confirmed`를 수행한다.
5. 실제 각 목표에서 같은 스크립트의 `capture --label 1`부터 `5`까지 수행한다. `--goal-dir`, `--run-dir`는 동일하다. 기존 캡처를 덮어쓰지 않는다.
6. `scripts/robot_goal_campaign.py seal --root <root>`로 지도와 목표 5개를 확정한다. Full 목표별 첫 회차 5개가 생성·검증된다. 일부 목표 누락이나 다른 지도·원점 혼입은 여기서 드러난다.
7. 시작점 복귀 후 `robot_map_session.py stop`으로 목표 수집용 입력·기록기를 종료한다. Full과 중복 오도메트리 발행자가 생기지 않도록 인수인계를 확인한다. 별도 카메라는 유지한다.
8. 선택한 `episodes/full-goal-N-001/run_trial.sh`를 실행한다. 처음에는 motion disabled다. 현재 시작점·방향·현장 준비 확인을 받은 뒤에만 해당 회차의 `START_GOAL`에 `fixed_start_confirmed`, `origin_id`, `confirmed_unix`를 기록한다. 출발 전에 기록 확인도 통과해야 한다.
9. 종료 시 궤적·결과·원본은 이미 회차 폴더에 있다. 현장 보고를 해당 `field-report.json`에 기록한다. 간단한 관찰만으로 실측 성공 값을 채우지 않는다. 다음 목표도 시작점으로 재정렬하여 수행한다.
10. 재시도는 `robot_goal_campaign.py prepare-trial --root <root> --goal-label N`으로 새 번호를 만든다. Direct 비교에는 `--mode direct_goal`을 추가한다. 같은 sealed goal-plan을 복사한다.
11. `robot_goal_campaign.py status --root <root>`로 목표별 준비·종료·보관 상태를 확인한다. 충전 중 `archive --trial-id <id>`로 각 회차를 정리한다. 실패·미완료도 포함하며, 보관 오류는 별도 상태로 남긴다.
12. 전체 종료 시 카메라도 종료한다. 지도·캡처·회차 원본을 보존한 뒤 공유용 수치와 그림을 추출한다. 원본 영상/payload 전체를 Git에 무조건 올리지는 않는다.

## 확인한 근거와 남은 현장 확인

- 목표·원점·지도 바인딩, 다섯 목표 누락, 다른 지도/스냅샷 변경, 재시도, 실패 회차 보존, 기록기 중단 감지 등을 포함한 최종 관련 시험 **92개 통과**.
- 설치된 이미지의 주행 패키지·Full/Direct launch 시험 **312개 통과**.
- 실제 Docker/Compose 설정 생성·검증에서 Full 5개, Direct 5개, 재시도 1개를 확인했다. 좌표는 명시적인 시험용 자료이며, 생성된 시험용 실행 파일은 이후 실행 불가 상태로 바꿨다. 실제 실험 디렉터리에는 시험 좌표를 넣지 않았다.
- 설치된 VLM 로그 작성기에 기존 기록 영상 바이트를 넣어 다섯 episode ID의 요청·영상·응답이 그대로 저장되는지 확인했다. 이 시험의 약 0.69MB 영상 한 장 기록 호출은 약 8~9ms였다. 네트워크 호출이나 VLM 추론을 수행한 시험은 아니다.
- 기존 실주행 자료에 새 검사기를 적용했을 때 몸체·RGB·정책 입력 기록을 확인하고, 회차 폴더에 없는 native bag을 누락으로 표시했다. 직전 후보에서 실제 MCAP 기록기와 점군 발행 토픽 전환을 격리 검증한 근거도 유지한다.

**본체가 켜진 실제 입력으로 다섯 에피소드를 기록한 것은 아직 아니다.** 새 지도와 목표 캡처, 부팅 후 센서 수신·현재 정지 경로·원본 기록 수신은 현장에서 확인해야 한다. 이 확인을 통과하면 새 매핑과 기록용 에피소드로 진행하며, 별도 5m/두 목표 예비 반복을 선행 조건으로 추가하지 않는다. SR/SPL의 최종 집계·최단경로 검토는 원본을 확보한 뒤 진행한다.

원본 검증 자료: `.local-data/five-goal-recording-prep-20260913/`.
공유 자료: [go2 five-goal recording preparation](https://github.com/CGV-HGU/go2_ws/tree/antarctica/experiments/0913/five_goal_recording_prep).
