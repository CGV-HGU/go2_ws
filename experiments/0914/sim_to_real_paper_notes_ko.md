ESCAPE-Nav Sim-to-Real 포팅 변경 사항과 논문용 서술

작성 기준: 2026-09-14. Codex 작업 「로봇 브랜치 적용 여부 확인」와 「ESCAPE-NAV 작업 인수인계」의 대화, 날짜별 실험 보고서, 현재 로봇 소스를 교차 확인했다. 이번 작업은 기록 조사와 문서 작성이며 새 물리 실험이나 제어 코드 수정을 수행하지 않았다.

실제 조사 대상은 `/home/unitree/s2e-vlm-async-framework-minimal`이다. 조회한 HEAD는 `abad1ffa471ff4b98ce0a7ee2c2d35c043375f90`이며 로봇 구현에 미커밋·미추적 파일이 있으므로 이 커밋만으로 배포본 전체를 지정할 수 없다. 아래의 최신 실험 설정은 9월 13~14일의 개별 회차 자료를 기준으로 한다. 저장소 기본 compose와 과거 문서의 기본값은 해당 회차의 최종 override와 다를 수 있다.

**논문에 넣을 핵심 관점**

기존 ESCAPE-Nav의 VLM–PixelNav 비동기 구조를 실제 로봇에 연결하려면, 시뮬레이터가 제공하던 관측·행동 완료·좌표·시간 정보를 실제 센서와 구동계로 구현해야 했다. 변경은 크게 카메라 입력 정합, 실행 불가능한 시선 행동의 대체, 실측 기반 이동·회전 실행, 비동기 인계 보완이다. 일부는 플랫폼 적응이고, 일부는 실로봇에서 드러난 공통 구현의 오류 수정이다. 이 모두를 새로운 알고리즘 기여나 Sim-to-Real 성능 향상으로 묶지는 않는다.

| 시뮬레이션과 달랐던 점 | 실제 적용·수정 | 논문에서의 위치 |
|---|---|---|
| 실제 카메라의 영상 비율·색상·왜곡·내참수가 정책 입력과 다름 | BGR→RGB, 왜곡 처리, 해상도에 따른 내참수 갱신, RGB와 목표 마스크를 함께 4:3 정책 카메라로 변환 | 입력 정합 |
| 고정 카메라에서 `look_up/down`을 시뮬레이터처럼 실행할 수 없음 | 몸체 pitch·디지털 시야 변경을 시험한 뒤, 최근 비교에서는 raw 룩 출력을 20cm 전진으로 대체 | 명시해야 할 행동 공간 적응 |
| 명령한 25cm/30°와 실제 변위·완료시간이 다름 | 오도메트리 기반 폐루프 실행, 가감속·헤딩 보정·정착 확인·시간 제한 | 실로봇 제어 구현 |
| 관측 회전 중 몸체 반동과 명령 사이의 방향 변화 발생 | 명령 회전량 합산 대신 관측 시작 시의 절대 odom 헤딩을 기준으로 각 관측·최종 복귀 계산 | 다중 관측 정합 |
| 로봇 센서와 Jetson 시계가 다르고 RGB/pose가 비동기로 도착 | 시작 시 공통 시각 오프셋 추정, 이미지 stamp에 맞춘 pose 보간, 새 RGB·pose와 실제 실행 결과 결합 | 시간 정합 |
| VLM 응답·동작 완료·관측 전환 순서가 일정하지 않음 | 현재 단일 동작 경계에서 인계, 정지 확인, 오래된 GO 폐기, 정상 관측 checkpoint와 실제 실패 구분 | 비동기 실행 보완 |
| 후보의 좌우 부호 및 가까운 바닥 목표의 가시성 문제 | VLM bearing 부호 수정, 가까운 목표에서 가시 바닥 후보 사용, 목표 정보 선별 전달, 미검사 후보 추가 검증 | 기하·후보 생성 보완 |
| 시뮬레이터의 좌표·충돌 정보에 대응하는 실측 정보가 제한됨 | 시작점 기준 SE(2) 좌표계와 odometry 사용, 물리 충돌 미보고를 무충돌로 해석하지 않음 | 실험 설정·한계 |

**1. 실제 카메라와 정책 입력의 정합**

원시 카메라 영상은 1280×720이며 ESCAPE 입력은 640×360으로 축소했다. 축소 시 시야각과 header stamp를 유지하고 내참수도 함께 조정한다. BGR 입력은 RGB로 변환하고, 왜곡 정보와 이미 보정된 영상 여부에 따라 영상·카메라 행렬을 처리한다. 이는 Python ROS 영상 직렬화 부담을 줄이는 배포 변경도 포함한다.

PixelNav에는 `calibrated_4_3` 처리를 추가했다. 저장된 카메라 보정값을 이용해 실제 영상에 존재하는 광선만으로 640×480 정책 영상을 구성하고 목표 마스크도 같은 광선 변환을 적용한다. 정책의 목표 추적 출력은 원래 RGB 좌표로 역변환한다. VLM은 원래 RGB 좌표계를 사용한다. 당시 계산된 지원 화각은 수평 약 77.125°, 수직 약 61.751°이다. 이는 저장된 calibration에 따른 값이며 새 실측 캘리브레이션 정확도나 체크포인트의 학습 카메라와 완전한 일치를 입증한 값은 아니다.

재생 비교에서는 기존 look 반복 후 STOP이던 두 초기 세션이 전진 출력으로 바뀌었으나, 다른 세션의 STOP은 남았다. 따라서 “입력 기하 정합을 구현했다”라고 쓸 수 있지만 “카메라 도메인 차이를 해결했다”라고 확대하면 안 된다.

근거: [카메라 입력 보정 기록](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_camera_contract_fix_20260910.md:31), [영상·내참수 처리](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_robot/s2e_vlm_robot/rtab_pointgoal_node.py:399), [정책 카메라 변환](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_core/s2e_vlm_core/fixed_camera_view.py:61).

**2. `look_up/down`: 실제로 가장 크게 달라진 행동**

실제 고정 카메라에는 시뮬레이터의 독립적인 상하 시선 변경을 그대로 대응시킬 구동축이 없다. 초기에는 영상 crop·확대·가상 pitch를 시험했으나 목표가 시야 밖으로 잘리거나 기존 전진 출력이 STOP으로 바뀌는 사례가 나왔다. 몸체를 기울여 실제 시선을 바꾸는 방식도 구현·시험했다. 9월 10일 실측에서는 약 14.6~15.2°의 아래보기 관측을 얻은 뒤에도 정책이 다시 look_down을 선택했고, 해당 Full 5m 시도에서 전진 명령은 0회였다.

이후 룩 출력은 짧은 실제 전진으로 대체했다. 9월 11~13일 초기 조건은 `forward_0p1`=10cm, 9월 13일 후반 및 14일 분석 대상은 `forward_0p2`=20cm다. 원래 정책의 여섯 행동 출력과 STOP은 보존한다. 실행 기록에는 raw 룩과 실제 전진을 별도로 남기고, 실제 완료 ACK와 그 이후의 새 관측이 준비되어야 다음 판단을 진행한다. 가상의 pitch 변화나 카메라 행동 성공으로 기록하지 않는다.

이는 시선 변경과 동등한 동작이라는 주장이 아니라, 현재 로봇에서 사용하는 행동 대체 규칙이다. 동일 비교 회차의 Ours와 Direct-goal PixelNav에 공통 적용한 조건으로 명시해야 한다. 20cm가 최적이라는 통제 실험의 결론도 없다. 실제 룩 표본이 대부분 look_down이므로 look_up까지 같은 물리 검증을 마쳤다고 일반화하지 않는다.

근거: [몸체 룩 실측](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_attained_body_look_trial_20260910.md), [대안 재검토](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_look_method_review_20260912.md), [20cm 적용 조건](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_self_service_goals_20260913.md:32), [현재 룩 변환 코드](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_nodes/s2e_vlm_nodes/runtime/pixnav.py:2214).

**3. 이산 행동을 실측 기반 보행·회전으로 실행**

일반 전진 0.25m와 정책 좌우 회전 30°라는 기존 매크로 규격을 실제 로봇의 속도 명령으로 연결했다. 일정 시간 명령을 보낸 뒤 완료로 간주하는 방식으로는 실제 변위를 보장할 수 없어, 오도메트리의 이동량·누적 yaw와 정지 상태를 확인하는 실행기를 사용한다.

전진에는 최대 0.5m/s, 가속도 제한, 감속, 헤딩 오차 보정, 정착 확인을 적용했다. 짧은 전진에서 목표 허용 범위 입구에 멈춘 뒤 몸체가 되밀려 보행을 반복하는 현상을 줄이기 위해 감속 목표를 허용 범위 안쪽으로 옮기고, 정지 확인 전 재보행을 억제했다. 회전에는 오차 기반 추종, 낮은 명령에서 움직이지 않던 현상에 대응한 최소 회전 명령, 정지 상태 유지와 잔여 오차 보정을 도입했다. 회전 설정은 여러 차례 바뀌었으며 최신 비교 조건의 상한·하한은 0.8rad/s였다. 이를 Go2의 정확히 식별된 물리 deadband로 서술하지 않는다.

실제 동작 시간은 시뮬레이터의 명목 시간과 다르다. 9월 14일 분석에서 동일 설정으로 완료된 일반 25cm 전진 216회의 완료시간 중앙값은 1.40초, 룩 대체 20cm 전진 178회는 1.27초, 정책 30° 회전 106회는 2.90초였다. 이는 명령·결과 로그의 동작 시간이며 외부 위치 정답에 기반한 속도 측정이 아니다. 0.5m/s를 전체 주행 평균 속도나 항상 달성하는 정속으로 쓰면 안 된다.

회전 완료 판정은 여전히 해결이 끝나지 않았다. 최신 5번 실패는 목표각 부근까지 약 2.25초에 도달했으나 미세 진동으로 정착 조건을 충족하지 못해 약 20.3초에 종료됐다. 이 기록은 제어 보완을 했다는 사실과 완전한 회전 안정성을 입증했다는 주장을 구분하게 한다.

근거: [실측 실행기](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_robot/s2e_vlm_robot/motion.py:77), [전진 정착 수정](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_jetson_post_campaign_fixes_20260911.md:53), [최신 동작 시간·회전 실패 분석](/home/unitree/go2_ws_antarctica/experiments/0914/drive_timing_review/README.md).

**4. 관측 회전의 시작 방향을 실제 pose로 유지**

실제로는 명령 종료 이후에도 몸체 방향이 변한다. 9월 11일 Run012에는 명령 종료와 다음 회전 사이 약 3.09°와 4.23°의 변화가 있었다. 각 명령이 보고한 회전량만 합하면 이 변화가 누락된다.

관측 시작 pose를 기준으로 현재 실측 odom 헤딩에서 다음 회전각을 계산하도록 수정했다. 각 관측 영상에는 그 영상 pose의 실제 상대 헤딩을 붙이고, 관측을 마친 뒤에도 시작 방향과의 오차를 새 pose로 확인한다. 구현에는 1° 복귀 확인과 최대 두 번의 추가 복귀가 있다. 실기에서는 정착 조건 때문에 실패할 수 있으므로 논문에 “복귀 오차 1° 달성”이라는 성능 수치로 쓰지 않는다.

근거: [관측 전후 방향 복귀 수정](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_jetson_post_campaign_fixes_20260911.md:32).

**5. 센서 시각·pose·행동 결과의 시간 정합**

Go2 센서 시계와 Jetson 시계의 차이에 대해 초기 1초 이상 관측에서 receipt−source의 최솟값으로 공통 오프셋을 추정하고 실행 중 고정한다. 중복·역순 메시지는 최신 상태를 갱신하지 않으며 시계 재설정·급변을 검출한다. IMU quaternion의 벤더 패킹 순서도 ROS 순서로 바로잡았다.

영상 header 시각을 앞뒤 odom 표본으로 보간하고, 해당 시각의 pose·PointGoal·내참수를 결합한다. RGB가 odom보다 꾸준히 앞서면 최신 영상 한 장만 보관하는 구현은 계속 보간에 실패할 수 있으므로, 버퍼에서 가장 최신의 보간 가능한 프레임을 선택한다. 동작 후 결과 pose와 새 영상을 확인한 뒤 정책을 갱신한다.

이 작업은 기존 비동기 좌표 보정을 실제 센서 입력에서 사용할 수 있게 한 것이다. 카메라의 현재 header는 수신 시각이며 실제 노출 시각은 보장되지 않는다. 오프셋에도 미지의 최소 전송 지연이 포함되므로 하드웨어 동기화·노출 시각 정합을 달성했다고 쓰지 않는다.

근거: [시각 보정](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_robot/s2e_vlm_robot/sensor_time.py:4), [IMU 순서 보정](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_robot/s2e_vlm_robot/sensor_input_node.py:46), [pose 보간](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_robot/s2e_vlm_robot/pose_history.py:29), [관측 버퍼 처리](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_robot/s2e_vlm_robot/rtab_pointgoal_node.py:399).

**6. 실제 동작 경계에서의 비동기 인계**

Run012에서는 회전·재관측이 필요한 다음 VLM 판단을 보류한 뒤에도 전진 명령이 네 개 더 발생했고, 해당 구간의 odom 변위가 약 1.03m였다. 기존 로컬 정책 세션이 끝날 때까지 기다리는 대신 현재 단일 행동이 완료되는 경계에서 관측 전환을 요청하도록 수정했다. 실행기 정지와 제어기 정지 확인 후 새로운 관측으로 판단한다. 동작 중 계산한 회전각을 그대로 실행하지 않는다.

함께 수정한 순서 문제는 다음과 같다.

- 관측 전환 취소 뒤 보류된 이전 GO를 다시 승인하는 경로를 차단했다.
- 완료 상태를 먼저 확정한 뒤 ACK를 보내 다음 세대와 이전 결과가 겹치지 않게 했다.
- 정상 관측 checkpoint를 BLOCKED 또는 무진행 실패로 처리하던 경로를 수정했다.
- 실제 `ODOM_STALE`, `NO_MOTION_PROGRESS` 등을 checkpoint로 덮어쓰지 않게 했다.
- 첫 정책 행동이 아직 없는 새 GO에서 관측 요청이 들어와 assertion과 ACK 누락으로 정체하던 경로를 수정했다.
- 실행 실패를 실제 다음 live VLM 요청의 `navigation_feedback`으로 전달하고, 이전 실패가 늦게 도착해 되살아나는 것을 막았다.

비동기 후보는 관측 시점의 세계 목표를 유지한 채 현재 pose에서 다시 해석·검증한다. 이미 지나쳤거나 만료된 후보를 앞으로 옮기거나 수명을 새로 부여해 수용하지 않는다. 이는 기존 causal warping 개념의 실로봇 연결·실행 순서 보완으로 설명하는 것이 정확하다. 인계 시간이 0이거나 물리 보행이 끊김 없이 연속적이라는 주장은 아니다.

근거: [회전 판단 인계 수정](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_jetson_deep_handoff_results_20260911.md:16), [checkpoint와 실제 실패 분리](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_look_method_review_20260912.md:197), [첫 동작 전 관측 전환 수정](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_jetson_postdrive_review_20260913.md:25), [live VLM 피드백 연결](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_camera_contract_fix_20260910.md:15).

**7. 좌우 부호·가까운 목표·후보 선택 보완**

실로봇 로그를 확인하다 VLM 후보의 방위각 설명이 뒤집히는 공통 구현 오류를 찾았다. ROS base_link의 +Y는 왼쪽인데 VLM 후보 설명은 오른쪽 양수를 사용했다. 이 경계에서 `atan2(y,x)`를 `atan2(-y,x)`로 수정하고 PointGoal 자체의 기존 부호는 유지했다. 이는 물리 domain gap 자체보다, 실로봇 분석으로 발견한 좌표 계약 버그 수정이다.

또 목표가 약 0.641m 남았을 때 실제 카메라가 볼 수 있는 전방 바닥은 약 0.782m부터여서, 남은 거리의 1/4·1/2·전체 거리 후보가 모두 사라지는 사례가 있었다. 가까운 목표에서는 calibration·현재 자세로 투영한 가시 바닥 후보를 사용하도록 보완했다. 후보 선별과 계획 양쪽에 목표 방향·거리에서 계산한 잔여 거리를 전달하고, 최초 세 후보가 상세 RGB 검증에서 모두 탈락하면 아직 검사하지 않은 원래 후보도 한 번씩 검증한다. RGB 검증을 통과하지 않은 후보를 강제로 승인하는 처리는 아니다.

현재의 목표 근처 정체가 모두 해결된 것은 아니다. 9월 14일 후속 분석에는 즉시 STOP 반복 후 탈출 후보를 선택해 최종 목표에서 멀어졌는데 실제 이동량 때문에 `progress=true`로 기록된 사례가 남아 있다. 따라서 “목표 근처 복구를 해결했다”보다 수정한 후보 생성·정보 전달 범위를 구체적으로 쓰는 편이 정확하다.

근거: [VLM 좌우 부호 수정](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_jetson_post_campaign_fixes_20260911.md:7), [가시 바닥·후보 선별 보완](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_near_goal_review_20260912.md:7), [남아 있는 탈출 처리 문제](/home/unitree/go2_ws_antarctica/experiments/0914/pixnav_failure_followup/README.md).

**8. 위치 기준, 실행 환경, 충돌·성공 판정의 차이**

처음에는 RTAB-Map 좌표와 최초 pose를 연결하는 SE(2) 변환을 구현했다. 초기 위치 p0뿐 아니라 초기 헤딩 yaw0도 제거하여 `p_session = R(-yaw0)(p_map-p0)`로 변환한다. 현재 위치 기준 PointGoal은 같은 좌표계 안에서 최종 목표와의 거리·방향으로 계산한다.

후속 실기에서 정지 중 map yaw가 약 22° 범위로 변하는 사례가 나와, 최근 반복 비교는 표시한 동일 시작 위치·방향에서 odometry 원점을 다시 설정하는 `fixed_start_odometry`를 사용했다. 목표 캡처와 매핑 자료가 있다는 사실이 주행 중 RTAB-Map 전역 정합을 사용했다는 뜻은 아니다. 현재 조건은 시작 배치 오차와 odometry drift가 남는 상대 위치 기반 주행이다. RTAB-Map/occupancy/Nav2를 ESCAPE 경로계획기로 사용한 실험으로 서술하면 안 된다.

배포는 Jetson의 정책 실행과 원격 VLM 호출을 연결하고, Jazzy 내비게이션과 Foxy Unitree 명령 sidecar를 Unix 소켓으로 연결했다. 명령 소유권·순서·만료시간을 확인하고 별도 스레드의 300ms deadman으로 명령 갱신 중단 시 StopMove를 보낸다. 이 기제는 새로운 장애물 회피 알고리즘이 아니다.

시뮬레이터의 충돌 플래그에 대응하는 검증된 물리 충돌 센서는 현재 없다. 초기 포팅 때 추가했던 LiDAR 근접 차단은 9월 7일 제거했으며, 현재 실행기에서 점군을 장애물 veto로 사용하지 않는다. `collided=false`는 충돌이 보고되지 않았다는 의미이지 접촉이 없었다는 증거가 아니다.

최근 회차는 최종 목표 반경 1m, 최종 yaw 미요청으로 평가했다. 이는 구현 설정·평가 프로토콜에 명시할 사항이다. 앞선 25cm 반경 회차와 섞지 않는다. 기존 360초 회차를 1800초 조건의 회차로 다시 분류하지 않는다. 전체 1800초 준비 변경과 별개로 Direct에는 300초 TTL과 33번째 판단의 native history 종료 제한이 남아 있다는 최신 분석이 있으므로, 장거리 비교의 시간·세션 조건이 이미 정합됐다고 쓰지 않는다.

근거: [초기 map 좌표 연결](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_minimal.md:50), [최근 고정 시작점과 map 불안정성](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_fixed_start_and_old_map_goal_20260913.md), [실로봇 인터페이스 설명](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_vlm_robot/README.md), [독립 deadman](/home/unitree/s2e-vlm-async-framework-minimal/src/s2e_unitree_foxy/s2e_unitree_foxy/unitree_driver_node.py:264), [충돌 처리 변경](/home/unitree/s2e-vlm-async-framework-minimal/docs/robot_full_audit.md:1).

**논문 본문용 초안 — 최근 20cm 비교 조건을 기술하는 경우**

ESCAPE-Nav를 Unitree Go2에 적용하기 위해 기존 VLM–PixelNav 비동기 구조에 실로봇 입출력 어댑터를 구현하였다. 실제 카메라 영상의 색상·왜곡·내참수를 처리하고, RGB 영상과 목표 마스크를 동일한 광선 변환으로 4:3 정책 입력에 정합하였다. 로봇 센서와 호스트 사이의 시각 오프셋을 초기 관측으로 추정하였으며, 영상 header 시각에 대응하는 오도메트리 자세를 보간하여 관측과 목표의 기하학적 일관성을 유지하였다.

시뮬레이터의 0.25m 전진 및 30° 회전은 오도메트리로 변위와 정착을 확인하는 폐루프 실행기로 변환하였다. 실행기는 가감속 제한과 전진 중 헤딩 보정을 적용하며, 명령 발송 후 실제 동작 결과와 후속 관측이 준비되어야 다음 정책 판단을 진행한다. 고정 카메라의 상하 시선 변경 제약을 처리하기 위해 최근 비교 실험에서는 look_up/down 출력을 0.20m 전진으로 대체하였다. 이 규칙은 ESCAPE-Nav와 Direct-goal PixelNav에 공통 적용하였으며, 원래 정책 출력과 실제 실행 행동을 구분하여 기록하였다.

실제 동작 시간과 응답 순서의 변동을 처리하기 위해 비동기 결정의 인계를 단일 행동 완료 경계에 연결하였다. 방향 변경이나 재관측이 필요한 경우 현재 행동의 완료와 정지를 확인한 뒤 관측을 수행하며, 각 영상의 실제 상대 헤딩과 시작 방향을 이용해 관측 회전 및 복귀를 계산한다. 또한 의도적인 관측 전환과 실행 실패를 구분하고, 취소되거나 만료된 이전 결정을 재실행하지 않도록 하였다. 가까운 목표에서 후보가 카메라의 가시 바닥 영역 밖으로 사라지는 경우에는 가시 바닥 후보를 사용하도록 후보 생성을 보완하였다.

실로봇 비교는 표시된 동일 시작 위치와 방향에서 원점을 설정한 상대 오도메트리 좌표계를 사용하였다. 최종 목표 반경은 1m로 설정하였고 최종 방향 정렬은 요구하지 않았다. 이러한 설정은 전역 재위치추정 또는 외부 정답 위치를 제공하지 않으므로, 추정 도착 거리와 경로 길이는 오도메트리 기반 지표로 해석하였다.

**초안 사용 시 반드시 맞춰야 하는 내용**

위 초안은 구현을 설명한다. 각 변경의 성능 향상이나 모든 결함의 해결을 주장하는 문단이 아니다. 카메라 header의 실제 노출 시각, 물리 캘리브레이션 정확도, 회전 정착과 센서 처리 지연, 목표 근처 STOP·탈출 문제는 남아 있다. 10cm 실험을 기술한다면 0.20m를 0.10m로 바꾸고 회전·시간·도착 기준도 해당 회차 manifest에 맞춰야 한다.

본문의 Sim-to-Real 설명에는 카메라 정합·행동 대체·실측 실행·시간 정합·비동기 인계를 우선 포함한다. 좌우 부호나 assertion 수정은 구현 검증 또는 부록에 두는 편이 적절하다. 실제 구현 개선량을 별도로 주장하려면 같은 초기 조건에서의 수정 전후 실기 비교가 필요하며, 단위 검사 수나 녹화 입력의 추론 재현 수를 로봇 성공률로 바꾸지 않는다.
