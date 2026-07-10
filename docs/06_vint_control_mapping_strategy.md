# 🎯 ViNT 궤적 제어 매핑 및 실물 구동 전략 (Control Mapping & Tuning)

본 문서는 AI 모델의 출력물(10x2 Waypoints)을 실물 Go2 로봇의 고해상도 운동학(Sport API)으로 변환하는 최적의 운동학 매핑 및 필드 튜닝 전략을 기술함.

---

## 📌 1. 제어 알고리즘 비교분석 (순정 vs 개선 피드백)

*   **ViNT 순정 제어기 ([pd_controller.py](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/visualnav-transformer/deployment/src/pd_controller.py#L43-L62))**:
    *   수식: $v = \frac{dx}{DT}$ / $w = \frac{\arctan(dy/dx)}{DT}$
    *   한계: 위치 에러 변위를 제어 주기($DT \approx 0.11\text{ s}$)로 직접 나누므로, AI가 출력하는 미세한 Jitter에도 모터 속도가 급변하여 Go2의 관절에 심한 하드웨어 충격 및 흔들림을 유발함.
*   **개선된 피드백 PD 제어기 (`ros_mock_runtime.py` 탑재)**:
    *   선속도: $v = K_{p\_lin} \times \text{hypot}(dx, dy)$ (선속도 비례 제어)
    *   각속도: $w = K_{p\_ang} \times \text{atan2}(dy, dx) + K_{d\_ang} \times d\_heading$ (각속도 비례-미분 제어)
    *   장점: 급격한 회전각 발생 시 감속 제어가 자동 인가되며, 미분 제어기($K_{d\_ang}$)가 로봇의 엉덩이가 좌우로 요동치는 피쉬테일링(Fishtailing) 현상을 댐핑하여 억제함.

---

## 📌 2. Go2 Sport API 매핑 및 보행 안정성 확보

로봇의 보행 구동을 담당하는 파이썬 드라이버(`python_direct_driver.py`) 및 C++ 드라이버는 수신된 속도 명령을 최종적으로 다음과 같이 Sport API에 매핑함.

$$\text{SportClient.Move}(v_x, v_y, v_{yaw}) \Longrightarrow (v, 0.0, w)$$

*   **측면 주행 제한 ($v_y = 0.0$)**: Go2는 게걸음(Strafing, 측면 이동) 보행 시 발끝의 지상고(Foot clearance) 마진이 줄어들어 미끄러운 바닥이나 장애물 환경에서 전도 위험이 급격히 증가함. 따라서 측면 속도는 의도적으로 $0.0$으로 하드코딩 격리하고, 회전($v_{yaw}$)과 전진($v_x$)만으로 곡선 궤적을 추종하도록 세팅함.

---

## 📌 3. 화요일 필드 최적화 튜닝 가이드 (Tuesday Field Tuning)

현장에서 S2E 모델을 탑재하고 주행을 켰을 때, 로봇의 움직임이 비정상적일 경우 대응하는 매개변수 조절법.

1.  **로봇이 뱀처럼 구불구불 걸어갈 때 (Wobbling / Fishtailing)**:
    *   **원인**: 회전 반응도($K_{p\_ang}$)가 너무 크거나 제동 댐핑($K_{d\_ang}$)이 너무 약함.
    *   **조치**: `kp_angular` 값을 줄이거나 `kd_angular` 값을 서서히 높여가며 흔들림을 댐핑함.
2.  **경로 모퉁이를 돌 때 로봇이 회전을 안 하고 들이받을 때 (Understeering)**:
    *   **원인**: 룩어헤드 목표점(Lookahead Point)을 너무 먼 곳으로 잡았음.
    *   **조치**: 기본 룩어헤드 인덱스 `lookahead_index`를 `3`에서 `2` 또는 `1`로 줄여서, 로봇이 아주 가까운 곳에 반응해 즉각 꺾이도록 제어 주기를 조여줌.
3.  **직진 시 속도가 너무 느리거나 지나치게 과속할 때**:
    *   **원인**: 선속도 게인($K_{p\_lin}$) 밸런스 불일치.
    *   **조치**: `kp_linear` 값을 조절하여 로봇의 최대 안전 속도 한계선(`max_linear_speed`, 권장: $0.3 \sim 0.4\text{ m/s}$)을 락온함.
