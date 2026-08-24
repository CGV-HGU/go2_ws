# 🐳 [04] 도커(Docker) 정지/포복 상태 전면 카메라 VLM 궤적 테스트 & 안전 매뉴얼

> **문서 번호**: `docs/docker_plan/04_docker_stationary_vlm_safety_test_runbook.md`  
> **작성 일자**: 2026년 8월 24일 (KST)  
> **문서 소유자**: **도커/S2E 자율주행 관리 AGY** & **민석 (Minseok)**  
> **논문 공식 명칭**: **`ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation` (ICRA)**  
> **대상 컨테이너**: `sdam_go2_container` (Ubuntu 24.04 LTS Noble / ROS 2 Jazzy ARM64 / Python 3.12)  
> **연동 스크립트**: [`scratch/test_docker_stationary_vlm_trajectory.py`](file:///home/unitree/go2_ws_antarctica/scratch/test_docker_stationary_vlm_trajectory.py)

---

## 📌 1. 개요 및 목적
로봇 본체(Go2 Base 메인보드 및 모터) 전원이 꺼져 있거나 로봇이 바닥에 포복(Prone)해 있는 상태에서, **원격 VLM 추론 서버(`100.96.60.15:8000`)와의 시각 추론, 50Hz 지상 주행 궤적 연산, 페일세이프 안전 인터록, 시각화 HUD 생성을 안전하게 단 1줄 명령어로 전수 검증**하기 위한 공식 운영 런북입니다.

---

## 🛡️ 2. 물리적 모터 구동 0 안전 규정 (Zero-Velocity Safety Clamping)
* 본 테스트 스크립트는 로봇의 돌발 움직임을 원천 방지하기 위해 **모터 제어 속도 명령($v_x, \omega_z$)을 `0.0`으로 완전 고정(Zero Clamped)**합니다.
* `AGENTS.md`의 Real-robot Safety 규정을 100% 준수하여 물리적 모터 구동 없이 알고리즘 파이프라인만 검증합니다.

---

## 🔍 3. 카메라 입력 하이브리드 폴백 구조
1. **로봇 본체 부팅 시**: GStreamer 파이프라인을 통해 Go2 내장 전면 카메라 H.264 실시간 스트림(`230.1.1.1:1720`, 720p 30fps)을 자동 캡처합니다.
2. **로봇 본체 미부팅 시**: `docs/docker/visualizations/01_robot_camera_fpv_view/01_real_corridor_vlm_subgoal_fpv.png` 등 실제 복도 실사 FPV 프레임으로 지연 없이 즉시 폴백합니다.

---

## 🐟 4. Go2 전면 카메라 초광각/어안 왜곡 분석 및 궤적 투영
* **VLM 인식 강건성**: Qwen3-VL 모델은 대규모 광각/액션캠 데이터로 사전 학습되어 왜곡 이미지에서도 복도 중앙 소실점($u=640$)과 바닥 자유공간을 정확히 식별합니다.
* **중심 시야($60^\circ$) 선형성**: 방사 왜곡이 외곽($>100^\circ$)에 집중되어 있어, 전방 주행 목표 영역은 왜곡률이 $3\%$ 미만으로 매우 안정적입니다.
* **지상 3D 역투영 공식**:
  $$X = \frac{h}{\tan(\theta_p + \Delta\theta_v)}, \quad Y = \frac{u - c_x}{f_x} \cdot X$$
  카메라 지상고 $h=0.35\text{m}$를 기반으로 10개의 연속 웨이포인트 $(x_0..x_9, y_0..y_9)$ 국소 주행 궤적을 연산합니다.

---

## 🚀 5. 1-Click 실행법 및 결과 확인

```bash
# 도커 컨테이너 내부 환경에서 실행
docker exec -it sdam_go2_container python3 /workspace/go2_ws_antarctica/scratch/test_docker_stationary_vlm_trajectory.py
```

* **산출물**:
  - `scratch/stationary_test_vlm_trajectory.png`: VLM 서브골 조준선 + 녹색 10-Waypoint 궤적선 + 안전 상태 HUD가 합성된 1280x720 고해상도 이미지.
