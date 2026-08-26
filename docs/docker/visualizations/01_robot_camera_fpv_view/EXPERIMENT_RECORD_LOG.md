# 📝 [Experiment Record Log] 실물 로봇 전면 카메라 실시간 라이브 영상 궤적 추출 실험 기록 일지

> **문서 위치**: `docs/docker/visualizations/01_robot_camera_fpv_view/EXPERIMENT_RECORD_LOG.md`  
> **총괄 책임**: **도커 관리자 & S2E 자율주행 총괄 (Docker Administrator & S2E Autonomy Lead)**  
> **기록 목적**: 실물 Unitree Go2 전면 카메라의 실시간 라이브 스트림(`230.1.1.1:1720`)을 원격 Qwen3.5-9B VLM 서버와 연동하여 추출한 모든 5초 동영상/GIF 및 50Hz 지면 궤적 추출 세션을 날짜, 시각, 로봇 상태별로 누적 기록합니다.

---

## 📊 1. 누적 실험 세션 요약표 (Experiment Summary Table)

| 세션 ID | 촬영 일시 (KST) | 로봇 자세 및 위치 | VLM 모델 | 지연시간 (ms) | 결정 Action | 목표점 [u, v] | 지면 거리 (X, Y) | 미디어 파일 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`EXP-20260826-002`** | **2026-08-26 14:45:49** | **기립 (Standing)<br/>연구실 중앙 통로** | `qwen3.5-9b-instruct` | **$820.9\text{ ms}$** | **`GO` (직진)** | **`[640, 503]`** | **$X=1.89\text{m}, Y=0.00\text{m}$** | [5초 GIF](live_robot_camera_trajectory_5s.gif) / [MP4](live_robot_camera_trajectory_5s.mp4) |
| **`EXP-20260826-001`** | **2026-08-26 14:40:48** | **와상 (Prone Standby)<br/>연구실 의자 사이** | `qwen3.5-9b-instruct` | **$725.7\text{ ms}$** | **`TURN_LEFT`** | **`[256, 360]`** | **$X=7.00\text{m}, Y=4.00\text{m}$** | [1차 와상 GIF](live_robot_camera_trajectory_5s.gif) |

---

## 🔬 2. 세션별 정밀 실험 분석

### 📍 [Session EXP-20260826-002] 연구실 중앙 직립(Standing) 5초 주행 궤적 추출 (최신 🌟)

* **촬영 일시**: `2026-08-26 14:45:49 KST`
* **로봇 하드웨어 상태**:
  * 로봇 베이스 IP: `192.168.123.161` (Online, Ping RTT $0.18\text{ms}$)
  * 로봇 자세: **Standing (직립 기립 모드)**
  * 카메라 렌즈 지상고: $h = 0.45\text{m}$
  * 위치: 연구실 좌/우 책상 정중앙 통로
* **원격 VLM 추론 메트릭**:
  * 원격 서버 엔드포인트: `http://100.96.60.15:8000/v1` (NetBird VPN)
  * 적용 모델: `qwen3.5-9b-instruct`
  * 추론 지연시간: **$820.9\text{ ms}$**
  * 결정 Action: **`GO`** (선속도 $v_x = +0.16\text{ m/s}$, 각속도 $\omega_z = 0.00\text{ rad/s}$)
  * Subgoal Pixel: `[640, 503]` (1280x720 화면의 정중앙 바닥면)
  * 3D 지면 역투영 거리: $X = +1.89\text{m}$, $Y = 0.00\text{m}$
* **VLM 실시간 추론 코멘트**:
  > *"The robot is positioned in a narrow corridor between two desks. The floor is clear of obstacles directly ahead, allowing for a straight path. The robot should continue moving forward along this clear path."*  
  > (로봇이 두 책상 사이의 좁은 통로에 위치해 있습니다. 정면 바닥에 장애물이 없어 직선 경로가 확보되므로, 로봇은 이 안전한 경로를 따라 계속 직진해야 합니다.)
* **미디어 산출물**:
  * 5초 애니메이션 GIF: [`live_robot_camera_trajectory_5s.gif`](live_robot_camera_trajectory_5s.gif)
  * 5초 원본 MP4 비디오: [`live_robot_camera_trajectory_5s.mp4`](live_robot_camera_trajectory_5s.mp4)

---

### 📍 [Session EXP-20260826-001] 의자 사이 와상(Prone Standby) 5초 주행 궤적 추출

* **촬영 일시**: `2026-08-26 14:40:48 KST`
* **로봇 하드웨어 상태**:
  * 로봇 자세: **Prone Standby (와상 대기 모드)**
  * 카메라 렌즈 지상고: $h = 0.35\text{m}$
* **원격 VLM 추론 메트릭**:
  * 추론 지연시간: **$725.7\text{ ms}$**
  * 결정 Action: **`TURN_LEFT`** (우측 의자 다리 장애물 회피 판단)
  * Subgoal Pixel: `[256, 360]` ($X = +7.00\text{m}$, $Y = +4.00\text{m}$)
* **VLM 실시간 추론 코멘트**:
  > *"The robot is currently facing a desk and office chairs. Identifying the open leftward space to navigate safely around leg obstacles."*

---

## 🚀 3. 실시간 5초 세션 자동 녹화 및 기록 명령어

```bash
# 1. 5초 실시간 영상 촬영
ffmpeg -protocol_whitelist file,udp,rtp -i /home/unitree/go2_ws_antarctica/scratch/go2_camera.sdp -t 5 -c:v libx264 -pix_fmt yuv420p -r 15 /home/unitree/go2_ws_antarctica/scratch/live_camera_raw_5s.mp4 -y

# 2. VLM 궤적 추출 및 MP4/GIF 생성
python3 /home/unitree/go2_ws_antarctica/scratch/process_live_video_trajectory.py
```


### 📍 [Session EXP-20260826-145125] 15초 실시간 연속 VLM 폐루프 스트리밍 세션

* **세션 일시**: `2026-08-26 14:51:25 KST`
* **총 실행 시간**: 15초 (152 프레임 @ 15.0fps)
* **총 VLM 질의 횟수**: **16회 연속 수행**
* **미디어 파일**: [`live_continuous_vlm_trajectory_15s.gif`](live_continuous_vlm_trajectory_15s.gif) / [`live_continuous_vlm_trajectory_15s.mp4`](live_continuous_vlm_trajectory_15s.mp4)

| 질의 ID | 질의 시각 | VLM 지연시간 | 결정 Action | Subgoal [u, v] | Metric (X, Y) | VLM 추론 내용 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`Query #01`** | `14:51:09.641` | **759.4 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The path ahead is clear between the two offic... |
| **`Query #02`** | `14:51:10.612` | **708.9 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The path ahead is clear between the two offic... |
| **`Query #03`** | `14:51:11.540` | **764.7 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, unobstructed p... |
| **`Query #04`** | `14:51:12.513` | **749.8 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, unobstructed p... |
| **`Query #05`** | `14:51:13.472` | **722.8 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The path forward is clear between the two off... |
| **`Query #06`** | `14:51:14.404` | **817.0 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, unobstructed p... |
| **`Query #07`** | `14:51:15.441` | **892.8 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, unobstructed p... |
| **`Query #08`** | `14:51:16.542` | **861.2 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, unobstructed p... |
| **`Query #09`** | `14:51:17.612` | **774.3 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, straight path ... |
| **`Query #10`** | `14:51:18.595` | **646.4 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The path ahead is clear between the two offic... |
| **`Query #11`** | `14:51:19.451` | **640.0 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The path ahead is clear and unobstructed, all... |
| **`Query #12`** | `14:51:20.299` | **672.5 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The path ahead is clear, with no immediate ob... |
| **`Query #13`** | `14:51:21.181` | **729.0 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, unobstructed p... |
| **`Query #14`** | `14:51:22.119` | **708.2 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The path ahead is clear and unobstructed, all... |
| **`Query #15`** | `14:51:23.037` | **772.9 ms** | `GO` | `[640, 432]` | `X=3.75m, Y=-0.00m` | The path directly ahead is clear, with the ma... |
| **`Query #16`** | `14:51:24.019` | **803.9 ms** | `GO` | `[640, 503]` | `X=1.89m, Y=-0.00m` | The camera view shows a clear, unobstructed p... |
