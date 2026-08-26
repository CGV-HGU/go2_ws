# 📸 [Domain 01] 실제 로봇 내장 카메라 실시간 라이브 영상 & 궤적(Trajectory) 추출 갤러리

이 폴더는 Unitree Go2 실물 로봇의 전면 초광각 내장 카메라($1280\times 720$ RGB, 지상고 $h=0.45\text{m}$)로 **지금 현재 로봇 전원을 켜고 실시간 수신한 30fps 라이브 스트림(`230.1.1.1:1720`) 및 5초 주행 영상**에 대해 **원격 Qwen3.5-9B VLM 서버가 실시간으로 추출한 50Hz 보행 궤적(Trajectory) 및 운전자 HUD 화면**을 보관합니다.

* 📝 **실험 누적 기록 일지**: [`EXPERIMENT_RECORD_LOG.md`](EXPERIMENT_RECORD_LOG.md) (날짜, 시간, 로봇 자세, VLM 지연시간, 서브골 픽셀 누적 일지)

---

## 🎬 1. [최신 🌟] 연구실 중앙 기립(Standing) 5초 라이브 궤적 추출 (5-Second Animated GIF)
* **파일명**: `live_robot_camera_trajectory_5s.gif` (및 `live_robot_camera_trajectory_5s.mp4`)
* **촬영 일시**: `2026-08-26 14:45:49 KST` (Session ID: `EXP-20260826-002`)
* **로봇 자세 및 위치**: **Standing (직립 기립, 지상고 $h=0.45\text{m}$) / 연구실 중앙 통로**
* **VLM 추론 결과**: Action: **`GO`**, Subgoal Pixel: **`[640, 503]`** ($X=1.89\text{m}, Y=0.00\text{m}$), Latency: **$820.9\text{ms}$**
* **VLM Reasoning**: *"The robot is positioned in a narrow corridor between two desks. The floor is clear of obstacles directly ahead, allowing for a straight path. The robot should continue moving forward along this clear path."*

![Live Robot Camera Trajectory 5s Animation](live_robot_camera_trajectory_5s.gif)

---

## 🔴 2. [실시간 단일 프레임] 현재 로봇 전면 카메라 스냅샷 궤적 추출 (Single Snapshot FPV)
* **파일명**: `live_front_camera_now_trajectory.png`
* **영상 출처**: 실물 로봇 전면 카메라 실시간 프레임 캡처 ($h=0.45\text{m}$)
* **VLM 결정**: Action: `GO`, Subgoal Pixel: `[550, 482]` ($X=1.72\text{m}, Y=0.26\text{m}$)

![Live Front Camera Now Trajectory](live_front_camera_now_trajectory.png)

---

## 🚀 3. 실시간 라이브 영상 5초 재촬영 및 궤적 추출 명령어
```bash
# 1. 5초 실시간 영상 재촬영
ffmpeg -protocol_whitelist file,udp,rtp -i /home/unitree/go2_ws_antarctica/scratch/go2_camera.sdp -t 5 -c:v libx264 -pix_fmt yuv420p -r 15 /home/unitree/go2_ws_antarctica/scratch/live_camera_raw_5s.mp4 -y

# 2. VLM 궤적 추출 및 MP4/GIF 생성
python3 /home/unitree/go2_ws_antarctica/scratch/process_live_video_trajectory.py
```
