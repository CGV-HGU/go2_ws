# 📸 [Domain 01] 실제 로봇 내장 카메라 실사 기반 서버 궤적(Trajectory) 추출 갤러리

이 폴더는 Unitree Go2 실물 로봇의 전면 초광각 내장 카메라($1280\times 720$ RGB, 지상고 $h=0.35\text{m}$)로 **지금 현재 실시간 수신한 라이브 영상 스트림 및 실제 환경**에 대해 **원격 Qwen3.5-9B VLM 서버가 실시간으로 추출한 50Hz 보행 궤적(Trajectory) 및 운전자 HUD 화면**을 보관합니다.

---

## 🔴 1. [방금 실시간 캡처] 로봇 전면 카메라 라이브 스트림 궤적 추출 (Live Stream FPV)
* **파일명**: `live_front_camera_now_trajectory.png`
* **영상 출처**: 로봇 내장 카메라 RTP 실시간 멀티캐스트 스트림 (`230.1.1.1:1720`)에서 직접 프레임 캡처!
* **VLM 추론 결과**: Action: `GO`, Subgoal: `[512, 380]` ($X=7.00\text{m}, Y=1.49\text{m}$), Latency: **$769.5\text{ms}$**
* **VLM Reasoning**: *"The camera is positioned low, looking forward between office chairs. The floor is clear and unobstructed in the immediate path ahead, allowing for safe forward movement."*

![Live Front Camera Now Trajectory](live_front_camera_now_trajectory.png)

---

## 🖼️ 2. 실제 복도 주행 실사 궤적 추출 (Corridor Hallway)
* **파일명**: `server_extracted_corridor_trajectory.png`
* **원본 키프레임**: `scratch/rtabmap_preview/node_0497.jpg` (Go2 실측 복도 SLAM 프레임)
* **VLM 추론 결과**: Action: `GO`, Subgoal: `[640, 503]` ($X=1.47\text{m}, Y=-0.00\text{m}$), Latency: **$721.4\text{ms}$**
* **VLM Reasoning**: *"The hallway is clear with no obstacles on the floor. The path ahead is straight and unobstructed, allowing the robot to continue moving forward."*

![Server Extracted Corridor Trajectory](server_extracted_corridor_trajectory.png)

---

## 🖼️ 3. 연구실 출발 지점 실사 궤적 추출 (Lab Room Start)
* **파일명**: `server_extracted_lab_trajectory.png`
* **원본 키프레임**: `scratch/rtabmap_preview/node_0001.jpg` (연구실 책상/의자 사이 FPV)
* **VLM 추론 결과**: Action: `GO`, Subgoal: `[640, 503]` ($X=1.47\text{m}, Y=-0.00\text{m}$), Latency: **$826.7\text{ms}$**
* **VLM Reasoning**: *"The robot is positioned in a clear area between two office chairs. The floor is unobstructed in the immediate foreground, allowing for safe forward movement."*

![Server Extracted Lab Trajectory](server_extracted_lab_trajectory.png)

---

## 🖼️ 4. 목표물 정밀 접근 실사 궤적 추출 (Target Approach)
* **파일명**: `server_extracted_approach_trajectory.png`
* **원본 키프레임**: `scratch/rtabmap_preview/node_0992.jpg` (복도 끝 콘센트/박스 타겟)
* **VLM 추론 결과**: Action: `GO`, Subgoal: `[640, 503]` ($X=1.47\text{m}, Y=-0.00\text{m}$), Latency: **$778.4\text{ms}$**
* **VLM Reasoning**: *"The camera view shows a clear, unobstructed path forward between the two office chairs. The floor is flat and free of obstacles in the immediate center."*

![Server Extracted Approach Trajectory](server_extracted_approach_trajectory.png)

---

## 🚀 5. 실시간 1-Click 라이브 궤적 추출 스크립트
```bash
python3 /home/unitree/go2_ws_antarctica/scratch/extract_server_trajectory_pipeline.py
```
