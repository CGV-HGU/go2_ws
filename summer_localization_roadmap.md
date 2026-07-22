# Summer Branch: Localization & Nav2 Integration Roadmap

이 문서는 `summer` 브랜치에서 `rtabmap`의 Localization(위치 추정) 모드와 `Nav2` 자율주행 패키지를 정상적으로 튜닝하고 연동하기 위한 상세 가이드라인입니다.

---

## 📌 1. 핵심 개선 대상 및 가이드

### 1.1 `run_localization.sh` 카메라 파라미터 경량화
현재 `run_localization.sh`의 RealSense 런치 부분에 `run_map.sh`에서 검증된 설정을 동일하게 이식해야 합니다.

* **수정 전 (대역폭 초과 및 크래시 유발)**:
  ```bash
  ros2 launch realsense2_camera rs_launch.py \
      initial_reset:=true \
      align_depth:=true \
      enable_sync:=true \
      depth_module.profile:=640x480x30 \
      rgb_camera.profile:=640x480x30
  ```
* **수정 후 (30fps 및 전원 관리 안정화)**:
  ```bash
  ros2 launch realsense2_camera rs_launch.py \
      initial_reset:=false \
      align_depth.enable:=true \
      enable_color:=true \
      enable_sync:=false \
      enable_sync.enable:=false \
      depth_module.profile:=424x240x30 \
      rgb_camera.profile:=424x240x30 \
      enable_accel:=true \
      enable_gyro:=true \
      unite_imu_method:=2 \
      publish_tf:=true \
      global_time_enabled:=false \
      hold_back_imu_for_frames:=true
  ```

### 1.2 RTAB-Map 뎁스 토픽 일치화
RTAB-Map Localization 노드 실행 시 잘못 지정된 뎁스 토픽을 aligned depth로 갱신합니다.
* **수정 전**: `depth_topic:=/camera/depth/image_rect_raw`
* **수정 후**: `depth_topic:=/camera/aligned_depth_to_color/image_raw`

### 1.3 비주얼 오도메트리 활성화 및 토픽 매핑
카메라 VIO 기반 주행을 위해 Localization 실행 시에도 `visual_odometry:=true`와 `odom_topic:=/odom`을 인자에 추가해야 합니다.
* **추가할 파라미터**:
  ```bash
  visual_odometry:=true \
  odom_topic:=/odom \
  odom_info_topic:=/odom_info \
  ```

### 1.4 Nav2 cmd_vel과 go2_driver 간의 토픽 매핑 검증
`go2_driver`는 `/cmd_vel`을 받아 Unitree Sport Mode SDK 형식(JSON)으로 변환해 로봇 바디에 전달합니다. 
* **주의**: Nav2 가 발행하는 속도 명령 토픽(`cmd_vel` 또는 `cmd_vel_nav` 등)이 `go2_bringup` 노드가 구독하는 토픽명과 100% 동일하게 일치(Remap)되도록 `navigation_launch.py` 설정을 꼼꼼히 대조해야 합니다. 그렇지 않으면 경로 계획은 되지만 로봇이 물리적으로 전혀 움직이지 않는 현상이 발생합니다.

---

## 🏃‍♂️ 2. `summer` 브랜치에서의 튜닝 절차 (체크리스트)

1. **브랜치 상태 확인**: 
   - 현재 브랜치가 `summer`인지 확인합니다 (`git branch`).
2. **`run_localization.sh` 수정**:
   - 위의 개선 가이드를 바탕으로 `run_localization.sh` 내부의 카메라 및 RTAB-Map 인자값을 변경합니다.
3. **네트워크 활성화**:
   - `./set_both.sh`를 실행하여 인터넷 연결과 로봇 통신을 동시에 활성화합니다.
4. **Localization 모드 기동**:
   - `pkill` 명령어로 기존 프로세스를 정리한 후, `./run_localization.sh`를 구동합니다.
5. **토픽 검증**:
   - 아래 명령어를 통해 `/odom` 토픽과 자율주행용 2D 레이저 스캔 `/scan` 토픽이 정상 속도로 방출되는지 확인합니다:
     ```bash
     ros2 topic hz /odom
     ros2 topic hz /scan
     ```
6. **RViz 상에서 자율주행 테스트**:
   - RViz가 실행되면 `2D Pose Estimate`로 로봇의 현재 위치를 지도의 특징점에 맞춰 설정해 줍니다.
   - `2D Goal Pose` 버튼을 클릭하여 목적지를 찍어주고, 로봇 바디가 자율주행 경로를 계획하고 움직이는지 모니터링합니다.
