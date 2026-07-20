# 📋 Unitree Go2 SLAM & 자율주행 통합 구동 검증 체크리스트 (통합본)

본 문서는 로봇을 부팅한 직후 **물리적 상태, 네트워크, DDS, 센서 데이터, TF 좌표계, 자율주행 제어 토픽까지 한 번에 점검하고 검증**할 수 있도록 설계된 통합 체크리스트입니다. 

각 항목별로 실행해야 하는 구체적인 검증 명령어와 예상되는 정상 출력 결과가 포함되어 있습니다.

---

## 1단계: 물리적 하드웨어 점검 (기초 셋업)
로봇 전원을 켜기 전/후 물리적으로 점검해야 하는 상태입니다.

- [ ] **D435i USB 3.0 포트 체결**: 
  * Jetson 보드의 **파란색 USB 3.0 포트**에 연결되었는지 확인합니다.
- [ ] **이더넷 LAN선 체결**:
  * Jetson 보드의 기본 유선 포트(`eth0`)에 LAN 케이블이 단단히 연결되었는지 확인합니다.
- [ ] **로봇 전원 기동**:
  * 로봇 배터리 전원을 켜고 로봇이 완전히 기립(Stand)하여 네트워크 통신이 준비될 때까지 기다립니다.

---

## 2단계: 네트워크 및 OS 환경 변수 점검 (통신 기초)
노트북과 로봇(Jetson) 간에 ROS 2 통신을 하기 위한 IP 및 DDS 환경 변수가 올바르게 셋업되었는지 한 번에 점검합니다.

- [ ] **Jetson 네트워크 멀티 IP 상태 확인**:
  * Jetson에서 `set_both.sh` 또는 `set_robot.sh`를 가동한 뒤, 아래 명령어로 확인합니다.
  ```bash
  ip addr show eth0 | grep "inet "
  ```
  * *정상 출력 예시*: 
    * 로봇 내부망 IP: `inet 192.168.123.99/24`
    * 외부망(학교랜/Wi-Fi) IP: `inet 203.252.107.219/25` (동시 활성화 확인)
- [ ] **양방향 네트워크 Ping 테스트**:
  * **노트북 터미널**: `ping 203.252.107.219` (Jetson 외부망 IP)
  * **Jetson 터미널**: `ping 192.168.123.161` (로봇 본체 내부 IP)
- [ ] **Jetson ROS 2 환경 변수 확인**:
  * Jetson 터미널에서 아래 명령어로 환경 변수가 제대로 설정되었는지 점검합니다.
  ```bash
  printenv | grep -E "ROS|RMW|CYCLONE"
  ```
  * *정상 출력 예시*:
    * `ROS_DISTRO=foxy`
    * `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
    * `CYCLONEDDS_URI=file:///home/unitree/go2_ws/cyclonedds.xml`
    * `ROS_DOMAIN_ID=0`

---

## 3단계: USB 및 카메라 하드웨어 점검 (센서 기초)
Jetson 내부에서 RealSense 카메라 및 IMU 센서가 하드웨어적으로 전원 제한 없이 완벽히 구동 중인지 확인합니다.

- [ ] **USB 자동 절전 모드 비활성화 상태 확인**:
  ```bash
  cat /sys/bus/usb/devices/2-2/power/control      # 정상 출력: on
  cat /sys/bus/usb/devices/2-2/power/autosuspend  # 정상 출력: -1
  ```
- [ ] **D435i IMU 하드웨어 드라이버(usbhid) 연결 검증**:
  ```bash
  ls -la /sys/bus/usb/devices/2-2:1.5/driver
  # 정상 출력: .../driver -> ../../../../../../../bus/usb/drivers/usbhid
  ```
- [ ] **RealSense 장치 정보 및 펌웨어 검증**:
  ```bash
  /usr/local/bin/rs-enumerate-devices | grep -E "Name|Firmware|IMU"
  # 정상 출력: 'Intel RealSense D435i' 제품명과 펌웨어 버전, 'IMU' 지원 스트림이 정상 감지되어야 함
  ```

---

## 4단계: 실내/실외 맵핑 및 주행 스크립트 가동 (SLAM 실행)
상황에 알맞은 가동 스크립트를 백그라운드 터미널에 띄웁니다.

- [ ] **실내 맵핑 실행** (VIO 기반 3원 융합 SLAM):
  ```bash
  cd /home/unitree/go2_ws
  ./run_map_indoor.sh
  ```
- [ ] **실내 자율주행 실행** (기존 맵 기반 자율주행):
  ```bash
  cd /home/unitree/go2_ws
  ./run_localization_indoor.sh
  ```
  * *주의*: 리얼센스 리셋 및 USB 안정화를 위한 **25초 대기** 후 모든 터미널 탭이 활성화되는지 확인합니다.

---

## 5단계: ROS 2 토픽 및 주파수 점검 (통신 무결성)
구동 스크립트가 뜬 뒤, 데이터 스트림이 누락 없이 정상적인 속도로 들어오는지 **한 번에 확인**합니다.

- [ ] **전체 토픽 활성화 여부 확인**:
  ```bash
  ros2 topic list
  # 아래 토픽들이 목록에 모두 나타나는지 확인
  ```
- [ ] **토픽별 발행 주파수(Hz) 전수 검사**:
  * 아래 명령어를 새 터미널에 순서대로 입력하여 데이터 수신 주기(Hz)가 정상인지 점검합니다.
  ```bash
  # 1. 카메라 IMU (정상 기준: 200 Hz 내외)
  ros2 topic hz /camera/imu
  
  # 2. Madgwick 필터링된 IMU (정상 기준: 200 Hz 내외)
  ros2 topic hz /imu/data
  
  # 3. 리얼센스 RGB 이미지 (정상 기준: 15 ~ 30 Hz)
  ros2 topic hz /camera/color/image_raw
  
  # 4. 리얼센스 Depth 이미지 (정상 기준: 15 ~ 30 Hz)
  ros2 topic hz /camera/aligned_depth_to_color/image_raw
  
  # 5. 로봇 다리/라이다 오도메트리 (정상 기준: 20 ~ 50 Hz)
  ros2 topic hz /odom
  
  # 6. VIO 오도메트리 (정상 기준: 15 ~ 30 Hz) - 실내 모드 가동 시
  ros2 topic hz /visual_odom
  
  # 7. 레이저 스캔 데이터 (정상 기준: 10 ~ 15 Hz) - 주행 모드 가동 시
  ros2 topic hz /scan
  ```

---

## 6단계: TF 좌표계 및 데이터 동기화 점검 (SLAM/자율주행 무결성)
좌표계 꼬임 및 토픽 충돌 여부를 모니터링하여 SLAM이 완벽히 가동 중인지 검증합니다.

- [ ] **TF 트리 모니터링 및 중복 부모 검증**:
  * TF 트리에 경고나 끊김이 없는지 확인합니다.
  ```bash
  ros2 run tf2_ros tf2_monitor
  ```
  * *체크*: `odom` ➔ `base_link` 좌표 변환을 퍼블리시하는 노드가 오직 **하나(go2_driver)** 인지 확인합니다. (rtabmap VIO가 `visual_odom`으로 분리되어 TF 충돌 경고가 나타나지 않아야 함)
- [ ] **정적 좌표계(Static TF) 발행 확인**:
  ```bash
  ros2 run tf2_ros tf2_echo base_link camera_link
  # 정상 출력: base_link와 camera_link 간의 거리/각도 오프셋이 정상적으로 수신되어 출력되어야 함
  ```
- [ ] **자율주행 제어 토픽(`/cmd_vel`) 송수신 점검**:
  * Nav2 혹은 원격 조종 패키지가 조향 속도를 정상 전달하는지 점검합니다.
  ```bash
  ros2 topic echo /cmd_vel
  # 노트북 혹은 조종기로 제어 시 선속도(linear) 및 각속도(angular) 값이 실시간으로 찍히는지 확인
  ```

---

## 🚨 트러블슈팅 즉시 대응 가이드

* **Q1. `/camera/imu` 및 `/imu/data`가 0 Hz로 멈춰있는 경우**
  * 스크립트 실행 콘솔을 열고 `LD_PRELOAD=/usr/local/lib/librealsense2.so` 환경 변수가 제대로 입력되었는지 다시 실행해 봅니다.
* **Q2. `realsense2_camera`가 전원 문제로 실행 즉시 죽는 경우**
  * `3단계`에 명시된 USB 자동 절전 모드 설정 명령어를 통해 autosuspend가 `-1`이고 control이 `on`인지 확인하고, 그래도 안 되면 다른 물리 USB 포트로 변경해 봅니다.
* **Q3. 로봇과 노트북 간에 통신이 안 되거나 토픽 목록이 안 보일 때**
  * 양단의 `cyclonedds.xml` 파일 내에 서로의 유니캐스트 IP 피어가 정확하게 하드코딩 매핑되어 있는지 주소를 대조해 봅니다.
