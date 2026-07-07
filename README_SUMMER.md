# 🚀 Unitree Go2 SLAM & Navigation Execution Guide (Branch: `summer`)

본 가이드는 `summer` 브랜치에서 분할/정비된 **실내 및 실외 전용 자율주행 실행 스크립트**의 구동 절차를 설명합니다. 원본 코드는 `master` 브랜치에 그대로 보존되어 있으며, 모든 개선 및 디버깅 사항은 `summer` 브랜치에만 반영되었습니다.

---

## 1. 스크립트 분류 체계 (Script Structure)

사용하기 복잡했던 설정 스위치를 배제하고, 목적에 맞게 즉시 실행할 수 있도록 **실내용(Indoor)**과 **실외용(Outdoor)** 스크립트 세트로 완전히 분할했습니다.

### 🏠 1. 실내용 (Indoor Set) - VIO + Leg + IMU 3원 융합
* **특징**: 실내에서는 태양광 노이즈가 없으므로 리얼센서 카메라를 활용한 정밀한 **비주얼 오도메트리(VIO)**를 주축으로 삼습니다. 단, 카메라 유실을 방지하기 위해 다리 엔코더 값(`/odom`)을 예측값(Guess)으로 넣고 D435i IMU 데이터로 흔들림을 필터링합니다. (OMO R1 방식 완벽 재현)
* **매핑**:
  ```bash
  ./run_map_indoor.sh
  ```
* **자율주행 (Nav2)**:
  ```bash
  ./run_localization_indoor.sh
  ```

### 🌲 2. 실외용 (Outdoor Set) - LiDAR-Inertial Odometry
* **특징**: 야외 직사광선 조건에서는 삼각측량식 D435i 카메라가 무력화(IR 워시아웃 및 롤링 셔터 왜곡)되므로, 카메라 오도메트리를 강제로 차단(`visual_odometry:=false`)하고 로봇 본체의 고강인성 **4D LiDAR L1 및 온보드 IMU 오도메트리**(`/odom`)를 기반으로 매핑과 주행을 실행합니다.
* **매핑**:
  ```bash
  ./run_map_outdoor.sh
  ```
* **자율주행 (Nav2)**:
  ```bash
  ./run_localization_outdoor.sh
  ```

---

## 2. ROS 2 `/cmd_vel` ➔ Sport API 제어 가교 (`go2_driver`)

자율주행 패키지(Nav2)가 경로 계획의 결과로 조향 속도 명령 토픽인 **`/cmd_vel`**(Twist 메시지)을 퍼블리시하면, 로봇 본체의 관절 모터를 제어해 주는 Unitree의 **Sport API**로의 변환이 필요합니다.

* **동작 확인**: 사용자가 추가적인 패키지를 다운로드하거나 연동할 필요 없이, 클로닝된 워크스페이스 내의 **`go2_driver`** 패키지가 이 가교 역할을 완전히 수행합니다.
* **연동 소스**: [go2_driver.cpp](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/src/go2_robot/go2_driver/src/go2_driver/go2_driver.cpp) 노드가 기동되어 `/cmd_vel` 속도값을 실시간으로 수신한 뒤, Unitree Sport API 포맷에 맞춰 JSON 문자열로 직렬화하여 본체 보드로 전송합니다.
  ```cpp
  nlohmann::json js;
  js["x"] = msg->linear.x; // 직진 속도
  js["y"] = msg->linear.y; // 횡방향 속도
  js["z"] = msg->angular.z; // 회전 속도
  ```
* **구동**: `run_localization_indoor.sh` 또는 `run_localization_outdoor.sh` 실행 시 백그라운드 첫 번째 탭에서 `ros2 launch go2_bringup go2.launch.py`가 실행되면서 이 가교 노드가 자동 기동되므로 사용자는 단순히 스크립트를 한 번 구동하는 것만으로 자율주행 조향이 가능합니다.

---

## 3. 트러블슈팅 가이드 (Troubleshooting)

### ⚠️ CycloneDDS 통신 불통 문제
* **증상**: 다른 PC나 노트북에서 로봇의 ROS 토픽이 조회되지 않거나 스크립트 실행 시 노드 검색이 안 되는 경우.
* **원인**: `cyclonedds.xml` 파일 내 네트워크 카드 이름이 `eth0`로 고정되어 있기 때문입니다.
* **해결**: 본인의 실제 네트워크 디바이스명(예: `wlan0`, `enp3s0`)에 맞춰 수정한 뒤 실행하거나, 환경변수 설정을 해제하십시오.
  ```bash
  # cyclonedds.xml 설정 일시 해제
  unset CYCLONEDDS_URI
  ```
