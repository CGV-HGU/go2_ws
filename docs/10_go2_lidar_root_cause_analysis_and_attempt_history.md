# 🔍 Unitree Go2 내장 라이다(L1) 동작 원인 분석 및 전수 시도 이력 보고서

> **문서 버전**: v1.0 (2026-08-19)  
> **대상 장비**: Unitree Go2 EDU Plus (Jetson Orin NX 16GB)  
> **보고 목적**: 라이다 토픽(`/utlidar/cloud`, `/pointcloud`)이 수신되지 않는 기술적 원인 분석 및 지금까지 수행한 모든 하드웨어/소프트웨어 시도 내역 정리

---

## 📜 1. 과거 실행 이력 및 히스토리 팩트체크

시스템 내부의 과거 스크립트(`unitree-ros2/run_map.sh`, `run_go2_mapping.sh`)와 이미지(`Pictures/Go2_imu+lidar+rgb.png`)를 전수 조사한 결과:

1. **과거 3D 점군 및 IMU 맵핑 구성**:
   * 이전에 RViz2로 3D 점군과 IMU를 성공적으로 시각화했던 구성은 **RealSense D435i 3D 뎁스 카메라(`realsense2_camera`) + Madgwick IMU 필터(`imu_filter_madgwick`) + RTAB-Map** 조합이었습니다.
2. **Go2 내장 L1 라이다의 역할**:
   * 로봇 스크립트(`go2_ws_new/run_map.sh:L13-16`)에 명시된 대로, Go2 내장 L1 라이다는 메인보드가 내부에서 **라이다+IMU 융합 3D 오도메트리(`/odom`)**를 연산하는 데 사용되었습니다.

---

## 🛠️ 2. 지금까지 수행한 7대 정밀 시도 내역 (All Attempts)

| 번호 | 시도 항목 | 실행 명령어 / 조치 내용 | 실측 결과 | 분석 |
| :---: | :--- | :--- | :---: | :--- |
| **1** | **C++ Hesai 드라이버 가동** | `ros2 launch go2_bringup go2.launch.py lidar:=True` | ⚪ 0 Hz | 외장 Hesai 라이다 IP(`192.168.1.201`) 핑 100% 손실 |
| **2** | **네이티브 DDS 점군 수신** | `ros2 topic hz /utlidar/cloud` / `/pointcloud` | ⚪ 0 Hz | 토픽은 등록되나 본체에서 데이터 패킷 미송출 |
| **3** | **QoS 정책 변경 (Best Effort)** | `ReliabilityPolicy.BEST_EFFORT`로 파이썬 구독 | ⚪ 0 패킷 | QoS 불일치 문제가 아닌 소스 패킷 부재 확인 |
| **4** | **CycloneDDS 피어 강제 등록** | `cyclonedds.xml`에 `<Peer address="192.168.123.161"/>` 등록 | 🟢 카메라 14.3Hz / ⚪ 라이다 0Hz | 카메라 및 통신망은 살아있으나 라이다 점군만 차단 |
| **5** | **유니트리 SDK2 서비스 스위치** | `/home/unitree/unitree_sdk2/build/bin/go2_robot_state_client eth0 lidar` | ⚠️ Error 3104 (Timeout) | 메인보드 DDS 서비스 인터페이스 타임아웃 |
| **6** | **DDS 문자열 ON 스위치 퍼블리시** | `ros2 topic pub --once rt/utlidar/switch std_msgs/msg/String "{data: 'ON'}"` | 🟢 전송 완료 / ⚪ 패킷 미출력 | 스위치 토픽 발행 성공했으나 펌웨어 락 미해제 |
| **7** | **네트워크 포트 & 패킷 캡처** | `tcpdump -i eth0`, `nc -zvw 1 192.168.123.161 [포트]` | 🟢 80번(HTTP) 열림, 22번(SSH) 닫힘 | 보안상 SSH 차단, 메인보드 단독 제어 확인 |

---

## 🔍 3. 근본 기술적 원인 분석 (Root Cause)

1. **로봇 본체 펌웨어의 라이다 스트리밍 차단**:
   * Unitree Go2 순정 펌웨어는 배터리 소모와 내부 이더넷 버스 과부하를 방지하기 위해, **기본 부팅 시 라이다 점군을 메인보드(`192.168.123.161`) 내부에서만 처리하고 외부 이더넷(`eth0`)으로는 점군을 내보내지 않도록(Mute) 잠겨 있습니다.**
2. **젯슨 단독 활성화 한계**:
   * 로봇 메인보드의 22번 SSH 포트가 보안상 닫혀 있으므로, 젯슨 OS 내부 명령어만으로는 로봇 메인보드 내부의 라이다 전원/송출 데몬을 강제로 재시작할 수 없습니다.
3. **공식 활성화 경로**:
   * Unitree Go2 공식 매뉴얼에 따르면, **스마트폰 Unitree Go2 앱 ➔ [Device] ➔ [Data] ➔ [Unitree Perception LiDAR]** 메뉴에서 스트리밍 스위치를 켜주어야 메인보드가 `eth0`로 점군을 송출하기 시작합니다.

---

## 📋 4. 사용자 검증 및 재가동 프로토콜

1. **스마트폰 앱 확인**:
   * Unitree 앱에 연결하여 라이다 데이터 스트리밍 스위치 상태 확인
2. **라이다 토픽 수신 즉시 테스트**:
   ```bash
   source /opt/ros/foxy/setup.bash
   source /home/unitree/cyclonedds_ws/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI="file:///home/unitree/go2_ws_antarctica/cyclonedds.xml"
   export ROS_DOMAIN_ID=0

   ros2 topic hz /utlidar/cloud
   ```
