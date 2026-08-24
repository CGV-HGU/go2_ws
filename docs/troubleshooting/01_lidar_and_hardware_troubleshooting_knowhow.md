# 🐕 [Know-How 01] Unitree Go2 4D 라이다 & 하드웨어 트러블슈팅 정밀 해설서

> **대상 시스템**: Unitree Go2 순정 내장 4D L1/L2 라이다, 메인보드 MCU (`192.168.123.161`), CycloneDDS  
> **문서 목적**: 순정 4D 라이다 스트리밍 차단 해제, IP 에일리어스, UDP 6201 포트 충돌, BMS 배터리 저전압 보호 메커니즘 전수 해설

---

## 1. 🔍 Go2 내장 4D 라이다의 통신 아키텍처 및 0Hz 차단 원리

1. **내장 DSP 오도메트리 전용 소비**:
   - Go2 메인보드는 기본 부팅 시, 4D 라이다의 원시 점군을 메인보드 내부 1000Hz 밸런싱 및 50Hz 오도메트리(`/odom`) 계산에만 소비합니다.
   - 외부 이더넷(`eth0`)으로 원시 점군(Point Cloud)을 송출하는 기능은 대역폭 및 CPU 부하 절감을 위해 기본 잠금(Mute) 상태로 설정되어 있습니다.
2. **원시 점군 활성화 방법**:
   - **방법 1 (모바일 앱)**: Unitree Go2 스마트폰 앱 ➔ Settings ➔ `Perception LiDAR / Raw Point Cloud Stream` 토글 활성화.
   - **방법 2 (공식 Service API)**: `unitree_ros2`를 통해 `/api/robot_state/request`에 활성화 커맨드 송신.
3. **네트워크 바인딩 규격**:
   - 라이다 IP: `192.168.1.62:6101` (송신)
   - Jetson 인터페이스: `eth0`에 에일리어스 `192.168.1.2/24` 필수 할당.
   - Jetson 수신 포트: UDP `6201` (`unitree_lidar_ros2_node`).

---

## 2. ⚡ 라이다 UDP 6201 포트 충돌(`bind udp port failed`) 해결법

* **원인**: 노드가 비정상 종료(Ctrl+Z 등)되거나 백그라운드에 남아 있을 때 소켓이 잠겨 신규 노드가 바인딩되지 못함.
* **1줄 즉각 해결 명령어**:
  ```bash
  echo admin | sudo -S fuser -k 6201/udp
  ```

---

## 3. 🔋 BMS 배터리 저전압 슬립 및 모터 잠금 보호 대책

* Go2 배터리가 20% 이하로 떨어지면 DSP가 모터 토크를 자동으로 컷오프(Damp 상태)하여 낙상할 수 있습니다.
* 실증 실험 전 반드시 배터리 잔량이 50% 이상인지 확인하십시오.
  ```bash
  # 배터리 잔량 및 LowState BMS 전압 확인
  ros2 topic echo /lowstate --field-match battery
  ```
