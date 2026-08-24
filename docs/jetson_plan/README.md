# ⚡ Unitree Go2 젯슨(Jetson) 호스트 OS 마스터 플랜 및 운영 런북

> **폴더 목적**: Unitree Go2 온보드 **Jetson Orin NX (Ubuntu 20.04 LTS / ROS 2 Foxy / CUDA 11.4)** 호스트 환경에서 하드웨어 센서, DDS 네트워크, LIVO SLAM 매핑, 도커 브릿지 및 모터 제어를 총괄 운영하기 위한 전용 런북 허브입니다.

---

## 📂 젯슨 호스트 전용 런북 및 검증 문서 체계

| 문서 번호 | 런북 문서명 | 주요 세부 내용 및 링크 | 상태 |
| :---: | :--- | :--- | :---: |
| **01** | **`01_jetson_hardware_network_and_dds_architecture.md`** | • 젯슨 4계층 아키텍처 및 IP 토폴로지 (`192.168.123.99`)<br/>• CycloneDDS `cyclonedds.xml` 네트워크 인터페이스 바인딩 | 🟢 **완료 (PASS)** |
| **02** | **`02_jetson_rtabmap_livo_pipeline_and_bringup.md`** | • RTAB-Map 50Hz LIVO Odometry 및 3D 점군 생성<br/>• 라이다 0Hz 이슈 해결 및 1-Click 실행법 | 🟢 **완료 (PASS)** |
| **03** | **`03_jetson_host_docker_bridge_and_motor_actuation.md`** | • 젯슨 ↔ 도커 간 50Hz 초저지연 UDP 루프백 통신 (<0.2ms)<br/>• Unitree SportClient 모터 토크 제어 파이프라인 | 🟢 **완료 (PASS)** |
| **04** | **`04_jetson_onboard_benchmark_and_logging_runbook.md`** | • 실기체 주행 로깅 및 ICRA Table 지표 계산 엔진<br/>• `calculate_icra_metrics.py` 및 자동 Rosbag 기록 | 🟢 **완료 (PASS)** |
| **05** | **`05_jetson_headless_boot_and_autologin_guide.md`** | • 젯슨 헤드리스 자동 로그인 & NetBird/SSH 부팅<br/>• GDM3 자동 로그인 및 모니터 프리 접속 셋업<br/>• 온보드 하드웨어 발열 및 nvv4l2decoder 점검 | 🟢 **완료 (PASS)** |
| **06** | **`06_jetson_obstacles_avoid_api_and_stall_detector_guide.md`** | • **[NEW] Unitree SDK2 공식 장애물 회피 API (`ObstaclesAvoidClient`)**<br/>• `SportClient::FreeAvoid` & `SwitchAvoidMode` 명세<br/>• 호스트 정체 감지기(Kinematic Stall Detector) & UDP 63B 규격 | 🟢 **최신 (Latest)** |

---

## 🔗 상위 연계 문서 바로가기
* **마스터 플랜 중앙 총평 허브**: [`docs/master_plan/README.md`](../master_plan/README.md)
* **도커 샌드박스 전용 런북 허브**: [`docs/docker_plan/README.md`](../docker_plan/README.md)
* **실시간 온보드 종합 진단표**: [`docs/14_real_robot_live_system_diagnostic_report.md`](../14_real_robot_live_system_diagnostic_report.md)
