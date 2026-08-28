# ⚡ Unitree Go2 젯슨(Jetson) 호스트 OS 마스터 플랜 및 운영 런북

> **현재 실행 기준**: 아래 과거 완료 표보다 [`docs/README.md`](../README.md)와 [`현재 상태와 RTAB-Map 순서`](../00_CURRENT_STATUS_AND_NEXT_STEPS.md)를 우선한다. 현재 mapping 진입점은 `run_map.sh`와 `map_headless.sh`뿐이며 planar 3DoF/global-loop 자격 검증 진행 중이다. physical autonomy는 NO-GO다.

> **폴더 목적**: Unitree Go2 온보드 **Jetson Orin NX (Ubuntu 20.04 LTS / ROS 2 Foxy / CUDA 11.4)** 호스트 환경에서 하드웨어 센서, DDS 네트워크, LIVO SLAM 매핑, 도커 브릿지 및 모터 제어를 총괄 운영하기 위한 전용 런북 허브입니다.

---

## 📂 젯슨 호스트 전용 런북 및 검증 문서 체계

| 문서 번호 | 런북 문서명 | 주요 세부 내용 및 링크 | 상태 |
| :---: | :--- | :--- | :---: |
| **01** | **`01_jetson_hardware_network_and_dds_architecture.md`** | • 젯슨 4계층 아키텍처 및 IP 토폴로지 (`192.168.123.99`)<br/>• CycloneDDS `cyclonedds.xml` 네트워크 인터페이스 바인딩<br/>👉 **[01_문서 보기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/01_jetson_hardware_network_and_dds_architecture.md)** | 🟢 **완료 (PASS)** |
| **02** | **`02_jetson_rtabmap_livo_pipeline_and_bringup.md`** | • Go2 `/livo/*` 입력과 planar RTAB-Map mapping<br/>• canonical GUI/headless 두 진입점<br/>👉 [02 문서](02_jetson_rtabmap_livo_pipeline_and_bringup.md) | 🟡 **Mapping qualification** |
| **03** | **`03_jetson_host_docker_bridge_and_motor_actuation.md`** | • 젯슨 ↔ 도커 간 50Hz 초저지연 UDP 루프백 통신 (<0.2ms)<br/>• Unitree SportClient 모터 토크 제어 파이프라인<br/>👉 **[03_문서 보기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/03_jetson_host_docker_bridge_and_motor_actuation.md)** | 🟢 **완료 (PASS)** |
| **04** | **`04_jetson_onboard_benchmark_and_logging_runbook.md`** | • 삭제된 legacy 4-terminal/자동 채점 절차의 역사 기록<br/>👉 [04 문서](04_jetson_onboard_benchmark_and_logging_runbook.md) | 🔴 **Archived / 실행 금지** |
| **05** | **`05_jetson_headless_boot_and_autologin_guide.md`** | • 젯슨 헤드리스 자동 로그인 & NetBird/SSH 부팅<br/>• GDM3 자동 로그인 및 모니터 프리 접속 셋업<br/>• 온보드 하드웨어 발열 및 nvv4l2decoder 점검<br/>👉 **[05_문서 보기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/05_jetson_headless_boot_and_autologin_guide.md)** | 🟢 **완료 (PASS)** |
| **06** | **`06_jetson_obstacles_avoid_api_and_stall_detector_guide.md`** | • **[NEW] Unitree SDK2 공식 장애물 회피 API (`ObstaclesAvoidClient`)**<br/>• `SportClient::FreeAvoid` & `SwitchAvoidMode` 명세<br/>• 호스트 정체 감지기(Kinematic Stall Detector) & UDP 63B 규격<br/>👉 **[06_문서 보기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/06_jetson_obstacles_avoid_api_and_stall_detector_guide.md)** | 🟢 **완료 (PASS)** |
| **07** | **`07_jetson_rtabmap_2d_clean_map_parameter_tuning_directive.md`** | • 삭제된 screen-recording mapping wrapper의 역사적 튜닝 기록<br/>👉 [07 문서](07_jetson_rtabmap_2d_clean_map_parameter_tuning_directive.md) | ⚪ **Archived** |
| **⭐ Control** | **`control/README.md` (신설)** | • **[NEW] 로봇 제어 및 모터 실측 검증 전용 런북**<br/>• Sport API ID (Move 1008, Damp 1001, StandUp 1002), 0.5초 워치독 안전 정지<br/>• 속도 리미터($0.35\text{ m/s}$), $\pm 30\text{cm}$ 미세 보행 실측 SOP<br/>👉 **[Control 폴더 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/control/README.md)** | 🟢 **신설 (New)** |

---

## 🔗 상위 연계 문서 바로가기
* **마스터 플랜 중앙 총평 허브**: [`docs/master_plan/README.md`](../master_plan/README.md)
* **도커 샌드박스 전용 런북 허브**: [`docs/docker_plan/README.md`](../docker_plan/README.md)
* **실시간 온보드 종합 진단표**: [`docs/14_real_robot_live_system_diagnostic_report.md`](../14_real_robot_live_system_diagnostic_report.md)
