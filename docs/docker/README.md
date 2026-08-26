# 🐳 Unitree Go2 ESCAPE-Nav Docker Documentation Index

이 폴더는 Unitree Go2 로봇의 온보드 도커 샌드박스(`sdam_go2_container`, Ubuntu 24.04 / ROS 2 Jazzy) 관련 아키텍처, 셋업 가이드 및 자율주행 배포 마스터 플랜 문서를 관리합니다.

---

## 📚 문서 목록

* **[`00_live_progress_and_system_status_dashboard.md`](00_live_progress_and_system_status_dashboard.md)**:
  - 도커 9대 핵심 서브시스템 실시간 상시 점검표 및 3초 원터치 자동 진단 스크립트 가이드
* **[`01_docker_autonomy_deployment_master_plan.md`](01_docker_autonomy_deployment_master_plan.md)**: 
  - 도커 컨테이너 스펙, ROS 2 Jazzy 패키지 구조, 원격 VLM 서버(`cgv-server-02`) 연동, 4단계 실전 실행 계획 및 치트시트 총괄
* **[`02_camera_selection_and_server_trajectory_architecture.md`](02_camera_selection_and_server_trajectory_architecture.md)**:
  - 내장 초광각(어안) vs RealSense D435i 8대 기술 비교 및 원격 VLM 서버 실시간 50Hz 궤적 추출 아키텍처 가이드
* **[`visualizations/README.md`](visualizations/README.md)**:
  - 실물 로봇 1인칭 카메라 시야(FPV), 실제 83.3m 라이다 SLAM 지도 궤적, 4방향 시각 메모리, 장애물 충돌 및 360도 능동 회복 등 **도커 자율주행 4대 분야별 실물 시각화 갤러리** (고해상도 PNG 수록)
