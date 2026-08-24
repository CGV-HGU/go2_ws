# 🛠️ [HUB] Unitree Go2 ESCAPE-Nav 트러블슈팅 & 기술 자산(Know-How) 허브

> **문서 목적**: 실물 4족 보행 로봇 Unitree Go2와 원격 비동기 VLM(Qwen3-VL) 자율주행 통합 시스템에서 발생하는 **모든 에러 로그, 근본 원인 분석, 10초 초고속 해결 쿡북, 및 서브시스템별 딥다이브 기술 자산(Know-How)**을 총괄하는 중앙 인덱스입니다.

---

## 📑 1. 핵심 트러블슈팅 문서 바로가기

| 문서 명칭 | 주요 내용 | 링크 |
| :--- | :--- | :---: |
| **⚡ 초고속 에러 해결 쿡북** | 터미널 에러 문자열별 10초 해결 1줄 명령어 색인집 | [**COOKBOOK**](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/QUICK_ERROR_LOOKUP_AND_REMEDY_COOKBOOK.md) |
| **📑 전수 에러 마스터 로그북** | 마일스톤별 11대 전수 에러, 원본 로그, Diff, 검증 데이터 | [**MASTER LOG**](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/ERROR_AND_RESOLUTION_MASTER_LOG.md) |

---

## 🔬 2. 서브시스템별 5대 딥다이브 기술 가이드 (Deep-Dive Know-How)

1. [**🐕 01. 4D 라이다 & 하드웨어 트러블슈팅**](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/01_lidar_and_hardware_troubleshooting_knowhow.md): 0Hz 스트리밍 차단 해제, IP 에일리어스(`192.168.1.2`), UDP 6201 포트 충돌, BMS 배터리 보호.
2. [**📷 02. 전면 카메라 & GStreamer 스트리밍**](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/02_camera_and_gstreamer_knowhow.md): RTP 멀티캐스트 `230.1.1.1:1720`, OpenCV `dlopen` glibc 링커, 30fps 스레드 동기화.
3. [**🗺️ 03. RTAB-Map LIVO SLAM 튜닝**](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/03_rtabmap_livo_slam_tuning_knowhow.md): Mathieu Labbé 교수 공식 포럼 파라미터, ICP 3DoF 2D 평면 구속, 2D 점유격자 가시 번짐 억제.
4. [**🐳 04. Docker 브릿지 & VLM 연동**](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/04_docker_jazzy_foxy_bridge_knowhow.md): Foxy-Jazzy 간 54B/62B UDP 소켓, Causal Pose Warping 수식, `network_mode: host`.
5. [**🕹️ 05. Unitree SDK2 Sport API & 모터 제어**](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/05_unitree_sport_api_and_motor_knowhow.md): High-Level Move(1008) 채택 이유, `/cmd_vel` + Sport API 이중 계층(Dual-Layer), 0.5s 워치독.
