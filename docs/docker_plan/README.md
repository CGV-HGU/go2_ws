# 🐳 Unitree Go2 도커(Docker) 샌드박스 자율주행 마스터 플랜 및 운영 런북

> **폴더 목적**: Host OS(Ubuntu 20.04 / Foxy)와 격리된 **도커 컨테이너(`sdam_go2_container`, Ubuntu 24.04 LTS / ROS 2 Jazzy ARM64 / Python 3.12)** 내부에서 비동기 VLM 추론 및 S2E 50Hz 고속 궤적 제어기를 안전하고 무결하게 배포/운영/검증하기 위한 전용 런북 허브입니다.

---

## 📂 도커 전용 런북 및 검증 문서 체계

| 문서 번호 | 런북 문서명 | 주요 세부 내용 및 링크 | 상태 |
| :---: | :--- | :--- | :---: |
| **01** | **`01_docker_autonomy_deployment_master_plan.md`** | • 도커 격리 샌드박스 아키텍처 및 4대 패키지 구조<br/>• Zero-Copy UDP 루프백 통신 및 NetBird VPN<br/>• 3단계 실증 배포 로드맵 및 비상 정지 매뉴얼<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/docker_plan/01_docker_autonomy_deployment_master_plan.md)** | 🟢 **완료 (PASS)** |
| **02** | **`02_docker_comprehensive_verification_checklist.md`** | • 5대 전 영역 실측 종합 검증 체크리스트<br/>• Pytest 59개 전수 통과 및 50Hz UDP 0% Loss<br/>• 720p 실시간 VLM 추론 및 S2E 풀 드라이런<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/docker_plan/02_docker_comprehensive_verification_checklist.md)** | 🟢 **완료 (PASS)** |
| **03** | **`03_docker_practical_testing_and_verification_suite.md`** | • **[NEW] 도커 6대 실용 테스트 스위트 매뉴얼**<br/>• `bash scratch/run_all_docker_tests.sh` 1-Click 실행법<br/>• 6대 서브시스템 세부 검증 기준 및 긴급 트러블슈팅<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/docker_plan/03_docker_practical_testing_and_verification_suite.md)** | 🟢 **최신 (Latest)** |

---

## 🔗 상위 연계 문서 바로가기
* **마스터 플랜 중앙 총평 허브**: [`docs/master_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/README.md)
* **호스트 OS 전용 런북 허브**: [`docs/jetson_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/README.md)
* **실시간 온보드 종합 진단표**: [`docs/14_real_robot_live_system_diagnostic_report.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/14_real_robot_live_system_diagnostic_report.md)
