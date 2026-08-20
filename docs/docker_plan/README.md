# 🐳 Unitree Go2 ESCAPE-Nav 도커(Docker) 자율주행 런북 허브 (`docs/docker_plan/`)

> **폴더 목적**: `sdam_go2_container` (Ubuntu 24.04 LTS / ROS 2 Jazzy ARM64) 환경에서 구동되는 **S2E 비동기 VLM 자율주행 정책 노드, 알고리즘 단위 테스트, UDP 소켓 브릿지 및 도커 5대 핵심 검증 런북을 총괄 관리하는 중앙 허브**입니다.

---

## 📂 도커 런북 문서 목록

| 문서 번호 | 런북 문서명 | 주요 내용 및 링크 | 상태 |
| :---: | :--- | :--- | :---: |
| **01** | **`01_docker_autonomy_deployment_master_plan.md`** | • 도커 격리 샌드박스 설계 배경 및 아키텍처<br/>• S2E 비동기 50Hz 궤적 생성기 구조<br/>• 원격 Qwen VLM 클라이언트 연동<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/docker_plan/01_docker_autonomy_deployment_master_plan.md)** | 🟢 **정식 등록** |
| **02** | **`02_docker_comprehensive_verification_checklist.md`** | • 도커 5대 핵심 점검 매트릭스 (S2E 단위 테스트, 50Hz 스트레스, 720p VLM, 풀 드라이런, 공유메모리)<br/>• 현장 1줄 실행 명령어 수록<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/docker_plan/02_docker_comprehensive_verification_checklist.md)** | 🟢 **최신 (Latest)** |

---

## 🔗 상위 및 하위 연계 런북 바로가기

1. **마스터 플랜 및 총평 허브**: [`docs/master_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/README.md)
2. **호스트 OS 전용 런북**: [`docs/jetson_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/README.md)
3. **실시간 온보드 종합 진단표**: [`docs/14_real_robot_live_system_diagnostic_report.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/14_real_robot_live_system_diagnostic_report.md)
