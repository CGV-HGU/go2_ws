# 🏆 Unitree Go2 ESCAPE-Nav 마스터 플랜 및 총평 허브 (`docs/master_plan/`)

> **폴더 목적**: `docs/jetson_plan/`(호스트 런북)과 `docs/docker_plan/`(도커 런북)의 성과를 집대성하여, **실시간 6대 시스템 진단 결과, 아키텍처 팩트체크, 장애물 회피 API/충돌 감지 설계서, PointNav 실로봇 5-Set 맵 계획 및 Habitat-GS/노이즈/Ablation 종합 명세서를 날짜별로 체계적으로 관리하는 중앙 총평 허브**입니다.

---

## 📂 마스터 플랜 및 현장 운영 런북 목록 (5대 마스터 문서 체계)

| 작성 일자 | 마스터 플랜 문서명 | 주요 내용 및 링크 | 상태 |
| :--- | :--- | :--- | :---: |
| **2026-08-21** | **`[2026-08-21]_PointNav_실로봇_5set_맵계획_및_HabitatGS_노이즈_Ablation_종합명세서.md`** | • 실로봇 5-Set 단일 맵 다중 좌표 운용 프로토콜<br/>• Habitat-GS ↔ S2E Async 연동 방안<br/>• GPS 센서 노이즈 vs Pose 드리프트 분리<br/>• VLM 모델 베리에이션 매트릭스 및 Ablation 인수인계<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-21%5D_PointNav_%EC%8B%A4%EB%A1%9C%EB%B4%87_5set_%EB%A7%B5%EA%B3%84%ED%9A%8D_%EB%B0%8F_HabitatGS_%EB%85%B8%EC%9D%B4%EC%A6%88_Ablation_%EC%A2%85%ED%95%A9%EB%AA%85%EC%84%B8%EC%84%9C.md)** | 🟢 **최신 (Latest)** |
| **2026-08-21** | **`[2026-08-21]_Go2_장애물_회피_API_및_충돌_감지_파이프라인_설계서.md`** | • Unitree SDK2 `ObstaclesAvoidClient` API 분석<br/>• 운동학적 정체 감지(Kinematic Stall Detector)<br/>• `/robot/collision_detected` 토픽 및 S2E 연동<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-21%5D_Go2_%EC%9E%A5%EC%95%A0%EB%AC%BC_%ED%9A%8C%ED%94%BC_API_%EB%B0%8F_%EC%B6%A9%EB%8F%8C_%EA%B0%90%EC%A7%80_%ED%8C%8C%EC%9D%B4%ED%94%84%EB%9D%BC%EC%9D%B8_%EC%84%A4%EA%B3%84%EC%84%9C.md)** | 🟢 **정식 등록** |
| **2026-08-20** | **`[2026-08-20]_ESCAPE-Nav_Jetson_현장_실증_실행_로드맵_및_운영_매뉴얼.md`** | • 4대 계층 1-Click 마스터 브링업 구조<br/>• 현장 즉시 실행 4단계 실증 주행 절차 (Step 0~4)<br/>• 비상 정지(E-Stop) 및 안전 종료 수칙 수록<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-20%5D_ESCAPE-Nav_Jetson_%ED%98%84%EC%9E%A5_%EC%8B%A4%EC%A6%9D_%EC%8B%A4%ED%96%89_%EB%A1%9C%EB%93%9C%EB%A7%B5_%EB%B0%8F_%EC%9A%B4%EC%98%81_%EB%A7%A4%EB%89%B4%EC%96%BC.md)** | 🟢 **정식 등록** |
| **2026-08-20** | **`[2026-08-20]_ESCAPE-Nav_마스터플랜_팩트체크_및_고성능_아키텍처_개선보고서.md`** | • 마스터 플랜 팩트체크 및 6대 시스템 타당성 검증<br/>• Iceoryx2 Zero-Copy & cuPCL 오프체인 최적화<br/>• SGLang RadixAttention & TensorRT-LLM FP8<br/>• $\Delta T \approx 124\text{ms}$ 정밀 수식 및 3단계 로드맵<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-20%5D_ESCAPE-Nav_%EB%A7%88%EC%8A%A4%ED%84%B0%ED%94%8C%EB%9E%9C_%ED%8C%A9%ED%8A%B8%EC%B2%B4%ED%81%AC_%EB%B0%8F_%EA%B3%A0%EC%84%B1%EB%8A%A5_%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98_%EA%B0%9C%EC%84%A0%EB%B3%B4%EA%B3%A0%EC%84%9C.md)** | 🟢 **정식 등록** |
| **2026-08-20** | **`[2026-08-20]_ESCAPE-Nav_실물_로봇_Jetson_및_Docker_통합_총평_및_마스터_플랜.md`** | • 6대 전 영역 실측 진단 총평 (ALL PASS)<br/>• Jetson ↔ Docker 2대 축 통합 아키텍처<br/>• ICRA Table VIII 20회 주행 프로토콜<br/>• 1-Click 실행 매뉴얼 수록<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-20%5D_ESCAPE-Nav_%EC%8B%A4%EB%AC%BC_%EB%A1%9C%EB%B4%87_Jetson_%EB%B0%8F_Docker_%ED%86%B5%ED%95%A9_%EC%B4%9D%ED%8F%89_%EB%B0%8F_%EB%A7%88%EC%8A%A4%ED%84%B0_%ED%94%8C%EB%9E%9C.md)** | 🟢 **정식 등록** |

---

## 🔗 상위 및 하위 연계 런북 바로가기

1. **호스트 OS 전용 런북**: [`docs/jetson_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/README.md)
2. **도커 샌드박스 전용 런북**: [`docs/docker_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/docker_plan/README.md)
3. **실시간 온보드 종합 진단표**: [`docs/14_real_robot_live_system_diagnostic_report.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/14_real_robot_live_system_diagnostic_report.md)
