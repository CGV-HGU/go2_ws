# 🏆 Unitree Go2 ESCAPE-Nav 마스터 플랜 및 총평 허브 (`docs/master_plan/`)

> **폴더 목적**: `docs/jetson_plan/`(호스트 런북)과 `docs/docker_plan/`(도커 런북)의 성과를 집대성하여, **실시간 6대 시스템 진단 결과, 아키텍처 팩트체크, 장애물 회피 API/충돌 감지 설계서, PointNav 5-Set 맵 계획, Jetson & Docker 담당자별 2인 최종 탑재 SOP, 서버 통신 6대 잠재 이슈 가이드, 그리고 RTAB-Map LIVO 아키텍처 타당성 및 센서별 동작원리 해설서를 날짜별로 체계적으로 관리하는 중앙 총평 허브**입니다.

---

## 📂 마스터 플랜 및 현장 운영 런북 목록 (8대 마스터 문서 체계)

| 작성 일자 | 마스터 플랜 문서명 | 주요 내용 및 링크 | 상태 |
| :--- | :--- | :--- | :---: |
| **2026-08-21** | **`[2026-08-21]_RTAB-Map_LIVO_아키텍처_타당성_및_센서별_동작원리_해설서.md`** | • LIVO 도입의 필연성 및 기술적 정당성<br/>• 4대 센서(L1 라이다, IMU, RGB 카메라, 오도메트리) 동작 원리<br/>• 센서 상보적 결합(Complementary Synergy) 매트릭스<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-21%5D_RTAB-Map_LIVO_%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98_%ED%83%80%EB%8B%B9%EC%84%B1_%EB%B0%8F_%EC%84%BC%EC%84%9C%EB%B3%84_%EB%8F%99%EC%9E%91%EC%9B%90%EB%A6%AC_%ED%95%B4%EC%84%A4%EC%84%9C.md)** | 🟢 **최신 (Latest)** |
| **2026-08-21** | **`[2026-08-21]_서버_통신_6대_잠재_이슈_분석_및_강건성_진단_가이드.md`** | • Wi-Fi 로밍 유실, vLLM 지연 스파이크, JSON 파싱 방어<br/>• 패킷 역전 방지, 720p 압축, 500ms 안전 워치독<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-21%5D_%EC%84%9C%EB%B2%84_%ED%86%B5%EC%8B%A0_6%EB%8C%80_%EC%9E%A0%EC%9E%AC_%EC%9D%B4%EC%8A%88_%EB%B6%84%EC%84%9D_%EB%B0%8F_%EA%B0%95%EA%B1%B4%EC%84%B1_%EC%A7%84%EB%8B%A8_%EA%B0%80%EC%9D%B4%EB%93%9C.md)** | 🟢 **정식 등록** |
| **2026-08-21** | **`[2026-08-21]_Jetson_및_Docker_담당자별_실물_로봇_최종_탑재_및_동시_실증_운영_SOP.md`** | • 2인 전담 역할 및 책임 분담표 (R&R Matrix)<br/>• Jetson 담당자 및 Docker 담당자별 사전 점검 체크리스트<br/>• 1-Click 동시 결합 실행 및 비상정지 E-Stop<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-21%5D_Jetson_%EB%B0%8F_Docker_%EB%8B%B4%EB%8B%B9%EC%9E%90%EB%B3%84_%EC%8B%A4%EB%AC%BC_%EB%A1%9C%EB%B4%87_%EC%B5%9C%EC%A2%85_%ED%83%91%EC%9E%AC_%EB%B0%8F_%EB%8F%99%EC%8B%9C_%EC%8B%A4%EC%A6%9D_%EC%9A%B4%EC%98%81_SOP.md)** | 🟢 **정식 등록** |
| **2026-08-21** | **`[2026-08-21]_PointNav_실로봇_5set_맵계획_및_HabitatGS_노이즈_Ablation_종합명세서.md`** | • 실로봇 5-Set 단일 맵 다중 좌표 운용 프로토콜<br/>• Habitat-GS ↔ S2E Async 연동 방안<br/>• GPS 센서 노이즈 vs Pose 드리프트 분리<br/>• VLM 모델 베리에이션 매트릭스 및 Ablation 인수인계<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-21%5D_PointNav_%EC%8B%A4%EB%A1%9C%EB%B4%87_5set_%EB%A7%B5%EA%B3%84%ED%9A%8D_%EB%B0%8F_HabitatGS_%EB%85%B8%EC%9D%B4%EC%A6%88_Ablation_%EC%A2%85%ED%95%A9%EB%AA%85%EC%84%B8%EC%84%9C.md)** | 🟢 **정식 등록** |
| **2026-08-21** | **`[2026-08-21]_Go2_장애물_회피_API_및_충돌_감지_파이프라인_설계서.md`** | • Unitree SDK2 `ObstaclesAvoidClient` API 분석<br/>• 운동학적 정체 감지(Kinematic Stall Detector)<br/>• `/robot/collision_detected` 토픽 및 S2E 연동<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-21%5D_Go2_%EC%9E%A5%EC%95%A0%EB%AC%BC_%ED%9A%8C%ED%94%BC_API_%EB%B0%8F_%EC%B6%A9%EB%8F%8C_%EA%B0%90%EC%A7%80_%ED%8C%8C%EC%9D%B4%ED%94%84%EB%9D%BC%EC%9D%B8_%EC%84%A4%EA%B3%84%EC%84%9C.md)** | 🟢 **정식 등록** |
| **2026-08-20** | **`[2026-08-20]_ESCAPE-Nav_Jetson_현장_실증_실행_로드맵_및_운영_매뉴얼.md`** | • 4대 계층 1-Click 마스터 브링업 구조<br/>• 현장 즉시 실행 4단계 실증 주행 절차 (Step 0~4)<br/>• 비상 정지(E-Stop) 및 안전 종료 수칙 수록<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-20%5D_ESCAPE-Nav_Jetson_%ED%98%84%EC%9E%A5_%EC%8B%A4%EC%A6%9D_%EC%8B%A4%ED%96%89_%EB%A1%9C%EB%93%9C%EB%A7%B5_%EB%B0%8F_%EC%9A%B4%EC%98%81_%EB%A7%A4%EB%89%B4%EC%96%BC.md)** | 🟢 **정식 등록** |
| **2026-08-20** | **`[2026-08-20]_ESCAPE-Nav_마스터플랜_팩트체크_및_고성능_아키텍처_개선보고서.md`** | • 마스터 플랜 팩트체크 및 6대 시스템 타당성 검증<br/>• Iceoryx2 Zero-Copy & cuPCL 오프체인 최적화<br/>• SGLang RadixAttention & TensorRT-LLM FP8<br/>• $\Delta T \approx 124\text{ms}$ 정밀 수식 및 3단계 로드맵<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-20%5D_ESCAPE-Nav_%EB%A7%88%EC%8A%A4%ED%84%B0%ED%94%8C%EB%9E%9C_%ED%8C%A9%ED%8A%B8%EC%B2%B4%ED%81%AC_%EB%B0%8F_%EA%B3%A0%EC%84%B1%EB%8A%A5_%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98_%EA%B0%9C%EC%84%A0%EB%B3%B4%EA%B3%A0%EC%84%9C.md)** | 🟢 **정식 등록** |
| **2026-08-20** | **`[2026-08-20]_ESCAPE-Nav_실물_로봇_Jetson_및_Docker_통합_총평_및_마스터_플랜.md`** | • 6대 전 영역 실측 진단 총평 (ALL PASS)<br/>• Jetson ↔ Docker 2대 축 통합 아키텍처<br/>• ICRA Table VIII 20회 주행 프로토콜<br/>• 1-Click 실행 매뉴얼 수록<br/>👉 **[문서 바로가기](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/master_plan/%5B2026-08-20%5D_ESCAPE-Nav_%EC%8B%A4%EB%AC%BC_%EB%A1%9C%EB%B4%87_Jetson_%EB%B0%8F_Docker_%ED%86%B5%ED%95%A9_%EC%B4%9D%ED%8F%89_%EB%B0%8F_%EB%A7%88%EC%8A%A4%ED%84%B0_%ED%94%8C%EB%9E%9C.md)** | 🟢 **정식 등록** |

---

## 🔗 상위 및 하위 연계 런북 바로가기

1. **호스트 OS 전용 런북**: [`docs/jetson_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/jetson_plan/README.md)
2. **도커 샌드박스 전용 런북**: [`docs/docker_plan/README.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/docker_plan/README.md)
3. **실시간 온보드 종합 진단표**: [`docs/14_real_robot_live_system_diagnostic_report.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/14_real_robot_live_system_diagnostic_report.md)
