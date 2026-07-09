# ❄️ Unitree Go2 Antarctic Navigation Project

본 저장소는 남극 및 극한 지형 환경에서 사족보행 로봇 **Unitree Go2**의 자율주행을 제어하기 위한 ROS 2 Humble/Foxy 기반의 전용 워크스페이스(`go2_ws`)임. 

이 브랜치(`antarctica`)는 공동 연구(LIO/S2E/VLM)와 개인 연구(VIO/RTAB-Map)를 효율적으로 병행할 수 있도록 불필요한 기존 Nav2 템플릿들을 배제하고, 전체 개발 및 배포 전략 문서를 모듈별 상세 문서로 분할 설계하여 기술 관리성을 극대화한 정돈된 샌드박스 환경임.

---

## 📂 프로젝트 핵심 기술 문서 일람 (Modular Documentation)

실물 로봇 연동 및 배포에 필요한 모든 기술 사양, 네트워크 트러블슈팅, 제어 매핑, 튜닝 가이드라인은 각 모듈별 상세 문서로 분리되어 관리되며 아래 링크를 통해 즉시 조회할 수 있음.

### 1. [시스템 아키텍처 및 파이프라인 명세서 (docs/01_system_architecture.md)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/docs/01_system_architecture.md)
*   전체 프로세스 흐름도(Mermaid) 및 3대 핵심 모듈(Odometry, Controller, Action API) 정의.
*   ViNT 및 Go2 SDK 간의 소스코드 레벨 인터페이스 결합점 분석.
*   학습 및 제어 단계에서의 궤적 정규화 및 복원 연산(Recovery) 프로세스 상세 정의.

### 2. [네트워크 및 DDS 환경 구성 가이드 (docs/02_network_and_dds_setup.md)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/docs/02_network_and_dds_setup.md)
*   학교망 및 VPN 환경 가동 시 로컬 DDS 통신 유실 및 멀티캐스트 차단 극복을 위한 CycloneDDS 설정 가이드.
*   화요일 현장 통신 검증을 위한 ROS 2 Foxy-Jazzy 간 DDS 루프백 다이렉트 테스트 명령어 세트.

### 3. [비동기 프레임워크 설계 분석 명세서 (docs/03_async_framework_analysis.md)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/docs/03_async_framework_analysis.md)
*   비동기 주행 백엔드 프레임워크(`s2e-vlm-async-framework`) 소스코드 기반 분석 보고서.
*   제자리 회전 액션 정책(Rotate-to-Front) 및 오도메트리 child_frame_id (`base_link`) 규격 정리.

### 4. [탑재 센서 하드웨어 세부 제원표 (docs/04_sensor_specifications.md)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/docs/04_sensor_specifications.md)
*   Go2 전면 내장 RGB 카메라 및 내장 4D LiDAR L2 센서 사양 정리.
*   추가 탑재된 Intel RealSense D435i 깊이 카메라의 상세 기술 제원.

### 5. [젯슨 하이브리드 분리 아키텍처 및 배포 가이드 (docs/05_hybrid_split_architecture.md)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/docs/05_hybrid_split_architecture.md)
*   Tegra 드라이버 Mismatch 버그를 우회하는 젯슨 온보드 하이브리드(GPU-CPU 분리) 구조도(Flowchart).
*   도커 이미지 수동 설치, CPU-only 컨테이너 기동, Colcon 빌드, 프레임워크 컴파일 및 테스트용 전체 CLI 명령어 가이드.

### 6. [ViNT 궤적 제어 매핑 및 튜닝 전략 (docs/06_vint_control_mapping_strategy.md)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/docs/06_vint_control_mapping_strategy.md)
*   ViNT 순정 제어기 물리 연산식의 한계 및 이를 대체하는 피드백 PD 제어기 비교 분석.
*   Go2 Sport API 보행 안전성 확보 설계($v_y=0.0$) 및 피쉬테일링(Fishtailing) 요동 제어 튜닝 가이드.

### 7. [유니트리 공식 SDK 레퍼런스 코드 모음 (docs/07_reference_code_snippets.md)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/docs/07_reference_code_snippets.md)
*   코드 안정성 확보를 위해 아카이빙된 C++/Python SDK 제어 및 실시간 센서/모션 상태 수신 예제 족보.

---

## 📂 8. antarctica 브랜치 디렉토리 구조 (Clean Workspace)

```text
go2_ws/
├── README.md                  <- 본 대문 가이드 포털 문서 (지속 업데이트)
├── cyclonedds.xml             <- CycloneDDS 네트워크 바인딩 설정 파일
├── docs/                      <- [신설] 모듈화된 상세 기술 문서 보관 폴더
│   ├── 01_system_architecture.md
│   ├── 02_network_and_dds_setup.md
│   ├── 03_async_framework_analysis.md
│   ├── 04_sensor_specifications.md
│   ├── 05_hybrid_split_architecture.md
│   ├── 06_vint_control_mapping_strategy.md
│   └── 07_reference_code_snippets.md
├── scratch/                   <- 실물 연동 검증용 UDP 소켓/파이썬 백업 드라이버 스크립트 폴더
│   ├── host_bridge.py
│   ├── docker_bridge.py
│   └── python_direct_driver.py
├── visualnav-transformer/     <- ViNT / NoMAD 모델 구현 및 pd_controller.py 코드
├── qwen_nav_memory_framework_v3/ <- 상위 VLM 기반 에피소딕 메모리 프레임워크 패키지
├── s2e-vlm-async-framework/   <- ROS 2 비동기 통합 프레임워크 패키지
└── src/
    ├── HesaiLidar_ROS_2.0/    <- Hesai 라이다 연동 ROS 2 드라이버
    ├── rtabmap_ros/           <- rtabmap SLAM 패키지 (이민석 개인 VIO 연구용)
    └── go2_robot/             <- Unitree Go2 ROS 2 통신 패키지
```
