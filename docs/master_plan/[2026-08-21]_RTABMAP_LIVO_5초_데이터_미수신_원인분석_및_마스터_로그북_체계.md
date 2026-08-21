# 📑 [2026-08-21] RTAB-Map LIVO 5초 데이터 미수신 원인분석 및 에러/해결 마스터 로그북 체계 수립 명세서

> **작성 일자**: 2026년 8월 21일 (KST)  
> **문서 소유자**: **민석 (Minseok - Hardware & Jetson Lead)** & **도커/자율주행 Lead**  
> **논문 공식 명칭**: **`ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation` (ICRA 2026)**  
> **문서 목적**: 현장에서 발생한 `rtabmap: Did not receive data since 5 seconds!` 경고의 3대 기술적 근본 원인을 규명하고, **향후 발생하는 모든 오류와 해결 로그를 중복 없이 영구적으로 관리하기 위한 마스터 트러블슈팅 로그북([`docs/troubleshooting/ERROR_AND_RESOLUTION_MASTER_LOG.md`](file:///home/unitree/go2_ws_antarctica/docs/troubleshooting/ERROR_AND_RESOLUTION_MASTER_LOG.md)) 체계를 공식 수립**합니다.

---

## 📌 1. 현 이슈 (`[ERR-2026-08-21-01]`) 팩트체크 요약

```text
[rtabmap-6] rtabmap subscribed to (approx sync):
[rtabmap-6]    /camera/front/image_raw   <-- ✅ 정상 (30.0 fps)
[rtabmap-6]    /camera/front/camera_info  <-- ✅ 정상 (30.0 fps)
[rtabmap-6]    /pointcloud                <-- ❌ 문제 발생: 0 Hz (발행자 없음)
[rtabmap-6] [WARN] rtabmap: Did not receive data since 5 seconds!
```

* **원인 1 (토픽 불일치)**: RTAB-Map 기본값이 `/pointcloud`로 설정되어 있었으나, 실제 라이다는 `/utlidar/cloud`로 발행함.
* **원인 2 (노드 기동 누락)**: `bringup_all_escape_nav.sh`에서 라이다 드라이버(`unitree_lidar_ros2_node`) 및 IMU 노드(`go2_native_sensor_node.py`) 실행 코드가 빠져 있었음.
* **원인 3 (IP 바인딩 누락)**: 라이다 UDP 통신을 위한 `192.168.1.2/24` 에일리어스 IP 미할당.

---

## 🛠️ 2. 중앙 에러 & 해결 마스터 로그북 규격화

앞으로 모든 에러는 아래 표준 로그북에 고유 식별자(`[ERR-YYYY-MM-DD-NN]`)와 함께 6단계로 누적 기록됩니다:
👉 **[중앙 에러 & 해결 마스터 로그북 바로가기](file:///home/unitree/go2_ws_antarctica/docs/troubleshooting/ERROR_AND_RESOLUTION_MASTER_LOG.md)**

1. **이슈 ID 및 발생 일시**: 고유 번호 발급
2. **증상 및 원본 에러 로그**: 터미널 텍스트 전문
3. **기술적 근본 원인 (Root Cause)**: 소스코드/토픽/네트워크 레벨 분석
4. **수정 내역 및 코드 Diff**: 변경 파일 및 수정 블록
5. **실측 검증 결과 (Verification)**: Hz, Latency, 정상 로그
6. **재발 방지 대책 (Prevention SOP)**: 런북 및 통합 스크립트 반영
