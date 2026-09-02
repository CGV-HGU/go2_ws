# 🗺️ 2D Map & SLAM Assets Guide (`2dmap/`)

이 디렉터리는 Unitree Go2의 3D LiDAR/비전 RTAB-Map SLAM을 통해 실측 생성된 **2D 점유 격자 지도(Occupancy Grid Map) 및 위치 추정용 에셋**을 관리합니다.

---

## 📂 파일 구성 및 역할

| 파일명 | 종류 | 설명 | 연동 시스템 |
|---|:---:|---|---|
| **`2d.png`** | 이미지 (PNG) | **최신 2D 복도 골든 맵 (Canonical Golden Map)**<br>노이즈 제거 및 구조 벽체 보강이 완료된 실주행 기준 2D 지도 | `go2_autonomous_navigator.py`<br>`go2_localization_and_goal_recorder.py` |
| **`2d_metadata.json`** | 메타데이터 (JSON) | **맵 실좌표 범위 및 해상도**<br>`min_x`, `max_x`, `min_y`, `max_y`, `resolution` (0.05m/px) 수록 | 네비게이터 바운더리 체크<br>E2E 자동 감사 테스트 |
| **`2d_goals_map.png`** | 이미지 (PNG) | **웨이포인트 골 핀 오버레이 맵**<br>`navigation_goals.json`에 등록된 골 위치가 시각적으로 표시된 실시간 맵 | 주행 전 골 확인 대시보드 |
| **`0833.pgm`** | 이미지 (PGM) | ROS 2 `nav2_map_server` 표준 호환용 2D Occupancy Grid 바이너리 | ROS 2 Map Server |
| **`rtabmap_native_cloud.ply`** | 3D 점군 (PLY) | RTAB-Map SLAM DB에서 직접 추출한 원본 3D 라이다 포인트클라우드 (126개 노드 통합) | 3D 형상 검증 및 분석 |
| **`rtabmap_native_poses.txt`** | 궤적 데이터 (TXT) | 매핑 당시 로봇이 주행한 3D 포즈 노드 시퀀스 | 정밀 궤적 분석 |
| **`rtabmap_native_pure_2d.png`** | 이미지 (PNG) | 3D 점군을 수평 절단하여 투영한 순수 2D 레이아웃 | 맵 정밀도 검증 |
| **`backup/`** | 디렉터리 | 과거 매핑 회차별(08-27, 08-31 등) 원본 데이터 및 이전 맵 아카이브 | 이력 관리 및 회귀 테스트 |

---

## 📐 좌표계 및 메타데이터 기준 (`2d_metadata.json`)

* **해상도 (Resolution)**: `0.05 m/pixel` (픽셀당 5cm)
* **X축 실좌표 범위**: `[-12.73m, -1.30m]` (가로 폭 ~11.4m)
* **Y축 실좌표 범위**: `[-27.04m, -4.39m]` (세로 길이 ~22.6m 일직선 복도)
* **이미지 크기**: 229 x 454 픽셀

---

## 🔄 지도 갱신 절차 (How to Update)

1. 신규 매핑 완료 후 `~/.ros/rtabmap.db`가 갱신되면:
   ```bash
   python3 scratch/extract_final_golden_map.py
   ```
2. 자동으로 `2d.png`, `2d_metadata.json`, `0833.pgm`이 추출 및 정제되어 즉시 네비게이터와 연동됩니다.
