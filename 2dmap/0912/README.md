# 🗺️ 2026-09-12 Realigned 2D SLAM Map & Goals (`0912/`)

2026년 9월 12일 현장에서 실측 매핑 및 LiDAR ICP 글로벌 루프 클로저 재정렬(Reprocessed)을 통해 생성된 최신 **2D 점유 격자 지도(Occupancy Grid Map)** 및 **등록된 내비게이션 목표 지점(Goal 1, Goal 2)** 에셋입니다.

---

## 📊 맵 데이터베이스 정보 (`map-reprocessed.db`)
- **원본 DB**: `s2e-vlm-async-framework-minimal/.local-data/prepared-terminal-campaign-20260912/map-session/map-reprocessed.db`
- **해시(SHA-256)**: `b7ad87461b81f2bef1e02a527e35e1976c765c00ff2a98d8f9bd2742f6e437eb`
- **총 노드(Nodes)**: 536개
- **최적화 포즈(Optimized Poses)**: 330개
- **글로벌 루프 클로저(Global Loop Closures)**: 25개
- **시작-종료점 오차(Endpoint Gap)**: **4.1 cm** (초기 88.6 cm에서 재정렬 후 대폭 개선)

---

## 🎯 등록된 내비게이션 골 포즈 (Navigation Goals)

| 골 번호 | 명칭 | X 좌표 (m) | Y 좌표 (m) | 헤딩 (Yaw) | 도착 허용 반경 | 설명 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **Goal #1** | `Waypoint_1` | **-61.124** | **7.532** | $92.2^\circ$ ($1.609\text{ rad}$) | 1.0 m | 복도 진입 직진 웨이포인트 |
| **Goal #2** | `Waypoint_2` | **-51.033** | **7.152** | $-35.7^\circ$ ($-0.624\text{ rad}$) | 1.0 m | 복도 코너 전방 웨이포인트 (직접 후보 정렬) |

---

## 📂 파일 구성
- `2d.png`: 5cm 해상도 정제 2D 점유 격자 지도
- `0833.pgm`: ROS 2 Map Server 표준 호환 파일
- `2d_metadata.json`: 실좌표 범위 `[-67.409m, -36.559m] x [-11.146m, 18.104m]` (617 x 585 px)
- `trajectory.png`: SLAM 최적화 주행 궤적 및 4.1cm 닫힘 플롯
- `2d_map_with_trajectory.png`: 2D 지도 위 실주행 궤적 오버레이
- `2d_goals_map.png`: 2D 지도 위 Goal 1, Goal 2 핀/헤딩 오버레이
- `navigation_goals.json` / `navigation_goals.yaml`: 자율주행 실행기 연동용 골 목록
- `goals/1.json`, `goals/2.json`: 원본 골 캡처 레코드
