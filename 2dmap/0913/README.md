# 🗺️ 2026-09-13 Full Area 2D SLAM Map & 5 Goals (`0913/`)

2026년 9월 13일 현장에서 신규 고정 시작점(Fixed Start, Origin `(0.0, 0.0)`) 기준으로 전 영역 SLAM 매핑 및 5개 내비게이션 목표 지점(Goal 1 ~ Goal 5)을 기록한 최신 **2D 점유 격자 지도(Occupancy Grid Map)** 및 **시각화 에셋**입니다.

---

## 📊 맵 데이터베이스 정보 (`map.db`)
- **원본 디렉토리**: `s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-20260913/mapping/map.db`
- **파일 크기**: 96.8 MB
- **총 노드(Nodes)**: 365개 (최적화 링크 904개)
- **시작-종료점(Start-End)**: 노드 1 `(0.003, 0.006) m` $\to$ 노드 365 `(0.014, -0.321) m` (완벽 루프 클로저 수렴)
- **맵 실좌표 바운딩 박스**: $X \in [-3.887, 20.589]\text{ m}$, $Y \in [-12.866, 13.091]\text{ m}$
- **해상도 및 크기**: $0.05\text{ m/px}$ (5 cm/px), $490 \times 520\text{ px}$

---

## 🎯 등록된 5개 내비게이션 골 포즈 (Navigation Goals)

기준 좌표계: `fixed_start` (원점 $X=0.0\text{ m}, Y=0.0\text{ m}$, 헤딩 $0.0^\circ$)

| 골 번호 | 명칭 | X 좌표 (m) | Y 좌표 (m) | 헤딩 (Yaw) | 원점 직선거리 | 허용 반경 | 비고 및 위치 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **Goal #1** | `Waypoint_1` | **+2.976** | **+1.675** | $-9.4^\circ$ ($-0.165\text{ rad}$) | **3.42 m** | 1.0 m | 좌측 상단 복도 진입로 |
| **Goal #2** | `Waypoint_2` | **+4.807** | **-2.420** | $+18.5^\circ$ ($+0.323\text{ rad}$) | **5.38 m** | 1.0 m | 좌측 하단 복도 통로 |
| **Goal #3** | `Waypoint_3` | **+9.300** | **+0.424** | $-11.6^\circ$ ($-0.202\text{ rad}$) | **9.31 m** | 1.0 m | 중앙 교차로 중심부 |
| **Goal #4** | `Waypoint_4` | **+10.614** | **-3.886** | $-36.4^\circ$ ($-0.635\text{ rad}$) | **11.30 m** | 1.0 m | 우하단 삼각 루프 복도 코너 |
| **Goal #5** | `Waypoint_5` | **+7.955** | **+4.189** | $+82.8^\circ$ ($+1.445\text{ rad}$) | **8.99 m** | 1.0 m | 상단 북측 복도 분기점 |

*공식 설정 파일: `config/navigation_goals.json`, `config/navigation_goals.yaml`*  
*원본 레코드: `s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-20260913/goal-plan.json`*

---

## 📂 파일 구성
- `2d.png`: 5cm 해상도 정제 2D 점유 격자 지도 (Occupancy Grid)
- `0833.pgm`: ROS 2 Map Server 표준 호환 PGM 파일
- `0833.yaml`: ROS 2 Map Server 메타데이터 YAML 설정
- `2d_metadata.json`: 실좌표 바운딩 박스 및 픽셀 변환 매개변수
- `trajectory.png`: SLAM 매핑 풀 궤적 플롯
- `2d_map_with_trajectory.png`: 2D 격자 지도 위 매핑 주행 궤적 오버레이
- `2d_goals_map.png`: 2D 지도 위 Start + 5개 골 핀, 1.0m 허용반경 원, Yaw 방향 화살표 오버레이
- `2d_clean_goals_map.png`: **ICRA 논문 Fig. 6 스타일** 고대비 복도 지도 (라이다 모래/점군 노이즈 완전 제거, 5m 스케일바, 범례 포함)
- `2d_wall_only_goals_map.png`: **CAD 스타일 벽면 전용** 지도 (순백 배경 + 선명한 검은 벽면 + 5개 골 마커)
- `fig_five_goals.pdf`: 논문 집필용 고해상도 벡터 PDF
- `navigation_goals.json` / `navigation_goals.yaml`: 5개 골 포즈 공식 정의
