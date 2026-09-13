# 🗺️ 2026-09-14 Recaptured Goal 5 & 5 Navigation Goals (`0914/`)

2026년 9월 14일 재부팅 후 출발점(`origin.json`) 및 로봇 복도 주행을 통해 **Goal #5를 올바른 복도 중앙 위치(직선거리 15.86m, 복도 남측 방향)**로 정상 재캡처(Recaptured)하여 봉인(`goals_sealed`)한 최신 2D 내비게이션 맵 및 에셋입니다.

---

## 📊 캠페인 메타데이터 (`recording-five-goals-recaptured5-20260914`)
- **캠페인 ID**: `80bf2e8fb62648deb43b107d22a6b836`
- **원본 경로**: `s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-recaptured5-20260914`
- **Goal Plan SHA256**: `4669df419d2d3e9d91c3e71d1fbd5ae9d4d4d245d3a8b2beb2f62636f308bf29`
- **상태**: `goals_sealed`
- **리턴 오차 검증(Return Closure)**: 위치 오차 $0.468\text{ m}$, 헤딩 오차 $5.43^\circ$ (동일 물리적 출발 마커 복귀 확인)
- **기존 0913 SLAM 맵 재사용**: `s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-20260913/mapping/map.db`

---

## 🎯 최종 확정된 5개 내비게이션 골 포즈 (Navigation Goals)

기준 좌표계: `fixed_start` (원점 $X=0.0\text{ m}, Y=0.0\text{ m}$, 헤딩 $0.0^\circ$)

| 골 번호 | 명칭 | X 좌표 (m) | Y 좌표 (m) | 헤딩 (Yaw) | 원점 직선거리 | 맵 오버레이 좌표 (X, Y) | 허용 반경 | 비고 및 위치 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **Goal #1** | `Waypoint_1` | **+2.976** | **+1.675** | $-9.4^\circ$ ($-0.165\text{ rad}$) | **3.42 m** | $(+2.941, +1.750)\text{ m}$ | 1.0 m | 좌상단 복도 진입로 코너 |
| **Goal #2** | `Waypoint_2` | **+4.807** | **-2.420** | $+18.5^\circ$ ($+0.323\text{ rad}$) | **5.38 m** | $(+5.057, -2.205)\text{ m}$ | 1.0 m | 좌하단 복도 통로 |
| **Goal #3** | `Waypoint_3` | **+9.300** | **+0.424** | $-11.6^\circ$ ($-0.202\text{ rad}$) | **9.31 m** | $(+9.337, +0.950)\text{ m}$ | 1.0 m | 중앙 교차로 중심부 |
| **Goal #4** | `Waypoint_4` | **+10.614** | **-3.886** | $-36.4^\circ$ ($-0.635\text{ rad}$) | **11.30 m** | $(+10.954, -3.256)\text{ m}$ | 1.0 m | 우하단 삼각 루프 복도 코너 |
| **Goal #5 (NEW)** | `Waypoint_5` | **+14.443** | **-6.560** | **$-94.0^\circ$ ($-1.641\text{ rad}$)** | **15.86 m** | **$(+14.963, -5.652)\text{ m}$** | 1.0 m | **최장거리 원거리 골 (우측 수직 복도 중앙, 남향)** |

*공식 설정 파일: `config/navigation_goals.json`, `config/navigation_goals.yaml`*

---

## 📂 파일 구성
- `2d_goals_map_recaptured5.png` / `.pdf` / `.csv` / `.json`: **Goal 5 재캡처 반영 공식 캠페인 맵 에셋**
- `2d_clean_goals_map.png`: **ICRA 논문 Fig. 6 스타일** 고대비 복도 지도 (라이다 모래/점군 노이즈 완전 제거, 5m 스케일바, 범례 포함)
- `2d_wall_only_goals_map.png`: **CAD 스타일 벽면 전용** 지도 (순백 배경 + 선명한 검은 벽면 + 5개 골 마커)
- `fig_five_goals.pdf`: 논문 집필용 고해상도 벡터 PDF
- `navigation_goals.json` / `navigation_goals.yaml`: 0914 확정 5개 골 포즈 공식 정의
