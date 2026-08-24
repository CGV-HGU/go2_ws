# 🗺️ [Know-How 03] RTAB-Map LIVO 3D/2D SLAM 튜닝 및 공식 포럼 파라미터 정밀 해설서

> **대상 시스템**: RTAB-Map ROS 2 (LIVO 융합 SLAM), 2D Occupancy Grid Map (`2dmap/0833`)  
> **문서 목적**: Mathieu Labbé 교수 공식 포럼 권고안, 3DoF 평면 구속, 2D 점유격자 가시 번짐 억제, 1초 클린맵 후처리 원리 해설

---

## 1. 🔍 LIVO 환경에서 RTAB-Map 핵심 파라미터 설계 근거

1. **`Reg/Strategy: 1` (ICP Scan Matching)**:
   - Depth 카메라가 없는 LIVO 환경에서는 카메라의 시각적 외형(Visual Odometry)만으로 거리 오차를 잡을 수 없습니다.
   - 4D 라이다의 3차원 점군(`/pointcloud`)을 이용한 ICP 정합으로 루프 클로저와 맵핑의 기하학적 정밀도를 극대화합니다.
2. **`Reg/Force3DoF: true` & `Optimizer/Slam2D: true` (2D 평면 구속)**:
   - 실내 평탄 복도 환경에서 4족 보행 로봇은 보행 트로팅 충격으로 미세한 Pitch/Roll 진동을 일으킵니다.
   - 이를 6DoF 자유도로 최적화하면 Z축으로 지도가 비틀리는 드리프트가 누적되므로, 2D 평면($x, y, \text{yaw}$)으로 강제 구속하여 복도 벽면의 수직도를 완벽히 유지합니다.
3. **`Grid/NormalsSegmentation: false` (고속 높이 패스스루 필터)**:
   - 비정형(Unorganized) 4D 라이다 점군에서 법선 벡터(Normals)를 계산하면 계산 비용이 크고 바닥 요철이 장애물로 튈 수 있습니다.
   - 단순 높이 필터(`MinGroundHeight: -0.20`, `MaxGroundHeight: 0.05`, `MaxObstacleHeight: 1.50`)로 분리하여 0ms 지연으로 깨끗한 장애물 격자를 생성합니다.

---

## 2. ⚡ 2D 점유격자 외곽 가시(Spike) 번짐 억제 및 후처리 정제

* `Grid/RangeMax: 8.0`을 설정하여 열린 문틈이나 복도 끝으로 레이트레이싱 광선이 길게 뻗어나가는 가시 번짐을 억제합니다.
* 생성된 맵은 [`scratch/clean_and_export_2d_map.py`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/scratch/clean_and_export_2d_map.py)를 통해 1초 만에 논문용 클린맵으로 자동 정제됩니다.
