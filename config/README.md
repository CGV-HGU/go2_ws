# 🎯 Navigation Goals Configuration (`config/`)

이 디렉터리는 Unitree Go2의 **자율주행 목표 웨이포인트(Waypoints) 및 시각적 스냅샷**을 관리합니다.

---

## 📂 파일 구성 및 역할

| 파일 / 폴더 | 형식 | 설명 |
|---|:---:|---|
| **`navigation_goals.json`** | JSON | **실주행 활성 골 목록 (Primary Source of Truth)**<br>네비게이터(`go2_autonomous_navigator.py`) 및 도커 S2E 파이프라인에서 직접 파싱 |
| **`navigation_goals.yaml`** | YAML | `navigation_goals.json`과 100% 동기화된 YAML 포맷 설정 파일 |
| **`navigation_goals_backup_oldmap.yaml`** | YAML | 이전 맵 좌표계 기준 과거 골 좌표 백업 파일 |
| **`goals/`** | 디렉터리 | 골 등록 시점(`run_local.sh`)에 로봇 전면 카메라로 자동 캡처된 실제 전방 시각 스냅샷 (`goal_XX_Name.jpg`) |

---

## 📝 골 데이터 스키마 (Schema)

```json
{
  "id": 1,
  "name": "Waypoint_1_Mid",
  "description": "Corridor destination waypoint #1",
  "x_m": -7.01,
  "y_m": -20.25,
  "z_m": 0.31,
  "yaw_deg": -90.0,
  "tolerance_m": 0.5,
  "snapshot_image": "config/goals/goal_01_Waypoint_1.jpg"
}
```

* `x_m`, `y_m`, `z_m`: 전역 맵 프레임(`map`) 기준 실좌표 (미터 단위)
* `yaw_deg`: 목표 지점 도착 시 지향해야 할 방향각 (도 단위)
* `tolerance_m`: 도착 인정 반경 (기본 0.5m)
* `snapshot_image`: VLM 시각 서보잉 및 비교용 전방 카메라 스냅샷 경로

---

## 🎮 신규 골 등록 방법

1. 로봇 전원을 켜고 `./run_local.sh` (또는 `./run_goal_recorder.sh`)를 실행합니다.
2. 5초 안정화 후 원하는 위치로 로봇을 이동한 뒤 **엔터(Enter)**를 누르면:
   - 현재 (X, Y, Z, Yaw) 좌표가 즉시 기록됩니다.
   - 전면 카메라 뷰(`goals/goal_XX.jpg`)가 자동 저장됩니다.
   - `navigation_goals.json`, `yaml`, `2dmap/2d_goals_map.png`가 모두 1-Click 자동 갱신됩니다.
