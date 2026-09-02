# 🛠️ 시스템 안전 장치 및 트러블슈팅 가이드 (Fail-Safes & Troubleshooting)

> **문서 버전**: v1.0.0  
> **대상 플랫폼**: Unitree Go2 EDU (Jetson Orin NX + RTAB-Map + PixNav/ESCAPE-Nav)  
> **참조 경로**: `system_bringup/02_fail_safes_and_troubleshooting.md`

---

## 1. 🚨 대칭 복도에서의 20m 텔레포트 오인 락온 (Perceptual Aliasing)

### 1.1 문제 현상
* 로봇을 복도 시작점($Y \approx -7\text{m}$)에 두고 위치추정을 켰는데, 1초 만에 복도 반대편 끝($Y \approx -21.6\text{m}$, Node 110)으로 좌표가 20미터 이상 순간이동하여 락온되는 현상.
* 이 상태에서 골을 등록하면 유효 맵 범위 바깥($Y = -27\text{m}$)으로 골 좌표가 왜곡되어 저장됩니다.

### 1.2 원인
* 일직선 복도는 흰 벽면, 문, 천장 형광등이 일정하게 반복되는 대칭 구조를 가집니다.
* RTAB-Map 부팅 시 카메라(DBoW2 비주얼 피처)와 3D 라이다가 복도 시작점의 뷰를 복도 끝점의 뷰와 동일한 것으로 착각하여 잘못된 전역 루프 클로저(False Relocalization)를 확정해 버리기 때문입니다.

### 1.3 현장 해결 수칙 (SOP)
1. **특징점이 뚜렷한 출발 지점에서 기동**:
   * 양쪽이 똑같은 복도 한가운데서 로봇을 켜지 마십시오.
   * 반드시 **매핑을 처음 시작했던 고유한 장소(예: 출입문 모서리, 소화전, 로비 연결부 등 비대칭 구조물이 있는 곳)**에 로봇을 두고 `./run_localization.sh`를 실행하십시오.
2. **HUD 실시간 좌표 확인**:
   * HUD에 출력되는 $Y$ 좌표가 복도 시작점($Y \approx -7\text{m} \sim -9\text{m}$) 부근인지 확인하십시오.
   * 만약 $Y \approx -21\text{m}$로 튀어 있다면, 로봇을 특징점이 있는 벽면 쪽으로 살짝 비춰주거나 재부팅하십시오.

---

## 2. 🔌 토픽 네임스페이스 점검 (`/localization_pose` 단절 해결)

### 2.1 문제 현상
* `go2_localization_and_goal_recorder.py`의 5초 웜업 화면에서는 `🟢 LOCALIZED`라고 뜨는데, 실제 CSV 로그에는 `LOCALIZED`가 0건이고 전부 `TF_TRACKING`으로만 기록되는 현상.

### 2.2 원인
* `go2_rtabmap.launch.py`에서 `rtabmap` 노드가 네임스페이스 없이 실행되어 실제 발행되는 토픽은 **`/localization_pose`**입니다.
* 그런데 수신 노드가 **`/rtabmap/localization_pose`**를 구독하고 있으면 토픽이 단절되어 `pose_callback`이 호출되지 않습니다.

### 2.3 점검 및 해결
터미널에서 실제 발행되는 토픽을 확인하십시오:
```bash
ros2 topic list | grep localization_pose
```
* 출력 결과가 `/localization_pose`로 나온다면, 수신 스크립트에서도 `/localization_pose`를 구독하도록 일치시켜 주어야 정밀 공분산(신뢰도) 데이터를 정상 수신할 수 있습니다.

---

## 3. 🔄 25° 강제 제자리 회전 가드와 헛바퀴 현상

### 3.1 문제 현상
* 주행을 시작하자마자 로봇이 앞으로 가지 않고 제자리에서 20초 이상 180도 뺑뺑이를 도는 현상.

### 3.2 원인
* [go2_autonomous_navigator.py](file:///home/unitree/go2_ws_antarctica/scratch/go2_autonomous_navigator.py) 라인 788에 들어있는 안전 가드 때문입니다:
  ```python
  is_aligning_in_place = abs(rel_heading_deg) > 25.0
  if is_aligning_in_place:
      target_vx = 0.0
      target_wz = math.copysign(0.40, rel_heading)
  ```
* 목표가 등 뒤($> 25^\circ$)에 있으면 복도 벽 충돌을 막기 위해 신경망의 전진 명령을 무시하고 제자리 회전만 시킵니다.
* 만약 위치추정이 튀어서 목표가 등 뒤에 있다고 잘못 인식되면, 이 코드가 작동하여 목표 각도로 정렬될 때까지 강제로 회전합니다.

### 3.3 해결
* 로봇이 복도 방향을 올바르게 바라보고 있는지 확인하고,
* 잘못 락온된 상태에서 골이 반대편에 찍혀 있지 않은지 `config/navigation_goals.json`을 점검하십시오.

---

## 4. 🧹 비정상 골 좌표 초기화 및 복구

골 좌표가 왜곡되었거나 맵 바깥으로 튀었을 때는 즉시 초기화하십시오:

```bash
cd /home/unitree/go2_ws_antarctica
# 방법 1: localization HUD에서 'clear' 입력
./run_localization.sh
# 프롬프트에서 'clear' 입력 후 엔터 -> 모든 골과 스냅샷 삭제 완료

# 방법 2: 백업된 정상 골 좌표 복원
cp config/legacy/navigation_goals_backup.json config/navigation_goals.json
```
