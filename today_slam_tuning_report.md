# 📝 Go2 SLAM & VIO 파라미터 튜닝 및 히스토리 리포트 (2026-07-31)

## 📌 개요
본 문서는 Go2 4족보행 로봇과 Intel RealSense D435i 카메라 환경에서 RTAB-Map 기반 Visual-Inertial Odometry (VIO) 매핑의 안정성과 루프 클로저(Loop Closure) 성능을 극대화하기 위해 진행한 파라미터 튜닝, 부팅 시퀀스 최적화, 그리고 시도 후 원복된 사항들을 종합 정리는 보고서입니다.

---

## 🟢 1. 최종 정식 반영된 수정 사항 (Applied Modifications)

### ① RTAB-Map 파라미터 최적화 (`run_map.sh`)
* **`--Vis/MinInliers 10`**:
  * 기존 `15`에서 **`10`**으로 완화.
  * **사유**: 밋밋한 실내 벽이나 복도에서 특징점이 순간 부족할 때 오도메트리가 끊기는 현상을 방지하고 추적 유지력을 대폭 상향 (Omorobot 검증 수치).
* **`--Rtabmap/MinVisInliers 10`**:
  * 기존 기본값(`15`~`20`)에서 **`10`**으로 동기화.
  * **사유**: 실시간 오도메트리 기준과 맞춰 출발지/과거 장소 재방문 시 3D 기하학 검증 성공률을 극대화하여 루프 클로저 터짐을 보장.
* **`--Rtabmap/DetectionRate 2.0`**:
  * 기존 기본값(`1.0Hz`)에서 **`2.0Hz` (초당 2개)**로 2배 밀집.
  * **사유**: 3D 지도 키프레임 수집 밀도를 높여 로봇 회전 시 지도의 듬성듬성함을 없애고 루프 클로저 매칭 확률 향상.
* **`--Grid/RangeMin 0.3` & `--Grid/RangeMax 3.0`**:
  * **사유**: Intel RealSense D435i 데이터시트 정밀 구간 사양(최소 거리 28cm + 마진 2cm = `0.3m`, 인텔 공식 2% 고정밀 신뢰 한계 = `3.0m`)을 적용하여 5m 너머의 지저분한 오차 노이즈 완전 차단.
* **`--Grid/MaxGroundHeight 0.1`**:
  * **사유**: Go2 4족보행 로봇 보행 시 발걸음으로 인해 몸통이 위아래로 2~5cm 흔들리는 보행 충격 노이즈 및 장판/카펫 무늬 오인을 100% 완충 흡수하여 깨끗한 흰색 통행 영역(2D Occupancy Grid) 생성.

---

### ② 부팅 시퀀스 (Boot Sequence) 및 Sleep 대기 시간 최적화 (`run_map.sh`)
* **Static TF 전진 배치**:
  * 기존 RTAB-Map 구동 후(맨 마지막) 구동되던 Static TF를 **카메라 구동 직후(RTAB-Map 구동 전)로 전진 배치**.
  * **사유**: RTAB-Map 시작 시점에 TF 트리가 100% 준비되어 부팅 초기 2~3초 간 발생하던 TF 미수신/메시지 드랍 에러 원천 차단.
* **Madgwick IMU 필터 대기 시간 확대 (`sleep 6` ➔ `sleep 8`)**:
  * **사유**: IMU 센서 중력 가속도 수평 정렬(Gravity Orientation Convergence)이 100% 안착된 후 RTAB-Map이 구동되도록 보장.

---

### ③ RealSense Optical ➔ ROS Body Frame IMU 좌표계 변환 (`imu_relay.py`)
* RealSense D435i raw IMU ($X_{opt}=\text{우}, Y_{opt}=\text{하}, Z_{opt}=\text{전방}$) 축을 ROS 보디 프레임(`camera_link`: $X_{body}=Z_{opt}, Y_{body}=-X_{opt}, Z_{body}=-Y_{opt}$)으로 90도 정밀 회전 변환하여, 구동 2~3초 후 로봇과 카메라이 90도로 꺾이던 현상 완전 해결.

---

## 🔴 2. 시도 후 원복/복구된 수정 사항 (Attempted & Reverted Modifications)

### ① `--Odom/Strategy 0` (Frame-to-Map) 직접 주입 ➔ **원복 (`Odom/Strategy 1` 기본값 유지)**
* **시도 내용**: `rtabmap_args` 또는 `odom_args`에 `--Odom/Strategy 0` (Frame-to-Map) 옵션을 던짐.
* **실패 원인 및 원복 이유**:
  1. ROS 2 Foxy의 `rtabmap.launch.py` 및 C++ 커널 파서가 ROS 1 방식의 `--Odom/Strategy 0` 구문을 파싱할 때 `queue_size` 수신 큐를 1로 강제 리셋시켜 **`Message Filter dropping message` 메시지 폭발 및 랙 발생**.
  2. 휠 엔코더가 없는 VIO 단독 환경에서 캡(Cap) 옵션 없이 구동 시 주변 3D 특징점이 누적되어 Jetson 연산 부하 급증.

### ② `--Grid/MaxGroundHeight 0.0` (높이 0cm 바닥 기준) ➔ **원복 (`0.1`로 수정)**
* **시도 내용**: 오모로봇 휠 로봇 코드에서 사용하던 `MaxGroundHeight 0.0` 옵션을 그대로 가져옴.
* **실패 원인 및 원복 이유**: Go2 로봇은 카메라이 지면 위 약 40cm 높이에 부착되어 있어, `0.0` 세팅 시 3D 포인트 클라우드가 바닥 아래 데이터로 처리되거나 전부 삭제되어 **`/map` 지도가 아예 생성되지 않는 현상 발생**. 4족보행 흔들림을 완충하는 `0.1` (10cm)로 최종 교정.

### ③ IMU 공분산(Covariance) 배열 복사 코드 ➔ **원복 (기존 원본 벡터 대입 유지)**
* **시도 내용**: `imu_relay.py`에 공분산 복사 및 $R \cdot \Sigma \cdot R^T$ 선형대수학 회전 변환 구문 주입.
* **실패 원인 및 원복 이유**: RealSense D435i 내장 IMU의 raw 공분산 배열이 비어있거나 불완전하여, 수식 적용 시 예기치 못한 필터 부작용 우려로 원래의 깨끗한 벡터 변환 원본 코드로 깔끔하게 복구.

---

## 💡 3. 핵심 요약 및 최종 파라미터 예시

### 📋 최종 적용된 `run_map.sh` 핵심 구문 ([87~90번 라인](file:///home/unitree/go2_ws/run_map.sh#L87-L90))

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start --Vis/MinInliers 10 --Rtabmap/MinVisInliers 10 --Rtabmap/DetectionRate 2.0 --Grid/RangeMin 0.3 --Grid/RangeMax 3.0 --Grid/MaxGroundHeight 0.1 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true --Optimizer/GravitySigma 0.3' \
    frame_id:=camera_link \
    visual_odometry:=true \
```

### 🌐 최적화된 부팅 시퀀스
1. **[1/5] Camera Node 구동** ➔ `sleep 10`
2. **[2/5] Static TF 미리 출판** ➔ `sleep 2`
3. **[3/5] IMU Relay 구동** ➔ `sleep 2`
4. **[4/5] IMU Filter (Madgwick) 구동** ➔ `sleep 8`
5. **[5/5] RTAB-Map 구동** ➔ 완료!
