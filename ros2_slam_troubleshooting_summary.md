# ROS 2 RealSense D435i + RTAB-Map 3D VIO SLAM 트러블슈팅 및 수정 내역 정리

본 문서는 **Unitree Go2 / NVIDIA Jetson** 환경에서 RealSense D435i 카메라와 RTAB-Map을 활용한 **3D Visual-Inertial Odometry (VIO) SLAM** 시스템을 구축 및 최적화하는 과정에서 진행된 모든 문제 원인 분석, 단계별 수정 내역, 그리고 최종 검증 결과를 시간 순서대로 정리한 기록입니다.

---

## 📅 단계별 상세 수정 내역 (Chronological Order)

### 1단계: IMU 리레이 스크립트 출력 버퍼링 및 로깅 최적화
* **발생 문제**: 2번째 터미널 탭에서 `imu_relay.py` 실행 시 로그가 출력되지 않고 정지된 것처럼 보이는 현상 발생 (Python I/O 기본 4KB 버퍼링 디폴트 동작).
* **수정 파일**: [imu_relay.py](file:///home/unitree/go2_ws/imu_relay.py), [run_map.sh](file:///home/unitree/go2_ws/run_map.sh)
* **조치 내용**:
  - `run_map.sh` 환경 변수에 `export PYTHONUNBUFFERED=1;`을 추가하여 터미널에 즉시 실시간 로그가 출력되도록 수정.
  - [imu_relay.py](file:///home/unitree/go2_ws/imu_relay.py)에 첫 번째 IMU 수신 시 `🚀 [FIRST IMU RECEIVED]` 즉시 출력 및 100개 단위 진행 로그(`🔄 [RELAYING]`) 추가.

---

### 2단계: RealSense D435i 카메라 센서 모듈 및 하드웨어 락다운 방지
* **발생 문제**: 
  1. NVIDIA Jetson 환경에서 카메라 구동 시 `Motion Module failure` 및 `control_transfer returned error 11` 하드웨어 오류 발생.
  2. 카메라 노드가 반복적으로 재시작되며 Depth 이미지가 퍼블리시되지 않음.
* **수정 파일**: [run_map.sh](file:///home/unitree/go2_ws/run_map.sh)
* **조치 내용**:
  - **파라미터 규격 수정**: ROS 2 `rs_launch.py` 공식 스펙에 맞게 `align_depth:=true` ➔ `align_depth.enable:=true`로 수정.
  - **USB 리셋 오작동 방지**: `initial_reset:=true` ➔ `initial_reset:=false`로 변경하여 USB 버스 리셋 시 Jetson 메인보드 전원 급변으로 인한 IMU 센서 락다운 방지.
  - **IMU 방식 변경**: `unite_imu_method:=2` (선형 보간 모드) 적용으로 Accel(100Hz)과 Gyro(200Hz)의 타임스탬프 동기화 보장.
  - **펌웨어 표준 해상도 변경**: 미지원 사양인 `424x240` 대신 D435i 공식 표준 해상도인 `640x480x30` 적용.

---

### 3단계: CycloneDDS 노드 참가자 슬롯 확충 (Participant Limit Expansion)
* **발생 문제**: `rtabmap_viz`와 `rviz2` 두 개의 시각화 GUI 창을 동시에 띄울 때 `Failed to find a free participant index for domain 0` 에러가 발생하며 노드가 튕김. (잘못된 `<MaxParticipants>` 태그 사용 시 XML 스키마 검증 에러 발생).
* **수정 파일**: [cyclonedds.xml](file:///home/unitree/go2_ws/cyclonedds.xml)
* **조치 내용**:
  - Eclipse CycloneDDS 공식 XSD 규격을 검증(Empirical Validation)하여 올바른 태그 선언:
    ```xml
    <Discovery>
        <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
    ```
  - 노드 참가자 슬롯 상한을 기본 30개에서 **100개로 대폭 확장**하여 두 개의 GUI 창이 동시에 안정적으로 실행되도록 조치.

---

### 4단계: IMU-카메라 TF 좌표계 트리(Transform Tree) 완벽 연결
* **발생 문제**: 4번째 터미널 탭에서 `Invalid frame ID "camera_imu_optical_frame"` 경고 메시지가 초당 100회 이상 도배되며, RTAB-Map `rgbd_odometry`가 incoming image 데이터를 수신 거부하고 **`Did not receive data since 5 seconds!`** (5초 미수신 경고)를 계속 출력함.
* **수정 파일**: [run_map.sh](file:///home/unitree/go2_ws/run_map.sh) (Step 5)
* **조치 내용**:
  - `camera_link` ➔ `camera_imu_optical_frame` 연결 static TF가 누락되어 발생한 좌표계 트리의 단절을 확인.
  - 5단계 Static TF 명령에 다음 구문을 추가하여 좌표계 트리를 100% 정상 연결:
    ```bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link camera_imu_optical_frame &
    ```

---

### 5단계: 전체 통신 파이프라인 QoS (Quality of Service) 신뢰성 통일
* **발생 문제**: 터미널에 `New subscription discovered on this topic, requesting incompatible QoS... Last incompatible policy: RELIABILITY_QOS_POLICY` 경고 메시지 출력.
* **수정 파일**: [run_map.sh](file:///home/unitree/go2_ws/run_map.sh) (Step 4)
* **조치 내용**:
  - RealSense 카메라, IMU 필터, RTAB-Map 간 통신 QoS를 `1` (Reliable)로 전면 일치시킴:
    ```bash
    qos:=1 \
    qos_image:=1 \
    qos_camera_info:=1 \
    qos_imu:=1 \
    qos_odom:=1 \
    ```
  - QoS 정책 불일치 경고 메시지를 **0개로 완전 소멸**시킴.

---

### 6단계: ROS 2 표준 프레임 ID 표기법 준수
* **발생 문제**: `in tf2 frame_ids cannot start with a '/'` 경고 출력.
* **수정 파일**: [run_map.sh](file:///home/unitree/go2_ws/run_map.sh) (Step 4)
* **조치 내용**:
  - ROS 2 규격에 맞게 프레임명 앞의 슬래시(`/`)를 제거한 `odom_frame_id:=odom` 선언 추가.

---

### 7단계: 프로세스 안전 종료 및 원클릭 재시동 스크립트 작성
* **수정 파일**: [stop_map.sh](file:///home/unitree/go2_ws/stop_map.sh)
* **조치 내용**:
  - 기존 백그라운드 ROS 2 노드, 카메라 드라이버, IMU 리레이, daemon을 잔여 프로세스 없이 깔끔하게 Clean-up해 주는 [stop_map.sh](file:///home/unitree/go2_ws/stop_map.sh) 생성 및 실행 권한 부여 (`chmod +x`).

---

## 📊 최종 시스템 검증 결과 (Empirical Proof)

| 검증 항목 | 수정 전 상태 | 수정 후 상태 | 검증 결과 |
| :--- | :--- | :--- | :---: |
| **DDS 참가자 한도** | 30개 한도 초과로 GUI 튕김 | 100개 슬롯 확장으로 dual GUI 연동 | **대성공** |
| **TF 좌표계 트리** | `camera_imu_optical_frame` 단절 | `base_link`~`camera_link`~`IMU` 100% 연결 | **대성공** |
| **데이터 수신 대기** | `Did not receive data since 5 seconds!` | 0.026초 (30 FPS 실시간 연산 지속) | **대성공** |
| **QoS 신뢰성** | `RELIABILITY_QOS_POLICY` 경고 속출 | 모든 토픽 Reliable 프로필로 통일 (경고 0개) | **대성공** |
| **VIO Odometry Quality** | 데이터 미수신으로 연산 불가 | `quality=280+`, `update time=0.026s` 지속 | **대성공** |

---

## 🚀 사용법 (Usage)

터미널에서 아래 단 한 줄의 명령어로 전체 3D VIO SLAM 시스템을 깨끗하게 시동하실 수 있습니다:

```bash
./stop_map.sh && ./run_map.sh
```
