# QoS 매칭 이슈 해결을 위한 수정 제안 가이드

이 문서에는 `run_map_default.sh` 실행 시 발생하는 IMU 데이터 대기(QoS 불일치) 현상을 해결하기 위해 적용할 수 있는 구체적인 코드 수정 방안들이 정리되어 있습니다. 

---

## 📌 옵션 1: `run_map_default.sh` 수정 (추천)
카메라 구동 시 강제로 설정한 `SYSTEM_DEFAULT` QoS 옵션을 제거하여 `imu_relay.py`와 통신 속성(`BEST_EFFORT`)을 매칭시킵니다.

### 대상 파일
* [run_map_default.sh](file:///home/unitree/go2_ws/run_map_default.sh)

### 수정 코드 (Diff)
```diff
<<<<
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=1 \
    accel_qos:=SYSTEM_DEFAULT \
    gyro_qos:=SYSTEM_DEFAULT; exec bash"
====
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=1; exec bash"
>>>>
```

---

## 📌 옵션 2: `imu_relay.py` 수정
카메라가 어떤 QoS 옵션으로 데이터를 보내더라도 수신할 수 있도록 릴레이 노드의 구독 속성을 변경합니다.

### 대상 파일
* [imu_relay.py](file:///home/unitree/go2_ws/imu_relay.py)

### 수정 코드 (Diff)
```diff
<<<<
        # Use best_effort QoS to match camera driver publisher
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
====
        # Use system_default/reliable QoS to match camera driver publisher
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
>>>>
```
*(참고: ROS 2 Foxy에서 발행자가 RELIABLE인 경우, 구독자도 RELIABLE 혹은 SYSTEM_DEFAULT여야 데이터 수신이 가능합니다.)*
