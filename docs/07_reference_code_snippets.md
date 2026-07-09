# 🔗 유니트리 공식 SDK 및 레퍼런스 오픈소스 코드 모음 (Reference Code Snippets)

본 문서는 실물 로봇 연동 및 백업 스크립트 작성 시 코드 강건성(Robustness)을 확보하기 위해, 유니트리 공식 SDK 2 및 오픈소스 저장소들의 핵심 제어 및 센서 수신 예제 코드를 아카이빙함.

---

## 📌 1. Unitree SDK2 Python High-Level 모션 제어 (SportClient)

로봇에게 걷기, 서기, 눕기 등의 고차원 모션 명령을 내리는 공식 C++ / Python SDK 사용 뼈대 코드.

```python
import time
import sys
from unitree_sdk2.common.channel import ChannelFactoryInitialize
from unitree_sdk2.go2.sport.sport_client import SportClient

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"사용법: python3 {sys.argv[0]} <네트워크_인터페이스_이름 (예: eth0)>")
        sys.exit(1)

    # 1. DDS 통신 채널 초기화 (젯슨의 유선 포트 지정 필수)
    ChannelFactoryInitialize(0, sys.argv[1])

    # 2. SportClient 생성 및 초기화
    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()

    print("로봇 스탠드업 시작...")
    client.StandUp()
    time.sleep(3.0)

    print("전진 주행 시작 (선속도: 0.1 m/s, 각속도: 0.0 rad/s)...")
    # Move(vx, vy, vyaw)
    client.Move(0.1, 0.0, 0.0)
    time.sleep(5.0)

    print("제자리 정지...")
    client.Move(0.0, 0.0, 0.0)
    time.sleep(2.0)

    print("안전 감쇠 착지 (Damp)...")
    client.Damp()
```

---

## 📌 2. Unitree SDK2 Python 로봇 상태 및 센서 데이터 수신 (SportStateClient)

로봇 내부 보드로부터 현재 보행 속도, IMU 가속도, 다리 상태, 에러 코드 등을 실시간 수신하여 파싱하는 뼈대 코드.

```python
import sys
import time
from unitree_sdk2.common.channel import ChannelFactoryInitialize
from unitree_sdk2.go2.sport.sport_state_client import SportStateClient

# 상태 메시지 수신 시 처리할 콜백 함수
def sport_state_callback(msg):
    if msg is None:
        return
    # IMU 쿼터니언 (자세 각도) 출력
    print(f"IMU Orientation [x, y, z, w]: {msg.imu_state.quaternion}")
    # 현재 측정 속도 출력 (vx, vy, vz)
    print(f"Current Velocity [x, y, z]: {msg.velocity}")
    # 현재 로봇 위치 오도메트리 출력
    print(f"Position [x, y, z]: {msg.position}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"사용법: python3 {sys.argv[0]} <네트워크_인터페이스_이름>")
        sys.exit(1)

    ChannelFactoryInitialize(0, sys.argv[1])

    # SportState 수신 클라이언트 생성
    state_client = SportStateClient()
    state_client.SetTimeout(5.0)
    state_client.Init()

    # 콜백 등록하여 실시간 상태 데이터 모니터링
    state_client.RegistStateCallback(sport_state_callback)

    print("로봇 상태 실시간 모니터링 가동 중... (종료: Ctrl + C)")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("모니터링 종료.")
```

---

## 📌 3. 공식 ROS 2 Wrapper (`go2_driver.cpp`) 속도 제어 변환 구조 (C++)

실표준 드라이버가 ROS 2 `/cmd_vel`을 수신해 Unitree Sport API로 변환하여 DDS 채널로 송출하는 직렬화 코드 핵심 구조.

```cpp
// go2_driver.cpp 내부 cmd_vel 콜백 핸들러 원형 예시
void Go2Driver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 1. ROS 2 속도 데이터 파싱
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float vyaw = msg->angular.z;

    // 2. 가용 스피드 리밋 검증 (Saturation)
    vx = std::max(-max_vx_, std::min(max_vx_, vx));
    vy = std::max(-max_vy_, std::min(max_vy_, vy));
    vyaw = std::max(-max_vyaw_, std::min(max_vyaw_, vyaw));

    // 3. JSON 규격 직렬화 구조 생성 (SportClient 내부 DDS 토픽 매핑 구조)
    // apiId = 1002 (Move API ID)
    std::string parameter = "{\"x\":" + std::to_string(vx) + 
                            ",\"y\":" + std::to_string(vy) + 
                            ",\"z\":" + std::to_string(vyaw) + "}";

    // 4. 로봇 내부 DDS 채널로 발행 요청 전송
    unitree_api::msg::dds_::Request request;
    request.header().identity().id() = 1002; // Move 명령 고유 식별 ID
    request.parameter() = parameter;

    request_publisher_->Write(request);
}
```
