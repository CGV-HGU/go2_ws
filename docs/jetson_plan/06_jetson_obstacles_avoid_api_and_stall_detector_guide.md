# 🛡️ [Jetson 06] Unitree Go2 장애물 회피 API(ObstaclesAvoidClient) 및 정체 감지 가이드

> **문서 번호**: `Jetson-06`  
> **문서 위치**: `docs/jetson_plan/06_jetson_obstacles_avoid_api_and_stall_detector_guide.md`  
> **대상 플랫폼**: Unitree Go2 EDU Plus (Jetson Orin NX 16GB / Ubuntu 20.04 / ROS 2 Foxy)  
> **상위 연계**: [`docs/master_plan/README.md`](../master_plan/README.md), [`docs/jetson_plan/README.md`](README.md)  
> **작성 일자**: 2026-08-24 (KST)

---

## 📌 1. 개요 및 아키텍처 역할 분담 (R&R)

Unitree Go2 사족보행 로봇의 자율주행(ESCAPE-Nav) 실증 시, 막다른 길(Dead-end)이나 전방 장애물 봉착 시 로봇의 장애물 회피 모드 제어 및 물리적 정체 감지는 **Tier 2 (Jetson Host OS)** 계층에서 전담합니다.

```mermaid
graph TD
    subgraph "Tier 1: Go2 Robot Hardware (192.168.123.161)"
        LIDAR["4D LiDAR Avoidance Engine (obstacles_avoid)"]
        SPORT["Sport Mode Controller (sport_mode)"]
    end

    subgraph "Tier 2: Jetson Host OS (docs/jetson_plan/)"
        OA_CLI["ObstaclesAvoidClient (C++)<br/>• SwitchSet(bool) [1001]<br/>• SwitchGet(bool&) [1002]<br/>• Move(vx, vy, yaw) [1003]"]
        SP_CLI["SportClient (C++)<br/>• FreeAvoid(bool) [2048]<br/>• SwitchAvoidMode() [2058]"]
        STALL["Kinematic Stall Detector<br/>• |v_cmd| ≥ 0.15 & |v_odom| ≤ 0.03"]
        H_BRG["host_bridge.py (Port 9091)"]
    end

    subgraph "Tier 3: Docker Sandbox (docs/docker_plan/)"
        D_BRG["docker_bridge.py (Port 9091 수신)"]
        S2E["S2E Navigation (vlm_s2e_async_node.py)<br/>• Active-View Recovery (360° 선회 기동)"]
    end

    LIDAR <-->|CycloneDDS: /api/obstacles_avoid/| OA_CLI
    SPORT <-->|CycloneDDS: /api/sport/| SP_CLI
    OA_CLI & SP_CLI & STALL --> H_BRG
    H_BRG -->|UDP Packet (63B / <0.1ms)| D_BRG
    D_BRG --> S2E
```

* **Jetson Host OS의 역할**:
  1. Unitree C++ SDK2(`libunitree_sdk2.a`)를 직접 링크하여 `ObstaclesAvoidClient` 및 `SportClient` API를 로봇 메인보드로 전송.
  2. 50Hz 하드웨어 융합 오도메트리(`/odom` 또는 `/rtabmap/odom`)와 상위 명령 속도(`/cmd_vel`)를 비교하여 **운동학적 정체(Kinematic Stall)** 실시간 판정.
  3. 충돌/정체 플래그를 도커(Docker) S2E 자율주행 노드로 초고속(<0.1ms) UDP 전송.
* **Docker Container의 역할**:
  1. 전달받은 장애물/충돌 플래그를 기반으로 **Active-View Recovery(360° 요 스윕)** 및 VLM 위상 메모리 그래프 상의 실패 에지(Dead-end) 가중치 페널티 부여 후 반대 방향 탈출 경로 재계획.

---

## 🔌 2. Unitree SDK2 공식 장애물 회피 API 명세

### 1) 전용 회피 클라이언트 (`ObstaclesAvoidClient`)
* **헤더 파일**: `<unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>` & `obstacles_avoid_api.hpp`
* **DDS 서비스명**: `obstacles_avoid` (요청: `/api/obstacles_avoid/request`, 응답: `/api/obstacles_avoid/response`)

| API ID | 메서드 이름 | 파라미터 | 상세 기능 |
| :--- | :--- | :--- | :--- |
| **`1001`** | `SwitchSet(bool enable)` | `enable: true/false` | 라이다 기반 자율 장애물 회피 기능 강제 On/Off |
| **`1002`** | `SwitchGet(bool& enable)` | `enable` (결과 반환) | 현재 장애물 회피 활성화 여부 조회 |
| **`1003`** | `Move(float x, float y, float yaw)` | `x, y, yaw, mode` | 장애물 회피가 적용된 주행 명령<br/>• Mode 0: 속도 제어(`vel`)<br/>• Mode 1: 증분 좌표(`increment pose`)<br/>• Mode 2: 절대 좌표(`absolute pose`) |
| **`1004`** | `UseRemoteCommandFromApi(bool flag)` | `flag: true/false` | 조종기 대신 API를 통한 원격 회피 명령 우선권 설정 |
| **-** | `MoveToAbsolutePosition(float x, y, yaw)` | `x, y, yaw` | 장애물을 우회하며 절대 좌표로 이동 |
| **-** | `MoveToIncrementPosition(float x, y, yaw)` | `x, y, yaw` | 장애물을 우회하며 상대 증분 좌표로 이동 |

### 2) 스포츠 모드 회피 토글 (`SportClient`)
* **헤더 파일**: `<unitree/robot/go2/sport/sport_client.hpp>` & `sport_api.hpp`
* **DDS 서비스명**: `sport`

| API ID | 메서드 이름 | 파라미터 | 상세 기능 |
| :--- | :--- | :--- | :--- |
| **`2048`** | `FreeAvoid(bool flag)` | `flag: true/false` | AI 강화학습 기반 프리 회피(Free Avoid) 보행 모드 On/Off |
| **`2058`** | `SwitchAvoidMode()` | 없음 | 스포츠 모드 내 회피 모드 상태 On ↔ Off 토글 |

---

## ⚙️ 3. Jetson 상보적 2중 방어: 운동학적 정체 감지 (Kinematic Stall Detector)

로봇이 물리적 유리벽, 낮은 턱, 또는 좁은 코너에 끼여 라이다 감지 전에 멈춰 서는 경우를 100% 방어하기 위해 호스트 OS에서 운동학적 정체 감지기를 상시 가동합니다:

$$\text{Stall Condition} = \left( |v_{\text{cmd}, x}| \ge 0.15\text{ m/s} \right) \;\land\; \left( |v_{\text{odom}, x}| \le 0.03\text{ m/s} \right) \;\land\; \left( \Delta t_{\text{stuck}} \ge 0.4\text{ s} \right)$$

---

## 💻 4. Jetson 온보드 C++ 및 Python 구현 코드

### 1) C++ SDK2 제어 예제
```cpp
#include <iostream>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

int main(int argc, char** argv) {
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    
    unitree::robot::go2::ObstaclesAvoidClient oa_client;
    oa_client.Init();

    // 1. 회피 모드 켜기
    oa_client.SwitchSet(true);

    // 2. 장애물을 우회하며 전진
    oa_client.Move(0.3f, 0.0f, 0.0f);

    return 0;
}
```

### 2) Host Bridge 내장형 정체 감지 Python 노드
```python
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class JetsonObstacleStallDetector(Node):
    def __init__(self):
        super().__init__('jetson_obstacle_stall_detector')
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_cb, 10)
        self.collision_pub = self.create_publisher(Bool, '/robot/collision_detected', 10)
        
        self.last_cmd_vx = 0.0
        self.stuck_start_time = None
        self.is_collided = False
        self.timer = self.create_timer(0.05, self.check_stall)

    def cmd_cb(self, msg: Twist):
        self.last_cmd_vx = msg.linear.x

    def odom_cb(self, msg: Odometry):
        curr_vx = msg.twist.twist.linear.x
        if abs(self.last_cmd_vx) >= 0.15 and abs(curr_vx) <= 0.03:
            if self.stuck_start_time is None:
                self.stuck_start_time = time.time()
            elif (time.time() - self.stuck_start_time) >= 0.4:
                self.is_collided = True
        else:
            self.stuck_start_time = None
            self.is_collided = False

    def check_stall(self):
        msg = Bool()
        msg.data = self.is_collided
        self.collision_pub.publish(msg)
```

---

## 📦 5. Host ↔ Docker UDP 패킷 구조 (Port 9091: 63B)

* `[0..3]`: Magic Header `0x53324501` (`'S2E\x01'`)
* `[4..59]`: 7 double floats (`x, y, z, qx, qy, qz, qw` = 56B)
* `[60]`: `collision_flag` (1 Byte: `0=Normal`, `1=Avoidance Active`, `2=Stalled`)
* `[61..62]`: CRC16 / Checksum (2B)
