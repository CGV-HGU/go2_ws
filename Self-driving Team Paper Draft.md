# Self-driving Team Paper Draft

# VL-MAG: A Vision-Language Memory-Action Graph for Asynchronous Robot Navigation

# 1\. Introduction

미지 환경에서 coarse position goal 또는 semantic object goal을 향해 이동하는 능력은 원격 탐사, 물류, 재난 대응, 생활 지원 로봇을 포함한 다양한 자율주행 응용의 핵심이다. 전통적인 navigation system은 일반적으로 사전에 구축하거나 주행 중 생성한 occupancy 또는 cost map 위에서 global path를 계획하고, 이를 여러 개의 local subgoal로 나누어 로봇이 추종하도록 한다. 이러한 방식은 충분히 정확한 지도와 localization이 제공되는 환경에서는 안정적으로 동작한다.

그러나 실제 미지 환경에서는 두 가지 어려움이 발생한다. 첫째, 지도 기반 navigation은 로봇의 현재 위치와 지도 좌표계가 정확하게 정렬되어 있다는 가정을 필요로 한다. 이를 위해서는 고정밀 GNSS, 외부 infrastructure, 사전 구축 지도, 또는 별도의 visual·LiDAR localization system이 필요하다. 하지만 처음 방문하는 환경이나 시각적·기하학적 특징이 부족한 장소에서는 이러한 localization의 정확도가 쉽게 저하될 수 있다. 설원이나 반복적인 복도처럼 뚜렷한 landmark가 부족한 환경은 이러한 문제를 보여주는 대표적인 예이다.

둘째, global planner가 제시한 subgoal이 실제 로봇의 현재 시야에서 주행 가능한 위치라는 보장은 없다. 지도에서는 free space로 표현된 위치라도 실제 환경에는 작은 장애물, 일시적으로 놓인 물체, 지도 생성 이후 추가된 구조물 또는 센서가 관측하지 못한 위험 요소가 존재할 수 있다. 또한 noisy GNSS나 odometry를 사용할 경우, 지도와 로봇 좌표계 사이의 오차로 인해 planner가 선택한 subgoal이 실제로는 장애물 위나 주행할 수 없는 영역에 놓일 수 있다. 따라서 coarse goal을 단순히 geometric coordinate로 전달하는 것을 넘어, 현재 관측을 바탕으로 실제로 주행 가능한 local goal로 변환하는 과정이 필요하다.

본 연구는 이러한 문제를 고려하여, **미지의 약한 구조 환경에서 monocular RGB와 local odometry만을 사용해 coarse point goal 또는 semantic object goal을 추종하는 dense-map-free zero-shot navigation 문제**를 다룬다. 제안 방법은 depth-derived occupancy map, voxel map, point cloud 또는 globally aligned 3D semantic map을 생성하지 않는다. 대신 로봇의 기본적인 trajectory tracking과 상대 이동 추정을 위해 local odometry를 사용하며, 현재 RGB observation을 기반으로 실제 주행 가능한 local subgoal을 선택한다.

최근 Vision-Language Model은 장면 내 객체, 공간 구조, affordance, room semantics를 이해할 수 있기 때문에 미지 환경에서의 navigation decision maker로 주목받고 있다. VLM은 coarse goal과 현재 영상을 함께 해석하여 목표와 관련된 doorway, corridor 또는 traversable floor를 찾고, 이를 local waypoint로 변환할 수 있다. 이러한 능력은 사전에 학습된 환경별 semantic map 없이도 새로운 object나 scene에 대해 zero-shot navigation을 수행할 가능성을 제공한다.

그러나 VLM을 실제 robot navigation loop에 직접 사용하는 경우에는 여전히 두 가지 구조적인 문제가 존재한다.

첫 번째 문제는 **VLM의 inference latency**이다. 대규모 VLM은 local navigation policy에 비해 추론 시간이 길다. VLM의 출력을 기다리는 동안 로봇이 정지하는 synchronous system에서는 반복적인 stop-and-go motion이 발생하며, 실제 주행 속도가 크게 감소할 수 있다. 반대로 VLM 추론 중에도 로봇이 계속 움직이면, VLM이 판단에 사용한 영상과 결과가 실제로 적용되는 시점의 robot state가 달라지는 temporal mismatch가 발생한다. 이러한 지연은 동적 장애물이나 갑작스러운 local collision risk에 즉각적으로 대응해야 하는 실제 로봇 시스템에서 특히 문제가 된다.

두 번째 문제는 **지속적인 navigation memory의 부족**이다. VLM이 현재 scene을 이해하더라도, 이전에 어떤 방향으로 이동하다가 실패했는지, 어느 공간이 막다른 길이었는지, 어떤 방향으로 탈출했는지를 명시적으로 기억하지 못하면 동일한 실패를 반복할 수 있다. 예를 들어 로봇이 막다른 방에서 빠져나온 뒤 다시 같은 doorway로 진입하거나, 두 개의 유사한 복도 사이를 반복적으로 왕복하는 oscillation에 빠질 수 있다. 단순히 최근 이미지 몇 장이나 action history를 VLM prompt에 추가하는 것만으로는 장소 간 관계, 이동 방향, 실패 원인, 탈출 경로를 일관되게 유지하기 어렵다.

본 연구는 이러한 두 문제를 함께 해결하기 위해 **Vision-Language Memory-Action Graph, VL-MAG**를 제안한다. VL-MAG는 저주기로 동작하는 VLM semantic supervisor와 고주기로 동작하는 end-to-end trajectory policy를 결합한 hierarchical asynchronous navigation framework이다. VLM은 직접 wheel velocity나 joint action을 생성하지 않는다. 대신 coarse goal, 현재 RGB, local odometry, 그리고 과거 navigation memory를 이용하여 fine subgoal 또는 high-level intervention을 생성한다. 빠른 end-to-end module은 VLM이 생성한 최신 subgoal을 따라 short-horizon trajectory를 반복적으로 생성하고, robot-specific PID 또는 locomotion controller가 local odometry를 사용해 해당 trajectory를 추종한다.

이 구조에서는 VLM이 추론하는 동안에도 fast controller가 이전에 검증된 subgoal을 계속 추종할 수 있다. 따라서 로봇의 local control frequency는 VLM의 inference latency에 직접적으로 제한되지 않는다. 새로운 VLM output이 도착하면 timestamp, robot displacement, command validity를 검사한 뒤 안전한 경우에만 최신 subgoal로 교체한다. 이를 통해 느린 semantic reasoning과 빠른 collision avoidance 및 trajectory control을 서로 다른 주기로 실행할 수 있다.

VL-MAG의 핵심 memory는 로봇이 실제로 방문한 장소와 이동 결과를 저장하는 sparse relative-pose graph이다. 각 node는 특정 장소에서 획득한 keyframe, semantic description, 방문 횟수, deadlock 상태와 같은 정보를 저장한다. 두 node 사이의 directed edge는 global position이 아니라 odometry로 측정된 relative motion을 저장하며, 해당 transition의 결과를 함께 기록한다.

예를 들어 복도에서 방으로 진입한 뒤 막혔다면 해당 edge는 `deadlock_entry`로 저장된다. 이후 방에서 복도로 되돌아오는 데 성공했다면 반대 방향 edge는 `escape_success`로 저장된다. 따라서 VL-MAG는 특정 방 전체를 무조건 위험한 장소로 표시하지 않고, **어느 방향으로 진입했을 때 실패했으며 어느 방향으로 이동했을 때 탈출했는지**를 구분할 수 있다. 이러한 directional memory는 로봇이 동일한 실패 branch로 다시 진입하는 것을 방지하는 데 사용된다.

VL-MAG는 memory를 단순히 과거 정보를 검색하는 수동적인 저장소로 사용하지 않는다. Graph에 저장된 장소와 transition은 다음 action을 선택하는 데 직접 사용된다. VLM이 현재 영상에서 안전한 주행 영역을 확인하면 image-space 또는 robot-relative fine subgoal을 생성한다. 반대로 현재 시야에 주행 가능한 영역이 충분히 보이지 않거나, deadlock 또는 oscillation이 의심되는 경우에는 fine goal을 억지로 생성하지 않고 `rotate` 또는 `request_observation`을 요청한다.

이 과정은 사람이 길을 잃었을 때 주변을 둘러보고 새로운 출구를 찾는 행동과 유사하다. 로봇은 특정 방향을 향해 회전하거나, goal이 있을 가능성이 높은 방향을 중심으로 여러 장의 RGB observation을 추가로 획득한다. VLM은 새 observation과 graph memory를 함께 검토하여 이전에 실패하지 않은 doorway, corridor 또는 free-space branch를 선택하고, 이를 fast controller가 추종할 수 있는 fine subgoal로 변환한다.

VL-MAG는 revisit 시 발생할 수 있는 잘못된 place association도 보수적으로 처리한다. 현재 영상이 과거 장소와 유사하더라도, 반복적인 복도나 비슷한 구조의 다른 방일 수 있다. 따라서 visual similarity만으로 node를 병합하지 않고, graph를 통해 계산한 relative pose, baseline, odometry uncertainty, semantic structure를 함께 검증한다. 같은 장소라는 확신이 충분하지 않은 경우에는 중복 node를 허용하며, 잘못된 merge로 graph 전체가 오염되는 것을 우선적으로 방지한다. 같은 장소로 판단된 경우에도 기존 node를 제거하지 않고 두 node 사이에 same-place soft link를 추가하여 서로 다른 관측 위치와 parallax를 보존한다.

또한 VL-MAG는 특정 VLM 또는 특정 robot action space에 직접 종속되지 않는 structured interface를 사용한다. VLM은 `go`, `rotate`, `request_observation`, `stop` 중 하나의 high-level action과 fine subgoal을 출력하고, backend는 출력 schema와 memory consistency를 검증한 후 실행한다. 따라서 같은 high-level memory system을 PixNav과 같은 pixel-goal navigation skill 또는 S2E와 같은 trajectory-generating policy에 연결할 수 있다. 이때 training-free라는 표현은 전체 navigation system이 아니라, **pretrained VLM을 navigation task에 맞게 추가 fine-tuning하지 않고 memory와 action schema를 통해 연결하는 VL-MAG interface**에 해당한다.

실험에서는 먼저 Habitat PointNav 및 ObjectNav 환경에서 VL-MAG가 reactive VLM과 flat-history baseline에 비해 deadlock과 oscillation을 더 안정적으로 해결하고, 탈출 이후에도 동일 failed branch로 재진입하지 않으면서 원래 목표를 향한 navigation을 재개할 수 있음을 검증한다. 이어서 NavBench-GS에서는 S2E를 fast trajectory policy로 사용하여, synchronous 방식과 asynchronous 방식의 성능을 비교한다. 특히 인위적으로 VLM latency를 증가시키는 실험을 통해, 제안하는 asynchronous system이 VLM 지연 상황에서도 높은 trajectory update frequency와 navigation success를 유지할 수 있음을 보인다. 마지막으로 실제 quadruped robot에 시스템을 적용하여 dead-end room, 반복적인 corridor, blocked goal direction 및 동적 장애물 시나리오에서 제안 방법의 실시간 동작과 recovery 성능을 평가한다.

---

본 논문의 주요 기여는 다음과 같다.

1. **실행 가능한 relative-pose episodic memory**Depth-derived dense map을 생성하지 않고, monocular RGB와 local odometry를 이용하여 방문 장소, relative motion, directional failure, escape transition 및 revisit evidence를 저장하는 sparse graph memory를 제안한다.
2. **Memory 기반 active observation 및 fine-subgoal generation**VLM이 현재 관측만으로 안전한 주행 영역을 선택하기 어렵다고 판단하면 추가 yaw observation이나 robot rotation을 요청하고, 새 관측과 과거 실패 이력을 이용해 실제로 실행 가능한 fine subgoal을 생성하는 structured memory-action interface를 제안한다.
3. **느린 VLM과 빠른 trajectory policy의 비동기 결합**VLM은 저주기로 semantic goal과 intervention을 생성하고, fast end-to-end policy는 최신의 검증된 subgoal을 지속적으로 추종한다. 이를 통해 VLM inference latency를 실시간 local control loop로부터 분리한다.
4. **Deadlock과 oscillation 중심의 강건성 평가**Habitat의 PointNav/ObjectNav, NavBench-GS, 실제 quadruped robot에서 standard SR 및 SPL뿐 아니라 Stable Escape Rate, Failed-Branch Re-entry Rate, Oscillation Burden, Goal Resume Rate, VLM latency 및 robot idle time을 평가한다.

# 4\. Experiments

#### 실험 환경

- PC: pro 6000
- Robot: Unitree Go2, ROS
- VLM : Qwenv3 VL instruct 32B

## Benchmark

### Habitat

#### objectNav

- problem definition
  - unseen environment에서 category로 지정된 object instance를 찾아 navigation
- Dataset
  - HM3D ObjectNav v1 또는 v2
- Input
  - monocular RGB
  - target object category
  - local odometry
- Metric
  - Success Rate (SR)
  - Success weighted by Path Length (SPL)
  - SoftSPL (SPL에 실패 episode의 final progress까지 반영)
  - Collision Density
- Comparison Methods (자세한 수식은 [backup](https://docs.nb.hsl.ee/s/I27/p/backup-ME2rC5q7fM))
  - Sensor-matched RGB baselines
    - VLM + PixNav
    - ZSON
    - VLMnav
    - Ours
  - Richer sensing references (RGB + depth/dense map)
    - VLFM
    - SG-Nav
    - UniGoal
- Ablation study
  - PixNav 고정

| ID  | Method |
| --- | --- |
| A1  | VLM + PixNav |
| A2  | VL-MAG without active observation |
| A3  | VL-MAG without directional failure/escape edges |
| A4  | VL-MAG without spatial revisit gate |
| A5  | Full VL-MAG v6 + PixNav |

- S2E 고정

| ID  | Method |
| --- | --- |
| B1  | VLM + S2E Sync |
| B2  | VLM + S2E Async |
| B3  | Full VL-MAG + S2E Sync |
| B4  | Full VL-MAG + S2E Async |

- Ours full로 두 controller만 비교
  - VL-MAG + PixNav
  - VL-MAG + S2E
- 추가 ablation
  - VLM parameter size

#### Habitat PointNav Deadlock Challenge

PointNav는 clean benchmark에서 VLM semantic reasoning의 필요성이 약할 수 있으므로, controlled recovery evaluation에 사용한다. 따라서 다음 scenario를 미리 annotation한다.

- Controlled scenario
  - single-exit dead-end room
  - exit behind initial camera
  - goal-aligned but blocked branch
  - repetitive A–B corridor
  - deceptive side room
  - backtracking-required path
  - occluded junction
- Metric (자세한 수식은 [backup](https://docs.nb.hsl.ee/s/I27/p/backup-ME2rC5q7fM))
  - Stable Escape Rate
  - Failed-Branch Re-entry Rate
  - Oscillation Burden
  - Time-to-Recovery
  - Goal Resume Rate
  - Recovery-to-Goal Success
- Methods
  - ObjectNav와 동일
- 추가 실험
  - Memory Quality
  - Async efficiency
  - Odometry Robustness (noise condition)

#### NavBench-GS

graph memory 자체보다 async execution과 dynamic safety를 검증

- Metric
  - Habitat ObjectNav와 동일
- Comparison with other methods
  - Methods
    - ZeroPolicy
    - GNM
    - NiNT
    - NoMaD
    - CityWalker
    - S2E-BC
    - S2E-RL
    - Ours

#### Real Robot

- 다음의 Scenario를 만들어서 실험 진행
  - dead-end room with hidden exit
  - repetitive corridor with similar doorways
  - goal-aligned blocked branch
  - dynamic pedestrian or moving obstacle
  - long-horizon route with delayed VLM
- 시나리오 장소에 대해 RTABMAP 기반 mapping & localization알고리즘 확보
  - goal 지정 및 robot initial position align을 위해 필요
- Metric은 기본적으로 SR을 사용하되, 가능한 metric 확보