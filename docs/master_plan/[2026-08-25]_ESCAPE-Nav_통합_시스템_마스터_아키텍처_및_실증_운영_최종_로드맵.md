# 🏛️ [Master Plan] ESCAPE-Nav 통합 시스템 마스터 아키텍처 및 실증 운영 최종 로드맵

> **작성 일자**: 2026년 8월 25일 (KST)  
> **시스템 총괄**: **Antigravity Master Plan Architect** & **민석 (Hardware, Sensor & Deployment Lead)**  
> **논문 공식 명칭**: **`ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation` (ICRA 2026)**  
> **대상 플랫폼**: Unitree Go2 EDU Plus (신형 **Unitree 4D LiDAR L2** + 전면 카메라 + Jetson Orin NX 16GB + Docker Jazzy + Qwen3-VL Server)  
> **문서 목적**: 4-Tier 전 계층의 하드웨어 정합화, VLM 실시간 자동 감지, 이중 계층 모터 구동, 정체 감지 및 안전 인터록 검증, ICRA 2026 20회 실증 벤치마크를 총망라한 최종 마스터 아키텍처 및 운영 로드맵입니다.

---

## 📌 1. 4-Tier 통합 시스템 토폴로지 (Single Source of Truth)

```mermaid
graph TD
    subgraph "Tier 1: Robot Hardware (Unitree Go2 EDU Plus)"
        L2["신형 Unitree 4D LiDAR L2 (192.168.1.62:6101)<br/>• 360°×90° 반구형 시야각, 15Hz 점군 (/pointcloud)"]
        CAM["전면 초광각 카메라 (230.1.1.1:1720)<br/>• H.264 RTP 멀티캐스트, 30fps (/camera/front/image_raw)"]
        MCU["메인보드 MCU (192.168.123.161)<br/>• 하드웨어 1000Hz 밸런싱 & 50Hz 오도메트리 (/odom)<br/>• SportClient.Move (API ID 1008) 제어 수신"]
    end

    subgraph "Tier 2: Jetson Host OS (docs/jetson_plan/)"
        J_NET["eth0: 192.168.123.99 (alias: 192.168.1.2/24)<br/>• CycloneDDS cyclonedds.xml 피어 바인딩<br/>• 헤드리스 자동 로그인 & NetBird VPN 부팅"]
        J_LIVO["RTAB-Map LIVO 50Hz SLAM<br/>• Reg/Strategy: 1 (ICP 점군 매칭)<br/>• Reg/Force3DoF: true & Optimizer/Slam2D: true (2D 평면 구속)<br/>• cloud_voxel_size: 0.05 (점군 70% 경량화)"]
        J_BRG["host_bridge.py (이중 계층 모터 구동)<br/>• Layer 1: ROS 2 /cmd_vel<br/>• Layer 2: CycloneDDS /api/sport/request (API 1008)<br/>• 0.5s 워치독 타이머 & 0.35m/s 속도 리미터"]
        J_STALL["Kinematic Stall Detector (63B UDP 송신)<br/>• |v_cmd| ≥ 0.15 & |v_odom| ≤ 0.03 (0.4s 지속 시)"]
    end

    subgraph "Tier 3: Docker Sandbox (docs/docker_plan/)"
        D_CONT["sdam_go2_container (network_mode: host, ipc: host)"]
        D_BRG["docker_bridge.py<br/>• UDP 9091 수신 (Pose) / 9090 송신 (Twist)<br/>• Magic Header 0x53324501 + CRC16 (지연 37.8µs)"]
        D_S2E["vlm_s2e_async_node.py<br/>• Causal Pose Warping (Δt = 300~800ms 지연 보상)<br/>• PointNav 2중 Stop Guard (r ≤ 0.5m 자동 정지)<br/>• Active-View Recovery (360° 요 스윕 & 위상 그래프 회복)"]
        D_ZERO["VLM Auto-Discovery (Zero-Config)<br/>• GET /v1/models ➔ 활성 서빙 모델 자동 바인딩"]
    end

    subgraph "Tier 4: Remote GPU VLM Server (100.96.60.15:8000)"
        S_VPN["NetBird P2P VPN Direct Tunnel (RTT 14ms)"]
        S_VLLM["vLLM / SGLang Engine<br/>• Qwen3.8-27B / Qwen3.5-9B / Qwen3-VL-32B<br/>• /v1/models (모델 목록) & /v1/chat/completions (비동기 추론)"]
    end

    L2 & CAM & MCU <-->|UDP / RTP / DDS| J_NET
    J_NET --> J_LIVO & J_STALL
    J_LIVO --> J_BRG
    J_BRG <-->|127.0.0.1 Loopback| D_BRG
    D_BRG <--> D_S2E
    D_S2E <--> D_ZERO
    D_ZERO <-->|HTTP REST / 14ms| S_VPN --> S_VLLM
```

---

## 🧭 2. 젯슨 AGY 및 도커 AGY 핵심 운영 수칙

1. **하드웨어 단일 기준**: 탑재 라이다는 **신형 Unitree 4D LiDAR L2**이며, 수신 토픽은 **`/pointcloud` (15Hz)**, 오도메트리는 **`/odom` (50Hz)**로 고정.
2. **이중 계층 모터 구동**: `host_bridge.py`가 `/cmd_vel`과 CycloneDDS `/api/sport/request` (API ID 1008)를 동시 발행.
3. **무설정 VLM 모델 자동 감지**: `vlm_client.py`가 `GET /v1/models`를 통해 서빙 모델 실시간 자동 바인딩.
4. **포복 대기 안전 인터록**: 로봇 전원 인가 후 리모컨 포복(Prone) 와상 상태에서 모터 $0.0\text{ m/s}$ 안전 인터록 상태로 사전 궤적 검증 수행.
5. **초고속 트러블슈팅**: 에러 발생 시 [`docs/troubleshooting/QUICK_ERROR_LOOKUP_AND_REMEDY_COOKBOOK.md`](file:///C:/Users/USER/Desktop/%EC%BA%A1%EC%8A%A4%ED%86%A4/go2/docs/troubleshooting/QUICK_ERROR_LOOKUP_AND_REMEDY_COOKBOOK.md)에서 10초 1줄 명령어로 복구.
