# 🕹️ [Know-How 05] Unitree SDK2 Sport API & 모터 제어 이중 계층(Dual-Layer) 해설서

> **대상 시스템**: Unitree Go2 SDK2 Sport API (ID 1008: Move, 1001: Damp, 1002: StandUp), `/cmd_vel`  
> **문서 목적**: High-Level Sport API 채택 이유, CycloneDDS `/api/sport/request` JSON 페이로드 구조, 속도 리미터 및 0.5초 워치독 안전 브레이크 전수 해설

---

## 1. 🔍 왜 12관절 직접 제어(LowCmd) 대신 High-Level Sport API를 쓰는가?

1. **1000Hz 실시간 밸런싱 MPC/WBC**:
   - 4족 보행 로봇은 지면과의 접촉 상태 추정, 슬립 방지 및 동적 균형 제어가 1000Hz 주기로 수행되어야 합니다.
   - Jetson(상위 제어기)에서 12개 모터의 위치/토크를 직접 계산하여 보내면 통신 지연이나 패킷 손실 시 로봇이 즉시 전도(Fall-down)됩니다.
2. **Sport API (Move 1008)**:
   - 상위 제어기는 바디 속도 $(v_x, v_y, \omega_z)$만 전송하고, 로봇 메인보드의 내장 DSP/MCU가 12개 모터의 역기구학(IK)과 밸런싱을 안전하게 계산하므로 안정성이 극대화됩니다.

---

## 2. ⚡ 이중 계층(Dual-Layer) 모터 구동 아키텍처

`host_bridge.py`는 도커로부터 속도 명령을 수신하면 아래 2개 경로로 동시 발행합니다:
1. **Layer 1**: ROS 2 표준 `/cmd_vel` (`geometry_msgs/Twist`)
2. **Layer 2**: 로봇 직결 CycloneDDS `/api/sport/request` (API ID 1008: Move, JSON `{"x": vx, "y": vy, "z": wz}`)

이로써 ROS 2 C++ 드라이버 노드(`go2_driver`)가 죽어 있거나 재기동 중이더라도 로봇이 100% 끊김 없이 주행을 지속합니다.
