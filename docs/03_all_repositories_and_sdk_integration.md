# 📦 [03] 5대 연동 레포지토리 및 Unitree SDK2 3-DOF 제어기 통합 가이드

> **문서 소유자**: **민석 (Minseok)**  
> **문서 목적**: 우리 연구실(CGV-HGU)의 **5대 핵심 연동 레포지토리(s2e-vlm-async-framework v6, antarctica-simul, isaac-go2-rl-training, visualnav-transformer, go2_ws)**의 역할 분석 및 Unitree SDK2 Python API, 3-DOF 홀로노믹 제어기(`pd_controller.py`) 연동 명세입니다.

---

## 📌 목차 (Table of Contents)
1. [5대 연동 레포지토리 아키텍처 및 역할 분석](#1-5대-연동-레포지토리-아키텍처-및-역할-분석)
2. [Unitree SDK2 Python API 모터 제어 바인딩](#2-unitree-sdk2-python-api-모터-제어-바인딩)
3. [3-DOF 전방향 홀로노믹 PD 제어기 (pd_controller.py)](#3-3-dof-전방향-홀로노믹-pd-제어기-pd_controllerpy)

---

## 🔍 1. 5대 연동 레포지토리 아키텍처 및 역할 분석

| 레포지토리 명칭 | 주요 담당자 및 브랜치 | 핵심 역할 및 기능 | 실물 로봇 연동 지점 |
| :--- | :--- | :--- | :--- |
| **`s2e-vlm-async-framework`** | 상준 님 (`tag: v6`) | 비동기 VLM 메모리 그래프 + S2E 궤적 생성 메인 두뇌 | S2E 출력 궤적을 실물 Go2 속도로 변환 |
| **`visualnav-transformer`** | SOTA 대조군 (ViNT/NoMAD) | NoMAD/ViNT 베이스라인 비교군 및 PD 제어기 모듈 | `pd_controller.py`를 3-DOF 홀로노믹 제어기로 커스텀 확장 |
| **`antarctica-simul`** | 현서 / 건민 님 | NavBench-GS 3D Gaussian Splatting 대규모 가상 시뮬레이션 | PointNav `[distance, bearing]` 관측 포맷 표준화 |
| **`isaac-go2-rl-training`** | 상준 / 건민 님 | Isaac Sim 기반 Go2 4족 보행 저수준 RL 강화학습 모델 | 로봇 물리 한계 속도($v_{max}=0.5\text{m/s}$) 제원 매핑 |
| **`go2_ws` (현재 레포)** | **민석 님 (`antarctica`)** | **Jetson Orin NX 온보드 센서, 50Hz VLIO, 실물 로봇 구동** | **50Hz RTAB-Map LIVO, 1-Click Rosbag, 자동 채점기** |

---

## 🤖 2. Unitree SDK2 Python API 모터 제어 바인딩

`scratch/python_direct_driver.py`에서 Go2의 모션 제어 보드에 저지연($<1\text{ms}$)으로 속도 명령을 인가합니다:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class Go2DirectDriver:
    def __init__(self, eth_interface: str = "eth0"):
        ChannelFactoryInitialize(0, eth_interface)
        self.client = SportClient()
        self.client.SetTimeout(10.0)
        self.client.Init()

    def send_velocity(self, vx: float, vy: float, vyaw: float):
        """3-DOF 전방향 속도 인가 (vx: 전진, vy: 횡이동, vyaw: 각속도)"""
        self.client.Move(float(vx), float(vy), float(vyaw))

    def emergency_stop(self):
        """비상 정지 및 댐핑 모드 전환"""
        self.client.Damp()
```

---

## 🏎️ 3. 3-DOF 전방향 홀로노믹 PD 제어기 (`pd_controller.py`)

90도 직각 코너 선회 시 문틀/벽면 찝힘을 방지하기 위해 횡속도($v_y$) 댐핑 통로를 개방한 3-DOF 제어기입니다:

```python
def pd_controller(waypoint: np.ndarray) -> Tuple[float, float, float]:
    """3-DOF Omnidirectional PD controller for Go2 quadruped (vx, vy, w)"""
    assert len(waypoint) in (2, 4), "waypoint must be a 2D or 4D vector"
    dx, dy = waypoint[0], waypoint[1]

    if np.abs(dx) < EPS:
        v_x = 0.0
        v_y = np.clip(dy / DT, -0.2, 0.2) # 90도 코너링 시 횡방향 홀로노믹 스트레이핑
        w = np.sign(dy) * np.pi / (2 * DT)
    else:
        v_x = np.clip(dx / DT, 0, MAX_V)
        v_y = np.clip(dy / (2 * DT), -0.2, 0.2) # 부드러운 횡이동 댐핑
        w = np.clip(np.arctan2(dy, dx) / DT, -MAX_W, MAX_W)

    return v_x, v_y, w
```
