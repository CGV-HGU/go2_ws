# 🗺️ URBAN-SIM 지형 및 맵 생성 파라미터 연동 가이드
**작성자**: 이민석 (CGV-HGU 2026 Capstone autonomousDriving)
**수신자**: 유건민 (시뮬레이션 환경 구축 및 PPO/S2E 학습 제어 담당)

본 문서는 **CVPR 2025 URBAN-SIM 논문**의 맵/지형 생성 이론적 근거와 **우리의 실제 소스코드 파일 내의 구현부**를 1:1 매칭한 매뉴얼입니다. 커리큘럼 훈련 난이도 조율 및 맵 파라미터 디버깅 시 참조하시기 바랍니다.

---

## 1. 🏗️ 계층적 맵 생성 파이프라인 (Procedural Generation)

### 📌 논문 근거
*   **논문 섹션**: `3.1. Hierarchical Urban Generation` (본문 3페이지)
*   **핵심 원리**: 도시 스케일의 맵을 4가지 계층(도로 블록 연결 ➔ 구역 계획 ➔ WFC 알고리즘 지형 생성 ➔ 오브젝트 배치)으로 나누어 매 에피소드마다 절차적 합성(Infinite variations).

### 💻 코드 구현부
*   **맵 합성 코어 모듈**: [urbansim/scene/urban_scene.py](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/s2e-urban-rl/urbansim/scene/urban_scene.py)
*   **YAML 파일 파라미터 파싱부**: [urbansim/learning/RL/train.py (Line 241)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/s2e-urban-rl/urbansim/learning/RL/train.py#L241)
    ```python
    # YAML에 기입된 generator_config가 이 코드 라인을 통해 pg_config로 일괄 병합됩니다.
    pg_config.update(env_config['Env'].get('generator_config', {}))
    ```
*   **환경 생성 파라미터 바인딩 모듈**: [s2e_auto_env_cfg.py](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/s2e-urban-rl/urbansim/primitives/navigation/s2e_auto_env_cfg.py)

---

## 2. 🎛️ 지형(Locomotion) 난이도 조율 물리 상수 (Curriculum Learning용)

로봇개가 비틀거리거나 낙상하는 주 원인이 되는 지형(계단, 경사, 자갈길)의 정량적 치수 가이드라인입니다.

### 📌 논문 근거
*   **논문 섹션**: `Appendix E.1 - 2) Urban-Loc` (논문 27페이지 & Table 3)
*   **지형별 난이도 균등분포(Uniform Distribution) 범위**:
    1.  **계단 높이 (Stair)**:
        *   학습(Train) 단계: $0.05\text{ m}$ ~ $0.23\text{ m}$ (Table 3 기준)
        *   평가(Test) 단계: $0.10\text{ m}$ ~ $0.30\text{ m}$ (Out-of-Distribution 평가용)
    2.  **경사판 각도 (Slope)**:
        *   학습(Train) 단계: $0.00\text{ rad}$ ~ $0.40\text{ rad}$ (약 $0^\circ$ ~ $22.9^\circ$)
        *   평가(Test) 단계: $0.05\text{ rad}$ ~ $0.80\text{ rad}$ (약 $2.8^\circ$ ~ $45.8^\circ$)
    3.  **자갈길 굴곡 높낮이 (Rough)**:
        *   학습(Train) 단계: $0.02\text{ m}$ ~ $0.10\text{ m}$ (요철 높이)
        *   평가(Test) 단계: $0.05\text{ m}$ ~ $0.20\text{ m}$

### 💻 코드 구현부
*   **Go2 전용 환경 변동기**: [urbansim/primitives/robot/unitree_go2.py (Line 71)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/s2e-urban-rl/urbansim/primitives/robot/unitree_go2.py#L71)
    ```python
    def GO2RoughModifyEnv(env):
        # 계단 및 비포장 요철 지형의 높낮이 한계를 물리 스케일에 맞게 아래 상수들로 조절합니다.
        env.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        env.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
    ```
*   **보행 기본 지능 모델 (Trot Gait) 로드 패스**: [urbansim/primitives/robot/unitree_go2.py (Line 132)](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/s2e-urban-rl/urbansim/primitives/robot/unitree_go2.py#L132)
    *   `policy_path = "assets/ckpts/locomotion/unitree_go2.pt"`
    *   *주의*: 만약 로봇개가 스폰되자마자 껑충껑충 뛰거나 비틀거리면, 이 로컬 가중치 파일이 누락된 상태입니다. 아래 명령어로 에셋을 반드시 먼저 설치해야 합니다:
        ```bash
        python scripts/tools/collectors/collect_asset.py
        ```

---

## 3. 🎯 에피소드 주행 반경 및 장애물/보행자 스폰 수량

### 📌 논문 근거
*   **논문 섹션**: `Appendix E.4 - Environments` (논문 27페이지)
*   **환경 구분별 공식 세팅**:
    *   **NavClear**: 훈련 시 $10\text{ m} \times 10\text{ m}$ 영역 (장애물 없음), 평가 시 $15\text{ m} \times 15\text{ m}$
    *   **NavStatic**: 훈련 시 $10\text{ m} \times 10\text{ m}$ + **정적 장애물 4개**, 평가 시 $15\text{ m} \times 15\text{ m}$ + **정적 장애물 8개** (장애물: 벤치, 쓰레기통, 광고판 등)
    *   **NavDynamic**: 훈련 시 $10\text{ m} \times 10\text{ m}$ + **장애물 4개 + 보행자 2명**, 평가 시 $15\text{ m} \times 15\text{ m}$ + **장애물 8개 + 보행자 3명**

---

## 4. 🛠️ 우리의 Stage 3~4 난이도 조율용 오버라이드 가이드 (`go2_s2e_stage3.yaml`)

인도를 완전히 장애물로 가로막아 로봇개가 차도로 우회하는 **회복 주행(Recovery)**을 학습시킬 때, 우리가 YAML을 통해 맵 생성기에 강제 주입하는 파라미터 가이드입니다.

### 💻 설정 YAML 위치
*   [scratch/sim_curriculum/go2_s2e_stage3.yaml](file:///C:/Users/USER/Desktop/캡스톤/캡2-논문/go2_ws/scratch/sim_curriculum/go2_s2e_stage3.yaml)

### ⚙️ 핵심 오버라이드 목록 및 파라미터 주석
```yaml
Env:
  name: static
  generator: s2e_auto               # 상준님 최적화 배치 생성기 구동
  map_region: 30                    # 주행 공간을 30m x 30m 로 확대하여 주행 선회각 확보
  num_objects: 30                   # 장애물 밀도를 대폭 올려 보도를 인위 차단
  num_peds: 0
  generator_config:
    sidewalk_min_width: 4.0         # 인도 최소 폭 4.0m
    sidewalk_max_width: 6.0         # 인도 최대 폭 6.0m
    
    object_min_spacing: 1.35        # 장애물 간 이격 간격을 1.35m로 규정하여 회피 가능한 물리 틈새 확보
    object_edge_margin: 0.55        # 연석(보도 경계)과 장애물의 마진을 0.55m로 두어 보도 이탈 유도
    endpoint_object_clearance: 2.5  # 스폰 즉시 충돌하는 것을 방지하기 위해 시작/도착지 주변 2.5m 스폰 차단
    
    # [★중요 - 이민석 패치부]
    # s2e_auto_env_cfg.py에 추가된 dynamic walkable margin을 트리거합니다.
    walkable_margin: -1.0           # 인도 이탈 시 즉시 에피소드가 폭파되는 DoneTerm 제한을 비활성화 (-1.0)
    walkable_penalty_weight: -2.0   # 보도를 벗어났을 때의 폭탄 감점 가중치(-200.0)를 -2.0으로 낮추어 차도 우회 학습 유도
```

위 파라미터 구조와 주석을 토대로 가상 훈련장의 난이도를 다각도로 조율할 수 있습니다. 추가적인 파라미터 튜닝이 필요할 경우 언제든지 논의해 주시기 바랍니다.
