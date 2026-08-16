# 🐕 [05] 학교 실내 4대 현실적 코스 현장 주행 프로토콜

> **문서 소유자**: **민석 (Minseok)**  
> **테스트 장소**: 한동대학교 연구동 건물 복도 (1~2층)  
> **문서 목적**: 별도의 번거로운 파티션 설치 없이 연구동 복도 환경을 100% 활용하는 **4대 현실적 실내 코스(직선 복도, 90도 직각 코너, T자 갈림길, 동적 장애물) 20회 현장 주행 및 데이터 수집 표준 프로토콜**입니다.

---

## 📌 목차 (Table of Contents)
1. [실내 4대 현실적 코스 배치도 및 규격](#1-실내-4대-현실적-코스-배치도-및-규격)
2. [하드웨어 & 센서 사전 점검 체크리스트](#2-하드웨어--센서-사전-점검-체크리스트)
3. [현장 4단계 온보드 실행 명령어 매뉴얼](#3-현장-4단계-온보드-실행-명령어-매뉴얼)

---

## 📐 1. 실내 4대 현실적 코스 배치도 및 규격

```text
========================================================================================
             REALISTIC INDOOR FIELD EXPERIMENTAL SCENARIOS (총 20회 주행)
========================================================================================
 [시나리오 1: 단거리 직선 복도 (Straight Corridor 5m/10m)]
 🔵 START ─────── (1.5m 폭 직선 복도 주행, vy=0.0 보행 안정성 검증) ───────> 🔴 GOAL

 [시나리오 2: 90도 직각 코너 선회 (90-Degree Sharp Corner Turn 10m)]
 🔵 START ─────── (직선 10m) ───────> ┌─────── (90도 직각 선회, 문틀 찝힘 회피) ───────> 🔴 GOAL
                                      │

 [시나리오 3: T자 갈림길 복도 (T-Junction Multi-Branch 15m)]
 🔵 START ─────── (15m 삼거리 갈림길 진입 후 올바른 복도 선택) ───────> 🔴 GOAL

 [시나리오 4: 동적 보행자 회피 (Dynamic Obstacle & Pedestrian 15m)]
 🔵 START ─────── (1.2m/s 이동 보행자 2명 통과 시 안전 감속/우회) ───────> 🔴 GOAL
========================================================================================
```

---

## 🔋 2. 하드웨어 & 센서 사전 점검 체크리스트

* [ ] **Go2 배터리 잔량**: $\ge 80\%$ (스마트 배터리)
* [ ] **Jetson Orin NX 부팅**: 12V 통전 및 파란색 LED 확인
* [ ] **네트워크 연결**: `ssh unitree@192.168.123.15` 통신 확인
* [ ] **센서 렌즈 청결도**: RGB 전면 카메라 및 L2 LiDAR 렌즈 먼지/지문 제거
* [ ] **안전 E-Stop**: 무선 리모컨 손에 지참

---

## 🚀 3. 현장 4단계 온보드 실행 명령어 매뉴얼

```bash
# 1단계: 젯슨 접속 및 코드 동기화
ssh unitree@192.168.123.15
cd ~/go2_ws && git pull cgv-hgu antarctica

# 2단계: 호스트 RTAB-Map LIVO 가동 (터미널 1)
ros2 launch rtabmap_launch go2_rtabmap.launch.py

# 3단계: Docker 정책(v6) 및 소켓 브릿지 가동 (터미널 2 & 3)
docker exec -it sdam_go2_container python3 src/vlm_s2e_async_node.py
python3 ~/go2_ws/scratch/host_bridge.py

# 4단계: 1-Click Rosbag 자동 로깅 (터미널 4)
bash ~/go2_ws/scratch/record_experiment.sh Straight_Corridor Ours_Async Trial1
bash ~/go2_ws/scratch/record_experiment.sh Corner_90Deg Ours_Async Trial1
bash ~/go2_ws/scratch/record_experiment.sh TJunction_15m Ours_Async Trial1
bash ~/go2_ws/scratch/record_experiment.sh Dynamic_Obstacle Ours_Async Trial1

# 5단계: 20회 주행 후 정량 비교표 자동 산출
python3 ~/go2_ws/scratch/calculate_icra_metrics.py
```
