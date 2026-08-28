# Phase 1: planar 3DoF 자격 검증 및 골든 맵 동결 가이드

> 실측 개정: 2026-08-28 KST
> 실행 스크립트: `/home/unitree/go2_ws_antarctica/mapping_planar_headless.sh`
> 현재 결정: 단층 평면 지도는 3DoF, 4DoF는 실제 경사 코스의 선택 비교군
> 현재 상태: Z 안정성 PASS, Type-1 global loop와 정상 종료 저장은 미통과

## 1. 3DoF를 쓰는 이유

2026-08-27 4DoF run은 raw LIO Z가 0.0212 m 범위에 머물렀지만 RTAB-Map graph Z가 6.452 m 발산했다. 2026-08-28 3DoF run은 raw Z span 0.0335 m, sampled map-to-base Z span 0.0175 m로 안정됐다. 따라서 같은 층의 평평한 복도에서는 `x/y/yaw` graph가 현재의 golden profile이다.

3DoF는 3D 기능을 끄는 설정이 아니다. 4D L2의 3D cloud, Unitree LIO, IMU, 3D LiDAR ICP, 3D occupancy 생성은 모두 유지한다.

```text
Reg/Strategy=1
Reg/Force3DoF=true
Icp/Force4DoF=false
RGBD/NeighborLinkRefining=false
```

`Optimizer/Slam2D`는 설치된 RTAB-Map 0.21.1에서 제거된 legacy 이름이므로 합격 판정에 사용하지 않는다.

## 2. 현재 run 판정

대상: `20260828_113542_planar3dof_headless`

| 지표 | 결과 | 판정 |
|---|---:|---|
| nodes / DB | 352 / 95.7 MB, integrity OK | PASS |
| raw LIO Z span | 0.0335 m | PASS |
| sampled map-to-base Z span | 0.0175 m | PASS |
| Type-2 LiDAR proximity closure | 9 | PASS |
| Type-1 global visual closure | 0 | FAIL |
| odometry lost / optimizer failure / NaN | 0 / 0 / 0 | PASS |
| wrapper / optimized pose save | status 141 / 0 poses | FAIL |

이 run은 3DoF 진단 증거로 보존하되 golden map이나 localization DB로 사용하지 않는다. `negative hessian index (-1)` 경고는 covariance 계산 경고이며, 다른 optimizer failure/NaN/map jump가 없으면 WARN으로만 기록한다.

## 3. 실험 계획표

| 순서 | 소요 | 실험 | 합격 기준 | 실패 시 조치 |
|---:|---:|---|---|---|
| 0 | 2분 | 정지/초단거리 실행 후 정상 종료 | status 0/130, DB integrity OK, optimized pose >0, 잔여 process 없음 | wrapper signal/cleanup 수정 후 재시험 |
| 1 | 5~10분 | 짧은 planar loop 1회 | odom lost=0, NaN=0, Z span ≤0.05 m, 지도 접힘 없음 | 0.05~0.10 m면 재시험, 0.10 m 초과면 설정/센서 점검 |
| 2 | 5~10분 | identity-guess global loop A/B | 올바른 Type-1 ≥1, false closure=0 | 동일 pose/heading 재현 후 D435i branch 검토 |
| 3 | 20~30분 | 같은 짧은 경로 3회 반복 | 3회 Gate 0/1 통과, Type-1 성공 ≥2/3 | 환경·조명·시작 pose 통제 강화 |
| 4 | 현장 경로 | 180m 전체 golden map | DB/PGM/YAML/3D map export, hash 고정, localization 재시작 PASS | 동결하지 말고 원인 분석 |

## 4. 현장 주행 SOP

1. 로봇을 기립시키고 특징적인 시작 장면을 같은 heading으로 3~5초 관측한다.
2. 0.2~0.3 m/s로 주행하며 급가속·급정지·제자리 급회전을 피한다.
3. 코너는 가능한 한 완만한 곡선으로 회전해 연속 cloud overlap을 유지한다.
4. 출발했던 위치와 heading으로 복귀해 3~5초 정지한다.
5. 로봇이 정지한 상태에서 터미널에 `Ctrl+C`를 한 번 입력한다.
6. 종료 메시지와 DB 저장 확인이 끝날 때까지 두 번째 interrupt나 SSH 종료를 하지 않는다.

global loop A/B run에서는 별도 시험 설정으로 `RGBD/LoopClosureIdentityGuess=true`를 사용한다. 이 설정은 같은 pose와 heading에 복귀했을 때 RGB가 찾은 후보를 identity initial guess에서 3D LiDAR ICP로 검증하기 위한 것이다. 큰 위치/heading 오차를 무조건 허용하는 설정은 아니다.

## 5. 실행과 확인

```bash
cd /home/unitree/go2_ws_antarctica
./mapping_planar_headless.sh
```

시작 배너의 필수 항목:

```text
profile=planar3dof
Reg/Force3DoF=true
Icp/Force4DoF=false
Recorder=false
Docker/motor=false
```

최신 증거 위치:

```bash
readlink -f /home/unitree/.ros/rtabmap_runs/latest
sed -n '1,120p' /home/unitree/.ros/rtabmap_runs/latest/run_manifest.txt
rg -n "GLOBAL LOOP ACCEPTED|PROXIMITY LOOP ACCEPTED|REJECTED" \
  /home/unitree/.ros/rtabmap_runs/latest/loop_logs
sqlite3 /home/unitree/.ros/rtabmap_runs/latest/rtabmap.db 'PRAGMA integrity_check;'
rtabmap-info /home/unitree/.ros/rtabmap_runs/latest/rtabmap.db
```

합격 시 다음이 모두 확인돼야 한다.

- `rtabmap_started=true`, `rtabmap_db_saved=true`
- 의도된 정상 종료 status
- DB integrity `ok`
- optimized graph pose가 0보다 큼
- Type-1/Type-2 수와 rejected 이유가 로그에 존재
- `odometry lost`, optimizer failure, NaN, false closure 없음

## 6. 골든 맵 동결

Gate 0~3을 통과한 뒤에만 전체 맵을 생성한다.

1. 최종 run의 `rtabmap.db`, PGM, YAML, 3D map/cloud를 함께 export한다.
2. DB와 지도 파일의 SHA-256을 같은 manifest에 기록한다.
3. launch/config, sensor bridge, camera calibration, git HEAD도 함께 보존한다.
4. 동결 DB로 localization을 최소 3회 재시작해 초기 pose와 map jump를 확인한다.
5. 그 뒤에만 Docker S2E 무구동 폐루프로 진행한다.

4DoF가 필요해지는 경우는 실제 경사로·고도 변화가 평가 대상일 때뿐이다. 그때는 3DoF baseline과 DB를 섞지 않고 별도 run ID, 별도 지도, 기준 고도 측정으로 비교한다.
