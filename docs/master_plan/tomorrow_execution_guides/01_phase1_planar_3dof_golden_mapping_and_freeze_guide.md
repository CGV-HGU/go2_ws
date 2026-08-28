# Phase 1: planar 3DoF 자격 검증 및 골든 맵 동결 가이드

> 실측 개정: 2026-08-28 KST
> 실행 스크립트: `/home/unitree/go2_ws_antarctica/map_headless.sh`
> 현재 결정: 단층 평면 지도는 3DoF, 4DoF는 실제 경사 코스의 선택 비교군
> 현재 상태: Z 안정성 + Type-1 global loop 기능 PASS, 최신 전체-map geometry FAIL
> 2026-08-28 결정: Type-1은 유지하고 왜곡을 만든 Type-2 spatial proximity는 기본 OFF

## 1. 3DoF를 쓰는 이유

2026-08-27 4DoF run은 raw LIO Z가 0.0212 m 범위에 머물렀지만 RTAB-Map graph Z가 6.452 m 발산했다. 2026-08-28 3DoF run은 raw Z span 0.0335 m, sampled map-to-base Z span 0.0175 m로 안정됐다. 따라서 같은 층의 평평한 복도에서는 `x/y/yaw` graph가 현재의 golden profile이다.

3DoF는 3D 기능을 끄는 설정이 아니다. 4D L2의 3D cloud, Unitree LIO, IMU, 3D LiDAR ICP, 3D occupancy 생성은 모두 유지한다.

```text
Reg/Strategy=1
Reg/Force3DoF=true
Icp/Force4DoF=false
RGBD/NeighborLinkRefining=false
RGBD/LoopClosureIdentityGuess=true
RGBD/ProximityBySpace=false
```

`Optimizer/Slam2D`는 설치된 RTAB-Map 0.21.1에서 제거된 legacy 이름이므로 합격 판정에 사용하지 않는다.

## 2. 현재 run 판정

대상: `20260828_124601_planar3dof_headless`

| 지표 | 결과 | 판정 |
|---|---:|---|
| nodes / DB | 249 / 73.9 MB, integrity `ok` | PASS |
| optimized poses / path | 164 / 41.04 m | PASS |
| optimized Z span | 0.0235 m | PASS |
| optimized start/end | 0.0335 m | PASS |
| Type-2 LiDAR proximity closure | logger 8, DB unique 9 | 기록; canonical에서는 OFF |
| Type-1 global visual closure | 2 (174→61, 211→1) | PASS |
| odometry lost / optimizer failure / NaN | 0 / 0 / 0 | PASS |
| DB save / optimized reload | true / 164 poses | PASS |
| wrapper | status 141 logging bug; 코드 수정 완료 | 다음 run 확인 |

이 run은 짧은 기능 자격 증거로 보존하되 golden map이나 localization DB로 사용하지 않는다. `negative hessian index (-1)` 118회는 covariance 계산 경고이며, 다른 optimizer failure/NaN/map jump가 없으므로 WARN으로만 기록한다.

후속 전체 run `20260828_141247_planar3dof_headless`는 DB integrity, Z 안정성, Type-1 36개를 기록했지만 실제 두 90도 코너가 접혀 **geometry FAIL**로 재판정했다. 복사 DB에서 Type-1과 Type-2를 각각 제거한 ablation 결과, Type-2만 제거한 궤적은 직교 코너를 회복했고 Type-1만 제거한 궤적에는 접힘이 남았다. 따라서 기존 Type-2 설정을 끄며 이 전체 DB는 localization에 사용하지 않는다.

## 3. 실험 계획표

| 순서 | 소요 | 실험 | 합격 기준 | 실패 시 조치 |
|---:|---:|---|---|---|
| 0 | 완료 | wrapper 정상 종료 | status 0, DB integrity OK, optimized pose >0 | 최신 전체 run에서 PASS |
| 1 | 1~2분 | Type-2 OFF 두-코너 loop | Type-2=0, odom lost=0, Z span ≤0.05 m, 두 90도 코너 보존 | 전체 맵으로 가지 않고 분석 |
| 2 | 같은 run | global visual loop | 물리적으로 올바른 Type-1 ≥1, 승인 직후 접힘/큰 jump=0 | 시작 view/heading과 overlap 보강 |
| 3 | 전체 경로 1회 | golden remap | 직선·평행 벽과 두 코너 보존, Type-2=0 | 동결하지 말고 targeted 분석 |
| 4 | 정지 시험 | export + cold-start localization | DB/PGM/YAML/3D map hash 고정, 재시작 localization PASS | map/config 동결 금지 |

## 4. 현장 주행 SOP

1. 로봇을 기립시키고 특징적인 시작 장면을 같은 heading으로 3~5초 관측한다.
2. 0.2~0.3 m/s로 주행하며 급가속·급정지·제자리 급회전을 피한다.
3. 코너는 가능한 한 완만한 곡선으로 회전해 연속 cloud overlap을 유지한다.
4. 출발했던 위치와 heading으로 복귀해 3~5초 정지한다.
5. 로봇이 정지한 상태에서 터미널에 `Ctrl+C`를 한 번 입력한다.
6. 종료 메시지와 DB 저장 확인이 끝날 때까지 두 번째 interrupt나 SSH 종료를 하지 않는다.

canonical run은 `RGBD/LoopClosureIdentityGuess=true`, `RGBD/ProximityBySpace=false`를 사용한다. 전자는 같은 pose와 heading에 복귀했을 때 RGB가 찾은 후보를 identity initial guess에서 3D LiDAR ICP로 검증한다. 후자는 최신 맵을 접은 Type-2 spatial proximity 검색만 끄며, 연속 LIO odometry와 3D cloud/ICP, Type-1 global visual loop는 그대로 유지한다.

## 5. 실행과 확인

```bash
cd /home/unitree/go2_ws_antarctica
./map_headless.sh
```

시작 배너의 필수 항목:

```text
profile=planar3dof
Reg/Force3DoF=true
Icp/Force4DoF=false
RGBD/ProximityBySpace=false
Recorder=false
Docker/motor=false
```

최신 증거 위치:

```bash
readlink -f /home/unitree/.ros/rtabmap_runs/latest
sed -n '1,120p' /home/unitree/.ros/rtabmap_runs/latest/run_manifest.txt
rg -n "ACCEPTED_GLOBAL|ACCEPTED_PROXIMITY|REJECTED|SUMMARY" \
  /home/unitree/.ros/rtabmap_runs/latest/loop_logs
python3 -c 'import sqlite3; p="file:/home/unitree/.ros/rtabmap_runs/latest/rtabmap.db?mode=ro"; c=sqlite3.connect(p, uri=True); print(c.execute("PRAGMA integrity_check").fetchone()[0]); c.close()'

# RTAB-Map CLI tools may update bookkeeping fields: analyze only a copy.
analysis_db=$(mktemp /tmp/rtabmap_analysis.XXXXXX.db)
cp -a /home/unitree/.ros/rtabmap_runs/latest/rtabmap.db "$analysis_db"
rtabmap-info "$analysis_db"
```

합격 시 다음이 모두 확인돼야 한다.

- `rtabmap_started=true`, `rtabmap_db_saved=true`
- 의도된 정상 종료 status
- DB integrity `ok`
- optimized graph pose가 0보다 큼
- 물리적으로 올바른 Type-1과 rejected 이유가 로그에 존재
- Type-2 accepted event와 DB Type-2 link가 모두 0
- `odometry lost`, optimizer failure, NaN, false closure 없음

## 6. 골든 맵 동결

Type-2 OFF 짧은 두-코너 Gate를 통과한 뒤에만 전체 맵을 생성한다.

1. 최종 run의 `rtabmap.db`, PGM, YAML, 3D map/cloud를 함께 export한다.
2. DB와 지도 파일의 SHA-256을 같은 manifest에 기록한다.
3. launch/config, sensor bridge, camera calibration, git HEAD도 함께 보존한다.
4. 동결 DB로 localization을 최소 3회 재시작해 초기 pose와 map jump를 확인한다.
5. 그 뒤에만 Docker S2E 무구동 폐루프로 진행한다.

4DoF가 필요해지는 경우는 실제 경사로·고도 변화가 평가 대상일 때뿐이다. 그때는 3DoF baseline과 DB를 섞지 않고 별도 run ID, 별도 지도, 기준 고도 측정으로 비교한다.
