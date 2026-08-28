# Phase 2 frozen PixNav 무구동 자격시험 안내

> 2026-08-28 정정: 최신 `paper` branch의 주 backend는 frozen PixNav이며 S2E는 별도
> NavBench-GS 보조 실험이다. 과거 이 문서는 `test_docker_s2e_dryrun.py`와
> `docker_bridge.py`를 사용해 실제 S2E/zero-actuation을 검증한다고 잘못 설명했다.
> 전자는 합성 fallback과 규칙 기반 속도를 사용하고, 후자는 UDP command path를 여는
> 코드이므로 자격 증거로 사용할 수 없다.

현재 authoritative 절차는 다음 문서 하나를 따른다.

- [`../../experiments/04_post_mapping_pixnav_zero_actuation_qualification.md`](../../experiments/04_post_mapping_pixnav_zero_actuation_qualification.md)

복귀 직후 첫 두 명령:

```bash
cd /home/unitree/go2_ws_antarctica
./pixnav_check.py --preflight-only
```

현재 합격 대상은 논문 고정 구현 pin, 공식 Checkpoint_A hash, 실제 RGB file-only inference다.
이 단계에서는 `/cmd_vel`, Sport API, UDP 9090/9091을 사용하지 않는다.
