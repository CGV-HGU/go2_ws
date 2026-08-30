# ESCAPE-Nav PixNav file-only adapter

This package validates frozen PixNav outputs and converts them into bounded
macro-action **proposals** for audit. It deliberately has no ROS, DDS, socket,
Unitree SDK, `/cmd_vel`, or actuator publisher.

The action contract follows the paper-pinned Pixel-Navigator implementation:

| ID | PixNav action | File-only proposal |
|---:|---|---|
| 0 | `stop` | zero hold |
| 1 | `forward` | 0.25 m body-frame translation target with safety caps |
| 2 | `turn_left` | +30 degree yaw target with safety caps |
| 3 | `turn_right` | -30 degree yaw target with safety caps |
| 4 | `look_up` | zero hold and request re-observation |
| 5 | `look_down` | zero hold and request re-observation |

All outputs contain `actuation_permitted=false`. P7 now validates freshness,
L2/odometry clearance and interlock inputs, while `gateway_core.py` implements
the robot-free P8-A single-authority/TTL/envelope/E-stop/deadman contract. Both
layers remain non-actuating. A separately reviewed P8-B ROS/SDK dispatcher,
physical E-stop and measured safe-stop test are still required before any
physical command path can be considered.

Replay a completed `pixnav_check.py` report:

```bash
PYTHONPATH=src/escape_nav_pixnav \
python3 -m escape_nav_pixnav.replay \
  /home/unitree/.ros/pixnav_runs/<RUN_ID>/report.json
```

Evidence is written under `~/.ros/pixnav_macro_runs/` as a hash-chained JSONL
audit plus `summary.json`. This replay performs no inference and no actuation.

Additional file-only tools validate the VLM-to-PixNav causal linkage, inject
fail-closed faults, and freeze a runtime/evidence manifest. None of them creates
a ROS publisher, network socket, or Unitree SDK client. The future live chain
must preserve this order:

`frame_captured -> vlm_submitted -> vlm_completed -> pixnav_completed -> macro_audited`

`CausalAdmissionLedger` rejects stale, duplicate, out-of-order, disconnected,
or actuation-enabled events and reports a zero-target deadman hold when a stage
times out. This is a software contract only, not measured physical stop latency.

From the repository root, the robot-free regression entry point is:

```bash
./test_pixnav_offline.sh quick     # build, tests and saved evidence hashes
./test_pixnav_offline.sh evidence  # regenerate file-only chain/fault/manifest
./test_pixnav_offline.sh cuda      # saved-RGB CUDA replay plus downstream evidence
```

See `docs/experiments/06_pixnav_components_where_why_and_robot_free_tests.md`
for where each component is used and what each PASS does or does not prove.
