# ESCAPE-Nav Go2 project instructions

## Mandatory context

- Before making architectural, deployment, or real-robot decisions, read
  `docs/CODEX_PROJECT_MEMORY.md`.
- Treat its dated live snapshot as historical evidence, not a permanent health
  claim. Re-run the relevant read-only checks when current state matters.
- The canonical S2E repository is
  `https://github.com/CGV-HGU/s2e-vlm-async-framework`. Use `main` for the
  current software baseline. `paper` is a divergent manuscript/experiment
  branch, not the robot deployment baseline.

## Evidence rules

- Prefer, in order: current runtime observation, current source/configuration,
  canonical source, tests that exercise the claimed path, then documentation.
- Never infer real-robot readiness from mock backends, synthetic images,
  loopback-only UDP tests, process listings, or hard-coded dashboard values.
- The older documents marked “NON-AUTHORITATIVE” are historical notes. Their
  `100%`, `ALL PASS`, `50 Hz LIVO`, and `Production-ready` statements are not
  acceptance evidence.
- Record exact command, time, commit, model, topic, and artifact path for new
  measurements. Label estimates and unverified claims explicitly.

## Real-robot safety

- Physical motion is disabled by default. Do not publish `/cmd_vel`, invoke
  Unitree Sport motion APIs, start a closed-loop autonomy launch, or run a
  micro-motion script without an explicit user request for that physical test.
- Before any authorized motion test, require a supervised test area, a human
  E-stop/operator, a working command watchdog and zero-command timeout, bounded
  velocity/acceleration, fresh odometry, sensor validity, and a verified
  stop-on-exit path.
- Do not use `scratch/bringup_all_escape_nav.sh` for motion in its current form.
  It references a missing node and its stop path and credential handling are
  unsafe.
- Do not start RTAB-Map mapping mode until `/home/unitree/.ros/rtabmap.db` is
  backed up and the user explicitly intends a new map. The current mapping
  launch passes `-d` and deletes the database at startup.
- Never add, repeat, or expose passwords, tokens, API keys, or embedded sudo
  credentials. Replace hard-coded credentials with an approved mechanism.

## Repository hygiene

- Work from `/home/unitree/go2_ws_antarctica`, the Git root. The nested
  `s2e-vlm-async-framework/` directory is tracked by the parent repository and
  is not an independent checkout.
- Preserve unrelated user changes and generated experiment/map artifacts.
- Do not overwrite map databases, bags, checkpoints, or calibration files.
- Keep deployment configuration explicit: ROS distribution, `ROS_DOMAIN_ID`,
  RMW implementation, backend (`mock` versus real), model identifier, and
  checkpoint hash must never rely on ambiguous defaults.

## Required verification for readiness claims

- A “ready for real driving” claim requires the complete camera/lidar/IMU/odom
  to VLM/S2E to trajectory to safety gateway to Go2 command path to pass with
  real inputs and fail-safe injection tests.
- Unit tests must be run by package until the duplicate top-level `test`
  collection issue is fixed. Report failures; do not collapse partial passes
  into a green aggregate.
- Real-robot experiment results require immutable rosbag/event evidence and a
  metrics pipeline that parses those artifacts. Sample Python dataclasses or
  generated plots are not experimental results.

