# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #1 (`Waypoint_1`) at $(-18.69m, +26.88m)$
- **Outcome**: ⚠️ **HALTED** (USER_INTERRUPT)
- **Final Distance Error**: `0.000 m` (Tolerance: `0.35 m`)
- **Total Duration**: `20.01 s`
- **Total Trajectory Length**: `5.30 m`
- **Average Travel Speed**: `0.26 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `200`
- **VLM Queries Count**: `0` (Mean Latency: `0.0 ms`)

## 📁 Saved Artifacts
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every VLM query, prompt, and sub-goal.
- `camera_snapshots/` : Annotated decision frames.
