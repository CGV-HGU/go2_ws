# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #1 (`Waypoint_1`) at $(-12.37m, +27.44m)$
- **Outcome**: ✅ **SUCCESS (ARRIVED)**
- **Final Distance Error**: `0.160 m` (Tolerance: `0.35 m`)
- **Total Duration**: `1.34 s`
- **Total Trajectory Length**: `0.26 m`
- **Average Travel Speed**: `0.19 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `13`
- **Policy Inferences / VLM Queries**: `2` (Mean Latency: `102.3 ms`)

## 📁 Saved Artifacts
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
