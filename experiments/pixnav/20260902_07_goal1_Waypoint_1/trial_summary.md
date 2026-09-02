# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #1 (`Waypoint_1`) at $(-18.26m, +27.58m)$
- **Outcome**: ✅ **SUCCESS (ARRIVED)**
- **Final Distance Error**: `0.277 m` (Tolerance: `0.35 m`)
- **Total Duration**: `17.72 s`
- **Total Trajectory Length**: `5.56 m`
- **Average Travel Speed**: `0.31 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `177`
- **Policy Inferences / VLM Queries**: `36` (Mean Latency: `83.1 ms`)

## 📁 Saved Artifacts
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
