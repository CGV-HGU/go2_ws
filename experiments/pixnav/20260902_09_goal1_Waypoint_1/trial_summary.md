# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #1 (`Waypoint_1`) at $(-9.33m, -23.85m)$
- **Outcome**: ✅ **SUCCESS (ARRIVED)**
- **Final Distance Error**: `0.335 m` (Tolerance: `0.35 m`)
- **Total Duration**: `29.92 s`
- **Total Trajectory Length**: `4.02 m`
- **Average Travel Speed**: `0.13 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `299`
- **Policy Inferences / VLM Queries**: `60` (Mean Latency: `77.8 ms`)

## 📁 Saved Artifacts
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
