# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #2 (`Waypoint_2`) at $(-14.48m, -32.68m)$
- **Outcome**: ✅ **SUCCESS (ARRIVED)**
- **Final Distance Error**: `0.259 m` (Tolerance: `0.35 m`)
- **Total Duration**: `13.00 s`
- **Total Trajectory Length**: `3.33 m`
- **Average Travel Speed**: `0.26 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `129`
- **Policy Inferences / VLM Queries**: `26` (Mean Latency: `81.3 ms`)

## 📁 Saved Artifacts
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
