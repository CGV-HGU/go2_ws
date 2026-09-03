# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #4 (`Waypoint_4`) at $(+83.25m, +8.49m)$
- **Outcome**: ⚠️ **HALTED** (USER_INTERRUPT)
- **Final Distance Error**: `0.000 m` (Tolerance: `0.35 m`)
- **Total Duration**: `23.70 s`
- **Total Trajectory Length**: `3.66 m`
- **Average Travel Speed**: `0.15 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `236`
- **Policy Inferences / VLM Queries**: `47` (Mean Latency: `77.3 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `0%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `0.000` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `6.56 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `3.66 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `179.3%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `0.04 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `23` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 18:09:29` |
| **Mission End Time (Local)** | `2026-09-03 18:09:53` |
| **Total Navigation Time** | `23.695 s` (0.39 min) |
| **Effective Control Loop Rate** | `9.96 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `77.3 ms` (Min: `59.7 ms`, Max: `137.2 ms`, P95: `120.6 ms`) |
| **Forward Translating Time** | `10.40 s` (44.1%) |
| **In-Place Rotating Time** | `13.21 s` (55.9%) |
| **Standby / Decel Time** | `0.00 s` (0.0%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
