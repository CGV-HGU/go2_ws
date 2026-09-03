# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #1 (`Waypoint_1`) at $(+71.83m, +7.09m)$
- **Outcome**: ✅ **SUCCESS (ARRIVED)**
- **Final Distance Error**: `0.340 m` (Tolerance: `0.35 m`)
- **Total Duration**: `10.44 s`
- **Total Trajectory Length**: `3.04 m`
- **Average Travel Speed**: `0.29 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `104`
- **Policy Inferences / VLM Queries**: `21` (Mean Latency: `89.6 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `100%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `1.000` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `3.30 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `3.04 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `108.6%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `1.00 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `0` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 17:50:37` |
| **Mission End Time (Local)** | `2026-09-03 17:50:47` |
| **Total Navigation Time** | `10.444 s` (0.17 min) |
| **Effective Control Loop Rate** | `9.96 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `89.6 ms` (Min: `62.9 ms`, Max: `148.2 ms`, P95: `135.5 ms`) |
| **Forward Translating Time** | `7.80 s` (75.4%) |
| **In-Place Rotating Time** | `2.50 s` (24.2%) |
| **Standby / Decel Time** | `0.04 s` (0.4%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
