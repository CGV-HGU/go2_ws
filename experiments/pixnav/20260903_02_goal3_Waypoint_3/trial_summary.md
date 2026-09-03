# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #3 (`Waypoint_3`) at $(+80.45m, +10.95m)$
- **Outcome**: ⚠️ **HALTED** (USER_INTERRUPT)
- **Final Distance Error**: `0.000 m` (Tolerance: `0.35 m`)
- **Total Duration**: `39.51 s`
- **Total Trajectory Length**: `7.99 m`
- **Average Travel Speed**: `0.20 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `395`
- **Policy Inferences / VLM Queries**: `79` (Mean Latency: `73.9 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `0%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `0.000` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `9.56 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `7.99 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `119.7%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `0.44 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `1` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 18:06:45` |
| **Mission End Time (Local)** | `2026-09-03 18:07:25` |
| **Total Navigation Time** | `39.511 s` (0.66 min) |
| **Effective Control Loop Rate** | `10.00 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `73.9 ms` (Min: `60.7 ms`, Max: `137.7 ms`, P95: `118.4 ms`) |
| **Forward Translating Time** | `21.00 s` (53.3%) |
| **In-Place Rotating Time** | `18.42 s` (46.7%) |
| **Standby / Decel Time** | `0.00 s` (0.0%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
