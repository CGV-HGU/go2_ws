# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #2 (`Waypoint_2`) at $(+10.58m, -0.02m)$
- **Outcome**: ⚠️ **HALTED** (USER_INTERRUPT)
- **Final Distance Error**: `0.000 m` (Tolerance: `0.35 m`)
- **Total Duration**: `84.42 s`
- **Total Trajectory Length**: `3.61 m`
- **Average Travel Speed**: `0.04 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `174`
- **Policy Inferences / VLM Queries**: `34` (Mean Latency: `95.9 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `0%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `0.000` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `6.56 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `3.61 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `181.6%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `0.67 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `0` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 22:05:34` |
| **Mission End Time (Local)** | `2026-09-03 22:06:59` |
| **Total Navigation Time** | `84.417 s` (1.41 min) |
| **Effective Control Loop Rate** | `2.06 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `95.9 ms` (Min: `64.7 ms`, Max: `142.3 ms`, P95: `128.7 ms`) |
| **Forward Translating Time** | `8.60 s` (49.5%) |
| **In-Place Rotating Time** | `8.79 s` (50.5%) |
| **Standby / Decel Time** | `0.00 s` (0.0%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
