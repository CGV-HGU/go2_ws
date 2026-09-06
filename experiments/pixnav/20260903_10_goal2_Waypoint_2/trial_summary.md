# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #2 (`Waypoint_2`) at $(+10.58m, -0.02m)$
- **Outcome**: ⚠️ **HALTED** (USER_INTERRUPT)
- **Final Distance Error**: `0.000 m` (Tolerance: `0.35 m`)
- **Total Duration**: `54.28 s`
- **Total Trajectory Length**: `9.00 m`
- **Average Travel Speed**: `0.17 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `275`
- **Policy Inferences / VLM Queries**: `55` (Mean Latency: `125.2 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `0%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `0.000` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `19.40 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `9.00 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `215.4%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `1.00 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `0` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 22:11:34` |
| **Mission End Time (Local)** | `2026-09-03 22:12:28` |
| **Total Navigation Time** | `54.277 s` (0.90 min) |
| **Effective Control Loop Rate** | `5.07 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `125.2 ms` (Min: `82.2 ms`, Max: `258.5 ms`, P95: `195.1 ms`) |
| **Forward Translating Time** | `23.30 s` (85.0%) |
| **In-Place Rotating Time** | `4.00 s` (14.6%) |
| **Standby / Decel Time** | `0.13 s` (0.5%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
