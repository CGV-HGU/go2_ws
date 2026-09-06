# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #1 (`Waypoint_1`) at $(+5.25m, -1.86m)$
- **Outcome**: ⚠️ **HALTED** (USER_INTERRUPT)
- **Final Distance Error**: `0.000 m` (Tolerance: `0.35 m`)
- **Total Duration**: `0.18 s`
- **Total Trajectory Length**: `0.00 m`
- **Average Travel Speed**: `0.00 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `1`
- **Policy Inferences / VLM Queries**: `0` (Mean Latency: `0.0 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `0%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `0.000` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `5.57 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `0.00 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `100.0%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `1.00 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `0` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 20:59:06` |
| **Mission End Time (Local)** | `2026-09-03 20:59:06` |
| **Total Navigation Time** | `0.176 s` (0.00 min) |
| **Effective Control Loop Rate** | `5.68 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `0.0 ms` (Min: `0.0 ms`, Max: `0.0 ms`, P95: `0.0 ms`) |
| **Forward Translating Time** | `0.00 s` (0.0%) |
| **In-Place Rotating Time** | `0.00 s` (0.0%) |
| **Standby / Decel Time** | `0.08 s` (100.0%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
