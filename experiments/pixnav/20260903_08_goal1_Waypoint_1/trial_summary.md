# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #1 (`Waypoint_1`) at $(+4.25m, -2.10m)$
- **Outcome**: ✅ **SUCCESS (ARRIVED)**
- **Final Distance Error**: `0.321 m` (Tolerance: `0.35 m`)
- **Total Duration**: `31.67 s`
- **Total Trajectory Length**: `5.20 m`
- **Average Travel Speed**: `0.16 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `316`
- **Policy Inferences / VLM Queries**: `63` (Mean Latency: `93.0 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `100%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `0.911` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `4.74 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `5.20 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `91.1%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `1.00 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `0` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 22:07:47` |
| **Mission End Time (Local)** | `2026-09-03 22:08:18` |
| **Total Navigation Time** | `31.673 s` (0.53 min) |
| **Effective Control Loop Rate** | `9.98 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `93.0 ms` (Min: `67.1 ms`, Max: `176.4 ms`, P95: `157.9 ms`) |
| **Forward Translating Time** | `12.40 s` (39.3%) |
| **In-Place Rotating Time** | `19.18 s` (60.7%) |
| **Standby / Decel Time** | `0.00 s` (0.0%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
