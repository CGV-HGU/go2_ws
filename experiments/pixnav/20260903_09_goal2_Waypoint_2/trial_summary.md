# 🏁 Benchmark Trial Executive Summary

- **Method / Mode**: `PIXNAV`
- **Target Goal**: Goal #2 (`Waypoint_2`) at $(+10.58m, -0.02m)$
- **Outcome**: ⚠️ **HALTED** (USER_INTERRUPT)
- **Final Distance Error**: `0.000 m` (Tolerance: `0.35 m`)
- **Total Duration**: `85.33 s`
- **Total Trajectory Length**: `2.77 m`
- **Average Travel Speed**: `0.03 m/s` (Max limit: `0.50 m/s`)
- **Pose Sample Count (10Hz)**: `163`
- **Policy Inferences / VLM Queries**: `33` (Mean Latency: `165.1 ms`)

## 🏆 ICRA 2026 Navigation Benchmark Performance

| Evaluation Metric | Value | Reference / Standard |
|---|---|---|
| **Success Rate (SR)** | `0%` | Goal tolerance $\le 0.35 m$ |
| **SPL (Success Path Length)** | `0.000` | **Gold Standard** ($S \times L_{opt} / \max(L_{opt}, L_{act})$) |
| **Shortest Distance ($L_{opt}$)** | `6.41 m` | Euclidean Start-to-Goal |
| **Actual Trajectory ($L_{act}$)** | `2.77 m` | Integrated 10Hz Odometry |
| **Path Length Efficiency** | `231.3%` | $L_{opt} / L_{act} \times 100$ |
| **Min Obstacle Clearance** | `1.00 m` | 4D LiDAR Closest Point Cloud |
| **LiDAR Wall Repulsions** | `0` | Corridor centering auto-steers |
| **Forward Collision Stops** | `0` | Obstacle emergency interlocks (< 0.50m) |

## ⏱️ Detailed Time Log & Latency Breakdown

| Timing & Profiling Metric | Recorded Value |
|---|---|
| **Mission Start Time (Local)** | `2026-09-03 22:08:41` |
| **Mission End Time (Local)** | `2026-09-03 22:10:06` |
| **Total Navigation Time** | `85.334 s` (1.42 min) |
| **Effective Control Loop Rate** | `1.91 Hz` (Target: 10.0 Hz) |
| **Policy / VLM Mean Latency** | `165.1 ms` (Min: `104.2 ms`, Max: `269.5 ms`, P95: `220.2 ms`) |
| **Forward Translating Time** | `6.39 s` (39.3%) |
| **In-Place Rotating Time** | `9.87 s` (60.7%) |
| **Standby / Decel Time** | `0.00 s` (0.0%) |

## 📁 Saved Artifacts
- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.
- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.
- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.
- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.
- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.
- `camera_snapshots/` : Annotated decision frames.
