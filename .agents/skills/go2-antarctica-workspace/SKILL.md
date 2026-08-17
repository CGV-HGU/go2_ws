---
name: go2-antarctica-workspace
description: Comprehensive Guide, Architecture, and Real-Robot Benchmark Protocol for Unitree Go2 ESCAPE-Nav (antarctica branch) ICRA 2026 Paper.
---

# 🧊 Unitree GO2 ESCAPE-Nav (`antarctica`) Workspace & Benchmark Guide

## 1. Overview & Repository Info
- **Workspace Path**: `/home/unitree/go2_ws_antarctica`
- **Git Repo & Branch**: `https://github.com/CGV-HGU/go2_ws.git` (Branch: `antarctica`)
- **Target Paper**: *ESCAPE-Nav: Experience-Shaped Causally Aligned Perception–Execution for Asynchronous VLM Navigation* (ICRA 2026)

## 2. Hybrid Asynchronous System Architecture
- **Host OS (NVIDIA Jetson Orin NX 16GB / Ubuntu 20.04 / ROS 2 Foxy / CUDA 11.4)**:
  - `go2_robot` (`go2_driver` C++ DDS driver for `SportClient.Move` API)
  - `rtabmap_ros` (`go2_rtabmap.launch.py` @ 50Hz LIVO odometry `/rtabmap/odom`)
  - `scratch/host_bridge.py` (Host UDP Receiver on `127.0.0.1:5005`)
- **Docker Container / External Server (ROS 2 Jazzy / Python 3.12)**:
  - `s2e-vlm-async-framework` (`vlm_s2e_async_node.py` tag v6)
  - `scratch/docker_bridge.py` (Docker UDP Transmitter on `127.0.0.1:5005`)

## 3. Real-Robot 4-Step Execution Pipeline
1. **Host RTAB-Map LIVO Odometry**:
   ```bash
   cd /home/unitree/go2_ws_antarctica
   source /opt/ros/foxy/setup.bash
   ros2 launch rtabmap_launch go2_rtabmap.launch.py
   ```
2. **VLM & Fast Trajectory Policy**:
   ```bash
   python3 s2e-vlm-async-framework/src/vlm_s2e_async_node.py
   ```
3. **Host-Docker UDP Socket Bridge (1ms Latency)**:
   ```bash
   python3 scratch/host_bridge.py
   ```
4. **1-Click Rosbag Logger & ICRA Table VIII Calculator**:
   ```bash
   bash scratch/record_experiment.sh Dead_End_Room Ours_Async Trial1
   python3 scratch/calculate_icra_metrics.py
   ```

## 4. Real-Robot Core Scenarios & Benchmark Metrics (Table VIII)
- **5 Core Scenarios**:
  1. `Dead-end room` (Rear exit escape via 360-deg active sweep)
  2. `Blocked goal direction` (Side-rerouting around obstacle)
  3. `Repeated corridor` (Directional memory suppressing failed-edge re-entry)
  4. `Active-view recovery` (Yaw sweep for new branch exploration)
  5. `Dynamic obstacle` (1.2m/s pedestrian avoidance & deceleration)
- **6 Benchmark Metrics**:
  - `Succ./5`: Success count (out of 5 trials)
  - `IF/5`: Intervention-free count
  - `Time (s)` ($T^\dagger$): Normalized completion time with timeout penalty
  - `Duty`: Driving duty ratio (active motion vs stop-and-go idle time)
  - `Rec. succ.` (DRS): Directional recovery success rate
  - `Re-entry` (FBR): Failed-branch re-entry rate
