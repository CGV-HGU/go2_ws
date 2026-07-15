# Project Rules: Unitree Go2 Antarctic Navigation Workspace

This rule file provides Google Antigravity with a comprehensive understanding of the development environment, system architecture, hardware specifications, and branch structure for the Unitree Go2 project.

---

## 1. Environment & Hardware Specifications

- **Robot Platform**: Unitree Go2 (Quadruped Robot)
- **Onboard Compute (Expansion Dock)**: Jetson Orin NX (16GB RAM)
- **Sudo Password**: `admin`
- **Deployment Domain**: Indoor & Outdoor Autonomous Navigation / SLAM (Antarctic & extreme environment research)
- **Core OS / ROS 2 Configuration**:
  - **Host OS**: Ubuntu with **ROS 2 Foxy** (Native)
    - *GPU Stack*: CUDA 11.4, TensorRT 8.5.2, PyTorch ARM64 (L4T jp v511), ONNX Runtime GPU (v1.11+)
    - *Role*: Heavy model inference (e.g., ViNT) and direct robot hardware interfacing (`go2_robot`, `go2_driver`, `go2_bringup`).
  - **Docker Container**: **ROS 2 Jazzy** (CPU-only, run with `--net=host` loopback sharing)
    - *Role*: Runs the asynchronous navigation control framework (`s2e-vlm-async-framework`).

---

## 2. Network & Bridge Architecture

Due to DDS deserialization issues between ROS 2 Foxy and Jazzy, a dual-layer communications bridge is utilized to isolate host and docker environments:
1. **Zenoh Bridge**: Default tool for DDS bridging (`zenoh-bridge-dds`).
2. **Python UDP Socket Bridge**: Backup connection layer (`scratch/host_bridge.py` on host, `scratch/docker_bridge.py` in container) that bypasses C++ build failures.
3. **CycloneDDS Configuration**:
  - High-traffic raw sensor topics are isolated locally to save bandwidth.
  - Multi-cast is disabled on university/VPN networks. **Unicast peer-to-peer mapping** is enforced via `cyclonedds.xml`.
  - The default configuration binds to network interface `eth0` (must be updated to `wlan0` or relevant interface if connection fails).

---

## 3. Git Branch Structure & Specializations

The repository contains distinct branches developed for different test phases and capabilities:

1. **`master`** (Latest Unified Integration)
   - Serves as the primary development branch.
   - Merges features from `antarctica` and `antarctica-simul`.
   - Includes async framework `s2e-vlm-async-framework`, memory-enhanced VLM framework `qwen_nav_memory_framework_v3`, and real-world deployment bridges.
2. **`summer`** (Real Robot SLAM & Nav2 Execution Scripts)
   - Houses specialized execution scripts divided specifically for indoor and outdoor operations.
   - **Indoor Set** (`run_map_indoor.sh`, `run_localization_indoor.sh`): Uses **VIO (D435i Camera)** + Leg Encoder + IMU 3-way fusion. Best for environments without direct sunlight.
   - **Outdoor Set** (`run_map_outdoor.sh`, `run_localization_outdoor.sh`): Disables Visual Odometry (`visual_odometry:=false`) due to outdoor IR washouts, relying on **4D LiDAR L1** + Onboard IMU Odometry.
   - Bridges Nav2 output (`/cmd_vel`) to Unitree Sport API (JSON serialization via `go2_driver`).
3. **`antarctica-simul`** (Simulation, Curriculum & Kinematic Verification)
   - Contains simulation curriculum tools (`scratch/sim_curriculum/`), kinematic diagnostics (`scratch/test_go2_pd_controller.py`), and PD controller nodes.
   - Implements `voca_deadlock_detector.py` to prevent getting stuck during VLM guidance.
4. **`antarctica`** (Real-World Deployment & VLM Integration)
   - Focuses on real-world Qwen VLM robot backend implementation (`Ros2RobotBackend`, `run_qwen_ros2.py`).
   - Contains utility script `check_repo_updates.py` to monitor remote changes.
5. **`main`** (Initial Baseline)
   - Initial repository setup commit.

---

## 4. Control, Trajectory & Tuning Strategies

- **Normalized Trajectory Recovery**:
  - E2E models predict a normalized trajectory (independent of speed scale) to prevent velocity-scale dependency:
    $$\text{Normalized Trajectory} = \frac{\text{GT Trajectory}}{\Delta t \times v_{GT}}$$
  - The control node restores the trajectory back to metric units by multiplying the predicted velocity scale ($v_{pred}$):
    $$\text{Recovered Trajectory} = \text{Normalized Trajectory} \times \Delta t \times v_{pred}$$
- **Quadruped Walk Stability**:
  - Crucially, lateral velocity ($v_y$) is **disabled ($v_y = 0.0$)** during navigation to prevent side-slipping and tripping hazards on rough terrain. Only forward velocity ($v_x$) and rotational yaw velocity ($v_{yaw}$) are sent to the controller.
- **Fishtailing Attenuation**:
  - Uses Proportional-Derivative (PD) angular control to damp oscillations (fishtailing) at the rear:
    $$w = K_{p\_ang} \times \text{atan2}(dy, dx) + K_{d\_ang} \times d\_heading$$
  - Reduce `kp_angular` and increase `kd_angular` if oscillations occur during walk tests.
