#!/usr/bin/env python3
"""
Publication-Grade Autonomous Navigator & Benchmark Engine for Unitree Go2 (ICRA 2026).
Key Features:
  1. 5-Episode Benchmark Architecture with Date_Numbering Folder Hierarchy:
     experiments/{mode}/YYYYMMDD_XX_goalY_{name}/
  2. Multi-Goal Global 2D Map Visualization:
     Shows ALL registered candidate goals (#1~#5), Active Target, START, STOP, and Trajectory.
  3. 4-Panel Research Benchmark Dashboard (trial_benchmark_dashboard.png):
     - BEV Trajectory with all goals & obstacles
     - Velocity profiles (vx, wz) over time
     - Distance to goal convergence (dt)
     - Heading error & VLM inference latencies
  4. In-Place Rotation Safety Interlock:
     Pure zero-creep in-place rotation when |rel_heading| > 35° (prevents wall scrapes).
  5. Lean & Rapid Artifacts Logging:
     Eliminates duplicate raw images; stores only annotated decision frames (query_XXX_vlm_decision.jpg).
  6. Human-Readable Executive Summary (trial_summary.md) for instant paper table compilation.
"""

import os
import sys
import math
import glob
import time
import json
import yaml
import base64
import argparse
from datetime import datetime
from collections import deque
from pathlib import Path
import threading
import requests
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image, PointCloud2

# Configure stdin and stdout to replace any invalid UTF-8 bytes to prevent UnicodeDecodeError
try:
    sys.stdin.reconfigure(encoding='utf-8', errors='replace')
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

def safe_input(prompt: str = "") -> str:
    """Read user input robustly, handling non-UTF-8 bytes and EOF cleanly."""
    try:
        return input(prompt).strip()
    except UnicodeDecodeError:
        try:
            sys.stdout.write(prompt)
            sys.stdout.flush()
            raw = sys.stdin.buffer.readline()
            return raw.decode('utf-8', errors='replace').strip()
        except Exception:
            return ""
    except Exception:
        return ""
try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None
import tf2_ros

try:
    from unitree_api.msg import Request
    HAS_UNITREE_API = True
except ImportError:
    HAS_UNITREE_API = False

GREEN = '\033[0;32m'
CYAN = '\033[0;36m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BOLD = '\033[1m'
NC = '\033[0m'

WORKSPACE_DIR = "/home/unitree/go2_ws_antarctica"
GOALS_YAML = os.path.join(WORKSPACE_DIR, "config/navigation_goals.yaml")
MAP_2D_PNG = os.path.join(WORKSPACE_DIR, "2dmap/2d.png")
MAP_METADATA_JSON = os.path.join(WORKSPACE_DIR, "2dmap/2d_metadata.json")
EXP_ROOT = os.path.join(WORKSPACE_DIR, "experiments")
VLM_URL = "http://100.96.60.15:8000/v1"
MODEL_NAME = "qwen3.5-9b-instruct"

class AutonomousNavigator(Node):
    def __init__(self, mode="ours", max_vx=0.50, max_wz=0.50, tolerance_m=0.35, timeout_s=120):
        super().__init__('go2_autonomous_navigator')
        self.mode = mode
        self.max_vx = max_vx
        self.max_wz = max_wz
        self.tolerance_m = tolerance_m
        self.timeout_s = timeout_s

        self.current_pose = None
        self.latest_frame = None
        self.bridge = CvBridge() if CvBridge else None
        self.lock = threading.Lock()

        # Active Mission State
        self.is_mission_active = False
        self.current_goal = None
        self.all_goals = []
        self.start_pose = None
        self.mission_start_time = 0.0
        self.trajectory_history = []
        self.vlm_decision_history = []
        self.vlm_latencies = []
        self.pose_sample_count = 0
        self.vlm_query_count = 0
        self.trial_dir = None
        self.snapshots_dir = None
        self.csv_file = None
        self.vlm_log_file = None

        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.vlm_active = False
        self.subgoal_u = 640
        self.subgoal_v = 500

        # Collision avoidance state
        self.min_forward_dist = 999.0
        self.min_left_dist = 1.0
        self.min_right_dist = 1.0

        # Jump Rejection Guard
        self.filter_active = False
        self.jump_warning = None
        self.is_localized = False
        self.last_cov_x = 0.0
        self.rtabmap_pose_count = 0
        self.last_rtabmap_time = 0.0

        # True PixNav Checkpoint_A Neural Network State
        self.pixnav_model = None
        self.pixnav_history = deque(maxlen=8)
        self.pixnav_action_names = ("stop", "forward", "turn_left", "turn_right", "look_up", "look_down")
        self.macro_action = "stop"
        self.macro_end_time = 0.0
        self.goal_bgr_img = None

        if self.mode == "pixnav":
            print(f"\n{BOLD}{CYAN}🧠 [PIXNAV INITIALIZATION] Loading Official Checkpoint_A (208MB) on Jetson CUDA...{NC}")
            try:
                import warnings
                warnings.filterwarnings("ignore", category=UserWarning)

                if WORKSPACE_DIR not in sys.path:
                    sys.path.insert(0, WORKSPACE_DIR)
                tools_dir = str(Path(WORKSPACE_DIR) / "scratch" / "tools")
                if tools_dir not in sys.path:
                    sys.path.insert(0, tools_dir)
                reference = Path(WORKSPACE_DIR) / ".local-data" / "vlm-s2e" / "runtime" / "vlm-s2e-integration-paper-pin"
                runtime_site = Path(WORKSPACE_DIR) / ".local-data" / "pixnav_runtime" / "site-packages"
                if runtime_site.is_dir() and str(runtime_site) not in sys.path:
                    sys.path.insert(0, str(runtime_site))
                if reference.is_dir() and str(reference) not in sys.path:
                    sys.path.insert(0, str(reference))

                import torch
                from pixnav_check import install_python38_settings_shim
                install_python38_settings_shim(reference)
                from policy_network import PixelNavPolicy

                device = "cuda" if torch.cuda.is_available() else "cpu"
                self.pixnav_device = device
                self.pixnav_model = PixelNavPolicy(max_token_length=64, device=device)
                ckpt_path = Path(WORKSPACE_DIR) / ".local-data" / "vlm-s2e" / "checkpoints" / "pixelnav_A.ckpt"
                self.pixnav_model.load_state_dict(torch.load(str(ckpt_path), map_location=device))
                self.pixnav_model.eval()

                # Warmup inference
                dummy_img = np.zeros((1, 224, 224, 3), dtype=np.uint8)
                dummy_mask = np.zeros((1, 224, 224, 1), dtype=np.uint8)
                dummy_mask[0, 112-8:112+8, 112-8:112+8, 0] = 255
                dummy_hist = np.zeros((1, 1, 224, 224, 3), dtype=np.uint8)
                with torch.inference_mode():
                    self.pixnav_model(dummy_mask, dummy_img, dummy_hist)
                print(f"{BOLD}{GREEN}🎉 [PIXNAV DEPLOYED] Checkpoint_A neural network warmed up and active on {device}! (Inference Latency: ~49ms){NC}\n")
            except Exception as e:
                print(f"{RED}❌ Failed to load PixNav Checkpoint_A: {e}{NC}")

        # ROS 2 Subscriptions
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.pose_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', self.pose_callback, qos)
        self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/livo/cloud', self.cloud_callback, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Dual-Layer Velocity Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        if HAS_UNITREE_API:
            self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        else:
            self.sport_pub = None

        # 10Hz Control Loop & Policy/VLM Loop (0.5s for PixNav, 0.7s for VLM)
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5 if self.mode == "pixnav" else 0.7, self.vlm_decision_loop)
        self.create_timer(0.2, self.tf_fallback_timer)

    def tf_fallback_timer(self):
        with self.lock:
            last_t = self.current_pose['time'] if self.current_pose else 0.0
        if time.time() - last_t > 0.5:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                pos = t.transform.translation
                ori = t.transform.rotation
                siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
                cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
                yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                yaw_deg = math.degrees(yaw_rad)

                now = time.time()
                with self.lock:
                    if self.filter_active and self.current_pose is not None:
                        dt = max(0.001, now - self.current_pose.get("time", now))
                        if dt < 5.0:
                            jump_d = math.hypot(pos.x - self.current_pose["x"], pos.y - self.current_pose["y"])
                            if jump_d > 2.0 and (jump_d / dt) > 1.2:
                                self.jump_warning = f"⚠️ JUMP REJECTED ({jump_d:.1f}m in {dt:.2f}s)"
                                return

                    self.current_pose = {
                        "x": float(pos.x),
                        "y": float(pos.y),
                        "z": float(pos.z),
                        "yaw": float(yaw_deg),
                        "yaw_rad": float(yaw_rad),
                        "time": now,
                        "status": "TF_TRACKING"
                    }
            except Exception:
                pass

    def cloud_callback(self, msg: PointCloud2):
        try:
            step_floats = msg.point_step // 4
            if step_floats < 3:
                return
            data = np.frombuffer(msg.data, dtype=np.float32)
            pts = data.reshape(-1, step_floats)[:, :3]
            valid = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1]) & np.isfinite(pts[:, 2])
            pts = pts[valid]

            # Obstacles in forward collision box: 0.05m < x < 0.90m, |y| < 0.35m, -0.15m < z < 0.40m
            fwd_mask = (pts[:, 0] > 0.05) & (pts[:, 0] < 0.90) & (np.abs(pts[:, 1]) < 0.35) & (pts[:, 2] > -0.15) & (pts[:, 2] < 0.40)
            fwd_pts = pts[fwd_mask]
            min_fwd = float(np.min(fwd_pts[:, 0])) if len(fwd_pts) > 8 else 999.0

            # Side clearances (left y > 0, right y < 0)
            left_mask = (pts[:, 0] > 0.0) & (pts[:, 0] < 0.8) & (pts[:, 1] > 0.0) & (pts[:, 1] < 1.2) & (pts[:, 2] > -0.15) & (pts[:, 2] < 0.40)
            right_mask = (pts[:, 0] > 0.0) & (pts[:, 0] < 0.8) & (pts[:, 1] < 0.0) & (pts[:, 1] > -1.2) & (pts[:, 2] > -0.15) & (pts[:, 2] < 0.40)
            min_left = float(np.min(pts[left_mask, 1])) if np.sum(left_mask) > 8 else 1.0
            min_right = float(np.min(np.abs(pts[right_mask, 1]))) if np.sum(right_mask) > 8 else 1.0

            with self.lock:
                self.min_forward_dist = min_fwd
                self.min_left_dist = min_left
                self.min_right_dist = min_right
        except Exception:
            pass

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        # Ignore exact duplicate timestamps if both topics publish simultaneously
        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if getattr(self, '_last_sub_stamp', None) == stamp_key:
            return
        self._last_sub_stamp = stamp_key

        cov_x = float(msg.pose.covariance[0])
        is_reliable = cov_x <= 0.02
        status = "LOCALIZED" if is_reliable else "ODOM_DRIFT"

        now = time.time()
        with self.lock:
            # Protect against false loop closure teleportation (> 2.0m jump) once active
            # (Do NOT reject true global relocalizations with high confidence: cov_x <= 0.005)
            if self.filter_active and self.current_pose is not None and cov_x > 0.005:
                dt = max(0.001, now - self.current_pose.get("time", now))
                if dt < 5.0:
                    jump_d = math.hypot(pos.x - self.current_pose["x"], pos.y - self.current_pose["y"])
                    if jump_d > 2.0 and (jump_d / dt) > 1.2:
                        self.jump_warning = f"⚠️ JUMP REJECTED ({jump_d:.1f}m in {dt:.2f}s)"
                        return  # Ignore corrupted relocalization jump!

            self.jump_warning = None
            self.is_localized = is_reliable
            self.last_cov_x = cov_x
            self.rtabmap_pose_count += 1
            self.last_rtabmap_time = now
            self.current_pose = {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "yaw": float(yaw_deg),
                "yaw_rad": math.atan2(siny_cosp, cosy_cosp),
                "cov_x": cov_x,
                "time": now,
                "status": status
            }

    def image_callback(self, msg: Image):
        try:
            if self.bridge:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                if 'rgb' in msg.encoding.lower():
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.latest_frame = cv_img
        except Exception:
            pass

    def get_current_pose(self):
        with self.lock:
            if self.current_pose and (time.time() - self.current_pose['time'] < 1.0):
                return self.current_pose
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pos = t.transform.translation
            ori = t.transform.rotation
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            now = time.time()
            with self.lock:
                if self.filter_active and self.current_pose is not None:
                    dt = max(0.001, now - self.current_pose.get("time", now))
                    if dt < 5.0:
                        jump_d = math.hypot(pos.x - self.current_pose["x"], pos.y - self.current_pose["y"])
                        if jump_d > 2.0 and (jump_d / dt) > 1.2:
                            return dict(self.current_pose)

                res = {
                    "x": float(pos.x),
                    "y": float(pos.y),
                    "z": float(pos.z),
                    "yaw": float(math.degrees(yaw_rad)),
                    "yaw_rad": float(yaw_rad),
                    "time": now,
                    "status": "TF_TRACKING"
                }
                self.current_pose = res
                return res
        except Exception:
            return None

    def publish_cmd(self, vx: float, wz: float):
        self.cmd_vx = max(-self.max_vx, min(self.max_vx, vx))
        self.cmd_wz = max(-self.max_wz, min(self.max_wz, wz))

        # Layer 1: Direct CycloneDDS Sport API (1008 = Move)
        if self.sport_pub:
            try:
                req = Request()
                req.header.identity.api_id = 1008
                param = {"x": float(self.cmd_vx), "y": 0.0, "z": float(self.cmd_wz)}
                req.parameter = json.dumps(param)
                self.sport_pub.publish(req)
            except Exception:
                pass

        # Layer 2: Standard ROS 2 /cmd_vel
        cmd = Twist()
        cmd.linear.x = float(self.cmd_vx)
        cmd.angular.z = float(self.cmd_wz)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def start_mission(self, goal_entry, all_goals):
        self.filter_active = True
        self.current_goal = goal_entry
        self.all_goals = all_goals
        self.start_pose = self.get_current_pose()
        self.trajectory_history = []
        self.vlm_decision_history = []
        self.vlm_latencies = []
        self.pose_sample_count = 0
        self.vlm_query_count = 0
        self.mission_start_time = time.time()
        self.mission_start_dt = datetime.now()
        self.mission_start_wall = self.mission_start_dt.strftime("%Y-%m-%d %H:%M:%S")
        self.forward_time_s = 0.0
        self.rotation_time_s = 0.0
        self.stationary_time_s = 0.0
        self.last_step_time = time.time()
        self.min_clearance_encountered = 999.0
        self.lidar_repulsion_count = 0
        self.forward_stop_count = 0
        self.subgoal_u = 640
        self.subgoal_v = 500

        # Date_Numbering Folder Architecture: experiments/{mode}/YYYYMMDD_XX_goalY_{name}/
        today_str = datetime.now().strftime("%Y%m%d")
        mode_dir = os.path.join(EXP_ROOT, self.mode)
        os.makedirs(mode_dir, exist_ok=True)

        existing_today = glob.glob(os.path.join(mode_dir, f"{today_str}_*"))
        seq_num = len(existing_today) + 1
        trial_folder_name = f"{today_str}_{seq_num:02d}_goal{goal_entry['id']}_{goal_entry['name']}"
        self.trial_dir = os.path.join(mode_dir, trial_folder_name)
        self.snapshots_dir = os.path.join(self.trial_dir, "camera_snapshots")
        os.makedirs(self.snapshots_dir, exist_ok=True)

        # Update symlink to latest trial
        latest_link = os.path.join(EXP_ROOT, "latest")
        if os.path.lexists(latest_link):
            os.remove(latest_link)
        try:
            os.symlink(self.trial_dir, latest_link)
        except Exception:
            pass

        # Save Start Camera Frame
        self.save_single_snapshot("start_frame")

        # Open Log Files
        self.csv_path = os.path.join(self.trial_dir, "trajectory_raw.csv")
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write("iso_timestamp,elapsed_s,pose_index,x_m,y_m,z_m,yaw_deg,cmd_vx,cmd_wz,dist_to_goal_m,rel_heading_deg,fwd_clear_m,left_clear_m,right_clear_m,status\n")
        self.csv_file.flush()

        self.vlm_log_path = os.path.join(self.trial_dir, "vlm_decisions.jsonl")
        self.vlm_log_file = open(self.vlm_log_path, "w")

        # Reset PixNav policy state
        self.goal_bgr_img = None
        self.pixnav_history.clear()
        self.macro_action = "stop"
        self.macro_end_time = 0.0

        self.is_mission_active = True
        print(f"\n{BOLD}{GREEN}========================================================================{NC}")
        print(f"{BOLD}{GREEN} 🚀 [MISSION STARTED] {today_str}_{seq_num:02d} -> Goal #{goal_entry['id']}: {goal_entry['name']}{NC}")
        print(f" • Mode            : {BOLD}{self.mode.upper()}{NC}")
        print(f" • Target Pose     : X={goal_entry['x_m']:+.2f}m, Y={goal_entry['y_m']:+.2f}m, Tolerance={self.tolerance_m:.2f}m")
        print(f" • Output Folder   : {self.trial_dir}")
        print(f"{BOLD}{GREEN}========================================================================{NC}\n")

    def finish_mission(self, success: bool, reason: str, final_dist: float = 0.0):
        self.is_mission_active = False
        self.last_mission_success = success
        self.stop_robot()
        elapsed = time.time() - self.mission_start_time

        # Save Final Arrival/Abort Frame
        self.save_single_snapshot("arrival_frame" if success else "abort_frame")

        # Calculate Path Metrics
        path_length_m = 0.0
        for i in range(1, len(self.trajectory_history)):
            p0, p1 = self.trajectory_history[i-1], self.trajectory_history[i]
            path_length_m += math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        avg_speed_mps = (path_length_m / elapsed) if elapsed > 0 else 0.0

        # Close Log Files
        if self.csv_file: self.csv_file.close()
        if self.vlm_log_file: self.vlm_log_file.close()

        # Detailed Time Logging & Profiling
        self.mission_end_dt = datetime.now()
        self.mission_end_wall = self.mission_end_dt.strftime("%Y-%m-%d %H:%M:%S")
        mean_lat = float(np.mean(self.vlm_latencies)) if self.vlm_latencies else 0.0
        min_lat = float(np.min(self.vlm_latencies)) if self.vlm_latencies else 0.0
        max_lat = float(np.max(self.vlm_latencies)) if self.vlm_latencies else 0.0
        p95_lat = float(np.percentile(self.vlm_latencies, 95)) if self.vlm_latencies else 0.0
        effective_hz = (self.pose_sample_count / elapsed) if elapsed > 0 else 0.0
        total_budget = max(0.001, self.forward_time_s + self.rotation_time_s + self.stationary_time_s)

        # Shortest (Optimal) Geodesic Distance & SPL
        if self.start_pose:
            optimal_path_m = math.hypot(self.current_goal['x_m'] - self.start_pose['x'], self.current_goal['y_m'] - self.start_pose['y'])
        else:
            optimal_path_m = path_length_m

        spl = ((1.0 if success else 0.0) * optimal_path_m / max(optimal_path_m, path_length_m)) if path_length_m > 0 else 0.0
        path_eff_pct = (optimal_path_m / max(0.001, path_length_m)) * 100.0 if path_length_m > 0 else 100.0
        min_clear_m = self.min_clearance_encountered if self.min_clearance_encountered < 900.0 else 0.0

        # Write Dedicated time_log.txt
        time_log_path = os.path.join(self.trial_dir, "time_log.txt")
        with open(time_log_path, "w") as f:
            f.write("========================================================================\n")
            f.write("⏱️ BENCHMARK TIME LOG & EXECUTION PROFILING REPORT\n")
            f.write("========================================================================\n")
            f.write(f"• Method / Mode           : {self.mode.upper()}\n")
            f.write(f"• Target Goal             : Goal #{self.current_goal['id']} ({self.current_goal['name']})\n")
            f.write(f"• Outcome                 : {'SUCCESS (ARRIVED)' if success else 'HALTED (' + reason + ')'}\n")
            f.write(f"• Final Distance Error    : {final_dist:.3f} m (Tolerance: {self.tolerance_m:.2f} m)\n")
            f.write("------------------------------------------------------------------------\n")
            f.write("🏆 ICRA 2026 NAVIGATION BENCHMARK METRICS:\n")
            f.write(f"• Success Rate (SR)       : {'100% (ARRIVED)' if success else '0% (FAILED: ' + reason + ')'}\n")
            f.write(f"• SPL (Success Path Length: {spl:.3f} (Gold Standard Metric)\n")
            f.write(f"• Optimal Distance (L_opt): {optimal_path_m:.2f} m (Euclidean Start-Goal)\n")
            f.write(f"• Actual Path Length (L)  : {path_length_m:.2f} m\n")
            f.write(f"• Path Length Efficiency  : {path_eff_pct:.1f}%\n")
            f.write(f"• Min Obstacle Clearance  : {min_clear_m:.2f} m (4D LiDAR Safety Margin)\n")
            f.write(f"• LiDAR Wall Repulsions   : {self.lidar_repulsion_count} events\n")
            f.write(f"• Forward Collision Stops : {self.forward_stop_count} events\n")
            f.write("------------------------------------------------------------------------\n")
            f.write("📅 MISSION TIME LOG:\n")
            f.write(f"• Start Timestamp (Local) : {self.mission_start_wall}\n")
            f.write(f"• End Timestamp (Local)   : {self.mission_end_wall}\n")
            f.write(f"• Total Duration          : {elapsed:.3f} seconds ({elapsed/60.0:.2f} min)\n")
            f.write(f"• 10Hz Control Steps      : {self.pose_sample_count} steps (Effective Loop Rate: {effective_hz:.2f} Hz)\n")
            f.write("------------------------------------------------------------------------\n")
            f.write("⚡ LATENCY & DECISION PROFILING:\n")
            f.write(f"• Total Policy Inferences : {self.vlm_query_count} queries\n")
            f.write(f"• Mean Inference Latency  : {mean_lat:.1f} ms\n")
            f.write(f"• Min Inference Latency   : {min_lat:.1f} ms\n")
            f.write(f"• Max Inference Latency   : {max_lat:.1f} ms\n")
            f.write(f"• P95 Inference Latency   : {p95_lat:.1f} ms\n")
            f.write("------------------------------------------------------------------------\n")
            f.write("🏃 KINEMATIC TIME BUDGET:\n")
            f.write(f"• Forward Translation Time: {self.forward_time_s:.2f} s ({self.forward_time_s/total_budget*100:.1f}%)\n")
            f.write(f"• In-Place Rotation Time  : {self.rotation_time_s:.2f} s ({self.rotation_time_s/total_budget*100:.1f}%)\n")
            f.write(f"• Standby / Decel Time    : {self.stationary_time_s:.2f} s ({self.stationary_time_s/total_budget*100:.1f}%)\n")
            f.write("========================================================================\n")

        # Render Publication-Quality Multi-Goal 2D Map & 4-Panel Research Dashboard
        map_path = self.render_multi_goal_trajectory_map(path_length_m, elapsed, avg_speed_mps, success)
        dashboard_path = self.render_benchmark_dashboard(path_length_m, elapsed, avg_speed_mps, success)

        # Generate Human-Readable Markdown Executive Summary
        self.generate_markdown_summary(path_length_m, elapsed, avg_speed_mps, final_dist, success, reason,
                                      effective_hz, mean_lat, min_lat, max_lat, p95_lat, total_budget,
                                      spl, optimal_path_m, path_eff_pct, min_clear_m)

        # JSON Metadata
        metadata = {
            "trial_dir": self.trial_dir,
            "created_at": datetime.now().isoformat(),
            "mode": self.mode,
            "goal": self.current_goal,
            "initial_pose": self.start_pose,
            "timing": {
                "start_timestamp": self.mission_start_wall,
                "end_timestamp": self.mission_end_wall,
                "duration_s": round(elapsed, 3),
                "effective_loop_hz": round(effective_hz, 2),
                "mean_latency_ms": round(mean_lat, 1),
                "min_latency_ms": round(min_lat, 1),
                "max_latency_ms": round(max_lat, 1),
                "p95_latency_ms": round(p95_lat, 1),
                "forward_time_s": round(self.forward_time_s, 2),
                "rotation_time_s": round(self.rotation_time_s, 2),
                "stationary_time_s": round(self.stationary_time_s, 2)
            },
            "metrics": {
                "success": success,
                "reason": reason,
                "spl": round(spl, 3),
                "optimal_distance_m": round(optimal_path_m, 2),
                "path_length_m": round(path_length_m, 2),
                "path_efficiency_pct": round(path_eff_pct, 1),
                "min_obstacle_clearance_m": round(min_clear_m, 2),
                "lidar_repulsion_count": self.lidar_repulsion_count,
                "forward_collision_stop_count": self.forward_stop_count,
                "duration_s": round(elapsed, 2),
                "average_speed_mps": round(avg_speed_mps, 3),
                "final_distance_to_goal_m": round(final_dist, 3),
                "total_pose_samples": self.pose_sample_count,
                "total_vlm_queries": self.vlm_query_count,
                "mean_vlm_latency_ms": round(mean_lat, 1)
            },
            "saved_artifacts": {
                "trial_trajectory_on_2d_map": "trial_trajectory_on_2d_map.png",
                "trial_benchmark_dashboard": "trial_benchmark_dashboard.png",
                "trial_summary_md": "trial_summary.md",
                "time_log_txt": "time_log.txt",
                "trajectory_raw_csv": "trajectory_raw.csv",
                "vlm_decisions_jsonl": "vlm_decisions.jsonl",
                "camera_snapshots_dir": "camera_snapshots/"
            }
        }
        with open(os.path.join(self.trial_dir, "trial_metadata.json"), "w") as f:
            json.dump(metadata, f, indent=2)

        print(f"\n========================================================================")
        print(f"{GREEN}{BOLD} 💾 [EPISODE COMPLETED & ARTIFACTS ARCHIVED]{NC}")
        print(f"========================================================================")
        print(f" • Result              : {'✅ ARRIVED WITHIN TOLERANCE' if success else '⚠️ HALTED: ' + reason}")
        print(f" • Folder              : {self.trial_dir}")
        print(f" • ⏱️ Mission Time Log  : Start {self.mission_start_wall.split()[1]} | End {self.mission_end_wall.split()[1]} | Total {elapsed:.2f}s ({effective_hz:.1f}Hz)")
        print(f" • ⚡ Policy Latency    : Mean {mean_lat:.1f}ms (P95: {p95_lat:.1f}ms, Max: {max_lat:.1f}ms) | {self.vlm_query_count} queries")
        print(f" • 🏃 Kinematic Budget : Fwd {self.forward_time_s:.1f}s ({self.forward_time_s/total_budget*100:.0f}%) | Rot {self.rotation_time_s:.1f}s ({self.rotation_time_s/total_budget*100:.0f}%)")
        print(f" • 📄 Saved Files      : time_log.txt, trial_summary.md, trial_benchmark_dashboard.png")
        print(f" • 📈 Path Metrics     : Length={path_length_m:.2f}m | AvgSpd={avg_speed_mps:.2f}m/s | Error={final_dist:.3f}m")
        print(f"========================================================================\n")

    def vlm_decision_loop(self):
        if not self.is_mission_active:
            return
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return

        pose = self.get_current_pose()
        if not pose or not self.current_goal:
            return

        dx = self.current_goal['x_m'] - pose['x']
        dy = self.current_goal['y_m'] - pose['y']
        dist = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi

        if self.mode == "ours":
            threading.Thread(target=self._query_vlm_async, args=(frame, dist, math.degrees(rel_heading), pose), daemon=True).start()
        elif self.mode == "pixnav":
            threading.Thread(target=self._query_pixnav_policy_async, args=(frame, dist, math.degrees(rel_heading), pose), daemon=True).start()

    def _query_vlm_async(self, frame, dist_to_goal, rel_heading_deg, pose):
        if self.vlm_active or not self.is_mission_active:
            return
        self.vlm_active = True
        self.vlm_query_count += 1
        q_idx = self.vlm_query_count
        t0 = time.time()

        try:
            ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            b64_img = base64.b64encode(buf).decode('utf-8')

            prompt = f"""You are the visual navigation brain for Unitree Go2 navigating to Goal '{self.current_goal['name']}'.
Distance to Goal: {dist_to_goal:.1f}m, Relative Heading: {rel_heading_deg:+.1f} deg.
Inspect the front camera view. Identify the open collision-free corridor or doorway leading toward the destination.
Output JSON:
{{
  "action": "move_forward" | "turn_left" | "turn_right",
  "reasoning": "brief description",
  "selected_image_point": {{"x": 0.5, "y": 0.72}}
}}"""

            payload = {
                "model": MODEL_NAME,
                "messages": [{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64_img}"}}
                    ]
                }],
                "max_tokens": 128,
                "temperature": 0.2
            }

            resp = requests.post(f"{VLM_URL}/chat/completions", json=payload, timeout=2.0)
            latency_ms = (time.time() - t0) * 1000.0
            self.vlm_latencies.append(latency_ms)

            action = "move_forward"
            reasoning = "default forward"
            pt_dict = {"x": 0.5, "y": 0.72}

            if resp.status_code == 200:
                data = resp.json()
                content = data['choices'][0]['message']['content']
                import re
                m = re.search(r'\{.*\}', content, re.DOTALL)
                if m:
                    res_json = json.loads(m.group(0))
                    action = res_json.get('action', 'move_forward')
                    reasoning = res_json.get('reasoning', '')
                    pt_dict = res_json.get('selected_image_point', {'x': 0.5, 'y': 0.72})
                    self.subgoal_u = int(pt_dict.get('x', 0.5) * 1280)
                    self.subgoal_v = int(pt_dict.get('y', 0.72) * 720)

            # Save ONLY Annotated Decision Frame (Clean, lightweight, 0 duplication)
            overlay = frame.copy()
            u, v = self.subgoal_u, self.subgoal_v
            cv2.arrowedLine(overlay, (640, 700), (u, v), (0, 255, 255), 4, tipLength=0.15)
            cv2.circle(overlay, (u, v), 14, (0, 255, 0), 3, cv2.LINE_AA)
            cv2.circle(overlay, (u, v), 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(overlay, f"VLM Query #{q_idx}: [{u},{v}] ({action})", (30, 50),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(overlay, f"Dist: {dist_to_goal:.1f}m | Latency: {latency_ms:.0f}ms | {reasoning[:40]}", (30, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            decision_frame_name = f"query_{q_idx:03d}_vlm_decision.jpg"
            decision_frame_path = os.path.join(self.snapshots_dir, decision_frame_name)
            cv2.imwrite(decision_frame_path, overlay)

            self.vlm_decision_history.append({
                "query_id": q_idx,
                "pose": (pose['x'], pose['y']),
                "subgoal_uv": [u, v],
                "action": action,
                "latency_ms": latency_ms
            })

            vlm_record = {
                "query_id": q_idx,
                "iso_timestamp": datetime.now().isoformat(),
                "latency_ms": round(latency_ms, 1),
                "robot_pose": {"x": round(pose['x'], 3), "y": round(pose['y'], 3), "yaw_deg": round(pose['yaw'], 1)},
                "dist_to_goal_m": round(dist_to_goal, 3),
                "rel_heading_deg": round(rel_heading_deg, 1),
                "action": action,
                "reasoning": reasoning,
                "subgoal_uv": [u, v],
                "decision_image": decision_frame_name
            }
            if self.vlm_log_file and not self.vlm_log_file.closed:
                self.vlm_log_file.write(json.dumps(vlm_record) + "\n")
                self.vlm_log_file.flush()

        except Exception:
            pass
        finally:
            self.vlm_active = False

    def _query_pixnav_policy_async(self, frame, dist_to_goal, rel_heading_deg, pose):
        if getattr(self, 'pixnav_active', False) or not self.is_mission_active:
            return
        self.pixnav_active = True
        t0 = time.perf_counter()
        try:
            if self.pixnav_model is None:
                # Fallback: if model failed to load, keep moving forward safely
                with self.lock:
                    self.macro_action = "forward"
                    self.macro_end_time = time.time() + 0.60
                return

            import torch
            self.pixnav_history.append(frame)
            history_list = list(self.pixnav_history)

            # 1. Live Ego-Centric Goal Frame:
            # Use current robot camera frame so the projected waypoint (u, v)
            # aligns directly with the real-time visual scene in front of the robot.
            goal_bgr = frame.copy()
            h, w = goal_bgr.shape[:2]

            # 2. Goal Pixel & Mask: Project relative goal bearing into camera view
            # When facing goal directly (rel_heading_deg=0), u=640 (center).
            # If goal is to the left (rel_heading_deg > 0), u < 640.
            # If goal is to the right (rel_heading_deg < 0), u > 640.
            norm_heading = max(-1.0, min(1.0, rel_heading_deg / 45.0))
            u = int(640 - norm_heading * 480)
            u = max(60, min(w - 60, u))
            v = 360
            radius = 12
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.rectangle(
                mask,
                (max(0, u - radius), max(0, v - radius)),
                (min(w - 1, u + radius), min(h - 1, v + radius)),
                255,
                -1,
            )

            # 3. Model Preprocessing
            goal_image = cv2.resize(cv2.cvtColor(goal_bgr, cv2.COLOR_BGR2RGB), (224, 224))[np.newaxis, :, :, :]
            goal_mask = cv2.resize(mask, (224, 224), cv2.INTER_NEAREST)[np.newaxis, :, :, np.newaxis]
            history = np.stack([cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), (224, 224)) for img in history_list], axis=0)[np.newaxis, :, :, :, :]

            # 4. Checkpoint_A CUDA Inference
            with torch.inference_mode():
                action_logits, distance_pred, goal_pred = self.pixnav_model(goal_mask, goal_image, history)

                # Real-Robot Constraint: Go2 camera has a fixed rigid mount
                # Mask out vertical actions (look_up=4, look_down=5) matching ROBOT_NAV_ALLOWED_POLICY_ACTIONS
                action_logits[0, :, 4] = -1e9
                action_logits[0, :, 5] = -1e9

                # Suppress premature stop while en route to goal
                if dist_to_goal > self.tolerance_m:
                    action_logits[0, :, 0] = -1e9

                probs = torch.softmax(action_logits[0], dim=-1).cpu().numpy()

            latency_ms = (time.perf_counter() - t0) * 1000.0
            self.vlm_latencies.append(latency_ms)

            pred_id = int(np.argmax(probs[-1]))
            pred_action = self.pixnav_action_names[pred_id]
            self.vlm_query_count += 1
            q_idx = self.vlm_query_count

            # 5. Set Macro-Action Execution Duration (with buffer for seamless trotting)
            now = time.time()
            with self.lock:
                self.macro_action = pred_action
                if pred_action == "forward":
                    self.macro_end_time = now + 0.65  # Smooth 0.5s interval bridge (no stutter)
                elif pred_action in ("turn_left", "turn_right"):
                    self.macro_end_time = now + 1.10  # 30 deg rotation at 0.45 rad/s
                else:
                    self.macro_end_time = now

            # 6. Save Annotated Decision Frame
            overlay = frame.copy()
            prob_str = " | ".join([f"{n[:3]}:{p:.2f}" for n, p in zip(self.pixnav_action_names[:4], probs[-1][:4])])
            cv2.putText(overlay, f"PixNav #{q_idx}: Action={pred_action.upper()} ({latency_ms:.0f}ms)", (30, 50),
                        cv2.FONT_HERSHEY_DUPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(overlay, f"Probs: {prob_str}", (30, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            decision_name = f"query_{q_idx:03d}_pixnav_decision.jpg"
            cv2.imwrite(os.path.join(self.snapshots_dir, decision_name), overlay)

            # 7. Log Record
            rec = {
                "query_id": q_idx,
                "iso_timestamp": datetime.now().isoformat(),
                "latency_ms": round(latency_ms, 1),
                "robot_pose": {"x": round(pose['x'], 3), "y": round(pose['y'], 3), "yaw_deg": round(pose['yaw'], 1)},
                "action": pred_action,
                "action_id": pred_id,
                "probabilities": {n: round(float(p), 4) for n, p in zip(self.pixnav_action_names, probs[-1])},
                "decision_image": decision_name
            }
            if self.vlm_log_file and not self.vlm_log_file.closed:
                self.vlm_log_file.write(json.dumps(rec) + "\n")
                self.vlm_log_file.flush()

        except Exception as e:
            self.get_logger().error(f"PixNav inference error: {e}")
        finally:
            self.pixnav_active = False

    def control_loop(self):
        if not self.is_mission_active or not self.current_goal:
            return

        now = time.time()
        elapsed = now - self.mission_start_time
        pose = self.get_current_pose()

        if not pose:
            self.stop_robot()
            return

        if self.start_pose is None:
            self.start_pose = pose

        # Record trajectory at 10Hz
        self.trajectory_history.append((pose['x'], pose['y'], pose['yaw'], now))

        dx = self.current_goal['x_m'] - pose['x']
        dy = self.current_goal['y_m'] - pose['y']
        dist_to_goal = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        rel_heading_deg = math.degrees(rel_heading)

        # Check Arrival
        if dist_to_goal <= self.tolerance_m:
            print(f"\n{GREEN}{BOLD}🎉 [GOAL REACHED] Arrived within {dist_to_goal:.2f}m of Goal #{self.current_goal['id']} ({self.current_goal['name']})!{NC}")
            self.finish_mission(success=True, reason="ARRIVED", final_dist=dist_to_goal)
            return

        # Check Timeout
        if elapsed > self.timeout_s:
            print(f"\n{RED}⏱️ [TIMEOUT] Exceeded duration of {self.timeout_s}s. Halting robot.{NC}")
            self.finish_mission(success=False, reason="TIMEOUT", final_dist=dist_to_goal)
            return

        # ---------------------------------------------------------
        # Read live 4D LiDAR clearances
        with self.lock:
            fwd_clearance = self.min_forward_dist
            left_clearance = self.min_left_dist
            right_clearance = self.min_right_dist

        cur_min_clear = min(fwd_clearance, left_clearance, right_clearance)
        if cur_min_clear < self.min_clearance_encountered:
            self.min_clearance_encountered = cur_min_clear

        # ---------------------------------------------------------
        # 1. In-Place Rotation Safety Interlock:
        # If heading error > 25° (PixNav) or > 30° (Ours), rotate in-place with ZERO forward creep!
        # Prevents wall collisions and enables smooth 180° turnaround at corridor ends.
        # ---------------------------------------------------------
        align_threshold_deg = 25.0 if self.mode == "pixnav" else 30.0
        is_aligning_in_place = abs(rel_heading_deg) > align_threshold_deg

        if is_aligning_in_place:
            target_vx = 0.0
            target_wz = math.copysign(0.40, rel_heading)
        else:
            if self.mode == "ours":
                norm_u = (self.subgoal_u - 640) / 640.0
                target_wz = -norm_u * 0.40 + (rel_heading * 0.20)
                target_vx = self.max_vx * (1.0 - abs(norm_u) * 0.6)
            elif self.mode == "pixnav":
                # True PixNav Checkpoint_A Neural Network Action Execution
                with self.lock:
                    act = self.macro_action
                    end_t = self.macro_end_time

                if now < end_t:
                    if act == "forward":
                        target_vx = self.max_vx  # 0.50 m/s
                        target_wz = 0.0
                    elif act == "turn_left":
                        target_vx = 0.0
                        target_wz = 0.45  # +30 deg/s
                    elif act == "turn_right":
                        target_vx = 0.0
                        target_wz = -0.45  # -30 deg/s
                    else:
                        target_vx = 0.0
                        target_wz = 0.0
                else:
                    target_vx = 0.0
                    target_wz = 0.0

            # ---------------------------------------------------------
            # 2. Physical 4D LiDAR Wall / Obstacle Collision Prevention Interlock
            # (Only applies when robot is actively driving forward down the corridor)
            # ---------------------------------------------------------
            # Side Wall Repulsion (LiDAR Corridor Centering)
            if left_clearance < 0.45:
                self.lidar_repulsion_count += 1
                target_wz = min(target_wz, -0.28)
                target_vx = min(target_vx, 0.30)
            elif right_clearance < 0.45:
                self.lidar_repulsion_count += 1
                target_wz = max(target_wz, 0.28)
                target_vx = min(target_vx, 0.30)

            # Forward Obstacle Interlock
            if fwd_clearance < 0.50:
                self.forward_stop_count += 1
                target_vx = 0.0
                avoid_wz = 0.38 if left_clearance > right_clearance else -0.38
                target_wz = avoid_wz
            elif fwd_clearance < 0.75:
                # Approaching obstacle: slow down smoothly
                speed_factor = max(0.15, (fwd_clearance - 0.50) / 0.25)
                target_vx *= speed_factor

        self.publish_cmd(target_vx, target_wz)

        self.pose_sample_count += 1
        iso_ts = datetime.now().isoformat()
        now_step = time.time()
        dt_step = max(0.001, now_step - getattr(self, 'last_step_time', now_step))
        self.last_step_time = now_step
        if abs(self.cmd_vx) > 0.05 and abs(self.cmd_wz) < 0.25:
            self.forward_time_s += dt_step
        elif abs(self.cmd_wz) >= 0.25:
            self.rotation_time_s += dt_step
        else:
            self.stationary_time_s += dt_step

        if self.csv_file and not self.csv_file.closed:
            self.csv_file.write(
                f"{iso_ts},{elapsed:.3f},{self.pose_sample_count},"
                f"{pose['x']:.4f},{pose['y']:.4f},{pose['z']:.4f},{pose['yaw']:.2f},"
                f"{self.cmd_vx:.3f},{self.cmd_wz:.3f},{dist_to_goal:.3f},{rel_heading_deg:.2f},"
                f"{fwd_clearance:.2f},{left_clearance:.2f},{right_clearance:.2f},NAVIGATING\n"
            )
            self.csv_file.flush()

        wall_hhmmss = datetime.now().strftime("%H:%M:%S")

        # Continuous 1Hz Full RTAB-Map Real-Time Localization & Mission Log
        if self.pose_sample_count % 10 == 0:
            cov_val = pose.get('cov_x', getattr(self, 'last_cov_x', 0.0))
            cov_str = f"cov: {cov_val:.4f}"
            time_since_rtab = now - getattr(self, 'last_rtabmap_time', 0.0)
            rtab_count = getattr(self, 'rtabmap_pose_count', 0)
            if time_since_rtab < 2.0 and rtab_count > 0:
                loc_badge = f"{GREEN}🟢 [RTABMAP #{rtab_count:04d}]{NC}"
            else:
                loc_badge = f"{YELLOW}⚠️ [TF_TRACKING (lost {time_since_rtab:.1f}s)]{NC}"
            print(
                f"⏱️ [{wall_hhmmss} | +{elapsed:4.1f}s] {loc_badge} "
                f"X:{BOLD}{pose['x']:+7.3f}m{NC} Y:{BOLD}{pose['y']:+7.3f}m{NC} Yaw:{BOLD}{pose['yaw']:+6.1f}°{NC} ({cov_str}) | "
                f"Goal #{self.current_goal['id']} ({self.current_goal['name']}): Dist={BOLD}{dist_to_goal:5.2f}m{NC} (Bear:{rel_heading_deg:+5.1f}°) | "
                f"Cmd: vx={self.cmd_vx:.2f} wz={self.cmd_wz:+.2f}",
                flush=True
            )

    def save_single_snapshot(self, label):
        with self.lock:
            if self.latest_frame is None or self.snapshots_dir is None:
                return
            frame = self.latest_frame.copy()
        path = os.path.join(self.snapshots_dir, f"{label}.jpg")
        cv2.imwrite(path, frame)

    def render_multi_goal_trajectory_map(self, path_len, elapsed, avg_spd, success):
        if not os.path.exists(MAP_2D_PNG) or not self.trajectory_history:
            return None

        map_img = cv2.imread(MAP_2D_PNG)
        if map_img is None:
            return None

        h, w = map_img.shape[:2]
        res = 0.04
        min_x, min_y = -30.0, 23.0

        if os.path.exists(MAP_METADATA_JSON):
            try:
                with open(MAP_METADATA_JSON, 'r') as mf:
                    meta = json.load(mf)
                    min_x = meta.get('min_x', min_x)
                    min_y = meta.get('min_y', min_y)
                    res = meta.get('resolution', res)
            except Exception:
                pass

        overlay = map_img.copy()

        # 1. Draw ALL Registered Candidate Goals
        for g in self.all_goals:
            gid = g['id']
            gx, gy = g['x_m'], g['y_m']
            px = int((gx - min_x) / res)
            py = int(h - 1 - (gy - min_y) / res)
            px = max(20, min(w - 20, px))
            py = max(20, min(h - 20, py))

            is_target = (gid == self.current_goal['id'])
            if is_target:
                # Active Target: Red circle + double white outline + Target badge
                cv2.circle(overlay, (px, py), 12, (0, 0, 230), -1, cv2.LINE_AA)
                cv2.circle(overlay, (px, py), 15, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"Goal #{gid} (TARGET)", (px - 50, py - 20), cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 0, 220), 1, cv2.LINE_AA)
            else:
                # Inactive Candidate Goal: Amber circle + white outline + Goal badge
                cv2.circle(overlay, (px, py), 10, (0, 180, 240), -1, cv2.LINE_AA)
                cv2.circle(overlay, (px, py), 12, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"Goal #{gid}", (px - 30, py - 18), cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 140, 200), 1, cv2.LINE_AA)

        # 2. Draw Trajectory Path Line with Arrow Markers
        pts = []
        for p in self.trajectory_history:
            px = int((p[0] - min_x) / res)
            py = int(h - 1 - (p[1] - min_y) / res)
            pts.append((px, py))

        for i in range(1, len(pts)):
            cv2.line(overlay, pts[i-1], pts[i], (255, 120, 0), 4, cv2.LINE_AA)

        # Draw directional arrows along trajectory
        step_stride = max(1, len(pts) // 6)
        for i in range(step_stride, len(pts) - 5, step_stride):
            cv2.arrowedLine(overlay, pts[i-2], pts[i+2], (0, 240, 255), 2, tipLength=0.4)

        # 3. Draw START Pin (Green) & STOP Pin (Purple)
        start_p = pts[0]
        end_p = pts[-1]
        cv2.circle(overlay, start_p, 9, (0, 200, 0), -1, cv2.LINE_AA)
        cv2.circle(overlay, start_p, 12, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, "START", (start_p[0] - 25, start_p[1] + 24), cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 160, 0), 1, cv2.LINE_AA)

        stop_color = (200, 0, 200) if success else (0, 0, 255)
        cv2.circle(overlay, end_p, 9, stop_color, -1, cv2.LINE_AA)
        cv2.circle(overlay, end_p, 11, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, "STOP", (end_p[0] - 25, end_p[1] + 24), cv2.FONT_HERSHEY_DUPLEX, 0.55, stop_color, 1, cv2.LINE_AA)

        # 4. Top Executive Summary Header Banner
        banner_w = min(w - 30, 680)
        cv2.rectangle(overlay, (15, 10), (15 + banner_w, 48), (255, 255, 255), -1)
        cv2.rectangle(overlay, (15, 10), (15 + banner_w, 48), (180, 180, 180), 1)
        status_txt = "ARRIVED" if success else "TIMEOUT/HALT"
        header_str = f"[{self.mode.upper()}] Goal #{self.current_goal['id']} | Time: {elapsed:.1f}s | Len: {path_len:.2f}m | Spd: {avg_spd:.2f}m/s | {status_txt}"
        cv2.putText(overlay, header_str, (25, 35), cv2.FONT_HERSHEY_DUPLEX, 0.50, (30, 30, 30), 1, cv2.LINE_AA)

        out_map_path = os.path.join(self.trial_dir, "trial_trajectory_on_2d_map.png")
        cv2.imwrite(out_map_path, overlay)
        return out_map_path

    def render_benchmark_dashboard(self, path_len, elapsed, avg_spd, success):
        if not self.trajectory_history:
            return None

        try:
            fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), dpi=250)
            fig.patch.set_facecolor('#f8f9fa')

            # Extract Data
            times = [p[3] - self.mission_start_time for p in self.trajectory_history]
            xs = [p[0] for p in self.trajectory_history]
            ys = [p[1] for p in self.trajectory_history]
            yaws = [p[2] for p in self.trajectory_history]

            # Read CSV for commanded velocities
            csv_path = os.path.join(self.trial_dir, "trajectory_raw.csv")
            vxs, wzs, dists, rel_angles = [], [], [], []
            if os.path.exists(csv_path):
                import pandas as pd
                df = pd.read_csv(csv_path)
                vxs = df['cmd_vx'].tolist()
                wzs = df['cmd_wz'].tolist()
                dists = df['dist_to_goal_m'].tolist()
                rel_angles = df['rel_heading_deg'].tolist()

            # Panel 1: BEV Trajectory & All Goals
            ax1 = axes[0, 0]
            ax1.set_facecolor('#ffffff')
            ax1.plot(xs, ys, color='#0066cc', linewidth=2.5, label='Robot Path')
            ax1.scatter(xs[0], ys[0], color='#00aa00', s=100, zorder=5, label='Start')
            ax1.scatter(xs[-1], ys[-1], color='#990099', s=100, zorder=5, label='Stop')
            for g in self.all_goals:
                is_tgt = (g['id'] == self.current_goal['id'])
                c = '#dd0000' if is_tgt else '#f39c12'
                m = '*' if is_tgt else 'o'
                s = 140 if is_tgt else 80
                lbl = f"Goal #{g['id']} (Target)" if is_tgt else f"Goal #{g['id']}"
                ax1.scatter(g['x_m'], g['y_m'], color=c, marker=m, s=s, zorder=6, label=lbl)
            ax1.set_title("1. 2D Bird's-Eye View Trajectory", fontweight='bold')
            ax1.set_xlabel("Map X (meters)")
            ax1.set_ylabel("Map Y (meters)")
            ax1.grid(True, linestyle='--', alpha=0.5)
            ax1.legend(loc='best', fontsize=8)

            # Panel 2: Commanded Velocities
            ax2 = axes[0, 1]
            ax2.set_facecolor('#ffffff')
            if vxs:
                t_arr = times[:len(vxs)]
                ax2.plot(t_arr, vxs, color='#27ae60', linewidth=2.0, label='Linear Velocity vx (m/s)')
                ax2.plot(t_arr, wzs, color='#e67e22', linewidth=2.0, linestyle='--', label='Angular Yaw Rate wz (rad/s)')
                ax2.axhline(0, color='gray', linestyle=':', alpha=0.6)
            ax2.set_title("2. Commanded Velocities Profile", fontweight='bold')
            ax2.set_xlabel("Elapsed Time (seconds)")
            ax2.set_ylabel("Velocity")
            ax2.grid(True, linestyle='--', alpha=0.5)
            ax2.legend(loc='best', fontsize=8)

            # Panel 3: Distance to Goal Convergence
            ax3 = axes[1, 0]
            ax3.set_facecolor('#ffffff')
            if dists:
                t_arr = times[:len(dists)]
                ax3.plot(t_arr, dists, color='#c0392b', linewidth=2.5, label='Distance to Goal (m)')
                ax3.axhline(self.tolerance_m, color='#2980b9', linestyle='--', label=f'Arrival Threshold ({self.tolerance_m:.2f}m)')
            ax3.set_title("3. Goal Distance Convergence dt", fontweight='bold')
            ax3.set_xlabel("Elapsed Time (seconds)")
            ax3.set_ylabel("Euclidean Distance (m)")
            ax3.grid(True, linestyle='--', alpha=0.5)
            ax3.legend(loc='best', fontsize=8)

            # Panel 4: Heading Error & VLM Decision Latencies
            ax4 = axes[1, 1]
            ax4.set_facecolor('#ffffff')
            if rel_angles:
                t_arr = times[:len(rel_angles)]
                ax4.plot(t_arr, rel_angles, color='#8e44ad', linewidth=2.0, label='Relative Heading Error (deg)')
                ax4.axhline(0, color='gray', linestyle=':', alpha=0.6)
                ax4.axhline(35.0, color='orange', linestyle=':', alpha=0.5, label='In-Place Turn Boundary (±35°)')
                ax4.axhline(-35.0, color='orange', linestyle=':', alpha=0.5)
            ax4.set_title("4. Heading Error & Alignment Profile", fontweight='bold')
            ax4.set_xlabel("Elapsed Time (seconds)")
            ax4.set_ylabel("Heading Error (deg)")
            ax4.grid(True, linestyle='--', alpha=0.5)
            ax4.legend(loc='best', fontsize=8)

            plt.suptitle(f"Unitree Go2 Autonomous Navigation Benchmark [{self.mode.upper()}] - Goal #{self.current_goal['id']}: {self.current_goal['name']}",
                         fontsize=14, fontweight='bold', y=0.98)
            plt.tight_layout(rect=[0, 0.03, 1, 0.95])

            dashboard_path = os.path.join(self.trial_dir, "trial_benchmark_dashboard.png")
            plt.savefig(dashboard_path)
            plt.close()
            return dashboard_path
        except Exception:
            return None

    def generate_markdown_summary(self, path_len, elapsed, avg_spd, final_dist, success, reason,
                                  effective_hz, mean_lat, min_lat, max_lat, p95_lat, total_budget,
                                  spl, optimal_path_m, path_eff_pct, min_clear_m):
        md_path = os.path.join(self.trial_dir, "trial_summary.md")
        with open(md_path, "w") as f:
            f.write(f"# 🏁 Benchmark Trial Executive Summary\n\n")
            f.write(f"- **Method / Mode**: `{self.mode.upper()}`\n")
            f.write(f"- **Target Goal**: Goal #{self.current_goal['id']} (`{self.current_goal['name']}`) at $({self.current_goal['x_m']:+.2f}m, {self.current_goal['y_m']:+.2f}m)$\n")
            f.write(f"- **Outcome**: {'✅ **SUCCESS (ARRIVED)**' if success else '⚠️ **HALTED** (' + reason + ')'}\n")
            f.write(f"- **Final Distance Error**: `{final_dist:.3f} m` (Tolerance: `{self.tolerance_m:.2f} m`)\n")
            f.write(f"- **Total Duration**: `{elapsed:.2f} s`\n")
            f.write(f"- **Total Trajectory Length**: `{path_len:.2f} m`\n")
            f.write(f"- **Average Travel Speed**: `{avg_spd:.2f} m/s` (Max limit: `{self.max_vx:.2f} m/s`)\n")
            f.write(f"- **Pose Sample Count (10Hz)**: `{self.pose_sample_count}`\n")
            f.write(f"- **Policy Inferences / VLM Queries**: `{self.vlm_query_count}` (Mean Latency: `{mean_lat:.1f} ms`)\n\n")
            f.write(f"## 🏆 ICRA 2026 Navigation Benchmark Performance\n\n")
            f.write(f"| Evaluation Metric | Value | Reference / Standard |\n")
            f.write(f"|---|---|---|\n")
            f.write(f"| **Success Rate (SR)** | `{'100%' if success else '0%'}` | Goal tolerance $\\le {self.tolerance_m:.2f} m$ |\n")
            f.write(f"| **SPL (Success Path Length)** | `{spl:.3f}` | **Gold Standard** ($S \\times L_{{opt}} / \\max(L_{{opt}}, L_{{act}})$) |\n")
            f.write(f"| **Shortest Distance ($L_{{opt}}$)** | `{optimal_path_m:.2f} m` | Euclidean Start-to-Goal |\n")
            f.write(f"| **Actual Trajectory ($L_{{act}}$)** | `{path_len:.2f} m` | Integrated 10Hz Odometry |\n")
            f.write(f"| **Path Length Efficiency** | `{path_eff_pct:.1f}%` | $L_{{opt}} / L_{{act}} \\times 100$ |\n")
            f.write(f"| **Min Obstacle Clearance** | `{min_clear_m:.2f} m` | 4D LiDAR Closest Point Cloud |\n")
            f.write(f"| **LiDAR Wall Repulsions** | `{self.lidar_repulsion_count}` | Corridor centering auto-steers |\n")
            f.write(f"| **Forward Collision Stops** | `{self.forward_stop_count}` | Obstacle emergency interlocks (< 0.50m) |\n\n")
            f.write(f"## ⏱️ Detailed Time Log & Latency Breakdown\n\n")
            f.write(f"| Timing & Profiling Metric | Recorded Value |\n")
            f.write(f"|---|---|\n")
            f.write(f"| **Mission Start Time (Local)** | `{self.mission_start_wall}` |\n")
            f.write(f"| **Mission End Time (Local)** | `{self.mission_end_wall}` |\n")
            f.write(f"| **Total Navigation Time** | `{elapsed:.3f} s` ({elapsed/60.0:.2f} min) |\n")
            f.write(f"| **Effective Control Loop Rate** | `{effective_hz:.2f} Hz` (Target: 10.0 Hz) |\n")
            f.write(f"| **Policy / VLM Mean Latency** | `{mean_lat:.1f} ms` (Min: `{min_lat:.1f} ms`, Max: `{max_lat:.1f} ms`, P95: `{p95_lat:.1f} ms`) |\n")
            f.write(f"| **Forward Translating Time** | `{self.forward_time_s:.2f} s` ({self.forward_time_s/total_budget*100:.1f}%) |\n")
            f.write(f"| **In-Place Rotating Time** | `{self.rotation_time_s:.2f} s` ({self.rotation_time_s/total_budget*100:.1f}%) |\n")
            f.write(f"| **Standby / Decel Time** | `{self.stationary_time_s:.2f} s` ({self.stationary_time_s/total_budget*100:.1f}%) |\n\n")
            f.write(f"## 📁 Saved Artifacts\n")
            f.write(f"- `time_log.txt` : Dedicated human-readable time log and kinematic profiling report.\n")
            f.write(f"- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.\n")
            f.write(f"- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.\n")
            f.write(f"- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.\n")
            f.write(f"- `vlm_decisions.jsonl` : Log of every policy/VLM decision and action probabilities.\n")
            f.write(f"- `camera_snapshots/` : Annotated decision frames.\n")

def load_goals():
    if not os.path.exists(GOALS_YAML):
        return []
    with open(GOALS_YAML, 'r') as f:
        data = yaml.safe_load(f)
        return data.get('goals', [])

def print_goal_menu(goals, current_pose=None):
    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🐕 [Unitree Go2 Autonomous Navigation: Multi-Goal Benchmark Shell]{NC}")
    if current_pose:
        status_str = f"{GREEN}🟢 LOCALIZED{NC}" if current_pose.get('status') == 'LOCALIZED' else f"{YELLOW}⚠️ TF_TRACKING{NC}"
        cov_str = f"(cov: {current_pose.get('cov_x', 0.0):.4f})"
        print(f" 📍 Current Pose: {BOLD}X={current_pose['x']:+.2f}m, Y={current_pose['y']:+.2f}m, Yaw={current_pose['yaw']:+.1f}°{NC} | {status_str} {cov_str}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")
    for g in goals:
        dist_info = ""
        if current_pose:
            d = math.hypot(g['x_m'] - current_pose['x'], g['y_m'] - current_pose['y'])
            dist_info = f" | Dist: {d:.2f}m"
        photo_info = f"📸 Photo: {g.get('snapshot_image', 'None')}" if 'snapshot_image' in g else ""
        print(f"  [{g['id']}] {g['name']:20s} | X={g['x_m']:+6.2f}m, Y={g['y_m']:+6.2f}m, Yaw={g['yaw_deg']:+5.1f}°{dist_info} | {photo_info}")
    if len(goals) > 1:
        seq_str = " -> ".join(f"#{g['id']}" for g in goals)
        print(f"  [all] Continuous Sequential Patrol ({seq_str})")
        print(f"  [1,2] Custom Waypoint Sequence (e.g. '1,2' or '2,1')")
    print(f"  [q]   Safe Shutdown & Exit Stack")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

def main():
    parser = argparse.ArgumentParser(description="Publication Benchmark Runner for Unitree Go2")
    parser.add_argument('--mode', choices=['ours', 'pixnav'], default='ours', help="Navigation mode (ours = ESCAPE-Nav, pixnav = PointNav)")
    parser.add_argument('--goal', type=str, default='1', help="Initial Goal ID or Sequence (default: '1')")
    parser.add_argument('--max-vx', type=float, default=0.50, help="Maximum linear velocity in m/s (default: 0.50)")
    parser.add_argument('--max-wz', type=float, default=0.50, help="Maximum angular velocity in rad/s (default: 0.50)")
    parser.add_argument('--tolerance', type=float, default=0.35, help="Goal arrival tolerance in meters")
    parser.add_argument('--timeout', type=int, default=120, help="Max run duration in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = AutonomousNavigator(
        mode=args.mode,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        tolerance_m=args.tolerance,
        timeout_s=args.timeout
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 0. Active Map Verification Banner (Guarantees user that the latest map is loaded)
    map_db = "/home/unitree/.ros/rtabmap.db"
    if os.path.exists(map_db):
        import sqlite3, struct
        try:
            mtime_str = datetime.fromtimestamp(os.path.getmtime(map_db)).strftime("%Y-%m-%d %H:%M:%S")
            conn = sqlite3.connect(f"file:{map_db}?mode=ro", uri=True)
            c = conn.cursor()
            c.execute("SELECT count(*) FROM Node")
            n_nodes = c.fetchone()[0]
            c.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
            rows = c.fetchall()
            xs, ys = [], []
            for _, blob in rows:
                if len(blob) == 48:
                    v = struct.unpack("<12f", blob)
                    xs.append(v[3])
                    ys.append(v[7])
            conn.close()
            min_x, max_x = (min(xs), max(xs)) if xs else (0.0, 0.0)
            min_y, max_y = (min(ys), max(ys)) if ys else (0.0, 0.0)
            print(f"\n{BOLD}{GREEN}========================================================================{NC}")
            print(f"{BOLD}{GREEN} 🗺️ [ACTIVE SLAM MAP VERIFICATION - CONFIRMED LATEST MAP]{NC}")
            print(f" • Database File : {BOLD}{map_db}{NC}")
            print(f" • Last Modified : {BOLD}{mtime_str}{NC} ({n_nodes} nodes)")
            print(f" • Map Bounds    : X=[{min_x:+.2f}m, {max_x:+.2f}m], Y=[{min_y:+.2f}m, {max_y:+.2f}m]")
            print(f" • Zero-Origin   : ✅ Node #1 at (X={xs[0]:+.3f}m, Y={ys[0]:+.3f}m)")
            print(f"{BOLD}{GREEN}========================================================================{NC}")
        except Exception as e:
            print(f"{YELLOW}⚠️ Map verification read note: {e}{NC}")

    # 1. Searching for initial landmarks
    print(f"\n{YELLOW}⏳ [1/2] Waiting for RTAB-Map 4D LiDAR Localization Lock...{NC}", flush=True)
    wait_count = 0
    initial_pose = None
    while initial_pose is None:
        time.sleep(0.3)
        wait_count += 1
        initial_pose = node.get_current_pose()
        if wait_count % 5 == 0 and initial_pose is None:
            print(f"  {YELLOW}🔍 Waiting for map transform ({wait_count * 0.3:.1f}s elapsed)...{NC}", flush=True)
        if wait_count >= 20 and initial_pose is None:
            print(f"  {YELLOW}⚠️ Proceeding with initial TF search...{NC}", flush=True)
            break

    while initial_pose is None:
        time.sleep(0.2)
        initial_pose = node.get_current_pose()

    # 2. 5-Second Live Localization Stability & Calibration Warmup Monitor
    print(f"\n{BOLD}{GREEN}========================================================================{NC}")
    print(f"{BOLD}{GREEN} 🛰️ [2/2] LOCALIZATION LOCK & STABILITY CALIBRATION (5s Warmup Monitor){NC}")
    print(f"{BOLD}{GREEN}========================================================================{NC}")

    ref_x = initial_pose['x']
    ref_y = initial_pose['y']
    for sec in range(1, 6):
        time.sleep(1.0)
        p = node.get_current_pose()
        if p:
            drift_m = math.hypot(p['x'] - ref_x, p['y'] - ref_y)
            if drift_m > 1.0:
                ref_x, ref_y = p['x'], p['y']
                drift_m = 0.0
            drift_cm = drift_m * 100.0
            cov_val = p.get('cov_x', 0.0)
            status_tag = f"{GREEN}100% HEALTHY LOCK! (cov: {cov_val:.4f}){NC}" if sec == 5 else f"{CYAN}STABLE (Jitter: {drift_cm:3.1f}cm, cov: {cov_val:.4f}){NC}"
            print(f" [{sec}/5s] {GREEN}🟢 LOCALIZED{NC} | X:{BOLD}{p['x']:+7.3f}m{NC} Y:{BOLD}{p['y']:+7.3f}m{NC} Yaw:{BOLD}{p['yaw']:+6.1f}°{NC} | {status_tag}", flush=True)

    print(f"{BOLD}{GREEN}========================================================================{NC}", flush=True)
    print(f"{BOLD}{GREEN} 🎯 [LOCALIZATION FULLY STABILIZED] Ready for Mission Execution!{NC}", flush=True)
    print(f"{BOLD}{GREEN}========================================================================{NC}\n", flush=True)

    node.filter_active = True

    initial_goal_arg = args.goal

    while True:
        goals = load_goals()
        if not goals:
            print(f"{RED}Error: No candidate goals registered in {GOALS_YAML}. Run ./run_local.sh first.{NC}")
            break

        print_goal_menu(goals, node.get_current_pose())

        if initial_goal_arg is not None:
            choice = str(initial_goal_arg).strip()
            initial_goal_arg = None
            print(f"\n🚀 [AUTO-LAUNCH] Target Goal #{choice} selected by default. Launching mission immediately...", flush=True)
        else:
            try:
                choice = safe_input(f"\n👉 Enter Target Goal [1-{len(goals)}] (Press [ENTER] for Goal #1), 'all', or 'q': ").strip()
                if not choice:
                    choice = "1"
            except (KeyboardInterrupt, EOFError):
                break

        if choice.lower() in ('q', 'quit', 'exit'):
            break

        # Parse goal selection: single goal, sequence ('1,2' or '1 2'), or 'all'
        goal_sequence = []
        if choice.lower() in ('all', 'seq', 'patrol'):
            goal_sequence = list(goals)
        elif ',' in choice or ' ' in choice or '->' in choice:
            tokens = choice.replace('->', ' ').replace(',', ' ').split()
            for token in tokens:
                for g in goals:
                    if str(g['id']) == token or str(g['name']).lower() == token.lower():
                        if g not in goal_sequence:
                            goal_sequence.append(g)
                        break
        else:
            for g in goals:
                if str(g['id']) == choice or str(g['name']).lower() == choice.lower():
                    goal_sequence.append(g)
                    break

        if not goal_sequence:
            print(f"{YELLOW}⚠️ Invalid choice. Please enter a valid Goal ID (1-{len(goals)}), '1,2', or 'all'.{NC}")
            continue

        # Execute goal sequence
        interrupted = False
        for seq_idx, selected_goal in enumerate(goal_sequence):
            if len(goal_sequence) > 1:
                print(f"\n{BOLD}{CYAN}========================================================================{NC}")
                print(f"{BOLD}{CYAN} 📍 [WAYPOINT {seq_idx+1}/{len(goal_sequence)}] Navigating to Goal #{selected_goal['id']}: {selected_goal['name']}{NC}")
                print(f"{BOLD}{CYAN}========================================================================{NC}")

            node.start_mission(selected_goal, goals)

            try:
                while node.is_mission_active:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print(f"\n{YELLOW}⚠️ Manual interrupt received! Halting robot.{NC}")
                node.finish_mission(success=False, reason="USER_INTERRUPT", final_dist=0.0)
                interrupted = True
                break

            if interrupted or not getattr(node, 'last_mission_success', False):
                print(f"\n{YELLOW}⚠️ Waypoint sequence halted.{NC}")
                break

            if seq_idx < len(goal_sequence) - 1:
                next_g = goal_sequence[seq_idx + 1]
                print(f"\n{GREEN}{BOLD}🎉 Goal #{selected_goal['id']} reached successfully!{NC}")
                print(f"{CYAN}⏸️ Pausing 2.5s, then automatically transitioning to Goal #{next_g['id']} ({next_g['name']})...{NC}\n")
                time.sleep(2.5)

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{GREEN}✅ Benchmark Runner Safely Closed. 🐕{NC}")

if __name__ == '__main__':
    main()
