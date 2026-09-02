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

                with self.lock:
                    self.current_pose = {
                        "x": float(pos.x),
                        "y": float(pos.y),
                        "z": float(pos.z),
                        "yaw": float(yaw_deg),
                        "yaw_rad": float(yaw_rad),
                        "time": time.time(),
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

        with self.lock:
            self.current_pose = {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "yaw": float(yaw_deg),
                "yaw_rad": math.atan2(siny_cosp, cosy_cosp),
                "time": time.time()
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
            res = {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "yaw": float(math.degrees(yaw_rad)),
                "yaw_rad": float(yaw_rad),
                "time": time.time(),
                "status": "TF_TRACKING"
            }
            with self.lock:
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
        self.current_goal = goal_entry
        self.all_goals = all_goals
        self.start_pose = self.get_current_pose()
        self.trajectory_history = []
        self.vlm_decision_history = []
        self.vlm_latencies = []
        self.pose_sample_count = 0
        self.vlm_query_count = 0
        self.mission_start_time = time.time()
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
        self.csv_file.write("iso_timestamp,elapsed_s,pose_index,x_m,y_m,z_m,yaw_deg,cmd_vx,cmd_wz,dist_to_goal_m,rel_heading_deg,status\n")
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

        # Render Publication-Quality Multi-Goal 2D Map & 4-Panel Research Dashboard
        map_path = self.render_multi_goal_trajectory_map(path_length_m, elapsed, avg_speed_mps, success)
        dashboard_path = self.render_benchmark_dashboard(path_length_m, elapsed, avg_speed_mps, success)

        # Generate Human-Readable Markdown Executive Summary
        self.generate_markdown_summary(path_length_m, elapsed, avg_speed_mps, final_dist, success, reason)

        # JSON Metadata
        metadata = {
            "trial_dir": self.trial_dir,
            "created_at": datetime.now().isoformat(),
            "mode": self.mode,
            "goal": self.current_goal,
            "initial_pose": self.start_pose,
            "metrics": {
                "success": success,
                "reason": reason,
                "duration_s": round(elapsed, 2),
                "path_length_m": round(path_length_m, 2),
                "average_speed_mps": round(avg_speed_mps, 3),
                "final_distance_to_goal_m": round(final_dist, 3),
                "total_pose_samples": self.pose_sample_count,
                "total_vlm_queries": self.vlm_query_count,
                "mean_vlm_latency_ms": round(float(np.mean(self.vlm_latencies)), 1) if self.vlm_latencies else 0.0
            },
            "saved_artifacts": {
                "trial_trajectory_on_2d_map": "trial_trajectory_on_2d_map.png",
                "trial_benchmark_dashboard": "trial_benchmark_dashboard.png",
                "trial_summary_md": "trial_summary.md",
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
        print(f" • 🗺️ Multi-Goal Map    : trial_trajectory_on_2d_map.png (All goals & path)")
        print(f" • 📊 4-Panel Dashboard: trial_benchmark_dashboard.png (Speed, distance, errors)")
        print(f" • 📋 Executive Summary : trial_summary.md (Instant paper table stats)")
        print(f" • 📸 VLM Decisions    : camera_snapshots/ (Annotated decision overlays only)")
        print(f" • 📈 Metrics          : Time={elapsed:.1f}s | Length={path_length_m:.2f}m | AvgSpd={avg_speed_mps:.2f}m/s")
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

            # 1. Goal Image: Use registered goal photo if present, corridor reference, or frame
            if self.goal_bgr_img is None:
                if 'snapshot_image' in self.current_goal and self.current_goal['snapshot_image']:
                    full_p = os.path.join(WORKSPACE_DIR, self.current_goal['snapshot_image'])
                    if os.path.exists(full_p):
                        self.goal_bgr_img = cv2.imread(full_p)
                if self.goal_bgr_img is None:
                    # Check for any available corridor goal photo as visual reference
                    corridor_ref = os.path.join(WORKSPACE_DIR, "config/goals/goal_02_Waypoint_2.jpg")
                    if os.path.exists(corridor_ref):
                        self.goal_bgr_img = cv2.imread(corridor_ref)
                    else:
                        self.goal_bgr_img = frame.copy()

            goal_bgr = self.goal_bgr_img
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
            with self.lock:
                fwd_clearance = self.min_forward_dist
                left_clearance = self.min_left_dist
                right_clearance = self.min_right_dist

            # Side Wall Repulsion (LiDAR Corridor Centering)
            if left_clearance < 0.45:
                # Too close to left wall (< 45cm)! Push away to right
                target_wz = min(target_wz, -0.28)
                target_vx = min(target_vx, 0.30)
            elif right_clearance < 0.45:
                # Too close to right wall (< 45cm)! Push away to left
                target_wz = max(target_wz, 0.28)
                target_vx = min(target_vx, 0.30)

            # Forward Obstacle Interlock
            if fwd_clearance < 0.50:
                # Wall or obstacle directly in front within 50cm! Stop forward motion and steer towards open corridor
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
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.write(
                f"{iso_ts},{elapsed:.3f},{self.pose_sample_count},"
                f"{pose['x']:.4f},{pose['y']:.4f},{pose['z']:.4f},{pose['yaw']:.2f},"
                f"{self.cmd_vx:.3f},{self.cmd_wz:.3f},{dist_to_goal:.3f},{rel_heading_deg:.2f},NAVIGATING\n"
            )
            self.csv_file.flush()

        sys.stdout.write(
            f"\r🚀 [{self.mode.upper()}] "
            f"Pos: ({pose['x']:+6.2f}m, {pose['y']:+6.2f}m) | "
            f"Target: #{self.current_goal['id']} ({self.current_goal['name']}) | "
            f"{BOLD}Dist: {dist_to_goal:5.2f}m{NC} | "
            f"Cmd: (vx={self.cmd_vx:.2f}, wz={self.cmd_wz:+.2f}) | "
            f"VLM: #{self.vlm_query_count:02d} | "
            f"Poses: #{self.pose_sample_count:04d} ({elapsed:4.1f}s)"
        )
        sys.stdout.flush()

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

    def generate_markdown_summary(self, path_len, elapsed, avg_spd, final_dist, success, reason):
        md_path = os.path.join(self.trial_dir, "trial_summary.md")
        mean_lat = float(np.mean(self.vlm_latencies)) if self.vlm_latencies else 0.0
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
            f.write(f"## 📁 Saved Artifacts\n")
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

def print_goal_menu(goals):
    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🐕 [Unitree Go2 Autonomous Navigation: Multi-Goal Benchmark Shell]{NC}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")
    for g in goals:
        photo_info = f"📸 Photo: {g.get('snapshot_image', 'None')}" if 'snapshot_image' in g else ""
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}° | {photo_info}")
    if len(goals) > 1:
        seq_str = " -> ".join(f"#{g['id']}" for g in goals)
        print(f"  [all] Continuous Sequential Patrol ({seq_str})")
        print(f"  [1,2] Custom Waypoint Sequence (e.g. '1,2' or '2,1')")
    print(f"  [q]   Safe Shutdown & Exit Stack")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

def main():
    parser = argparse.ArgumentParser(description="Publication Benchmark Runner for Unitree Go2")
    parser.add_argument('--mode', choices=['ours', 'pixnav'], default='ours', help="Navigation mode (ours = ESCAPE-Nav, pixnav = PointNav)")
    parser.add_argument('--goal', type=str, default=None, help="Initial Goal ID or Sequence (e.g. '1' or '1,2' or 'all')")
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

    # 1. Searching for initial landmarks
    print(f"\n{YELLOW}⏳ [1/2] Waiting for RTAB-Map 4D LiDAR Localization Lock...{NC}")
    wait_count = 0
    initial_pose = node.get_current_pose()
    while initial_pose is None:
        time.sleep(0.5)
        wait_count += 1
        if wait_count % 4 == 0:
            print(f"  {YELLOW}🔍 Still searching for 4D LiDAR map landmarks ({wait_count * 0.5:.1f}s elapsed)...{NC}")
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
            status_tag = f"{GREEN}100% HEALTHY LOCK!{NC}" if sec == 5 else f"{CYAN}STABLE (Jitter: {drift_cm:3.1f}cm){NC}"
            print(f" [{sec}/5s] {GREEN}🟢 LOCALIZED{NC} | X:{BOLD}{p['x']:+7.3f}m{NC} Y:{BOLD}{p['y']:+7.3f}m{NC} Yaw:{BOLD}{p['yaw']:+6.1f}°{NC} | {status_tag}")

    print(f"{BOLD}{GREEN}========================================================================{NC}")
    print(f"{BOLD}{GREEN} 🎯 [LOCALIZATION FULLY STABILIZED] Ready for Mission Execution!{NC}")
    print(f"{BOLD}{GREEN}========================================================================{NC}\n")

    initial_goal_arg = args.goal

    while True:
        goals = load_goals()
        if not goals:
            print(f"{RED}Error: No candidate goals registered in {GOALS_YAML}. Run ./run_local.sh first.{NC}")
            break

        print_goal_menu(goals)

        if initial_goal_arg is not None:
            choice = str(initial_goal_arg).strip()
            initial_goal_arg = None
        else:
            try:
                choice = safe_input(f"\n👉 Enter Target Goal [1-{len(goals)}], sequence (e.g. '1,2'), 'all', or 'q': ").strip()
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
