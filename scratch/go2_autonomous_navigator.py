#!/usr/bin/env python3
"""
Persistent Interactive Mission Control & Autonomous Navigator for Unitree Go2.
Features:
  1. Persistent ROS 2 Localization & Control Stack (Zero restart lag)
  2. Interactive Goal Selection Loop (1 -> Goal 1 -> Arrive & Hold -> 2 -> Goal 2)
  3. Dual-Layer Actuation: Direct CycloneDDS Sport API (1008) + ROS 2 /cmd_vel
  4. Automatic Per-Trial Hierarchical Artifacts Logging:
     - camera_snapshots/ (Communicated raw frames & subgoals)
     - trial_trajectory_on_2d_map.png (Clean 1-자 corridor overlay)
     - trajectory_plot_bev.png (300 DPI BEV plot)
     - trajectory_raw.csv & vlm_decisions.jsonl
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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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

class PersistentAutonomousNavigator(Node):
    def __init__(self, mode="ours", max_vx=0.30, max_wz=0.50, tolerance_m=0.50, timeout_s=120):
        super().__init__('go2_persistent_navigator')
        self.mode = mode
        self.max_vx = max_vx
        self.max_wz = max_wz
        self.tolerance_m = tolerance_m
        self.timeout_s = timeout_s

        self.current_pose = None
        self.latest_frame = None
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Active Mission State
        self.is_mission_active = False
        self.current_goal = None
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

        # ROS 2 Subscriptions
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', self.pose_callback, qos)
        self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Dual-Layer Velocity Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        if HAS_UNITREE_API:
            self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        else:
            self.sport_pub = None

        # 10Hz Control Loop & 1.5Hz VLM Loop
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.7, self.vlm_decision_loop)

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
            if self.is_mission_active:
                if self.start_pose is None:
                    self.start_pose = self.current_pose
                self.trajectory_history.append((self.current_pose['x'], self.current_pose['y'], self.current_pose['yaw'], time.time()))

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
            pose_dict = {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "yaw": float(math.degrees(yaw_rad)),
                "yaw_rad": float(yaw_rad),
                "time": time.time()
            }
            if self.is_mission_active and self.start_pose is None:
                self.start_pose = pose_dict
            return pose_dict
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

    def start_mission(self, goal_entry):
        self.current_goal = goal_entry
        self.start_pose = None
        self.trajectory_history = []
        self.vlm_decision_history = []
        self.vlm_latencies = []
        self.pose_sample_count = 0
        self.vlm_query_count = 0
        self.mission_start_time = time.time()
        self.subgoal_u = 640
        self.subgoal_v = 500

        # Setup Trial Directory
        goal_folder_name = f"goal_{goal_entry['id']}_{goal_entry['name']}"
        mode_goal_dir = os.path.join(EXP_ROOT, self.mode, goal_folder_name)
        os.makedirs(mode_goal_dir, exist_ok=True)

        existing_trials = glob.glob(os.path.join(mode_goal_dir, "trial_*"))
        trial_num = len(existing_trials) + 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.trial_dir = os.path.join(mode_goal_dir, f"trial_{trial_num:02d}_{timestamp}")
        self.snapshots_dir = os.path.join(self.trial_dir, "camera_snapshots")
        os.makedirs(self.snapshots_dir, exist_ok=True)

        # Update symlink
        latest_link = os.path.join(EXP_ROOT, "latest")
        if os.path.lexists(latest_link):
            os.remove(latest_link)
        try:
            os.symlink(self.trial_dir, latest_link)
        except Exception:
            pass

        # Open Log Files
        self.csv_path = os.path.join(self.trial_dir, "trajectory_raw.csv")
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write("iso_timestamp,elapsed_s,pose_index,x_m,y_m,z_m,yaw_deg,cmd_vx,cmd_wz,dist_to_goal_m,status\n")
        self.csv_file.flush()

        self.vlm_log_path = os.path.join(self.trial_dir, "vlm_decisions.jsonl")
        self.vlm_log_file = open(self.vlm_log_path, "w")

        self.is_mission_active = True
        print(f"\n{BOLD}{GREEN}========================================================================{NC}")
        print(f"{BOLD}{GREEN} 🚀 [MISSION STARTED] Navigating to Goal #{goal_entry['id']}: {goal_entry['name']}{NC}")
        print(f" • Mode        : {BOLD}{self.mode.upper()}{NC} ({'Qwen VLM + PixNav' if self.mode == 'ours' else 'Pure PixNav'})")
        print(f" • Destination : X={goal_entry['x_m']:+.2f}m, Y={goal_entry['y_m']:+.2f}m")
        print(f" • Trial Folder: {self.trial_dir}")
        print(f"{BOLD}{GREEN}========================================================================{NC}\n")

    def finish_mission(self, success: bool, reason: str, final_dist: float = 0.0):
        self.is_mission_active = False
        self.stop_robot()
        elapsed = time.time() - self.mission_start_time

        # Save Final Snapshot
        self.save_final_snapshot("arrival" if success else "abort")

        # Render Trajectory Map & Plots
        self.render_trajectory_on_2d_map()

        # Metadata
        metadata = {
            "trial_dir": self.trial_dir,
            "created_at": datetime.now().isoformat(),
            "mode": self.mode,
            "goal": self.current_goal,
            "initial_pose": self.start_pose,
            "final_status": {
                "success": success,
                "reason": reason,
                "duration_seconds": round(elapsed, 2),
                "final_distance_to_goal_m": round(final_dist, 3),
                "total_pose_samples": self.pose_sample_count,
                "total_vlm_queries": self.vlm_query_count
            },
            "saved_artifacts": {
                "trajectory_map_overlay": "trial_trajectory_on_2d_map.png",
                "trajectory_plot_bev": "trajectory_plot_bev.png",
                "trajectory_raw_csv": "trajectory_raw.csv",
                "vlm_decisions_jsonl": "vlm_decisions.jsonl",
                "camera_snapshots_dir": "camera_snapshots/"
            }
        }

        meta_path = os.path.join(self.trial_dir, "trial_metadata.json")
        with open(meta_path, "w") as f:
            json.dump(metadata, f, indent=2)

        if self.csv_file: self.csv_file.close()
        if self.vlm_log_file: self.vlm_log_file.close()

        print(f"\n\n========================================================================")
        print(f"{GREEN}{BOLD} 💾 [TRIAL #{self.current_goal['id']} DATA & VISUAL ARTIFACTS SAVED]{NC}")
        print(f"========================================================================")
        print(f" • Outcome             : {'✅ ARRIVED & HOLDING POSITION' if success else '⚠️ HALTED: ' + reason}")
        print(f" • Exact Trial Folder  : {self.trial_dir}")
        print(f" • 🗺️ 2D Map Trajectory : trial_trajectory_on_2d_map.png ⭐")
        print(f" • 📸 VLM Raw & Decision: camera_snapshots/ (query_XXX_raw.jpg & decision.jpg) ⭐")
        print(f" • 📄 Raw Trajectory CSV: trajectory_raw.csv ({self.pose_sample_count} poses)")
        print(f" • 📋 Trial Metadata   : trial_metadata.json")
        print(f"========================================================================\n")

    def vlm_decision_loop(self):
        if not self.is_mission_active or self.mode != "ours":
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

        threading.Thread(target=self._query_vlm_async, args=(frame, dist, math.degrees(rel_heading), pose), daemon=True).start()

    def _query_vlm_async(self, frame, dist_to_goal, rel_heading_deg, pose):
        if self.vlm_active or not self.is_mission_active:
            return
        self.vlm_active = True
        self.vlm_query_count += 1
        q_idx = self.vlm_query_count
        t0 = time.time()

        # 1. Save Exact Communicated Raw JPEG Frame
        raw_frame_name = f"query_{q_idx:03d}_raw.jpg"
        raw_frame_path = os.path.join(self.snapshots_dir, raw_frame_name)
        cv2.imwrite(raw_frame_path, frame)

        try:
            ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            b64_img = base64.b64encode(buf).decode('utf-8')

            prompt = f"""You are the visual navigation brain for Unitree Go2 navigating to Goal '{self.current_goal['name']}'.
Distance to Goal: {dist_to_goal:.1f}m, Relative Angle: {rel_heading_deg:+.1f} deg.
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

            # 2. Save Decision Overlay Image with Guidance Path Arrow
            overlay = frame.copy()
            u, v = self.subgoal_u, self.subgoal_v
            
            # Draw Guidance Arrow from camera base [640, 700] to Subgoal [u, v]
            cv2.arrowedLine(overlay, (640, 700), (u, v), (0, 255, 255), 4, tipLength=0.15)
            cv2.circle(overlay, (u, v), 14, (0, 255, 0), 3, cv2.LINE_AA)
            cv2.circle(overlay, (u, v), 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(overlay, f"VLM Subgoal #{q_idx}: [{u},{v}] ({action})", (30, 50),
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
                "action": action
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
                "raw_image_file": raw_frame_name,
                "decision_overlay_image": decision_frame_name
            }
            if self.vlm_log_file and not self.vlm_log_file.closed:
                self.vlm_log_file.write(json.dumps(vlm_record) + "\n")
                self.vlm_log_file.flush()

        except Exception:
            pass
        finally:
            self.vlm_active = False

    def control_loop(self):
        if not self.is_mission_active or not self.current_goal:
            return

        now = time.time()
        elapsed = now - self.mission_start_time
        pose = self.get_current_pose()

        if not pose:
            self.stop_robot()
            return

        dx = self.current_goal['x_m'] - pose['x']
        dy = self.current_goal['y_m'] - pose['y']
        dist_to_goal = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        rel_heading_deg = math.degrees(rel_heading)

        if dist_to_goal <= self.tolerance_m:
            print(f"\n{GREEN}{BOLD}🎉 [GOAL REACHED] Arrived within {dist_to_goal:.2f}m of Goal #{self.current_goal['id']} ({self.current_goal['name']})!{NC}")
            self.finish_mission(success=True, reason="ARRIVED", final_dist=dist_to_goal)
            return

        if elapsed > self.timeout_s:
            print(f"\n{RED}⏱️ [TIMEOUT] Exceeded duration of {self.timeout_s}s. Halting robot.{NC}")
            self.finish_mission(success=False, reason="TIMEOUT", final_dist=dist_to_goal)
            return

        if self.mode == "ours":
            norm_u = (self.subgoal_u - 640) / 640.0
            target_wz = -norm_u * 0.45 + (rel_heading * 0.20)
            target_vx = self.max_vx * max(0.2, (1.0 - abs(norm_u) * 0.8))
        else:
            if abs(rel_heading_deg) > 40.0:
                target_vx = 0.05
                target_wz = math.copysign(0.40, rel_heading)
            else:
                target_vx = self.max_vx * (1.0 - abs(rel_heading) / math.pi)
                target_wz = rel_heading * 0.50

        self.publish_cmd(target_vx, target_wz)

        self.pose_sample_count += 1
        iso_ts = datetime.now().isoformat()
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.write(
                f"{iso_ts},{elapsed:.3f},{self.pose_sample_count},"
                f"{pose['x']:.4f},{pose['y']:.4f},{pose['z']:.4f},{pose['yaw']:.2f},"
                f"{self.cmd_vx:.3f},{self.cmd_wz:.3f},{dist_to_goal:.3f},NAVIGATING\n"
            )
            self.csv_file.flush()

        sys.stdout.write(
            f"\r🚀 [{self.mode.upper()}] "
            f"Pos: ({pose['x']:+6.2f}m, {pose['y']:+6.2f}m) | "
            f"Target: #{self.current_goal['id']} ({self.current_goal['name']}) | "
            f"{BOLD}Dist: {dist_to_goal:5.2f}m{NC} | "
            f"Cmd: (vx={self.cmd_vx:.2f}, wz={self.cmd_wz:+.2f}) | "
            f"Frames: #{self.vlm_query_count:02d} | "
            f"Poses: #{self.pose_sample_count:04d} ({elapsed:4.1f}s)"
        )
        sys.stdout.flush()

    def save_final_snapshot(self, label):
        with self.lock:
            if self.latest_frame is None or self.snapshots_dir is None:
                return
            frame = self.latest_frame.copy()
        path = os.path.join(self.snapshots_dir, f"{label}_frame.jpg")
        cv2.imwrite(path, frame)

    def render_trajectory_on_2d_map(self):
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

        pts = []
        for p in self.trajectory_history:
            px = int((p[0] - min_x) / res)
            py = int(h - 1 - (p[1] - min_y) / res)
            pts.append((px, py))

        for i in range(1, len(pts)):
            cv2.line(overlay, pts[i-1], pts[i], (255, 120, 0), 4, cv2.LINE_AA)

        for dec in self.vlm_decision_history:
            dpx = int((dec['pose'][0] - min_x) / res)
            dpy = int(h - 1 - (dec['pose'][1] - min_y) / res)
            cv2.circle(overlay, (dpx, dpy), 5, (0, 215, 255), -1, cv2.LINE_AA)

        start_px = pts[0]
        goal_px = (int((self.current_goal['x_m'] - min_x) / res), int(h - 1 - (self.current_goal['y_m'] - min_y) / res))

        cv2.circle(overlay, start_px, 8, (0, 200, 0), -1, cv2.LINE_AA)
        cv2.circle(overlay, start_px, 10, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, f"START ({self.start_pose['x']:.1f}m)", (start_px[0] - 60, start_px[1] + 25), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 160, 0), 1, cv2.LINE_AA)

        cv2.circle(overlay, goal_px, 9, (0, 0, 240), -1, cv2.LINE_AA)
        cv2.circle(overlay, goal_px, 11, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, f"GOAL #{self.current_goal['id']}: {self.current_goal['name']}", (goal_px[0] - 80, goal_px[1] - 15), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 220), 1, cv2.LINE_AA)

        out_map_path = os.path.join(self.trial_dir, "trial_trajectory_on_2d_map.png")
        cv2.imwrite(out_map_path, overlay)

        try:
            fig, ax = plt.subplots(figsize=(10, 4.5), dpi=200)
            xs = [p[0] for p in self.trajectory_history]
            ys = [p[1] for p in self.trajectory_history]
            ax.plot(xs, ys, color='#0066cc', linewidth=2.5, label='Robot Trajectory (10Hz)')
            ax.scatter(xs[0], ys[0], color='#00aa00', s=100, zorder=6, label=f"Start ({xs[0]:.1f}m)")
            ax.scatter(self.current_goal['x_m'], self.current_goal['y_m'], color='#dd0000', s=130, marker='*', zorder=6, label=f"Goal #{self.current_goal['id']}: {self.current_goal['name']}")
            ax.set_title(f"Trial #{self.current_goal['id']} [{self.mode.upper()}]: Trajectory to {self.current_goal['name']}", fontsize=13, fontweight='bold', pad=10)
            ax.set_xlabel("Map X (meters)", fontsize=11)
            ax.set_ylabel("Map Y (meters)", fontsize=11)
            ax.grid(True, linestyle='--', alpha=0.5)
            ax.legend(loc='upper right', framealpha=0.95)
            plt.tight_layout()
            plot_path = os.path.join(self.trial_dir, "trajectory_plot_bev.png")
            plt.savefig(plot_path)
            plt.close()
        except Exception:
            pass

        return out_map_path

def load_goals():
    if not os.path.exists(GOALS_YAML):
        return []
    with open(GOALS_YAML, 'r') as f:
        data = yaml.safe_load(f)
        return data.get('goals', [])

def print_goal_menu(goals):
    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🐕 [Unitree Go2 Autonomous Navigation: Persistent Mission Control]{NC}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")
    for g in goals:
        photo_info = f"📸 Photo: {g.get('snapshot_image', 'None')}" if 'snapshot_image' in g else ""
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}° | {photo_info}")
    print(f"  [q] Safe Shutdown & Exit Stack")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

def main():
    parser = argparse.ArgumentParser(description="Persistent Goal-Directed Experiment Runner for Unitree Go2")
    parser.add_argument('--mode', choices=['ours', 'pixnav'], default='ours', help="Navigation mode (ours = ESCAPE-Nav, pixnav = PointNav)")
    parser.add_argument('--goal', type=str, default=None, help="Initial Goal ID (Optional)")
    parser.add_argument('--tolerance', type=float, default=0.50, help="Goal arrival tolerance in meters")
    parser.add_argument('--timeout', type=int, default=120, help="Max run duration in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = PersistentAutonomousNavigator(
        mode=args.mode,
        tolerance_m=args.tolerance,
        timeout_s=args.timeout
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for initial localization lock
    print(f"\n{YELLOW}⏳ Waiting for RTAB-Map 4D LiDAR Localization Lock...{NC}")
    while node.get_current_pose() is None:
        time.sleep(0.3)

    p0 = node.get_current_pose()
    print(f"{GREEN}{BOLD}🟢 [LOCALIZATION LOCKED] Robot ready at X={p0['x']:+.2f}m, Y={p0['y']:+.2f}m, Yaw={p0['yaw']:+.1f}°!{NC}")

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
                choice = input(f"\n👉 Enter Target Goal [1-{len(goals)}] to navigate (or 'q' to quit): ").strip()
            except (KeyboardInterrupt, EOFError):
                break

        if choice.lower() in ('q', 'quit', 'exit'):
            break

        selected_goal = None
        for g in goals:
            if str(g['id']) == choice or str(g['name']).lower() == choice.lower():
                selected_goal = g
                break

        if not selected_goal:
            print(f"{YELLOW}⚠️ Invalid choice. Please enter a valid Goal ID (1-{len(goals)}).{NC}")
            continue

        # Start Autonomous Navigation
        node.start_mission(selected_goal)

        # Wait until mission completes (goal reached or timeout or interrupted)
        try:
            while node.is_mission_active:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print(f"\n{YELLOW}⚠️ Manual interrupt received! Halting robot.{NC}")
            node.finish_mission(success=False, reason="USER_INTERRUPT", final_dist=0.0)

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{GREEN}✅ Navigation Stack Safely Closed. 🐕{NC}")

if __name__ == '__main__':
    main()
