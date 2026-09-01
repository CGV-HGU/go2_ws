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
    def __init__(self, mode="ours", max_vx=0.30, max_wz=0.45, tolerance_m=0.35, timeout_s=120):
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
            return {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "yaw": float(math.degrees(yaw_rad)),
                "yaw_rad": float(yaw_rad),
                "time": time.time()
            }
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

        self.is_mission_active = True
        print(f"\n{BOLD}{GREEN}========================================================================{NC}")
        print(f"{BOLD}{GREEN} 🚀 [MISSION STARTED] {today_str}_{seq_num:02d} -> Goal #{goal_entry['id']}: {goal_entry['name']}{NC}")
        print(f" • Mode            : {BOLD}{self.mode.upper()}{NC}")
        print(f" • Target Pose     : X={goal_entry['x_m']:+.2f}m, Y={goal_entry['y_m']:+.2f}m, Tolerance={self.tolerance_m:.2f}m")
        print(f" • Output Folder   : {self.trial_dir}")
        print(f"{BOLD}{GREEN}========================================================================{NC}\n")

    def finish_mission(self, success: bool, reason: str, final_dist: float = 0.0):
        self.is_mission_active = False
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
        # In-Place Rotation Safety Interlock:
        # If heading error > 35°, rotate in-place with ZERO forward creep!
        # Prevents robot from drifting into side walls while turning.
        # ---------------------------------------------------------
        if abs(rel_heading_deg) > 35.0:
            target_vx = 0.0
            target_wz = math.copysign(0.38, rel_heading)
        else:
            if self.mode == "ours":
                norm_u = (self.subgoal_u - 640) / 640.0
                target_wz = -norm_u * 0.40 + (rel_heading * 0.20)
                target_vx = self.max_vx * (1.0 - abs(norm_u) * 0.6)
            else:
                # PixNav Baseline
                target_wz = rel_heading * 0.50
                target_vx = self.max_vx * (1.0 - abs(rel_heading) / (math.pi / 2.0))

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
            f.write(f"- **VLM Queries Count**: `{self.vlm_query_count}` (Mean Latency: `{mean_lat:.1f} ms`)\n\n")
            f.write(f"## 📁 Saved Artifacts\n")
            f.write(f"- `trial_trajectory_on_2d_map.png` : 2D floor plan overlay with all candidate goals.\n")
            f.write(f"- `trial_benchmark_dashboard.png` : 4-panel publication-grade research dashboard.\n")
            f.write(f"- `trajectory_raw.csv` : 10Hz raw pose & velocity timeseries data.\n")
            f.write(f"- `vlm_decisions.jsonl` : Log of every VLM query, prompt, and sub-goal.\n")
            f.write(f"- `camera_snapshots/` : Annotated decision frames.\n")

def load_goals():
    if not os.path.exists(GOALS_YAML):
        return []
    with open(GOALS_YAML, 'r') as f:
        data = yaml.safe_load(f)
        return data.get('goals', [])

def print_goal_menu(goals):
    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🐕 [Unitree Go2 Autonomous Navigation: 5-Episode Benchmark Shell]{NC}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")
    for g in goals:
        photo_info = f"📸 Photo: {g.get('snapshot_image', 'None')}" if 'snapshot_image' in g else ""
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}° | {photo_info}")
    print(f"  [q] Safe Shutdown & Exit Stack")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

def main():
    parser = argparse.ArgumentParser(description="Publication Benchmark Runner for Unitree Go2")
    parser.add_argument('--mode', choices=['ours', 'pixnav'], default='ours', help="Navigation mode (ours = ESCAPE-Nav, pixnav = PointNav)")
    parser.add_argument('--goal', type=str, default=None, help="Initial Goal ID (Optional)")
    parser.add_argument('--tolerance', type=float, default=0.35, help="Goal arrival tolerance in meters")
    parser.add_argument('--timeout', type=int, default=120, help="Max run duration in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = AutonomousNavigator(
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

        # Start Autonomous Navigation Mission
        node.start_mission(selected_goal, goals)

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
    print(f"\n{GREEN}✅ Benchmark Runner Safely Closed. 🐕{NC}")

if __name__ == '__main__':
    main()
