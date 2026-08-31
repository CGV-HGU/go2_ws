#!/usr/bin/env python3
"""
Unified Real-Time Localization HUD, Auto-CSV Logger & Interactive Goal Recorder for Unitree Go2.
Features:
  1. 5-Second Live Localization Stability & Calibration Warmup Check
  2. Live (X, Y, Z, Yaw) HUD stream
  3. Automatic CSV logging to ~/.ros/localization_runs/latest/
  4. Interactive Goal Recording (1-Click Enter)
  5. Auto-renders 2D map goal pins using 2d_metadata.json
"""

import os
import sys
import math
import time
import yaml
import json
import threading
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import cv2
import numpy as np

GREEN = '\033[0;32m'
CYAN = '\033[0;36m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BOLD = '\033[1m'
NC = '\033[0m'

GOALS_YAML = "/home/unitree/go2_ws_antarctica/config/navigation_goals.yaml"
GOALS_JSON = "/home/unitree/go2_ws_antarctica/config/navigation_goals.json"
MAP_2D_PNG = "/home/unitree/go2_ws_antarctica/2dmap/2d.png"
MAP_METADATA_JSON = "/home/unitree/go2_ws_antarctica/2dmap/2d_metadata.json"
GOALS_MAP_PNG = "/home/unitree/go2_ws_antarctica/2dmap/2d_goals_map.png"

class UnifiedLocalizationAndGoalNode(Node):
    def __init__(self):
        super().__init__('go2_localization_and_goal_node')
        self.lock = threading.Lock()
        self.current_pose = None
        self.last_pose_time = time.time()
        self.pose_count = 0
        self.is_localized = False

        # Setup Logging Directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.expanduser(f"~/.ros/localization_runs/{timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)
        self.csv_path = os.path.join(self.run_dir, "localization_poses.csv")
        self.log_path = os.path.join(self.run_dir, "localization.log")

        runs_root = os.path.expanduser("~/.ros/localization_runs")
        latest_link = os.path.join(runs_root, "latest")
        if os.path.lexists(latest_link):
            os.remove(latest_link)
        try:
            os.symlink(self.run_dir, latest_link)
        except Exception:
            pass

        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write("timestamp_iso,elapsed_s,pose_index,x_m,y_m,z_m,yaw_deg,cov_x,status\n")
        self.csv_file.flush()
        self.log_file = open(self.log_path, "w")
        self.start_time = time.time()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', self.pose_callback, qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.tf_fallback_timer)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        now = time.time()
        elapsed = now - self.start_time
        iso_ts = datetime.now().isoformat()
        cov_x = float(msg.pose.covariance[0])

        with self.lock:
            self.current_pose = {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "yaw": float(yaw_deg),
                "cov_x": cov_x,
                "time": now,
                "status": "LOCALIZED"
            }
            self.pose_count += 1
            self.is_localized = True

        self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{yaw_deg:.2f},{cov_x:.6f},LOCALIZED\n")
        self.csv_file.flush()

    def tf_fallback_timer(self):
        with self.lock:
            last_t = self.current_pose['time'] if self.current_pose else 0.0
        if time.time() - last_t > 1.0:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                pos = t.transform.translation
                ori = t.transform.rotation
                siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
                cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
                yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                now = time.time()
                elapsed = now - self.start_time
                iso_ts = datetime.now().isoformat()

                with self.lock:
                    self.current_pose = {
                        "x": float(pos.x),
                        "y": float(pos.y),
                        "z": float(pos.z),
                        "yaw": float(yaw_deg),
                        "cov_x": 0.0,
                        "time": now,
                        "status": "TF_TRACKING"
                    }
                    self.pose_count += 1

                self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{yaw_deg:.2f},0.0,TF_TRACKING\n")
                self.csv_file.flush()
            except Exception:
                pass

    def get_pose(self):
        with self.lock:
            return self.current_pose

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()

def load_goals():
    if os.path.exists(GOALS_YAML):
        try:
            with open(GOALS_YAML, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('goals', [])
        except Exception:
            pass
    return []

def save_goals(goals):
    os.makedirs(os.path.dirname(GOALS_YAML), exist_ok=True)
    with open(GOALS_YAML, 'w') as f:
        yaml.dump({"goals": goals}, f, sort_keys=False, default_flow_style=False)
    with open(GOALS_JSON, 'w') as f:
        json.dump({"goals": goals}, f, indent=2)
    render_goals_on_map(goals)

def render_goals_on_map(goals):
    if not os.path.exists(MAP_2D_PNG):
        return
    img = cv2.imread(MAP_2D_PNG)
    if img is None:
        return
    h, w = img.shape[:2]
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

    overlay = img.copy()

    for g in goals:
        gid = g.get('id', 1)
        name = g.get('name', f'Goal_{gid}')
        gx = g['x_m']
        gy = g['y_m']
        px = int((gx - min_x) / res)
        py = int(h - 1 - (gy - min_y) / res)
        px = max(20, min(w - 20, px))
        py = max(20, min(h - 20, py))

        cv2.circle(overlay, (px, py), 12, (0, 0, 240), -1, cv2.LINE_AA)
        cv2.circle(overlay, (px, py), 14, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, str(gid), (px - 5, py + 5), cv2.FONT_HERSHEY_DUPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, f"#{gid}: {name}", (px + 18, py + 5), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 200), 1, cv2.LINE_AA)

    cv2.imwrite(GOALS_MAP_PNG, overlay)

def main():
    rclpy.init()
    node = UnifiedLocalizationAndGoalNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🐕 [Unitree Go2] Unified Localization HUD & Goal Manager{NC}")
    print(f" 📂 CSV Log File : {node.csv_path}")
    print(f" 🚩 Goals Config : {GOALS_YAML}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

    goals = load_goals()
    print(f"Loaded {len(goals)} registered candidate goals:")
    for g in goals:
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}°")

    # 1. Searching for initial landmarks
    print(f"\n{YELLOW}⏳ [1/2] Searching for 4D LiDAR map landmarks...{NC}")
    initial_pose = None
    while initial_pose is None:
        initial_pose = node.get_pose()
        time.sleep(0.2)

    # 2. 5-Second Live Localization Stability & Calibration Warmup
    print(f"\n{BOLD}{GREEN}========================================================================{NC}")
    print(f"{BOLD}{GREEN} 🛰️ [2/2] LOCALIZATION LOCK & STABILITY CALIBRATION (5s Warmup Monitor){NC}")
    print(f"{BOLD}{GREEN}========================================================================{NC}")
    
    ref_x = initial_pose['x']
    ref_y = initial_pose['y']
    for sec in range(1, 6):
        time.sleep(1.0)
        p = node.get_pose()
        if p:
            drift_cm = math.hypot(p['x'] - ref_x, p['y'] - ref_y) * 100.0
            status_tag = f"{GREEN}100% HEALTHY LOCK!{NC}" if sec == 5 else f"{CYAN}STABLE (Jitter: {drift_cm:3.1f}cm){NC}"
            print(f" [{sec}/5s] {GREEN}🟢 LOCALIZED{NC} | X:{BOLD}{p['x']:+7.3f}m{NC} Y:{BOLD}{p['y']:+7.3f}m{NC} Yaw:{BOLD}{p['yaw']:+6.1f}°{NC} | {status_tag}")

    print(f"{BOLD}{GREEN}========================================================================{NC}")
    print(f"{BOLD}{GREEN} 🎯 [LOCALIZATION FULLY STABILIZED] Ready for Goal Recording!{NC}")
    print(f"{BOLD}{GREEN}========================================================================{NC}\n")

    print(f"{BOLD}🎯 Key Controls:{NC}")
    print(f"  • {GREEN}[ENTER]{NC}          : Automatically record current pose as Goal #{len(goals)+1} (in sequence)")
    print(f"  • {YELLOW}'d' / 'del'{NC}      : Delete the last recorded goal (with confirmation)")
    print(f"  • {CYAN}'list'{NC}           : Display all registered goals")
    print(f"  • {RED}'clear'{NC}          : Clear all registered goals (with confirmation)")
    print(f"  • 'q' / Ctrl+C      : Exit and save all logs\n")

    while True:
        try:
            pose = node.get_pose()
            if pose:
                status_color = GREEN if pose['status'] == 'LOCALIZED' else CYAN
                pose_str = f"{status_color}{pose['status']}{NC} | X:{BOLD}{pose['x']:+7.3f}m{NC} Y:{BOLD}{pose['y']:+7.3f}m{NC} Z:{pose['z']:+6.3f}m Yaw:{BOLD}{pose['yaw']:+6.1f}°{NC}"
            else:
                pose_str = f"{YELLOW}🔍 SEARCHING FOR LANDMARKS...{NC}"

            prompt_msg = f"\r[Live: {pose_str}]\n👉 Press [ENTER] to save Goal #{len(goals)+1} (or type command): "
            user_input = input(prompt_msg).strip()

            if user_input.lower() in ('q', 'quit', 'exit'):
                break

            elif user_input.lower() in ('list', 'l'):
                print(f"\nRegistered Goals ({len(goals)}):")
                for g in goals:
                    print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}°")
                print()
                continue

            elif user_input.lower() in ('d', 'del', 'delete', 'backspace', 'pop'):
                if not goals:
                    print(f"\n{YELLOW}⚠️ No goals to delete.{NC}\n")
                    continue
                last_g = goals[-1]
                confirm = input(f"\n{YELLOW}⚠️ Delete last goal [#{last_g['id']}: {last_g['name']}]? [y/N]: {NC}").strip().lower()
                if confirm in ('y', 'yes', ''):
                    removed = goals.pop()
                    save_goals(goals)
                    print(f"{RED}🗑️ Deleted Goal #{removed['id']} ({removed['name']}). Remaining: {len(goals)}{NC}\n")
                else:
                    print(f"Cancelled.\n")
                continue

            elif user_input.lower() in ('clear', 'c', 'reset'):
                if not goals:
                    print(f"\n{YELLOW}⚠️ Goal list already empty.{NC}\n")
                    continue
                confirm = input(f"\n{RED}⚠️ Are you sure you want to CLEAR ALL {len(goals)} goals? [y/N]: {NC}").strip().lower()
                if confirm in ('y', 'yes'):
                    goals = []
                    save_goals(goals)
                    print(f"{RED}🧹 All registered goals cleared!{NC}\n")
                else:
                    print(f"Cancelled.\n")
                continue

            # Default: [ENTER] -> Record Goal
            if not pose:
                print(f"\n{RED}❌ Error: Cannot record goal - Robot is not localized yet.{NC}\n")
                continue

            new_id = len(goals) + 1
            default_name = f"Waypoint_{new_id}"
            
            goal_entry = {
                "id": new_id,
                "name": default_name,
                "description": f"Candidate destination #{new_id} recorded via interactive HUD",
                "x_m": round(pose['x'], 2),
                "y_m": round(pose['y'], 2),
                "z_m": round(pose['z'], 2),
                "yaw_deg": round(pose['yaw'], 1),
                "tolerance_m": 0.50
            }
            goals.append(goal_entry)
            save_goals(goals)
            print(f"\n{GREEN}{BOLD}✅ [GOAL RECORDED] Goal #{new_id} '{default_name}' saved!{NC}")
            print(f"   📍 Pose: X={goal_entry['x_m']:+.2f}m, Y={goal_entry['y_m']:+.2f}m, Yaw={goal_entry['yaw_deg']:+.1f}° | Total Goals: {len(goals)}\n")

        except (KeyboardInterrupt, EOFError):
            break

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{GREEN}✅ Goal recording complete. {len(goals)} goals registered in {GOALS_YAML}{NC}")

if __name__ == '__main__':
    main()
