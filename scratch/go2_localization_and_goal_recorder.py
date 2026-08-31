#!/usr/bin/env python3
"""
Unified Real-Time Localization HUD, Auto-CSV Logger & Interactive Goal Recorder for Unitree Go2.
Provides:
  1. Live (X, Y, Z, Yaw) HUD stream
  2. Automatic CSV and TXT logging to ~/.ros/localization_runs/latest/
  3. Interactive Goal Recording:
     - Press [ENTER]: Automatically saves current pose as Goal #1, Goal #2, etc.
     - Type 'd' / 'del' / Backspace: Asks confirmation to delete the last goal.
     - Type 'list': Displays all registered goals.
     - Type 'clear': Asks confirmation to clear all goals.
  4. Auto-renders 2D map goal pins to 2dmap/2d_goals_map.png
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
    res = 0.05
    overlay = img.copy()

    for g in goals:
        gid = g.get('id', 1)
        name = g.get('name', f'Goal_{gid}')
        gx = g['x_m']
        gy = g['y_m']
        px = int(w / 2 + (gx + 14.0) / res)
        py = int(h / 2 - (gy - 27.5) / res)
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

    print(f"\n{BOLD}🎯 Key Controls:{NC}")
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
                    deleted = goals.pop()
                    save_goals(goals)
                    print(f"{RED}🗑️ Deleted Goal #{deleted['id']} ({deleted['name']}). Remaining: {len(goals)}{NC}\n")
                else:
                    print("Deletion cancelled.\n")
                continue

            elif user_input.lower() in ('clear', 'c'):
                if not goals:
                    print(f"\n{YELLOW}⚠️ Goal list is already empty.{NC}\n")
                    continue
                confirm = input(f"\n{RED}⚠️ Clear ALL {len(goals)} registered goals? [y/N]: {NC}").strip().lower()
                if confirm in ('y', 'yes'):
                    goals = []
                    save_goals(goals)
                    print(f"{RED}🗑️ Cleared all goals.{NC}\n")
                else:
                    print("Clear cancelled.\n")
                continue

            # Default: Save current pose as new Goal
            curr = node.get_pose()
            if not curr:
                print(f"\n{YELLOW}⚠️ Cannot record goal: No active localization pose received yet.{NC}\n")
                continue

            new_id = len(goals) + 1
            goal_name = user_input if user_input else f"Waypoint_{new_id}"

            new_entry = {
                "id": new_id,
                "name": goal_name,
                "description": f"Candidate goal #{new_id}",
                "x_m": round(curr['x'], 3),
                "y_m": round(curr['y'], 3),
                "z_m": round(curr['z'], 3),
                "yaw_deg": round(curr['yaw'], 1),
                "tolerance_m": 0.50
            }

            goals.append(new_entry)
            save_goals(goals)

            print(f"\n{GREEN}✅ Saved Goal #{new_id}: '{goal_name}' at X={new_entry['x_m']:+.3f}m, Y={new_entry['y_m']:+.3f}m, Yaw={new_entry['yaw_deg']:+.1f}°{NC}")
            print(f"   (Saved to {GOALS_YAML} and rendered to {GOALS_MAP_PNG})\n")

        except (KeyboardInterrupt, EOFError):
            break

    print(f"\n{GREEN}🏁 Localization & Goal Session Ended. All logs saved safely.{NC}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
