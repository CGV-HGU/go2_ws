#!/usr/bin/env python3
"""
Interactive Goal Pose Recorder for Unitree Go2 Planar 3DoF SLAM.
Captures live (X, Y, Z, Yaw) coordinates from RTAB-Map localization in the map frame,
saves candidate goals to config/navigation_goals.yaml, and plots goal pins on 2D map.
"""

import os
import sys
import math
import time
import yaml
import json
import threading
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
BOLD = '\033[1m'
NC = '\033[0m'

GOALS_YAML = "/home/unitree/go2_ws_antarctica/config/navigation_goals.yaml"
GOALS_JSON = "/home/unitree/go2_ws_antarctica/config/navigation_goals.json"
MAP_2D_PNG = "/home/unitree/go2_ws_antarctica/2dmap/2d.png"
GOALS_MAP_PNG = "/home/unitree/go2_ws_antarctica/2dmap/2d_goals_map.png"

class GoalRecorder(Node):
    def __init__(self):
        super().__init__('go2_goal_recorder')
        self.current_pose = None
        self.last_update = 0.0
        self.lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.pose_callback,
            qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.tf_timer_callback)

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
                "source": "rtabmap_pose"
            }
            self.last_update = time.time()

    def tf_timer_callback(self):
        if time.time() - self.last_update > 1.0:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                pos = t.transform.translation
                ori = t.transform.rotation
                siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
                cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
                yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                with self.lock:
                    self.current_pose = {
                        "x": float(pos.x),
                        "y": float(pos.y),
                        "z": float(pos.z),
                        "yaw": float(yaw_deg),
                        "source": "tf_tracking"
                    }
                    self.last_update = time.time()
            except Exception:
                pass

    def get_pose(self):
        with self.lock:
            return self.current_pose

def load_existing_goals():
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
    # Reference origin estimate from trajectory overlay
    # Compute bounding box
    all_x = [g['x_m'] for g in goals]
    all_y = [g['y_m'] for g in goals]
    if not all_x:
        return

    overlay = img.copy()
    # Draw pins
    for g in goals:
        gid = g.get('id', 1)
        name = g.get('name', f'Goal_{gid}')
        gx = g['x_m']
        gy = g['y_m']
        # Map pixel coordinates relative to center
        px = int(w / 2 + (gx + 14.0) / res)
        py = int(h / 2 - (gy - 27.5) / res)
        px = max(20, min(w - 20, px))
        py = max(20, min(h - 20, py))

        # Goal Pin Circle & Text
        cv2.circle(overlay, (px, py), 12, (0, 0, 240), -1, cv2.LINE_AA)
        cv2.circle(overlay, (px, py), 14, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, str(gid), (px - 5, py + 5), cv2.FONT_HERSHEY_DUPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, f"#{gid}: {name}", (px + 18, py + 5), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 200), 1, cv2.LINE_AA)

    cv2.imwrite(GOALS_MAP_PNG, overlay)

def main():
    rclpy.init()
    node = GoalRecorder()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🚩 [Unitree Go2 Goal Pose Interactive Recorder]{NC}")
    print(f" 📂 Config File : {GOALS_YAML}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

    goals = load_existing_goals()
    print(f"\nCurrently loaded goals ({len(goals)} registered):")
    for g in goals:
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}°")

    print(f"\n{BOLD}Instructions:{NC}")
    print("  1. Drive the robot (via joystick/teleop) to your desired candidate goal pose.")
    print("  2. Type a name/label for the goal and press [ENTER] to save.")
    print("  3. Type 'list' to show goals, 'clear' to reset, or 'exit' (or Ctrl+C) when done.\n")

    while True:
        try:
            pose = node.get_pose()
            if pose:
                status_str = f"{GREEN}TRACKING{NC} (X={pose['x']:+7.3f}m, Y={pose['y']:+7.3f}m, Z={pose['z']:+6.3f}m, Yaw={pose['yaw']:+6.1f}°)"
            else:
                status_str = f"{YELLOW}WAITING FOR LOCALIZATION POSE...{NC}"

            user_input = input(f"\r[Live Pose: {status_str}]\nEnter Goal Name (or press ENTER to record #{len(goals)+1}): ").strip()

            if user_input.lower() in ('exit', 'quit', 'q'):
                break
            elif user_input.lower() == 'list':
                print(f"\nRegistered Goals ({len(goals)}):")
                for g in goals:
                    print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}°")
                continue
            elif user_input.lower() == 'clear':
                goals = []
                save_goals(goals)
                print(f"{YELLOW}Cleared all goals.{NC}")
                continue

            current = node.get_pose()
            if not current:
                print(f"{YELLOW}⚠️ Cannot record goal: No active localization pose received yet. Ensure localization is running.{NC}")
                continue

            goal_id = len(goals) + 1
            goal_name = user_input if user_input else f"Waypoint_{goal_id}"

            new_goal = {
                "id": goal_id,
                "name": goal_name,
                "description": f"Recorded candidate goal #{goal_id}",
                "x_m": round(current['x'], 3),
                "y_m": round(current['y'], 3),
                "z_m": round(current['z'], 3),
                "yaw_deg": round(current['yaw'], 1),
                "tolerance_m": 0.50
            }

            goals.append(new_goal)
            save_goals(goals)

            print(f"{GREEN}✅ Recorded Goal #{goal_id}: '{goal_name}' at X={new_goal['x_m']:+.3f}m, Y={new_goal['y_m']:+.3f}m, Yaw={new_goal['yaw_deg']:+.1f}°{NC}")
            print(f"   Saved to {GOALS_YAML}\n")

        except (KeyboardInterrupt, EOFError):
            break

    print(f"\n{GREEN}🏁 Done recording goals. Total registered: {len(goals)}{NC}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
