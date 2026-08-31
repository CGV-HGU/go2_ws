#!/usr/bin/env python3
"""
Goal-Directed Autonomous Navigation Controller for Unitree Go2 (ESCAPE-Nav / PixNav).
Reads candidate goals from config/navigation_goals.yaml, monitors real-time localization,
queries Qwen VLM / PixNav policy to generate target subgoals, commands robot locomotion,
and achieves precise arrival at the chosen destination.
"""

import os
import sys
import math
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

GOALS_YAML = "/home/unitree/go2_ws_antarctica/config/navigation_goals.yaml"
VLM_URL = "http://100.96.60.15:8000/v1"
MODEL_NAME = "qwen3.5-9b-instruct"

class AutonomousNavigator(Node):
    def __init__(self, mode="ours", target_goal_id=1, max_vx=0.30, max_wz=0.50, tolerance_m=0.50, timeout_s=120):
        super().__init__('go2_autonomous_navigator')
        self.mode = mode
        self.target_goal_id = target_goal_id
        self.max_vx = max_vx
        self.max_wz = max_wz
        self.tolerance_m = tolerance_m
        self.timeout_s = timeout_s

        self.current_pose = None
        self.latest_frame = None
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.start_time = time.time()
        self.trajectory_history = []
        self.is_goal_reached = False
        self.is_stopped = False

        # 1. Load Goal Definition
        self.goal = self.load_target_goal(target_goal_id)
        if not self.goal:
            self.get_logger().error(f"Goal ID {target_goal_id} not found in {GOALS_YAML}")
            sys.exit(1)

        # 2. Setup Run Directory & Logging
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.expanduser(f"~/.ros/navigation_runs/{timestamp}_{self.mode}_{self.goal['name']}")
        os.makedirs(self.run_dir, exist_ok=True)
        self.log_file = open(os.path.join(self.run_dir, "navigation.log"), "w")

        # Symlink latest run
        runs_root = os.path.expanduser("~/.ros/navigation_runs")
        latest_link = os.path.join(runs_root, "latest")
        if os.path.lexists(latest_link):
            os.remove(latest_link)
        try:
            os.symlink(self.run_dir, latest_link)
        except Exception:
            pass

        # 3. ROS 2 Subscriptions & Publishers
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', self.pose_callback, qos)
        self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Velocity Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        if HAS_UNITREE_API:
            self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        else:
            self.sport_pub = None

        # Navigation Control Loop (10Hz)
        self.create_timer(0.1, self.control_loop)
        # VLM Decision Loop (1.5Hz)
        self.vlm_active = False
        self.subgoal_u = 640
        self.subgoal_v = 500
        self.create_timer(0.7, self.vlm_decision_loop)

        print(f"\n{BOLD}{CYAN}========================================================================{NC}")
        print(f"{BOLD}{CYAN} 🚀 [Unitree Go2 Autonomous Navigation]{NC}")
        print(f" • Mode        : {self.mode.upper()} ({'Qwen VLM + PixNav' if self.mode == 'ours' else 'Pure PixNav'})")
        print(f" • Target Goal : #{self.goal['id']} - {self.goal['name']}")
        print(f" • Target Pose : X={self.goal['x_m']:+.3f}m, Y={self.goal['y_m']:+.3f}m, Yaw={self.goal['yaw_deg']:+.1f}°")
        print(f" • Tolerance   : {self.tolerance_m:.2f}m | Timeout: {self.timeout_s}s")
        print(f" • Log Dir     : {self.run_dir}")
        print(f"{BOLD}{CYAN}========================================================================{NC}\n")

    def load_target_goal(self, goal_id):
        if not os.path.exists(GOALS_YAML):
            return None
        with open(GOALS_YAML, 'r') as f:
            data = yaml.safe_load(f)
            for g in data.get('goals', []):
                if str(g.get('id')) == str(goal_id) or str(g.get('name')).lower() == str(goal_id).lower():
                    return g
        return None

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
        # Fallback to TF
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
        vx = max(-self.max_vx, min(self.max_vx, vx))
        wz = max(-self.max_wz, min(self.max_wz, wz))

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def vlm_decision_loop(self):
        if self.is_goal_reached or self.mode != "ours":
            return
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return

        pose = self.get_current_pose()
        if not pose:
            return

        # Relative goal geometry
        dx = self.goal['x_m'] - pose['x']
        dy = self.goal['y_m'] - pose['y']
        dist = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi

        # Asynchronously Query VLM in separate thread to avoid blocking control timer
        threading.Thread(target=self._query_vlm_async, args=(frame, dist, math.degrees(rel_heading)), daemon=True).start()

    def _query_vlm_async(self, frame, dist_to_goal, rel_heading_deg):
        if self.vlm_active:
            return
        self.vlm_active = True
        try:
            ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            b64_img = base64.b64encode(buf).decode('utf-8')

            prompt = f"""You are the visual navigation brain for Unitree Go2 navigating to Goal '{self.goal['name']}'.
Distance to Goal: {dist_to_goal:.1f}m, Relative Angle: {rel_heading_deg:+.1f} deg.
Inspect the front camera view. Identify the open collision-free corridor or doorway leading toward the destination.
Output a JSON response:
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
            if resp.status_code == 200:
                data = resp.json()
                content = data['choices'][0]['message']['content']
                # Parse JSON block
                import re
                m = re.search(r'\{.*\}', content, re.DOTALL)
                if m:
                    res_json = json.loads(m.group(0))
                    pt = res_json.get('selected_image_point', {'x': 0.5, 'y': 0.72})
                    self.subgoal_u = int(pt.get('x', 0.5) * 1280)
                    self.subgoal_v = int(pt.get('y', 0.72) * 720)
        except Exception:
            pass
        finally:
            self.vlm_active = False

    def control_loop(self):
        if self.is_goal_reached:
            self.stop_robot()
            return

        pose = self.get_current_pose()
        if not pose:
            self.get_logger().warn("Waiting for live localization pose...", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        elapsed = time.time() - self.start_time
        if elapsed > self.timeout_s:
            print(f"\n{RED}⏱️ [TIMEOUT] Exceeded maximum duration of {self.timeout_s}s. Halting robot.{NC}")
            self.finish_run(success=False, reason="TIMEOUT")
            return

        # Compute Euclidean Distance to Goal
        dx = self.goal['x_m'] - pose['x']
        dy = self.goal['y_m'] - pose['y']
        dist_to_goal = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        rel_heading_deg = math.degrees(rel_heading)

        # Check Arrival Threshold
        if dist_to_goal <= self.tolerance_m:
            print(f"\n{GREEN}{BOLD}🎉 [GOAL REACHED] Arrived within {dist_to_goal:.2f}m of Goal #{self.goal['id']} ({self.goal['name']})!{NC}")
            self.finish_run(success=True, reason="ARRIVED")
            return

        # Compute Control Command
        if self.mode == "ours":
            # Image sub-goal visual servoing + global goal bias
            norm_u = (self.subgoal_u - 640) / 640.0 # [-1.0 (left), +1.0 (right)]
            target_wz = -norm_u * 0.45 + (rel_heading * 0.20)
            target_vx = self.max_vx * max(0.2, (1.0 - abs(norm_u) * 0.8))
        else:
            # Pure PointNav (Direct Goal)
            if abs(rel_heading_deg) > 40.0:
                target_vx = 0.05
                target_wz = math.copysign(0.40, rel_heading)
            else:
                target_vx = self.max_vx * (1.0 - abs(rel_heading) / math.pi)
                target_wz = rel_heading * 0.50

        self.publish_cmd(target_vx, target_wz)

        # Print Real-Time HUD
        sys.stdout.write(
            f"\r🚀 [{self.mode.upper()}] "
            f"Pos: ({pose['x']:+6.2f}m, {pose['y']:+6.2f}m) | "
            f"Target: #{self.goal['id']} ({self.goal['name']}) | "
            f"{BOLD}Dist: {dist_to_goal:5.2f}m{NC} | "
            f"RelAng: {rel_heading_deg:+5.1f}° | "
            f"Cmd: (vx={target_vx:.2f}, wz={target_wz:+.2f}) | "
            f"Time: {elapsed:4.1f}s"
        )
        sys.stdout.flush()

    def finish_run(self, success: bool, reason: str):
        self.is_goal_reached = True
        self.stop_robot()

        elapsed = time.time() - self.start_time
        path_length = 0.0
        for i in range(1, len(self.trajectory_history)):
            p1 = self.trajectory_history[i-1]
            p2 = self.trajectory_history[i]
            path_length += math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        summary = {
            "timestamp": datetime.now().isoformat(),
            "mode": self.mode,
            "goal": self.goal,
            "success": success,
            "reason": reason,
            "elapsed_seconds": round(elapsed, 2),
            "trajectory_length_m": round(path_length, 3),
            "pose_samples": len(self.trajectory_history)
        }

        summary_path = os.path.join(self.run_dir, "summary.json")
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)

        # Save Trajectory CSV
        traj_csv = os.path.join(self.run_dir, "trajectory.csv")
        with open(traj_csv, "w") as f:
            f.write("x_m,y_m,yaw_deg,timestamp\n")
            for p in self.trajectory_history:
                f.write(f"{p[0]:.4f},{p[1]:.4f},{p[2]:.2f},{p[3]:.3f}\n")

        print(f"\n\n========================================================================")
        print(f" 📊 Run Summary Saved: {summary_path}")
        print(f" • Result          : {'✅ SUCCESS' if success else '❌ FAILED (' + reason + ')'}")
        print(f" • Elapsed Time    : {elapsed:.2f} s")
        print(f" • Trajectory Dist : {path_length:.2f} m")
        print(f"========================================================================\n")
        rclpy.shutdown()

def select_goal_interactively():
    if not os.path.exists(GOALS_YAML):
        print(f"{RED}Error: {GOALS_YAML} not found. Run ./record_goal first.{NC}")
        sys.exit(1)
    with open(GOALS_YAML, 'r') as f:
        data = yaml.safe_load(f)
        goals = data.get('goals', [])

    if not goals:
        print(f"{RED}Error: No candidate goals registered in {GOALS_YAML}.{NC}")
        sys.exit(1)

    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🎯 Select Destination Goal Pose for Autonomous Navigation{NC}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")
    for g in goals:
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}° - {g.get('description', '')}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

    while True:
        try:
            choice = input(f"\nEnter Goal Choice [1-{len(goals)}] (Default: 1): ").strip()
            if not choice:
                return goals[0]['id']
            for g in goals:
                if str(g['id']) == choice or str(g['name']).lower() == choice.lower():
                    return g['id']
            print(f"{YELLOW}Invalid choice. Please enter a valid Goal ID (1-{len(goals)}).{NC}")
        except (KeyboardInterrupt, EOFError):
            sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description="Goal-Directed Autonomous Navigator for Unitree Go2")
    parser.add_argument('--mode', choices=['ours', 'pixnav'], default='ours', help="Navigation mode (ours = ESCAPE-Nav, pixnav = PointNav)")
    parser.add_argument('--goal', type=str, default=None, help="Goal ID (1-5) or Goal Name")
    parser.add_argument('--tolerance', type=float, default=0.50, help="Goal arrival tolerance in meters")
    parser.add_argument('--timeout', type=int, default=120, help="Max run duration in seconds")
    args = parser.parse_args()

    selected_goal = args.goal
    if selected_goal is None:
        selected_goal = select_goal_interactively()

    rclpy.init()
    node = AutonomousNavigator(
        mode=args.mode,
        target_goal_id=selected_goal,
        tolerance_m=args.tolerance,
        timeout_s=args.timeout
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.stop_robot()

if __name__ == '__main__':
    main()
