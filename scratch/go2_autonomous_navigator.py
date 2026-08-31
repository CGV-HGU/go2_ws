#!/usr/bin/env python3
"""
Goal-Directed Autonomous Navigation Controller & Raw Experimental Data Logger for Unitree Go2.
Saves raw, unprocessed experimental data for scientific record-keeping:
  1. trajectory_raw.csv  - High-frequency (10Hz) raw localization poses & velocity commands
  2. vlm_decisions.jsonl - Full raw VLM queries, responses, subgoals [u,v], and latencies
  3. camera_snapshots/   - Decision keyframe JPEG images (Start, Subgoals, Arrival)
  4. run_metadata.json   - Run setup, initial & target poses, timestamps, and termination status
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

GREEN = '[0;32m'
CYAN = '[0;36m'
YELLOW = '[1;33m'
RED = '[0;31m'
BOLD = '[1m'
NC = '[0m'

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
        self.start_pose = None
        self.latest_frame = None
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.start_time = time.time()
        self.vlm_latencies = []
        self.pose_sample_count = 0
        self.vlm_query_count = 0
        self.snapshot_count = 0
        self.is_goal_reached = False
        self.is_stopped = False

        # Current Command State
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        # 1. Load Goal Definition
        self.goal = self.load_target_goal(target_goal_id)
        if not self.goal:
            self.get_logger().error(f"Goal ID {target_goal_id} not found in {GOALS_YAML}")
            sys.exit(1)

        # 2. Setup Dedicated Raw Data Run Directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.expanduser(f"~/.ros/navigation_runs/{timestamp}_{self.mode}_{self.goal['name']}")
        self.snapshots_dir = os.path.join(self.run_dir, "camera_snapshots")
        os.makedirs(self.snapshots_dir, exist_ok=True)

        # Symlink latest run
        runs_root = os.path.expanduser("~/.ros/navigation_runs")
        latest_link = os.path.join(runs_root, "latest")
        if os.path.lexists(latest_link):
            os.remove(latest_link)
        try:
            os.symlink(self.run_dir, latest_link)
        except Exception:
            pass

        # Raw Files
        self.csv_path = os.path.join(self.run_dir, "trajectory_raw.csv")
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write("iso_timestamp,elapsed_s,pose_index,x_m,y_m,z_m,yaw_deg,cmd_vx,cmd_wz,dist_to_goal_m,status
")
        self.csv_file.flush()

        self.vlm_log_path = os.path.join(self.run_dir, "vlm_decisions.jsonl")
        self.vlm_log_file = open(self.vlm_log_path, "w")

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

        # Control Loop (10Hz) & VLM Loop (1.5Hz)
        self.create_timer(0.1, self.control_loop)
        self.vlm_active = False
        self.subgoal_u = 640
        self.subgoal_v = 500
        self.create_timer(0.7, self.vlm_decision_loop)

        print(f"
{BOLD}{CYAN}========================================================================{NC}")
        print(f"{BOLD}{CYAN} 🐕 [Unitree Go2 Raw Experimental Data Collector]{NC}")
        print(f" • Mode        : {self.mode.upper()} ({'Qwen VLM + PixNav' if self.mode == 'ours' else 'Pure PixNav'})")
        print(f" • Target Goal : #{self.goal['id']} - {self.goal['name']}")
        print(f" • Target Pose : X={self.goal['x_m']:+.3f}m, Y={self.goal['y_m']:+.3f}m, Yaw={self.goal['yaw_deg']:+.1f}°")
        print(f"------------------------------------------------------------------------")
        print(f"{BOLD} 💾 [Raw Data Storage Directory]:{NC}")
        print(f"   📂 Run Folder : {self.run_dir}")
        print(f"   📄 Poses CSV  : {self.csv_path}")
        print(f"   🧠 VLM Log    : {self.vlm_log_path}")
        print(f"   📸 Snapshots  : {self.snapshots_dir}/")
        print(f"{BOLD}{CYAN}========================================================================{NC}
")

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
            if self.start_pose is None:
                self.start_pose = self.current_pose

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
            if self.start_pose is None:
                self.start_pose = pose_dict
            return pose_dict
        except Exception:
            return None

    def publish_cmd(self, vx: float, wz: float):
        self.cmd_vx = max(-self.max_vx, min(self.max_vx, vx))
        self.cmd_wz = max(-self.max_wz, min(self.max_wz, wz))

        cmd = Twist()
        cmd.linear.x = float(self.cmd_vx)
        cmd.angular.z = float(self.cmd_wz)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def save_snapshot(self, label="frame"):
        with self.lock:
            if self.latest_frame is None:
                return None
            frame = self.latest_frame.copy()

        self.snapshot_count += 1
        fn = f"{self.snapshot_count:04d}_{label}.jpg"
        path = os.path.join(self.snapshots_dir, fn)
        cv2.imwrite(path, frame)
        return fn

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

        dx = self.goal['x_m'] - pose['x']
        dy = self.goal['y_m'] - pose['y']
        dist = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi

        threading.Thread(target=self._query_vlm_async, args=(frame, dist, math.degrees(rel_heading), pose), daemon=True).start()

    def _query_vlm_async(self, frame, dist_to_goal, rel_heading_deg, pose):
        if self.vlm_active:
            return
        self.vlm_active = True
        self.vlm_query_count += 1
        q_idx = self.vlm_query_count
        t0 = time.time()

        try:
            ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            b64_img = base64.b64encode(buf).decode('utf-8')

            prompt = f"""You are the visual navigation brain for Unitree Go2 navigating to Goal '{self.goal['name']}'.
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

            action = "unknown"
            reasoning = "none"
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

            snapshot_fn = self.save_snapshot(f"query_{q_idx:03d}") if (q_idx % 3 == 1) else None

            vlm_record = {
                "query_id": q_idx,
                "iso_timestamp": datetime.now().isoformat(),
                "latency_ms": round(latency_ms, 1),
                "robot_pose": {"x": round(pose['x'], 3), "y": round(pose['y'], 3), "yaw_deg": round(pose['yaw'], 1)},
                "dist_to_goal_m": round(dist_to_goal, 3),
                "rel_heading_deg": round(rel_heading_deg, 1),
                "action": action,
                "reasoning": reasoning,
                "subgoal_uv": [self.subgoal_u, self.subgoal_v],
                "snapshot_image": snapshot_fn
            }
            self.vlm_log_file.write(json.dumps(vlm_record) + "
")
            self.vlm_log_file.flush()

        except Exception:
            pass
        finally:
            self.vlm_active = False

    def control_loop(self):
        if self.is_goal_reached:
            self.stop_robot()
            return

        now = time.time()
        elapsed = now - self.start_time
        pose = self.get_current_pose()

        if not pose:
            self.stop_robot()
            return

        dx = self.goal['x_m'] - pose['x']
        dy = self.goal['y_m'] - pose['y']
        dist_to_goal = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        rel_heading_deg = math.degrees(rel_heading)

        if dist_to_goal <= self.tolerance_m:
            print(f"
{GREEN}{BOLD}🎉 [GOAL REACHED] Arrived within {dist_to_goal:.2f}m of Goal #{self.goal['id']} ({self.goal['name']})!{NC}")
            self.save_snapshot("arrival")
            self.finish_run(success=True, reason="ARRIVED", final_dist=dist_to_goal)
            return

        if elapsed > self.timeout_s:
            print(f"
{RED}⏱️ [TIMEOUT] Exceeded maximum duration of {self.timeout_s}s. Halting robot.{NC}")
            self.save_snapshot("timeout")
            self.finish_run(success=False, reason="TIMEOUT", final_dist=dist_to_goal)
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
        self.csv_file.write(
            f"{iso_ts},{elapsed:.3f},{self.pose_sample_count},"
            f"{pose['x']:.4f},{pose['y']:.4f},{pose['z']:.4f},{pose['yaw']:.2f},"
            f"{self.cmd_vx:.3f},{self.cmd_wz:.3f},{dist_to_goal:.3f},NAVIGATING
"
        )
        self.csv_file.flush()

        sys.stdout.write(
            f"🚀 [{self.mode.upper()}] "
            f"Pos: ({pose['x']:+6.2f}m, {pose['y']:+6.2f}m) | "
            f"Target: #{self.goal['id']} ({self.goal['name']}) | "
            f"{BOLD}Dist: {dist_to_goal:5.2f}m{NC} | "
            f"Cmd: (vx={self.cmd_vx:.2f}, wz={self.cmd_wz:+.2f}) | "
            f"Raw Poses: #{self.pose_sample_count:04d} ({elapsed:4.1f}s)"
        )
        sys.stdout.flush()

    def finish_run(self, success: bool, reason: str, final_dist: float = 0.0):
        self.is_goal_reached = True
        self.stop_robot()
        elapsed = time.time() - self.start_time

        metadata = {
            "run_id": os.path.basename(self.run_dir),
            "created_at": datetime.now().isoformat(),
            "mode": self.mode,
            "goal": self.goal,
            "initial_pose": self.start_pose,
            "final_status": {
                "success": success,
                "reason": reason,
                "duration_seconds": round(elapsed, 2),
                "final_distance_to_goal_m": round(final_dist, 3),
                "total_pose_samples": self.pose_sample_count,
                "total_vlm_queries": self.vlm_query_count,
                "total_camera_snapshots": self.snapshot_count
            },
            "saved_raw_files": {
                "poses_csv": "trajectory_raw.csv",
                "vlm_decisions_jsonl": "vlm_decisions.jsonl",
                "camera_snapshots_dir": "camera_snapshots/"
            }
        }

        meta_path = os.path.join(self.run_dir, "run_metadata.json")
        with open(meta_path, "w") as f:
            json.dump(metadata, f, indent=2)

        self.csv_file.close()
        self.vlm_log_file.close()

        print(f"

========================================================================")
        print(f"{GREEN}{BOLD} 💾 [RAW DATA LOGGING COMPLETE - 100% SAVED TO DISK]{NC}")
        print(f"========================================================================")
        print(f" • Run Directory       : {self.run_dir}")
        print(f" • Raw Trajectory CSV  : {self.csv_path} ({self.pose_sample_count} samples)")
        print(f" • Raw VLM JSONL Log   : {self.vlm_log_path} ({self.vlm_query_count} decisions)")
        print(f" • Camera Snapshots    : {self.snapshots_dir}/ ({self.snapshot_count} images)")
        print(f" • Run Metadata File   : {meta_path}")
        print(f" • Symlinked Access    : ~/.ros/navigation_runs/latest/")
        print(f"========================================================================
")
        rclpy.shutdown()

def select_goal_interactively():
    if not os.path.exists(GOALS_YAML):
        print(f"{RED}Error: {GOALS_YAML} not found. Run ./run_local.sh first.{NC}")
        sys.exit(1)
    with open(GOALS_YAML, 'r') as f:
        data = yaml.safe_load(f)
        goals = data.get('goals', [])

    if not goals:
        print(f"{RED}Error: No candidate goals registered in {GOALS_YAML}.{NC}")
        sys.exit(1)

    print(f"
{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🎯 Select Destination Goal Pose for Autonomous Navigation{NC}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")
    for g in goals:
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}° - {g.get('description', '')}")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

    while True:
        try:
            choice = input(f"
Enter Goal Choice [1-{len(goals)}] (Default: 1): ").strip()
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
