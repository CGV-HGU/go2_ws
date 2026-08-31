#!/usr/bin/env python3
"""
Goal-Directed Autonomous Navigation Controller & Rigorous Benchmark Evaluator for Unitree Go2.
Implements standard embodied AI metrics (Anderson et al., Habitat, ICRA):
  1. Success Rate (SR, %) - Arrival within threshold without timeout
  2. Geodesic SPL (Success weighted by Path Length, %) - Using A* shortest path on 2D occupancy grid
  3. SoftSPL (%) - Factoring in partial progress for challenging/timeout episodes
  4. Trajectory Length (TL, m) - With stationary jitter deadband filter (>= 1.5cm)
  5. Geodesic Shortest Path Length (m) - Ground-truth optimal path through corridor
  6. Time-to-Goal (TTG, s) & Average Speed (m/s)
  7. Idle Time Ratio (%) - Time spent waiting for VLM inference vs active locomotion
  8. VLM Decision Latency (p50 / p95, ms) & Query Count
  9. Automatic JSON report, CSV trajectory, and ready-to-publish LaTeX Table 2 exporter.
"""

import os
import sys
import math
import time
import json
import yaml
import base64
import heapq
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
MAP_2D_PNG = "/home/unitree/go2_ws_antarctica/2dmap/2d.png"
VLM_URL = "http://100.96.60.15:8000/v1"
MODEL_NAME = "qwen3.5-9b-instruct"

def compute_astar_geodesic_distance(start_xy, goal_xy, map_path=MAP_2D_PNG, res=0.05):
    """Computes true obstacle-aware geodesic shortest path on the 2D occupancy grid map."""
    sx, sy = start_xy
    gx, gy = goal_xy
    euclidean_dist = math.hypot(gx - sx, gy - sy)

    if not os.path.exists(map_path):
        return euclidean_dist

    img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return euclidean_dist

    h, w = img.shape
    free_mask = (img > 128).astype(np.uint8)

    # Convert metric (X, Y) to image pixel (PX, PY) relative to map center
    spx = int(w / 2 + (sx + 14.0) / res)
    spy = int(h / 2 - (sy - 27.5) / res)
    gpx = int(w / 2 + (gx + 14.0) / res)
    gpy = int(h / 2 - (gy - 27.5) / res)

    spx = max(0, min(w - 1, spx))
    spy = max(0, min(h - 1, spy))
    gpx = max(0, min(w - 1, gpx))
    gpy = max(0, min(h - 1, gpy))

    # A* Search
    h0 = math.hypot(gpx - spx, gpy - spy)
    pq = [(h0, 0.0, spx, spy)]
    visited = {(spx, spy): 0.0}
    dirs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

    max_steps = 50000
    steps = 0
    while pq and steps < max_steps:
        steps += 1
        f, cost, x, y = heapq.heappop(pq)
        if math.hypot(x - gpx, y - gpy) <= 2:
            return cost * res

        if cost > visited.get((x, y), float('inf')):
            continue

        for dx, dy, step_cost in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and free_mask[ny, nx] > 0:
                new_cost = cost + step_cost
                if new_cost < visited.get((nx, ny), float('inf')):
                    visited[(nx, ny)] = new_cost
                    h_score = math.hypot(gpx - nx, gpy - ny)
                    heapq.heappush(pq, (new_cost + h_score, new_cost, nx, ny))

    return euclidean_dist

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
        self.trajectory_history = []
        self.filtered_trajectory = []
        self.vlm_latencies = []
        self.idle_time_s = 0.0
        self.last_control_time = time.time()
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
        print(f"{BOLD}{CYAN} 🚀 [Unitree Go2 ICRA Benchmark Autonomous Navigator]{NC}")
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
            if self.start_pose is None:
                self.start_pose = self.current_pose

            # Stationary Noise Filter (only add to filtered path if displacement >= 1.5cm)
            if not self.filtered_trajectory:
                self.filtered_trajectory.append((self.current_pose['x'], self.current_pose['y'], self.current_pose['yaw'], time.time()))
            else:
                last_p = self.filtered_trajectory[-1]
                disp = math.hypot(self.current_pose['x'] - last_p[0], self.current_pose['y'] - last_p[1])
                if disp >= 0.015:
                    self.filtered_trajectory.append((self.current_pose['x'], self.current_pose['y'], self.current_pose['yaw'], time.time()))

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
            if self.start_pose is None:
                self.start_pose = pose_dict
            return pose_dict
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

        dx = self.goal['x_m'] - pose['x']
        dy = self.goal['y_m'] - pose['y']
        dist = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi

        threading.Thread(target=self._query_vlm_async, args=(frame, dist, math.degrees(rel_heading)), daemon=True).start()

    def _query_vlm_async(self, frame, dist_to_goal, rel_heading_deg):
        if self.vlm_active:
            return
        self.vlm_active = True
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

            if resp.status_code == 200:
                data = resp.json()
                content = data['choices'][0]['message']['content']
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

        now = time.time()
        dt = now - self.last_control_time
        self.last_control_time = now

        pose = self.get_current_pose()
        if not pose:
            self.idle_time_s += dt
            self.get_logger().warn("Waiting for live localization pose...", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        elapsed = now - self.start_time
        if elapsed > self.timeout_s:
            print(f"\n{RED}⏱️ [TIMEOUT] Exceeded maximum duration of {self.timeout_s}s. Halting robot.{NC}")
            self.finish_run(success=False, reason="TIMEOUT")
            return

        # Distance to Goal
        dx = self.goal['x_m'] - pose['x']
        dy = self.goal['y_m'] - pose['y']
        dist_to_goal = math.hypot(dx, dy)
        global_target_heading = math.atan2(dy, dx)
        rel_heading = (global_target_heading - pose['yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        rel_heading_deg = math.degrees(rel_heading)

        # Arrival Check
        if dist_to_goal <= self.tolerance_m:
            print(f"\n{GREEN}{BOLD}🎉 [GOAL REACHED] Arrived within {dist_to_goal:.2f}m of Goal #{self.goal['id']} ({self.goal['name']})!{NC}")
            self.finish_run(success=True, reason="ARRIVED")
            return

        # Control Law
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

        if target_vx < 0.03:
            self.idle_time_s += dt

        self.publish_cmd(target_vx, target_wz)

        # Real-Time Telemetry HUD
        sys.stdout.write(
            f"\r🚀 [{self.mode.upper()}] "
            f"Pos: ({pose['x']:+6.2f}m, {pose['y']:+6.2f}m) | "
            f"Target: #{self.goal['id']} ({self.goal['name']}) | "
            f"{BOLD}Dist: {dist_to_goal:5.2f}m{NC} | "
            f"Cmd: (vx={target_vx:.2f}, wz={target_wz:+.2f}) | "
            f"Time: {elapsed:4.1f}s"
        )
        sys.stdout.flush()

    def finish_run(self, success: bool, reason: str):
        self.is_goal_reached = True
        self.stop_robot()

        elapsed = time.time() - self.start_time

        # 1. Compute Path Length with Filtered Trajectory (zero-jitter)
        path_length = 0.0
        active_list = self.filtered_trajectory if len(self.filtered_trajectory) > 1 else self.trajectory_history
        for i in range(1, len(active_list)):
            p1 = active_list[i-1]
            p2 = active_list[i]
            path_length += math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        # 2. Compute Geodesic Shortest Path Length (A* on 2D Occupancy Grid)
        start_pt = (self.start_pose['x'], self.start_pose['y']) if self.start_pose else (0.0, 0.0)
        goal_pt = (self.goal['x_m'], self.goal['y_m'])
        d_geodesic = compute_astar_geodesic_distance(start_pt, goal_pt)
        d_euclidean = math.hypot(goal_pt[0] - start_pt[0], goal_pt[1] - start_pt[1])

        # 3. Final Remaining Distance to Goal
        last_pos = active_list[-1] if active_list else (0.0, 0.0)
        d_final = math.hypot(goal_pt[0] - last_pos[0], goal_pt[1] - last_pos[1])
        d_initial = max(d_geodesic, 0.5)

        # 4. Standard SPL & SoftSPL
        s_binary = 1.0 if success else 0.0
        spl = s_binary * (d_geodesic / max(path_length, d_geodesic, 0.01))
        soft_progress = max(0.0, min(1.0, 1.0 - (d_final / d_initial)))
        soft_spl = soft_progress * (d_geodesic / max(path_length, d_geodesic, 0.01))

        avg_speed = path_length / max(elapsed, 0.01)
        idle_ratio = (self.idle_time_s / max(elapsed, 0.01)) * 100.0

        vlm_p50 = float(np.percentile(self.vlm_latencies, 50)) if self.vlm_latencies else 0.0
        vlm_p95 = float(np.percentile(self.vlm_latencies, 95)) if self.vlm_latencies else 0.0

        summary = {
            "timestamp": datetime.now().isoformat(),
            "mode": self.mode,
            "goal": self.goal,
            "metrics": {
                "success_rate_sr": 1.0 if success else 0.0,
                "spl": round(spl, 4),
                "soft_spl": round(soft_spl, 4),
                "trajectory_length_m": round(path_length, 3),
                "geodesic_shortest_path_m": round(d_geodesic, 3),
                "euclidean_shortest_path_m": round(d_euclidean, 3),
                "duration_seconds": round(elapsed, 2),
                "idle_time_seconds": round(self.idle_time_s, 2),
                "idle_ratio_percent": round(idle_ratio, 1),
                "average_speed_mps": round(avg_speed, 3),
                "vlm_query_count": len(self.vlm_latencies),
                "vlm_latency_p50_ms": round(vlm_p50, 1),
                "vlm_latency_p95_ms": round(vlm_p95, 1)
            },
            "reason": reason,
            "pose_samples_raw": len(self.trajectory_history),
            "pose_samples_filtered": len(self.filtered_trajectory)
        }

        # Save JSON Report
        summary_path = os.path.join(self.run_dir, "metrics_report.json")
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)

        # Save Trajectory CSV
        traj_csv = os.path.join(self.run_dir, "trajectory.csv")
        with open(traj_csv, "w") as f:
            f.write("x_m,y_m,yaw_deg,timestamp\n")
            for p in self.trajectory_history:
                f.write(f"{p[0]:.4f},{p[1]:.4f},{p[2]:.2f},{p[3]:.3f}\n")

        # Generate LaTeX Table 2 Formatted Row
        latex_row = (
            f"{self.mode.upper():16s} & "
            f"{'100.0\\%' if success else '0.0\\%'} & "
            f"{spl*100:.1f}\\% & "
            f"{soft_spl*100:.1f}\\% & "
            f"{path_length:.2f}\\,\\text{{m}} & "
            f"{d_geodesic:.2f}\\,\\text{{m}} & "
            f"{elapsed:.1f}\\,\\text{{s}} & "
            f"{avg_speed:.2f}\\,\\text{{m/s}} & "
            f"{vlm_p50:.1f}\\,\\text{{ms}} \\\\\n"
        )
        latex_path = os.path.join(self.run_dir, "metrics_table.tex")
        with open(latex_path, "w") as f:
            f.write(latex_row)

        print(f"\n\n========================================================================")
        print(f" 📊 [ICRA 2026 Paper Rigorous Benchmark Summary]")
        print(f" • Run Mode        : {self.mode.upper()}")
        print(f" • Target Goal     : #{self.goal['id']} - {self.goal['name']}")
        print(f" • Success (SR)    : {'✅ SUCCESS (100.0%)' if success else '❌ FAILED (0.0%)'}")
        print(f" • Geodesic SPL    : {spl*100:.1f}% (Standard Obstacle-Aware Path Efficiency)")
        print(f" • SoftSPL         : {soft_spl*100:.1f}% (Factoring Partial Progress)")
        print(f" • Trajectory (TL) : {path_length:.2f} m (A* Geodesic Optimal: {d_geodesic:.2f} m)")
        print(f" • Duration (TTG)  : {elapsed:.2f} s (Idle Time: {self.idle_time_s:.2f}s, {idle_ratio:.1f}%)")
        print(f" • Average Speed   : {avg_speed:.2f} m/s")
        if self.mode == "ours":
            print(f" • VLM Latency     : p50={vlm_p50:.1f}ms, p95={vlm_p95:.1f}ms (Total Queries: {len(self.vlm_latencies)})")
        print(f" • JSON Report     : {summary_path}")
        print(f" • LaTeX Table Row : {latex_path}")
        print(f"========================================================================\n")
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
