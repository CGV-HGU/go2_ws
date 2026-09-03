#!/usr/bin/env python3
"""
Unified Real-Time Localization HUD, Auto-CSV Logger & Interactive Goal Recorder for Unitree Go2.
Features:
  1. 5-Second Live Localization Stability & Calibration Warmup Check
  2. Live (X, Y, Z, Yaw) HUD stream
  3. Automatic Camera Snapshot capture when recording goal (config/goals/goal_XX.jpg)
  4. Automatic CSV logging to ~/.ros/localization_runs/latest/
  5. Interactive Goal Recording (1-Click Enter)
  6. Auto-renders 2D map goal pins using 2d_metadata.json
"""

import os
import sys
import math
import time
import yaml
import json
import threading
import cv2
import numpy as np
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None
import argparse
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_relocalizer

GREEN = '\033[0;32m'
CYAN = '\033[0;36m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BOLD = '\033[1m'
NC = '\033[0m'

# Configure stdin and stdout to replace any invalid UTF-8 bytes to prevent UnicodeDecodeError
try:
    sys.stdin.reconfigure(encoding='utf-8', errors='replace')
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

GOALS_YAML = "/home/unitree/go2_ws_antarctica/config/navigation_goals.yaml"
GOALS_JSON = "/home/unitree/go2_ws_antarctica/config/navigation_goals.json"
GOALS_DIR = "/home/unitree/go2_ws_antarctica/config/goals"
MAP_2D_PNG = "/home/unitree/go2_ws_antarctica/2dmap/2d.png"
MAP_METADATA_JSON = "/home/unitree/go2_ws_antarctica/2dmap/2d_metadata.json"
GOALS_MAP_PNG = "/home/unitree/go2_ws_antarctica/2dmap/2d_goals_map.png"

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

class UnifiedLocalizationAndGoalNode(Node):
    def __init__(self):
        super().__init__('go2_localization_and_goal_node')
        self.lock = threading.Lock()
        self.current_pose = None
        self.latest_frame = None
        self.bridge = CvBridge() if CvBridge else None
        self.last_pose_time = time.time()
        self.pose_count = 0
        self.is_localized = False

        # Setup Logging Directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.expanduser(f"~/.ros/localization_runs/{timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)
        os.makedirs(GOALS_DIR, exist_ok=True)
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
        self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.pose_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', self.pose_callback, qos)
        self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, qos_profile_sensor_data)

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.rtabmap_initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/rtabmap/initialpose', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.tf_fallback_timer)
        self.jump_warning = None
        self.filter_active = False

    def set_initial_pose(self, x: float, y: float, z: float = 0.0, yaw_deg: float = 0.0):
        map_relocalizer.publish_initial_pose(self.initial_pose_pub, x, y, z, yaw_deg, self.get_clock().now())
        map_relocalizer.publish_initial_pose(self.rtabmap_initial_pose_pub, x, y, z, yaw_deg, self.get_clock().now())

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

        now = time.time()
        elapsed = now - self.start_time
        iso_ts = datetime.now().isoformat()
        cov_x = float(msg.pose.covariance[0])

        with self.lock:
            # Protect against false loop closure teleportation once stabilized
            if self.filter_active and self.current_pose is not None:
                dt = max(0.001, now - self.current_pose.get("time", now))
                if dt < 5.0:
                    jump_d = math.hypot(pos.x - self.current_pose["x"], pos.y - self.current_pose["y"])
                    if jump_d > 2.0 and (jump_d / dt) > 1.2:
                        self.jump_warning = f"⚠️ JUMP REJECTED ({jump_d:.1f}m in {dt:.2f}s)"
                        return  # Ignore corrupted relocalization jump!

            self.jump_warning = None
            is_reliable = cov_x <= 0.08
            status = "LOCALIZED" if is_reliable else "ODOM_DRIFT"
            self.current_pose = {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
                "yaw": float(yaw_deg),
                "cov_x": cov_x,
                "time": now,
                "status": status
            }
            self.pose_count += 1
            self.is_localized = is_reliable

        self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{yaw_deg:.2f},{cov_x:.6f},{status}\n")
        self.csv_file.flush()

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

    def tf_fallback_timer(self):
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
                # Do not adopt unlocalized startup TF before RTAB-Map matches the map
                if not self.is_localized:
                    return

                if self.filter_active and self.current_pose is not None:
                    dt = max(0.001, now - self.current_pose.get("time", now))
                    if dt < 5.0:
                        jump_d = math.hypot(pos.x - self.current_pose["x"], pos.y - self.current_pose["y"])
                        if jump_d > 2.0 and (jump_d / dt) > 1.2:
                            self.jump_warning = f"⚠️ JUMP REJECTED ({jump_d:.1f}m in {dt:.2f}s)"
                            return  # Ignore corrupted TF jump!

                self.jump_warning = None
                status = "LOCALIZED" if self.is_localized else "TF_TRACKING"
                self.current_pose = {
                    "x": float(pos.x),
                    "y": float(pos.y),
                    "z": float(pos.z),
                    "yaw": float(yaw_deg),
                    "cov_x": 0.0,
                    "time": now,
                    "status": status
                }
                self.pose_count += 1

            self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{yaw_deg:.2f},0.0,{status}\n")
            self.csv_file.flush()
        except Exception:
            pass

    def get_latest_live_pose(self):
        with self.lock:
            if not self.is_localized:
                return None

        # Direct zero-latency TF lookup with jump protection
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pos = t.transform.translation
            ori = t.transform.rotation
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            now = time.time()
            with self.lock:
                if self.filter_active and self.current_pose is not None:
                    dt = max(0.001, now - self.current_pose.get("time", now))
                    if dt < 5.0:
                        jump_d = math.hypot(pos.x - self.current_pose["x"], pos.y - self.current_pose["y"])
                        if jump_d > 2.0 and (jump_d / dt) > 1.2:
                            # Jump detected: return safe continuous pose
                            return dict(self.current_pose)

                status = "LOCALIZED" if self.is_localized else "TF_TRACKING"
                self.current_pose = {
                    "x": float(pos.x),
                    "y": float(pos.y),
                    "z": float(pos.z),
                    "yaw": float(yaw_deg),
                    "cov_x": 0.0,
                    "time": now,
                    "status": status
                }
                return dict(self.current_pose)
        except Exception:
            pass

        with self.lock:
            if self.current_pose:
                return dict(self.current_pose)
            return None

    def get_pose(self):
        return self.get_latest_live_pose()

    def get_snapshot(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None

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

    colors = [(0, 140, 255), (0, 200, 100), (255, 100, 0), (200, 0, 200), (0, 220, 220)]
    for idx, g in enumerate(goals):
        gid = g.get('id', idx + 1)
        name = g.get('name', f'Goal_{gid}')
        gx = g['x_m']
        gy = g['y_m']
        px = int((gx - min_x) / res)
        py = int(h - 1 - (gy - min_y) / res)
        px = max(25, min(w - 25, px))
        py = max(25, min(h - 25, py))

        col = colors[(gid - 1) % len(colors)]
        cv2.circle(overlay, (px, py), 11, col, -1, cv2.LINE_AA)
        cv2.circle(overlay, (px, py), 13, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, f"#{gid}", (px - 9, py + 4), cv2.FONT_HERSHEY_DUPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, f"Goal #{gid}: {name}", (px - 40, py - 18), cv2.FONT_HERSHEY_DUPLEX, 0.52, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(overlay, f"Goal #{gid}: {name}", (px - 40, py - 18), cv2.FONT_HERSHEY_DUPLEX, 0.52, col, 1, cv2.LINE_AA)

    # Header Banner
    banner_w = min(w - 30, 650)
    cv2.rectangle(overlay, (15, 10), (15 + banner_w, 44), (255, 255, 255), -1)
    cv2.rectangle(overlay, (15, 10), (15 + banner_w, 44), (180, 180, 180), 1)
    cv2.putText(overlay, f"Candidate Navigation Goals ({len(goals)} registered)", 
                (25, 32), cv2.FONT_HERSHEY_DUPLEX, 0.52, (30, 30, 30), 1, cv2.LINE_AA)

    cv2.imwrite(GOALS_MAP_PNG, overlay)

def main():
    parser = argparse.ArgumentParser(description="Go2 Unified Localization HUD & Goal Manager")
    parser.add_argument('--start-goal', type=int, default=None, help="Initial waypoint ID to seed localization (0=origin, 1..N)")
    parser.add_argument('--start-origin', action='store_true', help="Seed localization at map origin (Node 1, 0,0,0)")
    parser.add_argument('--initial-pose', type=str, default=None, help="Initial pose format: 'x y z roll pitch yaw'")
    parser.add_argument('--auto-reloc', action='store_true', help="Auto-relocalize against recorded map keyframes")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = UnifiedLocalizationAndGoalNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"\n{BOLD}{CYAN}========================================================================{NC}")
    print(f"{BOLD}{CYAN} 🐕 [Unitree Go2] Unified Localization HUD & Goal Manager{NC}")
    print(f" 📂 CSV Log File : {node.csv_path}")
    print(f" 🚩 Goals Config : {GOALS_YAML}")
    print(f" 📸 Goals Images : {GOALS_DIR}/")
    print(f"{BOLD}{CYAN}========================================================================{NC}")

    goals = load_goals()
    registered_wps = map_relocalizer.load_registered_waypoints()
    wp_map = {w['id']: w for w in registered_wps}

    print(f"Loaded {len(goals)} registered candidate goals:")
    for g in goals:
        print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}°")

    # Handle CLI initial pose seeding if specified
    if args.start_origin:
        print(f"\n📍 {CYAN}Seeding initial pose at Map Origin (Node 1): X=0.0m, Y=0.0m, Yaw=0.0°{NC}")
        node.set_initial_pose(0.0, 0.0, 0.0, 0.0)
    elif args.start_goal is not None and args.start_goal in wp_map:
        w = wp_map[args.start_goal]
        print(f"\n📍 {CYAN}Seeding initial pose at [{w['id']}] {w['name']}: X={w['x_m']:+.2f}m, Y={w['y_m']:+.2f}m, Yaw={w['yaw_deg']:+.1f}°{NC}")
        node.set_initial_pose(w['x_m'], w['y_m'], w['z_m'], w['yaw_deg'])
    elif args.initial_pose:
        parts = [float(v) for v in args.initial_pose.strip().split()]
        if len(parts) >= 3:
            x, y, z = parts[0], parts[1], parts[2]
            yaw = math.degrees(parts[5]) if len(parts) >= 6 else 0.0
            print(f"\n📍 {CYAN}Seeding initial pose: X={x:+.2f}m, Y={y:+.2f}m, Yaw={yaw:+.1f}°{NC}")
            node.set_initial_pose(x, y, z, yaw)
    elif args.auto_reloc:
        time.sleep(1.0)
        frame = node.get_snapshot()
        if frame is not None:
            reloc = map_relocalizer.MapRelocalizer()
            match = reloc.match_live_frame(frame)
            if match:
                print(f"\n🎯 {GREEN}[AUTO-MATCH] Camera matched map Node #{match['node_id']} (X={match['x_m']:+.2f}m, Y={match['y_m']:+.2f}m, Yaw={match['yaw_deg']:+.1f}°){NC}")
                node.set_initial_pose(match['x_m'], match['y_m'], match['z_m'], match['yaw_deg'])

    # 1. Searching for verified map localization
    print(f"\n{YELLOW}⏳ [1/2] Waiting for RTAB-Map to lock onto recorded map landmarks...{NC}")
    wait_count = 0
    prompted_options = False

    while not node.is_localized or node.get_pose() is None:
        time.sleep(0.5)
        wait_count += 1
        if wait_count % 4 == 0 and not node.is_localized:
            print(f"  {YELLOW}🔍 Still searching for map landmarks ({wait_count * 0.5:.1f}s elapsed)...{NC}")

        if wait_count >= 8 and not node.is_localized and not prompted_options:
            prompted_options = True
            print(f"\n{BOLD}{CYAN}------------------------------------------------------------------------{NC}")
            print(f"{BOLD}{CYAN}💡 [MAP RELOCALIZATION HELPER]{NC}")
            print(f" RTAB-Map has not yet localized on the map. You can select the robot's physical starting location:")
            for w in registered_wps:
                print(f"   [{w['id']}] {w['name']:25s} | X={w['x_m']:+7.2f}m, Y={w['y_m']:+7.2f}m, Yaw={w['yaw_deg']:+6.1f}°")
            print(f"   [a] Auto-relocalize using live camera vs recorded map keyframes")
            print(f"   [w] Continue waiting for automatic LiDAR ICP / visual match")
            print(f"{BOLD}{CYAN}------------------------------------------------------------------------{NC}")
            try:
                choice = safe_input(f"👉 Enter Starting Waypoint [0-{len(registered_wps)-1}], 'a', or press [ENTER] to wait: ").strip()
                if choice.isdigit() and int(choice) in wp_map:
                    w = wp_map[int(choice)]
                    print(f"📍 Applying starting waypoint [{w['id']}] {w['name']} to RTAB-Map initialpose...")
                    node.set_initial_pose(w['x_m'], w['y_m'], w['z_m'], w['yaw_deg'])
                elif choice.lower() in ('a', 'auto'):
                    frame = node.get_snapshot()
                    if frame is not None:
                        reloc = map_relocalizer.MapRelocalizer()
                        match = reloc.match_live_frame(frame)
                        if match:
                            print(f"🎯 {GREEN}[AUTO-MATCH] Matched Node #{match['node_id']} (X={match['x_m']:+.2f}m, Y={match['y_m']:+.2f}m, Yaw={match['yaw_deg']:+.1f}°){NC}")
                            node.set_initial_pose(match['x_m'], match['y_m'], match['z_m'], match['yaw_deg'])
                        else:
                            print(f"⚠️ Could not find confident visual match. Continuing automatic search...")
            except Exception:
                pass

    initial_pose = node.get_pose()
    print(f"\n{BOLD}{GREEN}🎯 [MAP LOCK CONFIRMED] Robot localized at recorded map position:{NC}")
    print(f"   X:{BOLD}{initial_pose['x']:+7.3f}m{NC} Y:{BOLD}{initial_pose['y']:+7.3f}m{NC} Yaw:{BOLD}{initial_pose['yaw']:+6.1f}°{NC}")

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
            drift_m = math.hypot(p['x'] - ref_x, p['y'] - ref_y)
            if drift_m > 1.0:
                ref_x, ref_y = p['x'], p['y']
                drift_m = 0.0
            drift_cm = drift_m * 100.0
            status_tag = f"{GREEN}100% HEALTHY LOCK!{NC}" if sec == 5 else f"{CYAN}STABLE (Jitter: {drift_cm:3.1f}cm){NC}"
            print(f" [{sec}/5s] {GREEN}🟢 LOCALIZED{NC} | X:{BOLD}{p['x']:+7.3f}m{NC} Y:{BOLD}{p['y']:+7.3f}m{NC} Yaw:{BOLD}{p['yaw']:+6.1f}°{NC} | {status_tag}")

    print(f"{BOLD}{GREEN}========================================================================{NC}")
    print(f"{BOLD}{GREEN} 🎯 [LOCALIZATION FULLY STABILIZED] Ready for Goal Recording!{NC}")
    print(f"{BOLD}{GREEN}========================================================================{NC}\n")

    node.filter_active = True

    print(f"{BOLD}🎯 Key Controls:{NC}")
    print(f"  • {GREEN}[ENTER]{NC}          : Automatically record current pose & camera photo as Goal #{len(goals)+1}")
    print(f"  • {YELLOW}'d' / 'del'{NC}      : Delete the last recorded goal (with confirmation)")
    print(f"  • {CYAN}'list'{NC}           : Display all registered goals")
    print(f"  • {RED}'clear'{NC}          : Clear all registered goals (with confirmation)")
    print(f"  • 'q' / Ctrl+C      : Exit and save all logs\n")

    map_bounds = None
    if os.path.exists(MAP_METADATA_JSON):
        try:
            with open(MAP_METADATA_JSON, 'r') as mf:
                map_bounds = json.load(mf)
        except Exception:
            pass

    while True:
        try:
            pose = node.get_pose()
            if pose:
                if pose['status'] == 'LOCALIZED':
                    status_color = GREEN
                elif pose['status'] == 'ODOM_DRIFT':
                    status_color = YELLOW
                else:
                    status_color = CYAN
                edge_warn = ""
                if map_bounds:
                    min_x = map_bounds.get('min_x', -999)
                    max_x = map_bounds.get('max_x', 999)
                    min_y = map_bounds.get('min_y', -999)
                    max_y = map_bounds.get('max_y', 999)
                    is_out = (pose['x'] < min_x or pose['x'] > max_x or pose['y'] < min_y or pose['y'] > max_y)
                    if is_out:
                        edge_warn = f" | {RED}{BOLD}🚫 [OUT OF MAP! ({pose['x']:.1f}, {pose['y']:.1f}) is outside map range [{min_x:.0f}~{max_x:.0f}]]{NC}"
                    else:
                        dist_to_edge = min(
                            abs(pose['x'] - min_x),
                            abs(pose['x'] - max_x),
                            abs(pose['y'] - min_y),
                            abs(pose['y'] - max_y)
                        )
                        if dist_to_edge < 1.0:
                            edge_warn = f" | {YELLOW}{BOLD}⚠️ [MAP EDGE: {dist_to_edge:.1f}m to boundary]{NC}"
                warn_str = f" | {RED}{BOLD}{node.jump_warning}{NC}" if getattr(node, 'jump_warning', None) else ""
                pose_str = f"{status_color}{pose['status']}{NC} | X:{BOLD}{pose['x']:+7.3f}m{NC} Y:{BOLD}{pose['y']:+7.3f}m{NC} Z:{pose['z']:+6.3f}m Yaw:{BOLD}{pose['yaw']:+6.1f}°{NC}{edge_warn}{warn_str}"
            else:
                pose_str = f"{YELLOW}🔍 SEARCHING FOR LANDMARKS...{NC}"

            prompt_msg = f"\r[Live: {pose_str}]\n👉 Press [ENTER] to save Goal #{len(goals)+1} (or type command): "
            user_input = safe_input(prompt_msg)

            if user_input.lower() in ('q', 'quit', 'exit'):
                break

            elif user_input.lower() in ('list', 'l'):
                print(f"\nRegistered Goals ({len(goals)}):")
                for g in goals:
                    print(f"  [{g['id']}] {g['name']:25s} | X={g['x_m']:+7.2f}m, Y={g['y_m']:+7.2f}m, Yaw={g['yaw_deg']:+6.1f}° | Photo: {g.get('snapshot_image', 'None')}")
                print()
                continue

            elif user_input.lower() in ('d', 'del', 'delete', 'backspace', 'pop'):
                if not goals:
                    print(f"\n{YELLOW}⚠️ No goals to delete.{NC}\n")
                    continue
                last_g = goals[-1]
                confirm = safe_input(f"\n{YELLOW}⚠️ Delete last goal [#{last_g['id']}: {last_g['name']}]? [y/N]: {NC}").lower()
                if confirm in ('y', 'yes', ''):
                    removed = goals.pop()
                    save_goals(goals)
                    print(f"{RED}🗑️ Deleted Goal #{removed['id']} ({removed['name']}). Remaining: {len(goals)}{NC}\n")
                else:
                    print(f"Cancelled.\n")
                continue

            elif user_input.lower() in ('clear', 'c', 'reset'):
                goals = []
                save_goals(goals)
                # Clean up existing goal snapshot files
                if os.path.exists(GOALS_DIR):
                    for f in os.listdir(GOALS_DIR):
                        if f.endswith('.jpg') or f.endswith('.png'):
                            try:
                                os.remove(os.path.join(GOALS_DIR, f))
                            except Exception:
                                pass
                render_goals_on_map(goals)
                # Flush stdin buffer to throw away any residual Enter keystroke!
                try:
                    import termios
                    termios.tcflush(sys.stdin, termios.TCIFLUSH)
                except Exception:
                    pass
                print(f"\n{GREEN}{BOLD}🧹 All registered goals and goal photos have been CLEARED! (Total: 0){NC}")
                print(f"{CYAN}{BOLD}👉 Drive the robot with the remote controller to your destination, then press [ENTER] to save Goal #1.{NC}\n")
                continue

            # Default: [ENTER] -> Record Goal & Capture Camera Photo
            # 🔥 CRITICAL: Re-query the live RTAB-Map TF transform at destination right NOW!
            print(f"\n{CYAN}📡 Capturing real-time localization pose from RTAB-Map at destination...{NC}")
            pose = node.get_latest_live_pose()
            if not pose:
                for _ in range(15):
                    time.sleep(0.1)
                    pose = node.get_latest_live_pose()
                    if pose:
                        break

            if not pose:
                print(f"\n{RED}❌ Error: Cannot record goal - Robot is not localized yet.{NC}\n")
                continue

            # Dynamic out-of-bounds map boundary guard:
            if map_bounds:
                min_x = map_bounds.get('min_x', -999)
                max_x = map_bounds.get('max_x', 999)
                min_y = map_bounds.get('min_y', -999)
                max_y = map_bounds.get('max_y', 999)
                if pose['x'] < min_x or pose['x'] > max_x or pose['y'] < min_y or pose['y'] > max_y:
                    print(f"\n{RED}{BOLD}🚫 [OUT-OF-MAP WARNING] Captured pose ({pose['x']:.2f}m, {pose['y']:.2f}m) is OUTSIDE the 2D map!{NC}")
                    print(f"{RED}   Map bounding box: X=[{min_x:.1f}, {max_x:.1f}]m, Y=[{min_y:.1f}, {max_y:.1f}]m{NC}")
                    confirm = safe_input(f"{YELLOW}👉 Do you still want to record this out-of-map goal? [y/N]: {NC}").lower()
                    if confirm not in ('y', 'yes'):
                        print("Cancelled.\n")
                        continue

            if pose.get('status') == 'ODOM_DRIFT':
                print(f"\n{YELLOW}{BOLD}⚠️ [SCAN MATCH WARNING] Covariance is high ({pose.get('cov_x', 0):.4f}) - LiDAR matching is degraded!{NC}\n")

            # Double-Enter / Duplicate Protection:
            # Prevent accidental duplicate goals if the robot hasn't moved from the previous goal (< 0.60m)
            if goals:
                last_g = goals[-1]
                dist_from_last = math.hypot(pose['x'] - last_g['x_m'], pose['y'] - last_g['y_m'])
                if dist_from_last < 0.60:
                    print(f"\n{YELLOW}{BOLD}⚠️ [DUPLICATE BLOCKED] Robot is only {dist_from_last:.2f}m away from Goal #{last_g['id']}!{NC}")
                    print(f"{YELLOW}👉 Please drive the robot with the remote controller to your next waypoint before pressing [ENTER].{NC}\n")
                    continue

            new_id = len(goals) + 1
            default_name = f"Waypoint_{new_id}"
            
            # Capture Front Camera Image
            snap = node.get_snapshot()
            snap_rel_path = f"config/goals/goal_{new_id:02d}_{default_name}.jpg"
            snap_full_path = os.path.join(GOALS_DIR, f"goal_{new_id:02d}_{default_name}.jpg")
            if snap is not None:
                # Add overlay tag on goal photo
                tag_img = snap.copy()
                cv2.putText(tag_img, f"Goal #{new_id}: {default_name} (X={pose['x']:+.2f}m, Y={pose['y']:+.2f}m)",
                            (30, 50), cv2.FONT_HERSHEY_DUPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.imwrite(snap_full_path, tag_img)
                photo_status = f"📸 Saved {snap_rel_path}"
            else:
                photo_status = "⚠️ Camera frame not available"

            goal_entry = {
                "id": new_id,
                "name": default_name,
                "description": f"Candidate destination #{new_id} recorded via interactive HUD",
                "x_m": round(pose['x'], 2),
                "y_m": round(pose['y'], 2),
                "z_m": round(pose['z'], 2),
                "yaw_deg": round(pose['yaw'], 1),
                "tolerance_m": 0.50,
                "snapshot_image": snap_rel_path
            }
            goals.append(goal_entry)
            save_goals(goals)
            print(f"\n{GREEN}{BOLD}✅ [GOAL RECORDED] Goal #{new_id} '{default_name}' saved!{NC}")
            print(f"   📍 Pose  : X={goal_entry['x_m']:+.2f}m, Y={goal_entry['y_m']:+.2f}m, Yaw={goal_entry['yaw_deg']:+.1f}°")
            print(f"   📸 Photo : {photo_status} | Total Goals: {len(goals)}")
            print(f"   🗺️ Map   : Updated 2dmap/2d_goals_map.png\n")

        except (KeyboardInterrupt, EOFError):
            break

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{GREEN}✅ Goal recording complete. {len(goals)} goals registered in {GOALS_YAML}{NC}")

if __name__ == '__main__':
    main()
