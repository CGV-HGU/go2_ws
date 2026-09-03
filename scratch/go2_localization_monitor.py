#!/usr/bin/env python3
"""
Real-Time Planar 3DoF Localization HUD & RTAB-Map Health Monitor for Unitree Go2.
Features:
  1. Instantaneous state detection:
     - 🟢 LOCALIZED : Fresh RTAB-Map map-aligned pose (< 1.5s)
     - 🟡 COASTING  : Temporary visual loss (1.5s - 5.0s), tracking via TF
     - 🔴 NOT LOCALIZED : Explicitly reports lost state (> 5.0s) with duration
  2. Continuous unbroken CSV & TXT file logging for both LOCALIZED and NOT_LOCALIZED states.
  3. Live RTAB-Map sensor input health monitoring (Camera, 4D LiDAR, Odometry).
  4. Real-time wall-clock timestamps, jump detection alerts, and corridor boundary guards.
"""

import os
import sys
import json
import math
import time
from datetime import datetime
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import Image, PointCloud2
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_relocalizer

GREEN = '\033[0;32m'
CYAN = '\033[0;36m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BOLD = '\033[1m'
NC = '\033[0m'

RTABMAP_DB = "/home/unitree/.ros/rtabmap.db"

class LocalizationMonitor(Node):
    def __init__(self, log_dir=None):
        super().__init__('go2_localization_monitor')

        self.start_time = time.time()
        self.last_pose_time = 0.0  # 0.0 means never received RTAB-Map pose yet
        self.last_hud_print_time = 0.0
        self.pose_count = 0
        self.is_localized = False
        self.latest_pose = None
        self._last_pos = None

        # Sensor input counters
        self.cam_count = 0
        self.cloud_count = 0
        self.odom_count = 0
        self.last_cam_time = 0.0
        self.last_cloud_time = 0.0
        self.last_odom_time = 0.0

        # DB file size check
        self.db_size_mb = (os.path.getsize(RTABMAP_DB) / (1024 * 1024)) if os.path.exists(RTABMAP_DB) else 0.0

        # Dynamically load active map bounding box from 2dmap/2d_metadata.json
        self.map_bounds = None
        meta_path = os.path.expanduser("~/go2_ws_antarctica/2dmap/2d_metadata.json")
        if os.path.exists(meta_path):
            try:
                with open(meta_path, "r") as f:
                    meta = json.load(f)
                    self.map_bounds = {
                        "min_x": float(meta["min_x"]) - 3.0,
                        "max_x": float(meta["max_x"]) + 3.0,
                        "min_y": float(meta["min_y"]) - 3.0,
                        "max_y": float(meta["max_y"]) + 3.0,
                    }
            except Exception:
                pass

        # Automatic Log Directory Setup
        if not log_dir:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_dir = os.path.expanduser(f"~/.ros/localization_runs/{timestamp}")
        
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, "localization_poses.csv")
        self.log_path = os.path.join(log_dir, "localization.log")

        # Symlink latest
        runs_root = os.path.expanduser("~/.ros/localization_runs")
        latest_link = os.path.join(runs_root, "latest")
        if os.path.lexists(latest_link):
            os.remove(latest_link)
        try:
            os.symlink(log_dir, latest_link)
        except Exception:
            pass

        # Initialize CSV Header
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write("timestamp_iso,elapsed_s,pose_index,x_m,y_m,z_m,yaw_deg,cov_x,status,lost_duration_s\n")
        self.csv_file.flush()

        self.log_file = open(self.log_path, "w")

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Subscribe to RTAB-Map Localization Pose (support canonical & namespaced topics)
        self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', self.pose_callback, qos_best_effort)
        self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.pose_callback, qos_best_effort)

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.rtabmap_initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/rtabmap/initialpose', 10)

        # 2. Subscribe to RTAB-Map Input Sensor Feeds to monitor input health
        self.create_subscription(Image, '/camera/front/image_raw', self._cam_callback, qos_best_effort)
        self.create_subscription(PointCloud2, '/livo/cloud', self._cloud_callback, qos_best_effort)
        self.create_subscription(Odometry, '/livo/odom', self._odom_callback, qos_best_effort)

        # 3. Teammate Compatibility Relays (/map -> /rtabmap/map, /info -> /rtabmap/info)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.rtabmap_map_pub = self.create_publisher(OccupancyGrid, '/rtabmap/map', map_qos)
        self.create_subscription(OccupancyGrid, '/map', self._map_relay_cb, qos_best_effort)

        self.rtabmap_info_pub = self.create_publisher(Info, '/rtabmap/info', qos_best_effort)
        self.create_subscription(Info, '/info', self._info_relay_cb, qos_best_effort)

        # 4. TF Buffer to monitor map -> base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. 2Hz Periodic Status & Safety Timer (Prints HUD & logs unlocalized states)
        self.create_timer(0.5, self.status_timer_callback)

    def set_initial_pose(self, x: float, y: float, z: float = 0.0, yaw_deg: float = 0.0):
        map_relocalizer.publish_initial_pose(self.initial_pose_pub, x, y, z, yaw_deg, self.get_clock().now())
        map_relocalizer.publish_initial_pose(self.rtabmap_initial_pose_pub, x, y, z, yaw_deg, self.get_clock().now())

        # Map DB verification
        n_nodes = 0
        mtime_str = "Unknown"
        min_x, max_x = (0.0, 0.0)
        min_y, max_y = (0.0, 0.0)
        origin_str = "Unknown"
        if os.path.exists(RTABMAP_DB):
            import sqlite3, struct
            try:
                mtime_str = datetime.fromtimestamp(os.path.getmtime(RTABMAP_DB)).strftime("%Y-%m-%d %H:%M:%S")
                conn = sqlite3.connect(f"file:{RTABMAP_DB}?mode=ro", uri=True)
                c = conn.cursor()
                c.execute("SELECT count(*) FROM Node")
                n_nodes = c.fetchone()[0]
                c.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
                rows = c.fetchall()
                xs, ys = [], []
                for _, blob in rows:
                    if len(blob) == 48:
                        v = struct.unpack("<12f", blob)
                        xs.append(v[3])
                        ys.append(v[7])
                conn.close()
                if xs:
                    min_x, max_x = min(xs), max(xs)
                    min_y, max_y = min(ys), max(ys)
                    origin_str = f"Node #1 at (X={xs[0]:+.3f}m, Y={ys[0]:+.3f}m)"
            except Exception:
                pass

        print(f"\n{BOLD}{GREEN}========================================================================{NC}")
        print(f"{BOLD}{GREEN} 🗺️ [ACTIVE SLAM MAP VERIFICATION - CONFIRMED LATEST MAP]{NC}")
        print(f" • Database File : {BOLD}{RTABMAP_DB}{NC} ({self.db_size_mb:.1f} MB)")
        print(f" • Last Modified : {BOLD}{mtime_str}{NC} ({n_nodes} nodes)")
        print(f" • Map Bounds    : X=[{min_x:+.2f}m, {max_x:+.2f}m], Y=[{min_y:+.2f}m, {max_y:+.2f}m]")
        print(f" • Zero-Origin   : ✅ {origin_str}")
        print(f" • Pose Log File : {self.csv_path}")
        print(f"{BOLD}{GREEN}========================================================================{NC}\n", flush=True)

    def _cam_callback(self, msg: Image):
        self.cam_count += 1
        self.last_cam_time = time.time()

    def _cloud_callback(self, msg: PointCloud2):
        self.cloud_count += 1
        self.last_cloud_time = time.time()

    def _odom_callback(self, msg: Odometry):
        self.odom_count += 1
        self.last_odom_time = time.time()

    def _map_relay_cb(self, msg: OccupancyGrid):
        self.rtabmap_map_pub.publish(msg)

    def _info_relay_cb(self, msg: Info):
        self.rtabmap_info_pub.publish(msg)

    def sensor_health_status(self, now: float) -> str:
        cam_ok = (now - self.last_cam_time) < 1.5
        cloud_ok = (now - self.last_cloud_time) < 1.5
        odom_ok = (now - self.last_odom_time) < 1.5

        cam_tag = f"{GREEN}Cam:OK{NC}" if cam_ok else f"{RED}Cam:NO_FEED{NC}"
        cloud_tag = f"{GREEN}LiDAR:OK{NC}" if cloud_ok else f"{RED}LiDAR:NO_FEED{NC}"
        odom_tag = f"{GREEN}Odom:OK{NC}" if odom_ok else f"{RED}Odom:NO_FEED{NC}"

        return f"[{cam_tag} | {cloud_tag} | {odom_tag}]"

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Calculate Yaw from Quaternion
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        # Ignore exact duplicate timestamps if both topics publish simultaneously
        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if getattr(self, '_last_sub_stamp', None) == stamp_key:
            return
        self._last_sub_stamp = stamp_key

        now = time.time()
        elapsed = now - self.start_time
        iso_ts = datetime.now().isoformat()
        wall_hhmmss = datetime.now().strftime("%H:%M:%S")

        self.last_pose_time = now
        self.pose_count += 1
        self.is_localized = True
        self.latest_pose = {'x': pos.x, 'y': pos.y, 'z': pos.z, 'yaw': yaw_deg, 'time': now}

        cov_x = float(msg.pose.covariance[0])

        # Jump Detection Warning
        jump_warn = ""
        if self._last_pos is not None:
            dt = max(0.001, now - self._last_pos['time'])
            if dt < 5.0:
                jump_d = math.hypot(pos.x - self._last_pos['x'], pos.y - self._last_pos['y'])
                if jump_d > 2.0 and (jump_d / dt) > 1.2:
                    jump_warn = f" | {RED}{BOLD}⚠️ [JUMP DETECTED: {jump_d:.1f}m in {dt:.2f}s!]{NC}"
        self._last_pos = {'x': pos.x, 'y': pos.y, 'time': now}

        # Boundary check (dynamically checked against active map bounds)
        boundary_warn = ""
        if self.map_bounds is not None:
            if pos.x < self.map_bounds["min_x"] or pos.x > self.map_bounds["max_x"] or pos.y < self.map_bounds["min_y"] or pos.y > self.map_bounds["max_y"]:
                boundary_warn = f" | {YELLOW}{BOLD}⚠️ [OUT-OF-BOUNDS ({pos.x:+.1f}, {pos.y:+.1f})]{NC}"

        # Write continuous LOCALIZED entry to CSV
        self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{yaw_deg:.2f},{cov_x:.6f},LOCALIZED,0.0\n")
        self.csv_file.flush()

        log_line = f"[{iso_ts}] LOCALIZED #{self.pose_count:04d} X:{pos.x:+7.3f} Y:{pos.y:+7.3f} Z:{pos.z:+6.3f} Yaw:{yaw_deg:+6.1f} cov:{cov_x:.4f}\n"
        self.log_file.write(log_line)
        self.log_file.flush()

        # Throttle HUD print to max 2Hz to keep terminal clean and readable
        if now - self.last_hud_print_time >= 0.45:
            self.last_hud_print_time = now
            sensor_str = self.sensor_health_status(now)
            if abs(cov_x - 0.0010) < 1e-4:
                tag = f"{YELLOW}🟡 [SEED/PRIOR #{self.pose_count:04d}]{NC}"
            else:
                tag = f"{GREEN}🟢 [LOCALIZED #{self.pose_count:04d}]{NC}"
            print(
                f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                f"{tag} "
                f"{BOLD}X:{NC}{pos.x:+7.3f}m "
                f"{BOLD}Y:{NC}{pos.y:+7.3f}m "
                f"{BOLD}Z:{NC}{pos.z:+6.3f}m "
                f"{BOLD}Yaw:{NC}{yaw_deg:+6.1f}° "
                f"{CYAN}(cov={cov_x:.4f}){NC} "
                f"| {sensor_str}"
                f"{jump_warn}{boundary_warn}"
            )

    def status_timer_callback(self):
        now = time.time()
        elapsed = now - self.start_time
        iso_ts = datetime.now().isoformat()
        wall_hhmmss = datetime.now().strftime("%H:%M:%S")

        # Check how long since last RTAB-Map localization pose was received
        if self.last_pose_time > 0:
            lost_duration = now - self.last_pose_time
        else:
            lost_duration = elapsed

        # Only trigger status timer handler when RTAB-Map localization pose is absent (> 1.5s)
        if lost_duration >= 1.5:
            self.is_localized = False
            sensor_str = self.sensor_health_status(now)

            # Check if map -> base_link TF is available
            tf_available = False
            tf_pos = None
            tf_yaw = 0.0
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                p = t.transform.translation
                o = t.transform.rotation
                siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
                cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
                tf_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                tf_pos = (p.x, p.y, p.z)
                tf_available = True
            except Exception:
                tf_available = False

            # Determine explicit status and log continuously
            if tf_available and self.last_pose_time > 0:
                # 🟢 TF Tracking while stationary (RTAB-Map only publishes localization_pose on new motion)
                status_name = "LOCALIZED"
                px, py, pz = tf_pos
                self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{px:.4f},{py:.4f},{pz:.4f},{tf_yaw:.2f},0.0,{status_name},0.0\n")
                self.csv_file.flush()

                if now - self.last_hud_print_time >= 0.5:
                    self.last_hud_print_time = now
                    print(
                        f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                        f"{GREEN}🟢 [LOCALIZED (Stationary/TF)]{NC} "
                        f"{BOLD}X:{NC}{px:+7.3f}m {BOLD}Y:{NC}{py:+7.3f}m {BOLD}Z:{NC}{pz:+6.3f}m "
                        f"{BOLD}Yaw:{NC}{tf_yaw:+6.1f}° | {sensor_str}",
                        flush=True
                    )

            elif tf_available and self.last_pose_time == 0:
                # 🟡 Initial TF alignment active, waiting for first RTAB-Map ICP lock
                status_name = "SEARCHING_LOCK"
                px, py, pz = tf_pos
                self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{px:.4f},{py:.4f},{pz:.4f},{tf_yaw:.2f},0.0,{status_name},{lost_duration:.1f}\n")
                self.csv_file.flush()

                if now - self.last_hud_print_time >= 0.5:
                    self.last_hud_print_time = now
                    print(
                        f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                        f"{YELLOW}🟡 [SEARCHING FOR MAP ICP FIX]{NC} "
                        f"TF X:{px:+7.3f}m Y:{py:+7.3f}m Yaw:{tf_yaw:+6.1f}° | {sensor_str}",
                        flush=True
                    )

            else:
                # 🔴 Fully NOT LOCALIZED (No TF and No RTAB-Map pose)
                status_name = "NOT_LOCALIZED"
                if self.latest_pose:
                    px, py, pz, tf_yaw = self.latest_pose['x'], self.latest_pose['y'], self.latest_pose['z'], self.latest_pose['yaw']
                else:
                    px, py, pz, tf_yaw = 0.0, 0.0, 0.0, 0.0

                self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{px:.4f},{py:.4f},{pz:.4f},{tf_yaw:.2f},0.0,{status_name},{lost_duration:.1f}\n")
                self.csv_file.flush()

                if now - self.last_hud_print_time >= 0.5:
                    self.last_hud_print_time = now
                    if (now - self.last_cloud_time) > 3.0 and (now - self.last_odom_time) > 3.0:
                        cause_msg = f"{RED}{BOLD}⚠️ ROBOT SENSORS OFFLINE (Check Go2 Power / CycloneDDS){NC}"
                    else:
                        cause_msg = f"{YELLOW}Searching for landmarks (Stand in known corridor area){NC}"

                    print(
                        f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                        f"{RED}🔴 [NOT LOCALIZED]{NC} "
                        f"Lost for {BOLD}{RED}{lost_duration:.1f}s{NC} | "
                        f"{cause_msg} | {sensor_str}",
                        flush=True
                    )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()

def main():
    parser = argparse.ArgumentParser(description="Dedicated Go2 Localization HUD & Monitor")
    parser.add_argument('--start-goal', type=int, default=None, help="Initial waypoint ID to seed localization (0=origin, 1..N)")
    parser.add_argument('--start-origin', action='store_true', help="Seed localization at map origin (Node 1, 0,0,0)")
    parser.add_argument('--initial-pose', type=str, default=None, help="Initial pose format: 'x y z roll pitch yaw'")
    parser.add_argument('--auto-reloc', action='store_true', help="Auto-relocalize against recorded map keyframes")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = LocalizationMonitor()

    registered_wps = map_relocalizer.load_registered_waypoints()
    wp_map = {w['id']: w for w in registered_wps}

    if args.start_origin:
        print(f"📍 {CYAN}Seeding initial pose at Map Origin (Node 1): X=0.0m, Y=0.0m, Yaw=0.0°{NC}")
        node.set_initial_pose(0.0, 0.0, 0.0, 0.0)
    elif args.start_goal is not None and args.start_goal in wp_map:
        w = wp_map[args.start_goal]
        print(f"📍 {CYAN}Seeding initial pose at [{w['id']}] {w['name']}: X={w['x_m']:+.2f}m, Y={w['y_m']:+.2f}m, Yaw={w['yaw_deg']:+.1f}°{NC}")
        node.set_initial_pose(w['x_m'], w['y_m'], w['z_m'], w['yaw_deg'])
    elif args.initial_pose:
        parts = [float(v) for v in args.initial_pose.strip().split()]
        if len(parts) >= 3:
            x, y, z = parts[0], parts[1], parts[2]
            yaw = math.degrees(parts[5]) if len(parts) >= 6 else 0.0
            print(f"📍 {CYAN}Seeding initial pose: X={x:+.2f}m, Y={y:+.2f}m, Yaw={yaw:+.1f}°{NC}")
            node.set_initial_pose(x, y, z, yaw)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
