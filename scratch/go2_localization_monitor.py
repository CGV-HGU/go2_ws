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
import math
import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
import tf2_ros

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

        # 2. Subscribe to RTAB-Map Input Sensor Feeds to monitor input health
        self.create_subscription(Image, '/camera/front/image_raw', self._cam_callback, qos_best_effort)
        self.create_subscription(PointCloud2, '/livo/cloud', self._cloud_callback, qos_best_effort)
        self.create_subscription(Odometry, '/livo/odom', self._odom_callback, qos_best_effort)

        # 3. TF Buffer to monitor map -> base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 4. 2Hz Periodic Status & Safety Timer (Prints HUD & logs unlocalized states)
        self.create_timer(0.5, self.status_timer_callback)

        print(f"\n{BOLD}{CYAN}========================================================================{NC}")
        print(f"{BOLD}{CYAN} 🎯 [Go2 Real-Time Localization & RTAB-Map Sensor Health Monitor]{NC}")
        print(f" 📂 Log Dir : {log_dir}")
        print(f" 📄 CSV Log : {self.csv_path}")
        print(f" 🗺️ Map DB  : {RTABMAP_DB} ({self.db_size_mb:.1f} MB)")
        print(f"{BOLD}{CYAN}========================================================================{NC}\n")

    def _cam_callback(self, msg: Image):
        self.cam_count += 1
        self.last_cam_time = time.time()

    def _cloud_callback(self, msg: PointCloud2):
        self.cloud_count += 1
        self.last_cloud_time = time.time()

    def _odom_callback(self, msg: Odometry):
        self.odom_count += 1
        self.last_odom_time = time.time()

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

        # Boundary check
        boundary_warn = ""
        if pos.y < -26.5 or pos.y > -3.5:
            boundary_warn = f" | {YELLOW}{BOLD}⚠️ [OUT-OF-BOUNDS Y={pos.y:+.2f}m]{NC}"

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
            print(
                f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                f"{GREEN}🟢 [LOCALIZED #{self.pose_count:04d}]{NC} "
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
            if lost_duration < 5.0 and tf_available:
                # 🟡 Coasting on TF (Temporary visual feature dropout)
                status_name = "COASTING_TF"
                px, py, pz = tf_pos
                self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{px:.4f},{py:.4f},{pz:.4f},{tf_yaw:.2f},0.0,{status_name},{lost_duration:.1f}\n")
                self.csv_file.flush()
                self.log_file.write(f"[{iso_ts}] {status_name} (Lost for {lost_duration:.1f}s) X:{px:+7.3f} Y:{py:+7.3f} Yaw:{tf_yaw:+6.1f}\n")
                self.log_file.flush()

                if now - self.last_hud_print_time >= 0.5:
                    self.last_hud_print_time = now
                    print(
                        f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                        f"{YELLOW}🟡 [RELOCALIZING / COASTING]{NC} "
                        f"No fix for {BOLD}{lost_duration:.1f}s{NC} | "
                        f"TF X:{px:+7.3f}m Y:{py:+7.3f}m Yaw:{tf_yaw:+6.1f}° | {sensor_str}"
                    )

            else:
                # 🔴 Fully NOT LOCALIZED / LOST (> 5.0s or never localized)
                status_name = "NOT_LOCALIZED"
                if tf_available:
                    px, py, pz = tf_pos
                elif self.latest_pose:
                    px, py, pz, tf_yaw = self.latest_pose['x'], self.latest_pose['y'], self.latest_pose['z'], self.latest_pose['yaw']
                else:
                    px, py, pz, tf_yaw = 0.0, 0.0, 0.0, 0.0

                self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{px:.4f},{py:.4f},{pz:.4f},{tf_yaw:.2f},0.0,{status_name},{lost_duration:.1f}\n")
                self.csv_file.flush()
                self.log_file.write(f"[{iso_ts}] {status_name} (Lost for {lost_duration:.1f}s) - Searching for landmarks\n")
                self.log_file.flush()

                if now - self.last_hud_print_time >= 0.5:
                    self.last_hud_print_time = now
                    # Check if all sensors are disconnected
                    if (now - self.last_cloud_time) > 3.0 and (now - self.last_odom_time) > 3.0:
                        cause_msg = f"{RED}{BOLD}⚠️ ROBOT SENSORS OFFLINE (Check Go2 Power / CycloneDDS){NC}"
                    else:
                        cause_msg = f"{YELLOW}Searching for landmarks (Stand in known corridor area){NC}"

                    print(
                        f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                        f"{RED}🔴 [NOT LOCALIZED]{NC} "
                        f"Lost for {BOLD}{RED}{lost_duration:.1f}s{NC} | "
                        f"{cause_msg} | {sensor_str}"
                    )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
