#!/usr/bin/env python3
"""
Real-Time Planar 3DoF Localization HUD & Auto-File Logger for Unitree Go2.
Subscribes to /rtabmap/localization_pose, /map -> /base_link TF, and /rtabmap/info,
printing clean (X, Y, Z, Yaw) to console AND automatically logging to CSV and text files.
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
import tf2_ros

GREEN = '\033[0;32m'
CYAN = '\033[0;36m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BOLD = '\033[1m'
NC = '\033[0m'

class LocalizationMonitor(Node):
    def __init__(self, log_dir=None):
        super().__init__('go2_localization_monitor')

        self.last_pose_time = time.time()
        self.pose_count = 0
        self.is_localized = False

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
        self.csv_file.write("timestamp_iso,elapsed_s,pose_index,x_m,y_m,z_m,yaw_deg,cov_x,status\n")
        self.csv_file.flush()

        self.log_file = open(self.log_path, "w")
        self.start_time = time.time()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Subscribe to RTAB-Map Localization Pose (support both root and namespaced topics)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self.pose_callback,
            qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.pose_callback,
            qos
        )

        # 2. TF Buffer to monitor map -> base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3. 2Hz Status Timer (HUD print)
        self.create_timer(0.5, self.status_timer_callback)

        print(f"\n{BOLD}{CYAN}========================================================================{NC}")
        print(f"{BOLD}{CYAN} 🎯 [Go2 Real-Time Localization HUD & Auto-Logger]{NC}")
        print(f" 📂 Log Dir : {log_dir}")
        print(f" 📄 CSV Log : {self.csv_path}")
        print(f"{BOLD}{CYAN}========================================================================{NC}\n")

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

        cov_x = float(msg.pose.covariance[0])

        # Jump Detection Warning
        jump_warn = ""
        if hasattr(self, '_last_pos') and self._last_pos is not None:
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

        # Write to CSV
        self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{yaw_deg:.2f},{cov_x:.6f},LOCALIZED\n")
        self.csv_file.flush()

        log_line = f"[{iso_ts}] LOCALIZED #{self.pose_count:04d} X:{pos.x:+7.3f} Y:{pos.y:+7.3f} Z:{pos.z:+6.3f} Yaw:{yaw_deg:+6.1f} cov:{cov_x:.4f}\n"
        self.log_file.write(log_line)
        self.log_file.flush()

        print(
            f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
            f"{GREEN}🎯 [LOCALIZED #{self.pose_count:04d}]{NC} "
            f"{BOLD}X:{NC} {pos.x:+7.3f}m | "
            f"{BOLD}Y:{NC} {pos.y:+7.3f}m | "
            f"{BOLD}Z:{NC} {pos.z:+6.3f}m | "
            f"{BOLD}Yaw:{NC} {yaw_deg:+6.1f}° "
            f"{CYAN}(cov_x={cov_x:.4f}){NC}"
            f"{jump_warn}{boundary_warn}"
        )

    def status_timer_callback(self):
        now = time.time()
        # If no pose received for > 2.0 seconds, check TF or report searching
        if now - self.last_pose_time > 2.0:
            iso_ts = datetime.now().isoformat()
            elapsed = now - self.start_time
            wall_hhmmss = datetime.now().strftime("%H:%M:%S")
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                pos = t.transform.translation
                ori = t.transform.rotation
                siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
                cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
                yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                self.csv_file.write(f"{iso_ts},{elapsed:.3f},{self.pose_count},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{yaw_deg:.2f},0.0,TF_TRACKING\n")
                self.csv_file.flush()

                print(
                    f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] "
                    f"{CYAN}📍 [TF TRACKING]{NC} "
                    f"X: {pos.x:+7.3f}m | "
                    f"Y: {pos.y:+7.3f}m | "
                    f"Z: {pos.z:+6.3f}m | "
                    f"Yaw: {yaw_deg:+6.1f}°"
                )
            except Exception:
                print(f"⏱️ [{wall_hhmmss} | +{elapsed:04.1f}s] {YELLOW}🔍 [SEARCHING] Looking for visual/LiDAR landmarks in map... (Stand in known corridor area){NC}")

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
