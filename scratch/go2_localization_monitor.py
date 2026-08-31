#!/usr/bin/env python3
"""
Real-Time Planar 3DoF Localization HUD & Pose Logger for Unitree Go2.
Subscribes to /rtabmap/localization_pose, /map -> /base_link TF, and /rtabmap/info,
printing clean, formatted (X, Y, Z, Yaw) and localization status logs to console.
"""

import math
import time
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
    def __init__(self):
        super().__init__('go2_localization_monitor')

        self.last_pose_time = time.time()
        self.pose_count = 0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.last_yaw = 0.0
        self.is_localized = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Subscribe to RTAB-Map Localization Pose
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
        print(f"{BOLD}{CYAN} 🎯 [Go2 Real-Time Localization HUD] Monitoring Map Frame Coordinates...{NC}")
        print(f"{BOLD}{CYAN}========================================================================{NC}\n")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Calculate Yaw from Quaternion
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        self.last_x = pos.x
        self.last_y = pos.y
        self.last_z = pos.z
        self.last_yaw = yaw_deg
        self.last_pose_time = time.time()
        self.pose_count += 1
        self.is_localized = True

        print(
            f"{GREEN}🎯 [LOCALIZED #{self.pose_count:04d}]{NC} "
            f"{BOLD}X:{NC} {pos.x:+7.3f}m | "
            f"{BOLD}Y:{NC} {pos.y:+7.3f}m | "
            f"{BOLD}Z:{NC} {pos.z:+6.3f}m | "
            f"{BOLD}Yaw:{NC} {yaw_deg:+6.1f}° "
            f"{CYAN}(cov_x={msg.pose.covariance[0]:.4f}){NC}"
        )

    def status_timer_callback(self):
        now = time.time()
        # If no pose received for > 2.0 seconds, check TF or report searching
        if now - self.last_pose_time > 2.0:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                pos = t.transform.translation
                ori = t.transform.rotation
                siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
                cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
                yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                print(
                    f"{CYAN}📍 [TF TRACKING]{NC} "
                    f"X: {pos.x:+7.3f}m | "
                    f"Y: {pos.y:+7.3f}m | "
                    f"Z: {pos.z:+6.3f}m | "
                    f"Yaw: {yaw_deg:+6.1f}°"
                )
            except Exception:
                print(f"{YELLOW}🔍 [SEARCHING] Looking for visual/LiDAR landmarks in map... (Stand in known corridor area){NC}")

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
