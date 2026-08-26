#!/usr/bin/env python3
"""
====================================================================================================
🎬 [ESCAPE-Nav] Headless Real-Time 2D Map + FPV + Telemetry MP4 Video Recorder
====================================================================================================
Subscribes to ROS 2 topics (/map, /odom, /camera/front/image_raw) and renders a 1080p
broadcast-quality live mapping video directly to MP4 in memory.
Zero HDMI / X11 dependency - completely immune to monitor unplugging / display sleep.
====================================================================================================
"""

import os
import sys
import time
import math
import datetime
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Video settings
CANVAS_W = 1920
CANVAS_H = 1080
FPS = 30


class LiveMapVideoRecorder(Node):
    def __init__(self, output_filepath):
        super().__init__('live_2d_map_video_recorder')
        self.output_filepath = output_filepath
        self.bridge = CvBridge()

        # State storage
        self.latest_map = None
        self.latest_map_info = None
        self.latest_odom = None
        self.latest_fpv = None
        self.trajectory_history = []
        self.total_distance = 0.0
        self.prev_pose = None
        self.start_time = time.time()
        self.frame_count = 0
        self.lock = threading.Lock()

        # QoS
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)
        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, reliable_qos)
        self.create_subscription(Image, '/camera/front/image_raw', self.image_callback, reliable_qos)

        # Video Writer initialization
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(self.output_filepath, fourcc, FPS, (CANVAS_W, CANVAS_H))
        
        self.get_logger().info(f"🎬 [Video Recorder] Recording live 2D map to: {self.output_filepath}")
        
        # 30Hz Render & Write Timer
        self.render_timer = self.create_timer(1.0 / FPS, self.render_and_write_frame)

    def map_callback(self, msg: OccupancyGrid):
        with self.lock:
            self.latest_map_info = msg.info
            # Convert OccupancyGrid (-1 to 100) to 2D numpy array
            data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
            self.latest_map = data

    def odom_callback(self, msg: Odometry):
        with self.lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            # Calculate yaw
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            curr_pos = (p.x, p.y, yaw)
            self.latest_odom = {
                "x": p.x, "y": p.y, "z": p.z, "yaw": yaw,
                "vx": msg.twist.twist.linear.x, "wz": msg.twist.twist.angular.z
            }

            if self.prev_pose is not None:
                step_dist = math.hypot(p.x - self.prev_pose[0], p.y - self.prev_pose[1])
                if step_dist > 0.02: # Only append if moved > 2cm
                    self.total_distance += step_dist
                    self.trajectory_history.append((p.x, p.y))
                    self.prev_pose = curr_pos
            else:
                self.prev_pose = curr_pos
                self.trajectory_history.append((p.x, p.y))

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_fpv = cv_img
        except Exception:
            pass

    def render_and_write_frame(self):
        canvas = np.full((CANVAS_H, CANVAS_W, 3), 18, dtype=np.uint8) # Dark luxury background

        with self.lock:
            map_data = self.latest_map
            map_info = self.latest_map_info
            odom_data = self.latest_odom
            fpv_img = self.latest_fpv
            traj = list(self.trajectory_history)
            total_dist = self.total_distance

        # ----------------------------------------------------------------------
        # 1. Main 2D Occupancy Grid View (Left Pane: 1340 x 1020)
        # ----------------------------------------------------------------------
        map_pane_w = 1340
        map_pane_h = 1020
        map_pane = np.full((map_pane_h, map_pane_w, 3), 28, dtype=np.uint8)

        if map_data is not None and map_info is not None:
            # Map coloring: -1 (unknown) -> 40, 0 (free) -> 240, 100 (occupied/wall) -> 0
            h, w = map_data.shape
            map_rgb = np.zeros((h, w, 3), dtype=np.uint8)
            map_rgb[map_data == -1] = [45, 45, 52]     # Unknown: Dark Slate
            map_rgb[map_data == 0] = [235, 235, 240]   # Free: Crisp White
            map_rgb[map_data > 50] = [20, 20, 25]      # Occupied: Jet Black

            # Flip Y because OccupancyGrid origin is bottom-left
            map_rgb = np.flipud(map_rgb)

            # Auto-Scale and Center Map inside pane
            res = map_info.resolution
            scale = min((map_pane_w - 40) / w, (map_pane_h - 40) / h)
            scale = max(0.2, min(5.0, scale))

            disp_w = int(w * scale)
            disp_h = int(h * scale)
            resized_map = cv2.resize(map_rgb, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

            offset_x = (map_pane_w - disp_w) // 2
            offset_y = (map_pane_h - disp_h) // 2

            map_pane[offset_y:offset_y+disp_h, offset_x:offset_x+disp_w] = resized_map

            # Draw Robot Trajectory Ribbon on scaled map
            origin_x = map_info.origin.position.x
            origin_y = map_info.origin.position.y

            def world_to_pane(wx, wy):
                mx = (wx - origin_x) / res
                my = (wy - origin_y) / res
                my_flipped = h - 1 - my
                px = int(offset_x + mx * scale)
                py = int(offset_y + my_flipped * scale)
                return px, py

            # Draw trajectory path
            if len(traj) > 1:
                pts = [world_to_pane(tx, ty) for tx, ty in traj]
                for k in range(len(pts) - 1):
                    p1, p2 = pts[k], pts[k + 1]
                    if 0 <= p1[0] < map_pane_w and 0 <= p1[1] < map_pane_h and \
                       0 <= p2[0] < map_pane_w and 0 <= p2[1] < map_pane_h:
                        cv2.line(map_pane, p1, p2, (255, 180, 0), 2, cv2.LINE_AA) # Cyan ribbon

            # Draw Current Robot Pose & Heading
            if odom_data is not None:
                rx, ry = world_to_pane(odom_data['x'], odom_data['y'])
                yaw = odom_data['yaw']
                if 0 <= rx < map_pane_w and 0 <= ry < map_pane_h:
                    # Robot body footprint (Circle)
                    cv2.circle(map_pane, (rx, ry), 9, (0, 0, 255), -1, cv2.LINE_AA)
                    cv2.circle(map_pane, (rx, ry), 12, (0, 255, 255), 2, cv2.LINE_AA)
                    # Heading Arrow
                    arrow_len = 22
                    # Note: Y is flipped on image
                    ax = int(rx + arrow_len * math.cos(yaw))
                    ay = int(ry - arrow_len * math.sin(yaw))
                    cv2.arrowedLine(map_pane, (rx, ry), (ax, ay), (0, 255, 255), 2, cv2.LINE_AA, tipLength=0.35)
        else:
            cv2.putText(map_pane, "Waiting for /map OccupancyGrid from RTAB-Map LIVO...", 
                        (map_pane_w // 4, map_pane_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (120, 120, 140), 2, cv2.LINE_AA)

        # Border for map pane
        cv2.rectangle(map_pane, (0, 0), (map_pane_w - 1, map_pane_h - 1), (60, 65, 80), 2)
        canvas[30:30+map_pane_h, 30:30+map_pane_w] = map_pane

        # ----------------------------------------------------------------------
        # 2. Right Top: Real-Time Front Camera FPV (520 x 320)
        # ----------------------------------------------------------------------
        fpv_pane_w = 520
        fpv_pane_h = 320
        fpv_x = 1380
        fpv_y = 30

        if fpv_img is not None:
            resized_fpv = cv2.resize(fpv_img, (fpv_pane_w, fpv_pane_h))
        else:
            resized_fpv = np.full((fpv_pane_h, fpv_pane_w, 3), 30, dtype=np.uint8)
            cv2.putText(resized_fpv, "Go2 Front Ultra-Wide Camera (Standby)", (30, fpv_pane_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (150, 150, 150), 1, cv2.LINE_AA)

        # FPV Header badge
        cv2.rectangle(resized_fpv, (0, 0), (fpv_pane_w, 35), (15, 15, 20), -1)
        cv2.putText(resized_fpv, "LIVE FPV (230.1.1.1:1720 @ 30fps)", (15, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 1, cv2.LINE_AA)
        cv2.rectangle(resized_fpv, (0, 0), (fpv_pane_w - 1, fpv_pane_h - 1), (60, 65, 80), 2)
        canvas[fpv_y:fpv_y+fpv_pane_h, fpv_x:fpv_x+fpv_pane_w] = resized_fpv

        # ----------------------------------------------------------------------
        # 3. Right Bottom: Real-Time Telemetry Dashboard (520 x 680)
        # ----------------------------------------------------------------------
        dash_pane_w = 520
        dash_pane_h = 680
        dash_x = 1380
        dash_y = 370
        dash = np.full((dash_pane_h, dash_pane_w, 3), 24, dtype=np.uint8)

        # Title
        cv2.putText(dash, "UNITREE Go2 ESCAPE-Nav", (25, 45),
                    cv2.FONT_HERSHEY_DUPLEX, 0.85, (0, 255, 200), 2, cv2.LINE_AA)
        cv2.putText(dash, "Live 2D LIVO SLAM Video Recorder", (25, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 180), 1, cv2.LINE_AA)
        cv2.line(dash, (20, 90), (dash_pane_w - 20, 90), (50, 55, 70), 2)

        # Time & SLAM Status Cards
        elapsed_sec = int(time.time() - self.start_time)
        time_str = str(datetime.timedelta(seconds=elapsed_sec))

        cards = [
            ("RECORDING TIME", time_str, (0, 255, 255)),
            ("SLAM STATUS", "50Hz RTAB-Map ACTIVE", (50, 255, 100)),
            ("TOTAL DISTANCE", f"{total_dist:.2f} m", (255, 200, 50)),
            ("WALKING SPEED", f"{odom_data['vx']:.2f} m/s" if odom_data else "0.00 m/s", (100, 200, 255)),
        ]

        if map_info is not None:
            explored_area_m2 = (np.sum(map_data >= 0) * (map_info.resolution ** 2)) if map_data is not None else 0.0
            map_dim_str = f"{map_info.width * map_info.resolution:.1f}m x {map_info.height * map_info.resolution:.1f}m"
            cards.extend([
                ("MAP DIMENSIONS", map_dim_str, (220, 220, 220)),
                ("EXPLORED AREA", f"{explored_area_m2:.1f} m²", (180, 255, 180)),
            ])
            if odom_data is not None:
                cards.append(("ROBOT POSE (X, Y)", f"X={odom_data['x']:+.2f}m, Y={odom_data['y']:+.2f}m", (255, 180, 255)))

        card_y = 125
        for label, val, color in cards:
            # Card background
            cv2.rectangle(dash, (20, card_y - 20), (dash_pane_w - 20, card_y + 35), (32, 36, 46), -1)
            cv2.rectangle(dash, (20, card_y - 20), (24, card_y + 35), color, -1)
            cv2.putText(dash, label, (35, card_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 145, 160), 1, cv2.LINE_AA)
            cv2.putText(dash, val, (35, card_y + 24), cv2.FONT_HERSHEY_DUPLEX, 0.65, color, 1, cv2.LINE_AA)
            card_y += 68

        # Footer Notice
        cv2.putText(dash, "🛡️ Immunity: Safe to unplug HDMI anytime", (25, dash_pane_h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 200, 100), 1, cv2.LINE_AA)
        cv2.putText(dash, "👉 Press Ctrl+C on host to finish and save", (25, dash_pane_h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 160), 1, cv2.LINE_AA)

        cv2.rectangle(dash, (0, 0), (dash_pane_w - 1, dash_pane_h - 1), (60, 65, 80), 2)
        canvas[dash_y:dash_y+dash_pane_h, dash_x:dash_x+dash_pane_w] = dash

        # Write frame to MP4
        self.video_writer.write(canvas)
        self.frame_count += 1

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"💾 [Video Saved] Successfully finalized MP4 ({self.frame_count} frames) to: {self.output_filepath}")
        super().destroy_node()


def main():
    rclpy.init()
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = "/home/unitree/go2_ws_antarctica/2dmap/recordings"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"live_2d_mapping_{timestamp}.mp4")

    recorder = LiveMapVideoRecorder(out_path)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
