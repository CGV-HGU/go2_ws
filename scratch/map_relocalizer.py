#!/usr/bin/env python3
"""
Map-Based Relocalization & Initial Pose Helper for Unitree Go2.
Provides:
  1. Waypoint coordinate resolution from config/navigation_goals.yaml (or origin).
  2. Automatic visual relocalization by matching live camera frame against recorded keyframes in rtabmap.db.
  3. ROS2 initialpose publisher helper to seed RTAB-Map with verified map coordinates.
"""

import os
import sys
import math
import struct
import sqlite3
import yaml
import numpy as np

# Ensure OpenCV libraries are available
opencv_lib = '/home/unitree/opencv_build/opencv/build/lib'
if opencv_lib not in os.environ.get('LD_LIBRARY_PATH', ''):
    os.environ['LD_LIBRARY_PATH'] = f"{opencv_lib}:/usr/local/lib:" + os.environ.get('LD_LIBRARY_PATH', '')
import cv2

GOALS_YAML = "/home/unitree/go2_ws_antarctica/config/navigation_goals.yaml"
RTABMAP_DB = "/home/unitree/.ros/rtabmap.db"

def load_registered_waypoints():
    """Returns list of dicts with waypoint info: id, name, x, y, z, yaw_deg."""
    waypoints = [
        {"id": 0, "name": "Map Origin (Node 1)", "x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0}
    ]
    if os.path.exists(GOALS_YAML):
        try:
            with open(GOALS_YAML, 'r') as f:
                data = yaml.safe_load(f)
                for g in data.get('goals', []):
                    waypoints.append({
                        "id": int(g.get('id', len(waypoints))),
                        "name": str(g.get('name', f"Goal_{g.get('id')}")),
                        "x_m": float(g.get('x_m', 0.0)),
                        "y_m": float(g.get('y_m', 0.0)),
                        "z_m": float(g.get('z_m', 0.0)),
                        "yaw_deg": float(g.get('yaw_deg', 0.0))
                    })
        except Exception:
            pass
    return waypoints

def parse_rtabmap_pose(blob):
    """Parses 48-byte float32 3x4 transform matrix from RTAB-Map SQLite blob."""
    if not blob or len(blob) != 48:
        return None
    vals = struct.unpack("<12f", blob)
    # [r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz]
    tx, ty, tz = vals[3], vals[7], vals[11]
    yaw_rad = math.atan2(vals[4], vals[0])
    yaw_deg = math.degrees(yaw_rad)
    return tx, ty, tz, yaw_deg

class MapRelocalizer:
    def __init__(self, db_path=RTABMAP_DB):
        self.db_path = os.path.expanduser(db_path)
        self.keyframes = []
        self._cached_descriptors = []
        self.orb = cv2.ORB_create(nfeatures=400)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self._loaded = False

    def load_database_keyframes(self, step=2):
        """Loads and indexes keyframes from rtabmap.db."""
        if not os.path.exists(self.db_path):
            return 0
        try:
            conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
            c = conn.cursor()
            c.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id ASC")
            poses = {row[0]: row[1] for row in c.fetchall()}

            c.execute("SELECT id, image FROM Data WHERE image IS NOT NULL ORDER BY id ASC")
            rows = c.fetchall()
            conn.close()

            self.keyframes = []
            self._cached_descriptors = []
            for idx in range(0, len(rows), step):
                nid, img_bytes = rows[idx]
                if nid not in poses:
                    continue
                pose_tuple = parse_rtabmap_pose(poses[nid])
                if not pose_tuple:
                    continue
                img = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue
                kp, des = self.orb.detectAndCompute(img, None)
                if des is not None and len(des) >= 15:
                    self.keyframes.append((nid, pose_tuple))
                    self._cached_descriptors.append(des)

            self._loaded = True
            return len(self.keyframes)
        except Exception:
            return 0

    def match_live_frame(self, cv_img):
        """Matches a live camera frame against cached DB keyframes."""
        if not self._loaded:
            self.load_database_keyframes()
        if not self._cached_descriptors:
            return None

        if len(cv_img.shape) == 3:
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_img

        kp_q, des_q = self.orb.detectAndCompute(gray, None)
        if des_q is None or len(des_q) < 15:
            return None

        best_idx = -1
        max_good = 0
        for i, des_db in enumerate(self._cached_descriptors):
            try:
                matches = self.bf.match(des_q, des_db)
                good = sum(1 for m in matches if m.distance < 45)
                if good > max_good:
                    max_good = good
                    best_idx = i
            except Exception:
                continue

        if best_idx >= 0 and max_good >= 20:
            nid, (x, y, z, yaw) = self.keyframes[best_idx]
            return {
                "node_id": nid,
                "x_m": float(x),
                "y_m": float(y),
                "z_m": float(z),
                "yaw_deg": float(yaw),
                "match_count": int(max_good)
            }
        return None

def publish_initial_pose(publisher, x: float, y: float, z: float = 0.0, yaw_deg: float = 0.0, clock_now=None):
    """Publishes PoseWithCovarianceStamped to RTAB-Map initialpose topic."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    msg = PoseWithCovarianceStamped()
    if clock_now is not None:
        msg.header.stamp = clock_now.to_msg()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    yaw_rad = math.radians(yaw_deg)
    msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    cov = [0.0] * 36
    cov[0] = 0.01
    cov[7] = 0.01
    cov[14] = 0.01
    cov[35] = 0.05
    msg.pose.covariance = cov
    publisher.publish(msg)
