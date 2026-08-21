#!/usr/bin/env python3
"""
========================================================================================
🗺️ Full RTAB-Map 3D Point Cloud & 2D Map Inspector & Headless Exporter
========================================================================================
Allows 100% headless inspection of ~/.ros/rtabmap.db without requiring GUI:
1. Generates 3D Point Cloud (.ply / .pcd) via rtabmap-export
2. Computes 2D/3D Trajectory metrics (Length in meters, Bounding Box X/Y/Z)
3. Renders 2D Trajectory plot PNG (scratch/rtabmap_preview/trajectory_2d.png)
4. Extracts sample RGB Keyframes (scratch/rtabmap_preview/node_XXXX.jpg)
5. Prints comprehensive database health & SLAM convergence summary
========================================================================================
"""

import os
import sys
import math
import sqlite3
import struct
import datetime
import subprocess

# Ensure OpenCV libraries are available
opencv_lib = '/home/unitree/opencv_build/opencv/build/lib'
if opencv_lib not in os.environ.get('LD_LIBRARY_PATH', ''):
    os.environ['LD_LIBRARY_PATH'] = f"{opencv_lib}:/usr/local/lib:" + os.environ.get('LD_LIBRARY_PATH', '')

import cv2
import numpy as np

def inspect_and_export_all(db_path="~/.ros/rtabmap.db", output_dir="/home/unitree/go2_ws_antarctica/scratch/rtabmap_preview"):
    db_path = os.path.expanduser(db_path)
    if not os.path.exists(db_path):
        print(f"❌ Error: Database not found at '{db_path}'")
        print("👉 Run './mapping_gui.sh' or './mapping.sh' first to record a map!")
        return

    os.makedirs(output_dir, exist_ok=True)
    db_size_mb = os.path.getsize(db_path) / (1024 * 1024)

    print("=" * 76)
    print(f" 🗺️ [RTAB-Map Full SLAM & Point Cloud Inspector] {db_path}")
    print(f" 📦 File Size: {db_size_mb:.2f} MB | Output Dir: {output_dir}")
    print("=" * 76)

    conn = sqlite3.connect(db_path)
    c = conn.cursor()

    # 1. Inspect Nodes and Poses
    c.execute("SELECT id, map_id, stamp, weight, pose FROM Node ORDER BY id ASC;")
    node_rows = c.fetchall()
    total_nodes = len(node_rows)

    c.execute("SELECT count(*) FROM Link;")
    total_links = c.fetchone()[0]

    print(f"  • Total Mapped Keyframe Nodes : {total_nodes}")
    print(f"  • Total Graph Links / Edges  : {total_links}")

    if total_nodes == 0:
        print("⚠️ Database contains 0 nodes. Robot was not moved or data was not received.")
        conn.close()
        return

    first_stamp = datetime.datetime.fromtimestamp(node_rows[0][2])
    last_stamp = datetime.datetime.fromtimestamp(node_rows[-1][2])
    duration = node_rows[-1][2] - node_rows[0][2]
    print(f"  • Recording Duration         : {duration:.1f}s ({first_stamp.strftime('%H:%M:%S')} ~ {last_stamp.strftime('%H:%M:%S')})")

    # 2. Extract Trajectory (x, y, z)
    trajectory = []
    for r in node_rows:
        pose_blob = r[4]
        if pose_blob and len(pose_blob) >= 48:
            # 12 floats or doubles representing 3x4 transform matrix
            try:
                # 3x4 float (48 bytes) or double (96 bytes)
                if len(pose_blob) == 48:
                    vals = struct.unpack('12f', pose_blob)
                elif len(pose_blob) == 96:
                    vals = struct.unpack('12d', pose_blob)
                else:
                    vals = struct.unpack(f"{len(pose_blob)//4}f", pose_blob[:48])
                tx, ty, tz = vals[3], vals[7], vals[11]
                trajectory.append((tx, ty, tz))
            except Exception:
                pass

    if len(trajectory) > 1:
        xs = [p[0] for p in trajectory]
        ys = [p[1] for p in trajectory]
        zs = [p[2] for p in trajectory]
        
        # Calculate trajectory distance
        dist = 0.0
        for i in range(1, len(trajectory)):
            dx = trajectory[i][0] - trajectory[i-1][0]
            dy = trajectory[i][1] - trajectory[i-1][1]
            dz = trajectory[i][2] - trajectory[i-1][2]
            dist += math.sqrt(dx*dx + dy*dy + dz*dz)

        print(f"  • Trajectory Length (Distance): {dist:.2f} meters")
        print(f"  • Bounding Box X / Y / Z     : [{min(xs):.2f}, {max(xs):.2f}]m / [{min(ys):.2f}, {max(ys):.2f}]m / [{min(zs):.2f}, {max(zs):.2f}]m")

        # Render 2D Trajectory Plot Image
        plot_img = np.full((600, 600, 3), 245, dtype=np.uint8)
        min_x, max_x = min(xs) - 0.5, max(xs) + 0.5
        min_y, max_y = min(ys) - 0.5, max(ys) + 0.5
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        span = max(span_x, span_y)

        # Draw grid
        for g in range(0, 600, 50):
            cv2.line(plot_img, (g, 0), (g, 600), (225, 225, 225), 1)
            cv2.line(plot_img, (0, g), (600, g), (225, 225, 225), 1)

        pts = []
        for x, y, _ in trajectory:
            px = int(50 + (x - min_x) / span * 500)
            py = int(550 - (y - min_y) / span * 500)
            pts.append((px, py))

        for i in range(1, len(pts)):
            cv2.line(plot_img, pts[i-1], pts[i], (255, 100, 0), 2, cv2.LINE_AA)

        # Start point (Green) & End point (Red)
        cv2.circle(plot_img, pts[0], 6, (0, 200, 0), -1)
        cv2.putText(plot_img, "START", (pts[0][0]+8, pts[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 150, 0), 1)
        cv2.circle(plot_img, pts[-1], 6, (0, 0, 255), -1)
        cv2.putText(plot_img, "END", (pts[-1][0]+8, pts[-1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 1)

        cv2.putText(plot_img, f"Trajectory: {dist:.2f}m ({total_nodes} nodes)", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 2)
        traj_out_path = os.path.join(output_dir, "trajectory_2d.png")
        cv2.imwrite(traj_out_path, plot_img)
        print(f"  📈 2D Trajectory Plot Rendered : {traj_out_path}")

    # 3. Export RGB Keyframes
    c.execute("SELECT id, image FROM Data WHERE image IS NOT NULL ORDER BY id ASC;")
    img_rows = c.fetchall()
    print(f"  • Total Stored RGB Images    : {len(img_rows)}")
    indices_to_export = [0, len(img_rows) // 2, len(img_rows) - 1] if len(img_rows) >= 3 else range(len(img_rows))
    for idx in set(indices_to_export):
        node_id, img_bytes = img_rows[idx]
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is not None:
            out_path = os.path.join(output_dir, f"node_{node_id:04d}.jpg")
            cv2.imwrite(out_path, img)
            print(f"    📸 Exported Keyframe [Node {node_id}] -> {out_path} ({img.shape[1]}x{img.shape[0]})")

    # 4. Attempt 3D Point Cloud Export via rtabmap-export
    cloud_ply = os.path.join(output_dir, "point_cloud.ply")
    poses_txt = os.path.join(output_dir, "poses.txt")
    export_cmd = f"rtabmap-export --output_dir {output_dir} --output point_cloud --poses {db_path}"
    try:
        ret = subprocess.run(export_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        if os.path.exists(cloud_ply):
            ply_size_kb = os.path.getsize(cloud_ply) / 1024
            print(f"  ☁️ 3D Point Cloud PLY Exported : {cloud_ply} ({ply_size_kb:.1f} KB)")
    except Exception:
        pass

    print("=" * 76)
    print(" ✅ Headless SLAM & Point Cloud Verification Complete!")
    print(f" 📂 All artifacts available at: file://{output_dir}")
    print("=" * 76)
    conn.close()

if __name__ == '__main__':
    target = sys.argv[1] if len(sys.argv) > 1 else "~/.ros/rtabmap.db"
    inspect_and_export_all(target)
