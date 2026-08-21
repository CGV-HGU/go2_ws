#!/usr/bin/env python3
"""
========================================================================================
🗺️ [ESCAPE-Nav] RTAB-Map DB to 3D Point Cloud (.PLY) & Trajectory (.CSV) Exporter
========================================================================================
Extracts 3D Point Cloud and 6-DOF Trajectory from ~/.ros/rtabmap.db into:
  1. map_pointcloud.ply  -> Open with CloudCompare, MeshLab, or Windows 3D Viewer!
  2. trajectory_path.csv -> Open with Excel / Python matplotlib
========================================================================================
"""

import os
import sys
import sqlite3
import struct
import zlib
import numpy as np

DB_PATH = os.path.expanduser("~/.ros/rtabmap.db")
OUTPUT_DIR = "scratch/rtabmap_export"

def export_db_to_3d(db_path=DB_PATH, output_dir=OUTPUT_DIR):
    if not os.path.exists(db_path):
        print(f"❌ Database not found at: {db_path}")
        return False

    os.makedirs(output_dir, exist_ok=True)
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    print("=" * 76)
    print(f" 🗺️ [RTAB-Map 3D Exporter] Extracting 3D Data from: {db_path}")
    print("=" * 76)

    # 1. Extract Trajectory Poses
    cursor.execute("SELECT id, pose FROM Node ORDER BY id ASC")
    nodes = cursor.fetchall()
    
    trajectory = []
    for node_id, pose_blob in nodes:
        if pose_blob and len(pose_blob) >= 48:
            # 3x4 Transform Matrix (12 floats / doubles)
            try:
                vals = struct.unpack('12f', pose_blob[:48])
                x, y, z = vals[3], vals[7], vals[11]
                trajectory.append((node_id, x, y, z))
            except Exception:
                pass

    traj_csv_path = os.path.join(output_dir, "trajectory_path.csv")
    with open(traj_csv_path, "w") as f:
        f.write("node_id,x,y,z\n")
        for node_id, x, y, z in trajectory:
            f.write(f"{node_id},{x:.4f},{y:.4f},{z:.4f}\n")
    print(f"  ✅ Exported {len(trajectory)} Trajectory Poses -> {traj_csv_path}")

    # 2. Extract 3D Laser / RGBD Points
    cursor.execute("SELECT node_id, scan FROM LaserScan ORDER BY node_id ASC")
    scans = cursor.fetchall()

    all_points = []
    for node_id, scan_blob in scans:
        if scan_blob:
            try:
                # Decompress zlib if compressed
                try:
                    decompressed = zlib.decompress(scan_blob)
                except Exception:
                    decompressed = scan_blob

                # Parse float32 (x,y,z) points
                num_floats = len(decompressed) // 4
                raw_floats = struct.unpack(f'{num_floats}f', decompressed[:num_floats*4])
                
                # Reshape into (N, 3) or (N, 4/6)
                pts = np.array(raw_floats).reshape(-1, 3 if num_floats % 3 == 0 else (4 if num_floats % 4 == 0 else 1))
                if pts.shape[1] >= 3:
                    all_points.append(pts[:, :3])
            except Exception:
                pass

    if all_points:
        pts_concat = np.vstack(all_points)
        ply_path = os.path.join(output_dir, "map_pointcloud.ply")
        
        with open(ply_path, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(pts_concat)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for pt in pts_concat:
                f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")

        print(f"  ✅ Exported {len(pts_concat)} 3D Points -> {ply_path}")
    else:
        print("  ℹ️ No raw laser scans stored in DB (Poses only).")

    conn.close()
    print("=" * 76)
    print(" 🚀 [DONE] You can now open map_pointcloud.ply in CloudCompare or MeshLab on Windows!")
    print("=" * 76)
    return True

if __name__ == "__main__":
    export_db_to_3d()
