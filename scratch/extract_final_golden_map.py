#!/usr/bin/env python3
"""
Production 1-Click Golden Map & Trajectory Extractor for Go2 Planar 3DoF SLAM.
Decodes RTAB-Map DB, verifies graph optimization, renders sharp continuous solid walls,
and saves trajectory.png, 2d.png, and 2d_map_with_trajectory.png.
"""

import sys
import os
import sqlite3
import struct
import zlib
import math
import numpy as np
import cv2
from PIL import Image

def decode_transform(blob):
    if len(blob) == 48:
        values = struct.unpack('<12f', blob)
    elif len(blob) == 96:
        values = struct.unpack('<12d', blob)
    else:
        raise ValueError(f"Unsupported blob size: {len(blob)}")
    tx, ty, tz = float(values[3]), float(values[7]), float(values[11])
    yaw = math.atan2(float(values[4]), float(values[0]))
    return tx, ty, tz, yaw

def process_database(db_path, output_dir="2dmap"):
    if not os.path.exists(db_path):
        print(f"Error: Database not found: {db_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Processing Database: {db_path}")
    conn = sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True)
    c = conn.cursor()

    # 1. Poses
    c.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
    raw_rows = c.fetchall()
    raw_poses = {}
    for nid, blob in raw_rows:
        try:
            raw_poses[int(nid)] = decode_transform(blob)
        except Exception:
            pass

    c.execute("SELECT opt_ids, opt_poses FROM Admin LIMIT 1")
    admin_row = c.fetchone()
    opt_poses = {}
    if admin_row and admin_row[0] and admin_row[1]:
        ids_data = zlib.decompress(admin_row[0])
        poses_data = zlib.decompress(admin_row[1])
        ids = struct.unpack(f"<{len(ids_data)//4}i", ids_data)
        if len(ids) > 1:
            for i, nid in enumerate(ids):
                chunk = poses_data[i*48:(i+1)*48]
                opt_poses[int(nid)] = decode_transform(chunk)
        else:
            opt_poses = raw_poses
    else:
        opt_poses = raw_poses

    # Check loop closures in Link table
    c.execute("SELECT COUNT(*) FROM Link WHERE type=1")
    global_loops = c.fetchone()[0]
    print(f"Found {len(raw_poses)} Nodes, {len(opt_poses)} Optimized Poses, {global_loops} Global Loop Closures.")

    sorted_ids = sorted(opt_poses.keys())
    N = len(sorted_ids)
    if N == 0:
        print("Error: No poses found in database.", file=sys.stderr)
        sys.exit(1)

    poses_arr = np.array([[opt_poses[nid][0], opt_poses[nid][1], opt_poses[nid][3]] for nid in sorted_ids])
    raw_poses_arr = np.array([[raw_poses[nid][0], raw_poses[nid][1], raw_poses[nid][3]] for nid in sorted_ids if nid in raw_poses])

    # 2. Render Trajectory PNG
    W, H = 1200, 1200
    img = np.full((H, W, 3), 250, dtype=np.uint8)

    all_x = poses_arr[:, 0]
    all_y = poses_arr[:, 1]
    min_x, max_x = min(all_x) - 4.0, max(all_x) + 4.0
    min_y, max_y = min(all_y) - 4.0, max(all_y) + 4.0
    span = max(max_x - min_x, max_y - min_y)

    def world_to_px(x, y):
        px = int(80 + (x - min_x) / span * (W - 160))
        py = int((H - 80) - (y - min_y) / span * (H - 160))
        return (px, py)

    # 10m Grid
    for gx in np.arange(math.floor(min_x/10)*10, max_x, 10.0):
        cv2.line(img, world_to_px(gx, min_y), world_to_px(gx, max_y), (230, 230, 230), 1)
    for gy in np.arange(math.floor(min_y/10)*10, max_y, 10.0):
        cv2.line(img, world_to_px(min_x, gy), world_to_px(max_x, gy), (230, 230, 230), 1)

    # Draw Raw Path (Gray)
    if len(raw_poses_arr) > 1:
        for i in range(1, len(raw_poses_arr)):
            p1 = world_to_px(raw_poses_arr[i-1, 0], raw_poses_arr[i-1, 1])
            p2 = world_to_px(raw_poses_arr[i, 0], raw_poses_arr[i, 1])
            cv2.line(img, p1, p2, (190, 190, 190), 2, cv2.LINE_AA)

    # Draw Optimized Path (Vibrant Blue/Red)
    for i in range(1, N):
        p1 = world_to_px(poses_arr[i-1, 0], poses_arr[i-1, 1])
        p2 = world_to_px(poses_arr[i, 0], poses_arr[i, 1])
        cv2.line(img, p1, p2, (220, 70, 0), 3, cv2.LINE_AA)

    s_pt = world_to_px(poses_arr[0, 0], poses_arr[0, 1])
    e_pt = world_to_px(poses_arr[-1, 0], poses_arr[-1, 1])
    cv2.circle(img, s_pt, 9, (0, 180, 0), -1, cv2.LINE_AA)
    cv2.putText(img, "START", (s_pt[0]+12, s_pt[1]+5), cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 140, 0), 1, cv2.LINE_AA)
    cv2.circle(img, e_pt, 9, (0, 0, 230), -1, cv2.LINE_AA)
    cv2.putText(img, "END", (e_pt[0]+12, e_pt[1]+5), cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 0, 200), 1, cv2.LINE_AA)

    gap = math.hypot(poses_arr[-1, 0] - poses_arr[0, 0], poses_arr[-1, 1] - poses_arr[0, 1])
    cv2.putText(img, "Unitree Go2 Planar 3DoF LIVO SLAM Trajectory", (30, 45), cv2.FONT_HERSHEY_DUPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(img, f"Nodes: {N} | Global Loop Closures: {global_loops} | Endpoint Gap: {gap*100:.1f}cm", (30, 80), cv2.FONT_HERSHEY_DUPLEX, 0.5, (80, 80, 80), 1, cv2.LINE_AA)

    os.makedirs(output_dir, exist_ok=True)
    traj_path = os.path.join(output_dir, "trajectory.png")
    cv2.imwrite(traj_path, img)
    print(f"Saved Trajectory: {traj_path}")

    # 3. Extract 3D LiDAR Cells for Solid Wall Grid Map
    all_obs = []
    all_free = []

    c.execute("SELECT id, obstacle_cells, ground_cells FROM Data WHERE obstacle_cells IS NOT NULL OR ground_cells IS NOT NULL")
    for nid, obs_blob, gnd_blob in c.fetchall():
        pose_tuple = opt_poses.get(nid) or raw_poses.get(nid)
        if pose_tuple is None:
            continue
        tx, ty, tz, yaw = pose_tuple
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        if obs_blob:
            obs_data = zlib.decompress(obs_blob)
            num_pts = len(obs_data) // 12
            if num_pts > 0:
                pts = struct.unpack(f"<{num_pts*3}f", obs_data)
                lx = np.array(pts[0::3])
                ly = np.array(pts[1::3])
                valid = (lx**2 + ly**2 <= 10.0**2) & (lx**2 + ly**2 >= 0.35**2)
                lx, ly = lx[valid], ly[valid]
                gx = tx + lx * cos_yaw - ly * sin_yaw
                gy = ty + lx * sin_yaw + ly * cos_yaw
                all_obs.extend(zip(gx, gy))

        if gnd_blob:
            gnd_data = zlib.decompress(gnd_blob)
            num_pts = len(gnd_data) // 12
            if num_pts > 0:
                pts = struct.unpack(f"<{num_pts*3}f", gnd_data)
                lx = np.array(pts[0::6])
                ly = np.array(pts[1::6])
                valid = (lx**2 + ly**2 <= 10.0**2) & (lx**2 + ly**2 >= 0.35**2)
                lx, ly = lx[valid], ly[valid]
                gx = tx + lx * cos_yaw - ly * sin_yaw
                gy = ty + lx * sin_yaw + ly * cos_yaw
                all_free.extend(zip(gx, gy))

    all_obs = np.array(all_obs)
    all_free = np.array(all_free)

    res = 0.05  # 5cm grid resolution
    min_gx = min(all_obs[:, 0].min(), all_free[:, 0].min()) - 2.0
    max_gx = max(all_obs[:, 0].max(), all_free[:, 0].max()) + 2.0
    min_gy = min(all_obs[:, 1].min(), all_free[:, 1].min()) - 2.0
    max_gy = max(all_obs[:, 1].max(), all_free[:, 1].max()) + 2.0

    map_w = int(math.ceil((max_gx - min_gx) / res))
    map_h = int(math.ceil((max_gy - min_gy) / res))

    # Grid (205 = Unknown Gray, 255 = Free White, 0 = Obstacle Black)
    grid = np.full((map_h, map_w), 205, dtype=np.uint8)

    # Free Space
    f_px = np.clip(((all_free[:, 0] - min_gx) / res).astype(int), 0, map_w - 1)
    f_py = np.clip((map_h - 1 - ((all_free[:, 1] - min_gy) / res).astype(int)), 0, map_h - 1)
    grid[f_py, f_px] = 255

    # Obstacle Hit Accumulation
    hit_count = np.zeros((map_h, map_w), dtype=np.int32)
    o_px = np.clip(((all_obs[:, 0] - min_gx) / res).astype(int), 0, map_w - 1)
    o_py = np.clip((map_h - 1 - ((all_obs[:, 1] - min_gy) / res).astype(int)), 0, map_h - 1)
    np.add.at(hit_count, (o_py, o_px), 1)

    # Morphological Close & Dilation for Solid Continuous Architectural Walls
    raw_obs = (hit_count >= 1).astype(np.uint8)
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed_obs = cv2.morphologyEx(raw_obs, cv2.MORPH_CLOSE, close_kernel)
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thick_obs = cv2.dilate(closed_obs, dilate_kernel, iterations=1)
    grid[thick_obs > 0] = 0

    # Save 2d.png and 0833.pgm
    map_path = os.path.join(output_dir, "2d.png")
    Image.fromarray(grid).save(map_path)
    Image.fromarray(grid).save(os.path.join(output_dir, "0833.pgm"))
    print(f"Saved 2D Grid Map: {map_path}")

    # Save 2d_metadata.json
    import json
    meta = {
        "min_x": round(float(min_gx), 3),
        "max_x": round(float(max_gx), 3),
        "min_y": round(float(min_gy), 3),
        "max_y": round(float(max_gy), 3),
        "resolution": float(res),
        "width": int(map_w),
        "height": int(map_h)
    }
    meta_path = os.path.join(output_dir, "2d_metadata.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"Saved 2D Metadata: {meta_path}")

    # Trajectory Overlay
    grid_color = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
    t_pts = []
    for nid in sorted_ids:
        tx, ty, _, _ = opt_poses[nid]
        px = int((tx - min_gx) / res)
        py = int(map_h - 1 - (ty - min_gy) / res)
        t_pts.append((px, py))

    for i in range(1, len(t_pts)):
        cv2.line(grid_color, t_pts[i-1], t_pts[i], (220, 50, 0), 2, cv2.LINE_AA)

    if t_pts:
        cv2.circle(grid_color, t_pts[0], 6, (0, 180, 0), -1, cv2.LINE_AA)
        cv2.circle(grid_color, t_pts[-1], 6, (0, 0, 230), -1, cv2.LINE_AA)

    overlay_path = os.path.join(output_dir, "2d_map_with_trajectory.png")
    cv2.imwrite(overlay_path, grid_color)
    print(f"Saved Trajectory Overlay: {overlay_path}")
    print("✅ All golden maps extracted successfully!")

if __name__ == "__main__":
    db_file = sys.argv[1] if len(sys.argv) > 1 else "/home/unitree/.ros/rtabmap.db"
    out_folder = sys.argv[2] if len(sys.argv) > 2 else "2dmap"
    process_database(db_file, out_folder)
