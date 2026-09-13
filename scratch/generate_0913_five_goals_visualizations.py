#!/usr/bin/env python3
"""
Generate publication-ready visualizations of the 0913 SLAM 2D map and 5 recorded goal poses.
Outputs:
  1. 2dmap/0913/2d_goals_map.png (Standard grid map overlay)
  2. 2dmap/0913/2d_clean_goals_map.png (Paper Fig. 6 style with pure white walkable space, no sand speckles)
  3. 2dmap/0913/2d_wall_only_goals_map.png (CAD architectural wall-only style)
  4. 2dmap/0913/fig_five_goals.pdf (Vector publication figure for ICRA 2026)
  5. config/navigation_goals.json & config/navigation_goals.yaml (Updated 5 goals)
"""

import os
import json
import math
import yaml
from pathlib import Path
import numpy as np
import cv2
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow, FancyBboxPatch

REPO_ROOT = Path("/home/unitree/go2_ws_antarctica")
GOAL_PLAN_PATH = Path("/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-20260913/goal-plan.json")
MAP_DIR = REPO_ROOT / "2dmap/0913"
META_PATH = MAP_DIR / "2d_metadata.json"
PGM_PATH = MAP_DIR / "0833.pgm"

def load_data():
    meta = json.loads(META_PATH.read_text())
    raw = np.array(Image.open(PGM_PATH))
    goal_plan = json.loads(GOAL_PLAN_PATH.read_text())
    return meta, raw, goal_plan

def clean_map(raw):
    h, w = raw.shape
    walls = (raw == 0).astype(np.uint8)
    free = (raw == 255).astype(np.uint8)

    # 1. Morphological closing to fill small internal holes in walkable space
    kernel_fill = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    free_closed = cv2.morphologyEx(free, cv2.MORPH_CLOSE, kernel_fill)
    kernel_rect = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    free_solid = cv2.morphologyEx(free_closed, cv2.MORPH_CLOSE, kernel_rect)

    # Filter connected components
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_solid)
    clean_free = np.zeros_like(free_solid)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= 200:
            clean_free[labels == i] = 1

    # Fill small internal holes (< 400 px) inside clean_free, preserving the large center island
    contours, hierarchy = cv2.findContours(clean_free, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is not None:
        for i in range(len(contours)):
            # If it is an internal hole (has a parent)
            if hierarchy[0][i][3] != -1:
                area = cv2.contourArea(contours[i])
                if area < 500: # fill tiny islands/holes, keep large courtyards
                    cv2.drawContours(clean_free, [contours[i]], -1, 1, -1)

    # Thicken architectural walls slightly for crisp contrast
    kernel_wall = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thick_walls = cv2.dilate(walls, kernel_wall, iterations=1)
    
    # Remove floating single-pixel wall noise (< 4 px)
    num_w_labels, w_labels, w_stats, _ = cv2.connectedComponentsWithStats(thick_walls)
    filtered_walls = np.zeros_like(thick_walls)
    for i in range(1, num_w_labels):
        if w_stats[i, cv2.CC_STAT_AREA] >= 4:
            filtered_walls[w_labels == i] = 1

    clean_free[filtered_walls == 1] = 0

    # 1. Clean publication map (White walkable corridor, dark charcoal walls, soft gray background)
    clean_rgb = np.full((h, w, 3), (234, 236, 239), dtype=np.uint8) # #EAECEF
    clean_rgb[clean_free == 1] = (255, 255, 255) # Pure white walkable space
    clean_rgb[filtered_walls == 1] = (30, 32, 36) # Dark charcoal walls

    # 2. Wall-only CAD map (Crisp black walls, pure white background)
    wall_only_rgb = np.full((h, w, 3), (255, 255, 255), dtype=np.uint8)
    wall_only_rgb[filtered_walls == 1] = (20, 20, 20)

    return clean_rgb, wall_only_rgb, filtered_walls

def build_visualizations():
    meta, raw, goal_plan = load_data()
    h, w = raw.shape
    res = meta["resolution"]
    min_x = meta["min_x"]
    max_y = meta["max_y"]

    def to_px(x, y):
        return (x - min_x) / res, (max_y - y) / res

    clean_rgb, wall_only_rgb, walls = clean_map(raw)

    goals = []
    for g in goal_plan["goals"]:
        lbl = g["label"]
        x = g["point_goal_xy_m"]["x"]
        y = g["point_goal_xy_m"]["y"]
        yaw = g["yaw_rad"]
        dist = math.hypot(x, y)
        px, py = to_px(x, y)
        goals.append({
            "id": int(lbl),
            "label": f"Goal {lbl}",
            "x": x,
            "y": y,
            "yaw": yaw,
            "yaw_deg": math.degrees(yaw),
            "dist": dist,
            "px": px,
            "py": py
        })

    # Start Origin
    start_x, start_y = 0.0, 0.0
    start_px, start_py = to_px(start_x, start_y)

    # Goal visual styling
    goal_styles = [
        {"color": "#D83B01", "name": "Goal 1", "text_offset": (15, -15)}, # Orange-Red
        {"color": "#744DA9", "name": "Goal 2", "text_offset": (15, 18)},  # Purple
        {"color": "#0078D4", "name": "Goal 3", "text_offset": (15, -15)}, # Electric Blue
        {"color": "#E67E22", "name": "Goal 4", "text_offset": (15, 18)},  # Amber
        {"color": "#B4009E", "name": "Goal 5", "text_offset": (15, -15)}, # Magenta
    ]

    rad_px = 1.0 / res # 20 pixels for 1.0m tolerance
    arrow_len_px = 18.0 # Arrow length

    # -------------------------------------------------------------
    # 1. Standard 2D Goals Map (OpenCV on raw grid map)
    # -------------------------------------------------------------
    std_overlay = cv2.imread(str(MAP_DIR / "2d.png"))
    if std_overlay is None:
        std_overlay = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)

    # Draw Start
    cv2.circle(std_overlay, (int(start_px), int(start_py)), 10, (16, 124, 65), -1, cv2.LINE_AA)
    cv2.circle(std_overlay, (int(start_px), int(start_py)), 12, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(std_overlay, "Start (0,0)", (int(start_px) - 35, int(start_py) + 25),
                cv2.FONT_HERSHEY_DUPLEX, 0.5, (16, 124, 65), 1, cv2.LINE_AA)

    bgr_colors = [
        (1, 59, 216),   # G1 Red
        (169, 77, 116), # G2 Purple
        (212, 120, 0),  # G3 Blue
        (34, 126, 230), # G4 Amber
        (158, 0, 180),  # G5 Magenta
    ]

    for idx, g in enumerate(goals):
        gpx, gpy = int(g["px"]), int(g["py"])
        col = bgr_colors[idx]
        # 1m radius circle
        cv2.circle(std_overlay, (gpx, gpy), int(rad_px), col, 1, cv2.LINE_AA)
        # Goal dot
        cv2.circle(std_overlay, (gpx, gpy), 9, col, -1, cv2.LINE_AA)
        cv2.circle(std_overlay, (gpx, gpy), 11, (255, 255, 255), 2, cv2.LINE_AA)
        # Yaw arrow
        ax_end = int(gpx + arrow_len_px * math.cos(g["yaw"]))
        ay_end = int(gpy - arrow_len_px * math.sin(g["yaw"]))
        cv2.arrowedLine(std_overlay, (gpx, gpy), (ax_end, ay_end), (255, 255, 255), 3, cv2.LINE_AA, tipLength=0.35)
        cv2.arrowedLine(std_overlay, (gpx, gpy), (ax_end, ay_end), col, 2, cv2.LINE_AA, tipLength=0.35)
        # Label
        tag = f"#{g['id']} ({g['dist']:.1f}m)"
        cv2.putText(std_overlay, tag, (gpx + 12, gpy - 8), cv2.FONT_HERSHEY_DUPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(std_overlay, tag, (gpx + 12, gpy - 8), cv2.FONT_HERSHEY_DUPLEX, 0.45, col, 1, cv2.LINE_AA)

    # Title header banner
    cv2.rectangle(std_overlay, (12, 10), (478, 42), (255, 255, 255), -1)
    cv2.rectangle(std_overlay, (12, 10), (478, 42), (180, 180, 180), 1)
    cv2.putText(std_overlay, f"0913 Full SLAM Map & 5 Navigation Goals (1.0m tolerance)", 
                (20, 31), cv2.FONT_HERSHEY_DUPLEX, 0.45, (30, 30, 30), 1, cv2.LINE_AA)

    cv2.imwrite(str(MAP_DIR / "2d_goals_map.png"), std_overlay)
    print(f"Saved: {MAP_DIR / '2d_goals_map.png'}")

    # -------------------------------------------------------------
    # Helper for Matplotlib Publication Rendering (Clean & Wall-only)
    # -------------------------------------------------------------
    def render_publication_plot(base_img, title, out_png, out_pdf=None):
        fig, ax = plt.subplots(figsize=(10, 9), dpi=300)
        ax.imshow(base_img)

        # Plot Start
        ax.scatter([start_px], [start_py], s=190, color="#107C41", marker="o", edgecolors="white", linewidths=2.0, zorder=8, label="Start (Fixed Origin (0,0))")
        ax.text(start_px - 8, start_py + 22, "START (0,0)", fontsize=9.5, fontweight="bold", color="#107C41", zorder=9,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#107C41", lw=1.2, alpha=0.9))

        # Plot Goals
        for idx, g in enumerate(goals):
            st = goal_styles[idx]
            col = st["color"]
            gpx, gpy = g["px"], g["py"]

            # Tolerance circle (1.0m radius)
            circ = Circle((gpx, gpy), rad_px, fill=False, linestyle="--", edgecolor=col, linewidth=1.6, alpha=0.85, zorder=6)
            ax.add_patch(circ)

            # Goal Marker Star
            ax.scatter([gpx], [gpy], s=250, color=col, marker="*", edgecolors="black", linewidths=1.2, zorder=8,
                       label=f"{g['label']}: ({g['x']:+.2f}m, {g['y']:+.2f}m) | {g['dist']:.1f}m, {g['yaw_deg']:+.0f}°")

            # Directional Yaw Arrow
            dx = arrow_len_px * math.cos(g["yaw"])
            dy = -arrow_len_px * math.sin(g["yaw"]) # Invert for image y-axis
            ax.annotate("", xy=(gpx + dx, gpy + dy), xytext=(gpx, gpy),
                        arrowprops=dict(arrowstyle="-|>", color=col, lw=2.2, mutation_scale=14), zorder=9)

            # Annotation badge
            ox, oy = st["text_offset"]
            badge_text = f"{g['label']}\n{g['dist']:.1f}m, {g['yaw_deg']:+.0f}°"
            ax.text(gpx + ox, gpy + oy, badge_text, fontsize=8.5, fontweight="bold", color=col, zorder=10,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=col, lw=1.2, alpha=0.92))

        # Focus zoom on the corridor bounds with clean margin
        margin_px = 35
        all_px = [start_px] + [g["px"] for g in goals]
        all_py = [start_py] + [g["py"] for g in goals]
        min_x_lim = max(0, min(all_px) - margin_px - 20)
        max_x_lim = min(w, max(all_px) + margin_px + 80)
        min_y_lim = min(h, max(all_py) + margin_px + 30)
        max_y_lim = max(0, min(all_py) - margin_px - 30)

        ax.set_xlim(min_x_lim, max_x_lim)
        ax.set_ylim(min_y_lim, max_y_lim) # Matplotlib image coordinates (y inverted)
        ax.axis("off")

        # 5m Scale Bar
        scale_len_m = 5.0
        scale_len_px = scale_len_m / res # 100 pixels
        sb_x = min_x_lim + 20
        sb_y = min_y_lim - 20
        ax.plot([sb_x, sb_x + scale_len_px], [sb_y, sb_y], color="#1E2024", lw=4.5, zorder=11)
        ax.text(sb_x + scale_len_px / 2, sb_y - 8, "5 m", ha="center", va="bottom", fontsize=11, fontweight="bold", color="#1E2024", zorder=11)

        # Legend
        ax.legend(loc="upper right", frameon=True, framealpha=0.95, facecolor="white", edgecolor="#D0D0D0", fontsize=9.2, borderpad=0.8)
        plt.title(title, fontsize=13, fontweight="bold", pad=12)
        plt.tight_layout()

        plt.savefig(out_png, dpi=300, bbox_inches="tight")
        if out_pdf:
            plt.savefig(out_pdf, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out_png}")
        if out_pdf:
            print(f"Saved: {out_pdf}")

    # 2. Paper Fig 6 Clean Map
    render_publication_plot(
        clean_rgb,
        "Unitree Go2 Top-View Floor Plan & 5 Recorded Navigation Goals (0913 Mapping)",
        MAP_DIR / "2d_clean_goals_map.png",
        MAP_DIR / "fig_five_goals.pdf"
    )

    # 3. Wall-Only CAD Style Map
    render_publication_plot(
        wall_only_rgb,
        "Unitree Go2 Architectural Floor Plan & Goal Poses (Wall-Only CAD View)",
        MAP_DIR / "2d_wall_only_goals_map.png"
    )

    # Save navigation goals JSON & YAML
    nav_goals = []
    for g in goals:
        nav_goals.append({
            "id": g["id"],
            "name": f"Waypoint_{g['id']}",
            "description": f"Goal candidate #{g['id']} on 0913 full SLAM map ({g['dist']:.2f}m from origin)",
            "x_m": round(g["x"], 3),
            "y_m": round(g["y"], 3),
            "z_m": 0.0,
            "yaw_deg": round(g["yaw_deg"], 1),
            "tolerance_m": 1.0,
            "snapshot_image": f"goals/{g['id']}.json"
        })

    config_json = {"goals": nav_goals}
    (MAP_DIR / "navigation_goals.json").write_text(json.dumps(config_json, indent=2))
    (REPO_ROOT / "config/navigation_goals.json").write_text(json.dumps(config_json, indent=2))

    config_yaml = {"goals": nav_goals}
    with open(MAP_DIR / "navigation_goals.yaml", "w") as f:
        yaml.dump(config_yaml, f, sort_keys=False, default_flow_style=False)
    with open(REPO_ROOT / "config/navigation_goals.yaml", "w") as f:
        yaml.dump(config_yaml, f, sort_keys=False, default_flow_style=False)

    print("Updated navigation_goals.json and navigation_goals.yaml in both 2dmap/0913 and config/")

if __name__ == "__main__":
    build_visualizations()
