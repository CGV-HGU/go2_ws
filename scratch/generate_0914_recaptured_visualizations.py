#!/usr/bin/env python3
"""
Generate publication-ready visualizations and configuration files for 0914 recaptured Goal 5.
"""

import json
import math
import numpy as np
import cv2
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from pathlib import Path
import yaml

REPO_ROOT = Path("/home/unitree/go2_ws_antarctica")
MAP_0913 = REPO_ROOT / "2dmap/0913"
MAP_0914 = REPO_ROOT / "2dmap/0914"

def generate():
    meta = json.loads((MAP_0913 / "2d_metadata.json").read_text())
    raw = np.array(Image.open(MAP_0913 / "0833.pgm"))
    
    # Read recaptured goals from 0914 JSON
    meta_new = json.loads((MAP_0914 / "2d_goals_map_recaptured5.json").read_text())
    goals_data = meta_new["goals"]
    
    h, w = raw.shape
    res = meta["resolution"]
    min_x = meta["min_x"]
    max_y = meta["max_y"]
    
    def to_px(x, y):
        return (x - min_x) / res, (max_y - y) / res

    # Clean map processing
    walls = (raw == 0).astype(np.uint8)
    free = (raw == 255).astype(np.uint8)

    kernel_fill = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    free_closed = cv2.morphologyEx(free, cv2.MORPH_CLOSE, kernel_fill)
    kernel_rect = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    free_solid = cv2.morphologyEx(free_closed, cv2.MORPH_CLOSE, kernel_rect)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_solid)
    clean_free = np.zeros_like(free_solid)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= 200:
            clean_free[labels == i] = 1

    contours, hierarchy = cv2.findContours(clean_free, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is not None:
        for i in range(len(contours)):
            if hierarchy[0][i][3] != -1:
                area = cv2.contourArea(contours[i])
                if area < 500:
                    cv2.drawContours(clean_free, [contours[i]], -1, 1, -1)

    kernel_wall = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thick_walls = cv2.dilate(walls, kernel_wall, iterations=1)
    num_w_labels, w_labels, w_stats, _ = cv2.connectedComponentsWithStats(thick_walls)
    filtered_walls = np.zeros_like(thick_walls)
    for i in range(1, num_w_labels):
        if w_stats[i, cv2.CC_STAT_AREA] >= 4:
            filtered_walls[w_labels == i] = 1

    clean_free[filtered_walls == 1] = 0

    clean_rgb = np.full((h, w, 3), (234, 236, 239), dtype=np.uint8)
    clean_rgb[clean_free == 1] = (255, 255, 255)
    clean_rgb[filtered_walls == 1] = (30, 32, 36)

    wall_only_rgb = np.full((h, w, 3), (255, 255, 255), dtype=np.uint8)
    wall_only_rgb[filtered_walls == 1] = (20, 20, 20)

    # Start coordinates (map overlay)
    start_map_x, start_map_y = meta_new["map_overlay_origin_xy"]
    start_heading = meta_new["map_overlay_heading_rad"]
    start_px, start_py = to_px(start_map_x, start_map_y)

    goal_styles = [
        {"color": "#D83B01", "name": "Goal 1", "text_offset": (15, -15)}, # Red-orange
        {"color": "#744DA9", "name": "Goal 2", "text_offset": (15, 18)},  # Purple
        {"color": "#0078D4", "name": "Goal 3", "text_offset": (15, -15)}, # Blue
        {"color": "#E67E22", "name": "Goal 4", "text_offset": (15, 18)},  # Amber
        {"color": "#C02C72", "name": "Goal 5", "text_offset": (18, -12)}, # Magenta / Pink
    ]

    rad_px = 1.0 / res # 20 px
    arrow_len_px = 18.0

    def render_plot(base_img, title, out_png, out_pdf=None):
        fig, ax = plt.subplots(figsize=(10.5, 9.2), dpi=300)
        ax.imshow(base_img)

        # Plot Start
        ax.scatter([start_px], [start_py], s=180, color="#107C41", marker="o", edgecolors="white", linewidths=2.0, zorder=8, label="Start (Fixed Origin (0,0))")
        ax.text(start_px - 8, start_py + 22, "START (0,0)", fontsize=9.5, fontweight="bold", color="#107C41", zorder=9,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#107C41", lw=1.2, alpha=0.9))

        all_px = [start_px]
        all_py = [start_py]

        for idx, g in enumerate(goals_data):
            st = goal_styles[idx]
            col = st["color"]
            mx, my = g["map_overlay_x_m"], g["map_overlay_y_m"]
            gpx, gpy = to_px(mx, my)
            all_px.append(gpx)
            all_py.append(gpy)

            # Yaw on map
            yaw_map = start_heading + g["saved_yaw_rad"]

            # Tolerance circle
            circ = Circle((gpx, gpy), rad_px, fill=False, linestyle="--", edgecolor=col, linewidth=1.6, alpha=0.85, zorder=6)
            ax.add_patch(circ)

            # Star marker
            is_new = (g["goal_label"] == "5")
            lbl_tag = f"Goal 5 (Recaptured)" if is_new else f"Goal {g['goal_label']}"
            ax.scatter([gpx], [gpy], s=250, color=col, marker="*", edgecolors="black", linewidths=1.2, zorder=8,
                       label=f"{lbl_tag}: ({g['fixed_start_x_m']:+.2f}m, {g['fixed_start_y_m']:+.2f}m) | {g['straight_distance_m']:.1f}m, {math.degrees(g['saved_yaw_rad']):+.0f}°")

            # Yaw arrow
            dx = arrow_len_px * math.cos(yaw_map)
            dy = -arrow_len_px * math.sin(yaw_map)
            ax.annotate("", xy=(gpx + dx, gpy + dy), xytext=(gpx, gpy),
                        arrowprops=dict(arrowstyle="-|>", color=col, lw=2.4, mutation_scale=14), zorder=9)

            # Text badge
            ox, oy = st["text_offset"]
            badge_title = "Goal 5 (NEW)" if is_new else f"Goal {g['goal_label']}"
            badge_text = f"{badge_title}\n{g['straight_distance_m']:.1f}m, {math.degrees(g['saved_yaw_rad']):+.0f}°"
            ax.text(gpx + ox, gpy + oy, badge_text, fontsize=8.5, fontweight="bold", color=col, zorder=10,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=col, lw=1.2, alpha=0.92))

        # Margin
        margin_px = 35
        min_x_lim = max(0, min(all_px) - margin_px - 20)
        max_x_lim = min(w, max(all_px) + margin_px + 80)
        min_y_lim = min(h, max(all_py) + margin_px + 30)
        max_y_lim = max(0, min(all_py) - margin_px - 30)

        ax.set_xlim(min_x_lim, max_x_lim)
        ax.set_ylim(min_y_lim, max_y_lim)
        ax.axis("off")

        # 5m Scale Bar
        scale_len_m = 5.0
        scale_len_px = scale_len_m / res
        sb_x = min_x_lim + 20
        sb_y = min_y_lim - 20
        ax.plot([sb_x, sb_x + scale_len_px], [sb_y, sb_y], color="#1E2024", lw=4.5, zorder=11)
        ax.text(sb_x + scale_len_px / 2, sb_y - 8, "5 m", ha="center", va="bottom", fontsize=11, fontweight="bold", color="#1E2024", zorder=11)

        # Legend
        ax.legend(loc="upper right", frameon=True, framealpha=0.95, facecolor="white", edgecolor="#D0D0D0", fontsize=9.0, borderpad=0.8)
        plt.title(title, fontsize=13, fontweight="bold", pad=12)
        plt.tight_layout()

        plt.savefig(out_png, dpi=300, bbox_inches="tight")
        if out_pdf:
            plt.savefig(out_pdf, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved: {out_png}")

    # Render Clean Map
    render_plot(
        clean_rgb,
        "Unitree Go2 Top-View Floor Plan & 5 Recorded Goals (0914 Recaptured)",
        MAP_0914 / "2d_clean_goals_map.png",
        MAP_0914 / "fig_five_goals.pdf"
    )

    # Render Wall Only CAD Map
    render_plot(
        wall_only_rgb,
        "Unitree Go2 Architectural Floor Plan & Goal Poses (0914 Wall-Only CAD View)",
        MAP_0914 / "2d_wall_only_goals_map.png"
    )

    # Update navigation_goals.json and .yaml
    nav_goals = []
    for g in goals_data:
        gid = int(g["goal_label"])
        is_recaptured = (gid == 5)
        desc = f"Goal candidate #5 on 0914 map (Recaptured Long Goal: {g['straight_distance_m']:.2f}m from origin)" if is_recaptured else f"Goal candidate #{gid} on 0913 full SLAM map ({g['straight_distance_m']:.2f}m from origin)"
        nav_goals.append({
            "id": gid,
            "name": f"Waypoint_{gid}",
            "description": desc,
            "x_m": round(g["fixed_start_x_m"], 3),
            "y_m": round(g["fixed_start_y_m"], 3),
            "z_m": 0.0,
            "yaw_deg": round(math.degrees(g["saved_yaw_rad"]), 1),
            "tolerance_m": 1.0,
            "snapshot_image": f"goals/{gid}.json"
        })

    config_dict = {"goals": nav_goals}
    
    # Write to 0914
    (MAP_0914 / "navigation_goals.json").write_text(json.dumps(config_dict, indent=2))
    with open(MAP_0914 / "navigation_goals.yaml", "w") as f:
        yaml.dump(config_dict, f, sort_keys=False, default_flow_style=False)

    # Write to repo root config/
    (REPO_ROOT / "config/navigation_goals.json").write_text(json.dumps(config_dict, indent=2))
    with open(REPO_ROOT / "config/navigation_goals.yaml", "w") as f:
        yaml.dump(config_dict, f, sort_keys=False, default_flow_style=False)

    print("Updated navigation_goals.json and navigation_goals.yaml")

if __name__ == "__main__":
    generate()
