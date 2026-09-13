#!/usr/bin/env python3
"""
Generate publication-ready 2D Top-View floor plan and trajectory figures
matching IEEE ICRA Paper Fig. 6 standards (clean solid walls, pure white walkable
corridor, no pointcloud sand/speckles, scale bar, high-contrast trajectories).
"""
import os
import json
import math
import csv
from pathlib import Path
import numpy as np
import cv2
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

REPO_ROOT = Path("/home/unitree/go2_ws_antarctica")

def generate_paper_fig6():
    meta_path = REPO_ROOT / "2dmap/0912/2d_metadata.json"
    pgm_path = REPO_ROOT / "2dmap/0912/0833.pgm"

    meta = json.loads(meta_path.read_text())
    raw = np.array(Image.open(pgm_path))
    h, w = raw.shape

    walls = (raw == 0).astype(np.uint8)
    free = (raw == 255).astype(np.uint8)

    # 1. Fill all internal gaps and holes inside the corridor
    kernel_fill = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    free_closed = cv2.morphologyEx(free, cv2.MORPH_CLOSE, kernel_fill)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    free_solid = cv2.morphologyEx(free_closed, cv2.MORPH_CLOSE, kernel_close)

    # 2. Filter out isolated tiny noise speckles
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_solid)
    min_size = 200
    clean_free = np.zeros_like(free_solid)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= min_size:
            clean_free[labels == i] = 1

    # 3. Thicken architectural walls
    kernel_wall = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thick_walls = cv2.dilate(walls, kernel_wall, iterations=1)
    clean_free[thick_walls == 1] = 0

    # 4. Construct high-contrast publication base map
    # Background: Neutral Soft Gray #EAECEF
    map_rgb = np.full((h, w, 3), (234, 236, 239), dtype=np.uint8)
    # Walkable free space: Crisp Pure White #FFFFFF (NO sand speckles!)
    map_rgb[clean_free == 1] = (255, 255, 255)
    # Architectural walls: Dark Charcoal #1E2024
    map_rgb[thick_walls == 1] = (30, 32, 36)

    # Save clean standalone 2D floor plan
    out_map_png = REPO_ROOT / "2dmap/0912/2d_clean_publication.png"
    Image.fromarray(map_rgb).save(out_map_png)
    print(f"Saved clean base map to {out_map_png}")

    # Coordinate transform parameters
    res = meta["resolution"] # 0.05 m/px
    min_x = meta["min_x"]
    max_y = meta["max_y"]

    def to_px(mx, my):
        return (mx - min_x) / res, (max_y - my) / res

    start_x, start_y = -63.132, 10.395
    theta = 0.02176
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    # Goals
    g1_fwd, g1_lft = 3.968, -2.772
    g2_fwd, g2_lft = 2.849, 1.258

    g1_mx = start_x + g1_fwd * cos_t - g1_lft * sin_t
    g1_my = start_y + g1_fwd * sin_t + g1_lft * cos_t
    g1_px, g1_py = to_px(g1_mx, g1_my)

    g2_mx = start_x + g2_fwd * cos_t - g2_lft * sin_t
    g2_my = start_y + g2_fwd * sin_t + g2_lft * cos_t
    g2_px, g2_py = to_px(g2_mx, g2_my)

    # Render Paper Fig 6 (Trajectories Overlay)
    fig, ax = plt.subplots(figsize=(10, 8.5), dpi=300)
    ax.imshow(map_rgb)

    # Start marker
    spx, spy = to_px(start_x, start_y)
    ax.scatter([spx], [spy], s=170, color="#107C41", marker="o", edgecolors="white", linewidths=1.8, zorder=7, label="Start (Fixed Origin)")

    # Goal markers with 1.0m radius
    rad_px = 1.0 / res # 20 px
    ax.scatter([g1_px], [g1_py], s=230, color="#D83B01", marker="*", edgecolors="black", linewidths=1.2, zorder=7, label="Goal 1 (4.84m, 1m radius)")
    ax.add_patch(Circle((g1_px, g1_py), rad_px, fill=False, linestyle="--", edgecolor="#D83B01", linewidth=1.5, alpha=0.85))

    ax.scatter([g2_px], [g2_py], s=230, color="#744DA9", marker="*", edgecolors="black", linewidths=1.2, zorder=7, label="Goal 2 (3.11m, 1m radius)")
    ax.add_patch(Circle((g2_px, g2_py), rad_px, fill=False, linestyle="--", edgecolor="#744DA9", linewidth=1.5, alpha=0.85))

    # Trajectories
    trials = [
        (REPO_ROOT / "experiments/0913/goal_1/trial_01/trajectory.csv", "#72B7B2", "Goal 1 - Tr 1 (Timeout)"),
        (REPO_ROOT / "experiments/0913/goal_1/trial_02/trajectory.csv", "#4E79A7", "Goal 1 - Tr 2 (SPL 69.9%)"),
        (REPO_ROOT / "experiments/0913/goal_1/trial_03/trajectory.csv", "#1B4F72", "Goal 1 - Tr 3 (SPL 81.0%)"),
        (REPO_ROOT / "experiments/0913/goal_2/trial_01/trajectory.csv", "#F28E2B", "Goal 2 - Tr 1 (SPL 100%)"),
        (REPO_ROOT / "experiments/0913/goal_2/trial_02/trajectory.csv", "#EDC948", "Goal 2 - Tr 2 (SPL 100%)"),
        (REPO_ROOT / "experiments/0913/goal_2/trial_03/trajectory.csv", "#B07AA1", "Goal 2 - Tr 3 (SPL 100%)"),
    ]

    for path, color, label in trials:
        rows = list(csv.DictReader(open(path)))
        xs, ys = [], []
        for r in rows:
            mx = start_x + float(r["start_forward_m"]) * cos_t - float(r["start_left_m"]) * sin_t
            my = start_y + float(r["start_forward_m"]) * sin_t + float(r["start_left_m"]) * cos_t
            px, py = to_px(mx, my)
            xs.append(px)
            ys.append(py)
        ax.plot(xs, ys, color=color, lw=2.5, alpha=0.92, label=label, zorder=5)
        ax.scatter([xs[-1]], [ys[-1]], s=65, color=color, marker="s", edgecolors="white", linewidths=1.2, zorder=6)

    # Focused view on navigation corridor
    margin = 3.5
    min_view_x = min(start_x, g1_mx, g2_mx) - margin
    max_view_x = max(start_x, g1_mx, g2_mx) + margin
    min_view_y = min(start_y, g1_my, g2_my) - margin
    max_view_y = max(start_y, g1_my, g2_my) + margin

    x_min_px, y_max_px = to_px(min_view_x, min_view_y)
    x_max_px, y_min_px = to_px(max_view_x, max_view_y)

    ax.set_xlim(x_min_px, x_max_px)
    ax.set_ylim(y_max_px, y_min_px)
    ax.axis("off")

    # 5m Scale Bar
    scale_len_m = 5.0
    scale_len_px = scale_len_m / res # 100 px
    sb_x = x_min_px + 25
    sb_y = y_max_px - 35
    ax.plot([sb_x, sb_x + scale_len_px], [sb_y, sb_y], color="black", lw=4.0, zorder=8)
    ax.text(sb_x + scale_len_px / 2, sb_y - 12, "5 m", ha="center", va="bottom", fontsize=11, fontweight="bold", color="black", zorder=8)

    ax.legend(loc="upper left", frameon=True, framealpha=0.94, facecolor="white", edgecolor="#D0D0D0", fontsize=9.5)
    plt.title("Unitree Go2 Asynchronous VLM Navigation Trajectories (Paper Fig. 6)", fontsize=13.5, fontweight="bold", pad=12)
    plt.tight_layout()

    out_fig_png = REPO_ROOT / "experiments/0913/visualizations/fig6_topview_trajectories.png"
    out_fig_pdf = REPO_ROOT / "experiments/0913/visualizations/fig6_topview_trajectories.pdf"
    plt.savefig(out_fig_png, dpi=300)
    plt.savefig(out_fig_pdf)
    plt.close(fig)
    print(f"Saved Fig 6 style figures:\n  • {out_fig_png}\n  • {out_fig_pdf}")

if __name__ == "__main__":
    generate_paper_fig6()
