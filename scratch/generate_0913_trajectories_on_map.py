#!/usr/bin/env python3
"""
Generate comprehensive publication-quality trajectory visualizations on the 0913 SLAM 2D map:
  1. fig6_five_goals_trajectories.png / .pdf (Canonical 5-goal comparison, clean Fig. 6 style)
  2. fig6_wall_only_trajectories.png / .pdf (CAD architectural wall-only style)
  3. all_0913_trajectories_map.png (All 15 valid runs across the entire campaign)
  4. Individual goal trajectory maps (goal_1 to goal_5)
  5. Trajectory CSV exports and benchmark metrics JSON
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
MAP_DIR = REPO_ROOT / "2dmap/0913"
META_PATH = MAP_DIR / "2d_metadata.json"
EP_DIR = Path("/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-20260913/episodes")
EXP_DIR = REPO_ROOT / "experiments/0913"
VIS_DIR = EXP_DIR / "visualizations"
TRAJ_DIR = EXP_DIR / "trajectories"

VIS_DIR.mkdir(parents=True, exist_ok=True)
TRAJ_DIR.mkdir(parents=True, exist_ok=True)

meta = json.loads(META_PATH.read_text())
resolution = float(meta["resolution"]) # 0.05 m/px
min_x = meta["min_x"]
max_y = meta["max_y"]

def to_px(x, y):
    return (x - min_x) / resolution, (max_y - y) / resolution

goals_meta = {
    "1": {"coord": (2.976, 1.675), "yaw": -0.165, "yaw_deg": -9.4, "dist": 3.415, "color": "#D83B01", "name": "Goal 1"},
    "2": {"coord": (4.807, -2.420), "yaw": 0.323, "yaw_deg": 18.5, "dist": 5.381, "color": "#744DA9", "name": "Goal 2"},
    "3": {"coord": (9.300, 0.424), "yaw": -0.202, "yaw_deg": -11.6, "dist": 9.309, "color": "#0078D4", "name": "Goal 3"},
    "4": {"coord": (10.614, -3.886), "yaw": -0.635, "yaw_deg": -36.4, "dist": 11.303, "color": "#E67E22", "name": "Goal 4"},
    "5": {"coord": (7.955, 4.189), "yaw": 1.445, "yaw_deg": 82.8, "dist": 8.989, "color": "#B4009E", "name": "Goal 5"},
}

canonical_runs = [
    {"ep": "full-goal-1-007", "gid": "1", "label": "Goal 1 - Tr 7 (SUCCESS, SPL 77.4%, 0.97m)"},
    {"ep": "full-goal-2-006", "gid": "2", "label": "Goal 2 - Tr 6 (Timeout, 2.82m)"},
    {"ep": "full-goal-3-008", "gid": "3", "label": "Goal 3 - Tr 8 (SUCCESS, SPL 54.8%, 0.99m)"},
    {"ep": "full-goal-4-009", "gid": "4", "label": "Goal 4 - Tr 9 (SUCCESS, SPL 49.4%, 0.91m)"},
    {"ep": "full-goal-5-006", "gid": "5", "label": "Goal 5 - Tr 6 (Inhibited, 5.04m)"},
]

# Load and extract all runs
all_episodes_data = []
for ep in sorted(EP_DIR.glob("full-goal-*")):
    events_file = ep / "full5m/runtime/events.jsonl"
    if not events_file.exists():
        continue
    poses = []
    times = []
    with open(events_file) as f:
        for line in f:
            e = json.loads(line)
            if e.get("event") == "control_pose":
                p = e["data"]["pose"]["position"]
                poses.append((p["x"], p["y"]))
                times.append(e.get("elapsed_s", 0.0))
    if len(poses) < 50:
        continue
    
    gid = ep.name.split("-")[2]
    ginfo = goals_meta[gid]
    gx, gy = ginfo["coord"]
    gdist = ginfo["dist"]
    
    lx, ly = poses[-1]
    final_dist = math.hypot(lx - gx, ly - gy)
    plen = sum(math.hypot(poses[i][0] - poses[i-1][0], poses[i][1] - poses[i-1][1]) for i in range(1, len(poses)))
    dur = times[-1] - times[0] if times else 0.0
    success = final_dist <= 1.0
    spl = (gdist / plen) if (success and plen > 0) else 0.0
    
    res = "UNKNOWN"
    res_file = ep / "full5m/runtime/result.json"
    if res_file.exists():
        rdata = json.loads(res_file.read_text())
        res = rdata.get("result_reason", rdata.get("reason", "UNKNOWN"))
        
    run_entry = {
        "trial_id": ep.name,
        "goal_id": gid,
        "straight_line_m": gdist,
        "path_length_m": round(plen, 2),
        "duration_s": round(dur, 1),
        "final_goal_dist_m": round(final_dist, 2),
        "success": success,
        "spl": round(spl, 3),
        "result_reason": res,
        "poses": poses,
        "times": times
    }
    all_episodes_data.append(run_entry)
    
    # Save CSV
    csv_path = TRAJ_DIR / f"{ep.name}_trajectory.csv"
    with open(csv_path, "w", newline="") as cf:
        writer = csv.writer(cf)
        writer.writerow(["time_s", "x_m", "y_m", "dist_to_goal_m"])
        for (px, py), t in zip(poses, times):
            d_to_g = math.hypot(px - gx, py - gy)
            writer.writerow([round(t, 3), round(px, 4), round(py, 4), round(d_to_g, 4)])

# Save summary CSV
summary_csv = EXP_DIR / "five_goals_summary.csv"
with open(summary_csv, "w", newline="") as sf:
    fields = ["trial_id", "goal_id", "straight_line_m", "path_length_m", "duration_s", "final_goal_dist_m", "success", "spl", "result_reason"]
    writer = csv.DictWriter(sf, fieldnames=fields)
    writer.writeheader()
    for r in all_episodes_data:
        row = {k: r[k] for k in fields}
        writer.writerow(row)
print(f"Saved summary CSV: {summary_csv}")

clean_base = np.array(Image.open(MAP_DIR / "2d_clean_publication.png"))
wall_base = np.array(Image.open(MAP_DIR / "2d_wall_only.png"))
h, w = clean_base.shape[:2]

rad_px = 1.0 / resolution # 20 px

def draw_goals_and_start(ax):
    # Start
    spx, spy = to_px(0.0, 0.0)
    ax.scatter([spx], [spy], s=200, color="#107C41", marker="o", edgecolors="white", linewidths=2.0, zorder=8, label="Start (Fixed Origin (0,0))")
    ax.text(spx - 10, spy + 25, "START (0,0)", fontsize=9.2, fontweight="bold", color="#107C41", zorder=9,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#107C41", lw=1.2, alpha=0.92))

    # Goals
    for gid, ginfo in goals_meta.items():
        gx, gy = ginfo["coord"]
        gyaw = ginfo["yaw"]
        col = ginfo["color"]
        gpx, gpy = to_px(gx, gy)
        
        # 1.0m tolerance circle
        circ = Circle((gpx, gpy), rad_px, fill=False, linestyle="--", edgecolor=col, linewidth=1.5, alpha=0.85, zorder=6)
        ax.add_patch(circ)
        
        # Star marker
        ax.scatter([gpx], [gpy], s=260, color=col, marker="*", edgecolors="black", linewidths=1.2, zorder=7)
        
        # Directional Arrow
        adx = 18 * math.cos(gyaw)
        ady = -18 * math.sin(gyaw)
        ax.annotate("", xy=(gpx + adx, gpy + ady), xytext=(gpx, gpy),
                    arrowprops=dict(arrowstyle="-|>", color=col, lw=2.0, mutation_scale=13), zorder=8)
        
        # Badge
        badge_text = f"Goal {gid}\n{ginfo['dist']:.1f}m, {ginfo['yaw_deg']:+.0f}°"
        offset_y = -18 if gid in ["1", "3", "5"] else 20
        ax.text(gpx + 14, gpy + offset_y, badge_text, fontsize=8.5, fontweight="bold", color=col, zorder=9,
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=col, lw=1.2, alpha=0.92))

def setup_axes_and_scale(ax):
    # View window covering corridor with comfortable margins
    ax.set_xlim(25, 430)
    ax.set_ylim(440, 60)
    ax.axis("off")
    
    # 5m Scale Bar
    scale_len_px = 5.0 / resolution # 100 px
    sb_x = 45
    sb_y = 420
    ax.plot([sb_x, sb_x + scale_len_px], [sb_y, sb_y], color="#1E2024", lw=4.5, zorder=11)
    ax.text(sb_x + scale_len_px / 2, sb_y - 8, "5 m", ha="center", va="bottom", fontsize=11, fontweight="bold", color="#1E2024", zorder=11)

# -------------------------------------------------------------------------
# 1. Fig. 6 Canonical Trajectories (Clean Map & Wall-Only)
# -------------------------------------------------------------------------
for base_img, name_tag, out_dir in [(clean_base, "fig6_five_goals_trajectories", MAP_DIR),
                                    (wall_base, "fig6_wall_only_trajectories", MAP_DIR)]:
    fig, ax = plt.subplots(figsize=(11, 9.5), dpi=300)
    ax.imshow(base_img)
    draw_goals_and_start(ax)
    
    for cr in canonical_runs:
        match_run = next((r for r in all_episodes_data if r["trial_id"] == cr["ep"]), None)
        if not match_run:
            continue
        col = goals_meta[cr["gid"]]["color"]
        poses = match_run["poses"]
        pxs = [(x - min_x) / resolution for x, y in poses]
        pys = [(max_y - y) / resolution for x, y in poses]
        
        ax.plot(pxs, pys, color=col, lw=2.8, alpha=0.92, label=cr["label"], zorder=7)
        
        # End marker
        is_succ = match_run["success"]
        marker_sym = "P" if is_succ else "X"
        ax.scatter([pxs[-1]], [pys[-1]], s=150, color=col, marker=marker_sym, edgecolors="white", linewidths=1.5, zorder=10)
        
    setup_axes_and_scale(ax)
    ax.legend(loc="upper right", frameon=True, framealpha=0.96, facecolor="white", edgecolor="#C0C0C0", fontsize=9.2, borderpad=0.7)
    title_text = "Unitree Go2 VLM Navigation Trajectories Across 5 Goal Poses (Paper Fig. 6)" if "wall" not in name_tag else "Unitree Go2 VLM Navigation Trajectories (Wall-Only CAD View)"
    plt.title(title_text, fontsize=13.5, fontweight="bold", pad=12)
    plt.tight_layout()
    
    png_path = out_dir / f"{name_tag}.png"
    pdf_path = out_dir / f"{name_tag}.pdf"
    plt.savefig(png_path, dpi=300, bbox_inches="tight")
    plt.savefig(pdf_path, bbox_inches="tight")
    # Also save copy to VIS_DIR
    plt.savefig(VIS_DIR / f"{name_tag}.png", dpi=300, bbox_inches="tight")
    plt.savefig(VIS_DIR / f"{name_tag}.pdf", bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {png_path} & {pdf_path}")

# -------------------------------------------------------------------------
# 2. All-Trials Overview Map (All 15 valid runs)
# -------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11, 9.5), dpi=300)
ax.imshow(clean_base)
draw_goals_and_start(ax)

# Plot all runs with varying alpha
for r in all_episodes_data:
    gid = r["goal_id"]
    col = goals_meta[gid]["color"]
    poses = r["poses"]
    pxs = [(x - min_x) / resolution for x, y in poses]
    pys = [(max_y - y) / resolution for x, y in poses]
    
    is_canonical = any(cr["ep"] == r["trial_id"] for cr in canonical_runs)
    alpha = 0.95 if is_canonical else 0.40
    lw = 2.6 if is_canonical else 1.5
    
    ax.plot(pxs, pys, color=col, lw=lw, alpha=alpha, zorder=6 if not is_canonical else 7)
    if r["success"]:
        ax.scatter([pxs[-1]], [pys[-1]], s=120, color=col, marker="P", edgecolors="white", linewidths=1.2, zorder=9)

setup_axes_and_scale(ax)
# Add custom legend for goals
for gid, ginfo in goals_meta.items():
    ax.plot([], [], color=ginfo["color"], lw=2.5, label=f"Goal {gid} Trajectories ({ginfo['dist']:.1f}m)")
ax.scatter([], [], color="#107C41", s=100, marker="P", label="Goal Reached (<1.0m)")

ax.legend(loc="upper right", frameon=True, framealpha=0.96, facecolor="white", edgecolor="#C0C0C0", fontsize=9.2, borderpad=0.7)
plt.title("Unitree Go2 VLM Navigation: All 15 Field Trials (0913 Full SLAM)", fontsize=13.5, fontweight="bold", pad=12)
plt.tight_layout()

all_png = MAP_DIR / "all_0913_trajectories_map.png"
all_pdf = MAP_DIR / "all_0913_trajectories_map.pdf"
plt.savefig(all_png, dpi=300, bbox_inches="tight")
plt.savefig(all_pdf, bbox_inches="tight")
plt.savefig(VIS_DIR / "all_0913_trajectories_map.png", dpi=300, bbox_inches="tight")
plt.savefig(VIS_DIR / "all_0913_trajectories_map.pdf", bbox_inches="tight")
plt.close(fig)
print(f"Saved: {all_png} & {all_pdf}")

# -------------------------------------------------------------------------
# 3. Per-Goal Individual Trajectory Breakdown
# -------------------------------------------------------------------------
for gid in ["1", "2", "3", "4", "5"]:
    gruns = [r for r in all_episodes_data if r["goal_id"] == gid]
    if not gruns:
        continue
    ginfo = goals_meta[gid]
    col = ginfo["color"]
    
    fig, ax = plt.subplots(figsize=(10, 8.5), dpi=300)
    ax.imshow(clean_base)
    
    # Start
    spx, spy = to_px(0.0, 0.0)
    ax.scatter([spx], [spy], s=190, color="#107C41", marker="o", edgecolors="white", linewidths=2.0, zorder=8, label="Start (0,0)")
    
    # This goal
    gx, gy = ginfo["coord"]
    gpx, gpy = to_px(gx, gy)
    circ = Circle((gpx, gpy), rad_px, fill=False, linestyle="--", edgecolor=col, linewidth=1.8, alpha=0.9, zorder=6)
    ax.add_patch(circ)
    ax.scatter([gpx], [gpy], s=260, color=col, marker="*", edgecolors="black", linewidths=1.2, zorder=7,
               label=f"Goal {gid}: ({gx:+.2f}m, {gy:+.2f}m) | {ginfo['dist']:.1f}m")
    
    # Plot each trial with distinct color shade / style
    trial_colors = ["#4E79A7", "#F28E2B", "#E15759", "#76B7B2", "#59A14F", "#EDC948"]
    for t_idx, r in enumerate(gruns):
        tc = trial_colors[t_idx % len(trial_colors)]
        poses = r["poses"]
        pxs = [(x - min_x) / resolution for x, y in poses]
        pys = [(max_y - y) / resolution for x, y in poses]
        
        status_lbl = f"SUCCESS (SPL {r['spl']*100:.1f}%)" if r["success"] else f"Dist {r['final_goal_dist_m']:.2f}m ({r['result_reason'][:18]})"
        label_t = f"{r['trial_id']}: {status_lbl}"
        ax.plot(pxs, pys, color=tc, lw=2.4, alpha=0.9, label=label_t, zorder=7)
        marker_sym = "P" if r["success"] else "X"
        ax.scatter([pxs[-1]], [pys[-1]], s=130, color=tc, marker=marker_sym, edgecolors="black", linewidths=1.2, zorder=9)
        
    setup_axes_and_scale(ax)
    ax.legend(loc="upper right", frameon=True, framealpha=0.95, facecolor="white", edgecolor="#C0C0C0", fontsize=9)
    plt.title(f"Unitree Go2 VLM Navigation: Goal {gid} Multi-Trial Trajectories", fontsize=13, fontweight="bold", pad=12)
    plt.tight_layout()
    
    g_png = VIS_DIR / f"goal_{gid}_all_trajectories.png"
    plt.savefig(g_png, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {g_png}")

print("All trajectory visualizations successfully generated!")
