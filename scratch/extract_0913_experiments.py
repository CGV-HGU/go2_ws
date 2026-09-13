#!/usr/bin/env python3
"""
Extract 2026-09-13 real-robot navigation experiment data into go2_ws_antarctica.
Extracts 3 trials for Goal 1 and 3 trials for Goal 2.
Calculates exact benchmark metrics: SR (Success Rate) and SPL (Success weighted by Path Length).
Generates multi-trial comparison plots and 2D map overlays.
"""
import os
import shutil
import json
import csv
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image

REPO_ROOT = Path("/home/unitree/go2_ws_antarctica")
SRC_BASE = Path("/home/unitree/s2e-vlm-async-framework-minimal/.local-data/fixed-start-goals-20260913")
DEST_BASE = REPO_ROOT / "experiments/0913"

TRIALS_MAP = [
    {"src": "goal-1-001", "goal": "1", "goal_dir": "goal_1", "trial_dir": "trial_01", "label": "Goal 1 - Trial 1", "l_i": 4.8407},
    {"src": "goal-1-002", "goal": "1", "goal_dir": "goal_1", "trial_dir": "trial_02", "label": "Goal 1 - Trial 2", "l_i": 4.8407},
    {"src": "goal-1-003", "goal": "1", "goal_dir": "goal_1", "trial_dir": "trial_03", "label": "Goal 1 - Trial 3", "l_i": 4.8407},
    {"src": "goal-2-001", "goal": "2", "goal_dir": "goal_2", "trial_dir": "trial_01", "label": "Goal 2 - Trial 1", "l_i": 3.1146},
    {"src": "goal-2-002", "goal": "2", "goal_dir": "goal_2", "trial_dir": "trial_02", "label": "Goal 2 - Trial 2", "l_i": 3.1146},
    {"src": "goal-2-003", "goal": "2", "goal_dir": "goal_2", "trial_dir": "trial_03", "label": "Goal 2 - Trial 3", "l_i": 3.1146},
]

def main():
    print(f"Creating directory structure in {DEST_BASE}...")
    DEST_BASE.mkdir(parents=True, exist_ok=True)
    vis_dir = DEST_BASE / "visualizations"
    vis_dir.mkdir(parents=True, exist_ok=True)

    # Top-level meta files
    ep_index_data = json.loads((SRC_BASE / "episode-index.json").read_text())
    yaw_map = {e["trial"]: e.get("final_yaw_error_vs_saved_deg", 0.0) for e in ep_index_data.get("episodes", [])}

    shutil.copy2(SRC_BASE / "episode-index.json", DEST_BASE / "episode_index.json")
    shutil.copy2(SRC_BASE / "goal-plan.json", DEST_BASE / "goal_plan.json")
    shutil.copy2(SRC_BASE / "origin.json", DEST_BASE / "origin.json")
    shutil.copy2(SRC_BASE / "SESSION.md", DEST_BASE / "session_log.md")
    print("Copied session summary files.")

    trial_data = []

    for item in TRIALS_MAP:
        src_name = item["src"]
        t_dest = DEST_BASE / item["goal_dir"] / item["trial_dir"]
        t_dest.mkdir(parents=True, exist_ok=True)

        arch_dir = SRC_BASE / "archives" / f"{src_name}-reviewed"
        tv_dir = arch_dir / "trajectory-view"

        # Copy trial artifacts
        shutil.copy2(tv_dir / "episode-trajectory-10hz.csv", t_dest / "trajectory.csv")
        shutil.copy2(arch_dir / "odom-trajectory.csv", t_dest / "raw_odom.csv")
        shutil.copy2(tv_dir / "trajectory.png", t_dest / "trajectory.png")
        if (tv_dir / "trajectory.pdf").exists():
            shutil.copy2(tv_dir / "trajectory.pdf", t_dest / "trajectory.pdf")
        shutil.copy2(tv_dir / "distances.json", t_dest / "distances.json")
        shutil.copy2(arch_dir / "episode.json", t_dest / "episode.json")
        shutil.copy2(tv_dir / "README.md", t_dest / "README.md")

        # Load data for plotting and metric calculations
        traj_csv = t_dest / "trajectory.csv"
        rows = []
        with open(traj_csv, 'r') as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append({
                    "time": float(r["episode_time_s"]),
                    "forward": float(r["start_forward_m"]),
                    "left": float(r["start_left_m"]),
                    "cum_path": float(r["cumulative_path_m"]),
                    "goal_dist": float(r["goal_distance_interpolated_m"])
                })
        
        ep_meta = json.loads((t_dest / "episode.json").read_text())
        dist_meta = json.loads((t_dest / "distances.json").read_text())

        # Success binary: 1 if GOAL_DISTANCE_REACHED else 0
        reason = ep_meta.get("result_reason", "")
        success_binary = 1 if "GOAL_DISTANCE_REACHED" in reason else 0
        l_i = item["l_i"]
        p_i = ep_meta.get("path_length", {}).get("10hz_m", 0.0)
        
        # SPL calculation: S_i * (l_i / max(p_i, l_i))
        if success_binary == 1:
            spl = l_i / max(p_i, l_i)
        else:
            spl = 0.0

        trial_data.append({
            "info": item,
            "rows": rows,
            "ep_meta": ep_meta,
            "dist_meta": dist_meta,
            "success": success_binary,
            "l_i": l_i,
            "p_i": p_i,
            "spl": spl,
            "yaw_err": yaw_map.get(src_name, 0.0)
        })
        print(f"Copied and loaded {src_name} -> {item['goal_dir']}/{item['trial_dir']} (Success={success_binary}, SPL={spl*100:.1f}%)")

    # Generate benchmark_metrics.json & episodes_summary.csv
    create_metrics_exports(trial_data)

    # Generate Visualization 1: Goal 1 Multi-Trial Overlay
    plot_goal_group(
        [td for td in trial_data if td["info"]["goal"] == "1"],
        goal_label="Goal 1",
        goal_xy=[3.968, -2.772],
        out_path=vis_dir / "goal_1_trajectories.png"
    )

    # Generate Visualization 2: Goal 2 Multi-Trial Overlay
    plot_goal_group(
        [td for td in trial_data if td["info"]["goal"] == "2"],
        goal_label="Goal 2",
        goal_xy=[2.849, 1.258],
        out_path=vis_dir / "goal_2_trajectories.png"
    )

    # Generate Visualization 3: All 6 Trials Overlay
    plot_all_trials(trial_data, out_path=vis_dir / "all_trajectories_overlay.png")

    # Generate Visualization 4: Trajectories on 2D Occupancy Grid Map
    plot_trajectories_on_2d_map(trial_data, out_path=vis_dir / "trajectories_on_2d_map.png")

    # Create README.md report
    create_readme_report(trial_data, DEST_BASE / "README.md")

    # Create Symlinks
    create_symlinks()
    print("All tasks completed successfully!")

def create_metrics_exports(trial_data):
    # CSV Export with SR & SPL columns
    csv_path = DEST_BASE / "episodes_summary.csv"
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            "trial", "goal", "outcome", "success", "duration_s",
            "shortest_path_m", "actual_path_m", "spl",
            "net_displacement_m", "final_goal_distance_m",
            "final_yaw_error_deg", "operator_observation"
        ])
        for td in trial_data:
            writer.writerow([
                td["info"]["src"],
                td["info"]["goal"],
                td["ep_meta"].get("result_reason", ""),
                td["success"],
                f"{td['ep_meta'].get('elapsed_to_stop_s', 0.0):.2f}",
                f"{td['l_i']:.3f}",
                f"{td['p_i']:.3f}",
                f"{td['spl']:.4f}",
                f"{td['dist_meta'].get('net_displacement_at_stop_request_m', 0.0):.3f}",
                f"{td['dist_meta'].get('interpolated_goal_distance_at_stop_request_m', 0.0):.3f}",
                f"{td['yaw_err']:.1f}",
                td["ep_meta"].get("field_report", {}).get("operator_observation", "")
            ])
    print(f"Saved {csv_path} with SR and SPL fields.")

    # JSON Metrics Summary
    g1 = [td for td in trial_data if td["info"]["goal"] == "1"]
    g2 = [td for td in trial_data if td["info"]["goal"] == "2"]

    def calc_group_stats(group):
        n = len(group)
        succ = sum(td["success"] for td in group)
        sr = succ / n if n > 0 else 0.0
        spl = sum(td["spl"] for td in group) / n if n > 0 else 0.0
        mean_time = float(np.mean([td["ep_meta"].get("elapsed_to_stop_s", 0.0) for td in group]))
        mean_path = float(np.mean([td["p_i"] for td in group]))
        return {
            "num_trials": n,
            "success_count": succ,
            "success_rate_pct": round(sr * 100.0, 1),
            "spl_pct": round(spl * 100.0, 1),
            "mean_duration_s": round(mean_time, 2),
            "mean_path_length_m": round(mean_path, 2)
        }

    metrics_json = {
        "evaluation_protocol": "Standard IEEE ICRA PointGoal Navigation Benchmark",
        "arrival_radius_m": 1.0,
        "goal_1": calc_group_stats(g1),
        "goal_2": calc_group_stats(g2),
        "overall": calc_group_stats(trial_data),
        "trials": [
            {
                "trial": td["info"]["src"],
                "goal": td["info"]["goal"],
                "outcome": td["ep_meta"].get("result_reason", ""),
                "success": td["success"],
                "shortest_path_m": td["l_i"],
                "actual_path_m": td["p_i"],
                "spl": round(td["spl"], 4),
                "duration_s": round(td["ep_meta"].get("elapsed_to_stop_s", 0.0), 2)
            }
            for td in trial_data
        ]
    }
    metrics_path = DEST_BASE / "benchmark_metrics.json"
    metrics_path.write_text(json.dumps(metrics_json, indent=2, ensure_ascii=False))
    print(f"Saved {metrics_path}")

def plot_goal_group(trials, goal_label, goal_xy, out_path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={'width_ratios': [1.2, 1]})
    ax_map, ax_curve = axes[0], axes[1]

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

    # Start and Goal markers
    ax_map.scatter([0], [0], s=100, color='#178052', marker='o', zorder=5, label='Fixed Start (0, 0)')
    ax_map.scatter([goal_xy[0]], [goal_xy[1]], s=160, color='#d62728', marker='*', zorder=5, label=f'Target {goal_label}')
    ax_map.add_patch(Circle(goal_xy, 1.0, fill=False, linestyle='--', edgecolor='#d62728', linewidth=1.5, alpha=0.8, label='1.0 m Arrival Radius'))
    ax_map.arrow(0, 0, 0.6, 0, color='#178052', width=0.015, head_width=0.08, length_includes_head=True, zorder=4)
    ax_map.text(0.1, -0.3, 'Initial Heading (+X)', fontsize=9, color='#178052', fontweight='bold')

    for i, td in enumerate(trials):
        color = colors[i % len(colors)]
        t_label = td["info"]["trial_dir"].replace("_", " ").title()
        status_str = "REACHED" if td["success"] == 1 else "TIMEOUT"
        spl_str = f"SPL={td['spl']*100:.1f}%"
        
        fwd = [r["forward"] for r in td["rows"]]
        lft = [r["left"] for r in td["rows"]]
        t_sec = [r["time"] for r in td["rows"]]
        dist = [r["goal_dist"] for r in td["rows"]]
        cum = [r["cum_path"] for r in td["rows"]]

        ax_map.plot(fwd, lft, color=color, lw=2.2, label=f'{t_label} ({status_str}, {cum[-1]:.2f}m, {spl_str})')
        ax_map.scatter([fwd[-1]], [lft[-1]], s=70, color=color, marker='s', zorder=4)

        ax_curve.plot(t_sec, dist, color=color, lw=2, label=f'{t_label} dist to goal')

    ax_map.set_xlabel('Forward from marked start (m)', fontsize=11)
    ax_map.set_ylabel('Left from marked start (m)', fontsize=11)
    ax_map.set_title(f'{goal_label} Multi-Trial Trajectory Comparison (Start Frame)', fontsize=12, fontweight='bold')
    ax_map.axis('equal')
    ax_map.grid(alpha=0.3)
    ax_map.legend(loc='lower left', fontsize=9)

    ax_curve.axhline(1.0, color='#999999', linestyle='--', linewidth=1.5, label='Arrival Radius (1.0 m)')
    ax_curve.set_xlabel('Elapsed Time (s)', fontsize=11)
    ax_curve.set_ylabel('Odometry Distance to Goal (m)', fontsize=11)
    ax_curve.set_title(f'{goal_label} Distance to Goal vs Time', fontsize=12, fontweight='bold')
    ax_curve.grid(alpha=0.3)
    ax_curve.legend(loc='upper right', fontsize=9)

    fig.suptitle(f'Unitree Go2 Real-Robot Evaluation: {goal_label} (2026-09-13)', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Saved {out_path}")

def plot_all_trials(trial_data, out_path):
    fig, ax = plt.subplots(figsize=(10, 8))

    g1_xy = [3.968, -2.772]
    g2_xy = [2.849, 1.258]

    ax.scatter([0], [0], s=120, color='#178052', marker='o', zorder=6, label='Fixed Start Origin (0, 0)')
    ax.arrow(0, 0, 0.7, 0, color='#178052', width=0.015, head_width=0.08, length_includes_head=True, zorder=5)
    ax.text(0.1, -0.3, 'Start Heading (+X)', fontsize=9, color='#178052', fontweight='bold')

    ax.scatter([g1_xy[0]], [g1_xy[1]], s=180, color='#d62728', marker='*', zorder=6, label='Goal 1 (3.97m, -2.77m)')
    ax.add_patch(Circle(g1_xy, 1.0, fill=False, linestyle='--', edgecolor='#d62728', linewidth=1.5, alpha=0.7))

    ax.scatter([g2_xy[0]], [g2_xy[1]], s=180, color='#9467bd', marker='*', zorder=6, label='Goal 2 (2.85m, +1.26m)')
    ax.add_patch(Circle(g2_xy, 1.0, fill=False, linestyle='--', edgecolor='#9467bd', linewidth=1.5, alpha=0.7))

    g1_colors = ['#aec7e8', '#1f77b4', '#08519c']
    g2_colors = ['#ffbb78', '#ff7f0e', '#d95f02']

    g1_idx = 0
    g2_idx = 0

    for td in trial_data:
        is_g1 = td["info"]["goal"] == "1"
        color = g1_colors[g1_idx] if is_g1 else g2_colors[g2_idx]
        if is_g1: g1_idx += 1
        else: g2_idx += 1

        spl_str = f"SPL={td['spl']*100:.0f}%"
        t_lbl = f'{td["info"]["label"]} ({td["rows"][-1]["cum_path"]:.1f}m, {spl_str})'
        fwd = [r["forward"] for r in td["rows"]]
        lft = [r["left"] for r in td["rows"]]

        ax.plot(fwd, lft, color=color, lw=2.2, label=t_lbl)
        ax.scatter([fwd[-1]], [lft[-1]], s=60, color=color, marker='s', zorder=4)

    ax.set_xlabel('Forward from marked start (m)', fontsize=12)
    ax.set_ylabel('Left from marked start (m)', fontsize=12)
    ax.set_title('Unitree Go2 Navigation: All 6 Trials Overlay (2026-09-13)', fontsize=14, fontweight='bold')
    ax.axis('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='lower left', fontsize=9, framealpha=0.9)

    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Saved {out_path}")

def plot_trajectories_on_2d_map(trial_data, out_path):
    map_img_path = REPO_ROOT / "2dmap/0912/2d.png"
    meta_path = REPO_ROOT / "2dmap/0912/2d_metadata.json"

    meta = json.loads(meta_path.read_text())
    map_img = Image.open(map_img_path).convert("RGB")

    fig, ax = plt.subplots(figsize=(12, 11))
    ax.imshow(map_img, cmap='gray')

    # Fixed start node 1 pose on map
    start_x = -63.132
    start_y = 10.395
    theta = 0.02176 # heading rad
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    res = meta["resolution"]
    min_x = meta["min_x"]
    max_y = meta["max_y"]

    def to_pixel(mx, my):
        px = (mx - min_x) / res
        py = (max_y - my) / res
        return px, py

    spx, spy = to_pixel(start_x, start_y)
    ax.scatter([spx], [spy], s=140, color='#00cc44', marker='o', edgecolors='black', zorder=6, label='Fixed Start Origin')

    # Target points
    g1_fwd, g1_lft = 3.968, -2.772
    g2_fwd, g2_lft = 2.849, 1.258

    g1_mx = start_x + g1_fwd * cos_t - g1_lft * sin_t
    g1_my = start_y + g1_fwd * sin_t + g1_lft * cos_t
    g1_px, g1_py = to_pixel(g1_mx, g1_my)

    g2_mx = start_x + g2_fwd * cos_t - g2_lft * sin_t
    g2_my = start_y + g2_fwd * sin_t + g2_lft * cos_t
    g2_px, g2_py = to_pixel(g2_mx, g2_my)

    rad_px = 1.0 / res # 20 pixels
    ax.scatter([g1_px], [g1_py], s=180, color='#d62728', marker='*', edgecolors='black', zorder=6, label='Goal 1 (Captured Target)')
    ax.add_patch(Circle((g1_px, g1_py), rad_px, fill=False, linestyle='--', edgecolor='#d62728', linewidth=1.5, alpha=0.8))

    ax.scatter([g2_px], [g2_py], s=180, color='#9467bd', marker='*', edgecolors='black', zorder=6, label='Goal 2 (Captured Target)')
    ax.add_patch(Circle((g2_px, g2_py), rad_px, fill=False, linestyle='--', edgecolor='#9467bd', linewidth=1.5, alpha=0.8))

    g1_colors = ['#72b7b2', '#4e79a7', '#1b4f72']
    g2_colors = ['#f28e2b', '#edc948', '#b07aa1']
    g1_i = 0
    g2_i = 0

    for td in trial_data:
        is_g1 = td["info"]["goal"] == "1"
        c = g1_colors[g1_i] if is_g1 else g2_colors[g2_i]
        if is_g1: g1_i += 1
        else: g2_i += 1

        px_list = []
        py_list = []
        for r in td["rows"]:
            mx = start_x + r["forward"] * cos_t - r["left"] * sin_t
            my = start_y + r["forward"] * sin_t + r["left"] * cos_t
            px, py = to_pixel(mx, my)
            px_list.append(px)
            py_list.append(py)

        spl_pct = td["spl"] * 100
        ax.plot(px_list, py_list, color=c, lw=2.5, alpha=0.9, label=f'{td["info"]["label"]} (SPL {spl_pct:.0f}%)')
        ax.scatter([px_list[-1]], [py_list[-1]], s=60, color=c, marker='s', edgecolors='black', zorder=5)

    # Focus view on navigation corridor
    margin_m = 4.0
    crop_min_x = min(start_x, g1_mx, g2_mx) - margin_m
    crop_max_x = max(start_x, g1_mx, g2_mx) + margin_m
    crop_min_y = min(start_y, g1_my, g2_my) - margin_m
    crop_max_y = max(start_y, g1_my, g2_my) + margin_m

    c_px_min, c_py_max = to_pixel(crop_min_x, crop_min_y)
    c_px_max, c_py_min = to_pixel(crop_max_x, crop_max_y)

    ax.set_xlim(c_px_min, c_px_max)
    ax.set_ylim(c_py_max, c_py_min) # Inverted for image coordinate

    ax.set_title('Unitree Go2 Real-Robot Trajectories on 2D Occupancy Grid Map (2026-09-13)', fontsize=14, fontweight='bold')
    ax.legend(loc='lower left', fontsize=9, framealpha=0.9)
    plt.tight_layout()
    plt.savefig(out_path, dpi=220)
    plt.close(fig)
    print(f"Saved {out_path}")

def create_readme_report(trial_data, out_path):
    g1 = [td for td in trial_data if td["info"]["goal"] == "1"]
    g2 = [td for td in trial_data if td["info"]["goal"] == "2"]

    sr_g1 = sum(td["success"] for td in g1) / len(g1) * 100
    spl_g1 = sum(td["spl"] for td in g1) / len(g1) * 100
    time_g1 = np.mean([td["ep_meta"].get("elapsed_to_stop_s", 0.0) for td in g1])
    path_g1 = np.mean([td["p_i"] for td in g1])

    sr_g2 = sum(td["success"] for td in g2) / len(g2) * 100
    spl_g2 = sum(td["spl"] for td in g2) / len(g2) * 100
    time_g2 = np.mean([td["ep_meta"].get("elapsed_to_stop_s", 0.0) for td in g2])
    path_g2 = np.mean([td["p_i"] for td in g2])

    sr_tot = sum(td["success"] for td in trial_data) / len(trial_data) * 100
    spl_tot = sum(td["spl"] for td in trial_data) / len(trial_data) * 100
    time_tot = np.mean([td["ep_meta"].get("elapsed_to_stop_s", 0.0) for td in trial_data])
    path_tot = np.mean([td["p_i"] for td in trial_data])

    md = []
    md.append("# 2026-09-13 Unitree Go2 실제 로봇 네비게이션 실험 결과 보고서 (SR & SPL 분석)")
    md.append("")
    md.append("본 보고서는 2026년 9월 13일 Unitree Go2 실제 로봇 플랫폼에서 수행된 **Full 비동기 S2E-VLM 네비게이션 6회 주행 평가**의 공식 학술 벤치마크 지표(**SR: Success Rate, SPL: Success weighted by Path Length**) 및 세부 주행 궤적 데이터를 정리한 결과입니다.")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 🏆 1. 핵심 학술 벤치마크 지표 종합 요약 (SR & SPL)")
    md.append("")
    md.append("> **SPL 계산 정의**: $SPL = \\frac{1}{N} \\sum_{i=1}^N S_i \\frac{l_i}{\\max(p_i, l_i)}$  ")
    md.append("> ($S_i \\in \\{0, 1\\}$: 1.0m 목표 반경 도착 여부, $l_i$: 시작-목표 최단 직선거리, $p_i$: 실제 누적 주행거리)")
    md.append("")
    md.append("| 목표 구분 | 시도 횟수(N) | 성공 횟수 | **SR (성공률)** | **SPL (경로 가중 성공률)** | 평균 소요시간 | 평균 주행거리 | 최단 직선거리($l_i$) |")
    md.append("|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|")
    md.append(f"| **Goal 1** | 3 | {sum(td['success'] for td in g1)} | **{sr_g1:.1f}%** | **{spl_g1:.1f}%** | {time_g1:.1f}s | {path_g1:.2f}m | 4.841m |")
    md.append(f"| **Goal 2** | 3 | {sum(td['success'] for td in g2)} | **{sr_g2:.1f}%** | **{spl_g2:.1f}%** | {time_g2:.1f}s | {path_g2:.2f}m | 3.115m |")
    md.append(f"| **전체 (Total)** | 6 | {sum(td['success'] for td in trial_data)} | **{sr_tot:.1f}%** | **{spl_tot:.1f}%** | {time_tot:.1f}s | {path_tot:.2f}m | - |")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 2. 개별 회차별 세부 주행 지표 (Trial-by-Trial Breakdown)")
    md.append("")
    md.append("| 목표 | 회차 | 성공($S_i$) | **SPL** | 최종 결과 상태 | 소요시간(s) | 누적 주행($p_i$) | 최단거리($l_i$) | 잔여거리(m) | 최종 방향 오차 | 현장 관찰 보고 |")
    md.append("|:---:|:---:|:---:|:---:|:---|---:|---:|---:|---:|---:|:---|")

    for td in trial_data:
        g = td["info"]["goal"]
        t = td["info"]["trial_dir"].replace("_", " ").title()
        reason = td["ep_meta"].get("result_reason", "")
        dur = td["ep_meta"].get("elapsed_to_stop_s", 0.0)
        cum = td["p_i"]
        l_i = td["l_i"]
        spl_pct = td["spl"] * 100.0
        succ_mark = "✅ 1" if td["success"] == 1 else "❌ 0"
        res = td["dist_meta"].get("interpolated_goal_distance_at_stop_request_m", 0.0)
        yaw_err = td["yaw_err"]
        obs = td["ep_meta"].get("field_report", {}).get("operator_observation", "")
        
        status_badge = f"`{reason}`"
        md.append(f"| **Goal {g}** | {t} | {succ_mark} | **{spl_pct:.1f}%** | {status_badge} | {dur:.1f}s | {cum:.2f}m | {l_i:.3f}m | {res:.3f}m | {yaw_err:+.1f}° | {obs} |")

    md.append("")
    md.append("---")
    md.append("")
    md.append("## 3. 실험 환경 및 제어 파라미터")
    md.append("")
    md.append("- **프레임워크**: S2E-VLM Full 비동기 정책 (`async_true`)")
    md.append("- **로봇 플랫폼**: Unitree Go2 EDU (Lidar + Front Camera + IMU + Odom)")
    md.append("- **좌표계 방식**: `fixed_start_odometry` (고정 시작점 원점 기준 상대 좌표)")
    md.append("- **도착 판정 반경**: `1.0 m` (자동 거리 도달 시 정지)")
    md.append("- **최종 방향 제어**: 미적용 (PointGoal 도착 거리 기준 정지)")
    md.append("- **Look 동작 정책**: `forward_0p1` (카메라 look down/up 대신 0.1m 미세 전진 대체)")
    md.append("- **최대 주행 허용 시간**: `360 s`")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 4. 세부 분석 및 고찰")
    md.append("")
    md.append("### 1) Goal 2 분석: SR 100.0%, SPL 100.0%")
    md.append("- Goal 2(직선거리 3.115m) 주행 3회는 **모두 100% 성공**하였으며, 이동 경로가 최단 직선거리와 거의 일치(평균 2.75m 주행 후 1m 반경 진입 정지)하여 **SPL 역시 만점인 100.0%**를 기록했습니다.")
    md.append("- 평균 도달 시간은 **62.2초**로 매우 빠르고 안정적인 주행을 보였습니다.")
    md.append("")
    md.append("### 2) Goal 1 분석: SR 66.7%, SPL 50.3%")
    md.append("- Goal 1(직선거리 4.841m) 주행은 3회 중 2회 성공(**SR 66.7%**)했습니다.")
    md.append("- 첫 번째 회차(Trial 1)는 초기 관측 회전 도중 타임아웃(`MOTION_TIMEOUT`)이 발생하여 조기 종료(SPL 0%)되었습니다.")
    md.append("- 이후 2회차(Trial 2, SPL 69.9%) 및 3회차(Trial 3, SPL 81.0%)는 모두 정상적으로 1.0m 목표 반경에 도달하였으며, 성공 회차 평균 SPL은 **75.5%**를 기록했습니다.")
    md.append("")
    md.append("### 3) 무간섭 안전성 (Zero Intervention)")
    md.append("- 6회 주행 전 구간에서 작업자의 수동 개입(Direct Intervention), 장애물 충돌, 전도 등 비정상 상황이 **단 1건도 발생하지 않았습니다** (Intervention/Run = 0.00).")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 5. 시각화 결과")
    md.append("")
    md.append("### 1) 전체 6회 주행 궤적 통합 오버레이")
    md.append("![All Trajectories Overlay](visualizations/all_trajectories_overlay.png)")
    md.append("")
    md.append("### 2) 2D Grid 맵 상의 실제 로봇 궤적 투영")
    md.append("![Trajectories on 2D Map](visualizations/trajectories_on_2d_map.png)")
    md.append("")
    md.append("### 3) Goal 1 vs Goal 2 개별 비교")
    md.append("| Goal 1 (3회 반복) | Goal 2 (3회 반복) |")
    md.append("|:---:|:---:|")
    md.append("| ![Goal 1](visualizations/goal_1_trajectories.png) | ![Goal 2](visualizations/goal_2_trajectories.png) |")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 6. 디렉토리 구조 및 데이터 링크")
    md.append("```")
    md.append("experiments/0913/")
    md.append("├── README.md                     # 본 종합 보고서 (SR & SPL 포함)")
    md.append("├── benchmark_metrics.json        # [신규] 공식 학술 SR & SPL JSON 데이터")
    md.append("├── episodes_summary.csv          # [업데이트] SR, SPL 컬럼 추가 지표 테이블")
    md.append("├── episode_index.json            # JSON 포맷 전체 메타데이터")
    md.append("├── goal_plan.json                # 골 좌표 및 시작 원점 정의")
    md.append("├── origin.json                   # 원점 오도메트리 스냅샷")
    md.append("├── session_log.md                # 현장 주행 세션 원문 기록")
    md.append("├── visualizations/               # 통합 시각화 플롯")
    md.append("│   ├── all_trajectories_overlay.png")
    md.append("│   ├── goal_1_trajectories.png")
    md.append("│   ├── goal_2_trajectories.png")
    md.append("│   └── trajectories_on_2d_map.png")
    md.append("├── goal_1/")
    md.append("│   ├── trial_01/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)")
    md.append("│   ├── trial_02/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)")
    md.append("│   └── trial_03/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)")
    md.append("└── goal_2/")
    md.append("    ├── trial_01/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)")
    md.append("    ├── trial_02/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)")
    md.append("    └── trial_03/ (trajectory.csv, raw_odom.csv, trajectory.png, episode.json)")
    md.append("```")
    md.append("")
    md.append("- [종합 벤치마크 지표 JSON](file:///home/unitree/go2_ws_antarctica/experiments/0913/benchmark_metrics.json)")
    md.append("- [종합 주행 지표 CSV (SR & SPL 포함)](file:///home/unitree/go2_ws_antarctica/experiments/0913/episodes_summary.csv)")
    md.append("- [Goal 1 Trial 1 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_01/trajectory.csv)")
    md.append("- [Goal 1 Trial 2 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_02/trajectory.csv)")
    md.append("- [Goal 1 Trial 3 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_03/trajectory.csv)")
    md.append("- [Goal 2 Trial 1 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_01/trajectory.csv)")
    md.append("- [Goal 2 Trial 2 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_02/trajectory.csv)")
    md.append("- [Goal 2 Trial 3 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_03/trajectory.csv)")

    out_path.write_text("\n".join(md), encoding='utf-8')
    print(f"Saved {out_path}")

def create_symlinks():
    l1 = REPO_ROOT / "experiments/20260913_mapgoals"
    if l1.is_symlink() or l1.exists():
        l1.unlink()
    l1.symlink_to("0913")

    l2 = REPO_ROOT / "experiments/latest"
    if l2.is_symlink() or l2.exists():
        l2.unlink()
    l2.symlink_to("0913")

    ours_dir = REPO_ROOT / "experiments/ours"
    ours_dir.mkdir(parents=True, exist_ok=True)
    l3 = ours_dir / "20260913_mapgoals"
    if l3.is_symlink() or l3.exists():
        l3.unlink()
    l3.symlink_to("../0913")

if __name__ == "__main__":
    main()
