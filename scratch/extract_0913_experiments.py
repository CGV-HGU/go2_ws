#!/usr/bin/env python3
"""
Extract 2026-09-13 real-robot navigation experiment data into go2_ws_antarctica.
Extracts 3 trials for Goal 1 and 3 trials for Goal 2.
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
    {"src": "goal-1-001", "goal": "1", "goal_dir": "goal_1", "trial_dir": "trial_01", "label": "Goal 1 - Trial 1"},
    {"src": "goal-1-002", "goal": "1", "goal_dir": "goal_1", "trial_dir": "trial_02", "label": "Goal 1 - Trial 2"},
    {"src": "goal-1-003", "goal": "1", "goal_dir": "goal_1", "trial_dir": "trial_03", "label": "Goal 1 - Trial 3"},
    {"src": "goal-2-001", "goal": "2", "goal_dir": "goal_2", "trial_dir": "trial_01", "label": "Goal 2 - Trial 1"},
    {"src": "goal-2-002", "goal": "2", "goal_dir": "goal_2", "trial_dir": "trial_02", "label": "Goal 2 - Trial 2"},
    {"src": "goal-2-003", "goal": "2", "goal_dir": "goal_2", "trial_dir": "trial_03", "label": "Goal 2 - Trial 3"},
]

def main():
    print(f"Creating directory structure in {DEST_BASE}...")
    DEST_BASE.mkdir(parents=True, exist_ok=True)
    vis_dir = DEST_BASE / "visualizations"
    vis_dir.mkdir(parents=True, exist_ok=True)

    # Top-level meta files
    shutil.copy2(SRC_BASE / "episodes.csv", DEST_BASE / "episodes_summary.csv")
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

        # Load data for plotting
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

        trial_data.append({
            "info": item,
            "rows": rows,
            "ep_meta": ep_meta,
            "dist_meta": dist_meta
        })
        print(f"Copied and loaded {src_name} -> {item['goal_dir']}/{item['trial_dir']}")

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
        reason = td["ep_meta"].get("result_reason", "")
        status_str = "REACHED" if "GOAL_DISTANCE_REACHED" in reason else "TIMEOUT"
        
        fwd = [r["forward"] for r in td["rows"]]
        lft = [r["left"] for r in td["rows"]]
        t_sec = [r["time"] for r in td["rows"]]
        dist = [r["goal_dist"] for r in td["rows"]]
        cum = [r["cum_path"] for r in td["rows"]]

        ax_map.plot(fwd, lft, color=color, lw=2.2, label=f'{t_label} ({status_str}, {cum[-1]:.2f}m)')
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

        t_lbl = f'{td["info"]["label"]} ({td["rows"][-1]["cum_path"]:.1f}m, {td["rows"][-1]["time"]:.0f}s)'
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

        ax.plot(px_list, py_list, color=c, lw=2.5, alpha=0.9, label=f'{td["info"]["label"]}')
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
    md = []
    md.append("# 2026-09-13 Unitree Go2 실제 로봇 네비게이션 실험 결과 보고서")
    md.append("")
    md.append("본 디렉토리는 2026년 9월 13일 Go2 실제 로봇 플랫폼에서 진행된 **Full 비동기 VLM (S2E-VLM) 네비게이션 반복 주행 실험 데이터**를 체계적으로 정리하여 보관합니다.")
    md.append("동일한 고정 시작점(Fixed-Start Origin)에서 **1번 목표(Goal 1)** 및 **2번 목표(Goal 2)**에 대해 각각 3회씩 총 6회 주행을 수행하였으며, 원본 오도메트리 궤적(10Hz 보간 및 고주파 원본), 실행 메타데이터, 개별/통합 시각화 자료를 포함합니다.")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 1. 실험 환경 및 제어 파라미터")
    md.append("")
    md.append("- **프레임워크**: S2E-VLM Full 비동기 정책 (`async_true`)")
    md.append("- **로봇 플랫폼**: Unitree Go2 EDU (Lidar + Front Camera + IMU + Odom)")
    md.append("- **좌표계 방식**: `fixed_start_odometry` (고정 시작점 원점 기준 상대 좌표)")
    md.append("- **도착 판정 반경**: `1.0 m` (자동 거리 도달 시 정지)")
    md.append("- **최종 방향 제어**: 미적용 (PointGoal 도착 거리 기준 정지)")
    md.append("- **Look 동작 정책**: `forward_0p1` (카메라 look down/up 대신 0.1m 미세 전진 대체)")
    md.append("- **최대 주행 허용 시간**: `360 s`")
    md.append("- **목표 위치 (시작점 기준)**:")
    md.append("  - **Goal 1**: 전방 `+3.968 m`, 좌측 `-2.772 m` (직선 거리 `4.841 m`)")
    md.append("  - **Goal 2**: 전방 `+2.849 m`, 좌측 `+1.258 m` (직선 거리 `3.115 m`)")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 2. 6회 주행 종합 결과 요약")
    md.append("")
    md.append("| 목표 | 회차 | 최종 결과 | 소요 시간(s) | 누적 이동거리(m) | 직선 변위(m) | 최종 목표 잔여거리(m) | 최종 방향 오차(deg) | 현장 관찰 보고 (Field Report) |")
    md.append("|:---:|:---:|:---|---:|---:|---:|---:|---:|:---|")

    for td in trial_data:
        g = td["info"]["goal"]
        t = td["info"]["trial_dir"].replace("_", " ").title()
        reason = td["ep_meta"].get("result_reason", "")
        dur = td["ep_meta"].get("elapsed_to_stop_s", 0.0)
        cum = td["ep_meta"].get("path_length", {}).get("10hz_m", 0.0)
        disp = td["dist_meta"].get("net_displacement_at_stop_request_m", 0.0)
        res = td["dist_meta"].get("interpolated_goal_distance_at_stop_request_m", 0.0)
        yaw_err = td["ep_meta"].get("final_yaw_error_vs_saved_deg", 0.0)
        obs = td["ep_meta"].get("field_report", {}).get("operator_observation", "")
        
        status_badge = f"`{reason}`"
        md.append(f"| **Goal {g}** | {t} | {status_badge} | {dur:.1f}s | {cum:.2f}m | {disp:.2f}m | {res:.3f}m | {yaw_err:+.1f}° | {obs} |")

    md.append("")
    md.append("---")
    md.append("")
    md.append("## 3. 세부 분석 및 고찰")
    md.append("")
    md.append("### Goal 1 주행 분석 (직선 거리 4.84m)")
    md.append("- **Trial 1 (`goal-1-001`)**: 시작 직후 초기 관측 회전(Initial observation rotation) 중 시간 초과(`MOTION_TIMEOUT`)가 발생하여 정책 결정 전에 종료되었습니다 (이동거리 0.56m).")
    md.append("- **Trial 2 (`goal-1-002`)**: 성공적으로 1m 도착 반경에 진입하여 자동 정지(`GOAL_DISTANCE_REACHED`, 잔여 0.999m). 소요 시간 259.9s, 누적 이동거리 6.92m. 현장 보고상 골포즈 촬영 시점과 반대 방향(~172도 차이)으로 정지하였으나 PointGoal 거리 조건 충족.")
    md.append("- **Trial 3 (`goal-1-003`)**: 안정적으로 1m 도착 반경에 진입(`GOAL_DISTANCE_REACHED`, 잔여 1.000m). 소요 시간 195.3s, 누적 이동거리 5.98m.")
    md.append("")
    md.append("### Goal 2 주행 분석 (직선 거리 3.11m)")
    md.append("- **Trial 1 (`goal-2-001`)**: 안정적 도달 (`GOAL_DISTANCE_REACHED`, 잔여 0.988m). 소요 시간 88.9s, 누적 이동거리 3.10m.")
    md.append("- **Trial 2 (`goal-2-002`)**: 신속 도달 (`GOAL_DISTANCE_REACHED`, 잔여 0.961m). 소요 시간 47.5s, 누적 이동거리 2.55m.")
    md.append("- **Trial 3 (`goal-2-003`)**: 신속 도달 (`GOAL_DISTANCE_REACHED`, 잔여 0.958m). 소요 시간 50.3s, 누적 이동거리 2.61m.")
    md.append("- **Goal 2 종합**: **3회 전회(3/3) 성공 (SR 100%)**, 평균 소요시간 **62.2초**, 평균 주행거리 **2.75m**로 매우 우수한 재현성을 보였습니다.")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 4. 시각화 결과")
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
    md.append("## 5. 디렉토리 구조 및 데이터 링크")
    md.append("```")
    md.append("experiments/0913/")
    md.append("├── README.md                     # 본 종합 보고서")
    md.append("├── episodes_summary.csv          # 6회 주행 지표 종합 테이블")
    md.append("├── episode_index.json            # JSON 포맷 상세 지표")
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
    md.append("- [Goal 1 Trial 1 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_01/trajectory.csv)")
    md.append("- [Goal 1 Trial 2 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_02/trajectory.csv)")
    md.append("- [Goal 1 Trial 3 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_1/trial_03/trajectory.csv)")
    md.append("- [Goal 2 Trial 1 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_01/trajectory.csv)")
    md.append("- [Goal 2 Trial 2 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_02/trajectory.csv)")
    md.append("- [Goal 2 Trial 3 궤적 데이터](file:///home/unitree/go2_ws_antarctica/experiments/0913/goal_2/trial_03/trajectory.csv)")

    out_path.write_text("\n".join(md), encoding='utf-8')
    print(f"Saved {out_path}")

def create_symlinks():
    # experiments/20260913_mapgoals -> 0913
    l1 = REPO_ROOT / "experiments/20260913_mapgoals"
    if l1.is_symlink() or l1.exists():
        l1.unlink()
    l1.symlink_to("0913")

    # experiments/latest -> 0913
    l2 = REPO_ROOT / "experiments/latest"
    if l2.is_symlink() or l2.exists():
        l2.unlink()
    l2.symlink_to("0913")

    # experiments/ours/20260913_mapgoals -> ../0913
    ours_dir = REPO_ROOT / "experiments/ours"
    ours_dir.mkdir(parents=True, exist_ok=True)
    l3 = ours_dir / "20260913_mapgoals"
    if l3.is_symlink() or l3.exists():
        l3.unlink()
    l3.symlink_to("../0913")
    print("Created symlinks: experiments/latest, experiments/20260913_mapgoals, experiments/ours/20260913_mapgoals")

if __name__ == "__main__":
    main()
