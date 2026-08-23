#!/usr/bin/env python3
"""
========================================================================================
🏆 ICRA ESCAPE-Nav Real-Robot Table Evaluator (tab:real_robot_quantitative)
========================================================================================
Matches exact IEEE ICRA Paper Protocol (table_real_robot.tex & results_template.csv):
  1. SR       : Success Rate (within 1.0m goal radius, with Wilson 95% CI)
  2. Intv.    : Human intervention count / run
  3. Time     : Normalized completion time T^dagger (mean ± std)
  4. Rec.     : Recovery success ratio (successful / triggered count)
  5. Lat. (s) : Mean end-to-end VLM latency
  6. Duty     : Motion duty cycle (active movement duration / wall-clock time)
  7. Yield    : Application yield (applied decisions / completed decisions)
  8. Exports  : Formatted LaTeX table row & results_template.csv row
========================================================================================
"""

import os
import csv
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class Go2TableEpisode:
    episode_id: str
    scenario_name: str        # e.g., "Pair1_Newton_To_Oseok", "Pair2_Corner_To_Stairs", etc.
    method_name: str          # "Direct-goal" or "ESCAPE-Nav (Ours)"
    success: bool             # Reached goal within 1.0m
    interventions: int        # Count of human / safety interventions (0 = clean)
    shortest_path_m: float    # Geodesic distance (l_i)
    actual_path_length_m: float
    wall_time_s: float        # Total wall-clock time
    active_motion_time_s: float # Duration robot was actively executing motion
    timeout_s: float          # T_max (default: 60.0s)
    vlm_latencies_s: List[float] # VLM submit-to-receive latency trace
    completed_decisions: int  # Total VLM responses completed
    applied_decisions: int    # Decisions admitted and applied by Causal Pose Warping
    recoveries_triggered: int # Deadlocks / stall recovery events triggered
    recoveries_successful: int# Successful recovery escapes

class RealRobotPaperEvaluator:
    def __init__(self, default_timeout_s: float = 60.0):
        self.default_timeout_s = default_timeout_s

    def compute_t_dagger(self, ep: Go2TableEpisode) -> float:
        """T_i^dagger = S_i * min(T_i, T_max) + (1 - S_i) * T_max"""
        t_max = ep.timeout_s if ep.timeout_s > 0 else self.default_timeout_s
        if ep.success:
            return min(ep.wall_time_s, t_max)
        return t_max

    def wilson_score_interval(self, k: int, n: int, confidence: float = 0.95) -> Tuple[float, float, float]:
        if n == 0:
            return 0.0, 0.0, 0.0
        p = k / n
        z = 1.95996  # 95% two-sided normal quantile
        denom = 1 + z**2 / n
        centre = p + z**2 / (2 * n)
        adj_err = z * math.sqrt((p * (1 - p) + z**2 / (4 * n)) / n)
        return p * 100.0, max(0.0, (centre - adj_err) / denom) * 100.0, min(100.0, (centre + adj_err) / denom) * 100.0

    def evaluate_method(self, episodes: List[Go2TableEpisode], method_name: str):
        eps = [e for e in episodes if e.method_name == method_name]
        n = len(eps)
        if n == 0:
            return None

        # 1. SR
        succ_cnt = sum(1 for e in eps if e.success)
        sr, sr_low, sr_high = self.wilson_score_interval(succ_cnt, n)

        # 2. Intv
        mean_intv = np.mean([e.interventions for e in eps])

        # 3. Time (T^dagger)
        t_daggers = [self.compute_t_dagger(e) for e in eps]
        mean_time = np.mean(t_daggers)
        std_time = np.std(t_daggers)

        # 4. Rec
        total_rec_trig = sum(e.recoveries_triggered for e in eps)
        total_rec_succ = sum(e.recoveries_successful for e in eps)
        rec_str = f"{total_rec_succ}/{total_rec_trig}" if total_rec_trig > 0 else "--"

        # 5. Latency
        all_lats = [lat for e in eps for lat in e.vlm_latencies_s]
        mean_lat = np.mean(all_lats) if len(all_lats) > 0 else 0.0

        # 6. Duty (Active motion ratio)
        duty_ratios = [e.active_motion_time_s / max(0.001, e.wall_time_s) for e in eps]
        mean_duty = np.mean(duty_ratios) * 100.0

        # 7. Yield (Applied / Completed VLM decisions)
        total_comp = sum(e.completed_decisions for e in eps)
        total_appl = sum(e.applied_decisions for e in eps)
        yield_pct = (total_appl / total_comp * 100.0) if total_comp > 0 else 0.0

        return {
            "method": method_name,
            "N": n,
            "SR": sr,
            "SR_CI": (sr_low, sr_high),
            "Intv": mean_intv,
            "Time": mean_time,
            "Time_std": std_time,
            "Rec": rec_str,
            "Lat": mean_lat,
            "Duty": mean_duty,
            "Yield": yield_pct,
            "raw_episodes": eps
        }

    def print_latex_table(self, direct_res, escape_res):
        print("=" * 80)
        print(" 📄 [ICRA Table tab:real_robot_quantitative] LaTeX Export")
        print("=" * 80)
        print("\\begin{table}[t]")
        print("  \\centering")
        print("  \\caption{Go2 paired navigation in one fixed map ($5P$ trials/method). Time: $T^\\dagger$; Intv.: interventions/run; Rec.: successful/triggered recovery; Lat.: mean VLM latency.}")
        print("  \\label{tab:real_robot_quantitative}")
        print("  \\scriptsize")
        print("  \\setlength{\\tabcolsep}{1.35pt}")
        print("  \\begin{tabular}{@{}lccccccc@{}}")
        print("    \\toprule")
        print("    Method & SR $\\uparrow$ & Intv. $\\downarrow$ & Time $\\downarrow$ & Rec. $\\uparrow$ & Lat. (s) $\\downarrow$ & Duty $\\uparrow$ & Yield $\\uparrow$ \\\\")
        print("    \\midrule")
        if direct_res:
            print(f"    Direct-goal & {direct_res['SR']:.1f} & {direct_res['Intv']:.2f} & {direct_res['Time']:.1f} & {direct_res['Rec']} & {direct_res['Lat']:.2f} & {direct_res['Duty']:.1f}\\% & -- \\\\")
        if escape_res:
            print(f"    \\textbf{{\\method}} & \\textbf{{{escape_res['SR']:.1f}}} & \\textbf{{{escape_res['Intv']:.2f}}} & \\textbf{{{escape_res['Time']:.1f}}} & \\textbf{{{escape_res['Rec']}}} & \\textbf{{{escape_res['Lat']:.2f}}} & \\textbf{{{escape_res['Duty']:.1f}\\%}} & \\textbf{{{escape_res['Yield']:.1f}\\%}} \\\\")
        print("    \\bottomrule")
        print("  \\end{tabular}")
        print("\\end{table}")
        print("=" * 80)

def main():
    evaluator = RealRobotPaperEvaluator(default_timeout_s=60.0)
    
    # Example / Standby dataset with 5 paired runs for Newton-Oseok corridor
    sample_episodes = [
        # Direct-goal baseline (Stops during VLM inference, naive stale execution)
        Go2TableEpisode("ep1", "Newton_To_Oseok_P1", "Direct-goal", True, 0, 24.5, 26.2, 48.5, 22.1, 60.0, [2.1, 2.3, 1.9], 15, 15, 0, 0),
        Go2TableEpisode("ep2", "Newton_To_Oseok_P2", "Direct-goal", False, 1, 31.0, 28.0, 60.0, 24.0, 60.0, [2.2, 2.5], 14, 14, 2, 0),
        # ESCAPE-Nav (Ours: 50Hz Causal Pose Warping + Continuous Motion)
        Go2TableEpisode("ep3", "Newton_To_Oseok_P1", "ESCAPE-Nav (Ours)", True, 0, 24.5, 25.1, 31.2, 28.5, 60.0, [0.21, 0.22, 0.19], 42, 41, 1, 1),
        Go2TableEpisode("ep4", "Newton_To_Oseok_P2", "ESCAPE-Nav (Ours)", True, 0, 31.0, 32.2, 38.4, 35.0, 60.0, [0.20, 0.21], 48, 47, 1, 1),
    ]

    direct_res = evaluator.evaluate_method(sample_episodes, "Direct-goal")
    escape_res = evaluator.evaluate_method(sample_episodes, "ESCAPE-Nav (Ours)")
    evaluator.print_latex_table(direct_res, escape_res)

if __name__ == "__main__":
    main()
