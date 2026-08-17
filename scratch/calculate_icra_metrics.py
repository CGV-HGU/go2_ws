#!/usr/bin/env python3
"""
ICRA 2026 ESCAPE-Nav Table VIII Quantitative Evaluator
========================================================================================
Calculates:
1. Success Count (Succ./5) & Intervention-free Count (IF/5) with 95% Wilson CIs
2. Normalized Completion Time T^dagger (Time s, Mean ± SD)
3. Motion Duty Cycle (Duty)
4. Recovery Success Events (Rec. succ.) & Failed-Edge Re-entry Count (Re-entry)
5. Non-parametric Statistical Hypothesis Test (Mann-Whitney U-test p-value vs Direct-goal)
"""

import math
import numpy as np
from scipy import stats
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Go2TableVIIIEpisode:
    episode_id: str
    scenario_name: str        # "Dead_end_room", "Blocked_goal_direction", "Repeated_corridor", "Active_view_recovery", "Dynamic_obstacle"
    method_name: str          # "Direct_goal", "Full_ESCAPE_Nav"
    success: bool             # Reached goal within 1.0m
    intervention_free: bool   # Completed without manual E-stop / human intervention
    shortest_path_m: float    # Geodesic distance (l_i)
    actual_positions: List[Tuple[float, float]] # List of (x, y) from /rtabmap/odom
    timestamps_s: List[float] # Timestamps
    moving_duration_s: float  # Actual motion duration
    total_duration_s: float   # Total elapsed time
    timeout_s: float          # Max allowable time T_max (e.g. 60.0s)
    recovery_triggered: int   # Number of triggered recovery events
    recovery_success: int     # Number of successful escape events
    failed_edge_reentries: int# Number of re-entries into known failed branch

class ESCAPENavTableVIIICalculator:
    def __init__(self, default_timeout_s: float = 60.0):
        self.default_timeout_s = default_timeout_s

    def compute_t_dagger(self, ep: Go2TableVIIIEpisode) -> float:
        """Normalized completion time: T_i^dagger = S_i * min(T_i, T_max) + (1 - S_i) * T_max"""
        actual_time = ep.total_duration_s
        t_max = ep.timeout_s if ep.timeout_s > 0 else self.default_timeout_s
        if ep.success:
            return min(actual_time, t_max)
        else:
            return t_max

    def wilson_score_interval(self, k: int, n: int, confidence: float = 0.95) -> Tuple[float, float, float]:
        """Compute Wilson Score Interval for binomial metrics"""
        if n == 0:
            return 0.0, 0.0, 0.0
        p = k / n
        z = stats.norm.ppf(1 - (1 - confidence) / 2)
        denominator = 1 + z**2 / n
        centre_adjusted_probability = p + z**2 / (2 * n)
        adjusted_standard_error = z * math.sqrt((p * (1 - p) + z**2 / (4 * n)) / n)
        lower_bound = (centre_adjusted_probability - adjusted_standard_error) / denominator
        upper_bound = (centre_adjusted_probability + adjusted_standard_error) / denominator
        return p * 100.0, lower_bound * 100.0, upper_bound * 100.0

    def evaluate_scenario(self, scenario_name: str, direct_goal_eps: List[Go2TableVIIIEpisode], escape_nav_eps: List[Go2TableVIIIEpisode]):
        """Print official Table VIII row for a specific scenario"""
        print(f"\n[{scenario_name}]")
        print("-" * 95)
        print(f"{'Method':<20} | {'Succ./5':<10} | {'IF/5':<8} | {'Time (s) T^dag':<18} | {'Duty':<8} | {'Rec. succ.':<12} | {'Re-entry':<10}")
        print("-" * 95)

        for method_label, ep_list in [("Direct-goal", direct_goal_eps), ("Full ESCAPE-Nav", escape_nav_eps)]:
            if not ep_list:
                continue
            n = len(ep_list)
            succ_count = sum(1 for ep in ep_list if ep.success)
            if_count = sum(1 for ep in ep_list if ep.intervention_free)
            t_daggers = [self.compute_t_dagger(ep) for ep in ep_list]
            duties = [ep.moving_duration_s / max(ep.total_duration_s, 1e-3) for ep in ep_list]
            rec_succ = sum(ep.recovery_success for ep in ep_list)
            rec_trig = sum(ep.recovery_triggered for ep in ep_list)
            re_entries = sum(ep.failed_edge_reentries for ep in ep_list)

            time_mean, time_sd = np.mean(t_daggers), np.std(t_daggers)
            duty_mean = np.mean(duties)

            rec_str = f"{rec_succ}/{rec_trig}" if rec_trig > 0 else f"{rec_succ}"

            print(f"{method_label:<20} | {succ_count}/{n:<8} | {if_count}/{n:<6} | {time_mean:5.1f} ± {time_sd:4.1f} s    | {duty_mean:5.2f}    | {rec_str:<12} | {re_entries:<10}")

        if direct_goal_eps and escape_nav_eps:
            dg_times = [self.compute_t_dagger(ep) for ep in direct_goal_eps]
            esc_times = [self.compute_t_dagger(ep) for ep in escape_nav_eps]
            if len(dg_times) > 0 and len(esc_times) > 0:
                stat, p_val = stats.mannwhitneyu(esc_times, dg_times, alternative='less')
                print(f" -> Mann-Whitney U-test on T^dagger: U={stat:.1f}, p-value = {p_val:.4f} {'(p < 0.05 Sig.)' if p_val < 0.05 else ''}")
        print("-" * 95)

if __name__ == '__main__':
    calc = ESCAPENavTableVIIICalculator()
    print("=" * 95)
    print("     🏆 ICRA 2026 ESCAPE-Nav TABLE VIII REAL-ROBOT EVALUATION MATRIX")
    print("=" * 95)

    # Dummy test demo
    dg_demo = [
        Go2TableVIIIEpisode("dg1", "Dead_end_room", "Direct_goal", False, False, 10.0, [(0,0)], [0.0], 10.0, 60.0, 60.0, 0, 0, 3),
        Go2TableVIIIEpisode("dg2", "Dead_end_room", "Direct_goal", False, False, 10.0, [(0,0)], [0.0], 12.0, 60.0, 60.0, 0, 0, 2),
    ]
    esc_demo = [
        Go2TableVIIIEpisode("esc1", "Dead_end_room", "Full_ESCAPE_Nav", True, True, 10.0, [(0,0), (10,0)], [0.0, 24.0], 20.0, 24.0, 60.0, 1, 1, 0),
        Go2TableVIIIEpisode("esc2", "Dead_end_room", "Full_ESCAPE_Nav", True, True, 10.0, [(0,0), (10,0)], [0.0, 22.0], 19.0, 22.0, 60.0, 1, 1, 0),
    ]
    calc.evaluate_scenario("Dead-end room", dg_demo, esc_demo)
