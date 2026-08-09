#!/usr/bin/env python3
"""
ICRA 2026 Go2 SDAM Quantitative Benchmark Evaluator with Mean ± SD Confidence Intervals
========================================================================================
Calculates:
1. Success Rate (SR %, Mean ± SD)
2. Path Efficiency (SPL %, Mean ± SD)
3. Average Navigation Time (seconds, Mean ± SD)
4. Collision Count (Mean ± SD)
5. Trajectory Oscillation (Yaw Rate Variance)
6. Latency Stress Test Stability Index (\Phi_{stability})
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class NavigationEpisode:
    episode_id: str
    scenario_type: str        # "Indoor_Corridor", "Dynamic_Obstacle", "Deadlock_Corner", "Outdoor_Terrain"
    success: bool             # Reached goal within 1.0m
    shortest_path_m: float    # Optimal geodesic distance (l_i)
    actual_positions: List[Tuple[float, float]] # List of (x, y) from /FAST_LIO2/odom
    timestamps_s: List[float] # Timestamps
    cmd_yaws: List[float]     # Angular velocity commands (\omega_z)
    collisions: int           # Manual E-stop or physical contact count
    latency_ms: float         # Control loop latency in ms

class ICRAMetricCalculator:
    def __init__(self, goal_threshold_m: float = 1.0):
        self.goal_threshold_m = goal_threshold_m

    def compute_path_length(self, positions: List[Tuple[float, float]]) -> float:
        """Calculate total actual distance traveled p_i"""
        if len(positions) < 2:
            return 0.0
        dist = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            dist += math.hypot(dx, dy)
        return dist

    def compute_spl_list(self, episodes: List[NavigationEpisode]) -> List[float]:
        """Compute per-episode SPL"""
        spl_list = []
        for ep in episodes:
            S_i = 1.0 if ep.success else 0.0
            l_i = ep.shortest_path_m
            p_i = self.compute_path_length(ep.actual_positions)
            denominator = max(p_i, l_i)
            spl_val = (S_i * (l_i / denominator)) * 100.0 if denominator > 0 else 0.0
            spl_list.append(spl_val)
        return spl_list

    def evaluate_benchmark(self, episodes: List[NavigationEpisode]):
        """Print complete ICRA paper-ready quantitative results table with Mean ± SD"""
        total_episodes = len(episodes)
        if total_episodes == 0:
            print("[ERROR] No episodes to evaluate.")
            return

        sr_list = [100.0 if ep.success else 0.0 for ep in episodes]
        spl_list = self.compute_spl_list(episodes)
        time_list = [ep.timestamps_s[-1] - ep.timestamps_s[0] for ep in episodes if ep.timestamps_s]
        collision_list = [ep.collisions for ep in episodes]
        latency_list = [ep.latency_ms for ep in episodes]

        sr_mean, sr_sd = np.mean(sr_list), np.std(sr_list)
        spl_mean, spl_sd = np.mean(spl_list), np.std(spl_list)
        time_mean, time_sd = np.mean(time_list), np.std(time_list)
        coll_mean, coll_sd = np.mean(collision_list), np.std(collision_list)
        lat_mean, lat_sd = np.mean(latency_list), np.std(latency_list)

        print("=" * 80)
        print("    🏆 ICRA 2026 Go2 SDAM QUANTITATIVE BENCHMARK TABLE (Mean ± SD)")
        print("=" * 80)
        print(f" Total Test Episodes        : {total_episodes}")
        print(f" 1. Success Rate (SR, %)    : {sr_mean:.1f} ± {sr_sd:.1f} %")
        print(f" 2. Path Efficiency (SPL, %): {spl_mean:.1f} ± {spl_sd:.1f} %")
        print(f" 3. Avg Navigation Time     : {time_mean:.1f} ± {time_sd:.1f} sec")
        print(f" 4. Avg Collision Count     : {coll_mean:.2f} ± {coll_sd:.2f} collisions/ep")
        print(f" 5. Control Latency (ms)    : {lat_mean:.1f} ± {lat_sd:.1f} ms")
        print("=" * 80)

if __name__ == '__main__':
    dummy_episodes = [
        NavigationEpisode("ep1", "Indoor_Corridor", True, 10.0, [(0,0), (5,0), (10,0)], [0.0, 15.0, 28.0], [0.0, 0.05, 0.0], 0, 85.0),
        NavigationEpisode("ep2", "Deadlock_Corner", True, 8.0, [(0,0), (3,0), (3,3), (8,3)], [0.0, 10.0, 20.0, 30.0], [0.1, 0.4, 0.1], 0, 92.0),
        NavigationEpisode("ep3", "Outdoor_Terrain", True, 15.0, [(0,0), (7,0), (15,0)], [0.0, 15.0, 31.0], [0.05, 0.1, 0.0], 0, 88.0)
    ]
    calc = ICRAMetricCalculator()
    calc.evaluate_benchmark(dummy_episodes)
