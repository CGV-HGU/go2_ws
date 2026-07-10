import sys
import unittest
from pathlib import Path
from typing import Any, cast

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "s2e_vlm_core"))
sys.path.insert(0, str(ROOT / "s2e_vlm_nodes"))

from s2e_vlm_nodes.mock_graph import MockGraphSimulator


def graph_simulator(**kwargs: Any) -> Any:
    return cast(Any, MockGraphSimulator)(**kwargs)


class MockGraphIntegrationTest(unittest.TestCase):
    def test_e2e_reuses_one_vlm_reasoning_for_multiple_trajectories(self):
        graph = graph_simulator(vlm_period_s=2.0, e2e_period_s=0.2)

        graph.run_for(1.0)

        self.assertGreaterEqual(graph.count_topic("/s2e/e2e/trajectory"), 3)
        self.assertEqual(graph.count_topic("/s2e/vlm/reasoning"), 1)

    def test_supervisor_health_blocks_cached_vlm_trajectory_generation(self):
        graph = graph_simulator(vlm_period_s=2.0, e2e_period_s=0.2)
        graph.run_for(0.5)
        produced_before = graph.count_topic("/s2e/e2e/trajectory")

        graph.mark_vlm_missing()
        graph.run_for(0.6)

        self.assertEqual(graph.count_topic("/s2e/e2e/trajectory"), produced_before)
        self.assertEqual(graph.latest_e2e_status().active_mode, "SUPERVISOR_BLOCKED")

    def test_first_vlm_delay_blocks_trajectory_publication(self):
        graph = graph_simulator(vlm_period_s=2.0, e2e_period_s=0.2, first_vlm_delay_s=1.0)

        graph.run_for(0.8)

        self.assertEqual(graph.count_topic("/s2e/e2e/trajectory"), 0)
        self.assertEqual(graph.latest_e2e_status().active_mode, "WAITING_FIRST_VLM")

    def test_vlm_stop_publishes_motion_blocking_status_without_trajectory(self):
        graph = graph_simulator(vlm_scenarios=["stop"], vlm_period_s=2.0, e2e_period_s=0.2)

        graph.run_for(0.5)

        self.assertEqual(graph.count_topic("/s2e/e2e/trajectory"), 0)
        self.assertEqual(graph.latest_e2e_status().active_mode, "STOPPED_BY_VLM")

    def test_cached_vlm_expires_after_ttl(self):
        graph = graph_simulator(vlm_period_s=100.0, e2e_period_s=0.5, vlm_ttl_s=1.0)
        graph.run_for(0.5)
        produced_before = graph.count_topic("/s2e/e2e/trajectory")

        graph.run_for(1.5)

        self.assertEqual(graph.count_topic("/s2e/e2e/trajectory"), produced_before)
        self.assertEqual(graph.latest_e2e_status().active_mode, "VLM_STALE")

    def test_controller_stops_after_trajectory_ttl(self):
        graph = graph_simulator(vlm_period_s=100.0, e2e_period_s=0.2, trajectory_ttl_s=0.5)
        graph.run_for(0.3)
        self.assertEqual(graph.latest_controller_status().active_mode, "FOLLOWING")

        graph.disable_e2e()
        graph.run_for(0.7)

        self.assertEqual(graph.latest_controller_status().active_mode, "WAITING_TRAJECTORY")

    def test_controller_rejects_non_base_link_trajectory_frame(self):
        graph = graph_simulator()

        graph.inject_trajectory_with_frame("map")

        self.assertEqual(graph.latest_controller_status().active_mode, "FAULT")
        self.assertIsNone(graph.controller.current_trajectory)

    def test_rotate_preempts_following_and_clears_trajectory(self):
        graph = graph_simulator(vlm_period_s=10.0, e2e_period_s=0.2)
        graph.run_for(0.5)

        result = graph.send_rotate_goal(target_yaw_delta_deg=30.0, timeout_s=5.0)

        self.assertTrue(result.success)
        self.assertEqual(graph.latest_controller_status().active_mode, "WAITING_TRAJECTORY")
        self.assertIsNone(graph.controller.current_trajectory)

    def test_visualizer_renders_malformed_vlm_without_command_output(self):
        graph = graph_simulator(vlm_period_s=10.0, e2e_period_s=0.2)

        overlay = graph.render_visualizer_frame(vlm_payload="{bad-json")

        self.assertIn("INVALID_VLM", overlay.labels)
        self.assertEqual(graph.count_topic("/s2e/controller/command"), 0)


if __name__ == "__main__":
    unittest.main()
