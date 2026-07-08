import importlib
import sys
import unittest
from pathlib import Path
from typing import Any, cast

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "s2e_vlm_core"))
sys.path.insert(0, str(ROOT / "s2e_vlm_nodes"))

dummy_integration = cast(Any, importlib.import_module("s2e_vlm_nodes.dummy_integration"))
DEFAULT_STATE_COVERAGE = dummy_integration.DEFAULT_STATE_COVERAGE
run_dummy_integration_trials = dummy_integration.run_dummy_integration_trials


class DummyIntegrationTest(unittest.TestCase):
    def test_trials_drive_every_documented_interface_with_async_periods(self):
        result = run_dummy_integration_trials(trial_count=9, duration_s=3.0)

        self.assertEqual(result.trial_count, 9)
        self.assertEqual(result.missing_publish_topics(), {})
        self.assertEqual(result.missing_subscribe_topics(), {})
        self.assertEqual(result.missing_action_interfaces(), {})
        self.assertGreater(result.tick_count("imu_node"), result.tick_count("camera_node"))
        self.assertGreater(result.tick_count("e2e_node"), result.tick_count("vlm_node"))
        self.assertGreater(result.tick_count("controller_node"), result.tick_count("supervisor_node"))

    def test_dummy_trials_cover_all_documented_state_machine_states(self):
        result = run_dummy_integration_trials(trial_count=9, duration_s=3.0)

        self.assertEqual(result.missing_states(DEFAULT_STATE_COVERAGE), {})


if __name__ == "__main__":
    unittest.main()
