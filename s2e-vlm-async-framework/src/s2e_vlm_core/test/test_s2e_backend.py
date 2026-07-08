import math
import sys
import unittest
from pathlib import Path

# pyright: reportMissingImports=false

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s2e_vlm_core.s2e_backend import OnnxS2ENavigator, S2EFrameContext, S2EPlanner, convert_s2e_output_to_points, ros_rgb8_to_chw_float


class S2EBackendTest(unittest.TestCase):
    def test_ros_rgb8_bytes_convert_to_chw_float_256(self):
        data = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255])

        frame = ros_rgb8_to_chw_float(data, width=2, height=2)

        self.assertEqual(frame.shape, (3, 256, 256))
        self.assertEqual(frame.dtype, np.float32)
        self.assertGreaterEqual(float(frame.min()), 0.0)
        self.assertLessEqual(float(frame.max()), 1.0)
        self.assertAlmostEqual(float(frame[0, 0, 0]), 1.0, places=5)

    def test_frame_context_waits_until_eleven_frames_then_returns_batched_stack(self):
        context = S2EFrameContext(context_size=11)
        frame = np.ones((3, 256, 256), dtype=np.float32)

        for index in range(10):
            context.append(frame * (index / 10.0))
            self.assertIsNone(context.batch())

        context.append(frame)
        batch = context.batch()

        self.assertIsNotNone(batch)
        assert batch is not None
        self.assertEqual(batch.shape, (1, 11, 3, 256, 256))
        self.assertEqual(batch.dtype, np.float32)
        self.assertAlmostEqual(float(batch[0, -1, 0, 0, 0]), 1.0, places=5)

    def test_s2e_output_converts_to_exactly_ten_finite_points(self):
        output = np.zeros((1, 1, 10, 2), dtype=np.float32)
        output[0, 0, :, 0] = np.linspace(0.1, 1.0, 10)
        output[0, 0, :, 1] = np.linspace(-0.5, 0.5, 10)

        points = convert_s2e_output_to_points(output)

        self.assertEqual(len(points), 10)
        self.assertTrue(all(math.isfinite(x) and math.isfinite(y) for x, y in points))
        self.assertEqual(points[0], (0.10000000149011612, -0.5))
        self.assertEqual(points[-1], (1.0, 0.5))

    def test_s2e_output_rejects_bad_shape_and_nonfinite_values(self):
        with self.assertRaises(ValueError):
            convert_s2e_output_to_points(np.zeros((1, 10, 2), dtype=np.float32))

        bad = np.zeros((1, 1, 10, 2), dtype=np.float32)
        bad[0, 0, 2, 0] = np.nan
        with self.assertRaises(ValueError):
            convert_s2e_output_to_points(bad)

    def test_onnx_navigator_runs_session_with_obs_and_goal_without_torch_wrapper(self):
        class FakeInput:
            def __init__(self, name):
                self.name = name

        class FakeSession:
            def __init__(self):
                self.inputs = [FakeInput("obs_images"), FakeInput("goal")]
                self.feed = None

            def get_inputs(self):
                return self.inputs

            def run(self, _output_names, feed):
                self.feed = feed
                trajectory = np.zeros((1, 10, 3), dtype=np.float32)
                trajectory[:, :, 0] = 4.0
                scores = np.ones((1, 1), dtype=np.float32)
                return [trajectory, scores]

        session = FakeSession()
        navigator = OnnxS2ENavigator(session=session)
        obs = np.ones((1, 11, 3, 256, 256), dtype=np.float32)
        goal = np.array([5.0, 0.0], dtype=np.float32)

        trajectory, scores = navigator.inference_trajectory(obs, goal_xy=goal)

        self.assertEqual(trajectory.shape, (1, 1, 10, 2))
        self.assertEqual(scores.shape, (1, 1))
        assert session.feed is not None
        np.testing.assert_array_equal(session.feed["obs_images"], obs)
        np.testing.assert_allclose(session.feed["goal"], np.array([[0.025, 1.0, 0.0]], dtype=np.float32))
        self.assertAlmostEqual(float(trajectory[0, 0, 0, 0]), 1.0, places=5)

    def test_s2e_planner_uses_onnx_navigator_by_default(self):
        class FakeNavigator:
            def __init__(self):
                self.called = False

            def inference_trajectory(self, obs, goal_xy):
                self.called = True
                self.obs_shape = obs.shape
                self.goal_xy = goal_xy
                return np.zeros((1, 1, 10, 2), dtype=np.float32), np.ones((1, 1), dtype=np.float32)

        navigator = FakeNavigator()
        planner = S2EPlanner("/models/s2e/S2E", navigator=navigator)

        plan = planner.plan(np.ones((1, 11, 3, 256, 256), dtype=np.float32), (5.0, 0.0), None)

        self.assertTrue(navigator.called)
        self.assertEqual(navigator.obs_shape, (1, 11, 3, 256, 256))
        np.testing.assert_array_equal(navigator.goal_xy, np.array([5.0, 0.0], dtype=np.float32))
        self.assertEqual(len(plan.points), 10)
        self.assertEqual(plan.status, "S2E_FRESH")


if __name__ == "__main__":
    unittest.main()
