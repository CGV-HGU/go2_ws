import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s2e_vlm_core.mock_algorithms import (
    MockE2EPlanner,
    MockOdometryEstimator,
    MockVlmReasoner,
    base_link_ground_point_to_image,
    camera_image_to_base_link_ground,
    image_goal_to_base_link,
    resample_path_to_ten_points,
)
from s2e_vlm_core.pose_buffer import Pose2D
from s2e_vlm_core.vlm_schema import VlmAction, parse_vlm_reasoning


class MockAlgorithmsTest(unittest.TestCase):
    def test_vlm_go_stop_rotate_and_malformed_scenarios_are_deterministic(self):
        reasoner = MockVlmReasoner(["go", "stop", "rotate", "malformed"])

        actions = []
        for _ in range(4):
            payload = reasoner.reason(stamp=1.0, pose=Pose2D(0.0, 0.0, 0.0))
            parsed = parse_vlm_reasoning(payload)
            actions.append(parsed.action)

        self.assertEqual(actions, [VlmAction.GO, VlmAction.STOP, VlmAction.ROTATE, VlmAction.NO_COMMAND])

    def test_image_goal_to_base_link_is_bounded_and_config_driven(self):
        center = image_goal_to_base_link((320.0, 240.0), image_size=(640, 480), max_forward_m=4.0, max_lateral_m=2.0)
        left = image_goal_to_base_link((0.0, 240.0), image_size=(640, 480), max_forward_m=4.0, max_lateral_m=2.0)
        right = image_goal_to_base_link((640.0, 240.0), image_size=(640, 480), max_forward_m=4.0, max_lateral_m=2.0)
        top = image_goal_to_base_link((320.0, 0.0), image_size=(640, 480), max_forward_m=4.0, max_lateral_m=2.0)

        self.assertEqual(center, (2.5, 0.0))
        self.assertEqual(left, (2.5, 2.0))
        self.assertEqual(right, (2.5, -2.0))
        self.assertEqual(top, (4.0, 0.0))

    def test_camera_image_to_base_link_ground_round_trips_through_intrinsics_and_extrinsics(self):
        camera_matrix = (640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0)
        base_from_camera_translation = (0.25, 0.0, 0.35)
        base_from_camera_rotation = (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)
        goal_uv = (410.0, 330.0)

        ground_point = camera_image_to_base_link_ground(
            goal_uv,
            camera_matrix_row_major=camera_matrix,
            base_from_camera_translation_m=base_from_camera_translation,
            base_from_camera_rotation_matrix_row_major=base_from_camera_rotation,
        )

        self.assertIsNotNone(ground_point)
        assert ground_point is not None
        self.assertGreater(ground_point[0], 0.0)
        round_trip_uv = base_link_ground_point_to_image(
            (ground_point[0], ground_point[1], 0.0),
            camera_matrix_row_major=camera_matrix,
            base_from_camera_translation_m=base_from_camera_translation,
            base_from_camera_rotation_matrix_row_major=base_from_camera_rotation,
        )
        self.assertIsNotNone(round_trip_uv)
        assert round_trip_uv is not None
        self.assertAlmostEqual(round_trip_uv[0], goal_uv[0], delta=1.0)
        self.assertAlmostEqual(round_trip_uv[1], goal_uv[1], delta=1.0)

    def test_camera_image_to_base_link_ground_rejects_above_horizon_pixels(self):
        camera_matrix = (640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0)
        base_from_camera_translation = (0.25, 0.0, 0.35)
        base_from_camera_rotation = (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)

        ground_point = camera_image_to_base_link_ground(
            (320.0, 120.0),
            camera_matrix_row_major=camera_matrix,
            base_from_camera_translation_m=base_from_camera_translation,
            base_from_camera_rotation_matrix_row_major=base_from_camera_rotation,
        )

        self.assertIsNone(ground_point)

    def test_camera_image_to_base_link_ground_rejects_unbounded_near_horizon_goals(self):
        camera_matrix = (640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0)
        base_from_camera_translation = (0.25, 0.0, 0.35)
        base_from_camera_rotation = (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)

        ground_point = camera_image_to_base_link_ground(
            (320.0, 242.0),
            camera_matrix_row_major=camera_matrix,
            base_from_camera_translation_m=base_from_camera_translation,
            base_from_camera_rotation_matrix_row_major=base_from_camera_rotation,
            max_ground_distance_m=8.0,
        )

        self.assertIsNone(ground_point)

    def test_base_link_projection_rejects_points_behind_camera(self):
        camera_matrix = (640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0)
        base_from_camera_translation = (0.25, 0.0, 0.35)
        base_from_camera_rotation = (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)

        uv = base_link_ground_point_to_image(
            (-1.0, 0.0, 0.0),
            camera_matrix_row_major=camera_matrix,
            base_from_camera_translation_m=base_from_camera_translation,
            base_from_camera_rotation_matrix_row_major=base_from_camera_rotation,
        )

        self.assertIsNone(uv)

    def test_vlm_go_goal_moves_smoothly_within_640_by_480_image(self):
        reasoner = MockVlmReasoner(["go", "go", "go", "go"])

        goals = []
        for stamp in [0.0, 0.5, 1.0, 1.5]:
            parsed = parse_vlm_reasoning(reasoner.reason(stamp=stamp, pose=Pose2D(0.0, 0.0, 0.0)))
            self.assertIsNotNone(parsed.goal_uv)
            goals.append(parsed.goal_uv)

        self.assertGreater(len(set(goals)), 1)
        for u, v in goals:
            self.assertGreaterEqual(u, 0.0)
            self.assertLess(u, 640.0)
            self.assertGreaterEqual(v, 0.0)
            self.assertLess(v, 480.0)
        for before, after in zip(goals, goals[1:]):
            self.assertLess(math.dist(before, after), 80.0)

    def test_vlm_smooth_goal_is_relative_to_first_stamp(self):
        early = MockVlmReasoner(["go", "go"])
        late = MockVlmReasoner(["go", "go"])

        early_goals = [parse_vlm_reasoning(early.reason(stamp, Pose2D(0.0, 0.0, 0.0))).goal_uv for stamp in (0.0, 0.5)]
        late_goals = [parse_vlm_reasoning(late.reason(stamp, Pose2D(0.0, 0.0, 0.0))).goal_uv for stamp in (1_234_567.0, 1_234_567.5)]

        self.assertEqual(early_goals, late_goals)

    def test_vlm_smooth_goal_stays_projectable_for_visualizer_run(self):
        camera_matrix = (640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0)
        base_from_camera_translation = (0.25, 0.0, 0.35)
        base_from_camera_rotation = (0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0)
        reasoner = MockVlmReasoner(["go"] * 25)

        for index in range(25):
            parsed = parse_vlm_reasoning(reasoner.reason(stamp=index * 0.5, pose=Pose2D(0.0, 0.0, 0.0)))
            assert parsed.goal_uv is not None
            ground_point = camera_image_to_base_link_ground(
                parsed.goal_uv,
                camera_matrix_row_major=camera_matrix,
                base_from_camera_translation_m=base_from_camera_translation,
                base_from_camera_rotation_matrix_row_major=base_from_camera_rotation,
                max_ground_distance_m=8.0,
            )
            self.assertIsNotNone(ground_point, msg=f"goal_uv={parsed.goal_uv} at index={index} should project to bounded ground")

    def test_e2e_planner_returns_exactly_ten_finite_points(self):
        planner = MockE2EPlanner()
        trajectory = planner.plan(goal_point_base_link=(4.0, 1.0), current_pose=Pose2D(0.0, 0.0, 0.0))

        self.assertEqual(len(trajectory.points), 10)
        self.assertTrue(all(math.isfinite(x) and math.isfinite(y) for x, y in trajectory.points))
        self.assertEqual(trajectory.goal_point_base_link, (4.0, 1.0))

    def test_reference_six_by_two_output_is_resampled_to_ten_points(self):
        six_points = [(float(i), float(i * 2)) for i in range(6)]

        ten_points = resample_path_to_ten_points(six_points)

        self.assertEqual(len(ten_points), 10)
        self.assertEqual(ten_points[0], six_points[0])
        self.assertEqual(ten_points[-1], six_points[-1])

    def test_mock_odometry_estimator_outputs_base_link_pose(self):
        estimator = MockOdometryEstimator()
        pose = estimator.estimate(stamp=2.0)

        self.assertEqual(pose.frame_id, "odom")
        self.assertEqual(pose.child_frame_id, "base_link")
        self.assertGreater(pose.pose.x, 0.0)


if __name__ == "__main__":
    unittest.main()
