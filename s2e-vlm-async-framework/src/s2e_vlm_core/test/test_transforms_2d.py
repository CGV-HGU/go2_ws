import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s2e_vlm_core.pose_buffer import Pose2D
from s2e_vlm_core.transforms_2d import (
    compensation_within_bounds,
    relative_pose_2d,
    transform_point_2d,
    wrap_angle,
)


class Transform2DTest(unittest.TestCase):
    def test_wrap_angle_normalizes_to_pi_range(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi), -math.pi)

    def test_relative_pose_expresses_current_pose_from_reference(self):
        reference = Pose2D(x=1.0, y=1.0, yaw=math.pi / 2)
        current = Pose2D(x=1.0, y=3.0, yaw=math.pi)

        relative = relative_pose_2d(reference, current)

        self.assertAlmostEqual(relative.x, 2.0)
        self.assertAlmostEqual(relative.y, 0.0, places=6)
        self.assertAlmostEqual(relative.yaw, math.pi / 2)

    def test_transform_point_moves_point_between_pose_frames(self):
        relative = Pose2D(x=1.0, y=0.0, yaw=math.pi / 2)

        x, y = transform_point_2d((2.0, 0.0), relative)

        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, -1.0)

    def test_compensation_bounds_reject_large_translation_or_yaw(self):
        self.assertTrue(compensation_within_bounds(Pose2D(1.0, 0.0, 0.1), 1.5, math.radians(30)))
        self.assertFalse(compensation_within_bounds(Pose2D(2.0, 0.0, 0.1), 1.5, math.radians(30)))
        self.assertFalse(compensation_within_bounds(Pose2D(1.0, 0.0, math.radians(45)), 1.5, math.radians(30)))


if __name__ == "__main__":
    unittest.main()
