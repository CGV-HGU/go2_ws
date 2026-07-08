import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s2e_vlm_core.pose_buffer import Pose2D, PoseBuffer, PoseSample


class PoseBufferTest(unittest.TestCase):
    def test_lookup_returns_exact_timestamp_with_zero_age(self):
        buffer = PoseBuffer(max_samples=5)
        pose = Pose2D(x=1.0, y=2.0, yaw=0.3)
        buffer.add(PoseSample(stamp=10.0, pose=pose, frame_id="odom", child_frame_id="base_link"))

        result = buffer.lookup_latest_before(target_stamp=10.0, max_age=0.2)

        self.assertTrue(result.found)
        self.assertEqual(result.pose, pose)
        self.assertEqual(result.age, 0.0)
        self.assertEqual(result.reason, "OK")

    def test_lookup_uses_latest_pose_before_target(self):
        buffer = PoseBuffer(max_samples=5)
        buffer.add(PoseSample(stamp=9.7, pose=Pose2D(1.0, 0.0, 0.0)))
        buffer.add(PoseSample(stamp=9.9, pose=Pose2D(2.0, 0.0, 0.0)))
        buffer.add(PoseSample(stamp=10.1, pose=Pose2D(3.0, 0.0, 0.0)))

        result = buffer.lookup_latest_before(target_stamp=10.0, max_age=0.2)

        self.assertTrue(result.found)
        assert result.pose is not None
        assert result.age is not None
        self.assertEqual(result.pose.x, 2.0)
        self.assertAlmostEqual(result.age, 0.1)

    def test_lookup_rejects_stale_pose(self):
        buffer = PoseBuffer(max_samples=5)
        buffer.add(PoseSample(stamp=9.0, pose=Pose2D(1.0, 0.0, 0.0)))

        result = buffer.lookup_latest_before(target_stamp=10.0, max_age=0.2)

        self.assertFalse(result.found)
        assert result.age is not None
        self.assertEqual(result.reason, "STALE")
        self.assertAlmostEqual(result.age, 1.0)

    def test_lookup_empty_buffer_reports_empty(self):
        result = PoseBuffer(max_samples=5).lookup_latest_before(target_stamp=10.0, max_age=0.2)

        self.assertFalse(result.found)
        self.assertEqual(result.reason, "EMPTY")

    def test_out_of_order_inserts_are_sorted_and_bounded(self):
        buffer = PoseBuffer(max_samples=3)
        for stamp in [3.0, 1.0, 4.0, 2.0]:
            buffer.add(PoseSample(stamp=stamp, pose=Pose2D(stamp, 0.0, 0.0)))

        self.assertEqual(buffer.stamps(), [2.0, 3.0, 4.0])


if __name__ == "__main__":
    unittest.main()
