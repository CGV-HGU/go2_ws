import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s2e_vlm_core.vlm_schema import VlmAction, encode_vlm_reasoning, parse_vlm_reasoning


def valid_payload(action="go"):
    payload = {
        "schema_version": 0,
        "stamp": {"sec": 7, "nanosec": 500_000_000},
        "frame_id": "camera",
        "action": action,
        "goal_uv": {"u": 640.0, "v": 360.0},
        "rotate_deg": 0.0,
        "pose": {
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        },
        "reasoning": "clear path",
    }
    if action == "rotate":
        payload["rotate_deg"] = 30.0
    return json.dumps(payload)


class VlmSchemaTest(unittest.TestCase):
    def test_valid_go_payload_parses_goal_and_pose(self):
        result = parse_vlm_reasoning(valid_payload("go"))

        self.assertTrue(result.valid)
        self.assertEqual(result.action, VlmAction.GO)
        self.assertEqual(result.stamp, 7.5)
        self.assertEqual(result.goal_uv, (640.0, 360.0))
        assert result.pose is not None
        self.assertEqual(result.pose.child_frame_id, "base_link")

    def test_valid_stop_payload_does_not_require_goal(self):
        payload = json.loads(valid_payload("stop"))
        payload.pop("goal_uv")

        result = parse_vlm_reasoning(json.dumps(payload))

        self.assertTrue(result.valid)
        self.assertEqual(result.action, VlmAction.STOP)

    def test_valid_rotate_payload_requires_nonzero_angle(self):
        result = parse_vlm_reasoning(valid_payload("rotate"))

        self.assertTrue(result.valid)
        self.assertEqual(result.action, VlmAction.ROTATE)
        self.assertEqual(result.rotate_deg, 30.0)

    def test_malformed_json_returns_no_command(self):
        result = parse_vlm_reasoning("{not-json")

        self.assertFalse(result.valid)
        self.assertEqual(result.action, VlmAction.NO_COMMAND)
        self.assertEqual(result.reason, "MALFORMED_JSON")

    def test_go_missing_goal_returns_no_command(self):
        payload = json.loads(valid_payload("go"))
        payload.pop("goal_uv")

        result = parse_vlm_reasoning(json.dumps(payload))

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "MISSING_GOAL_UV")

    def test_unknown_action_and_schema_mismatch_are_rejected(self):
        unknown = json.loads(valid_payload("go"))
        unknown["action"] = "dance"
        mismatch = json.loads(valid_payload("go"))
        mismatch["schema_version"] = 1

        self.assertEqual(parse_vlm_reasoning(json.dumps(unknown)).reason, "INVALID_ACTION")
        self.assertEqual(parse_vlm_reasoning(json.dumps(mismatch)).reason, "UNSUPPORTED_SCHEMA")

    def test_non_finite_numeric_values_are_rejected(self):
        payload = json.loads(valid_payload("go"))
        payload["goal_uv"]["u"] = float("nan")
        payload["pose"]["x"] = float("inf")

        result = parse_vlm_reasoning(json.dumps(payload))

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "NON_FINITE_NUMERIC_FIELD")

    def test_json_nan_and_infinity_constants_are_rejected(self):
        payload = valid_payload("rotate").replace("30.0", "Infinity", 1)

        result = parse_vlm_reasoning(payload)

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "NON_FINITE_NUMERIC_FIELD")

    def test_encoder_outputs_parseable_strict_json(self):
        encoded = encode_vlm_reasoning(
            stamp=1.25,
            action=VlmAction.GO,
            goal_uv=(100.0, 200.0),
            pose_frame="odom",
            pose_child_frame="base_link",
            pose_xy_yaw=(0.0, 0.0, 0.0),
            reasoning="mock",
        )

        self.assertEqual(parse_vlm_reasoning(encoded).action, VlmAction.GO)


if __name__ == "__main__":
    unittest.main()
