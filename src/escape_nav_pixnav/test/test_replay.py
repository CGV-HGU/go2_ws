import ast
import json
from pathlib import Path

import pytest

from escape_nav_pixnav.audit_sink import verify_audit_chain
from escape_nav_pixnav.replay import replay_report


CHECKPOINT_HASH = "0b1faff7631962351bbbfe8cb115a3a03069f33fab499865f887ffbb5a3cabe3"


def report():
    probabilities = {
        "stop": 0.01,
        "forward": 0.90,
        "turn_left": 0.03,
        "turn_right": 0.02,
        "look_up": 0.02,
        "look_down": 0.02,
    }
    return {
        "schema_version": "go2_pixnav_file_only_v2",
        "run_id": "fixture",
        "overall": "PASS_FILE_ONLY_REPLAY",
        "inference_executed": True,
        "published": False,
        "actuation_calls": 0,
        "checkpoint_sha256_actual": CHECKPOINT_HASH,
        "checkpoint_sha256_expected": CHECKPOINT_HASH,
        "reference_commit_actual": "6341a5d33903131ddfce74498c04e1c0ae04ec61",
        "reference_commit_expected": "6341a5d33903131ddfce74498c04e1c0ae04ec61",
        "goal_frame": {"index": 10, "path": "/not/read.jpg", "sha256": "1" * 64},
        "input_contract": {
            "history_start_index": 10,
            "history_rule": "observations_must_be_at_or_after_goal_capture",
        },
        "frames": [{"index": 10, "path": "/not/read.jpg", "sha256": "1" * 64}],
        "predictions": [
            {
                "frame_index": 10,
                "action_id": 1,
                "action": "forward",
                "action_probabilities": probabilities,
                "finite": True,
            }
        ],
    }


def test_report_replay_writes_verified_file_only_evidence(tmp_path):
    source = tmp_path / "report.json"
    source.write_text(json.dumps(report()), encoding="utf-8")

    run_dir = replay_report(source, tmp_path / "output")
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    verification = verify_audit_chain(run_dir / "macro_actions.jsonl")

    assert summary["overall"] == "PASS_FILE_ONLY_MACRO_REPLAY"
    assert summary["accepted_proposal_count"] == 1
    assert summary["actuation_calls"] == 0
    assert summary["actuation_permitted_count"] == 0
    assert verification["record_count"] == 1


@pytest.mark.parametrize(
    "mutate",
    [
        lambda value: value.update(overall="BLOCKED"),
        lambda value: value.update(inference_executed=False),
        lambda value: value.update(published=True),
        lambda value: value.update(actuation_calls=1),
        lambda value: value.update(frames=[]),
        lambda value: value.update(schema_version="go2_pixnav_file_only_v1"),
        lambda value: value.update(checkpoint_sha256_actual="2" * 64),
        lambda value: value["input_contract"].update(history_start_index=9),
    ],
)
def test_replay_rejects_unqualified_source_report(tmp_path, mutate):
    raw = report()
    mutate(raw)
    source = tmp_path / "report.json"
    source.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(ValueError):
        replay_report(source, tmp_path / "output")


def test_runtime_modules_do_not_import_robot_or_transport_apis():
    package_dir = Path(__file__).resolve().parents[1] / "escape_nav_pixnav"
    banned_roots = {"rclpy", "socket", "unitree_sdk2", "geometry_msgs"}
    imported_roots = set()
    for source in package_dir.glob("*.py"):
        tree = ast.parse(source.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_roots.add(node.module.split(".")[0])

    assert not imported_roots.intersection(banned_roots)
