import hashlib
import json
from pathlib import Path

import pytest

from escape_nav_pixnav.causal_chain import validate_offline_chain
from escape_nav_pixnav.contracts import PIXNAV_CHECKPOINT_A_SHA256, sha256_canonical
from escape_nav_pixnav.replay import replay_report


REFERENCE_COMMIT = "6341a5d33903131ddfce74498c04e1c0ae04ec61"


def sha256_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def rebuild_vlm_manifest(vlm_dir):
    files = sorted(
        path
        for path in vlm_dir.rglob("*")
        if path.is_file() and path.name != "SHA256SUMS"
    )
    (vlm_dir / "SHA256SUMS").write_text(
        "".join(
            f"{sha256_file(path)}  {path.relative_to(vlm_dir)}\n"
            for path in files
        ),
        encoding="utf-8",
    )


def fixture_chain(tmp_path):
    vlm_dir = tmp_path / "vlm"
    frames_dir = vlm_dir / "frames"
    frames_dir.mkdir(parents=True)
    frame_path = frames_dir / "frame_00.jpg"
    frame_path.write_bytes(b"recorded-real-frame-fixture")
    frame_hash = sha256_file(frame_path)
    frames = [
        {
            "index": 0,
            "file": str(frame_path),
            "sha256_file": frame_hash,
        }
    ]
    vlm_input = {
        "observation": {
            "frame_index": 0,
            "image_width": 1280,
            "image_height": 720,
            "views": [{"view_id": 0, "image": str(frame_path)}],
        },
        "constraints": {
            "physical_actuation_allowed": False,
            "output_sink": "FILE_ONLY_AUDIT",
        },
    }
    sanitized = {
        "schema_version": "nav_vlm_waypoint_v1",
        "action": "go",
        "selected_view_id": 0,
        "selected_image_point": [640, 600],
        "fine_goal": {"point_px": [640, 600]},
    }
    write_json(vlm_dir / "report.json", {
        "physical_actuation_allowed": False,
        "ros_publishers_created": False,
        "udp_command_senders_created": False,
        "stages": {"live_vlm_schema": "DEGRADED_SANITIZED"},
    })
    write_json(vlm_dir / "frames.json", frames)
    write_json(vlm_dir / "vlm_input.json", vlm_input)
    write_json(vlm_dir / "vlm_raw.json", {"action": "go"})
    write_json(vlm_dir / "vlm_sanitized.json", sanitized)
    write_json(vlm_dir / "vlm_runtime.json", {"model": "fixture", "latency_s": 0.1})
    write_json(vlm_dir / "command_audit.json", {
        "published": False,
        "linear_x_mps": 0.0,
        "angular_z_radps": 0.0,
    })
    rebuild_vlm_manifest(vlm_dir)

    pixnav_path = tmp_path / "pixnav_report.json"
    probabilities = {
        "stop": 0.01,
        "forward": 0.90,
        "turn_left": 0.03,
        "turn_right": 0.02,
        "look_up": 0.02,
        "look_down": 0.02,
    }
    write_json(pixnav_path, {
        "schema_version": "go2_pixnav_file_only_v2",
        "run_id": "fixture",
        "overall": "PASS_FILE_ONLY_REPLAY",
        "inference_executed": True,
        "published": False,
        "actuation_calls": 0,
        "checkpoint_sha256_actual": PIXNAV_CHECKPOINT_A_SHA256,
        "checkpoint_sha256_expected": PIXNAV_CHECKPOINT_A_SHA256,
        "reference_commit_actual": REFERENCE_COMMIT,
        "reference_commit_expected": REFERENCE_COMMIT,
        "goal_pixel": {"u": 640, "v": 600},
        "goal_frame": {"index": 0, "path": str(frame_path), "sha256": frame_hash},
        "source_frames": [{"index": 0, "path": str(frame_path), "sha256": frame_hash}],
        "frames": [{"index": 0, "path": str(frame_path), "sha256": frame_hash}],
        "input_contract": {
            "history_rule": "observations_must_be_at_or_after_goal_capture",
            "history_start_index": 0,
        },
        "predictions": [{
            "frame_index": 0,
            "action_id": 1,
            "action": "forward",
            "action_probabilities": probabilities,
            "finite": True,
        }],
    })
    macro_dir = replay_report(pixnav_path, tmp_path / "macro")
    return vlm_dir, pixnav_path, macro_dir


def test_valid_offline_chain_links_all_artifacts(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)

    result = validate_offline_chain(vlm_dir, pixnav_path, macro_dir)

    assert result["overall"] == "PASS_OFFLINE_CAUSAL_CHAIN_SANITIZED_VLM"
    assert len(result["causal_identity_sha256"]) == 64
    assert result["published"] is False
    assert result["actuation_calls"] == 0


def test_vlm_pixel_mismatch_is_blocked_even_with_updated_manifest(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    path = vlm_dir / "vlm_sanitized.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    value["selected_image_point"] = [641, 600]
    value["fine_goal"]["point_px"] = [641, 600]
    write_json(path, value)
    rebuild_vlm_manifest(vlm_dir)

    with pytest.raises(ValueError, match="goal-u mismatch"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)


def test_vlm_actuation_permission_is_blocked(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    path = vlm_dir / "report.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    value["physical_actuation_allowed"] = True
    write_json(path, value)
    rebuild_vlm_manifest(vlm_dir)

    with pytest.raises(ValueError, match="actuation interlock"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)


def test_pixnav_history_before_capture_is_blocked(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    value = json.loads(pixnav_path.read_text(encoding="utf-8"))
    value["input_contract"]["history_start_index"] = -1
    write_json(pixnav_path, value)

    with pytest.raises(ValueError, match="history predates capture"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)


def test_wrong_pixnav_checkpoint_is_blocked(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    value = json.loads(pixnav_path.read_text(encoding="utf-8"))
    value["checkpoint_sha256_actual"] = "2" * 64
    write_json(pixnav_path, value)

    with pytest.raises(ValueError, match="Checkpoint_A hash mismatch"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)


def test_macro_source_report_hash_mismatch_is_blocked(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    path = macro_dir / "summary.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    value["source_report_sha256"] = "3" * 64
    write_json(path, value)

    with pytest.raises(ValueError, match="macro source report hash mismatch"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)


def test_hash_valid_macro_record_with_actuation_true_is_still_blocked(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    path = macro_dir / "macro_actions.jsonl"
    record = json.loads(path.read_text(encoding="utf-8"))
    record["proposal"]["actuation_permitted"] = True
    body = dict(record)
    body.pop("record_sha256")
    record["record_sha256"] = sha256_canonical(body)
    path.write_text(json.dumps(record) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="actuation interlock"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)


def test_vlm_artifact_byte_tamper_is_blocked_by_manifest(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    (vlm_dir / "vlm_raw.json").write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError, match="artifact hash mismatch"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)


def test_required_vlm_artifact_omitted_from_hash_manifest_is_blocked(tmp_path):
    vlm_dir, pixnav_path, macro_dir = fixture_chain(tmp_path)
    manifest = vlm_dir / "SHA256SUMS"
    lines = [
        line for line in manifest.read_text(encoding="utf-8").splitlines()
        if not line.endswith("  vlm_runtime.json")
    ]
    manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="absent from SHA256SUMS"):
        validate_offline_chain(vlm_dir, pixnav_path, macro_dir)
