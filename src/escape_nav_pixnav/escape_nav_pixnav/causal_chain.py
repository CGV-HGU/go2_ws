"""Offline provenance validation across VLM, PixNav and macro evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path
from typing import Any

from .audit_sink import verify_audit_chain
from .contracts import (
    PIXNAV_CHECKPOINT_A_SHA256,
    PIXNAV_REFERENCE_COMMIT,
    is_sha256,
    sha256_canonical,
)


DEFAULT_OUTPUT_ROOT = Path.home() / ".ros" / "pixnav_chain_runs"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {path}: {error}") from error


def verify_sha256s(run_dir: Path) -> dict[str, Any]:
    manifest = run_dir / "SHA256SUMS"
    if not manifest.is_file():
        raise ValueError("VLM run has no SHA256SUMS")
    checked = 0
    checked_paths: set[str] = set()
    for line_number, line in enumerate(manifest.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            expected, relative = line.split(None, 1)
        except ValueError as error:
            raise ValueError(f"invalid SHA256SUMS line {line_number}") from error
        relative = relative.strip().lstrip("*")
        if not is_sha256(expected):
            raise ValueError(f"invalid SHA-256 at line {line_number}")
        if relative in checked_paths:
            raise ValueError(f"duplicate SHA256SUMS path: {relative}")
        candidate = (run_dir / relative).resolve()
        try:
            candidate.relative_to(run_dir.resolve())
        except ValueError as error:
            raise ValueError(f"SHA256SUMS path escapes run directory: {relative}") from error
        if not candidate.is_file() or sha256_file(candidate) != expected:
            raise ValueError(f"VLM artifact hash mismatch: {relative}")
        checked_paths.add(relative)
        checked += 1
    if checked == 0:
        raise ValueError("VLM SHA256SUMS is empty")
    return {
        "valid": True,
        "checked_file_count": checked,
        "checked_paths": sorted(checked_paths),
    }


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def validate_offline_chain(
    vlm_run_dir: Path,
    pixnav_report_path: Path,
    macro_run_dir: Path,
) -> dict[str, Any]:
    vlm_dir = vlm_run_dir.expanduser().resolve()
    pixnav_path = pixnav_report_path.expanduser().resolve()
    macro_dir = macro_run_dir.expanduser().resolve()
    required_vlm = {
        name: vlm_dir / name
        for name in (
            "report.json",
            "frames.json",
            "vlm_input.json",
            "vlm_raw.json",
            "vlm_sanitized.json",
            "vlm_runtime.json",
            "command_audit.json",
        )
    }
    for name, path in required_vlm.items():
        _require(path.is_file(), f"missing VLM artifact: {name}")
    _require(pixnav_path.is_file(), "missing PixNav report")
    macro_summary_path = macro_dir / "summary.json"
    macro_audit_path = macro_dir / "macro_actions.jsonl"
    _require(macro_summary_path.is_file(), "missing macro summary")
    _require(macro_audit_path.is_file(), "missing macro audit")

    vlm_hashes = verify_sha256s(vlm_dir)
    missing_hash_entries = sorted(set(required_vlm) - set(vlm_hashes["checked_paths"]))
    _require(not missing_hash_entries, f"VLM artifacts absent from SHA256SUMS: {missing_hash_entries}")
    vlm_report = load_json(required_vlm["report.json"])
    frames = load_json(required_vlm["frames.json"])
    vlm_input = load_json(required_vlm["vlm_input.json"])
    vlm_sanitized = load_json(required_vlm["vlm_sanitized.json"])
    vlm_runtime = load_json(required_vlm["vlm_runtime.json"])
    command_audit = load_json(required_vlm["command_audit.json"])
    pixnav = load_json(pixnav_path)
    macro_summary = load_json(macro_summary_path)
    macro_audit = verify_audit_chain(macro_audit_path)

    _require(vlm_report.get("physical_actuation_allowed") is False, "VLM actuation interlock not false")
    _require(vlm_report.get("ros_publishers_created") is False, "VLM ROS publisher was created")
    _require(vlm_report.get("udp_command_senders_created") is False, "VLM UDP sender was created")
    _require(command_audit.get("published") is False, "legacy command audit published")
    _require(float(command_audit.get("linear_x_mps", 1.0)) == 0.0, "legacy linear command nonzero")
    _require(float(command_audit.get("angular_z_radps", 1.0)) == 0.0, "legacy angular command nonzero")
    constraints = vlm_input.get("constraints", {})
    _require(constraints.get("physical_actuation_allowed") is False, "VLM input allowed actuation")
    _require(constraints.get("output_sink") == "FILE_ONLY_AUDIT", "VLM input sink is not file-only")

    _require(isinstance(frames, list) and frames, "VLM frame manifest is empty")
    for expected_index, frame in enumerate(frames):
        _require(int(frame.get("index", -1)) == expected_index, "VLM frame indices are not contiguous")
        frame_path = Path(str(frame.get("file", ""))).resolve()
        _require(frame_path.is_file(), f"missing recorded frame {expected_index}")
        _require(sha256_file(frame_path) == frame.get("sha256_file"), f"frame hash mismatch {expected_index}")

    observation = vlm_input.get("observation", {})
    selected_frame_index = int(observation.get("frame_index", -1))
    _require(0 <= selected_frame_index < len(frames), "VLM selected frame index out of range")
    views = observation.get("views", [])
    _require(isinstance(views, list) and len(views) == 1, "expected one front VLM view")
    _require(int(views[0].get("view_id", -1)) == 0, "VLM selected view ID mismatch")
    selected_frame = frames[selected_frame_index]
    selected_frame_path = Path(str(selected_frame["file"])).resolve()
    _require(Path(str(views[0].get("image", ""))).resolve() == selected_frame_path, "VLM view/frame path mismatch")

    _require(vlm_sanitized.get("schema_version") == "nav_vlm_waypoint_v1", "VLM schema mismatch")
    _require(vlm_sanitized.get("action") == "go", "VLM action is not go")
    _require(int(vlm_sanitized.get("selected_view_id", -1)) == 0, "sanitized view ID mismatch")
    point = vlm_sanitized.get("selected_image_point")
    fine_goal = vlm_sanitized.get("fine_goal", {})
    _require(isinstance(point, list) and len(point) == 2, "sanitized pixel missing")
    _require(fine_goal.get("point_px") == point, "sanitized fine/top-level pixel mismatch")
    width = int(observation.get("image_width", 0))
    height = int(observation.get("image_height", 0))
    _require(0 <= int(point[0]) < width and 0 <= int(point[1]) < height, "sanitized pixel out of image")

    _require(pixnav.get("schema_version") == "go2_pixnav_file_only_v2", "PixNav report is not capture-contract v2")
    _require(pixnav.get("overall") == "PASS_FILE_ONLY_REPLAY", "PixNav replay did not pass")
    _require(pixnav.get("inference_executed") is True, "PixNav inference was not executed")
    _require(pixnav.get("published") is False, "PixNav report published")
    _require(int(pixnav.get("actuation_calls", -1)) == 0, "PixNav actuation call count is nonzero")
    _require(
        pixnav.get("checkpoint_sha256_actual") == PIXNAV_CHECKPOINT_A_SHA256
        and pixnav.get("checkpoint_sha256_expected") == PIXNAV_CHECKPOINT_A_SHA256,
        "PixNav Checkpoint_A hash mismatch",
    )
    _require(
        pixnav.get("reference_commit_actual") == PIXNAV_REFERENCE_COMMIT
        and pixnav.get("reference_commit_expected") == PIXNAV_REFERENCE_COMMIT,
        "PixNav reference commit mismatch",
    )
    _require(pixnav.get("goal_pixel", {}).get("u") == int(point[0]), "VLM/PixNav goal-u mismatch")
    _require(pixnav.get("goal_pixel", {}).get("v") == int(point[1]), "VLM/PixNav goal-v mismatch")
    goal_frame = pixnav.get("goal_frame", {})
    _require(int(goal_frame.get("index", -1)) == selected_frame_index, "VLM/PixNav capture frame index mismatch")
    _require(goal_frame.get("sha256") == selected_frame.get("sha256_file"), "VLM/PixNav capture frame hash mismatch")
    _require(Path(str(goal_frame.get("path", ""))).resolve() == selected_frame_path, "VLM/PixNav capture path mismatch")
    source_frames = pixnav.get("source_frames", [])
    _require(len(source_frames) == len(frames), "VLM/PixNav source frame count mismatch")
    for expected, actual in zip(frames, source_frames):
        _require(expected.get("sha256_file") == actual.get("sha256"), "VLM/PixNav frame hash sequence mismatch")
    contract = pixnav.get("input_contract", {})
    _require(contract.get("history_rule") == "observations_must_be_at_or_after_goal_capture", "PixNav history rule mismatch")
    _require(int(contract.get("history_start_index", -1)) >= selected_frame_index, "PixNav history predates capture")

    pixnav_hash = sha256_file(pixnav_path)
    _require(macro_summary.get("overall") == "PASS_FILE_ONLY_MACRO_REPLAY", "macro replay did not pass")
    _require(Path(str(macro_summary.get("source_report", ""))).resolve() == pixnav_path, "macro source report path mismatch")
    _require(macro_summary.get("source_report_sha256") == pixnav_hash, "macro source report hash mismatch")
    _require(macro_summary.get("source_checkpoint_sha256") == pixnav.get("checkpoint_sha256_actual"), "macro checkpoint mismatch")
    _require(macro_summary.get("published") is False, "macro summary published")
    _require(int(macro_summary.get("actuation_calls", -1)) == 0, "macro actuation calls nonzero")
    _require(int(macro_summary.get("actuation_permitted_count", -1)) == 0, "macro actuation permitted")
    _require(macro_audit["record_count"] == len(pixnav.get("predictions", [])), "macro/prediction count mismatch")

    nodes = {
        "vlm_input": sha256_file(required_vlm["vlm_input.json"]),
        "vlm_raw": sha256_file(required_vlm["vlm_raw.json"]),
        "vlm_sanitized": sha256_file(required_vlm["vlm_sanitized.json"]),
        "capture_frame": str(selected_frame["sha256_file"]),
        "pixnav_report": pixnav_hash,
        "pixnav_checkpoint": str(pixnav["checkpoint_sha256_actual"]),
        "macro_summary": sha256_file(macro_summary_path),
        "macro_audit_tail": str(macro_audit["last_record_sha256"]),
    }
    edges = [
        ["vlm_input", "vlm_raw"],
        ["vlm_raw", "vlm_sanitized"],
        ["capture_frame", "vlm_sanitized"],
        ["vlm_sanitized", "pixnav_report"],
        ["pixnav_checkpoint", "pixnav_report"],
        ["pixnav_report", "macro_summary"],
        ["macro_summary", "macro_audit_tail"],
    ]
    causal_identity = sha256_canonical({"nodes": nodes, "edges": edges})
    return {
        "schema_version": "go2_pixnav_offline_causal_chain_v1",
        "overall": "PASS_OFFLINE_CAUSAL_CHAIN_SANITIZED_VLM",
        "causal_identity_sha256": causal_identity,
        "nodes": nodes,
        "edges": edges,
        "vlm": {
            "run_dir": str(vlm_dir),
            "artifact_hashes": vlm_hashes,
            "contract_status": vlm_report.get("stages", {}).get("live_vlm_schema"),
            "model": vlm_runtime.get("model"),
            "latency_s": vlm_runtime.get("latency_s"),
            "selected_frame_index": selected_frame_index,
            "selected_pixel": [int(point[0]), int(point[1])],
        },
        "pixnav": {
            "report": str(pixnav_path),
            "goal_frame_index": int(goal_frame["index"]),
            "history_frame_count": len(pixnav.get("frames", [])),
            "prediction_count": len(pixnav.get("predictions", [])),
        },
        "macro": {
            "run_dir": str(macro_dir),
            "audit": macro_audit,
        },
        "published": False,
        "actuation_calls": 0,
        "claim_scope": (
            "Immutable offline artifact linkage with a sanitized VLM response; "
            "not a strict live schema, timing, controller, localization, or motion proof."
        ),
    }


def write_chain_evidence(
    vlm_run_dir: Path,
    pixnav_report_path: Path,
    macro_run_dir: Path,
    output_root: Path,
) -> Path:
    result = validate_offline_chain(vlm_run_dir, pixnav_report_path, macro_run_dir)
    run_id = time.strftime("%Y%m%d_%H%M%S_pixnav_offline_chain")
    run_dir = output_root.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    result["run_id"] = run_id
    output = run_dir / "causal_manifest.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (run_dir / "SHA256SUMS").write_text(
        f"{sha256_file(output)}  causal_manifest.json\n",
        encoding="utf-8",
    )
    return run_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate an offline VLM→PixNav→macro artifact chain")
    parser.add_argument("--vlm-run-dir", type=Path, required=True)
    parser.add_argument("--pixnav-report", type=Path, required=True)
    parser.add_argument("--macro-run-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_dir = write_chain_evidence(
            args.vlm_run_dir,
            args.pixnav_report,
            args.macro_run_dir,
            args.output_root,
        )
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(f"PixNav offline causal chain BLOCKED: {error}")
        return 2
    print("PixNav offline causal chain: PASS_OFFLINE_CAUSAL_CHAIN_SANITIZED_VLM")
    print("No live service or actuator was contacted.")
    print(f"Evidence: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
