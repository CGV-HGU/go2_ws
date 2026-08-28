"""Convert a completed PixNav inference report into file-only macro evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from collections import Counter
from pathlib import Path
from typing import Any

from .adapter import PixNavMacroAdapter
from .audit_sink import AuditJsonlSink, verify_audit_chain
from .contracts import (
    AdapterConfig,
    PIXNAV_CHECKPOINT_A_SHA256,
    PIXNAV_REFERENCE_COMMIT,
    TimeBasis,
    canonical_json,
    is_sha256,
)


DEFAULT_OUTPUT_ROOT = Path.home() / ".ros" / "pixnav_macro_runs"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_report(path: Path) -> dict[str, Any]:
    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid PixNav report JSON: {error}") from error
    if report.get("schema_version") != "go2_pixnav_file_only_v2":
        raise ValueError("source report is not capture-contract v2")
    if report.get("overall") != "PASS_FILE_ONLY_REPLAY":
        raise ValueError("source report is not PASS_FILE_ONLY_REPLAY")
    if report.get("inference_executed") is not True:
        raise ValueError("source report did not execute inference")
    if report.get("published") is not False or int(report.get("actuation_calls", -1)) != 0:
        raise ValueError("source report violates the no-actuation contract")
    if (
        report.get("checkpoint_sha256_actual") != PIXNAV_CHECKPOINT_A_SHA256
        or report.get("checkpoint_sha256_expected") != PIXNAV_CHECKPOINT_A_SHA256
    ):
        raise ValueError("source report does not use official Checkpoint_A")
    if (
        report.get("reference_commit_actual") != PIXNAV_REFERENCE_COMMIT
        or report.get("reference_commit_expected") != PIXNAV_REFERENCE_COMMIT
    ):
        raise ValueError("source report does not use the pinned PixNav reference")
    goal_frame = report.get("goal_frame")
    contract = report.get("input_contract")
    if not isinstance(goal_frame, dict) or not isinstance(contract, dict):
        raise ValueError("source report has no capture-view contract")
    goal_index = goal_frame.get("index")
    history_start = contract.get("history_start_index")
    if (
        isinstance(goal_index, bool)
        or not isinstance(goal_index, int)
        or isinstance(history_start, bool)
        or not isinstance(history_start, int)
        or history_start < goal_index
        or contract.get("history_rule") != "observations_must_be_at_or_after_goal_capture"
    ):
        raise ValueError("source report history predates or lacks capture-view goal")
    predictions = report.get("predictions")
    frames = report.get("frames")
    if not isinstance(predictions, list) or not predictions:
        raise ValueError("source report has no predictions")
    if not isinstance(frames, list) or len(frames) != len(predictions):
        raise ValueError("frame/prediction count mismatch")
    for frame, prediction in zip(frames, predictions):
        if not isinstance(frame, dict) or not isinstance(prediction, dict):
            raise ValueError("frame/prediction entry is not an object")
        frame_index = frame.get("index")
        if (
            isinstance(frame_index, bool)
            or not isinstance(frame_index, int)
            or frame_index < history_start
            or prediction.get("frame_index") != frame_index
            or not is_sha256(str(frame.get("sha256", "")))
        ):
            raise ValueError("frame/prediction capture order or hash mismatch")
    return report


def replay_report(report_path: Path, output_root: Path) -> Path:
    source = report_path.expanduser().resolve()
    report = _load_report(source)
    checkpoint_hash = str(report.get("checkpoint_sha256_actual", ""))
    config = AdapterConfig(expected_checkpoint_sha256=checkpoint_hash)
    adapter = PixNavMacroAdapter(config)
    run_id = time.strftime("%Y%m%d_%H%M%S_pixnav_macro_file_only")
    run_dir = output_root.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    audit_path = run_dir / "macro_actions.jsonl"
    sink = AuditJsonlSink(audit_path)
    proposal_kinds: Counter[str] = Counter()
    reasons: Counter[str] = Counter()
    accepted_count = 0

    for sequence_id, (frame, prediction) in enumerate(
        zip(report["frames"], report["predictions"])
    ):
        frame_hash = str(frame["sha256"])
        event_seed = {
            "source_report_sha256": _sha256_file(source),
            "source_run_id": report.get("run_id"),
            "sequence_id": sequence_id,
            "source_frame_sha256": frame_hash,
        }
        event_id = f"pixnav:{hashlib.sha256(canonical_json(event_seed).encode()).hexdigest()[:32]}"
        decision = {
            "schema_version": "go2_pixnav_decision_v1",
            "event_id": event_id,
            "sequence_id": sequence_id,
            "source_frame_sha256": frame_hash,
            "checkpoint_sha256": checkpoint_hash,
            "observed_at_ns": 0,
            "inferred_at_ns": 0,
            "time_basis": TimeBasis.OFFLINE_REPLAY.value,
            "action_id": prediction["action_id"],
            "action": prediction["action"],
            "action_probabilities": prediction["action_probabilities"],
            "finite": prediction["finite"],
        }
        proposal = adapter.adapt(decision, evaluated_at_ns=0)
        sink.append(proposal)
        proposal_kinds[proposal.proposal_kind.value] += 1
        reasons[proposal.reason] += 1
        accepted_count += int(proposal.accepted)

    verification = verify_audit_chain(audit_path)
    summary = {
        "schema_version": "go2_pixnav_macro_replay_summary_v1",
        "run_id": run_id,
        "source_report": str(source),
        "source_report_sha256": _sha256_file(source),
        "source_checkpoint_sha256": checkpoint_hash,
        "adapter_config": config.to_dict(),
        "adapter_config_sha256": config.sha256,
        "input_prediction_count": len(report["predictions"]),
        "accepted_proposal_count": accepted_count,
        "proposal_kind_counts": dict(sorted(proposal_kinds.items())),
        "reason_counts": dict(sorted(reasons.items())),
        "audit": verification,
        "published": False,
        "actuation_calls": 0,
        "actuation_permitted_count": 0,
        "overall": "PASS_FILE_ONLY_MACRO_REPLAY",
        "claim_scope": (
            "PixNav-output validation and bounded macro proposal audit only; "
            "not a controller, safety gateway, localization, or robot motion proof."
        ),
    }
    (run_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return run_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a PixNav report into a no-actuation macro-action JSONL audit",
    )
    parser.add_argument("report", type=Path)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_dir = replay_report(args.report, args.output_root)
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(f"PixNav macro replay BLOCKED: {error}")
        return 2
    print("PixNav macro replay: PASS_FILE_ONLY_MACRO_REPLAY")
    print("No ROS, socket, SDK, command publisher, or actuator was used.")
    print(f"Evidence: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
