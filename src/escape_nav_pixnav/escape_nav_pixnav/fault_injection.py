"""No-actuation fault injection for PixNav contracts and offline artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from .adapter import PixNavMacroAdapter
from .audit_sink import AuditJsonlSink, verify_audit_chain
from .causal_chain import sha256_file, validate_offline_chain
from .contracts import ACTION_NAMES, PIXNAV_CHECKPOINT_A_SHA256, sha256_canonical


DEFAULT_OUTPUT_ROOT = Path.home() / ".ros" / "pixnav_fault_runs"


def _decision(sequence_id: int = 0) -> dict[str, Any]:
    probabilities = {name: 0.02 for name in ACTION_NAMES}
    probabilities["forward"] = 0.90
    return {
        "schema_version": "go2_pixnav_decision_v1",
        "event_id": f"fault:{sequence_id}",
        "sequence_id": sequence_id,
        "source_frame_sha256": "1" * 64,
        "checkpoint_sha256": PIXNAV_CHECKPOINT_A_SHA256,
        "observed_at_ns": 1_000_000_000,
        "inferred_at_ns": 1_100_000_000,
        "time_basis": "monotonic_live",
        "action_id": 1,
        "action": "forward",
        "action_probabilities": probabilities,
        "finite": True,
    }


def _rebuild_vlm_manifest(vlm_dir: Path) -> None:
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


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _adapter_scenarios() -> list[dict[str, Any]]:
    adapter = PixNavMacroAdapter()
    scenarios: list[tuple[str, Callable[[dict[str, Any]], None], int, str]] = [
        ("source_frame_stale", lambda raw: raw.update(observed_at_ns=0), 1_200_000_000, "SOURCE_FRAME_STALE"),
        (
            "decision_stale",
            lambda raw: raw.update(observed_at_ns=800_000_000, inferred_at_ns=900_000_000),
            1_500_000_000,
            "PIXNAV_DECISION_STALE",
        ),
        ("checkpoint_mismatch", lambda raw: raw.update(checkpoint_sha256="2" * 64), 1_200_000_000, "CHECKPOINT_HASH_MISMATCH"),
        ("nonfinite_output", lambda raw: raw.update(finite=False), 1_200_000_000, "MODEL_OUTPUT_NOT_FINITE"),
        (
            "evaluation_before_inference",
            lambda raw: None,
            1_050_000_000,
            "EVALUATION_PRECEDES_INFERENCE",
        ),
        ("action_id_name_mismatch", lambda raw: raw.update(action="turn_left"), 1_200_000_000, "INVALID_DECISION:ACTION_ID_NAME_MISMATCH"),
        (
            "fixed_camera_look_down",
            lambda raw: raw.update(
                action_id=5,
                action="look_down",
                action_probabilities={
                    "stop": 0.01,
                    "forward": 0.01,
                    "turn_left": 0.01,
                    "turn_right": 0.01,
                    "look_up": 0.01,
                    "look_down": 0.95,
                },
            ),
            1_200_000_000,
            "FIXED_CAMERA_VERTICAL_ACTION_UNSUPPORTED",
        ),
        (
            "low_motion_probability",
            lambda raw: raw.update(
                action_probabilities={
                    "stop": 0.15,
                    "forward": 0.40,
                    "turn_left": 0.12,
                    "turn_right": 0.11,
                    "look_up": 0.11,
                    "look_down": 0.11,
                }
            ),
            1_200_000_000,
            "SELECTED_PROBABILITY_BELOW_THRESHOLD",
        ),
    ]
    results = []
    for name, mutate, evaluated_at_ns, expected_reason in scenarios:
        raw = _decision()
        mutate(raw)
        proposal = adapter.adapt(raw, evaluated_at_ns=evaluated_at_ns)
        passed = (
            proposal.accepted is False
            and proposal.actuation_permitted is False
            and proposal.reason == expected_reason
            and proposal.target_dx_m == 0.0
            and proposal.target_dyaw_deg == 0.0
        )
        results.append(
            {
                "layer": "adapter",
                "scenario": name,
                "expected_reason": expected_reason,
                "observed_reason": proposal.reason,
                "safe_zero": proposal.target_dx_m == 0.0 and proposal.target_dyaw_deg == 0.0,
                "actuation_permitted": proposal.actuation_permitted,
                "passed": passed,
            }
        )
    return results


def _sink_scenarios(temp_root: Path) -> list[dict[str, Any]]:
    adapter = PixNavMacroAdapter()
    first = adapter.adapt(_decision(1), evaluated_at_ns=1_200_000_000)
    duplicate = adapter.adapt(_decision(1), evaluated_at_ns=1_200_000_000)
    earlier = adapter.adapt(_decision(0), evaluated_at_ns=1_200_000_000)
    unsafe = replace(first, actuation_permitted=True)
    results = []
    for name, candidate, expected in (
        ("duplicate_sequence", duplicate, "strictly increasing"),
        ("out_of_order_sequence", earlier, "strictly increasing"),
        ("actuation_flag_true", unsafe, "refuses"),
    ):
        path = temp_root / f"sink_{name}.jsonl"
        sink = AuditJsonlSink(path, fsync=False)
        sink.append(first)
        try:
            sink.append(candidate)
            observed = "ACCEPTED_UNSAFELY"
            passed = False
        except ValueError as error:
            observed = str(error)
            passed = expected in observed
        results.append(
            {
                "layer": "audit_sink",
                "scenario": name,
                "expected_error_contains": expected,
                "observed": observed,
                "passed": passed,
            }
        )

    tamper_path = temp_root / "sink_tamper.jsonl"
    AuditJsonlSink(tamper_path, fsync=False).append(first)
    record = _load(tamper_path)
    record["proposal"]["target_dx_m"] = 99.0
    tamper_path.write_text(json.dumps(record) + "\n", encoding="utf-8")
    try:
        verify_audit_chain(tamper_path)
        observed = "TAMPER_ACCEPTED_UNSAFELY"
        passed = False
    except ValueError as error:
        observed = str(error)
        passed = "hash mismatch" in observed
    results.append(
        {
            "layer": "audit_sink",
            "scenario": "record_byte_tamper",
            "expected_error_contains": "hash mismatch",
            "observed": observed,
            "passed": passed,
        }
    )
    return results


def _copy_chain(
    root: Path,
    vlm_source: Path,
    pixnav_source: Path,
    macro_source: Path,
) -> tuple[Path, Path, Path]:
    vlm = root / "vlm"
    pixnav = root / "pixnav_report.json"
    macro = root / "macro"
    shutil.copytree(vlm_source, vlm)
    shutil.copy2(pixnav_source, pixnav)
    shutil.copytree(macro_source, macro)
    macro_summary_path = macro / "summary.json"
    macro_summary = _load(macro_summary_path)
    macro_summary["source_report"] = str(pixnav)
    _write(macro_summary_path, macro_summary)
    return vlm, pixnav, macro


def _causal_scenarios(
    temp_root: Path,
    vlm_source: Path,
    pixnav_source: Path,
    macro_source: Path,
) -> list[dict[str, Any]]:
    def vlm_actuation(vlm: Path, pixnav: Path, macro: Path) -> None:
        path = vlm / "report.json"
        value = _load(path)
        value["physical_actuation_allowed"] = True
        _write(path, value)
        _rebuild_vlm_manifest(vlm)

    def vlm_pixel(vlm: Path, pixnav: Path, macro: Path) -> None:
        path = vlm / "vlm_sanitized.json"
        value = _load(path)
        value["selected_image_point"][0] += 1
        value["fine_goal"]["point_px"][0] += 1
        _write(path, value)
        _rebuild_vlm_manifest(vlm)

    def pixnav_history(vlm: Path, pixnav: Path, macro: Path) -> None:
        value = _load(pixnav)
        value["input_contract"]["history_start_index"] = -1
        _write(pixnav, value)

    def pixnav_checkpoint(vlm: Path, pixnav: Path, macro: Path) -> None:
        value = _load(pixnav)
        value["checkpoint_sha256_actual"] = "2" * 64
        _write(pixnav, value)

    def pixnav_published(vlm: Path, pixnav: Path, macro: Path) -> None:
        value = _load(pixnav)
        value["published"] = True
        _write(pixnav, value)

    def macro_source_hash(vlm: Path, pixnav: Path, macro: Path) -> None:
        path = macro / "summary.json"
        value = _load(path)
        value["source_report_sha256"] = "3" * 64
        _write(path, value)

    def macro_actuation(vlm: Path, pixnav: Path, macro: Path) -> None:
        path = macro / "macro_actions.jsonl"
        records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
        records[0]["proposal"]["actuation_permitted"] = True
        body = dict(records[0])
        body.pop("record_sha256")
        records[0]["record_sha256"] = sha256_canonical(body)
        path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")

    def vlm_byte_tamper(vlm: Path, pixnav: Path, macro: Path) -> None:
        (vlm / "vlm_raw.json").write_text("{}\n", encoding="utf-8")

    def vlm_malformed_json(vlm: Path, pixnav: Path, macro: Path) -> None:
        (vlm / "vlm_sanitized.json").write_text("{not-json\n", encoding="utf-8")
        _rebuild_vlm_manifest(vlm)

    def vlm_result_missing(vlm: Path, pixnav: Path, macro: Path) -> None:
        (vlm / "vlm_sanitized.json").unlink()
        _rebuild_vlm_manifest(vlm)

    scenarios: list[tuple[str, Callable[[Path, Path, Path], None], str]] = [
        ("vlm_actuation_allowed", vlm_actuation, "actuation interlock"),
        ("vlm_pixel_mismatch", vlm_pixel, "goal-u mismatch"),
        ("pixnav_history_before_capture", pixnav_history, "history predates capture"),
        ("pixnav_checkpoint_mismatch", pixnav_checkpoint, "Checkpoint_A hash mismatch"),
        ("pixnav_published_true", pixnav_published, "PixNav report published"),
        ("macro_source_hash_mismatch", macro_source_hash, "macro source report hash mismatch"),
        ("macro_actuation_flag_true", macro_actuation, "actuation interlock"),
        ("vlm_artifact_byte_tamper", vlm_byte_tamper, "artifact hash mismatch"),
        ("vlm_malformed_json", vlm_malformed_json, "invalid JSON"),
        ("vlm_result_missing", vlm_result_missing, "missing VLM artifact"),
    ]
    results = []
    for name, mutate, expected in scenarios:
        scenario_root = temp_root / f"causal_{name}"
        vlm, pixnav, macro = _copy_chain(
            scenario_root,
            vlm_source,
            pixnav_source,
            macro_source,
        )
        mutate(vlm, pixnav, macro)
        try:
            validate_offline_chain(vlm, pixnav, macro)
            observed = "FAULT_ACCEPTED_UNSAFELY"
            passed = False
        except ValueError as error:
            observed = str(error)
            passed = expected in observed
        results.append(
            {
                "layer": "offline_causal_chain",
                "scenario": name,
                "expected_error_contains": expected,
                "observed": observed,
                "passed": passed,
            }
        )
    return results


def run_fault_injection(
    vlm_run_dir: Path,
    pixnav_report: Path,
    macro_run_dir: Path,
    output_root: Path,
) -> Path:
    vlm = vlm_run_dir.expanduser().resolve()
    pixnav = pixnav_report.expanduser().resolve()
    macro = macro_run_dir.expanduser().resolve()
    baseline = validate_offline_chain(vlm, pixnav, macro)
    with tempfile.TemporaryDirectory(prefix="pixnav_fault_", dir="/tmp") as temp_value:
        temp_root = Path(temp_value)
        results = _adapter_scenarios()
        results.extend(_sink_scenarios(temp_root))
        results.extend(_causal_scenarios(temp_root, vlm, pixnav, macro))

    passed_count = sum(int(result["passed"]) for result in results)
    run_id = time.strftime("%Y%m%d_%H%M%S_pixnav_fault_injection")
    run_dir = output_root.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": "go2_pixnav_fault_injection_v1",
        "run_id": run_id,
        "source_vlm_run": str(vlm),
        "source_pixnav_report": str(pixnav),
        "source_macro_run": str(macro),
        "source_hashes": {
            "pixnav_report": sha256_file(pixnav),
            "macro_summary": sha256_file(macro / "summary.json"),
        },
        "baseline_causal_identity_sha256": baseline["causal_identity_sha256"],
        "scenario_count": len(results),
        "passed_count": passed_count,
        "failed_count": len(results) - passed_count,
        "results": results,
        "published": False,
        "actuation_calls": 0,
        "overall": "PASS_ALL_FAULTS_FAIL_CLOSED" if passed_count == len(results) else "FAIL_FAULT_ACCEPTED",
        "claim_scope": "Pure/file-copy fault injection only; no live timeout or physical stop-latency proof.",
    }
    output = run_dir / "fault_report.json"
    _write(output, report)
    (run_dir / "SHA256SUMS").write_text(
        f"{sha256_file(output)}  fault_report.json\n",
        encoding="utf-8",
    )
    return run_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run no-actuation PixNav fault injection")
    parser.add_argument("--vlm-run-dir", type=Path, required=True)
    parser.add_argument("--pixnav-report", type=Path, required=True)
    parser.add_argument("--macro-run-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_dir = run_fault_injection(
            args.vlm_run_dir,
            args.pixnav_report,
            args.macro_run_dir,
            args.output_root,
        )
        report = _load(run_dir / "fault_report.json")
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(f"PixNav fault injection BLOCKED: {error}")
        return 2
    print(f"PixNav fault injection: {report['overall']} ({report['passed_count']}/{report['scenario_count']})")
    print("All mutations were applied to temporary copies; no live service was contacted.")
    print(f"Evidence: {run_dir}")
    return 0 if report["failed_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
