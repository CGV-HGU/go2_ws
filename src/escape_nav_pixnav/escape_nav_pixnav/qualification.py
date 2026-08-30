"""Freeze a no-actuation PixNav qualification manifest.

This module performs read-only validation of existing evidence and writes one
new JSON manifest. It never imports ROS, networking, or robot SDK modules.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import time
from typing import Any

from .causal_chain import sha256_file, validate_offline_chain
from .contracts import AdapterConfig, PIXNAV_CHECKPOINT_A_SHA256


DEFAULT_OUTPUT_ROOT = Path.home() / ".ros" / "pixnav_qualification_runs"
REFERENCE_COMMIT = "6341a5d33903131ddfce74498c04e1c0ae04ec61"
RUNTIME_MODULES = (
    "adapter.py",
    "audit_sink.py",
    "causal_chain.py",
    "contracts.py",
    "event_ledger.py",
    "fault_injection.py",
    "gateway_core.py",
    "replay.py",
    "safety_admission.py",
)
BANNED_IMPORT_ROOTS = {
    "geometry_msgs",
    "launch",
    "launch_ros",
    "nav_msgs",
    "rclpy",
    "rospy",
    "socket",
    "unitree_sdk2",
    "unitree_sdk2py",
}


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    # Preserve the leading status column used by ``git status --short``.
    return result.stdout.rstrip()


def audit_runtime_sources(package_dir: Path) -> dict[str, Any]:
    """Hash runtime sources and reject direct transport/robot dependencies."""
    imported_roots: set[str] = set()
    source_hashes: dict[str, str] = {}
    missing = []
    for name in RUNTIME_MODULES:
        path = package_dir / name
        if not path.is_file():
            missing.append(name)
            continue
        source_hashes[name] = sha256_file(path)
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_roots.update(alias.name.split(".", 1)[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_roots.add(node.module.split(".", 1)[0])
    forbidden = sorted(imported_roots & BANNED_IMPORT_ROOTS)
    return {
        "scope": list(RUNTIME_MODULES),
        "source_sha256": source_hashes,
        "missing_modules": missing,
        "import_roots": sorted(imported_roots),
        "forbidden_import_roots": forbidden,
        "passed": not missing and not forbidden,
        "claim_scope": "Static direct-import audit only; not proof about a future downstream gateway.",
    }


def create_qualification_manifest(
    *,
    repo_root: Path,
    checkpoint: Path,
    reference_dir: Path,
    vlm_run_dir: Path,
    pixnav_report: Path,
    macro_run_dir: Path,
    causal_run_dir: Path,
    fault_run_dir: Path,
    output_root: Path,
) -> Path:
    repo = repo_root.expanduser().resolve()
    checkpoint_path = checkpoint.expanduser().resolve()
    reference = reference_dir.expanduser().resolve()
    vlm = vlm_run_dir.expanduser().resolve()
    pixnav_path = pixnav_report.expanduser().resolve()
    macro = macro_run_dir.expanduser().resolve()
    causal = causal_run_dir.expanduser().resolve()
    fault = fault_run_dir.expanduser().resolve()

    required_files = (
        checkpoint_path,
        pixnav_path,
        macro / "summary.json",
        macro / "macro_actions.jsonl",
        causal / "causal_manifest.json",
        fault / "fault_report.json",
    )
    missing = [str(path) for path in required_files if not path.is_file()]
    if missing:
        raise ValueError(f"qualification inputs missing: {missing}")

    baseline = validate_offline_chain(vlm, pixnav_path, macro)
    pixnav = _load_json(pixnav_path)
    macro_summary = _load_json(macro / "summary.json")
    causal_manifest = _load_json(causal / "causal_manifest.json")
    fault_report = _load_json(fault / "fault_report.json")
    checkpoint_hash = sha256_file(checkpoint_path)
    reference_head = _git(reference, "rev-parse", "HEAD")
    static_audit = audit_runtime_sources(
        repo / "src" / "escape_nav_pixnav" / "escape_nav_pixnav"
    )

    checks = {
        "offline_chain_revalidated": baseline["overall"].startswith("PASS_"),
        "causal_identity_frozen": causal_manifest.get("causal_identity_sha256")
        == baseline["causal_identity_sha256"],
        "fault_baseline_matches": fault_report.get("baseline_causal_identity_sha256")
        == baseline["causal_identity_sha256"],
        "fault_suite_passed": fault_report.get("overall") == "PASS_ALL_FAULTS_FAIL_CLOSED"
        and fault_report.get("failed_count") == 0,
        "pixnav_input_contract_v2": pixnav.get("schema_version") == "go2_pixnav_file_only_v2"
        and pixnav.get("input_contract", {}).get("history_rule")
        == "observations_must_be_at_or_after_goal_capture",
        "checkpoint_bytes_match": checkpoint_hash == PIXNAV_CHECKPOINT_A_SHA256,
        "pixnav_checkpoint_matches": pixnav.get("checkpoint_sha256_actual")
        == PIXNAV_CHECKPOINT_A_SHA256,
        "reference_commit_matches": reference_head == REFERENCE_COMMIT
        and pixnav.get("reference_commit_actual") == REFERENCE_COMMIT,
        "macro_is_file_only": macro_summary.get("published") is False
        and macro_summary.get("actuation_calls") == 0
        and macro_summary.get("actuation_permitted_count") == 0,
        "static_no_transport_import_audit": static_audit["passed"],
    }
    if not all(checks.values()):
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise ValueError(f"qualification checks failed: {failed}")

    try:
        repo_commit = _git(repo, "rev-parse", "HEAD")
        repo_branch = _git(repo, "branch", "--show-current")
        dirty_paths = [
            line[3:]
            for line in _git(repo, "status", "--short", "--untracked-files=all").splitlines()
            if len(line) > 3
        ]
    except (subprocess.SubprocessError, OSError):
        repo_commit = "UNAVAILABLE"
        repo_branch = "UNAVAILABLE"
        dirty_paths = []

    run_id = time.strftime("%Y%m%d_%H%M%S_pixnav_qualification")
    run_dir = output_root.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": "go2_pixnav_no_actuation_qualification_v1",
        "run_id": run_id,
        "overall": "PASS_JETSON_FILE_ONLY_QUALIFICATION",
        "checks": checks,
        "repository": {
            "root": str(repo),
            "branch": repo_branch,
            "commit": repo_commit,
            "dirty_paths": dirty_paths,
        },
        "frozen_runtime": {
            "reference_dir": str(reference),
            "reference_commit": reference_head,
            "checkpoint": str(checkpoint_path),
            "checkpoint_size_bytes": checkpoint_path.stat().st_size,
            "checkpoint_sha256": checkpoint_hash,
            "adapter_config": AdapterConfig().to_dict(),
            "adapter_config_sha256": AdapterConfig().sha256,
            "static_audit": static_audit,
        },
        "evidence": {
            "vlm_run_dir": str(vlm),
            "pixnav_report": str(pixnav_path),
            "pixnav_report_sha256": sha256_file(pixnav_path),
            "macro_run_dir": str(macro),
            "macro_summary_sha256": sha256_file(macro / "summary.json"),
            "causal_run_dir": str(causal),
            "causal_manifest_sha256": sha256_file(causal / "causal_manifest.json"),
            "causal_identity_sha256": baseline["causal_identity_sha256"],
            "fault_run_dir": str(fault),
            "fault_report_sha256": sha256_file(fault / "fault_report.json"),
            "fault_scenarios_passed": fault_report["passed_count"],
        },
        "published": False,
        "actuation_calls": 0,
        "actuation_permitted": False,
        "claim_scope": (
            "Jetson file-only inference/artifact/adapter qualification. This is not live camera "
            "timing, localization, obstacle safety, controller, physical stop-latency, or robot motion proof."
        ),
    }
    report_path = run_dir / "qualification_manifest.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (run_dir / "SHA256SUMS").write_text(
        f"{sha256_file(report_path)}  qualification_manifest.json\n",
        encoding="utf-8",
    )
    return run_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Freeze no-actuation PixNav qualification")
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--reference-dir", type=Path, required=True)
    parser.add_argument("--vlm-run-dir", type=Path, required=True)
    parser.add_argument("--pixnav-report", type=Path, required=True)
    parser.add_argument("--macro-run-dir", type=Path, required=True)
    parser.add_argument("--causal-run-dir", type=Path, required=True)
    parser.add_argument("--fault-run-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_dir = create_qualification_manifest(**vars(args))
        report = _load_json(run_dir / "qualification_manifest.json")
    except (json.JSONDecodeError, KeyError, OSError, subprocess.SubprocessError, ValueError) as error:
        print(f"PixNav qualification BLOCKED: {error}")
        return 2
    print(f"PixNav qualification: {report['overall']}")
    print("No ROS publisher, socket, or robot SDK was used.")
    print(f"Evidence: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
