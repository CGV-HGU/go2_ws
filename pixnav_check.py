#!/usr/bin/env python3
"""File-only Pixel-Navigator checkpoint qualification for the Go2 project.

This tool intentionally has no ROS, socket, Unitree SDK, or command publisher.
It verifies the paper-pinned lab implementation and official Checkpoint_A, then
optionally replays recorded real Go2 RGB frames into the frozen PixNav policy.
The output is evidence JSON only; action IDs are never converted to robot motion.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import types
from typing import Any


REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REFERENCE = (
    REPO_ROOT
    / ".local-data"
    / "vlm-s2e"
    / "runtime"
    / "vlm-s2e-integration-paper-pin"
)
DEFAULT_CHECKPOINT = (
    REPO_ROOT / ".local-data" / "vlm-s2e" / "checkpoints" / "pixelnav_A.ckpt"
)
DEFAULT_RUNTIME_SITE = REPO_ROOT / ".local-data" / "pixnav_runtime" / "site-packages"
DEFAULT_OUTPUT_ROOT = Path.home() / ".ros" / "pixnav_runs"

PAPER_COMMIT = "126f2f024c3cbbaa091734d0557e9d6f554adbde"
REFERENCE_COMMIT = "6341a5d33903131ddfce74498c04e1c0ae04ec61"
CHECKPOINT_SHA256 = "0b1faff7631962351bbbfe8cb115a3a03069f33fab499865f887ffbb5a3cabe3"
ACTION_NAMES = ("stop", "forward", "turn_left", "turn_right", "look_up", "look_down")
MAPPING_MARKERS = ("map_headless.sh", "rtabmap", "icp_odometry", "go2_livo_sensor_bridge")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Paper-pinned PixNav file-only checkpoint/replay qualification",
    )
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--reference-dir", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument(
        "--runtime-site",
        type=Path,
        default=DEFAULT_RUNTIME_SITE,
        help="isolated Python package directory (keeps Foxy/system Python untouched)",
    )
    parser.add_argument(
        "--frames-dir",
        type=Path,
        help="directory containing recorded real Go2 JPG/PNG frames in time order",
    )
    parser.add_argument("--goal-u", type=int, default=640)
    parser.add_argument("--goal-v", type=int, default=600)
    parser.add_argument("--goal-radius", type=int, default=8)
    parser.add_argument("--max-frames", type=int, default=11)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="verify commits, hash, files, and Python modules without loading the model",
    )
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def git_head(path: Path) -> str | None:
    try:
        return subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def running_mapping_processes() -> list[dict[str, Any]]:
    matches: list[dict[str, Any]] = []
    own_pid = os.getpid()
    for proc_dir in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(proc_dir.name)
            if pid == own_pid:
                continue
            raw_tokens = [token for token in (proc_dir / "cmdline").read_bytes().split(b"\0") if token]
            command = b" ".join(raw_tokens).decode("utf-8", errors="replace")
            basenames = {
                os.path.basename(token.decode("utf-8", errors="replace"))
                for token in raw_tokens
            }
        except (OSError, ValueError):
            continue
        marker = next((value for value in MAPPING_MARKERS if value in basenames), None)
        if marker:
            matches.append({"pid": pid, "marker": marker, "command": command})
    return matches


def collect_frames(path: Path | None, limit: int) -> list[Path]:
    if path is None or not path.is_dir():
        return []
    images = sorted(
        candidate
        for candidate in path.iterdir()
        if candidate.is_file() and candidate.suffix.lower() in {".jpg", ".jpeg", ".png"}
    )
    return images[: max(1, limit)]


def module_status(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def install_python38_settings_shim(reference: Path) -> dict[str, Any]:
    """Load pinned settings with deferred annotations on Python 3.8 only.

    The paper-pinned reference uses PEP 585 annotations such as ``list[str]``.
    Python 3.8 evaluates those eagerly and rejects them. Compiling the unmodified
    source with the future flag changes annotation evaluation only; the reference
    checkout and policy computation stay intact.
    """
    settings_path = reference / "settings.py"
    result: dict[str, Any] = {
        "required": sys.version_info < (3, 9),
        "applied": False,
        "method": "none",
        "source": str(settings_path),
        "source_sha256": sha256_file(settings_path) if settings_path.is_file() else None,
    }
    if sys.version_info >= (3, 9):
        return result
    if not settings_path.is_file():
        raise FileNotFoundError(f"pinned settings module not found: {settings_path}")

    module = types.ModuleType("settings")
    module.__file__ = str(settings_path)
    module.__package__ = ""
    source = "from __future__ import annotations\n" + settings_path.read_text(
        encoding="utf-8"
    )
    exec(compile(source, str(settings_path), "exec"), module.__dict__)
    sys.modules["settings"] = module
    result.update(
        {
            "applied": True,
            "method": "deferred_type_annotations_only",
        }
    )
    return result


def main() -> int:
    args = parse_args()
    run_id = time.strftime("%Y%m%d_%H%M%S_pixnav_file_only")
    run_dir = args.output_root.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)

    checkpoint = args.checkpoint.expanduser().resolve()
    reference = args.reference_dir.expanduser().resolve()
    runtime_site = args.runtime_site.expanduser().resolve()
    if runtime_site.is_dir() and str(runtime_site) not in sys.path:
        sys.path.insert(0, str(runtime_site))
    frames = collect_frames(
        args.frames_dir.expanduser().resolve() if args.frames_dir else None,
        args.max_frames,
    )
    mapping = running_mapping_processes()
    checkpoint_hash = sha256_file(checkpoint) if checkpoint.is_file() else None
    reference_head = git_head(reference) if reference.is_dir() else None
    modules = {name: module_status(name) for name in ("torch", "torchvision", "cv2", "numpy")}

    checks = {
        "reference_exists": reference.is_dir(),
        "reference_commit_matches": reference_head == REFERENCE_COMMIT,
        "checkpoint_exists": checkpoint.is_file(),
        "checkpoint_hash_matches": checkpoint_hash == CHECKPOINT_SHA256,
        "runtime_modules_present": all(modules.values()),
        "recorded_frames_present": bool(frames),
        "mapping_inactive": not mapping,
        "file_only_interlock": True,
    }
    report: dict[str, Any] = {
        "schema_version": "go2_pixnav_file_only_v1",
        "run_id": run_id,
        "paper_commit": PAPER_COMMIT,
        "reference_commit_expected": REFERENCE_COMMIT,
        "reference_commit_actual": reference_head,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256_expected": CHECKPOINT_SHA256,
        "checkpoint_sha256_actual": checkpoint_hash,
        "runtime_site": str(runtime_site),
        "frames": [
            {"path": str(path), "sha256": sha256_file(path)} for path in frames
        ],
        "goal_pixel": {"u": args.goal_u, "v": args.goal_v, "radius": args.goal_radius},
        "modules": modules,
        "mapping_processes": mapping,
        "checks": checks,
        "published": False,
        "actuation_calls": 0,
        "claim_scope": (
            "Checkpoint/runtime and recorded-RGB input-contract qualification only; "
            "not navigation success, localization, calibration, or Go2 actuation proof."
        ),
    }

    preflight_required = (
        checks["reference_commit_matches"]
        and checks["checkpoint_hash_matches"]
        and checks["runtime_modules_present"]
        and checks["file_only_interlock"]
    )
    if args.preflight_only:
        report["overall"] = "PASS" if preflight_required else "BLOCKED"
        report["inference_executed"] = False
        write_json(run_dir / "report.json", report)
        print(f"PixNav preflight: {report['overall']}")
        print(f"Evidence: {run_dir}")
        return 0 if preflight_required else 2

    if mapping:
        report["overall"] = "BLOCKED_MAPPING_ACTIVE"
        report["inference_executed"] = False
        write_json(run_dir / "report.json", report)
        print("PixNav replay blocked: mapping is active; run it after map_headless.sh exits.")
        print(f"Evidence: {run_dir}")
        return 3
    if not preflight_required or not frames:
        report["overall"] = "BLOCKED_PREREQUISITE"
        report["inference_executed"] = False
        write_json(run_dir / "report.json", report)
        print("PixNav replay blocked: inspect report.json prerequisites.")
        print(f"Evidence: {run_dir}")
        return 2

    import cv2
    import numpy as np
    import torch

    if str(reference) not in sys.path:
        sys.path.insert(0, str(reference))
    report["runtime_compatibility"] = install_python38_settings_shim(reference)
    from policy_network import PixelNavPolicy  # type: ignore

    if args.device == "cuda" and not torch.cuda.is_available():
        report["overall"] = "BLOCKED_CUDA_UNAVAILABLE"
        report["inference_executed"] = False
        write_json(run_dir / "report.json", report)
        print("PixNav replay blocked: CUDA requested but unavailable.")
        print(f"Evidence: {run_dir}")
        return 2
    device = "cuda" if args.device == "cuda" or (
        args.device == "auto" and torch.cuda.is_available()
    ) else "cpu"

    model = PixelNavPolicy(max_token_length=64, device=device)
    try:
        state_dict = torch.load(str(checkpoint), map_location=device, weights_only=True)
    except TypeError:
        state_dict = torch.load(str(checkpoint), map_location=device)
    model.load_state_dict(state_dict)
    model.eval()

    decoded = [cv2.imread(str(path), cv2.IMREAD_COLOR) for path in frames]
    if any(image is None or image.size == 0 for image in decoded):
        raise RuntimeError("one or more recorded RGB frames could not be decoded")
    height, width = decoded[0].shape[:2]
    u = max(0, min(width - 1, args.goal_u))
    v = max(0, min(height - 1, args.goal_v))
    radius = max(1, args.goal_radius)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.rectangle(
        mask,
        (max(0, u - radius), max(0, v - radius)),
        (min(width - 1, u + radius), min(height - 1, v + radius)),
        255,
        -1,
    )
    goal_image = cv2.resize(
        cv2.cvtColor(decoded[0], cv2.COLOR_BGR2RGB), (224, 224)
    )[np.newaxis, :, :, :]
    goal_mask = cv2.resize(mask, (224, 224), cv2.INTER_NEAREST)[
        np.newaxis, :, :, np.newaxis
    ]
    history = np.stack(
        [
            cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), (224, 224))
            for image in decoded
        ],
        axis=0,
    )[np.newaxis, :, :, :, :]

    started = time.perf_counter()
    with torch.inference_mode():
        action_logits, distance_pred, goal_pred = model(goal_mask, goal_image, history)
        probabilities = torch.softmax(action_logits[0], dim=-1).detach().cpu().numpy()
        distances = distance_pred[0, :, 0].detach().cpu().numpy()
        tracked_goals = goal_pred[0].detach().cpu().numpy()
    latency_s = time.perf_counter() - started

    predictions = []
    for index, (probability, distance_raw, tracked_goal) in enumerate(
        zip(probabilities, distances, tracked_goals)
    ):
        action_id = int(np.argmax(probability))
        predictions.append(
            {
                "frame_index": index,
                "action_id": action_id,
                "action": ACTION_NAMES[action_id],
                "action_probabilities": {
                    name: round(float(value), 7)
                    for name, value in zip(ACTION_NAMES, probability)
                },
                "distance_raw": round(float(distance_raw), 7),
                "tracked_goal_normalized": [round(float(value), 7) for value in tracked_goal],
                "finite": bool(
                    np.all(np.isfinite(probability))
                    and math.isfinite(float(distance_raw))
                    and np.all(np.isfinite(tracked_goal))
                ),
            }
        )

    report.update(
        {
            "overall": "PASS_FILE_ONLY_REPLAY"
            if all(item["finite"] for item in predictions)
            else "FAIL_NONFINITE_OUTPUT",
            "inference_executed": True,
            "device": device,
            "torch_version": torch.__version__,
            "torchvision_available": modules["torchvision"],
            "input_shapes": {
                "goal_image": list(goal_image.shape),
                "goal_mask": list(goal_mask.shape),
                "history": list(history.shape),
            },
            "latency_s": round(latency_s, 6),
            "predictions": predictions,
            "published": False,
            "actuation_calls": 0,
        }
    )
    write_json(run_dir / "report.json", report)
    print(f"PixNav replay: {report['overall']} ({device}, {latency_s:.3f}s)")
    print("No robot command was created or published.")
    print(f"Evidence: {run_dir}")
    return 0 if report["overall"] == "PASS_FILE_ONLY_REPLAY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
