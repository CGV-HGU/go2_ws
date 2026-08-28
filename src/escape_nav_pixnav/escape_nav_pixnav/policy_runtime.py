"""Persistent, file-only runtime for the paper-pinned frozen PixNav policy.

The runtime intentionally has no ROS, network, Unitree SDK, controller, or
actuator dependency.  It keeps the CUDA model resident so live qualification
does not reload a 218 MB checkpoint after every camera observation.
"""

from __future__ import annotations

import hashlib
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import time
import types
from typing import Any, Optional, Sequence

from .contracts import ACTION_NAMES, PIXNAV_CHECKPOINT_A_SHA256, PIXNAV_REFERENCE_COMMIT


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_REFERENCE = (
    REPO_ROOT / ".local-data" / "vlm-s2e" / "runtime" / "vlm-s2e-integration-paper-pin"
)
DEFAULT_CHECKPOINT = REPO_ROOT / ".local-data" / "vlm-s2e" / "checkpoints" / "pixelnav_A.ckpt"
DEFAULT_RUNTIME_SITE = REPO_ROOT / ".local-data" / "pixnav_runtime" / "site-packages"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_head(path: Path) -> Optional[str]:
    try:
        return subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _module_present(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def _install_python38_settings_shim(reference: Path) -> dict[str, Any]:
    settings_path = reference / "settings.py"
    result: dict[str, Any] = {
        "required": sys.version_info < (3, 9),
        "applied": False,
        "method": "none",
        "source": str(settings_path),
        "source_sha256": _sha256_file(settings_path) if settings_path.is_file() else None,
    }
    if sys.version_info >= (3, 9):
        return result
    if not settings_path.is_file():
        raise FileNotFoundError(f"pinned settings module not found: {settings_path}")
    module = types.ModuleType("settings")
    module.__file__ = str(settings_path)
    module.__package__ = ""
    source = "from __future__ import annotations\n" + settings_path.read_text(encoding="utf-8")
    exec(compile(source, str(settings_path), "exec"), module.__dict__)
    sys.modules["settings"] = module
    result.update({"applied": True, "method": "deferred_type_annotations_only"})
    return result


class FrozenPixNavRuntime:
    """Keep one verified Checkpoint_A policy resident for repeated inference."""

    def __init__(
        self,
        *,
        device: str = "cuda",
        checkpoint: Path = DEFAULT_CHECKPOINT,
        reference: Path = DEFAULT_REFERENCE,
        runtime_site: Path = DEFAULT_RUNTIME_SITE,
    ) -> None:
        if device not in {"cuda", "cpu"}:
            raise ValueError("device must be cuda or cpu")
        self.checkpoint = checkpoint.expanduser().resolve()
        self.reference = reference.expanduser().resolve()
        self.runtime_site = runtime_site.expanduser().resolve()
        if self.runtime_site.is_dir() and str(self.runtime_site) not in sys.path:
            sys.path.insert(0, str(self.runtime_site))
        modules = {
            name: _module_present(name) for name in ("torch", "torchvision", "cv2", "numpy")
        }
        if not all(modules.values()):
            raise RuntimeError(f"PIXNAV_RUNTIME_MODULE_MISSING:{modules}")
        if not self.reference.is_dir():
            raise RuntimeError("PIXNAV_REFERENCE_MISSING")
        reference_head = _git_head(self.reference)
        if reference_head != PIXNAV_REFERENCE_COMMIT:
            raise RuntimeError(f"PIXNAV_REFERENCE_COMMIT_MISMATCH:{reference_head}")
        if not self.checkpoint.is_file():
            raise RuntimeError("PIXNAV_CHECKPOINT_MISSING")
        checkpoint_hash = _sha256_file(self.checkpoint)
        if checkpoint_hash != PIXNAV_CHECKPOINT_A_SHA256:
            raise RuntimeError(f"PIXNAV_CHECKPOINT_HASH_MISMATCH:{checkpoint_hash}")

        import torch

        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("PIXNAV_CUDA_UNAVAILABLE")
        if str(self.reference) not in sys.path:
            sys.path.insert(0, str(self.reference))
        compatibility = _install_python38_settings_shim(self.reference)
        from policy_network import PixelNavPolicy  # type: ignore

        load_started_ns = time.monotonic_ns()
        model = PixelNavPolicy(max_token_length=64, device=device)
        try:
            state_dict = torch.load(str(self.checkpoint), map_location=device, weights_only=True)
        except TypeError:
            state_dict = torch.load(str(self.checkpoint), map_location=device)
        model.load_state_dict(state_dict)
        model.eval()
        if device == "cuda":
            torch.cuda.synchronize()
        load_finished_ns = time.monotonic_ns()

        self.device = device
        self.model = model
        self.torch = torch
        self.metadata = {
            "runtime_kind": "persistent_in_process_file_only",
            "device": device,
            "torch_version": torch.__version__,
            "reference": str(self.reference),
            "reference_commit": reference_head,
            "checkpoint": str(self.checkpoint),
            "checkpoint_sha256": checkpoint_hash,
            "runtime_site": str(self.runtime_site),
            "modules": modules,
            "runtime_compatibility": compatibility,
            "load_started_monotonic_ns": load_started_ns,
            "load_finished_monotonic_ns": load_finished_ns,
            "load_latency_s": round((load_finished_ns - load_started_ns) / 1e9, 6),
            "ros_publishers_created": 0,
            "unitree_sdk_clients_created": 0,
            "udp_command_senders_created": 0,
            "actuation_calls": 0,
        }

    def _synchronize(self) -> None:
        if self.device == "cuda":
            self.torch.cuda.synchronize()

    def warmup(self, *, sequence_length: int = 1) -> dict[str, Any]:
        """Warm CUDA kernels with synthetic arrays; result is never a decision."""

        if sequence_length < 1 or sequence_length > 64:
            raise ValueError("sequence_length must be in [1, 64]")
        import numpy as np

        goal_mask = np.zeros((1, 224, 224, 1), dtype=np.uint8)
        goal_image = np.zeros((1, 224, 224, 3), dtype=np.uint8)
        history = np.zeros((1, sequence_length, 224, 224, 3), dtype=np.uint8)
        self._synchronize()
        started_ns = time.monotonic_ns()
        with self.torch.inference_mode():
            self.model(goal_mask, goal_image, history)
        self._synchronize()
        finished_ns = time.monotonic_ns()
        return {
            "input_kind": "synthetic_zero_warmup_not_a_decision",
            "sequence_length": sequence_length,
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": finished_ns,
            "latency_s": round((finished_ns - started_ns) / 1e9, 6),
            "decision_created": False,
            "actuation_calls": 0,
        }

    def infer_files(
        self,
        frame_paths: Sequence[Path],
        *,
        goal_frame_index: int,
        history_start_index: int,
        goal_u: int,
        goal_v: int,
        goal_radius: int = 8,
    ) -> dict[str, Any]:
        """Infer over capture-ordered image files and return a file-only report."""

        import cv2
        import numpy as np

        paths = [Path(path).expanduser().resolve() for path in frame_paths]
        if not paths:
            raise ValueError("at least one frame is required")
        if not 0 <= goal_frame_index < len(paths):
            raise ValueError("goal_frame_index is outside frame list")
        if not goal_frame_index <= history_start_index < len(paths):
            raise ValueError("history_start_index must be at or after goal frame")
        history_paths = paths[history_start_index:]
        if len(history_paths) > 64:
            raise ValueError("history exceeds PixNav maximum token length")

        goal_bgr = cv2.imread(str(paths[goal_frame_index]), cv2.IMREAD_COLOR)
        decoded = [cv2.imread(str(path), cv2.IMREAD_COLOR) for path in history_paths]
        if goal_bgr is None or goal_bgr.size == 0 or any(
            image is None or image.size == 0 for image in decoded
        ):
            raise RuntimeError("one or more PixNav frames could not be decoded")
        height, width = goal_bgr.shape[:2]
        if any(image.shape[:2] != (height, width) for image in decoded):
            raise RuntimeError("goal and history frame dimensions do not match")
        if not 0 <= goal_u < width or not 0 <= goal_v < height:
            raise ValueError("goal pixel is outside capture-view frame")
        if goal_radius < 1:
            raise ValueError("goal_radius must be positive")

        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.rectangle(
            mask,
            (max(0, goal_u - goal_radius), max(0, goal_v - goal_radius)),
            (min(width - 1, goal_u + goal_radius), min(height - 1, goal_v + goal_radius)),
            255,
            -1,
        )
        goal_image = cv2.resize(
            cv2.cvtColor(goal_bgr, cv2.COLOR_BGR2RGB), (224, 224)
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

        self._synchronize()
        started_ns = time.monotonic_ns()
        with self.torch.inference_mode():
            action_logits, distance_pred, goal_pred = self.model(
                goal_mask, goal_image, history
            )
            probabilities = self.torch.softmax(action_logits[0], dim=-1).detach().cpu().numpy()
            distances = distance_pred[0, :, 0].detach().cpu().numpy()
            tracked_goals = goal_pred[0].detach().cpu().numpy()
        self._synchronize()
        finished_ns = time.monotonic_ns()

        predictions = []
        for offset, (probability, distance_raw, tracked_goal) in enumerate(
            zip(probabilities, distances, tracked_goals)
        ):
            action_id = int(np.argmax(probability))
            predictions.append(
                {
                    "frame_index": history_start_index + offset,
                    "history_offset": offset,
                    "action_id": action_id,
                    "action": ACTION_NAMES[action_id],
                    "action_probabilities": {
                        name: round(float(value), 7)
                        for name, value in zip(ACTION_NAMES, probability)
                    },
                    "distance_raw": round(float(distance_raw), 7),
                    "tracked_goal_normalized": [
                        round(float(value), 7) for value in tracked_goal
                    ],
                    "finite": bool(
                        np.all(np.isfinite(probability))
                        and math.isfinite(float(distance_raw))
                        and np.all(np.isfinite(tracked_goal))
                    ),
                }
            )
        return {
            "schema_version": "go2_pixnav_persistent_file_only_v1",
            "overall": (
                "PASS_PERSISTENT_FILE_ONLY_INFERENCE"
                if all(item["finite"] for item in predictions)
                else "FAIL_NONFINITE_OUTPUT"
            ),
            "checkpoint_sha256_actual": PIXNAV_CHECKPOINT_A_SHA256,
            "reference_commit_actual": PIXNAV_REFERENCE_COMMIT,
            "device": self.device,
            "source_frames": [
                {"index": index, "path": str(path), "sha256": _sha256_file(path)}
                for index, path in enumerate(paths)
            ],
            "goal_frame_index": goal_frame_index,
            "history_start_index": history_start_index,
            "goal_pixel": {"u": goal_u, "v": goal_v, "radius": goal_radius},
            "input_shapes": {
                "goal_image": list(goal_image.shape),
                "goal_mask": list(goal_mask.shape),
                "history": list(history.shape),
            },
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": finished_ns,
            "latency_s": round((finished_ns - started_ns) / 1e9, 6),
            "predictions": predictions,
            "published": False,
            "actuation_calls": 0,
        }
