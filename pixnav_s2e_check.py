#!/usr/bin/env python3
"""Strict live-camera PixNav/S2E qualification with no actuation path.

This program deliberately has no ROS publisher and no UDP command sender.  It
captures real Go2 RGB frames, validates the S2E input tensor, calls the real VLM
API, and (only when a real checkpoint and ONNXRuntime are present) runs S2E
inference into a file-only command audit sink.

It never substitutes a saved/synthetic image, mock trajectory, or heuristic
velocity when a real dependency is missing.  Missing prerequisites are reported
as BLOCKED in ``report.json``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any


REPO_ROOT = Path(__file__).resolve().parent
CORE_SOURCE = REPO_ROOT / "s2e-vlm-async-framework" / "src" / "s2e_vlm_core"
QWEN_SOURCE = REPO_ROOT / "qwen_nav_memory_framework_v3" / "qwen_nav_memory_framework"

for source_dir in (CORE_SOURCE, QWEN_SOURCE):
    if str(source_dir) not in sys.path:
        sys.path.insert(0, str(source_dir))


ACTUATION_MARKERS = (
    "host_bridge.py",
    "docker_bridge.py",
    "official_unitree_bridge.py",
    "python_direct_driver.py",
    "test_lab_micro_motion.py",
    "bringup_all_escape_nav.sh",
)
MAPPING_MARKERS = ("map_headless.sh", "rtabmap", "icp_odometry", "rgbd_odometry")
COMMAND_UDP_PORTS = (9090, 9091)
FRAME_COUNT = 11
IMAGE_SIZE = 256


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Real RGB + VLM + optional real S2E, with file-only output",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path.home() / ".ros" / "pixnav_s2e_runs",
        help="parent directory for immutable-style evidence runs",
    )
    parser.add_argument(
        "--frames-dir",
        type=Path,
        help="use exactly 11 real frame files instead of opening the live camera",
    )
    parser.add_argument(
        "--model",
        type=Path,
        help="path to s2e.onnx or its containing S2E directory",
    )
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument("--goal-x", type=float, default=2.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--frame-interval", type=float, default=0.20)
    parser.add_argument("--camera-timeout", type=float, default=20.0)
    parser.add_argument("--vlm-timeout", type=float, default=120.0)
    parser.add_argument(
        "--skip-vlm",
        action="store_true",
        help="skip the remote semantic decision (useful for isolated ONNX replay)",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="check safety/runtime prerequisites without camera, VLM, or inference",
    )
    return parser.parse_args()


def json_write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def process_matches() -> dict[str, list[dict[str, Any]]]:
    matches: dict[str, list[dict[str, Any]]] = {"actuation": [], "mapping": []}
    own_pid = os.getpid()
    for proc_dir in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(proc_dir.name)
            if pid == own_pid:
                continue
            raw = (proc_dir / "cmdline").read_bytes()
            raw_tokens = [token for token in raw.split(b"\0") if token]
            command = b" ".join(raw_tokens).decode("utf-8", errors="replace").strip()
            basenames = {
                os.path.basename(token.decode("utf-8", errors="replace"))
                for token in raw_tokens
            }
        except (OSError, ValueError):
            continue
        if not command:
            continue
        for marker in ACTUATION_MARKERS:
            if marker in basenames:
                matches["actuation"].append({"pid": pid, "marker": marker, "command": command})
                break
        for marker in MAPPING_MARKERS:
            if marker in basenames:
                matches["mapping"].append({"pid": pid, "marker": marker, "command": command})
                break
    return matches


def bound_udp_command_ports() -> list[int]:
    ports: set[int] = set()
    for table in (Path("/proc/net/udp"), Path("/proc/net/udp6")):
        try:
            lines = table.read_text(encoding="utf-8").splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 2:
                continue
            try:
                port = int(fields[1].split(":")[1], 16)
            except (IndexError, ValueError):
                continue
            if port in COMMAND_UDP_PORTS:
                ports.add(port)
    return sorted(ports)


def camera_pipeline() -> str:
    return (
        "udpsrc port=1720 multicast-group=230.1.1.1 multicast-iface=eth0 "
        "auto-multicast=true timeout=5000000000 ! "
        "application/x-rtp,media=video,clock-rate=90000,payload=96,encoding-name=H264 ! "
        "rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! "
        "video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def capture_live_frames(
    frames_dir: Path,
    *,
    interval_s: float,
    timeout_s: float,
) -> tuple[list[Any], list[dict[str, Any]]]:
    import cv2

    capture = cv2.VideoCapture(camera_pipeline(), cv2.CAP_GSTREAMER)
    if not capture.isOpened():
        raise RuntimeError("live Go2 GStreamer camera could not be opened")

    frames: list[Any] = []
    metadata: list[dict[str, Any]] = []
    deadline = time.monotonic() + timeout_s
    next_sample = 0.0
    try:
        while len(frames) < FRAME_COUNT and time.monotonic() < deadline:
            ok, frame = capture.read()
            now = time.monotonic()
            if not ok or frame is None or frame.size == 0:
                continue
            if now < next_sample:
                continue
            next_sample = now + max(0.0, interval_s)
            frame = frame.copy()
            index = len(frames)
            path = frames_dir / f"frame_{index:02d}.jpg"
            if not cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
                raise RuntimeError(f"failed to write evidence frame: {path}")
            frames.append(frame)
            metadata.append(
                {
                    "index": index,
                    "capture_unix_ns": time.time_ns(),
                    "width": int(frame.shape[1]),
                    "height": int(frame.shape[0]),
                    "sha256_bgr": sha256_bytes(frame.tobytes()),
                    "file": str(path),
                    "sha256_jpeg": sha256_bytes(path.read_bytes()),
                    "provenance": "LIVE_GO2_RTP_230.1.1.1_1720",
                }
            )
    finally:
        capture.release()

    if len(frames) != FRAME_COUNT:
        raise RuntimeError(f"captured {len(frames)} live frames, required {FRAME_COUNT}")
    return frames, metadata


def load_recorded_frames(source: Path, frames_dir: Path) -> tuple[list[Any], list[dict[str, Any]]]:
    import cv2

    paths = sorted(
        path for path in source.iterdir() if path.suffix.lower() in {".jpg", ".jpeg", ".png"}
    )
    if len(paths) != FRAME_COUNT:
        raise RuntimeError(f"--frames-dir must contain exactly {FRAME_COUNT} images, found {len(paths)}")
    frames: list[Any] = []
    metadata: list[dict[str, Any]] = []
    for index, source_path in enumerate(paths):
        frame = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            raise RuntimeError(f"could not decode frame: {source_path}")
        evidence_path = frames_dir / f"frame_{index:02d}{source_path.suffix.lower()}"
        evidence_path.write_bytes(source_path.read_bytes())
        frames.append(frame)
        metadata.append(
            {
                "index": index,
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "sha256_bgr": sha256_bytes(frame.tobytes()),
                "file": str(evidence_path),
                "sha256_file": sha256_bytes(evidence_path.read_bytes()),
                "source_file": str(source_path.resolve()),
                "provenance": "RECORDED_REAL_REPLAY_USER_SUPPLIED",
            }
        )
    return frames, metadata


def make_s2e_batch(frames: list[Any]) -> Any:
    import cv2
    import numpy as np
    from s2e_vlm_core.s2e_backend import S2EFrameContext, ros_rgb8_to_chw_float

    context = S2EFrameContext(context_size=FRAME_COUNT)
    for bgr in frames:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        context.append(
            ros_rgb8_to_chw_float(rgb.tobytes(), width=int(rgb.shape[1]), height=int(rgb.shape[0]))
        )
    batch = context.batch()
    if batch is None:
        raise RuntimeError("S2E context did not produce a batch after 11 frames")
    if batch.shape != (1, FRAME_COUNT, 3, IMAGE_SIZE, IMAGE_SIZE):
        raise RuntimeError(f"unexpected S2E batch shape: {batch.shape}")
    if batch.dtype != np.float32 or not np.isfinite(batch).all():
        raise RuntimeError("S2E batch is not finite float32")
    if float(batch.min()) < 0.0 or float(batch.max()) > 1.0:
        raise RuntimeError("S2E batch is outside [0,1]")
    return batch


def build_vlm_input(frame_path: Path, *, width: int, height: int, run_id: str) -> dict[str, Any]:
    return {
        "schema_version": "nav_vlm_input_v1",
        "task": {
            "task_mode": "PointNav",
            "coarse_goal": {
                "goal_type": "audit_only",
                "relative_bearing_deg": 0.0,
                "distance_m": 5.0,
                "instruction": "Select a visible collision-free local floor waypoint; do not actuate the robot.",
            },
        },
        "coordinate_frame": {
            "robot_frame": "base_link",
            "image_frame": "camera_optical_frame",
        },
        "robot_state": {"map_xy": [0.0, 0.0], "heading_rad": 0.0},
        "observation": {
            "mode": "current_only",
            "sequence_id": run_id,
            "frame_index": FRAME_COUNT - 1,
            "image_width": width,
            "image_height": height,
            "views": [
                {
                    "view_id": 0,
                    "view_type": "front",
                    "relative_heading_deg": 0.0,
                    "image": str(frame_path),
                }
            ],
        },
        "memory": {"runtime_state": {"force_front_view_waypoint": True}},
        "constraints": {
            "physical_actuation_allowed": False,
            "output_sink": "FILE_ONLY_AUDIT",
        },
    }


def query_vlm(vlm_input: dict[str, Any], *, timeout_s: float) -> tuple[dict[str, Any], dict[str, Any], list[str], dict[str, Any]]:
    from nav_memory_qwen.safety import sanitize_vlm_output
    from nav_memory_qwen.vlm_client import OpenAICompatibleVLMClient, auto_detect_served_model

    base_url = os.environ.get("QWEN_BASE_URL", "http://100.96.60.15:8000/v1").rstrip("/")
    model = os.environ.get("QWEN_MODEL", "auto")
    if not model or model.lower() in {"auto", "none"}:
        model = auto_detect_served_model(base_url, timeout_s=min(5.0, timeout_s))
    client = OpenAICompatibleVLMClient(
        base_url=base_url,
        api_key=os.environ.get("QWEN_API_KEY", "none"),
        model=model,
        timeout_s=timeout_s,
        temperature=0.0,
    )
    started = time.perf_counter()
    raw = client.decide(vlm_input)
    latency_s = time.perf_counter() - started
    safe, warnings = sanitize_vlm_output(raw, vlm_input)
    server = {"base_url": base_url, "model": model, "latency_s": latency_s}
    return raw, safe, warnings, server


def raw_vlm_contract_issues(raw: Any, vlm_input: dict[str, Any]) -> list[str]:
    """Return exact wire-schema issues before the safety sanitizer repairs output."""
    issues: list[str] = []
    if not isinstance(raw, dict):
        return ["raw output is not a JSON object"]
    if raw.get("schema_version") != "nav_vlm_waypoint_v1":
        issues.append("schema_version must be nav_vlm_waypoint_v1")
    action = raw.get("action")
    if action not in {"go", "rotate", "stop", "request_observation"}:
        issues.append(f"unsupported action: {action!r}")
        return issues

    fine_goal = raw.get("fine_goal")
    if not isinstance(fine_goal, dict):
        issues.append("fine_goal must be an object")
        return issues
    if action != "go":
        if fine_goal.get("valid") is not False:
            issues.append("non-go action requires fine_goal.valid=false")
        return issues

    observation = vlm_input.get("observation", {})
    width = int(observation.get("image_width", 0) or 0)
    height = int(observation.get("image_height", 0) or 0)
    available = {
        (view.get("view_id"), view.get("view_type"))
        for view in observation.get("views", []) or []
    }

    top_view_id = raw.get("selected_view_id")
    top_view_type = raw.get("selected_view_type")
    top_point = raw.get("selected_image_point")
    nested_point = fine_goal.get("point_px")
    if (top_view_id, top_view_type) not in available:
        issues.append("selected_view_id/type must identify an attached observation view")
    if not (
        isinstance(top_point, (list, tuple))
        and len(top_point) == 2
        and all(isinstance(value, int) and not isinstance(value, bool) for value in top_point)
        and 0 <= top_point[0] < width
        and 0 <= top_point[1] < height
    ):
        issues.append("selected_image_point must be an in-range integer [u_px,v_px]")
    if fine_goal.get("valid") is not True:
        issues.append("go action requires fine_goal.valid=true")
    if fine_goal.get("view_id") != top_view_id or fine_goal.get("view_type") != top_view_type:
        issues.append("fine_goal view must match top-level selected view")
    if nested_point != top_point:
        issues.append("fine_goal.point_px must match selected_image_point")
    return issues


def resolve_model(argument: Path | None) -> Path | None:
    candidates: list[Path] = []
    if argument is not None:
        candidates.append(argument)
    env_path = os.environ.get("E2E_MODEL_PATH")
    if env_path:
        candidates.append(Path(env_path))
    candidates.extend(
        [
            REPO_ROOT / "nav_model_zoo" / "S2E",
            REPO_ROOT / "Navigation-Model-Zoo-Public" / "S2E",
            Path("/models/s2e/S2E"),
        ]
    )
    for candidate in candidates:
        model = candidate / "s2e.onnx" if candidate.is_dir() else candidate
        if model.is_file() and model.name == "s2e.onnx":
            return model.resolve()
    return None


def controller_file_sink(points: list[tuple[float, float]]) -> dict[str, Any]:
    """Mirror the project's real controller law without publishing Twist."""
    lookahead_index = min(3, len(points) - 1)
    if lookahead_index < 0:
        return {"linear_x_mps": 0.0, "angular_z_radps": 0.0, "reason": "EMPTY_TRAJECTORY"}
    dx, dy = points[lookahead_index]
    heading_error = math.atan2(dy, dx)
    distance_error = math.hypot(dx, dy)
    linear = max(0.0, min(0.4, 0.5 * distance_error))
    angular = max(-0.8, min(0.8, (1.2 + 0.05) * heading_error))
    return {
        "linear_x_mps": linear,
        "angular_z_radps": angular,
        "lookahead_index": lookahead_index,
        "lookahead_xy_m": [dx, dy],
        "provenance": "REAL_S2E_TRAJECTORY_PROJECT_CONTROLLER_LAW",
        "sink": "FILE_ONLY_NO_ROS_NO_UDP",
        "published": False,
    }


def run_real_s2e(batch: Any, model: Path, *, device: str, goal_xy: tuple[float, float]) -> dict[str, Any]:
    import numpy as np
    import onnxruntime as ort
    from s2e_vlm_core.s2e_backend import OnnxS2ENavigator, convert_s2e_output_to_points

    navigator = OnnxS2ENavigator(model, device=device)
    providers = list(navigator._session.get_providers())
    if device == "cuda" and "CUDAExecutionProvider" not in providers:
        raise RuntimeError(f"CUDA was requested but ONNXRuntime providers are {providers}")
    started = time.perf_counter()
    trajectory, scores = navigator.inference_trajectory(
        batch,
        goal_xy=np.asarray(goal_xy, dtype=np.float32),
    )
    latency_s = time.perf_counter() - started
    points = convert_s2e_output_to_points(trajectory)
    return {
        "model": str(model),
        "model_sha256": sha256_file(model),
        "onnxruntime_version": ort.__version__,
        "requested_device": device,
        "providers": providers,
        "goal_xy_m": list(goal_xy),
        "goal_provenance": "FIXED_POINT_GOAL_FOR_MODEL_SMOKE_NOT_VLM_PIXEL_PROJECTION",
        "latency_s": latency_s,
        "trajectory_shape": list(trajectory.shape),
        "trajectory_sha256": sha256_bytes(trajectory.tobytes()),
        "scores_shape": list(scores.shape),
        "points_xy_m": [[x, y] for x, y in points],
        "finite": bool(np.isfinite(trajectory).all()),
        "controller_file_sink": controller_file_sink(points),
    }


def write_hash_manifest(run_dir: Path) -> None:
    lines: list[str] = []
    for path in sorted(item for item in run_dir.rglob("*") if item.is_file() and item.name != "SHA256SUMS"):
        lines.append(f"{sha256_bytes(path.read_bytes())}  {path.relative_to(run_dir)}")
    (run_dir / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    run_id = time.strftime("%Y%m%d_%H%M%S") + "_pixnav_s2e_no_actuation"
    run_dir = args.output_root.expanduser().resolve() / run_id
    frames_dir = run_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=False)

    report: dict[str, Any] = {
        "run_id": run_id,
        "created_unix_ns": time.time_ns(),
        "mode": "STRICT_ZERO_ACTUATION",
        "physical_actuation_allowed": False,
        "ros_publishers_created": False,
        "udp_command_senders_created": False,
        "stages": {},
        "overall": "BLOCKED",
    }

    processes = process_matches()
    ports = bound_udp_command_ports()
    report["safety"] = {
        "actuation_processes": processes["actuation"],
        "mapping_processes": processes["mapping"],
        "bound_command_udp_ports": ports,
    }
    if processes["actuation"] or ports:
        report["stages"]["safety_interlock"] = "FAIL"
        report["blocker"] = "actuation bridge/process or command UDP port is active"
        json_write(run_dir / "report.json", report)
        write_hash_manifest(run_dir)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2
    report["stages"]["safety_interlock"] = "PASS"

    runtime = {
        "python": sys.version,
        "opencv": None,
        "onnxruntime": None,
        "onnx_providers": [],
        "model": None,
    }
    try:
        import cv2

        runtime["opencv"] = cv2.__version__
    except Exception as exc:
        runtime["opencv_error"] = str(exc)
    try:
        import onnxruntime as ort

        runtime["onnxruntime"] = ort.__version__
        runtime["onnx_providers"] = ort.get_available_providers()
    except Exception as exc:
        runtime["onnxruntime_error"] = str(exc)
    model = resolve_model(args.model)
    runtime["model"] = str(model) if model else None
    report["runtime"] = runtime

    if args.preflight_only:
        report["stages"]["preflight"] = "PASS"
        report["overall"] = "PREFLIGHT_PASS"
        json_write(run_dir / "report.json", report)
        write_hash_manifest(run_dir)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(f"Evidence: {run_dir}")
        return 0

    if processes["mapping"]:
        report["stages"]["exclusive_sensor_use"] = "FAIL"
        report["blocker"] = "mapping is still active; stop map_headless.sh cleanly before this test"
        json_write(run_dir / "report.json", report)
        write_hash_manifest(run_dir)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 3
    report["stages"]["exclusive_sensor_use"] = "PASS"

    try:
        if args.frames_dir:
            frames, frame_metadata = load_recorded_frames(args.frames_dir.expanduser().resolve(), frames_dir)
        else:
            frames, frame_metadata = capture_live_frames(
                frames_dir,
                interval_s=args.frame_interval,
                timeout_s=args.camera_timeout,
            )
        json_write(run_dir / "frames.json", frame_metadata)
        report["stages"]["real_rgb_11_frames"] = "PASS"
        report["frame_provenance"] = frame_metadata[0]["provenance"]
    except Exception as exc:
        report["stages"]["real_rgb_11_frames"] = "FAIL"
        report["camera_error"] = str(exc)
        json_write(run_dir / "report.json", report)
        write_hash_manifest(run_dir)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 4

    try:
        batch = make_s2e_batch(frames)
        batch_meta = {
            "shape": list(batch.shape),
            "dtype": str(batch.dtype),
            "min": float(batch.min()),
            "max": float(batch.max()),
            "finite": True,
            "sha256": sha256_bytes(batch.tobytes()),
            "frame_order": "oldest_to_newest",
        }
        json_write(run_dir / "s2e_input.json", batch_meta)
        report["stages"]["s2e_input_contract"] = "PASS"
    except Exception as exc:
        report["stages"]["s2e_input_contract"] = "FAIL"
        report["s2e_input_error"] = str(exc)
        json_write(run_dir / "report.json", report)
        write_hash_manifest(run_dir)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 5

    if args.skip_vlm:
        report["stages"]["live_vlm_transport"] = "SKIPPED"
        report["stages"]["live_vlm_schema"] = "SKIPPED"
        report["stages"]["vlm_decision"] = "SKIPPED"
        report["stages"]["live_vlm"] = "SKIPPED"
    else:
        try:
            newest = Path(frame_metadata[-1]["file"])
            height, width = frames[-1].shape[:2]
            vlm_input = build_vlm_input(newest, width=int(width), height=int(height), run_id=run_id)
            json_write(run_dir / "vlm_input.json", vlm_input)
            raw, safe, warnings, server = query_vlm(vlm_input, timeout_s=args.vlm_timeout)
            contract_issues = raw_vlm_contract_issues(raw, vlm_input)
            json_write(run_dir / "vlm_raw.json", raw)
            json_write(run_dir / "vlm_sanitized.json", safe)
            json_write(
                run_dir / "vlm_runtime.json",
                {**server, "sanitizer_warnings": warnings, "raw_contract_issues": contract_issues},
            )
            raw_action = raw.get("action") if isinstance(raw, dict) else None
            safe_action = safe.get("action")
            report["stages"]["live_vlm_transport"] = "PASS"
            if warnings or contract_issues:
                report["stages"]["live_vlm_schema"] = "DEGRADED_SANITIZED"
                if raw_action != safe_action:
                    report["stages"]["vlm_decision"] = "REJECTED_TO_SAFE_FALLBACK"
                    report["stages"]["live_vlm"] = "DEGRADED_SAFE_FALLBACK"
                else:
                    report["stages"]["vlm_decision"] = "ACCEPTED_AFTER_SANITIZATION"
                    report["stages"]["live_vlm"] = "DEGRADED_SANITIZED"
            else:
                report["stages"]["live_vlm_schema"] = "PASS"
                report["stages"]["vlm_decision"] = "ACCEPTED"
                report["stages"]["live_vlm"] = "PASS"
            report["vlm_raw_action"] = raw_action
            report["vlm_action"] = safe.get("action")
            report["vlm_warnings"] = warnings
            report["vlm_raw_contract_issues"] = contract_issues
            report["vlm_output_applied_to_robot"] = False
        except Exception as exc:
            report["stages"]["live_vlm_transport"] = "FAIL"
            report["stages"]["live_vlm_schema"] = "NOT_RUN"
            report["stages"]["vlm_decision"] = "NOT_RUN"
            report["stages"]["live_vlm"] = "FAIL"
            report["vlm_error"] = str(exc)

    zero_hold = {
        "linear_x_mps": 0.0,
        "angular_z_radps": 0.0,
        "reason": "DEFAULT_ZERO_HOLD_UNLESS_REAL_S2E_TRAJECTORY_IS_AVAILABLE",
        "sink": "FILE_ONLY_NO_ROS_NO_UDP",
        "published": False,
    }
    json_write(run_dir / "command_audit.json", zero_hold)

    if model is None:
        report["stages"]["real_s2e_onnx"] = "BLOCKED_MODEL_MISSING"
        report["stages"]["controller_file_sink"] = "BLOCKED_NO_REAL_TRAJECTORY"
        report["blocker"] = "real s2e.onnx is not installed"
    elif runtime["onnxruntime"] is None:
        report["stages"]["real_s2e_onnx"] = "BLOCKED_ONNXRUNTIME_MISSING"
        report["stages"]["controller_file_sink"] = "BLOCKED_NO_REAL_TRAJECTORY"
        report["blocker"] = "onnxruntime is not installed in this runtime"
    else:
        try:
            s2e_result = run_real_s2e(
                batch,
                model,
                device=args.device,
                goal_xy=(args.goal_x, args.goal_y),
            )
            json_write(run_dir / "s2e_output.json", s2e_result)
            json_write(run_dir / "command_audit.json", s2e_result["controller_file_sink"])
            report["stages"]["real_s2e_onnx"] = "PASS"
            report["stages"]["controller_file_sink"] = "PASS"
        except Exception as exc:
            report["stages"]["real_s2e_onnx"] = "FAIL"
            report["stages"]["controller_file_sink"] = "BLOCKED_NO_VALID_TRAJECTORY"
            report["s2e_error"] = str(exc)

    report["stages"]["vlm_pixel_to_metric_goal"] = "BLOCKED_CAMERA_CALIBRATION_NOT_QUALIFIED"
    report["end_to_end_note"] = (
        "The ONNX smoke goal is a fixed metric point. It is not claimed as a calibrated "
        "projection of the VLM pixel goal. Physical autonomy remains NO-GO."
    )
    if report["stages"].get("real_s2e_onnx") == "PASS" and report["stages"].get("live_vlm") in {"PASS", "SKIPPED"}:
        report["overall"] = "REAL_S2E_MODEL_SMOKE_PASS_E2E_PROJECTION_BLOCKED"
    elif report["stages"].get("s2e_input_contract") == "PASS":
        report["overall"] = "PARTIAL_PASS_WITH_REPORTED_BLOCKERS"

    json_write(run_dir / "report.json", report)
    write_hash_manifest(run_dir)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"Evidence: {run_dir}")
    return 0 if report["overall"] != "BLOCKED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
