#!/usr/bin/env python3
"""One-cycle Go2 camera -> Docker VLM -> Jetson PixNav qualification.

This runner is deliberately file-only.  It creates no ROS publisher, Unitree
SDK client, command UDP sender, controller, or actuator process.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Any, Mapping, Optional


REPO_ROOT = Path(__file__).resolve().parent
PACKAGE_SOURCE = REPO_ROOT / "src" / "escape_nav_pixnav"
DOCKER_REPO_ROOT = Path("/workspace/go2_ws_antarctica")
DOCKER_PACKAGE_SOURCE = DOCKER_REPO_ROOT / "src" / "escape_nav_pixnav"
DEFAULT_OUTPUT_ROOT = Path.home() / ".ros" / "pixnav_live_runs"
DEFAULT_EXCHANGE_ROOT = REPO_ROOT / ".local-data" / "pixnav_live_exchange"
PIXNAV_CHECK = REPO_ROOT / "pixnav_check.py"
LIVE_SENSOR_GUARD = REPO_ROOT / "scratch" / "pixnav_live_sensor_guard.py"

if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

from escape_nav_pixnav import AuditJsonlSink, PixNavMacroAdapter, verify_audit_chain
from escape_nav_pixnav.contracts import sha256_canonical
from escape_nav_pixnav.event_ledger import CausalAdmissionLedger, EventStage, make_event
from escape_nav_pixnav.live_contract import (
    assess_vlm_grounding,
    live_decision_from_report,
    make_upstream_hold,
)
from escape_nav_pixnav.policy_runtime import FrozenPixNavRuntime
from escape_nav_pixnav.safety_admission import evaluate_safety_admission
from escape_nav_pixnav.vlm_grounding import validate_grounding


ACTUATION_MARKERS = (
    "host_bridge.py",
    "docker_bridge.py",
    "official_unitree_bridge.py",
    "python_direct_driver.py",
    "test_lab_micro_motion.py",
    "bringup_all_escape_nav.sh",
    "go2_driver",
)
MAPPING_MARKERS = (
    "map_headless.sh",
    "run_map.sh",
    "rtabmap",
    "rtabmap_viz",
    "icp_odometry",
    "go2_livo_sensor_bridge.py",
)
COMMAND_UDP_PORTS = (9090, 9091)
ROS_FOXY_SETUP = Path("/opt/ros/foxy/setup.bash")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Real-camera PixNav P6 one-cycle qualification with no actuation",
    )
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--exchange-root", type=Path, default=DEFAULT_EXCHANGE_ROOT)
    parser.add_argument("--history-frames", type=int, default=4)
    parser.add_argument("--frame-interval", type=float, default=0.20)
    parser.add_argument("--camera-timeout", type=float, default=15.0)
    parser.add_argument("--vlm-timeout", type=float, default=20.0)
    parser.add_argument(
        "--vlm-confidence-min",
        type=float,
        default=0.55,
        help="minimum self-reported VLM confidence required before PixNav",
    )
    parser.add_argument("--event-ttl", type=float, default=120.0)
    parser.add_argument(
        "--server-base",
        default=os.getenv("QWEN_BASE_URL", "http://100.96.60.15:8000/v1"),
    )
    parser.add_argument("--model", default=os.getenv("QWEN_MODEL", "auto"))
    parser.add_argument("--vlm-executor", choices=("docker", "host"), default="docker")
    parser.add_argument("--container", default="sdam_go2_container")
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument(
        "--pixnav-runtime",
        choices=("persistent", "subprocess"),
        default="persistent",
        help="persistent preloaded model is required for meaningful live latency",
    )
    return parser.parse_args()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.chmod(path, 0o600)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_head() -> Optional[str]:
    try:
        return subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def git_worktree_status() -> list[str]:
    try:
        output = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "status", "--short", "--untracked-files=all"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        return ["GIT_STATUS_UNAVAILABLE"]
    return [line for line in output.splitlines() if line]


def source_manifest() -> dict[str, Any]:
    paths = (
        REPO_ROOT / "pixnav_live_check.py",
        REPO_ROOT / "scratch" / "pixnav_live_sensor_guard.py",
        PACKAGE_SOURCE / "escape_nav_pixnav" / "adapter.py",
        PACKAGE_SOURCE / "escape_nav_pixnav" / "contracts.py",
        PACKAGE_SOURCE / "escape_nav_pixnav" / "event_ledger.py",
        PACKAGE_SOURCE / "escape_nav_pixnav" / "live_contract.py",
        PACKAGE_SOURCE / "escape_nav_pixnav" / "policy_runtime.py",
        PACKAGE_SOURCE / "escape_nav_pixnav" / "safety_admission.py",
        PACKAGE_SOURCE / "escape_nav_pixnav" / "vlm_grounding.py",
    )
    return {
        "schema_version": "go2_pixnav_live_source_manifest_v1",
        "git_head": git_head(),
        "files": [
            {
                "path": str(path.relative_to(REPO_ROOT)),
                "sha256": sha256_file(path) if path.is_file() else None,
            }
            for path in paths
        ],
    }


def process_matches() -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {"actuation": [], "mapping": []}
    own_pid = os.getpid()
    for process_dir in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(process_dir.name)
            if pid == own_pid:
                continue
            tokens = [token for token in (process_dir / "cmdline").read_bytes().split(b"\0") if token]
            basenames = {
                os.path.basename(token.decode("utf-8", errors="replace"))
                for token in tokens
            }
            command = b" ".join(tokens).decode("utf-8", errors="replace")
        except (OSError, ValueError):
            continue
        for marker in ACTUATION_MARKERS:
            if marker in basenames:
                result["actuation"].append({"pid": pid, "marker": marker, "command": command})
                break
        for marker in MAPPING_MARKERS:
            if marker in basenames:
                result["mapping"].append({"pid": pid, "marker": marker, "command": command})
                break
    return result


def bound_command_ports() -> list[int]:
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


def read_frame(capture: Any, *, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ok, frame = capture.read()
        if ok and frame is not None and frame.size > 0:
            return frame.copy()
    raise RuntimeError("LIVE_CAMERA_FRAME_TIMEOUT")


def save_frame(frame: Any, path: Path, *, index: int, role: str) -> dict[str, Any]:
    import cv2

    captured_monotonic_ns = time.monotonic_ns()
    captured_unix_ns = time.time_ns()
    if not cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
        raise RuntimeError(f"failed to save frame: {path}")
    os.chmod(path, 0o600)
    return {
        "index": index,
        "role": role,
        "capture_monotonic_ns": captured_monotonic_ns,
        "capture_unix_ns": captured_unix_ns,
        "width": int(frame.shape[1]),
        "height": int(frame.shape[0]),
        "file": str(path),
        "sha256_file": sha256_file(path),
        "provenance": "LIVE_GO2_RTP_230.1.1.1_1720",
    }


def docker_runtime(container: str) -> dict[str, Any]:
    running = subprocess.run(
        ["docker", "inspect", "--format", "{{.State.Running}}", container],
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    if running.returncode != 0 or running.stdout.strip() != "true":
        raise RuntimeError(f"DOCKER_CONTAINER_NOT_RUNNING:{container}")
    details = subprocess.run(
        ["docker", "inspect", "--format", "{{.Config.Image}}|{{.Id}}", container],
        check=True,
        capture_output=True,
        text=True,
        timeout=5.0,
    ).stdout.strip()
    image, _, container_id = details.partition("|")
    return {"container": container, "image": image, "container_id": container_id}


def ros_foxy_environment() -> dict[str, str]:
    if not ROS_FOXY_SETUP.is_file():
        raise RuntimeError("ROS_FOXY_SETUP_MISSING")
    completed = subprocess.run(
        [
            "bash",
            "--noprofile",
            "--norc",
            "-c",
            f"source {ROS_FOXY_SETUP} >/dev/null 2>&1 && env -0",
        ],
        check=True,
        capture_output=True,
        timeout=10.0,
    )
    environment: dict[str, str] = {}
    for entry in completed.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        environment[key.decode("utf-8")] = value.decode("utf-8", errors="surrogateescape")
    existing_pythonpath = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = f"{PACKAGE_SOURCE}:{existing_pythonpath}"
    return environment


def start_live_sensor_guard(output_path: Path) -> tuple[subprocess.Popen[str], dict[str, Any]]:
    command = [
        sys.executable,
        str(LIVE_SENSOR_GUARD),
        "--output",
        str(output_path),
        "--duration",
        "120.0",
        "--interval",
        "0.05",
    ]
    started_ns = time.monotonic_ns()
    process = subprocess.Popen(
        command,
        cwd=str(REPO_ROOT),
        env=ros_foxy_environment(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.monotonic() + 8.0
    last_status = "NO_SNAPSHOT"
    while time.monotonic() < deadline:
        if process.poll() is not None:
            stdout, stderr = process.communicate()
            raise RuntimeError(f"LIVE_SENSOR_GUARD_EXITED:{process.returncode}:{stderr[-1000:]}")
        if output_path.is_file():
            try:
                value = json.loads(output_path.read_text(encoding="utf-8"))
                last_status = str(value.get("status"))
                if last_status == "PASS_LIVE_L2_ODOM_SNAPSHOT":
                    return process, {
                        "command": command,
                        "started_monotonic_ns": started_ns,
                        "ready_monotonic_ns": time.monotonic_ns(),
                        "ready_status": last_status,
                        "ros_publishers_created": 0,
                        "actuation_calls": 0,
                    }
            except (OSError, json.JSONDecodeError):
                pass
        time.sleep(0.05)
    process.send_signal(signal.SIGINT)
    stdout, stderr = process.communicate(timeout=5.0)
    raise RuntimeError(f"LIVE_SENSOR_GUARD_NOT_READY:{last_status}:{stderr[-1000:]}")


def stop_live_sensor_guard(
    process: subprocess.Popen[str],
    telemetry: dict[str, Any],
) -> dict[str, Any]:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        stdout, stderr = process.communicate(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        stdout, stderr = process.communicate(timeout=5.0)
    return {
        **telemetry,
        "finished_monotonic_ns": time.monotonic_ns(),
        "returncode": process.returncode,
        "stdout": stdout[-2000:],
        "stderr": stderr[-4000:],
    }


def query_vlm_subprocess(
    *,
    executor: str,
    container: str,
    host_image: Path,
    container_image: Path,
    width: int,
    height: int,
    server_base: str,
    model: str,
    timeout_s: float,
    confidence_min: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    module_args = [
        "-m",
        "escape_nav_pixnav.vlm_grounding",
        "--image",
        str(container_image if executor == "docker" else host_image),
        "--width",
        str(width),
        "--height",
        str(height),
        "--base-url",
        server_base,
        "--model",
        model,
        "--timeout",
        str(timeout_s),
        "--confidence-min",
        str(confidence_min),
    ]
    if executor == "docker":
        command = ["docker", "exec", "-e", f"PYTHONPATH={DOCKER_PACKAGE_SOURCE}"]
        if os.getenv("QWEN_API_KEY"):
            command.extend(["-e", "QWEN_API_KEY"])
        command.extend([container, "python3", *module_args])
        environment = None
    else:
        command = [sys.executable, *module_args]
        environment = {**os.environ, "PYTHONPATH": str(PACKAGE_SOURCE)}
    started_ns = time.monotonic_ns()
    completed = subprocess.run(
        command,
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout_s + 10.0,
        env=environment,
    )
    finished_ns = time.monotonic_ns()
    telemetry = {
        "executor": executor,
        "command": command,
        "started_monotonic_ns": started_ns,
        "finished_monotonic_ns": finished_ns,
        "returncode": completed.returncode,
        "stderr": completed.stderr[-4000:],
    }
    if completed.returncode != 0:
        raise RuntimeError(f"VLM_SUBPROCESS_FAILED:{completed.stderr[-1000:]}")
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("VLM_SUBPROCESS_EMPTY_OUTPUT")
    envelope = json.loads(lines[-1])
    if not isinstance(envelope, dict):
        raise RuntimeError("VLM_SUBPROCESS_OUTPUT_NOT_OBJECT")
    return envelope, telemetry


def find_pixnav_report(output_root: Path) -> Path:
    reports = sorted(output_root.glob("*/report.json"), key=lambda path: path.stat().st_mtime_ns)
    if len(reports) != 1:
        raise RuntimeError(f"expected one PixNav report, found {len(reports)}")
    return reports[0]


def run_pixnav(
    *,
    frames_dir: Path,
    frame_count: int,
    goal_u: int,
    goal_v: int,
    device: str,
    output_root: Path,
) -> tuple[dict[str, Any], dict[str, Any], Path]:
    command = [
        sys.executable,
        str(PIXNAV_CHECK),
        "--device",
        device,
        "--frames-dir",
        str(frames_dir),
        "--max-frames",
        str(frame_count),
        "--goal-frame-index",
        "0",
        "--history-start-index",
        "0",
        "--goal-u",
        str(goal_u),
        "--goal-v",
        str(goal_v),
        "--output-root",
        str(output_root),
    ]
    started_ns = time.monotonic_ns()
    completed = subprocess.run(
        command,
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=180.0,
    )
    finished_ns = time.monotonic_ns()
    report_path = find_pixnav_report(output_root)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    telemetry = {
        "command": command,
        "started_monotonic_ns": started_ns,
        "finished_monotonic_ns": finished_ns,
        "returncode": completed.returncode,
        "stdout": completed.stdout[-4000:],
        "stderr": completed.stderr[-4000:],
        "report": str(report_path),
        "report_sha256": sha256_file(report_path),
    }
    if completed.returncode != 0 or report.get("overall") != "PASS_FILE_ONLY_REPLAY":
        raise RuntimeError(f"PIXNAV_INFERENCE_FAILED:{report.get('overall')}")
    return report, telemetry, report_path


def run_pixnav_persistent(
    runtime: FrozenPixNavRuntime,
    *,
    frame_metadata: list[dict[str, Any]],
    goal_u: int,
    goal_v: int,
    output_root: Path,
) -> tuple[dict[str, Any], dict[str, Any], Path]:
    report = runtime.infer_files(
        [Path(item["file"]) for item in frame_metadata],
        goal_frame_index=0,
        history_start_index=0,
        goal_u=goal_u,
        goal_v=goal_v,
    )
    report_dir = output_root / "persistent"
    report_dir.mkdir(mode=0o700, exist_ok=False)
    report_path = report_dir / "report.json"
    write_json(report_path, report)
    telemetry = {
        "runtime": "persistent_in_process_file_only",
        "started_monotonic_ns": report["started_monotonic_ns"],
        "finished_monotonic_ns": report["finished_monotonic_ns"],
        "returncode": 0,
        "report": str(report_path),
        "report_sha256": sha256_file(report_path),
        "subprocess_created": False,
        "actuation_calls": 0,
    }
    if report.get("overall") != "PASS_PERSISTENT_FILE_ONLY_INFERENCE":
        raise RuntimeError(f"PIXNAV_INFERENCE_FAILED:{report.get('overall')}")
    return report, telemetry, report_path


def write_hash_manifest(run_dir: Path) -> None:
    lines = []
    for path in sorted(item for item in run_dir.rglob("*") if item.is_file() and item.name != "SHA256SUMS"):
        lines.append(f"{sha256_file(path)}  {path.relative_to(run_dir)}")
    manifest = run_dir / "SHA256SUMS"
    manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.chmod(manifest, 0o600)


def main() -> int:
    args = parse_args()
    if args.history_frames < 1 or args.history_frames > 10:
        raise SystemExit("--history-frames must be in [1, 10]")
    if not math.isfinite(args.frame_interval) or args.frame_interval < 0.0:
        raise SystemExit("--frame-interval must be finite and nonnegative")
    if not math.isfinite(args.event_ttl) or args.event_ttl <= 0.0:
        raise SystemExit("--event-ttl must be positive and finite")
    if not math.isfinite(args.vlm_confidence_min) or not 0.0 <= args.vlm_confidence_min <= 1.0:
        raise SystemExit("--vlm-confidence-min must be finite and in [0, 1]")

    os.umask(0o077)
    run_id = time.strftime("%Y%m%d_%H%M%S_pixnav_live_no_actuation")
    run_dir = args.output_root.expanduser().resolve() / run_id
    frames_dir = run_dir / "frames"
    pixnav_output_root = run_dir / "pixnav_runs"
    exchange_dir = args.exchange_root.expanduser().resolve() / run_id
    frames_dir.mkdir(parents=True, mode=0o700, exist_ok=False)
    pixnav_output_root.mkdir(mode=0o700)
    exchange_dir.mkdir(parents=True, mode=0o700, exist_ok=False)

    report: dict[str, Any] = {
        "schema_version": "go2_pixnav_live_p6_report_v2",
        "run_id": run_id,
        "created_unix_ns": time.time_ns(),
        "git_head": git_head(),
        "git_worktree_status": git_worktree_status(),
        "mode": "LIVE_REAL_CAMERA_STRICT_ZERO_ACTUATION",
        "physical_actuation_allowed": False,
        "ros_publishers_created": 0,
        "unitree_sdk_clients_created": 0,
        "udp_command_senders_created": 0,
        "controller_processes_started": 0,
        "vlm_executor": args.vlm_executor,
        "vlm_confidence_min": args.vlm_confidence_min,
        "pixnav_runtime_kind": args.pixnav_runtime,
        "stages": {},
        "overall": "BLOCKED",
        "motion_readiness": False,
    }
    frame_metadata: list[dict[str, Any]] = []
    ledger = CausalAdmissionLedger()
    causal_events: list[dict[str, Any]] = []
    event_sequence = 0
    last_event_hash = "0" * 64
    causal_id = "0" * 64
    proposal = None
    live_decision = None
    pixnav_runtime = None
    sensor_guard_process = None
    sensor_guard_telemetry = None
    sensor_snapshot_path = run_dir / "l2_odom_safety_snapshot.json"
    write_json(run_dir / "source_manifest.json", source_manifest())

    def append_event(stage: EventStage, payload: Mapping[str, Any]) -> None:
        nonlocal event_sequence, last_event_hash
        event_at_ns = time.monotonic_ns()
        event = make_event(
            causal_id_sha256=causal_id,
            sequence_id=event_sequence,
            stage=stage,
            event_at_ns=event_at_ns,
            expires_at_ns=event_at_ns + int(args.event_ttl * 1_000_000_000),
            payload_sha256=sha256_canonical(dict(payload)),
            parent_event_sha256=last_event_hash,
        )
        admission = ledger.append(event, now_ns=event_at_ns)
        causal_events.append(
            {
                "event": event.to_dict(),
                "event_sha256": event.sha256,
                "admission": admission.__dict__,
            }
        )
        if not admission.accepted:
            raise RuntimeError(f"CAUSAL_LEDGER_REJECTED:{admission.reason}")
        last_event_hash = event.sha256
        event_sequence += 1

    try:
        safety_processes = process_matches()
        command_ports = bound_command_ports()
        report["safety"] = {
            "actuation_processes": safety_processes["actuation"],
            "mapping_processes": safety_processes["mapping"],
            "bound_command_udp_ports": command_ports,
        }
        if safety_processes["actuation"] or safety_processes["mapping"] or command_ports:
            raise RuntimeError("EXCLUSIVE_NO_ACTUATION_INTERLOCK_FAILED")
        report["stages"]["safety_interlock"] = "PASS"

        if args.vlm_executor == "docker":
            report["docker"] = docker_runtime(args.container)
        report["stages"]["vlm_executor_preflight"] = "PASS"

        if args.pixnav_runtime == "persistent":
            pixnav_runtime = FrozenPixNavRuntime(device=args.device)
            report["pixnav_runtime"] = pixnav_runtime.metadata
            report["pixnav_runtime_warmup"] = pixnav_runtime.warmup(
                sequence_length=args.history_frames + 1,
            )
            report["stages"]["pixnav_runtime_preflight"] = "PASS_PRELOADED_WARM"
        else:
            report["stages"]["pixnav_runtime_preflight"] = "PASS_SUBPROCESS_COMPATIBILITY"

        sensor_guard_process, sensor_guard_telemetry = start_live_sensor_guard(
            sensor_snapshot_path
        )
        report["stages"]["live_l2_odom_guard"] = "PASS_READ_ONLY_SUBSCRIPTIONS"

        opencv_lib = "/home/unitree/opencv_build/opencv/build/lib"
        current_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
        if opencv_lib not in current_ld_path.split(":"):
            os.environ["LD_LIBRARY_PATH"] = f"{opencv_lib}:/usr/local/lib:{current_ld_path}"
        import cv2

        capture = cv2.VideoCapture(camera_pipeline(), cv2.CAP_GSTREAMER)
        if not capture.isOpened():
            raise RuntimeError("LIVE_CAMERA_PIPELINE_OPEN_FAILED")
        try:
            goal_frame = read_frame(capture, timeout_s=args.camera_timeout)
            goal_path = frames_dir / "frame_00_goal.jpg"
            goal_meta = save_frame(goal_frame, goal_path, index=0, role="vlm_capture_view_goal")
            frame_metadata.append(goal_meta)
            causal_id = sha256_canonical(
                {"run_id": run_id, "goal_frame_sha256": goal_meta["sha256_file"]}
            )
            append_event(EventStage.FRAME_CAPTURED, goal_meta)
            report["stages"]["live_goal_frame"] = "PASS"

            exchange_image_host = exchange_dir / "frame_00_goal.jpg"
            shutil.copy2(str(goal_path), str(exchange_image_host))
            os.chmod(exchange_image_host, 0o600)
            exchange_image_container = DOCKER_REPO_ROOT / exchange_image_host.relative_to(REPO_ROOT)
            append_event(
                EventStage.VLM_SUBMITTED,
                {
                    "goal_frame_sha256": goal_meta["sha256_file"],
                    "executor": args.vlm_executor,
                    "server_base": args.server_base,
                    "requested_model": args.model,
                },
            )
            envelope, vlm_process = query_vlm_subprocess(
                executor=args.vlm_executor,
                container=args.container,
                host_image=exchange_image_host,
                container_image=exchange_image_container,
                width=goal_meta["width"],
                height=goal_meta["height"],
                server_base=args.server_base,
                model=args.model,
                timeout_s=args.vlm_timeout,
                confidence_min=args.vlm_confidence_min,
            )
            write_json(run_dir / "vlm_transport.json", {**envelope, "raw": "stored_in_vlm_raw.json"})
            write_json(run_dir / "vlm_process.json", vlm_process)
            write_json(run_dir / "vlm_raw.json", envelope.get("raw"))
            grounding = validate_grounding(
                envelope.get("raw"),
                width=goal_meta["width"],
                height=goal_meta["height"],
            )
            write_json(run_dir / "vlm_validated.json", grounding)
            vlm_admitted, vlm_admission_reason = assess_vlm_grounding(
                grounding,
                confidence_min=args.vlm_confidence_min,
            )
            report["vlm_admission"] = {
                "admitted_to_pixnav": vlm_admitted,
                "reason": vlm_admission_reason,
                "confidence": grounding["confidence"],
                "confidence_min": args.vlm_confidence_min,
            }
            append_event(EventStage.VLM_COMPLETED, {"transport": envelope, "validated": grounding})
            report["stages"]["docker_to_server_vlm"] = "PASS"
            report["stages"]["strict_vlm_semantics"] = "PASS"

            if vlm_admitted:
                next_capture = time.monotonic()
                for offset in range(1, args.history_frames + 1):
                    while time.monotonic() < next_capture:
                        time.sleep(min(0.01, next_capture - time.monotonic()))
                    history_frame = read_frame(capture, timeout_s=args.camera_timeout)
                    history_path = frames_dir / f"frame_{offset:02d}_history.jpg"
                    frame_metadata.append(
                        save_frame(
                            history_frame,
                            history_path,
                            index=offset,
                            role="post_goal_observation",
                        )
                    )
                    next_capture = time.monotonic() + args.frame_interval
        finally:
            capture.release()
        write_json(run_dir / "frames.json", frame_metadata)
        report["stages"]["post_capture_history"] = (
            "PASS" if vlm_admitted else "SKIPPED_UPSTREAM_HOLD"
        )

        if not vlm_admitted:
            proposal = make_upstream_hold(
                event_id=f"p6.{run_id}.vlm_hold",
                sequence_id=0,
                source_frame_sha256=goal_meta["sha256_file"],
                reason=vlm_admission_reason,
            )
            append_event(
                EventStage.PIXNAV_COMPLETED,
                {"executed": False, "reason": vlm_admission_reason},
            )
            report["stages"]["pixnav_cuda"] = f"SKIPPED_{vlm_admission_reason}"
        else:
            goal_u, goal_v = grounding["selected_image_point"]
            if pixnav_runtime is not None:
                pixnav_report, pixnav_process, pixnav_report_path = run_pixnav_persistent(
                    pixnav_runtime,
                    frame_metadata=frame_metadata,
                    goal_u=goal_u,
                    goal_v=goal_v,
                    output_root=pixnav_output_root,
                )
            else:
                pixnav_report, pixnav_process, pixnav_report_path = run_pixnav(
                    frames_dir=frames_dir,
                    frame_count=len(frame_metadata),
                    goal_u=goal_u,
                    goal_v=goal_v,
                    device=args.device,
                    output_root=pixnav_output_root,
                )
            write_json(run_dir / "pixnav_process.json", pixnav_process)
            live_decision = live_decision_from_report(
                pixnav_report,
                frame_metadata,
                run_id=run_id,
                sequence_id=0,
                inferred_at_ns=int(pixnav_process["finished_monotonic_ns"]),
            )
            write_json(run_dir / "pixnav_live_decision.json", live_decision)
            proposal = PixNavMacroAdapter().adapt(
                live_decision,
                evaluated_at_ns=time.monotonic_ns(),
            )
            append_event(
                EventStage.PIXNAV_COMPLETED,
                {
                    "executed": True,
                    "report": str(pixnav_report_path),
                    "report_sha256": sha256_file(pixnav_report_path),
                    "live_decision": live_decision,
                },
            )
            report["stages"]["pixnav_cuda"] = "PASS"

        sensor_snapshot = json.loads(sensor_snapshot_path.read_text(encoding="utf-8"))
        safety_admission = evaluate_safety_admission(
            proposal.to_dict(),
            sensor_snapshot,
            evaluated_at_ns=time.monotonic_ns(),
            decision_observed_at_ns=(
                int(live_decision["observed_at_ns"]) if live_decision is not None else None
            ),
            decision_inferred_at_ns=(
                int(live_decision["inferred_at_ns"]) if live_decision is not None else None
            ),
            operator_enabled=False,
            estop_clear=False,
            global_localization_available=False,
        )
        write_json(run_dir / "safety_admission.json", safety_admission)
        report["safety_admission"] = safety_admission
        report["stages"]["safety_admission_p7"] = (
            "PASS_CANDIDATE_ONLY_NO_ACTUATION"
            if safety_admission["admitted_to_gateway"]
            else "PASS_EVALUATED_FAIL_CLOSED"
        )
        report["gateway_candidate"] = safety_admission["admitted_to_gateway"]

        sink = AuditJsonlSink(run_dir / "macro_actions.jsonl")
        record_hash = sink.append(proposal)
        audit_result = verify_audit_chain(run_dir / "macro_actions.jsonl")
        append_event(
            EventStage.MACRO_AUDITED,
            {"proposal": proposal.to_dict(), "record_sha256": record_hash, "audit": audit_result},
        )
        report["stages"]["file_only_macro_audit"] = "PASS"
        report["proposal"] = proposal.to_dict()
        report["proposal_record_sha256"] = record_hash
        report["audit"] = audit_result
        report["causal_chain_complete"] = True
        report["overall"] = (
            "PASS_ONE_CYCLE_LIVE_CHAIN_NO_ACTUATION"
            if report["stages"].get("pixnav_cuda") == "PASS"
            else "PASS_LIVE_UPSTREAM_HOLD_FILE_ONLY"
        )
    except Exception as error:
        report["error_type"] = type(error).__name__
        report["error"] = str(error)[:2000]
        report["overall"] = "SAFE_HOLD_LIVE_CHAIN_BLOCKED"
        if frame_metadata and proposal is None:
            proposal = make_upstream_hold(
                event_id=f"p6.{run_id}.blocked",
                sequence_id=0,
                source_frame_sha256=frame_metadata[0]["sha256_file"],
                reason=f"UPSTREAM_BLOCKED:{type(error).__name__}",
            )
            try:
                sink = AuditJsonlSink(run_dir / "macro_actions.jsonl")
                report["proposal_record_sha256"] = sink.append(proposal)
                report["proposal"] = proposal.to_dict()
                report["audit"] = verify_audit_chain(run_dir / "macro_actions.jsonl")
            except Exception as sink_error:
                report["audit_error"] = str(sink_error)[:1000]
        report["deadman_holds"] = ledger.deadman_holds(
            now_ns=time.monotonic_ns() + int(args.event_ttl * 1_000_000_000) + 1
        )
    finally:
        if sensor_guard_process is not None and sensor_guard_telemetry is not None:
            try:
                sensor_guard_result = stop_live_sensor_guard(
                    sensor_guard_process,
                    sensor_guard_telemetry,
                )
                write_json(run_dir / "sensor_guard_process.json", sensor_guard_result)
                report["sensor_guard_process"] = sensor_guard_result
            except Exception as sensor_guard_error:
                report["sensor_guard_stop_error"] = str(sensor_guard_error)[:1000]
        report["frames_captured"] = len(frame_metadata)
        report["causal_id_sha256"] = causal_id
        report["actuation_permitted"] = False
        report["claim_scope"] = (
            "One-cycle live real-camera, Docker VLM transport, frozen PixNav and file-only "
            "proposal causality plus read-only L2/odom P7 evaluation only; not a 10-minute "
            "soak, physical operator-enable/E-stop, P8 gateway, controller, global localization "
            "or physical navigation proof."
        )
        write_json(run_dir / "causal_events.json", causal_events)
        write_json(run_dir / "causal_ledger.json", ledger.snapshot())
        write_json(run_dir / "report.json", report)
        write_hash_manifest(run_dir)
        print(f"PixNav live P6: {report['overall']}")
        print("Actuation permitted: false")
        if proposal is not None:
            print(f"Proposal: {proposal.proposal_kind.value} ({proposal.reason})")
        print(f"Evidence: {run_dir}")
    return 0 if report["overall"].startswith("PASS_") else 2


if __name__ == "__main__":
    raise SystemExit(main())
