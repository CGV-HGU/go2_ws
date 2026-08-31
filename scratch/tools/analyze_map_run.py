#!/usr/bin/env python3
"""Generate a read-only RTAB-Map run report without touching the source DB."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import struct
import time
from typing import Any, Iterable, Sequence
import zlib


DEFAULT_RUN = Path.home() / ".ros" / "rtabmap_runs" / "latest"
DEFAULT_OUTPUT_ROOT = Path.home() / ".ros" / "rtabmap_analysis_runs"
LINK_NAMES = {
    0: "neighbor",
    1: "global_closure",
    2: "local_space_closure",
    3: "local_time_closure",
    4: "user_closure",
    5: "virtual_closure",
    6: "neighbor_merged",
    7: "pose_prior",
    8: "landmark",
    9: "gravity",
}
RTAB_TIME_RE = re.compile(r"RTAB-Map=([0-9]+(?:\.[0-9]+)?)s")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _decode_transform(blob: bytes) -> tuple[float, float, float]:
    if len(blob) == 48:
        values = struct.unpack("<12f", blob)
    elif len(blob) == 96:
        values = struct.unpack("<12d", blob)
    else:
        raise ValueError(f"unsupported Transform blob size: {len(blob)}")
    return float(values[3]), float(values[7]), float(values[11])


def _decompress(blob: bytes | None) -> bytes:
    if not blob:
        return b""
    try:
        return zlib.decompress(blob)
    except zlib.error:
        return blob


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _round(value: float | None, digits: int = 6) -> float | None:
    return round(value, digits) if value is not None else None


def trajectory_metrics(
    points: Iterable[tuple[int, float, float, float]],
) -> dict[str, Any]:
    values = list(points)
    if not values:
        return {
            "pose_count": 0,
            "path_xy_m": None,
            "path_3d_m": None,
            "endpoint_xy_gap_m": None,
            "z_span_m": None,
        }
    path_xy = 0.0
    path_3d = 0.0
    for previous, current in zip(values, values[1:]):
        dx = current[1] - previous[1]
        dy = current[2] - previous[2]
        dz = current[3] - previous[3]
        path_xy += math.hypot(dx, dy)
        path_3d += math.sqrt(dx * dx + dy * dy + dz * dz)
    endpoint = math.hypot(values[-1][1] - values[0][1], values[-1][2] - values[0][2])
    zs = [item[3] for item in values]
    return {
        "pose_count": len(values),
        "first_node_id": values[0][0],
        "last_node_id": values[-1][0],
        "path_xy_m": _round(path_xy),
        "path_3d_m": _round(path_3d),
        "endpoint_xy_gap_m": _round(endpoint),
        "z_min_m": _round(min(zs)),
        "z_max_m": _round(max(zs)),
        "z_span_m": _round(max(zs) - min(zs)),
    }


def _load_manifest(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def _analyze_runtime_log(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"available": False}
    text = path.read_text(encoding="utf-8", errors="replace")
    durations = [float(value) for value in RTAB_TIME_RE.findall(text)]
    warnings = {
        "imu_interpolation": text.count("cannot interpolate imu transform"),
        "negative_hessian_index": text.count("negative hessian index"),
        "loop_rejected": len(re.findall(r"(?:Rejected loop closure|Loop closure .* rejected)", text)),
        "odometry_lost": len(re.findall(r"odometry[^\n]{0,80}lost", text, re.IGNORECASE)),
        "optimizer_failure": len(re.findall(r"optimizer[^\n]{0,80}(?:failed|failure)", text, re.IGNORECASE)),
        "nan_or_inf": len(re.findall(r"\b(?:nan|inf)\b", text, re.IGNORECASE)),
    }
    return {
        "available": True,
        "sha256": sha256_file(path),
        "rtabmap_update_count": len(durations),
        "rtabmap_time_mean_s": _round(sum(durations) / len(durations)) if durations else None,
        "rtabmap_time_p95_s": _round(_percentile(durations, 0.95)),
        "rtabmap_time_max_s": _round(max(durations)) if durations else None,
        "rtabmap_time_over_0_5s": sum(value > 0.5 for value in durations),
        "warnings": warnings,
    }


def _decode_optimized(
    ids_blob: bytes | None,
    poses_blob: bytes | None,
) -> list[tuple[int, float, float, float]]:
    ids_data = _decompress(ids_blob)
    poses_data = _decompress(poses_blob)
    if not ids_data and not poses_data:
        return []
    if len(ids_data) % 4 != 0 or len(poses_data) % 48 != 0:
        raise ValueError("Admin optimized pose cache has unexpected byte lengths")
    ids = struct.unpack(f"<{len(ids_data) // 4}i", ids_data)
    if len(ids) != len(poses_data) // 48:
        raise ValueError("Admin opt_ids and opt_poses count mismatch")
    points = []
    for index, node_id in enumerate(ids):
        start = index * 48
        x, y, z = _decode_transform(poses_data[start : start + 48])
        points.append((int(node_id), x, y, z))
    return points


def analyze_run(source: Path) -> dict[str, Any]:
    resolved = source.expanduser().resolve()
    run_dir = resolved if resolved.is_dir() else resolved.parent
    db_path = run_dir / "rtabmap.db" if resolved.is_dir() else resolved
    if not db_path.is_file():
        raise FileNotFoundError(f"RTAB-Map DB not found: {db_path}")
    db_hash_before = sha256_file(db_path)
    uri = f"file:{db_path}?mode=ro&immutable=1"
    connection = sqlite3.connect(uri, uri=True)
    try:
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        node_rows = connection.execute(
            "SELECT id, weight, stamp, pose FROM Node ORDER BY id"
        ).fetchall()
        raw_points = []
        raw_by_id: dict[int, tuple[int, float, float, float]] = {}
        stamps = []
        weights = Counter()
        invalid_pose_count = 0
        for node_id, weight, stamp, pose_blob in node_rows:
            weights[str(weight)] += 1
            if stamp is not None:
                stamps.append(float(stamp))
            if pose_blob is None:
                invalid_pose_count += 1
                continue
            try:
                x, y, z = _decode_transform(pose_blob)
            except ValueError:
                invalid_pose_count += 1
                continue
            point = (int(node_id), x, y, z)
            raw_points.append(point)
            raw_by_id[int(node_id)] = point

        link_rows = connection.execute(
            "SELECT type, COUNT(*) FROM Link GROUP BY type ORDER BY type"
        ).fetchall()
        unique_link_rows = connection.execute(
            """
            SELECT type, COUNT(*) FROM (
              SELECT DISTINCT
                CASE WHEN from_id < to_id THEN from_id ELSE to_id END AS low_id,
                CASE WHEN from_id < to_id THEN to_id ELSE from_id END AS high_id,
                type
              FROM Link
            ) GROUP BY type ORDER BY type
            """
        ).fetchall()
        type1_pairs = connection.execute(
            """
            SELECT DISTINCT
              CASE WHEN from_id < to_id THEN from_id ELSE to_id END AS low_id,
              CASE WHEN from_id < to_id THEN to_id ELSE from_id END AS high_id
            FROM Link WHERE type=1 ORDER BY low_id, high_id
            """
        ).fetchall()
        admin = connection.execute(
            "SELECT version, opt_ids, opt_poses FROM Admin LIMIT 1"
        ).fetchone()
        optimized = _decode_optimized(admin[1], admin[2]) if admin else []
    finally:
        connection.close()
    db_hash_after = sha256_file(db_path)
    if db_hash_before != db_hash_after:
        raise RuntimeError("source DB hash changed during read-only analysis")

    optimized_ids = [item[0] for item in optimized]
    raw_subset = [raw_by_id[node_id] for node_id in optimized_ids if node_id in raw_by_id]
    corrections = []
    for optimized_point in optimized:
        raw_point = raw_by_id.get(optimized_point[0])
        if raw_point is not None:
            corrections.append(
                math.hypot(optimized_point[1] - raw_point[1], optimized_point[2] - raw_point[2])
            )
    raw_metrics = trajectory_metrics(raw_points)
    raw_subset_metrics = trajectory_metrics(raw_subset)
    optimized_metrics = trajectory_metrics(optimized)
    link_counts = {
        str(link_type): {
            "name": LINK_NAMES.get(int(link_type), "unknown"),
            "rows": int(count),
            "unique_pairs": 0,
        }
        for link_type, count in link_rows
    }
    for link_type, count in unique_link_rows:
        link_counts.setdefault(
            str(link_type),
            {"name": LINK_NAMES.get(int(link_type), "unknown"), "rows": 0},
        )["unique_pairs"] = int(count)
    runtime = _analyze_runtime_log(run_dir / "runtime.log")
    manifest = _load_manifest(run_dir / "run_manifest.txt")
    type1_unique = link_counts.get("1", {}).get("unique_pairs", 0)
    type2_unique = link_counts.get("2", {}).get("unique_pairs", 0)
    automated_checks = {
        "database_integrity_ok": integrity == "ok",
        "source_db_unchanged": db_hash_before == db_hash_after,
        "manifest_planar_3dof": (
            manifest.get("profile") == "planar3dof"
            and manifest.get("Reg/Force3DoF") == "true"
            and manifest.get("Icp/Force4DoF") == "false"
        ),
        "manifest_type2_disabled": manifest.get("RGBD/ProximityBySpace") == "false",
        "manifest_detection_rate_2hz": manifest.get("Rtabmap/DetectionRate") == "2.0",
        "manifest_icp_voxel_0_05m": manifest.get("Icp/VoxelSize") == "0.05",
        "optimized_cache_available": bool(optimized),
        "type2_proximity_zero": type2_unique == 0,
        "type1_global_loop_present": type1_unique >= 1,
        "optimized_z_span_le_0_10m": (
            optimized_metrics["z_span_m"] is not None
            and optimized_metrics["z_span_m"] <= 0.10
        ),
        "optimized_gap_not_worse_than_same_raw_subset": (
            optimized_metrics["endpoint_xy_gap_m"] is not None
            and raw_subset_metrics["endpoint_xy_gap_m"] is not None
            and optimized_metrics["endpoint_xy_gap_m"]
            <= raw_subset_metrics["endpoint_xy_gap_m"]
        ),
        "odometry_lost_zero": runtime.get("warnings", {}).get("odometry_lost", 0) == 0,
        "rtabmap_p95_below_0_5s": (
            runtime.get("rtabmap_time_p95_s") is not None
            and runtime["rtabmap_time_p95_s"] < 0.5
        ),
    }
    return {
        "schema_version": "go2_rtabmap_readonly_run_report_v1",
        "source": {
            "run_dir": str(run_dir),
            "database": str(db_path),
            "database_size_bytes": db_path.stat().st_size,
            "database_sha256": db_hash_before,
            "database_hash_after_analysis": db_hash_after,
            "database_open_mode": "sqlite mode=ro, immutable=1",
            "runtime_log": str(run_dir / "runtime.log"),
        },
        "database": {
            "integrity": integrity,
            "rtabmap_version": admin[0] if admin else None,
            "node_count": len(node_rows),
            "node_weights": dict(sorted(weights.items())),
            "invalid_pose_count": invalid_pose_count,
            "duration_s": _round(max(stamps) - min(stamps)) if stamps else None,
            "links": link_counts,
            "type1_global_pairs": [[int(a), int(b)] for a, b in type1_pairs],
        },
        "trajectory": {
            "raw_all_nodes": raw_metrics,
            "raw_same_ids_as_optimized": raw_subset_metrics,
            "optimized_admin_cache": optimized_metrics,
            "raw_to_optimized_xy_correction_m": {
                "count": len(corrections),
                "p95": _round(_percentile(corrections, 0.95)),
                "max": _round(max(corrections)) if corrections else None,
            },
        },
        "runtime": runtime,
        "manifest": manifest,
        "automated_checks": automated_checks,
        "automated_overall": (
            "PASS_AUTOMATED_GATES_MANUAL_GEOMETRY_REQUIRED"
            if all(automated_checks.values())
            else "FAIL_OR_INCOMPLETE_AUTOMATED_GATES"
        ),
        "manual_checks_required": [
            "Verify both physical 90-degree corners and parallel walls.",
            "Inspect every Type-1 pair using RGB keyframes and L2 overlap.",
            "Check for self-intersection, duplicated walls, and 3D layer tilt.",
        ],
        "claim_scope": (
            "Read-only DB/log automation. It cannot prove physical geometry, absolute accuracy, "
            "camera/L2 calibration, or localization repeatability."
        ),
    }


def _markdown(report: dict[str, Any]) -> str:
    source = report["source"]
    database = report["database"]
    trajectory = report["trajectory"]
    runtime = report["runtime"]
    checks = report["automated_checks"]
    links = database["links"]
    lines = [
        "# RTAB-Map read-only run report",
        "",
        f"- Overall: **{report['automated_overall']}**",
        f"- Source DB: `{source['database']}`",
        f"- SHA-256: `{source['database_sha256']}`",
        f"- Integrity: `{database['integrity']}`",
        f"- Nodes: {database['node_count']}",
        "",
        "## Graph",
        "",
        "| Type | Name | Rows | Unique pairs |",
        "|---:|---|---:|---:|",
    ]
    for key in sorted(links, key=int):
        value = links[key]
        lines.append(
            f"| {key} | {value['name']} | {value['rows']} | {value['unique_pairs']} |"
        )
    lines.extend(
        [
            "",
            "## Trajectory",
            "",
            "| Source | Poses | XY path m | Endpoint gap m | Z span m |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for label, key in (
        ("raw all", "raw_all_nodes"),
        ("raw optimized IDs", "raw_same_ids_as_optimized"),
        ("optimized", "optimized_admin_cache"),
    ):
        value = trajectory[key]
        lines.append(
            f"| {label} | {value['pose_count']} | {value['path_xy_m']} | "
            f"{value['endpoint_xy_gap_m']} | {value['z_span_m']} |"
        )
    lines.extend(
        [
            "",
            "## Runtime",
            "",
            f"- RTAB-Map updates: {runtime.get('rtabmap_update_count')}",
            f"- Processing mean / p95 / max: {runtime.get('rtabmap_time_mean_s')} / "
            f"{runtime.get('rtabmap_time_p95_s')} / {runtime.get('rtabmap_time_max_s')} s",
            f"- Warnings: `{json.dumps(runtime.get('warnings', {}), sort_keys=True)}`",
            "",
            "## Automated checks",
            "",
        ]
    )
    lines.extend(f"- [{'x' if passed else ' '}] {name}" for name, passed in checks.items())
    lines.extend(
        [
            "",
            "## Manual checks still required",
            "",
            *[f"- {value}" for value in report["manual_checks_required"]],
            "",
            f"> {report['claim_scope']}",
            "",
        ]
    )
    return "\n".join(lines)


def write_report(report: dict[str, Any], output_root: Path) -> Path:
    source_name = Path(report["source"]["run_dir"]).name
    run_id = time.strftime(f"%Y%m%d_%H%M%S_{source_name}_readonly_analysis")
    output_dir = output_root.expanduser().resolve() / run_id
    output_dir.mkdir(parents=True, exist_ok=False)
    json_path = output_dir / "report.json"
    markdown_path = output_dir / "report.md"
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(_markdown(report), encoding="utf-8")
    manifest = output_dir / "SHA256SUMS"
    manifest.write_text(
        f"{sha256_file(json_path)}  report.json\n{sha256_file(markdown_path)}  report.md\n",
        encoding="utf-8",
    )
    return output_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only RTAB-Map DB/log analyzer; never invokes rtabmap-export"
    )
    parser.add_argument(
        "source",
        nargs="?",
        type=Path,
        default=DEFAULT_RUN,
        help="run directory or rtabmap.db (default: ~/.ros/rtabmap_runs/latest)",
    )
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--stdout", action="store_true", help="print JSON only; create no files")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report = analyze_run(args.source)
        if args.stdout:
            print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        else:
            output_dir = write_report(report, args.output_root)
            print(f"RTAB-Map read-only analysis: {report['automated_overall']}")
            print(f"Evidence: {output_dir}")
            print("Source database hash was unchanged; manual geometry inspection is still required.")
    except (FileNotFoundError, OSError, RuntimeError, sqlite3.DatabaseError, ValueError) as error:
        print(f"RTAB-Map read-only analysis BLOCKED: {error}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
