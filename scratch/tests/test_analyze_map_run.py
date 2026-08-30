import hashlib
from pathlib import Path
import sqlite3
import struct
import zlib

from analyze_map_run import analyze_run, trajectory_metrics


def pose(x, y, z):
    return struct.pack(
        "<12f",
        1.0, 0.0, 0.0, x,
        0.0, 1.0, 0.0, y,
        0.0, 0.0, 1.0, z,
    )


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fixture_run(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    db = run_dir / "rtabmap.db"
    connection = sqlite3.connect(str(db))
    connection.executescript(
        """
        CREATE TABLE Node(id INTEGER PRIMARY KEY, weight INTEGER, stamp FLOAT, pose BLOB);
        CREATE TABLE Link(from_id INTEGER, to_id INTEGER, type INTEGER);
        CREATE TABLE Admin(version TEXT, opt_ids BLOB, opt_poses BLOB);
        """
    )
    raw = [(1, 0.0, 0.0, 0.0), (2, 1.0, 0.0, 0.01), (3, 0.1, 0.0, 0.02)]
    for node_id, x, y, z in raw:
        connection.execute(
            "INSERT INTO Node VALUES(?, 0, ?, ?)",
            (node_id, float(node_id), pose(x, y, z)),
        )
    connection.executemany(
        "INSERT INTO Link VALUES(?, ?, ?)",
        [(1, 2, 0), (2, 1, 0), (3, 1, 1), (1, 3, 1)],
    )
    ids = zlib.compress(struct.pack("<3i", 1, 2, 3))
    optimized = zlib.compress(pose(0.0, 0.0, 0.0) + pose(1.0, 0.0, 0.01) + pose(0.05, 0.0, 0.02))
    connection.execute("INSERT INTO Admin VALUES('0.21.1', ?, ?)", (ids, optimized))
    connection.commit()
    connection.close()
    (run_dir / "runtime.log").write_text(
        "rtabmap (1): Rate=0.50s, RTAB-Map=0.1000s\n"
        "rtabmap (2): Rate=0.50s, RTAB-Map=0.2000s\n",
        encoding="utf-8",
    )
    (run_dir / "run_manifest.txt").write_text(
        "profile=planar3dof\n"
        "Reg/Force3DoF=true\n"
        "Icp/Force4DoF=false\n"
        "RGBD/ProximityBySpace=false\n"
        "Rtabmap/DetectionRate=2.0\n"
        "Icp/VoxelSize=0.05\n",
        encoding="utf-8",
    )
    return run_dir, db


def test_readonly_report_decodes_graph_and_preserves_database(tmp_path):
    run_dir, db = fixture_run(tmp_path)
    before = sha256(db)

    report = analyze_run(run_dir)

    assert sha256(db) == before
    assert report["database"]["integrity"] == "ok"
    assert report["database"]["node_count"] == 3
    assert report["database"]["links"]["0"]["rows"] == 2
    assert report["database"]["links"]["0"]["unique_pairs"] == 1
    assert report["database"]["links"]["1"]["unique_pairs"] == 1
    assert report["trajectory"]["optimized_admin_cache"]["pose_count"] == 3
    assert report["runtime"]["rtabmap_time_p95_s"] == 0.195


def test_trajectory_metrics_are_empty_safe_and_compute_gap():
    assert trajectory_metrics([])["pose_count"] == 0
    value = trajectory_metrics([(1, 0.0, 0.0, 0.0), (2, 3.0, 4.0, 0.5)])
    assert value["path_xy_m"] == 5.0
    assert value["endpoint_xy_gap_m"] == 5.0
    assert value["z_span_m"] == 0.5
