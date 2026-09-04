#!/bin/bash
# Canonical headless RTAB-Map mapping entry point for the Go2 flat-floor map.
# Keeps the Unitree L2 3D cloud/LIO/IMU and RGB place recognition while the
# RTAB-Map graph is constrained to x/y/yaw. Mapping never starts recording,
# Docker/VLM, a command bridge or motor output.

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." >/dev/null 2>&1 && pwd)"
EXEC_SCRIPT="$WORKSPACE_DIR/scratch/bringup_all_escape_nav.sh"
LAUNCH_SOURCE="$WORKSPACE_DIR/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py"
BRIDGE_SOURCE="$WORKSPACE_DIR/scratch/go2_livo_sensor_bridge.py"
LOGGER_SOURCE="$WORKSPACE_DIR/scratch/rtabmap_loop_logger.py"
RTABMAP_DB="/home/unitree/.ros/rtabmap.db"
RUN_ROOT="/home/unitree/.ros/rtabmap_runs"
DISPLAY_MODE="headless"
BRINGUP_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  ./map_headless.sh          Start planar 3DoF mapping over SSH/tmux
  ./map_headless.sh --help   Show this help

For a Jetson desktop GUI, use ./run_map.sh.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)
            DISPLAY_MODE="gui"
            BRINGUP_ARGS+=("--gui")
            shift
            ;;
        --print-config)
            bash "$EXEC_SCRIPT" --mapping --planar --print-config
            config_status=$?
            if [ "$config_status" -eq 0 ]; then
                echo "RGBD/NeighborLinkRefining=false"
                echo "RGBD/ProximityBySpace=false"
                echo "Rtabmap/DetectionRate=2.0"
                echo "Icp/VoxelSize=0.05"
                echo "Grid/3D=true"
            fi
            exit "$config_status"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: unsupported option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

RUN_ID="$(date +%Y%m%d_%H%M%S)_planar3dof_${DISPLAY_MODE}"
RUN_DIR="$RUN_ROOT/$RUN_ID"
RUNTIME_LOG="$RUN_DIR/runtime.log"
MANIFEST="$RUN_DIR/run_manifest.txt"

mkdir -p "$RUN_DIR/config" "$RUN_DIR/loop_logs"
export RTABMAP_RUN_DIR="$RUN_DIR"
ln -sfn "$RUN_ID" "$RUN_ROOT/latest"

cp -a "$LAUNCH_SOURCE" "$RUN_DIR/config/go2_rtabmap.launch.py"
cp -a "$BRIDGE_SOURCE" "$RUN_DIR/config/go2_livo_sensor_bridge.py"
cp -a "$LOGGER_SOURCE" "$RUN_DIR/config/rtabmap_loop_logger.py"

{
    echo "run_id=$RUN_ID"
    echo "started_at=$(date --iso-8601=seconds)"
    echo "workspace=$WORKSPACE_DIR"
    echo "git_head=$(git -C "$WORKSPACE_DIR" rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "entrypoint=$([ "$DISPLAY_MODE" = gui ] && echo run_map.sh || echo map_headless.sh)"
    echo "profile=planar3dof"
    echo "Reg/Force3DoF=true"
    echo "Icp/Force4DoF=false"
    echo "RGBD/LoopClosureIdentityGuess=true"
    echo "RGBD/NeighborLinkRefining=false"
    echo "RGBD/ProximityBySpace=false"
    echo "RGBD/ProximityAngle=45"
    echo "RGBD/ProximityMaxGraphDepth=50"
    echo "RGBD/ProximityPathMaxNeighbors=0"
    echo "Rtabmap/LoopThr=0.11"
    echo "RGBD/OptimizeMaxError=3.0"
    echo "Optimizer/Robust=false"
    echo "Rtabmap/DetectionRate=2.0"
    echo "Icp/VoxelSize=0.05"
    echo "Grid/3D=true"
    echo "recorder=false"
    echo "docker_vlm=false"
    echo "host_command_bridge=false"
} > "$MANIFEST"

git -C "$WORKSPACE_DIR" status --short > "$RUN_DIR/git_status.txt" 2>/dev/null || true

finalize_run() {
    local exit_status=$?
    local rtabmap_started=false
    local rtabmap_db_saved=false
    local db_report="not saved (RTAB-Map startup gate did not pass)"
    trap - EXIT
    set +e

    if [ -f "$RUN_DIR/RTABMAP_STARTED" ]; then
        rtabmap_started=true
        if [ -f "$RTABMAP_DB" ] && cp -a "$RTABMAP_DB" "$RUN_DIR/rtabmap.db"; then
            rtabmap_db_saved=true
            db_report="$RUN_DIR/rtabmap.db"
            if command -v sqlite3 >/dev/null 2>&1; then
                sqlite3 -readonly "$RUN_DIR/rtabmap.db" 'PRAGMA integrity_check;' \
                    > "$RUN_DIR/database_integrity.txt" 2>&1
            else
                # Jetson images do not always include the sqlite3 CLI. Python's
                # standard sqlite3 module performs the same read-only check.
                python3 - "$RUN_DIR/rtabmap.db" \
                    > "$RUN_DIR/database_integrity.txt" 2>&1 <<'PY'
import sqlite3
import sys

connection = sqlite3.connect(f"file:{sys.argv[1]}?mode=ro", uri=True)
try:
    print(connection.execute("PRAGMA integrity_check;").fetchone()[0])
finally:
    connection.close()
PY
            fi
        else
            db_report="not saved (startup passed, but no readable RTAB-Map DB was found)"
        fi
    fi

    {
        echo "finished_at=$(date --iso-8601=seconds)"
        echo "wrapper_exit_status=$exit_status"
        echo "rtabmap_started=$rtabmap_started"
        echo "rtabmap_db_saved=$rtabmap_db_saved"
    } >> "$MANIFEST"

    (
        cd "$RUN_DIR" || exit 0
        : > SHA256SUMS
        for artifact in \
            run_manifest.txt \
            git_status.txt \
            runtime.log \
            RTABMAP_STARTED \
            rtabmap.db \
            database_integrity.txt \
            config/go2_rtabmap.launch.py \
            config/go2_livo_sensor_bridge.py \
            config/rtabmap_loop_logger.py; do
            if [ -f "$artifact" ]; then
                sha256sum "$artifact" >> SHA256SUMS
            fi
        done
        find loop_logs -maxdepth 1 -type f -print0 2>/dev/null \
            | sort -z \
            | xargs -0 -r sha256sum >> SHA256SUMS
    )

    ln -sfn "$RUN_ID" "$RUN_ROOT/latest"

    echo ""
    echo "========================================================================"
    echo " Mapping evidence saved"
    echo " Run ID : $RUN_ID"
    echo " Run dir: $RUN_DIR"
    echo " Started: $rtabmap_started"
    echo " DB     : $db_report"
    echo " Logs   : $RUN_DIR/runtime.log and $RUN_DIR/loop_logs/"
    echo " Hashes : $RUN_DIR/SHA256SUMS"
    echo "========================================================================"

    exit "$exit_status"
}

trap finalize_run EXIT

echo "========================================================================"
echo " Starting Go2 planar 3DoF RTAB-Map (${DISPLAY_MODE})"
echo " 3D cloud/ICP : enabled"
echo " Graph        : x/y/yaw"
echo " Global loop  : RGB retrieval + identity guess + 3D LiDAR ICP check"
echo " Proximity    : disabled (Type-2 distortion ablation)"
echo " Recorder     : false"
echo " Docker/motor : false"
echo " Run ID       : $RUN_ID"
echo " Evidence dir : $RUN_DIR"
echo " Return to the start with the same camera heading, wait 3-5 s,"
echo " then press Ctrl+C once."
echo "========================================================================"

operator_interrupt=false
trap 'operator_interrupt=true' INT
set +e
bash "$EXEC_SCRIPT" --mapping --planar "${BRINGUP_ARGS[@]}" 2>&1 \
    | tee --ignore-interrupts -a "$RUNTIME_LOG"
bringup_status=${PIPESTATUS[0]}
set -e
trap - INT

# Ctrl+C is the documented, successful end of an established mapping run.
# tee ignores the terminal interrupt so it can record the inner cleanup and
# RTAB-Map shutdown instead of closing the pipe early and causing SIGPIPE 141.
operator_stop=false
wrapper_status=$bringup_status
if [ "$operator_interrupt" = true ] && [ "$bringup_status" -eq 130 ] && \
   [ -f "$RUN_DIR/RTABMAP_STARTED" ]; then
    operator_stop=true
    wrapper_status=0
fi
{
    echo "bringup_exit_status=$bringup_status"
    echo "operator_interrupt=$operator_interrupt"
    echo "operator_stop=$operator_stop"
} >> "$MANIFEST"

exit "$wrapper_status"
