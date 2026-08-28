#!/bin/bash
# Planar 3DoF qualification mapping with a self-contained evidence directory.
#
# This wrapper keeps the existing 4DoF mapping entry points unchanged. It uses
# the same Unitree L2 3D cloud, LIO and RGB place recognition, but constrains
# the RTAB-Map graph to x/y/yaw for a flat single-floor environment.

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
EXEC_SCRIPT="$WORKSPACE_DIR/scratch/bringup_all_escape_nav.sh"
LAUNCH_SOURCE="$WORKSPACE_DIR/src/rtabmap_ros/rtabmap_launch/launch/go2_rtabmap.launch.py"
BRIDGE_SOURCE="$WORKSPACE_DIR/scratch/go2_livo_sensor_bridge.py"
LOGGER_SOURCE="$WORKSPACE_DIR/scratch/rtabmap_loop_logger.py"
RTABMAP_DB="/home/unitree/.ros/rtabmap.db"
RUN_ROOT="/home/unitree/.ros/rtabmap_runs"
RUN_ID="$(date +%Y%m%d_%H%M%S)_planar3dof_headless"
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
    echo "profile=planar3dof"
    echo "Reg/Force3DoF=true"
    echo "Icp/Force4DoF=false"
    echo "Optimizer/Slam2D=true"
    echo "RGBD/NeighborLinkRefining=false"
    echo "Rtabmap/DetectionRate=2.0"
    echo "recorder=false"
    echo "docker_vlm=false"
    echo "host_command_bridge=false"
    echo "command=$WORKSPACE_DIR/mapping_planar_headless.sh"
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
    echo " Planar mapping evidence saved"
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
echo " Starting PLANAR 3DoF headless RTAB-Map qualification"
echo " 3D cloud/ICP : enabled"
echo " Graph        : x/y/yaw"
echo " profile=planar3dof"
echo " Reg/Force3DoF=true"
echo " Icp/Force4DoF=false"
echo " Optimizer/Slam2D=true"
echo " Recorder=false"
echo " Docker/motor=false"
echo " Run ID       : $RUN_ID"
echo " Evidence dir : $RUN_DIR"
echo " Keep SSH/tmux alive and press Ctrl+C once after returning to the start."
echo "========================================================================"

set +e
bash "$EXEC_SCRIPT" --mapping --planar "$@" 2>&1 | tee -a "$RUNTIME_LOG"
bringup_status=${PIPESTATUS[0]}
set -e

exit "$bringup_status"
