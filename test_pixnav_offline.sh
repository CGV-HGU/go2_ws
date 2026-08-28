#!/usr/bin/env bash
# Robot-free PixNav qualification for the Jetson host.
# This script never starts ROS nodes, mapping, networking, a command bridge,
# /cmd_vel, a Unitree SDK client, or an actuator process.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-quick}"
PACKAGE_DIR="$ROOT_DIR/src/escape_nav_pixnav"
VLM_RUN="/home/unitree/.ros/pixnav_s2e_runs/20260828_135901_pixnav_s2e_no_actuation"
PIXNAV_REPORT="/home/unitree/.ros/pixnav_runs/20260828_162002_pixnav_file_only/report.json"
MACRO_RUN="/home/unitree/.ros/pixnav_macro_runs/20260828_162023_pixnav_macro_file_only"
SAVED_CHAIN_RUN="/home/unitree/.ros/pixnav_chain_runs/20260828_162122_pixnav_offline_chain"
SAVED_FAULT_RUN="/home/unitree/.ros/pixnav_fault_runs/20260828_163454_pixnav_fault_injection"
SAVED_QUALIFICATION_RUN="/home/unitree/.ros/pixnav_qualification_runs/20260828_163514_pixnav_qualification"
CHECKPOINT="$ROOT_DIR/.local-data/vlm-s2e/checkpoints/pixelnav_A.ckpt"
REFERENCE_DIR="$ROOT_DIR/.local-data/vlm-s2e/runtime/vlm-s2e-integration-paper-pin"
RUNTIME_SITE="$ROOT_DIR/.local-data/pixnav_runtime/site-packages"
FRAMES_DIR="$VLM_RUN/frames"

usage() {
    echo "Usage: ./test_pixnav_offline.sh [quick|evidence|cuda]"
    echo "  quick    : build, 56 unit tests, syntax and saved-evidence hash checks"
    echo "  evidence : quick + regenerate causal/fault/qualification evidence"
    echo "  cuda     : quick + rerun frozen PixNav on saved RGB + downstream evidence"
}

evidence_dir_from_output() {
    awk '/^Evidence: / {print $2}' | tail -n 1
}

require_file() {
    if [[ ! -f "$1" ]]; then
        echo "BLOCKED: required file is missing: $1" >&2
        exit 2
    fi
}

require_dir() {
    if [[ ! -d "$1" ]]; then
        echo "BLOCKED: required directory is missing: $1" >&2
        exit 2
    fi
}

if [[ "$MODE" != "quick" && "$MODE" != "evidence" && "$MODE" != "cuda" ]]; then
    usage
    exit 2
fi

cd "$ROOT_DIR"
export PYTHONPATH="$PACKAGE_DIR${PYTHONPATH:+:$PYTHONPATH}"

echo "========================================================================"
echo " PixNav robot-free qualification: $MODE"
echo " ROS nodes=false  Network=false  SDK=false  Actuation=false"
echo " Repository: $ROOT_DIR"
echo "========================================================================"

echo "[1/4] Syntax, package build and isolated tests"
python3 -m py_compile pixnav_check.py "$PACKAGE_DIR"/escape_nav_pixnav/*.py
colcon build --packages-select escape_nav_pixnav --symlink-install
colcon test --packages-select escape_nav_pixnav --event-handlers console_direct+
colcon test-result --verbose

echo "[2/4] Saved immutable evidence hashes"
require_file "$VLM_RUN/SHA256SUMS"
require_file "$SAVED_CHAIN_RUN/SHA256SUMS"
require_file "$SAVED_FAULT_RUN/SHA256SUMS"
require_file "$SAVED_QUALIFICATION_RUN/SHA256SUMS"
(cd "$VLM_RUN" && sha256sum -c SHA256SUMS)
(cd "$SAVED_CHAIN_RUN" && sha256sum -c SHA256SUMS)
(cd "$SAVED_FAULT_RUN" && sha256sum -c SHA256SUMS)
(cd "$SAVED_QUALIFICATION_RUN" && sha256sum -c SHA256SUMS)

if [[ "$MODE" == "quick" ]]; then
    echo "[3/4] Skipped current-source evidence regeneration (select evidence or cuda)"
    echo "[4/4] PASS_ROBOT_FREE_QUICK"
    exit 0
fi

require_dir "$VLM_RUN"
require_file "$PIXNAV_REPORT"
require_dir "$MACRO_RUN"
require_file "$CHECKPOINT"
require_dir "$REFERENCE_DIR"

if [[ "$MODE" == "cuda" ]]; then
    echo "[3/4] Frozen PixNav CUDA replay on saved Go2 RGB"
    require_dir "$RUNTIME_SITE"
    require_dir "$FRAMES_DIR"
    set +e
    pixnav_output="$(
        ./pixnav_check.py \
            --device cuda \
            --runtime-site "$RUNTIME_SITE" \
            --frames-dir "$FRAMES_DIR" \
            --goal-frame-index 10 \
            --history-start-index 10
    )"
    pixnav_status=$?
    set -e
    echo "$pixnav_output"
    pixnav_dir="$(printf '%s\n' "$pixnav_output" | evidence_dir_from_output)"
    require_file "$pixnav_dir/report.json"
    if [[ $pixnav_status -ne 0 ]]; then
        echo "BLOCKED_GPU_ACCESS_OR_PREREQUISITE" >&2
        echo "Inspect: $pixnav_dir/report.json" >&2
        echo "No CPU fallback was used because this mode explicitly qualifies CUDA." >&2
        exit "$pixnav_status"
    fi
    PIXNAV_REPORT="$pixnav_dir/report.json"

    macro_output="$(
        python3 -m escape_nav_pixnav.replay "$PIXNAV_REPORT"
    )"
    echo "$macro_output"
    MACRO_RUN="$(printf '%s\n' "$macro_output" | evidence_dir_from_output)"
    require_file "$MACRO_RUN/summary.json"
else
    echo "[3/4] Reusing the accepted v2 CUDA report; no model/GPU execution"
fi

echo "[4/4] Current-source causal, fault and qualification evidence"
chain_output="$(
    python3 -m escape_nav_pixnav.causal_chain \
        --vlm-run-dir "$VLM_RUN" \
        --pixnav-report "$PIXNAV_REPORT" \
        --macro-run-dir "$MACRO_RUN"
)"
echo "$chain_output"
chain_run="$(printf '%s\n' "$chain_output" | evidence_dir_from_output)"
require_file "$chain_run/causal_manifest.json"

fault_output="$(
    python3 -m escape_nav_pixnav.fault_injection \
        --vlm-run-dir "$VLM_RUN" \
        --pixnav-report "$PIXNAV_REPORT" \
        --macro-run-dir "$MACRO_RUN"
)"
echo "$fault_output"
fault_run="$(printf '%s\n' "$fault_output" | evidence_dir_from_output)"
require_file "$fault_run/fault_report.json"

qualification_output="$(
    python3 -m escape_nav_pixnav.qualification \
        --repo-root "$ROOT_DIR" \
        --checkpoint "$CHECKPOINT" \
        --reference-dir "$REFERENCE_DIR" \
        --vlm-run-dir "$VLM_RUN" \
        --pixnav-report "$PIXNAV_REPORT" \
        --macro-run-dir "$MACRO_RUN" \
        --causal-run-dir "$chain_run" \
        --fault-run-dir "$fault_run"
)"
echo "$qualification_output"
qualification_run="$(printf '%s\n' "$qualification_output" | evidence_dir_from_output)"
require_file "$qualification_run/qualification_manifest.json"

echo "------------------------------------------------------------------------"
echo "PASS_ROBOT_FREE_${MODE^^}"
echo "PixNav report : $PIXNAV_REPORT"
echo "Macro run     : $MACRO_RUN"
echo "Causal run    : $chain_run"
echo "Fault run     : $fault_run"
echo "Qualification: $qualification_run"
echo "No physical motion or live service was used."
