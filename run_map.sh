#!/bin/bash
# Canonical Jetson-desktop entry point for GUI mapping and saved-DB viewing.

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

usage() {
    cat <<'EOF'
Usage:
  ./run_map.sh                 Start planar 3DoF mapping with rtabmap_viz
  ./run_map.sh --view [DB]     View a safe temporary copy of a saved DB
  ./run_map.sh --print-config  Print the canonical mapping configuration
  ./run_map.sh --help          Show this help

For SSH/tmux without a display, use ./map_headless.sh.
EOF
}

find_display() {
    if [ -n "${DISPLAY:-}" ] && xdpyinfo >/dev/null 2>&1; then
        return 0
    fi
    for display_candidate in :1 :0 :1001; do
        if DISPLAY="$display_candidate" xdpyinfo >/dev/null 2>&1; then
            export DISPLAY="$display_candidate"
            return 0
        fi
    done
    echo "Error: no accessible X display. Use ./map_headless.sh over SSH/tmux." >&2
    return 1
}

case "${1:-}" in
    "")
        find_display
        exec bash "$WORKSPACE_DIR/map_headless.sh" --gui
        ;;
    --view)
        find_display
        db_path="${2:-/home/unitree/.ros/rtabmap.db}"
        if [ ! -f "$db_path" ]; then
            echo "Error: map database not found: $db_path" >&2
            exit 1
        fi
        viewer_dir="$(mktemp -d /tmp/rtabmap_view.XXXXXX)"
        viewer_db="$viewer_dir/$(basename "$db_path")"
        cp -aL "$db_path" "$viewer_db"
        cleanup_view_copy() {
            rm -rf -- "$viewer_dir"
        }
        trap cleanup_view_copy EXIT INT TERM
        echo "Opening temporary viewer copy: $viewer_db"
        echo "Original DB is protected; viewer changes will be discarded."
        set +e
        rtabmap-databaseViewer "$viewer_db"
        viewer_status=$?
        set -e
        exit "$viewer_status"
        ;;
    --print-config)
        exec bash "$WORKSPACE_DIR/map_headless.sh" --print-config
        ;;
    -h|--help)
        usage
        ;;
    *)
        echo "Error: unsupported option: $1" >&2
        usage >&2
        exit 2
        ;;
esac
