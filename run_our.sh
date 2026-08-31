#!/bin/bash
# ==============================================================================
# Canonical 1-Click Launcher for Proposed: Full ESCAPE-Nav (Qwen VLM + PixNav)
# ==============================================================================
# Usage:
#   ./run_our.sh                        # Run Dead_end_room Full_ESCAPE_Nav Trial1
#   ./run_our.sh Corridor_turn Trial2   # Custom Scenario and Trial
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
SCENARIO="${1:-Dead_end_room}"
TRIAL="${2:-Trial1}"

echo "========================================================================"
echo " 🌟 [PROPOSED RUN] Full ESCAPE-Nav (Qwen VLM Frontier + PixNav Policy)"
echo " Scenario : $SCENARIO"
echo " Model    : Full_ESCAPE_Nav"
echo " Trial    : $TRIAL"
echo "========================================================================"

exec bash "$WORKSPACE_DIR/scratch/bringup_all_escape_nav.sh" --record "$SCENARIO" Full_ESCAPE_Nav "$TRIAL"
