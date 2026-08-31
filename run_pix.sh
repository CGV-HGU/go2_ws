#!/bin/bash
# ==============================================================================
# Canonical 1-Click Launcher for Baseline: Direct Goal / Pure PixNav
# ==============================================================================
# Usage:
#   ./run_pix.sh                        # Run Dead_end_room Direct_Goal Trial1
#   ./run_pix.sh Corridor_turn Trial2   # Custom Scenario and Trial
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
SCENARIO="${1:-Dead_end_room}"
TRIAL="${2:-Trial1}"

echo "========================================================================"
echo " 🤖 [BASELINE RUN] Direct Goal (Pure PixNav Policy)"
echo " Scenario : $SCENARIO"
echo " Model    : Direct_Goal"
echo " Trial    : $TRIAL"
echo "========================================================================"

exec bash "$WORKSPACE_DIR/scratch/bringup_all_escape_nav.sh" --record "$SCENARIO" Direct_Goal "$TRIAL"
