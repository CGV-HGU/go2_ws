#!/usr/bin/env bash
# Direct PixelNav uses the shared ESCAPE campaign runner and recording pipeline.
set -euo pipefail
exec bash /home/unitree/s2e-vlm-async-framework-minimal/scripts/run_pixnav.sh "$@"
