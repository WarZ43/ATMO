#!/usr/bin/env bash
# The pure-unit suite: no ROS, no torch, no vehicle.
#
# PYTHONPATH IS DELIBERATELY STRIPPED. Measured on the Jetson 2026-08-17:
# with the colcon build/install dirs on PYTHONPATH the same `atmo` package is
# visible three times (src via cwd, build/atmo, install/atmo), and pytest's
# launch_testing collection hook binds `atmo` to a duplicate in a way that
# loses the newest module -- collection died on `atmo.mocap_frames` even
# though every copy on disk contained the file, and a direct
# `python3 -c "import atmo.mocap_frames"` from the same shell succeeded.
# With PYTHONPATH empty the suite passes (99 passed, 1 skipped). Do not
# "fix" this by sourcing the workspace first; that is what breaks it.
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/../src/atmo"
PYTHONPATH= exec python3 -m pytest test/ -q \
    --ignore=test/test_copyright.py \
    --ignore=test/test_flake8.py \
    --ignore=test/test_pep257.py \
    "$@"
