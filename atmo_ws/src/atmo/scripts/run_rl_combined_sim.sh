#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || ( "$1" != "takeoff" && "$1" != "landing" ) ]]; then
    echo "Usage: $0 takeoff|landing [runner arguments...]" >&2
    exit 1
fi

ROUTE="$1"
shift
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

export ATMO_RL_ROUTE="${ROUTE}"
export ATMO_RL_POLICY_PATH="${ATMO_RL_POLICY_PATH:-$HOME/policies/atmo_combined_stage1_policy.npz}"
export ATMO_RL_RANDOMIZE_RESET=0
export ATMO_RL_SIM_ROTOR_THRUST_SCALE="${ATMO_RL_SIM_ROTOR_THRUST_SCALE:-1.0}"
if [[ "${ROUTE}" == "takeoff" ]]; then
    export ATMO_RL_INITIAL_TILT_ANGLE="${ATMO_RL_INITIAL_TILT_ANGLE:-1.5707963267948966}"
else
    export ATMO_RL_INITIAL_TILT_ANGLE="${ATMO_RL_INITIAL_TILT_ANGLE:-0}"
fi

eval "$(PYTHONPATH="${PACKAGE_ROOT}:${PYTHONPATH:-}" python3 -m atmo.rl_combined_runtime --shell-profile "${ROUTE}")"

echo "ATMO combined route: ${ROUTE}, deterministic=${ATMO_RL_DETERMINISTIC_TRAJECTORY:-0}"
exec bash "${SCRIPT_DIR}/run_rl_landing_drop_sim.sh" "$@"
