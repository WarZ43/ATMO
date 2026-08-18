#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

# Distro-agnostic: the robot runs Humble, not the Foxy this was written
# against. scripts/atmo_env.sh detects it and sources the workspace.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/atmo_env.sh"

: "${ATMO_RL_GROUND_Z:?Set ATMO_RL_GROUND_Z to the measured landed vehicle z coordinate}"

export ATMO_RL_TASK=landing
export ATMO_RL_HARDWARE_MODE=policy
export ATMO_RL_TARGET_Z="$ATMO_RL_GROUND_Z"
export ATMO_RL_RANDOMIZE_RESET=0
export ATMO_RL_RANDOMIZE_MOTOR_DYNAMICS=0
export ATMO_RL_OBSERVATION_NOISE=0
export ATMO_RL_OBSERVATION_DELAY_MIN_STEPS=0
export ATMO_RL_OBSERVATION_DELAY_MAX_STEPS=0

echo "Starting fixed vertical RL landing"
echo "Ground target z: ${ATMO_RL_TARGET_Z} m"
echo "RC channel indexes: offboard=${ATMO_RL_OFFBOARD_CHANNEL:-8}, RL=${ATMO_RL_CHANNEL:-7}"
echo "A PX4 motor-kill switch must be configured separately"

exec ros2 launch atmo rl_control.launch.py
