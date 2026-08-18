#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

task="${1:?Usage: $0 landing|takeoff}"
if [[ "$task" != "landing" && "$task" != "takeoff" ]]; then
    echo "Task must be landing or takeoff" >&2
    exit 2
fi

# Distro-agnostic: the robot runs Humble, not the Foxy this was written
# against. scripts/atmo_env.sh detects it and sources the workspace.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/atmo_env.sh"

if [[ "$task" == "landing" ]]; then
    : "${ATMO_RL_GROUND_Z:?Set ATMO_RL_GROUND_Z to the measured landed vehicle z coordinate}"
    export ATMO_RL_TARGET_Z="$ATMO_RL_GROUND_Z"
fi

mkdir -p "$workspace_dir/shadow_logs"
export ATMO_RL_TASK="$task"
export ATMO_RL_HARDWARE_MODE=shadow
export ATMO_RL_SHADOW_LOG="${ATMO_RL_SHADOW_LOG:-$workspace_dir/shadow_logs/${task}_$(date +%Y%m%d_%H%M%S).jsonl}"
export ATMO_RL_RANDOMIZE_RESET=0
export ATMO_RL_RANDOMIZE_MOTOR_DYNAMICS=0
export ATMO_RL_OBSERVATION_NOISE=0
export ATMO_RL_OBSERVATION_DELAY_MIN_STEPS=0
export ATMO_RL_OBSERVATION_DELAY_MAX_STEPS=0

echo "SHADOW MODE: this process creates no actuator, tilt, drive, offboard, or arm publishers"
echo "Task: $ATMO_RL_TASK"
echo "Raise only the RL switch to reset the reference and start logging"
echo "Observation log: $ATMO_RL_SHADOW_LOG"

exec ros2 launch atmo rl_control.launch.py command_hardware:=false
