#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

task="${1:?Usage: $0 landing|takeoff 'lift,roll,pitch,yaw,tilt,drive,turn'}"
action="${2:?Usage: $0 landing|takeoff 'lift,roll,pitch,yaw,tilt,drive,turn'}"
if [[ "$task" != "landing" && "$task" != "takeoff" ]]; then
    echo "Task must be landing or takeoff" >&2
    exit 2
fi

# Distro-agnostic: the robot runs Humble, not the Foxy this was written
# against. scripts/atmo_env.sh detects it and sources the workspace.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/atmo_env.sh"

export ATMO_RL_TASK="$task"
export ATMO_RL_HARDWARE_MODE=fixed_action
export ATMO_RL_FIXED_ACTION="$action"
export ATMO_RL_RANDOMIZE_RESET=0
export ATMO_RL_RANDOMIZE_MOTOR_DYNAMICS=0
export ATMO_RL_OBSERVATION_NOISE=0
export ATMO_RL_OBSERVATION_DELAY_MIN_STEPS=0
export ATMO_RL_OBSERVATION_DELAY_MAX_STEPS=0

echo "FIXED-ACTION HARDWARE MODE: REMOVE ALL PROPELLERS"
echo "Task: $ATMO_RL_TASK"
echo "Raw action [lift,roll,pitch,yaw,tilt,drive,turn]: $ATMO_RL_FIXED_ACTION"
echo "Both offboard and RL switches are required before any command is sent"

exec ros2 launch atmo rl_control.launch.py command_hardware:=true
