#!/usr/bin/env bash
# Record an ATMO session, but only once the custom message types resolve.
#
# rosbag needs the type to be discoverable when it subscribes. Starting the
# recorder before custom_msgs is on the graph gets you a bag that is missing
# exactly the topics you started it for -- and you find out after the session,
# not during it. Waiting is cheap; re-flying is not.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 OUTPUT_DIR [TOPIC ...]" >&2
    exit 2
fi
OUTPUT_DIR="$1"
shift

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/atmo_env.sh"

# The default topic set mirrors what rl_control.launch.py recorded, plus the
# mocap bridge's outputs, which the launch never had.
DEFAULT_TOPICS=(
    /fmu/in/actuator_motors
    /fmu/in/vehicle_visual_odometry
    /fmu/out/battery_status_v1
    /fmu/out/estimator_status_flags
    /fmu/out/failsafe_flags
    /fmu/out/input_rc
    /fmu/out/vehicle_command_ack
    /fmu/out/vehicle_control_mode
    /fmu/out/vehicle_odometry
    /fmu/out/vehicle_status_v1
    /tilt_vel
    # Measured tilt angle. Its absence cost the 2026-08-18 post-mortem its
    # tilt ground truth: phi had to be reconstructed by integrating commanded
    # /tilt_vel, leaving the S(phi) roll residual unresolvable.
    /fmu/in/tilt_angle
    /atmo/rl/policy_action
    /atmo/rl/observation_state
    # Post-mixer per-rotor commands. /fmu/in/actuator_motors carries the same
    # values but ONLY when a command publisher exists, so it is empty in the
    # shadow profile -- this one is published straight off the policy tick.
    /atmo/rl/actuator_commands
    # RAW Motive pose, before mocap_bridge applies mount_yaw. Recorded next to
    # /atmo/groundtruth_odom so the 180 deg mount correction can be checked
    # after the fact instead of taken on faith.
    /vrpn_mocap/${ATMO_MOCAP_BODY:-M4}/pose
    /drive_vel
    /atmo/rl/manual_override
    /atmo/groundtruth_odom
)
TOPICS=("$@")
[[ ${#TOPICS[@]} -eq 0 ]] && TOPICS=("${DEFAULT_TOPICS[@]}")

WAIT_SECONDS="${ATMO_BAG_WAIT_SECONDS:-30}"
if [[ ! "${WAIT_SECONDS}" =~ ^[0-9]+$ ]] || (( WAIT_SECONDS < 1 )); then
    echo "ATMO_BAG_WAIT_SECONDS must be a positive integer" >&2
    exit 2
fi

wait_for_type() {
    local topic="$1" expected="$2" deadline=$((SECONDS + WAIT_SECONDS)) types
    echo "Waiting for ${topic} (${expected})..."
    while true; do
        types="$(ros2 topic type "${topic}" 2>/dev/null || true)"
        if grep -Fxq "${expected}" <<<"${types}"; then
            return 0
        fi
        if (( SECONDS >= deadline )); then
            echo "Timed out waiting for ${topic}." >&2
            echo "The stack may have failed to start, or command_hardware is false" >&2
            echo "for this profile (shadow and sensor create no command publishers)." >&2
            return 1
        fi
        sleep 1
    done
}

# Shadow and sensor profiles deliberately create no command publishers, so a
# missing /tilt_vel there is correct rather than a fault. Warn and record what
# does exist instead of refusing.
if [[ "${ATMO_RL_HARDWARE_MODE:-}" == "shadow" ]]; then
      wait_for_type /atmo/rl/policy_action std_msgs/msg/Float32MultiArray
  else
      wait_for_type /tilt_vel custom_msgs/msg/TiltVel
  fi


echo "Recording bag: ${OUTPUT_DIR}"
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Stop this with Ctrl-C so metadata.yaml is written."
exec ros2 bag record -o "${OUTPUT_DIR}" "${TOPICS[@]}"
