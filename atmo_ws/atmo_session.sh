#!/usr/bin/env bash
# Build an ATMO hardware session in tmux, with a safe stop path.
#
# Replaces the bare `startup_robot.sh` tmux script. Three differences, each one
# a lesson from the m4-direct-rl bring-up:
#
#   1. Ordered startup with a real readiness check instead of `sleep 10`. A
#      fixed sleep is a guess that is either too short (races) or too long, and
#      it never tells you which.
#   2. The rosbag runs in its own window so it can be SIGINTed and WAITED FOR.
#      A bag killed with its session writes only the .db3 and no metadata.yaml,
#      which rosbag cannot reopen -- the session's data is gone.
#   3. Ctrl-C in the stack window IS the stop. Every node zeroes its actuators
#      in a `finally` before teardown, so SIGINT is the safe path -- there is
#      no separate teardown script to remember, and nothing to get wrong under
#      time pressure. The kill switch and dropping the RL gate stop it too.
#
# Usage:
#   ./atmo_session.sh shadow            # no command publishers at all
#   ./atmo_session.sh sensor            # connectivity only
#   ./atmo_session.sh policy --route takeoff
#   ./atmo_session.sh action --action-test tilt --action-sign positive
#
# Stop with:  Ctrl-C in the stack window (or the kill switch, or drop the RL gate)

set -euo pipefail
WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="${WORKSPACE_DIR}/scripts"

usage() {
    cat <<'EOF'
Usage: atmo_session.sh PROFILE [options]

PROFILE:
  shadow    policy runs, NO command publishers are created at all
  sensor    connectivity only, no command publishers
  action    single-axis action test (see src/atmo/HARDWARE_TESTS.md)
  ground    closed-loop policy on TILT AND WHEELS ONLY, rotors cut, no mocap
  policy    full closed-loop policy run

Options:
  --route takeoff|landing|full combined policy route (default: landing).
                               full = takeoff, hover ATMO_RL_HOVER_S (3 s),
                               land where it took off. No --ground-z needed:
                               the anchored start pose is the landed z.
  --mocap on|off               run the mocap bridge (default: on)
  --mocap-body NAME            Motive rigid body name (default: m4_base)
  --mocap-frame y_up|z_up      Motive streaming convention (default: y_up)
  --px4-relay on|off           feed the PX4 EKF (default: on for policy, else off)
  --policy PATH                policy .npz (default: $ATMO_RL_POLICY_PATH)
  --action-test NAME           lift|roll|pitch|yaw|tilt|drive|turn
  --action-sign positive|negative
  --action-magnitude VALUE
  --kill-test-passed           required before roll/pitch/yaw action tests
  --ground-z VALUE             measured landed z, required for landing
  --reference auto|mocap|virtual   pose source. auto probes the mocap topic
                               and falls back to virtual. ground default: auto
  --drive on|off               launch the wheel node (default: off -- the
                               drive RoboClaw died 2026-08-17, regen through
                               the 12V regulator; see docs/session_state.md.
                               Turn on only after the replacement board is in,
                               behind the battery bypass diode)
  --drive-only                 pin the route to DRIVE, never transition
                               (default ON for the ground profile)
  --drive-speed M_S            drive-only reference ground speed (default 0.0)
  --bag-dir DIR                default: <workspace>/bags
  --no-bag                     do not record
  --session NAME               tmux session name
  --detach                     create without attaching
EOF
}

[[ $# -gt 0 ]] || { usage >&2; exit 2; }
PROFILE="$1"; shift
case "${PROFILE}" in shadow|sensor|action|ground|policy) ;; *) usage >&2; exit 2 ;; esac

ROUTE="landing"
# Ground runs without mocap by design: the rotors are cut and the reference is
# synthetic, so there is nothing for a pose to feed. Starting the bridge anyway
# just fails noisily against a rig that is not there.
MOCAP="on"
# Motive's rigid-body name on this rig. m4-direct-rl streams it as "M4"
# (/vrpn_mocap/M4/pose) and it is the same Motive install. A wrong name
# gives no topic and no error.
MOCAP_BODY="${ATMO_MOCAP_BODY:-M4}"
# This rig is configured to stream Z-up, so the bridge passes axes through.
# m4-direct-rl runs mocap_bridge --frame z_up --rate-hz 120 against the same
# Motive. STILL AN ASSUMPTION until confirmed by motion -- move the vehicle
# by hand in each axis and check the signs before trusting a run.
MOCAP_FRAME="${ATMO_MOCAP_FRAME:-z_up}"
PX4_RELAY=""
POLICY="${ATMO_RL_POLICY_PATH:-}"
ACTION_TEST="lift"
ACTION_SIGN="positive"
ACTION_MAGNITUDE="0.1"
KILL_TEST_PASSED="false"
GROUND_Z="${ATMO_RL_GROUND_Z:-}"
REFERENCE=""
DRIVE_ONLY=""
DRIVE_SPEED="${ATMO_RL_DRIVE_ONLY_SPEED:-0.0}"
BAG_DIR="${ATMO_BAG_DIR:-${WORKSPACE_DIR}/bags}"
RECORD_BAG=1
SESSION="atmo-${PROFILE}"
DETACH=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --route) ROUTE="${2:?}"; shift 2 ;;
        --route=*) ROUTE="${1#*=}"; shift ;;
        --mocap) MOCAP="${2:?}"; MOCAP_EXPLICIT=1; shift 2 ;;
        --mocap=*) MOCAP="${1#*=}"; MOCAP_EXPLICIT=1; shift ;;
        --mocap-body) MOCAP_BODY="${2:?}"; shift 2 ;;
        --mocap-body=*) MOCAP_BODY="${1#*=}"; shift ;;
        --mocap-frame) MOCAP_FRAME="${2:?}"; shift 2 ;;
        --mocap-frame=*) MOCAP_FRAME="${1#*=}"; shift ;;
        --px4-relay) PX4_RELAY="${2:?}"; PX4_RELAY_EXPLICIT=1; shift 2 ;;
        --px4-relay=*) PX4_RELAY="${1#*=}"; PX4_RELAY_EXPLICIT=1; shift ;;
        --policy) POLICY="${2:?}"; shift 2 ;;
        --policy=*) POLICY="${1#*=}"; shift ;;
        --action-test) ACTION_TEST="${2:?}"; shift 2 ;;
        --action-test=*) ACTION_TEST="${1#*=}"; shift ;;
        --action-sign) ACTION_SIGN="${2:?}"; shift 2 ;;
        --action-sign=*) ACTION_SIGN="${1#*=}"; shift ;;
        --action-magnitude) ACTION_MAGNITUDE="${2:?}"; shift 2 ;;
        --action-magnitude=*) ACTION_MAGNITUDE="${1#*=}"; shift ;;
        --kill-test-passed) KILL_TEST_PASSED="true"; shift ;;
        --reference) REFERENCE="${2:?}"; shift 2 ;;
        --reference=*) REFERENCE="${1#*=}"; shift ;;
        --drive) DRIVE_HW="${2:?}"; shift 2 ;;
        --drive=*) DRIVE_HW="${1#*=}"; shift ;;
        --drive-only) DRIVE_ONLY=1; shift ;;
        --no-drive-only) DRIVE_ONLY=0; shift ;;
        --drive-speed) DRIVE_SPEED="${2:?}"; shift 2 ;;
        --drive-speed=*) DRIVE_SPEED="${1#*=}"; shift ;;
        --ground-z) GROUND_Z="${2:?}"; shift 2 ;;
        --ground-z=*) GROUND_Z="${1#*=}"; shift ;;
        --bag-dir) BAG_DIR="${2:?}"; shift 2 ;;
        --no-bag) RECORD_BAG=0; shift ;;
        --session) SESSION="${2:?}"; shift 2 ;;
        --detach) DETACH=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "${PROFILE}" == ground && -z "${MOCAP_EXPLICIT:-}" ]]; then
    MOCAP="off"
fi
# --- pose source ----------------------------------------------------------
# Mirrors m4-direct-rl: probe for live mocap and fall back, rather than
# assuming. `policy` REFUSES to run on a virtual pose -- flying on a
# fabricated position is not a degraded run.
REFERENCE="${REFERENCE:-$([[ "${PROFILE}" == policy ]] && echo mocap || echo auto)}"
case "${REFERENCE}" in auto|mocap|virtual) ;; *) echo "Bad --reference: ${REFERENCE}" >&2; exit 2 ;; esac
MOCAP_TOPIC="/vrpn_mocap/${MOCAP_BODY}/pose"
if [[ "${REFERENCE}" == auto ]]; then
    probe="${ATMO_MOCAP_DETECT_SECONDS:-5}"
    echo "Probing ${MOCAP_TOPIC} for ${probe}s..."
    if timeout "${probe}" ros2 topic echo --once "${MOCAP_TOPIC}" >/dev/null 2>&1; then
        REFERENCE=mocap
        echo "  live mocap detected -> using the measured pose"
    else
        REFERENCE=virtual
        echo "  no mocap -> using virtual dead-reckoned odometry"
    fi
fi
if [[ "${PROFILE}" == policy && "${REFERENCE}" != mocap ]]; then
    echo "policy requires live mocap; refusing reference '${REFERENCE}'." >&2
    echo "Flying on a fabricated position is not a degraded run." >&2
    exit 2
fi
if [[ "${REFERENCE}" == mocap ]]; then
    # The mocap pose goes STRAIGHT into the observation. PX4's EKF is not in
    # the loop: it is an estimator with its own convergence time, failure
    # modes and frame conventions sitting between the measurement and the
    # policy, and the mocap already IS the measurement.
    VIRTUAL_POSE=0
    POSE_SOURCE=mocap
    [[ -z "${MOCAP_EXPLICIT:-}" ]] && MOCAP="on"
    # The relay only feeds PX4 so its own logs and failsafes are sane. The
    # policy does not read anything that comes back out of it.
    [[ -z "${PX4_RELAY_EXPLICIT:-}" ]] && PX4_RELAY="on"
else
    VIRTUAL_POSE=1
    POSE_SOURCE=px4
fi

# Ground defaults to DRIVE-ONLY. On the full routes the policy spends most of
# its time in FLIGHT, where the wheel actions are unconstrained by training --
# applying those to real wheels is not a test of anything.
if [[ -z "${DRIVE_ONLY}" ]]; then
    DRIVE_ONLY=$([[ "${PROFILE}" == ground ]] && echo 1 || echo 0)
fi
case "${ROUTE}" in takeoff|landing|full) ;; *) echo "Bad route: ${ROUTE}" >&2; exit 2 ;; esac
case "${MOCAP}" in on|off) ;; *) echo "Bad --mocap: ${MOCAP}" >&2; exit 2 ;; esac
case "${MOCAP_FRAME}" in y_up|z_up) ;; *) echo "Bad --mocap-frame: ${MOCAP_FRAME}" >&2; exit 2 ;; esac

DRIVE_HW="${DRIVE_HW:-off}"
case "${DRIVE_HW}" in on|off) ;; *) echo "Bad --drive: ${DRIVE_HW}" >&2; exit 2 ;; esac
if [[ "${DRIVE_HW}" == off && "${PROFILE}" == action ]]; then
    case "${ACTION_TEST}" in
        drive|turn)
            echo "Action test '${ACTION_TEST}' needs the wheel node, and the drive" >&2
            echo "board is disabled (--drive off, the 2026-08-17 default: dead board)." >&2
            echo "Install the replacement behind the bypass diode, then pass --drive on." >&2
            exit 2 ;;
    esac
fi

if [[ -z "${PX4_RELAY}" ]]; then
    PX4_RELAY=$([[ "${PROFILE}" == policy ]] && echo on || echo off)
fi
case "${PX4_RELAY}" in on|off) ;; *) echo "Bad --px4-relay: ${PX4_RELAY}" >&2; exit 2 ;; esac

# A landing route without the measured ground z will descend to the wrong
# place. Refuse rather than default.
if [[ "${ROUTE}" == landing && "${PROFILE}" == policy && -z "${GROUND_Z}" ]]; then
    echo "Landing needs --ground-z (the measured landed vehicle z)." >&2
    exit 2
fi

command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 2; }
if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "tmux session '${SESSION}' already exists. Attach: tmux attach -t ${SESSION}" >&2
    echo "Or stop it: Ctrl-C in its stack window, then tmux kill-session -t ${SESSION}" >&2
    exit 2
fi

echo "Preflight..."
bash "${SCRIPT_DIR}/check_host.sh" || {
    echo
    echo "Preflight reported problems. Fix them, or re-run with them understood." >&2
    read -r -p "Continue anyway? [y/N] " reply
    [[ "${reply}" == y || "${reply}" == Y ]] || exit 2
}

mkdir -p "${BAG_DIR}"

shell_join() { printf '%q ' "$@"; }
pane_command() {
    printf 'cd %q && source %q && %s; code=$?; echo; echo "[window exited: $code]"; exec bash' \
        "${WORKSPACE_DIR}" "${SCRIPT_DIR}/atmo_env.sh" "$1"
}
add_window() {
    tmux new-window -d -t "${SESSION}" -n "$1" "$(pane_command "$2")"
}

# --- the stack -------------------------------------------------------------
case "${PROFILE}" in
    shadow) HARDWARE_MODE=shadow; COMMAND_HARDWARE=false ;;
    sensor) HARDWARE_MODE=sensor_test; COMMAND_HARDWARE=false ;;
    action) HARDWARE_MODE=action_test
            # Rotor-axis tests must not launch the physical tilt/drive nodes;
            # tilt/drive/turn tests need them.
            case "${ACTION_TEST}" in
                lift|roll|pitch|yaw) COMMAND_HARDWARE=false ;;
                *) COMMAND_HARDWARE=true ;;
            esac ;;
    # ground: the policy loop with the rotors cut. Commands tilt and drive,
    # never publishes actuator_motors, never arms, never goes offboard, and
    # needs no mocap. See rl_controller_hardware VALID_MODES.
    ground) HARDWARE_MODE=ground; COMMAND_HARDWARE=true ;;
    policy) HARDWARE_MODE=policy; COMMAND_HARDWARE=true ;;
esac

stack=(ros2 launch atmo rl_control.launch.py
    "route:=${ROUTE}"
    "hardware_mode:=${HARDWARE_MODE}"
    "command_hardware:=${COMMAND_HARDWARE}"
    "drive_hardware:=$([[ "${DRIVE_HW}" == on ]] && echo true || echo false)"
    "action_test:=${ACTION_TEST}"
    "action_sign:=${ACTION_SIGN}"
    "action_magnitude:=${ACTION_MAGNITUDE}"
    "kill_test_passed:=${KILL_TEST_PASSED}"
    # This session runs its own recorder in a window it can SIGINT and wait
    # for, so the launch must not also record.
    "record:=false")
STACK_COMMAND="$(shell_join "${stack[@]}")"

# Deterministic runs: never inherit randomization from an earlier shell.
PREFIX="ATMO_RL_RANDOMIZE_RESET=0 ATMO_RL_RANDOMIZE_MOTOR_DYNAMICS=0"
PREFIX="${PREFIX} ATMO_RL_OBSERVATION_NOISE=0"
PREFIX="${PREFIX} ATMO_RL_OBSERVATION_DELAY_MIN_STEPS=0"
PREFIX="${PREFIX} ATMO_RL_OBSERVATION_DELAY_MAX_STEPS=0"
# The gate channels must be forwarded EXPLICITLY. tmux hands a new session the
# environment its server started with, not the caller's, so an exported
# ATMO_RL_OFFBOARD_CHANNEL does not reliably reach the node when a tmux server
# is already running. Falling back to the code default is silent: the ratchet
# never leaves `waiting_prepare` and it reads as a hung script.
PREFIX="${PREFIX} ATMO_MOCAP_POSE_TOPIC=$(printf '%q' "${MOCAP_TOPIC}")"
PREFIX="${PREFIX} ATMO_MOCAP_BODY=$(printf '%q' "${MOCAP_BODY}")"
PREFIX="${PREFIX} ATMO_RL_POSE_SOURCE=$(printf '%q' "${POSE_SOURCE}")"
PREFIX="${PREFIX} ATMO_RL_VIRTUAL_POSE=$(printf '%q' "${VIRTUAL_POSE}")"
PREFIX="${PREFIX} ATMO_RL_DRIVE_ONLY=$(printf '%q' "${DRIVE_ONLY}")"
PREFIX="${PREFIX} ATMO_RL_DRIVE_ONLY_SPEED=$(printf '%q' "${DRIVE_SPEED}")"
PREFIX="${PREFIX} ATMO_RL_CHANNEL=$(printf '%q' "${ATMO_RL_CHANNEL:-7}")"
PREFIX="${PREFIX} ATMO_RL_OFFBOARD_CHANNEL=$(printf '%q' "${ATMO_RL_OFFBOARD_CHANNEL:--1}")"
# Same reasoning, higher stakes: the RoboClaw ports default to /dev/ttyACM0
# and ttyACM1, which are enumeration order rather than identity. If those
# defaults are silently used and enumeration differs from the last boot, tilt
# commands reach the drive board or drive commands reach the tilt motor.
for _rc_var in ATMO_TILT_ROBOCLAW ATMO_DRIVE_ROBOCLAW; do
    if [[ -n "${!_rc_var:-}" ]]; then
        PREFIX="${PREFIX} ${_rc_var}=$(printf '%q' "${!_rc_var}")"
    else
        echo "atmo_session.sh: ${_rc_var} is unset; the node will fall back to" >&2
        echo "  a /dev/ttyACM* default, which is enumeration order, not identity." >&2
        echo "  Source scripts/atmo_env.sh first." >&2
        exit 2
    fi
done
[[ -n "${POLICY}" ]] && PREFIX="${PREFIX} ATMO_RL_POLICY_PATH=$(printf '%q' "${POLICY}")"
[[ -n "${GROUND_Z}" ]] && PREFIX="${PREFIX} ATMO_RL_TARGET_Z=$(printf '%q' "${GROUND_Z}")"
if [[ "${PROFILE}" == shadow ]]; then
    mkdir -p "${WORKSPACE_DIR}/shadow_logs"
    log="${WORKSPACE_DIR}/shadow_logs/${ROUTE}_$(date +%Y%m%d_%H%M%S).jsonl"
    PREFIX="${PREFIX} ATMO_RL_SHADOW_LOG=$(printf '%q' "${log}")"
    echo "Shadow observation log: ${log}"
fi
STACK_COMMAND="${PREFIX} ${STACK_COMMAND}"

tmux new-session -d -s "${SESSION}" -n stack "$(pane_command "${STACK_COMMAND}")"

# --- mocap -----------------------------------------------------------------
if [[ "${MOCAP}" == on ]]; then
    mocap=(ros2 run atmo mocap_bridge --ros-args
        -p "body:=${MOCAP_BODY}"
        -p "source_frame:=${MOCAP_FRAME}"
        -p "px4_relay:=$([[ "${PX4_RELAY}" == on ]] && echo true || echo false)")
    add_window mocap "$(shell_join "${mocap[@]}")"
fi

# --- bag -------------------------------------------------------------------
if [[ "${RECORD_BAG}" -eq 1 ]]; then
    output="${BAG_DIR}/atmo_${PROFILE}_$(date +%Y%m%d_%H%M%S)"
    bag=(bash "${SCRIPT_DIR}/record_atmo_bag.sh" "${output}")
    add_window bag "$(shell_join "${bag[@]}")"
fi

# --- operator --------------------------------------------------------------
operator=(bash "${SCRIPT_DIR}/atmo_operator.sh" "${PROFILE}" "${SESSION}" "${ROUTE}")
add_window operator "$(shell_join "${operator[@]}")"

tmux select-window -t "${SESSION}:operator"
echo
echo "Created tmux session '${SESSION}' (profile=${PROFILE}, route=${ROUTE}, reference=${REFERENCE})."
echo "STOP: Ctrl-C in the stack window. The kill switch and dropping the RL"
echo "      gate also stop motion immediately."
[[ "${DETACH}" -eq 0 ]] && exec tmux attach -t "${SESSION}"
