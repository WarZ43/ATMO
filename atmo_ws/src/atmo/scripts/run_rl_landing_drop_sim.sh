#!/usr/bin/env bash
set -euo pipefail

ATMO_WS="${ATMO_WS:-$HOME/m4_ws/ATMO/atmo_ws}"
PX4_DIR="${PX4_DIR:-$HOME/m4_ws/X1-PX4}"
PX4_MAVLINK_SHELL_ENDPOINT="${PX4_MAVLINK_SHELL_ENDPOINT:-udp:127.0.0.1:14550}"
MODEL_NAME="${ATMO_RL_GAZEBO_MODEL:-m4}"
DROP_X_WAS_SET="${ATMO_RL_DROP_X+x}"
DROP_Y_WAS_SET="${ATMO_RL_DROP_Y+x}"
DROP_Z_WAS_SET="${ATMO_RL_DROP_Z+x}"
DROP_ROLL_WAS_SET="${ATMO_RL_DROP_ROLL+x}"
DROP_PITCH_WAS_SET="${ATMO_RL_DROP_PITCH+x}"
DROP_YAW_WAS_SET="${ATMO_RL_DROP_YAW+x}"
DROP_VX_WAS_SET="${ATMO_RL_DROP_VX+x}"
DROP_VY_WAS_SET="${ATMO_RL_DROP_VY+x}"
DROP_VZ_WAS_SET="${ATMO_RL_DROP_VZ+x}"
DROP_ROLL_RATE_WAS_SET="${ATMO_RL_DROP_ROLL_RATE+x}"
DROP_PITCH_RATE_WAS_SET="${ATMO_RL_DROP_PITCH_RATE+x}"
DROP_YAW_RATE_WAS_SET="${ATMO_RL_DROP_YAW_RATE+x}"
REFERENCE_START_VX_WAS_SET="${ATMO_RL_REFERENCE_START_VX+x}"
REFERENCE_START_VY_WAS_SET="${ATMO_RL_REFERENCE_START_VY+x}"
REFERENCE_END_VX_WAS_SET="${ATMO_RL_REFERENCE_END_VX+x}"
REFERENCE_END_VY_WAS_SET="${ATMO_RL_REFERENCE_END_VY+x}"
REFERENCE_VERTICAL_SPEED_WAS_SET="${ATMO_RL_REFERENCE_VERTICAL_SPEED+x}"
INITIAL_TILT_ANGLE_WAS_SET="${ATMO_RL_INITIAL_TILT_ANGLE+x}"
DROP_X="${ATMO_RL_DROP_X:-0}"
DROP_Y="${ATMO_RL_DROP_Y:-0}"
DROP_Z="${ATMO_RL_DROP_Z:-2}"
DROP_ROLL="${ATMO_RL_DROP_ROLL:-0}"
DROP_PITCH="${ATMO_RL_DROP_PITCH:-0}"
DROP_YAW="${ATMO_RL_DROP_YAW:-1.7}"
DROP_VX="${ATMO_RL_DROP_VX:-0}"
DROP_VY="${ATMO_RL_DROP_VY:-0}"
DROP_VZ="${ATMO_RL_DROP_VZ:-0}"
DROP_ROLL_RATE="${ATMO_RL_DROP_ROLL_RATE:-0}"
DROP_PITCH_RATE="${ATMO_RL_DROP_PITCH_RATE:-0}"
DROP_YAW_RATE="${ATMO_RL_DROP_YAW_RATE:-0}"
START_X="${ATMO_RL_START_X:-${ATMO_RL_TARGET_X:-0.0}}"
START_Y="${ATMO_RL_START_Y:-${ATMO_RL_TARGET_Y:-0.0}}"
START_Z="${ATMO_RL_START_Z:-${ATMO_RL_TARGET_Z:-0.24}}"
HOLD_GRAVITY="0,0,0"
RESTORE_GRAVITY="0,0,-9.8"
CLEANUP_GRAVITY="${RESTORE_GRAVITY}"
READY_TIMEOUT="${ATMO_RL_READY_TIMEOUT:-30}"
LOG_FILE="${ATMO_RL_LOG_FILE:-/tmp/atmo_rl_landing_drop_sim.log}"
TILT_LOG_FILE="${ATMO_RL_TILT_LOG_FILE:-/tmp/atmo_rl_tilt_controller_sim.log}"
POLICY_GATE_FILE="${ATMO_RL_START_GATE_FILE:-/tmp/atmo_rl_policy_start_gate}"
RUN_MODE="${ATMO_RL_MODE:-}"
TILT_CONTROL_MODE="${ATMO_RL_SIM_TILT_CONTROL_MODE:-direct}"
LEGACY_TEST_MODE="${ATMO_RL_TEST_MODE:-0}"
RESTORE_GRAVITY_AFTER_READY=1
DRIVE_TEST_TILT_SETTLE_S="${ATMO_RL_DRIVE_TEST_TILT_SETTLE_S:-4.5}"

is_truthy() {
    case "${1:-}" in
        1|true|TRUE|yes|YES|on|ON)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

if [[ -z "${RUN_MODE}" ]]; then
    case "${LEGACY_TEST_MODE}" in
        1|true|TRUE|yes|YES|on|ON)
            RUN_MODE="zero_g_test"
            ;;
        *)
            RUN_MODE="rl_test"
            ;;
    esac
fi

case "${RUN_MODE}" in
    zero_g_test|zero-gravity-test|zero_g|zero-gravity|action_test|actuator_test|test)
        RUN_MODE="zero_g_test"
        RESTORE_GRAVITY_AFTER_READY=0
        export ATMO_RL_MODE="${RUN_MODE}"
        export ATMO_RL_TEST_MODE=1
        if [[ -z "${ATMO_RL_TEST_LIFT:-}" && -z "${ATMO_RL_TEST_THRUST:-}" ]]; then
            export ATMO_RL_TEST_LIFT="0"
        fi
        ;;
    rl_test|rl|policy|policy_test)
        RUN_MODE="rl_test"
        RESTORE_GRAVITY_AFTER_READY=1
        export ATMO_RL_MODE="${RUN_MODE}"
        export ATMO_RL_TEST_MODE=0
        unset ATMO_RL_TEST_LIFT ATMO_RL_TEST_THRUST ATMO_RL_TEST_ROLL ATMO_RL_TEST_PITCH
        unset ATMO_RL_TEST_YAW ATMO_RL_TEST_TILT ATMO_RL_TEST_DRIVE ATMO_RL_TEST_TURN
        ;;
    drive_test|drive|wheel_test|wheels|ground_drive_test)
        RUN_MODE="drive_test"
        RESTORE_GRAVITY_AFTER_READY=1
        export ATMO_RL_MODE="${RUN_MODE}"
        export ATMO_RL_TEST_MODE=0
        DROP_Z="${ATMO_RL_DRIVE_TEST_DROP_Z:-0.3}"
        export ATMO_RL_TEST_TILT_ANGLE="${ATMO_RL_TEST_TILT_ANGLE:-1.5708}"
        export ATMO_RL_INITIAL_TILT_ANGLE="${ATMO_RL_INITIAL_TILT_ANGLE:-${ATMO_RL_TEST_TILT_ANGLE}}"
        unset ATMO_RL_TEST_LIFT ATMO_RL_TEST_THRUST ATMO_RL_TEST_ROLL ATMO_RL_TEST_PITCH
        unset ATMO_RL_TEST_YAW ATMO_RL_TEST_TILT
        ;;
    *)
        echo "Unknown ATMO_RL_MODE='${RUN_MODE}'. Use rl_test, zero_g_test, or drive_test." >&2
        exit 1
        ;;
esac

case "${TILT_CONTROL_MODE}" in
    direct|actuator|actuator_position|position)
        TILT_CONTROL_MODE="direct"
        ;;
    topic_velocity|tilt_topic|tilt_vel_topic|tilt_angle_topic|velocity|tilt_vel|tilt_velocity|velocity_controller)
        TILT_CONTROL_MODE="topic_velocity"
        ;;
    actuator_velocity|actuator_vel|direct_velocity)
        TILT_CONTROL_MODE="actuator_velocity"
        ;;
    *)
        TILT_CONTROL_MODE="direct"
        ;;
esac
export ATMO_RL_SIM_TILT_CONTROL_MODE="${TILT_CONTROL_MODE}"

RANDOMIZE_RESET="${ATMO_RL_RANDOMIZE_RESET:-}"
if [[ -z "${RANDOMIZE_RESET}" ]]; then
    RANDOMIZE_RESET=0
fi
export ATMO_RL_RANDOMIZE_RESET="${RANDOMIZE_RESET}"

if is_truthy "${RANDOMIZE_RESET}"; then
    eval "$(
        ATMO_RL_TARGET_X="${ATMO_RL_TARGET_X:-0.0}" \
        ATMO_RL_TARGET_Y="${ATMO_RL_TARGET_Y:-0.0}" \
        ATMO_RL_RANDOM_FORWARD_YAW_OFFSET="${ATMO_RL_RANDOM_FORWARD_YAW_OFFSET:-3.141592653589793}" \
        python3 - <<'PY'
import math
import os
import random

seed = os.environ.get("ATMO_RL_RANDOM_SEED")
if seed not in (None, ""):
    random.seed(int(seed))

def env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)

target_x = env_float("ATMO_RL_TARGET_X", 0.0)
target_y = env_float("ATMO_RL_TARGET_Y", 0.0)
xy_range = max(env_float("ATMO_RL_RANDOM_INITIAL_XY_RANGE", 2.0), 0.0)
z_min = env_float("ATMO_RL_RANDOM_INITIAL_Z_MIN", 0.5)
z_max = env_float("ATMO_RL_RANDOM_INITIAL_Z_MAX", 1.75)
if z_max < z_min:
    z_min, z_max = z_max, z_min
roll_pitch_range = max(env_float("ATMO_RL_RANDOM_ROLL_PITCH_RANGE", 5.0 * math.pi / 180.0), 0.0)
yaw_range = max(env_float("ATMO_RL_RANDOM_YAW_RANGE", 30.0 * math.pi / 180.0), 0.0)
ang_vel_min = env_float("ATMO_RL_RANDOM_INITIAL_ANG_VEL_MIN", -0.1)
ang_vel_max = env_float("ATMO_RL_RANDOM_INITIAL_ANG_VEL_MAX", 0.1)
if ang_vel_max < ang_vel_min:
    ang_vel_min, ang_vel_max = ang_vel_max, ang_vel_min
start_speed_min = env_float("ATMO_RL_REFERENCE_START_SPEED_MIN", 0.0)
start_speed_max = env_float("ATMO_RL_REFERENCE_START_SPEED_MAX", 1.0)
velocity_delta_min = env_float("ATMO_RL_REFERENCE_END_SPEED_DELTA_MIN", -0.5)
velocity_delta_max = env_float("ATMO_RL_REFERENCE_END_SPEED_DELTA_MAX", 0.5)
if start_speed_max < start_speed_min:
    start_speed_min, start_speed_max = start_speed_max, start_speed_min
if velocity_delta_max < velocity_delta_min:
    velocity_delta_min, velocity_delta_max = velocity_delta_max, velocity_delta_min
cone_range = max(env_float("ATMO_RL_REFERENCE_BOUNDARY_CONE_RANGE", 30.0 * math.pi / 180.0), 0.0)
forward_yaw_offset = env_float("ATMO_RL_RANDOM_FORWARD_YAW_OFFSET", math.pi)

drop_x = env_float("ATMO_RL_DROP_X", target_x + random.uniform(-xy_range, xy_range))
drop_y = env_float("ATMO_RL_DROP_Y", target_y + random.uniform(-xy_range, xy_range))
drop_z = env_float("ATMO_RL_DROP_Z", random.uniform(z_min, z_max))
dx = target_x - drop_x
dy = target_y - drop_y
bearing = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0
start_heading = bearing + random.uniform(-cone_range, cone_range)
start_speed = random.uniform(start_speed_min, start_speed_max) if start_speed_max > 0.0 else 0.0
start_vx = math.cos(start_heading) * start_speed
start_vy = math.sin(start_heading) * start_speed
end_speed = min(max(start_speed + random.uniform(velocity_delta_min, velocity_delta_max), 0.0), 1.0)
end_vx = math.cos(start_heading) * end_speed
end_vy = math.sin(start_heading) * end_speed
vertical_speed = random.uniform(0.4, 0.7)
reference_duration = max(abs(0.24 - drop_z) / max(vertical_speed, 1e-3), math.hypot(dx, dy), 0.75) * 1.875
accel_duration = 0.2 * reference_duration + 0.2
decel_duration = 0.2 * reference_duration + 0.5
xy_distance = math.hypot(dx, dy)
xy_direction = (dx / max(xy_distance, 1e-6), dy / max(xy_distance, 1e-6))
boundary_progress = 0.5 * accel_duration * (start_vx * xy_direction[0] + start_vy * xy_direction[1])
boundary_progress += 0.5 * decel_duration * (end_vx * xy_direction[0] + end_vy * xy_direction[1])
boundary_scale = min(0.95 * xy_distance / max(boundary_progress, 1e-6), 1.0) if xy_distance > 1e-6 else 0.0
start_vx *= boundary_scale
start_vy *= boundary_scale
end_vx *= boundary_scale
end_vy *= boundary_scale
drop_roll = env_float("ATMO_RL_DROP_ROLL", random.uniform(-roll_pitch_range, roll_pitch_range))
drop_pitch = env_float("ATMO_RL_DROP_PITCH", random.uniform(-roll_pitch_range, roll_pitch_range))
drop_yaw = env_float("ATMO_RL_DROP_YAW", start_heading - forward_yaw_offset + random.uniform(-yaw_range, yaw_range))
drop_vx = env_float("ATMO_RL_DROP_VX", start_vx)
drop_vy = env_float("ATMO_RL_DROP_VY", start_vy)
drop_vz = env_float("ATMO_RL_DROP_VZ", 0.0)
drop_roll_rate = env_float("ATMO_RL_DROP_ROLL_RATE", random.uniform(ang_vel_min, ang_vel_max))
drop_pitch_rate = env_float("ATMO_RL_DROP_PITCH_RATE", random.uniform(ang_vel_min, ang_vel_max))
drop_yaw_rate = env_float("ATMO_RL_DROP_YAW_RATE", random.uniform(ang_vel_min, ang_vel_max))
reference_start_vx = env_float("ATMO_RL_REFERENCE_START_VX", drop_vx)
reference_start_vy = env_float("ATMO_RL_REFERENCE_START_VY", drop_vy)
reference_end_vx = env_float("ATMO_RL_REFERENCE_END_VX", end_vx)
reference_end_vy = env_float("ATMO_RL_REFERENCE_END_VY", end_vy)
reference_vertical_speed = env_float("ATMO_RL_REFERENCE_VERTICAL_SPEED", vertical_speed)
initial_tilt_angle = env_float("ATMO_RL_INITIAL_TILT_ANGLE", random.uniform(0.0, math.pi / 12.0))

values = {
    "SAMPLED_DROP_X": drop_x,
    "SAMPLED_DROP_Y": drop_y,
    "SAMPLED_DROP_Z": drop_z,
    "SAMPLED_DROP_ROLL": drop_roll,
    "SAMPLED_DROP_PITCH": drop_pitch,
    "SAMPLED_DROP_YAW": drop_yaw,
    "SAMPLED_DROP_VX": drop_vx,
    "SAMPLED_DROP_VY": drop_vy,
    "SAMPLED_DROP_VZ": drop_vz,
    "SAMPLED_DROP_ROLL_RATE": drop_roll_rate,
    "SAMPLED_DROP_PITCH_RATE": drop_pitch_rate,
    "SAMPLED_DROP_YAW_RATE": drop_yaw_rate,
    "SAMPLED_REFERENCE_START_VX": reference_start_vx,
    "SAMPLED_REFERENCE_START_VY": reference_start_vy,
    "SAMPLED_REFERENCE_END_VX": reference_end_vx,
    "SAMPLED_REFERENCE_END_VY": reference_end_vy,
    "SAMPLED_REFERENCE_VERTICAL_SPEED": reference_vertical_speed,
    "SAMPLED_INITIAL_TILT_ANGLE": initial_tilt_angle,
}
for key, value in values.items():
    print(f"{key}={value:.9g}")
PY
    )"

    [[ -z "${DROP_X_WAS_SET}" ]] && DROP_X="${SAMPLED_DROP_X}"
    [[ -z "${DROP_Y_WAS_SET}" ]] && DROP_Y="${SAMPLED_DROP_Y}"
    [[ -z "${DROP_Z_WAS_SET}" ]] && DROP_Z="${SAMPLED_DROP_Z}"
    [[ -z "${DROP_ROLL_WAS_SET}" ]] && DROP_ROLL="${SAMPLED_DROP_ROLL}"
    [[ -z "${DROP_PITCH_WAS_SET}" ]] && DROP_PITCH="${SAMPLED_DROP_PITCH}"
    [[ -z "${DROP_YAW_WAS_SET}" ]] && DROP_YAW="${SAMPLED_DROP_YAW}"
    [[ -z "${DROP_VX_WAS_SET}" ]] && DROP_VX="${SAMPLED_DROP_VX}"
    [[ -z "${DROP_VY_WAS_SET}" ]] && DROP_VY="${SAMPLED_DROP_VY}"
    [[ -z "${DROP_VZ_WAS_SET}" ]] && DROP_VZ="${SAMPLED_DROP_VZ}"
    [[ -z "${DROP_ROLL_RATE_WAS_SET}" ]] && DROP_ROLL_RATE="${SAMPLED_DROP_ROLL_RATE}"
    [[ -z "${DROP_PITCH_RATE_WAS_SET}" ]] && DROP_PITCH_RATE="${SAMPLED_DROP_PITCH_RATE}"
    [[ -z "${DROP_YAW_RATE_WAS_SET}" ]] && DROP_YAW_RATE="${SAMPLED_DROP_YAW_RATE}"
    [[ -z "${REFERENCE_START_VX_WAS_SET}" ]] && export ATMO_RL_REFERENCE_START_VX="${SAMPLED_REFERENCE_START_VX}"
    [[ -z "${REFERENCE_START_VY_WAS_SET}" ]] && export ATMO_RL_REFERENCE_START_VY="${SAMPLED_REFERENCE_START_VY}"
    [[ -z "${REFERENCE_END_VX_WAS_SET}" ]] && export ATMO_RL_REFERENCE_END_VX="${SAMPLED_REFERENCE_END_VX}"
    [[ -z "${REFERENCE_END_VY_WAS_SET}" ]] && export ATMO_RL_REFERENCE_END_VY="${SAMPLED_REFERENCE_END_VY}"
    [[ -z "${REFERENCE_VERTICAL_SPEED_WAS_SET}" ]] && export ATMO_RL_REFERENCE_VERTICAL_SPEED="${SAMPLED_REFERENCE_VERTICAL_SPEED}"
    [[ -z "${INITIAL_TILT_ANGLE_WAS_SET}" ]] && export ATMO_RL_INITIAL_TILT_ANGLE="${SAMPLED_INITIAL_TILT_ANGLE}"
fi

if ! is_truthy "${RANDOMIZE_RESET}" && [[ -z "${DROP_YAW_WAS_SET}" ]]; then
    DROP_YAW="$(
        ATMO_RL_TARGET_X="${ATMO_RL_TARGET_X:-0.0}" \
        ATMO_RL_TARGET_Y="${ATMO_RL_TARGET_Y:-0.0}" \
        ATMO_RL_DROP_X="${DROP_X}" \
        ATMO_RL_DROP_Y="${DROP_Y}" \
        python3 - <<'PY'
import math
import os

target_x = float(os.environ["ATMO_RL_TARGET_X"])
target_y = float(os.environ["ATMO_RL_TARGET_Y"])
drop_x = float(os.environ["ATMO_RL_DROP_X"])
drop_y = float(os.environ["ATMO_RL_DROP_Y"])
heading = math.atan2(target_y - drop_y, target_x - drop_x)
print(f"{heading - math.pi:.17g}")
PY
    )"
fi

RL_PID=""
TILT_PID=""
TAIL_PID=""
GRAVITY_HELD=0

warn_thrust_setpoint_publishers() {
    local info
    local count
    info="$(ros2 topic info /fmu/in/vehicle_thrust_setpoint 2>/dev/null || true)"
    count="$(printf "%s\n" "${info}" | awk -F': ' '/Publisher count:/ {print $2; exit}')"
    if [[ -n "${count}" && "${count}" != "0" ]]; then
        echo "WARNING: ${count} publisher(s) are active on /fmu/in/vehicle_thrust_setpoint."
        echo "The RL node does not publish this topic; stop any MPC/control node that appears below:"
        printf "%s\n" "${info}"
    fi
}

warn_preexisting_publishers() {
    local topic="$1"
    local label="$2"
    local info
    local count
    info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
    count="$(printf "%s\n" "${info}" | awk -F': ' '/Publisher count:/ {print $2; exit}')"
    if [[ -n "${count}" && "${count}" != "0" ]]; then
        echo "WARNING: ${count} pre-existing publisher(s) are active on ${topic}."
        echo "Stop ${label} before running RL if tilt or actuator commands look discontinuous:"
        printf "%s\n" "${info}"
    fi
}

configure_px4_for_rl_sim() {
    local shell_script="${PX4_DIR}/Tools/mavlink_shell.py"

    if [[ ! -f "${shell_script}" ]]; then
        echo "PX4 MAVLink shell not found: ${shell_script}" >&2
        echo "Set PX4_DIR to the X1-PX4 checkout used by this simulation." >&2
        exit 1
    fi

    echo "Configuring PX4 safety and battery parameters for Gazebo RL testing"
    if ! printf '%s\n' \
        'attitude_estimator_q start' \
        'param set FD_FAIL_R 0' \
        'param set FD_FAIL_P 0' \
        'param set SIM_BAT_MIN_PCT 100' \
        'param set SIM_BAT_DRAIN 86400' \
        'param set COM_LOW_BAT_ACT 0' \
        'param set FD_ACT_EN 0' | \
        timeout 8s python3 "${shell_script}" "${PX4_MAVLINK_SHELL_ENDPOINT}" >/tmp/atmo_rl_px4_setup.log 2>&1; then
        echo "Could not configure PX4 through ${PX4_MAVLINK_SHELL_ENDPOINT}." >&2
        echo "Stop QGroundControl if it owns UDP port 14550, or set PX4_MAVLINK_SHELL_ENDPOINT." >&2
        cat /tmp/atmo_rl_px4_setup.log >&2
        exit 1
    fi
}

try_set_model_state() {
    local x="$1"
    local y="$2"
    local z="$3"
    local roll="$4"
    local pitch="$5"
    local yaw="$6"
    local vx="$7"
    local vy="$8"
    local vz="$9"
    local roll_rate="${10}"
    local pitch_rate="${11}"
    local yaw_rate="${12}"

    ATMO_RL_SET_MODEL_NAME="${MODEL_NAME}" \
    ATMO_RL_SET_X="${x}" \
    ATMO_RL_SET_Y="${y}" \
    ATMO_RL_SET_Z="${z}" \
    ATMO_RL_SET_ROLL="${roll}" \
    ATMO_RL_SET_PITCH="${pitch}" \
    ATMO_RL_SET_YAW="${yaw}" \
    ATMO_RL_SET_VX="${vx}" \
    ATMO_RL_SET_VY="${vy}" \
    ATMO_RL_SET_VZ="${vz}" \
    ATMO_RL_SET_ROLL_RATE="${roll_rate}" \
    ATMO_RL_SET_PITCH_RATE="${pitch_rate}" \
    ATMO_RL_SET_YAW_RATE="${yaw_rate}" \
    python3 - <<'PY' >/dev/null 2>&1
import math
import os
import sys
import time

try:
    import rclpy
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
    from geometry_msgs.msg import Quaternion
    from rclpy.node import Node
except Exception:
    sys.exit(1)

def env_float(name, default=0.0):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)

def quat_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )

rclpy.init()
node = Node("atmo_rl_set_model_state_once")
client = node.create_client(SetEntityState, "/gazebo/set_entity_state")
deadline = time.monotonic() + 1.0
try:
    while rclpy.ok() and not client.wait_for_service(timeout_sec=0.1):
        if time.monotonic() >= deadline:
            sys.exit(1)
    state = EntityState()
    state.name = os.environ["ATMO_RL_SET_MODEL_NAME"]
    state.reference_frame = "world"
    state.pose.position.x = env_float("ATMO_RL_SET_X")
    state.pose.position.y = env_float("ATMO_RL_SET_Y")
    state.pose.position.z = env_float("ATMO_RL_SET_Z")
    state.pose.orientation = quat_from_euler(
        env_float("ATMO_RL_SET_ROLL"),
        env_float("ATMO_RL_SET_PITCH"),
        env_float("ATMO_RL_SET_YAW"),
    )
    state.twist.linear.x = env_float("ATMO_RL_SET_VX")
    state.twist.linear.y = env_float("ATMO_RL_SET_VY")
    state.twist.linear.z = env_float("ATMO_RL_SET_VZ")
    state.twist.angular.x = env_float("ATMO_RL_SET_ROLL_RATE")
    state.twist.angular.y = env_float("ATMO_RL_SET_PITCH_RATE")
    state.twist.angular.z = env_float("ATMO_RL_SET_YAW_RATE")
    request = SetEntityState.Request()
    request.state = state
    future = client.call_async(request)
    end = time.monotonic() + 2.0
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        sys.exit(1)
    response = future.result()
    sys.exit(0 if getattr(response, "success", False) else 1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

place_model() {
    local x="$1"
    local y="$2"
    local z="$3"
    local roll="$4"
    local pitch="$5"
    local yaw="$6"
    local vx="$7"
    local vy="$8"
    local vz="$9"
    local roll_rate="${10}"
    local pitch_rate="${11}"
    local yaw_rate="${12}"

    if try_set_model_state \
        "${x}" "${y}" "${z}" "${roll}" "${pitch}" "${yaw}" \
        "${vx}" "${vy}" "${vz}" "${roll_rate}" "${pitch_rate}" "${yaw_rate}"; then
        echo "Set ${MODEL_NAME} pose/twist through /gazebo/set_entity_state"
    else
        gz model -m "${MODEL_NAME}" \
            -x "${x}" -y "${y}" -z "${z}" \
            -R "${roll}" -P "${pitch}" -Y "${yaw}"
        if [[ "${vx}" != "0" || "${vy}" != "0" || "${vz}" != "0" || "${roll_rate}" != "0" || "${pitch_rate}" != "0" || "${yaw_rate}" != "0" ]]; then
            echo "WARNING: /gazebo/set_entity_state unavailable; pose was set but sampled reset twist was not applied."
        fi
    fi
}

cleanup() {
    if [[ -n "${TAIL_PID}" ]] && kill -0 "${TAIL_PID}" >/dev/null 2>&1; then
        kill "${TAIL_PID}" >/dev/null 2>&1 || true
    fi

    if [[ -n "${RL_PID}" ]] && kill -0 "${RL_PID}" >/dev/null 2>&1; then
        kill -- "-${RL_PID}" >/dev/null 2>&1 || kill "${RL_PID}" >/dev/null 2>&1 || true
    fi

    if [[ -n "${TILT_PID}" ]] && kill -0 "${TILT_PID}" >/dev/null 2>&1; then
        kill -- "-${TILT_PID}" >/dev/null 2>&1 || kill "${TILT_PID}" >/dev/null 2>&1 || true
    fi

    if [[ "${GRAVITY_HELD}" == "1" ]]; then
        gz physics -g "${CLEANUP_GRAVITY}" >/dev/null 2>&1 || true
    fi

    rm -f "${POLICY_GATE_FILE}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

if ! command -v gz >/dev/null 2>&1; then
    echo "gz command not found. Start this from the m4server Gazebo/PX4 environment." >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

set +u
source "${ATMO_WS}/install/setup.bash"
set -u
export ATMO_RL_POLICY_PATH="${ATMO_RL_POLICY_PATH:-$HOME/policies/atmo_combined_stage1.pth}"
export ATMO_RL_LOG_ACTIONS=1
export ATMO_RL_LOG_OBS=1
export ATMO_RL_LOG_INTERVAL_S="${ATMO_RL_LOG_INTERVAL_S:-2.0}"
export ATMO_RL_START_GATE_FILE="${POLICY_GATE_FILE}"
unset PYTHONNOUSERSITE
rm -f "${POLICY_GATE_FILE}"

echo "ATMO RL drop-sim mode: ${RUN_MODE}"
echo "ATMO RL tilt control mode: ${TILT_CONTROL_MODE}"
echo "ATMO RL reset: randomize=${RANDOMIZE_RESET}, pos=(${DROP_X}, ${DROP_Y}, ${DROP_Z}), rpy=(${DROP_ROLL}, ${DROP_PITCH}, ${DROP_YAW})"
echo "ATMO RL reset twist: linear=(${DROP_VX}, ${DROP_VY}, ${DROP_VZ}), angular=(${DROP_ROLL_RATE}, ${DROP_PITCH_RATE}, ${DROP_YAW_RATE})"
echo "ATMO RL logging: observations/actions/rotor commands every ${ATMO_RL_LOG_INTERVAL_S}s"
echo "Waiting for Gazebo model '${MODEL_NAME}'..."
while ! gz model -m "${MODEL_NAME}" -p >/dev/null 2>&1; do
    sleep 0.1
done

if [[ "${RESTORE_GRAVITY_AFTER_READY}" == "0" ]]; then
    echo "Holding gravity at ${HOLD_GRAVITY}"
    gz physics -g "${HOLD_GRAVITY}"
    GRAVITY_HELD=1
    echo "Moving ${MODEL_NAME} to zero-gravity test pose"
    place_model \
        "${DROP_X}" "${DROP_Y}" "${DROP_Z}" \
        "${DROP_ROLL}" "${DROP_PITCH}" "${DROP_YAW}" \
        "${DROP_VX}" "${DROP_VY}" "${DROP_VZ}" \
        "${DROP_ROLL_RATE}" "${DROP_PITCH_RATE}" "${DROP_YAW_RATE}"
else
    echo "Keeping normal gravity at ${RESTORE_GRAVITY} for PX4 estimator initialization"
    gz physics -g "${RESTORE_GRAVITY}"
    echo "Staging ${MODEL_NAME} on the ground before arming"
    place_model \
        "${START_X}" "${START_Y}" "${START_Z}" \
        0 0 "${DROP_YAW}" \
        0 0 0 0 0 0
fi

configure_px4_for_rl_sim
echo "Waiting 2s for the PX4 attitude estimator to initialize"
sleep 2

warn_preexisting_publishers "/fmu/in/actuator_motors" "other direct actuator controllers"
if [[ "${TILT_CONTROL_MODE}" == "topic_velocity" ]]; then
    warn_preexisting_publishers "/fmu/in/tilt_angle" "stale tilt_controller_sim or MPC tilt publishers"
    echo "Starting tilt controller sim node. Log: ${TILT_LOG_FILE}"
    : > "${TILT_LOG_FILE}"
    setsid ros2 run atmo tilt_controller_sim > "${TILT_LOG_FILE}" 2>&1 &
    TILT_PID=$!
else
    warn_preexisting_publishers "/fmu/in/tilt_angle" "tilt_controller_sim or MPC tilt publishers"
fi

echo "Starting RL sim node. Log: ${LOG_FILE}"
: > "${LOG_FILE}"
setsid ros2 run atmo rl_controller_sim > "${LOG_FILE}" 2>&1 &
RL_PID=$!
tail -n +1 -f "${LOG_FILE}" &
TAIL_PID=$!
warn_thrust_setpoint_publishers

echo "Waiting for PX4 armed + offboard confirmation..."
ATMO_RL_READY_TIMEOUT="${READY_TIMEOUT}" python3 - <<'PY'
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import VehicleControlMode, VehicleStatus


class ReadyWatcher(Node):
    def __init__(self):
        super().__init__("atmo_rl_drop_ready_watcher")
        self.armed = False
        self.offboard = False
        self.nav_state = None
        self.create_subscription(
            VehicleControlMode,
            "/fmu/out/vehicle_control_mode",
            self._control_mode,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._status,
            qos_profile_sensor_data,
        )

    def _control_mode(self, msg):
        self.offboard = bool(msg.flag_control_offboard_enabled)

    def _status(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = int(msg.nav_state)


timeout = float(os.environ.get("ATMO_RL_READY_TIMEOUT", "30"))
rclpy.init()
node = ReadyWatcher()
deadline = time.monotonic() + timeout
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.armed and node.offboard:
            sys.exit(0)
    print(
        f"Timed out waiting for armed/offboard: armed={node.armed}, "
        f"offboard={node.offboard}, nav_state={node.nav_state}",
        file=sys.stderr,
    )
    sys.exit(1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

case "${RESTORE_GRAVITY_AFTER_READY}" in
    1|true|TRUE|yes|YES|on|ON)
        if [[ "${RUN_MODE}" == "drive_test" ]]; then
            echo "Drive test: waiting ${DRIVE_TEST_TILT_SETTLE_S}s for tilt to reach the ground-driving pose"
            sleep "${DRIVE_TEST_TILT_SETTLE_S}"
        fi
        echo "PX4 is armed + offboard. Moving ${MODEL_NAME} to the configured RL start state"
        place_model \
            "${DROP_X}" "${DROP_Y}" "${DROP_Z}" \
            "${DROP_ROLL}" "${DROP_PITCH}" "${DROP_YAW}" \
            "${DROP_VX}" "${DROP_VY}" "${DROP_VZ}" \
            "${DROP_ROLL_RATE}" "${DROP_PITCH_RATE}" "${DROP_YAW_RATE}"
        echo "Releasing RL policy gate: ${POLICY_GATE_FILE}"
        : > "${POLICY_GATE_FILE}"
        ;;
    *)
        echo "PX4 is armed + offboard. Keeping gravity at ${HOLD_GRAVITY}"
        echo "Releasing RL policy gate: ${POLICY_GATE_FILE}"
        : > "${POLICY_GATE_FILE}"
        ;;
esac

echo "RL node is still running; press Ctrl+C to stop."
wait "${RL_PID}"
