#!/usr/bin/env bash
# The operator window: what to do, in order, and what must be true first.
#
# This exists because a bench session under time pressure is where gates get
# skipped. Having the sequence on screen, in the session, next to the stop
# command, is what keeps the order honest.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROFILE="${1:-policy}"
SESSION="${2:-atmo-${PROFILE}}"
ROUTE="${3:-landing}"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/atmo_env.sh"

cat <<EOF

  ATMO session: profile=${PROFILE}  route=${ROUTE}  tmux=${SESSION}

  STOP:  Ctrl-C in the stack window, or the kill switch, or drop the RL gate
  The physical kill switch is the authority. The script is the software half.

EOF

case "${PROFILE}" in
    shadow)
        cat <<'EOF'
  SHADOW. This process creates no actuator, tilt, drive, offboard or arm
  publishers. Nothing can move from this window.

    1. Confirm the mocap window shows a steady rate and no gaps.
    2. Raise ONLY the RL switch to reset the reference and start logging.
    3. Watch the policy output: finite actions, collective near hover,
       attitude commands OPPOSING a hand-held tilt.

  What shadow does NOT validate: flight dynamics, the rotor map, and every
  frame convention that only appears under motion.
EOF
        ;;
    sensor)
        cat <<'EOF'
  SENSOR TEST. No command publishers. Move the unpowered vehicle by hand.

    1. Check message counts and ages for each subscribed topic.
    2. Check the PX4 NED pose and velocity track the hand motion.
    3. Check the tilt angle reads the physical arm position.
    4. Check the RC gates read the switch positions you are setting.
EOF
        ;;
    action)
        cat <<'EOF'
  ACTION TEST. Single axis, five seconds, RC gated.

    PROPELLERS OFF for lift/roll/pitch/yaw. Vehicle secured.

    1. Start with BOTH Offboard and RL switches LOW.
    2. Raise both, then lower ONLY RL.
    3. Raise RL to begin the five-second test.
    4. Lower either switch to stop early.

  The lift/kill test comes first and must prove the physical kill switch
  stops all four motors. Do not run roll, pitch or yaw until it passes;
  then pass --kill-test-passed to say so explicitly.
EOF
        ;;
    policy)
        cat <<'EOF'
  POLICY. Closed loop. Everything below must already be true.

    [ ] Stage C mocap gates passed THIS session (pose, twist, rate, static)
    [ ] Kill switch tested in both directions today
    [ ] Action tests passed for every axis, both signs
    [ ] Shadow run reviewed and sane
    [ ] Area clear, everyone behind the line

    1. Raise Offboard, confirm PX4 reports offboard in the stack window.
    2. Raise RL to hand control to the policy.
    3. Lower either switch to hand it back.
EOF
        ;;
esac

cat <<'EOF'

  Useful while running:
    ros2 topic hz /atmo/groundtruth_odom
    ros2 topic hz /fmu/in/actuator_motors
    ros2 topic echo /fmu/out/vehicle_status --once
    python3 scripts/hardware_optitrack_check.py --mode rate --duration 10

EOF

exec bash
