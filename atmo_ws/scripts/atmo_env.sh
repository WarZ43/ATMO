#!/usr/bin/env bash
# Shared environment for every ATMO session window. Source this, do not run it.
#
# The override block is deliberately LAST in this file, and this file is
# deliberately sourced by every window. On the m4 bring-up an entire OptiTrack
# session was lost to the robot's ~/.bashrc sourcing a different autonomy stack
# that set ROS_DOMAIN_ID, CycloneDDS and a CYCLONEDDS_URI -- every network-layer
# test passed and the two machines still could not see each other, because they
# were on different domains with different middleware. Setting these explicitly
# per-window is what makes a session reproducible regardless of what a login
# shell did.
#
# Both machines -- the robot AND the mocap laptop -- must agree on all three.

# Remember whether the CALLER had `set -u`, and restore only what they had.
# Forcing it on leaks into an interactive shell, and then every later
# `source /opt/ros/.../setup.bash` dies on "AMENT_TRACE_SETUP_FILES: unbound
# variable" -- ROS setup scripts reference unset variables freely. The failure
# looks like a broken workspace rather than a broken shell option.
_atmo_had_nounset=0
case "$-" in *u*) _atmo_had_nounset=1 ;; esac
set +u

# Do NOT hardcode the distro. The ATMO scripts were written against Foxy, but
# the robot this deploys to runs Humble -- and hardcoding it means every script
# fails at line 1 on the machine it is meant to run on. Detect, honour an
# explicit override, and say which one was used.
if [[ -n "${ATMO_ROS_DISTRO:-}" ]]; then
    ATMO_DETECTED_DISTRO="${ATMO_ROS_DISTRO}"
elif [[ -n "${ROS_DISTRO:-}" && -r "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ATMO_DETECTED_DISTRO="${ROS_DISTRO}"
else
    ATMO_DETECTED_DISTRO=""
    for candidate in humble foxy iron jazzy; do
        if [[ -r "/opt/ros/${candidate}/setup.bash" ]]; then
            ATMO_DETECTED_DISTRO="${candidate}"
            break
        fi
    done
fi
if [[ -z "${ATMO_DETECTED_DISTRO}" || ! -r "/opt/ros/${ATMO_DETECTED_DISTRO}/setup.bash" ]]; then
    echo "atmo_env.sh: no ROS 2 install found under /opt/ros." >&2
    echo "Set ATMO_ROS_DISTRO to the one to use." >&2
    # Restore on this path too, or a failed source silently leaves the caller
    # with different shell options than it had.
    if [[ "${_atmo_had_nounset}" == "1" ]]; then set -u; fi
    unset _atmo_had_nounset
    return 1 2>/dev/null || exit 1
fi
source "/opt/ros/${ATMO_DETECTED_DISTRO}/setup.bash"
export ATMO_DETECTED_DISTRO

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
if [[ -r "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi
if [[ "${_atmo_had_nounset}" == "1" ]]; then set -u; fi
unset _atmo_had_nounset

# --- the override block: keep this at the end ------------------------------
# Default 0, because that is what PX4 uses: the flight controller's
# UXRCE_DDS_DOM_ID decides which DDS domain the bridge publishes on, it
# defaults to 0, and this vehicle is measured at 0. A mismatch here shows
# up as ZERO /fmu/ topics with a healthy-looking agent, which reads as a
# dead flight controller. The m4 project used 42 to escape a conflicting
# stack in that robot's ~/.bashrc; this robot has no such conflict, and
# importing 42 here cost two debugging rounds.
export ROS_DOMAIN_ID="${ATMO_ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${ATMO_RMW:-rmw_fastrtps_cpp}"
# A leftover discovery server keeps Fast DDS off multicast discovery even after
# the domain is right, and a discovery-server CLIENT shows empty `ros2 topic`
# and `ros2 node` lists while data flows perfectly well. Unset both.
unset CYCLONEDDS_URI
# ROS_DISCOVERY_SERVER is unset by default, because a STALE one is a classic
# silent breakage. But a discovery server is also the measured fix for the
# dual-homed laptop: m4-direct-rl tried the Fast DDS interface whitelist three
# times on this lab network and it broke the stream every time, including the
# laptop's own topic. So allow it to be requested DELIBERATELY.
if [[ -n "${ATMO_DDS_DISCOVERY_SERVER:-}" ]]; then
    export ROS_DISCOVERY_SERVER="${ATMO_DDS_DISCOVERY_SERVER}"
    # Under a discovery server the ROS 2 CLI joins as a plain CLIENT and
    # learns only the endpoints it needs, so `ros2 topic list` comes back
    # EMPTY while data flows perfectly. That cost m4 an evening on
    # 2026-08-12. SUPER_CLIENT is a profile, not an env var.
    export FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_DIR}/scripts/fastdds_super_client.xml"
else
    unset ROS_DISCOVERY_SERVER
fi

# RC gate channels, as 0-BASED INDICES into InputRc.values. Set here rather
# than left to the code defaults because getting them wrong is silent: the
# action-test ratchet simply never leaves `waiting_prepare` and reads as a
# broken script rather than a misconfigured gate.
#
# Measured on this vehicle 2026-08-14 by sweeping every switch on the
# transmitter. Only two switches are mapped at all:
#   ch8  (index 7)  1094 <-> 1934   the only free switch  -> RL gate
#   ch13 (index 12) 944  <-> 2084   kill, mirrored onto ch17
# ch9 (index 8), the offboard gate's inherited default, has never had a
# switch on it -- the channel map came from a Futaba T18SZ and the
# transmitter is a T14SG. -1 selects the single-gate ratchet, which cycles
# the RL switch twice instead. See the README.
export ATMO_RL_CHANNEL="${ATMO_RL_CHANNEL:-7}"
export ATMO_RL_OFFBOARD_CHANNEL="${ATMO_RL_OFFBOARD_CHANNEL:--1}"

# RoboClaw ports, BY-PATH. The code defaults are /dev/ttyACM0 and ttyACM1,
# which are USB enumeration order and not identity -- and both boards
# enumerate as `usb-Basicmicro_Inc._USB_Roboclaw_2x7A-if00`, so by-id cannot
# tell them apart either. Confirmed 2026-08-14: with both plugged in there is
# exactly ONE by-id symlink and the second board has none at all.
#
# by-path keys on the physical USB socket, so it is stable across reboots as
# long as nothing is replugged into a different port.
#
#   2.1 -> TILT     2.2 -> DRIVE
#
# MEASURED 2026-08-14 by commanding one motor at a time and watching, after
# two rounds of getting it wrong by inference:
#
#   2.1 M1   0.00 A, no motion          nothing connected
#   2.1 M2   1.25 A, encoder 0 -> -1    TILT: the only encoder that counted,
#                                       and the only board with one motor
#   2.2 M1   0.71 A, wheels turned      DRIVE
#   2.2 M2   0.68-1.10 A, wheels turned DRIVE
#
# A single connected motor with an encoder is the tilt board's signature; two
# motors and no encoders is the drive board's.
#
# THIS DISAGREES WITH ~/CATMO/atmo_ports.env, which says tilt=2.2 drive=2.1.
# His file is not wrong, it is STALE: the boards were moved between USB
# sockets during this session, and by-path names the socket, not the board.
# Anyone who replugs them must re-measure rather than trust either file.
#
# NOTE the variable NAMES differ between the two trees: Carlo's are
# ATMO_*_ROBOCLAW_PORT, these are ATMO_*_ROBOCLAW. Exporting one does not
# affect the other, so do not assume sourcing his env configures this tree.
#
# Getting this backwards drives the wheels with tilt commands, or worse, runs
# the tilt motor on drive commands -- the arms are on worm gears, are NOT
# backdrivable, and the travel guard only blocks one direction.
export ATMO_TILT_ROBOCLAW="${ATMO_TILT_ROBOCLAW:-/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0}"
export ATMO_DRIVE_ROBOCLAW="${ATMO_DRIVE_ROBOCLAW:-/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0}"
