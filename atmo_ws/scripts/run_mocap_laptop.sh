#!/usr/bin/env bash
# Run this on the LAPTOP that is wired to Motive. Ported from m4-direct-rl's
# run_mocap_laptop.sh, with the values this vehicle actually uses.
#
#   laptop   run_mocap_laptop.sh          -> /vrpn_mocap/<body>/pose
#   jetson   ./atmo_session.sh ground --reference mocap
#              -> mocap_bridge -> /atmo/groundtruth_odom -> the policy
#
# The consume side is the bridge inside the policy runner's stack. There is
# nothing else to start on the Jetson.
#
# ---------------------------------------------------------------------------
# ROS_DOMAIN_ID IS 0 HERE, NOT 42.
#
# m4-direct-rl uses domain 42 everywhere and its docs call it "not optional".
# That is m4's convention, adopted to escape ghost publishers on the default
# domain. It is WRONG for this vehicle: ATMO's domain must equal the flight
# controller's UXRCE_DDS_DOM_ID, which is 0. Importing 42 from the m4 stack
# has already cost this project two debugging rounds -- a mismatch produces an
# empty topic list on the far machine with no error on either side.
#
# Do not "restore" 42 when porting anything else from that repo.
# ---------------------------------------------------------------------------
#
# DUAL-HOMED LAPTOP. Wired to Motive (10.0.0.x) and on the robot router
# (192.168.0.x). Fast DDS announces a participant on EVERY interface it can
# see, so the Jetson is handed a 10.0.0.x locator it cannot route to and the
# stream is dead with nothing logged anywhere.
#
# THE FIX IS A DISCOVERY SERVER, NOT AN INTERFACE WHITELIST.
#
# m4-direct-rl measured this on this same lab network on 2026-08-12: the
# Fast DDS interface whitelist "was tried three times and broke the stream
# each time, including on the laptop's OWN topic", because
# useBuiltinTransports=false also kills shared memory and forces same-host
# participants onto one NIC. Adding an explicit SHM descriptor did not rescue
# it. Its conclusion was to use the discovery server instead, and that
# sidesteps dual-homing entirely because discovery becomes unicast to an
# address named explicitly.
#
# scripts/fastdds_wifi_only.xml is kept for reference and is NOT used here.
#
# The catch, also measured and also costly: under a discovery server the ROS 2
# CLI joins as a plain CLIENT and learns only the endpoints it needs, so
# `ros2 topic list` returns EMPTY while data flows perfectly. SUPER_CLIENT is a
# profile rather than an env var, which is why fastdds_super_client.xml is
# committed and exported on BOTH machines.
#
#   ATMO_VRPN_SERVER    Motive host        (default 10.0.0.3)
#   ATMO_VRPN_PORT      VRPN port          (default 3883)
#   ATMO_ROS_DOMAIN_ID  must match the FC  (default 0)
#
#   ./scripts/run_mocap_laptop.sh
#   ATMO_VRPN_SERVER=10.0.0.7 ./scripts/run_mocap_laptop.sh
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ATMO_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${ATMO_RMW:-rmw_fastrtps_cpp}"
# Same reasoning as scripts/atmo_env.sh: a leftover discovery server or Cyclone
# URI silently breaks cross-machine discovery, in either direction.
unset CYCLONEDDS_URI
unset ROS_DISCOVERY_SERVER

SERVER="${ATMO_VRPN_SERVER:-10.0.0.3}"
PORT="${ATMO_VRPN_PORT:-3883}"
SUPER_CLIENT="${SCRIPT_DIR}/fastdds_super_client.xml"

# --- discovery server ------------------------------------------------------
# The address the Jetson will be told to contact. It must be this laptop's
# ROBOT-ROUTER address, not the mocap-subnet one and not campus WiFi.
ROUTER_IP="${ATMO_LAPTOP_ROUTER_IP:-$(ip -4 -o addr show 2>/dev/null |
    awk '{print $4}' | cut -d/ -f1 | grep -E '^192\.168\.0\.' | head -1)}"
if [[ -z "${ROUTER_IP}" ]]; then
    echo "This laptop has no 192.168.0.x address, so the Jetson has no way to" >&2
    echo "reach a discovery server here. Join the robot router first:" >&2
    ip -4 -brief addr show | sed 's/^/    /' >&2
    exit 2
fi

# Detect by the BOUND PORT, not by process name. Two traps here, both hit:
#   pgrep -f  matches the string anywhere in any command line, so it
#             self-matches the shell that invoked this script;
#   pgrep -x  matches `comm`, which Linux truncates to 15 characters, and
#             "fast-discovery-server" is 21 -- so it never matches at all.
# The port either is bound or it is not.
server_up() { ss -lun 2>/dev/null | grep -q ":11811 "; }
if ! server_up; then
    echo "Starting Fast DDS discovery server on ${ROUTER_IP}:11811"
    # -l is deliberately omitted so it listens on ALL interfaces; pinning it
    # means re-editing every time campus DHCP moves the laptop.
    setsid nohup fast-discovery-server -i 0 -p 11811 \
        > /tmp/atmo_discovery_server.log 2>&1 < /dev/null &
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        server_up && break
        sleep 0.3
    done
fi
if ! server_up; then
    echo "fast-discovery-server did not stay up. Log:" >&2
    tail -5 /tmp/atmo_discovery_server.log >&2 || true
    exit 2
fi

export ROS_DISCOVERY_SERVER="${ROUTER_IP}:11811"
# Without SUPER_CLIENT every introspection command returns empty while data
# flows. See the header.
[[ -r "${SUPER_CLIENT}" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="${SUPER_CLIENT}"
echo "Discovery server: ${ROS_DISCOVERY_SERVER}  (super-client profile exported)"
echo
echo "ON THE JETSON, before anything else in that shell:"
echo "    export ATMO_DDS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}"
echo "    source scripts/atmo_env.sh    # exports the super-client profile too"
echo "    ros2 daemon stop"

# The daemon caches discovery state across domains and profiles. A stale cache
# reports topics that no longer exist and omits ones that do.
ros2 daemon stop >/dev/null 2>&1 || true

# --- reachability, before launching something that will sit there silently --
if ! timeout 3 bash -c "cat < /dev/null > /dev/tcp/${SERVER}/${PORT}" 2>/dev/null; then
    echo >&2
    echo "Cannot reach VRPN at ${SERVER}:${PORT}." >&2
    echo "  Motive runs on Windows and its firewall blocks ping, so ICMP" >&2
    echo "  proves nothing -- but this is a TCP connect and it failed." >&2
    echo "  Check: Motive is running; Data Streaming has VRPN enabled;" >&2
    echo "         the wired NIC shares Motive's subnet." >&2
    echo "  Neighbours seen on the wired link:" >&2
    ip neigh show 2>/dev/null | grep -v FAILED | sed 's/^/    /' >&2 || true
    exit 2
fi

echo "VRPN ${SERVER}:${PORT}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  (0, matching UXRCE_DDS_DOM_ID)"
echo
echo "Rigid body name comes from Motive. m4's rig streams it as 'M4', so the"
echo "topic should be /vrpn_mocap/M4/pose. A wrong name gives NO topic and no"
echo "error -- list them rather than assuming:"
echo "    ros2 topic list | grep vrpn"
echo
echo "Then verify it CROSSES to the Jetson, which is the separate failure:"
echo "    ssh m4@192.168.0.44 'source /opt/ros/humble/setup.bash && \\"
echo "        ROS_DOMAIN_ID=0 ros2 topic list | grep vrpn'"
echo

exec ros2 launch vrpn_mocap client.launch.yaml \
    "server:=${SERVER}" "port:=${PORT}" "$@"
