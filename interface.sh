#!/usr/bin/env bash
# uXRCE-DDS agent: the bridge between PX4 and ROS 2.
#
# Three things have to agree or you get ZERO /fmu/ topics from an agent that
# looks perfectly healthy:
#
#   1. The DEVICE must be the adapter on the port PX4 runs its client on.
#      PX4's UXRCE_DDS_CFG says which (102 = TELEM2 on this vehicle).
#   2. The BAUD must match PX4's SER_TEL<n>_BAUD. Measured 2026-08-14: 921600
#      made the session flap -- repeated `session re-established` with only ~6
#      of ~67 topics created, because the session reset before topic creation
#      finished. 460800 is stable. PX4's payload is ~35 kB/s (~350 kbaud), so
#      460800 has headroom and 230400 would not.
#   3. The DOMAIN must match PX4's UXRCE_DDS_DOM_ID, which is 0 here.
#      ROS_DOMAIN_ID is an environment variable, so an agent started from a
#      shell that set it differently silently publishes where nobody looks.
#
# A healthy start is ONE `create_participant` followed by a burst of
# `create_topic` / `create_datawriter`, then quiet. Repeated
# `session re-established` means the link is too fast or too marginal.
#
# See the README for the measured configuration.

set -euo pipefail

ATMO_XRCE_DEV="${ATMO_XRCE_DEV:-/dev/ttyUSB0}"
ATMO_XRCE_BAUD="${ATMO_XRCE_BAUD:-921600}"
export ROS_DOMAIN_ID="${ATMO_ROS_DOMAIN_ID:-0}"

if [[ ! -e "${ATMO_XRCE_DEV}" ]]; then
    echo "interface.sh: ${ATMO_XRCE_DEV} does not exist." >&2
    echo "Check the USB adapter and 'ls -l /dev/serial/by-path/'." >&2
    exit 1
fi

echo "uXRCE agent: dev=${ATMO_XRCE_DEV} baud=${ATMO_XRCE_BAUD} domain=${ROS_DOMAIN_ID}"
echo "This must match PX4's UXRCE_DDS_CFG, SER_TEL<n>_BAUD and UXRCE_DDS_DOM_ID."
# When the ROS 2 stack runs against a discovery server (the arena/mocap
# topology), the agent MUST register there too or PX4 data is invisible to
# every node: measured 2026-08-17, agent on plain multicast + nodes on the
# server formed two parallel worlds with no error anywhere.
if [ -n "${ATMO_DDS_DISCOVERY_SERVER:-}" ]; then
    export ROS_DISCOVERY_SERVER="${ATMO_DDS_DISCOVERY_SERVER}"
    echo "agent joining discovery server ${ROS_DISCOVERY_SERVER}"
fi

exec MicroXRCEAgent serial --dev "${ATMO_XRCE_DEV}" -b "${ATMO_XRCE_BAUD}" "$@"
