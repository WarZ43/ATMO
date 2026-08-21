#!/usr/bin/env bash
# Preflight: confirm this host can actually run and see an ATMO session.
#
# Run on BOTH the robot and the mocap laptop before a session, and compare the
# two outputs. Most "it broke again" moments on the m4 bring-up were one of
# these three lines differing between the machines.

set -uo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/atmo_env.sh"

status=0
note() { printf '  %-28s %s\n' "$1" "$2"; }
fail() { printf '  %-28s %s  <-- CHECK\n' "$1" "$2"; status=1; }

echo "ATMO host check: $(hostname)"
echo
echo "ROS environment (must MATCH on the robot and the laptop):"
note "ROS_DISTRO" "${ROS_DISTRO:-unset}"
note "ROS_DOMAIN_ID" "${ROS_DOMAIN_ID:-unset (defaults to 0)}"
note "RMW_IMPLEMENTATION" "${RMW_IMPLEMENTATION:-unset}"
if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    fail "CYCLONEDDS_URI" "${CYCLONEDDS_URI}"
else
    note "CYCLONEDDS_URI" "unset (good)"
fi
if [[ -n "${ROS_DISCOVERY_SERVER:-}" && "${ROS_DISCOVERY_SERVER}" == "${ATMO_DDS_DISCOVERY_SERVER:-}" ]]; then
    # Deliberate: the dual-homed laptop topology runs a discovery server
    # (see the README); atmo_env.sh sets both variables
    # together. A LEFTOVER server -- set without ATMO_DDS_DISCOVERY_SERVER --
    # still fails below, which is the failure mode this check exists for.
    note "ROS_DISCOVERY_SERVER" "${ROS_DISCOVERY_SERVER} (deliberate, matches ATMO_DDS_DISCOVERY_SERVER)"
elif [[ -n "${ROS_DISCOVERY_SERVER:-}" ]]; then
    fail "ROS_DISCOVERY_SERVER" "${ROS_DISCOVERY_SERVER} (forces off multicast)"
else
    note "ROS_DISCOVERY_SERVER" "unset (good)"
fi

echo
echo "Workspace:"
if [[ -r "${SCRIPT_DIR}/../install/setup.bash" ]]; then
    note "install/setup.bash" "present"
else
    fail "install/setup.bash" "MISSING -- run colcon build"
fi
for package in atmo custom_msgs px4_msgs; do
    if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
        note "package ${package}" "$(ros2 pkg prefix "${package}")"
    else
        fail "package ${package}" "not found"
    fi
done

echo
echo "Contract:"
contract="${ATMO_RL_CONTRACT:-${SCRIPT_DIR}/../src/atmo/atmo/contracts/atmo_combined_v1.json}"
if [[ -r "${contract}" ]]; then
    note "contract" "${contract}"
else
    fail "contract" "not readable at ${contract}"
fi

echo
echo "Policy:"
policy="${ATMO_RL_POLICY_PATH:-}"
if [[ -z "${policy}" ]]; then
    note "ATMO_RL_POLICY_PATH" "unset (runtime default will be used)"
elif [[ -r "${policy}" ]]; then
    note "policy" "${policy}"
    case "${policy}" in
        *.npz) note "policy format" "npz (numpy actor, no torch needed)" ;;
        *.pth) fail "policy format" "pth -- needs torch on this host; export with scripts/export_policy_npz.py" ;;
    esac
else
    fail "policy" "not readable at ${policy}"
fi

echo
echo "Serial devices:"
for device in /dev/ttyUSB0 /dev/ttyACM0; do
    if [[ -e "${device}" ]]; then
        note "${device}" "present"
    else
        note "${device}" "absent"
    fi
done

echo
echo "Live topics (empty is normal before startup; suspicious during a session):"
mapfile -t topics < <(timeout 5 ros2 topic list 2>/dev/null | head -20)
if [[ ${#topics[@]} -eq 0 ]]; then
    note "ros2 topic list" "empty"
else
    printf '  %s\n' "${topics[@]}"
fi

echo
# A raw multicast test proves nothing about DDS discovery -- it is plain UDP and
# passes on networks where discovery is broken. Recorded here so nobody spends
# an afternoon on it again.
echo "Note: 'ros2 multicast send/receive' is a raw UDP test. It passing does NOT"
echo "mean DDS discovery works. Compare the three env lines above instead."

if [[ ${status} -ne 0 ]]; then
    echo
    echo "One or more checks need attention."
fi
exit ${status}
