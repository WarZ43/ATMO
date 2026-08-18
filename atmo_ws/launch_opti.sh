# Distro-agnostic: the robot runs Humble, not the Foxy this was written
# against. scripts/atmo_env.sh detects it and sources the workspace.
#
# LEGACY: this ran the VRPN client on the Jetson, back when the Motive machine
# sat on the robot router. Motive is now FIXED at 10.0.0.3 on the wired mocap
# subnet, which the Jetson cannot reach -- the VRPN client belongs on the
# laptop now (scripts/run_mocap_laptop.sh). Kept only for a rig where the
# machine running this script has a route to the Motive host.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/atmo_env.sh"
ros2 launch vrpn_mocap client.launch.yaml server:="${ATMO_VRPN_SERVER:-10.0.0.3}" port:="${ATMO_VRPN_PORT:-3883}"
