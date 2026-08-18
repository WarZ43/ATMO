# Distro-agnostic: the robot runs Humble, not the Foxy this was written
# against. scripts/atmo_env.sh detects it and sources the workspace.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/atmo_env.sh"
ros2 run atmo drive_controller_hardware
