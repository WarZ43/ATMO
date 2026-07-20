import unittest
from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "atmo" / "rl_controller_hardware.py").read_text(encoding="utf-8")
LAUNCH_SOURCE = (Path(__file__).parents[1] / "launch" / "rl_control.launch.py").read_text(
    encoding="utf-8"
)


class CombinedHardwareStaticTest(unittest.TestCase):
    def test_terminal_handoff_contract(self):
        self.assertIn('transition in {"takeoff_to_flight", "landing_to_drive"}', SOURCE)
        self.assertIn('self.mode == "policy"', SOURCE)
        self.assertIn("VehicleStatus.NAVIGATION_STATE_POSCTL", SOURCE)
        self.assertIn("VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0", SOURCE)
        self.assertIn("not self.offboard_switch and not self.rl_switch", SOURCE)
        self.assertIn('Bool, "/atmo/rl/manual_override"', SOURCE)
        self.assertIn("self._publish_manual_override(True)", SOURCE)

    def test_launch_selects_route_and_records_handoff(self):
        self.assertIn("'ATMO_RL_ROUTE': route", LAUNCH_SOURCE)
        self.assertIn("'ATMO_RL_HARDWARE_MODE': hardware_mode", LAUNCH_SOURCE)
        self.assertIn("'/atmo/rl/manual_override'", LAUNCH_SOURCE)

    def test_named_action_tests_are_gated(self):
        self.assertIn('ACTION_NAMES = ("lift", "roll", "pitch", "yaw", "tilt", "drive", "turn")', SOURCE)
        self.assertNotIn("ATMO_RL_FIXED_ACTION", SOURCE)
        self.assertIn('self.test_phase = "waiting_low"', SOURCE)
        self.assertIn('self.test_phase = "waiting_trigger"', SOURCE)
        self.assertIn('self.test_phase = "ready"', SOURCE)
        self.assertIn("Run the propeller-free lift/kill test first", SOURCE)

    def test_sensor_mode_has_no_command_publishers(self):
        self.assertIn('self.mode not in {"shadow", "sensor_test"}', SOURCE)
        self.assertIn('"/vrpn_mocap/m4_base/pose"', SOURCE)
        self.assertIn('"/fmu/in/vehicle_visual_odometry"', SOURCE)


if __name__ == "__main__":
    unittest.main()
