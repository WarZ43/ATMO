import re
import unittest
from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "atmo" / "rl_controller_hardware.py").read_text(
    encoding="utf-8"
)
LAUNCH_SOURCE = (Path(__file__).parents[1] / "launch" / "rl_control.launch.py").read_text(
    encoding="utf-8"
)
TOPICS_SOURCE = (Path(__file__).parents[1] / "atmo" / "px4_topics.py").read_text(
    encoding="utf-8"
)


def method_body(source, name):
    """Return the body of `def name(...)` up to the next method at the same indent."""
    match = re.search(r"\n    def %s\(.*?\n(.*?)(?=\n    (?:@|def ))" % re.escape(name),
                      source, re.DOTALL)
    if match is None:
        raise AssertionError("method %s not found" % name)
    return match.group(1)


class CombinedHardwareStaticTest(unittest.TestCase):
    def test_terminal_handoff_contract(self):
        # Terminal handoff still exists, and the FULL route must not treat
        # takeoff_to_flight as terminal (hover and landing come after it).
        self.assertIn("transition in terminal", SOURCE)
        self.assertIn('{"takeoff_to_flight", "landing_to_drive"}', SOURCE)
        self.assertIn('{"landing_to_drive"}', SOURCE)
        self.assertIn('self.route == "full"', SOURCE)
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
        self.assertIn(
            'ACTION_NAMES = ("lift", "roll", "pitch", "yaw", "tilt", "drive", "turn")',
            SOURCE)
        self.assertNotIn("ATMO_RL_FIXED_ACTION", SOURCE)
        # Ratchet is still the default; autostart (bench, non-rotor only)
        # bypasses the START choreography but must keep every STOP path.
        self.assertIn('self.test_phase = "ready" if self.autostart else "waiting_low"', SOURCE)
        self.assertIn("self.action_test not in ROTOR_ACTIONS", SOURCE)
        self.assertIn("self.rc_deadman_ok = False", SOURCE)
        self.assertIn('self.test_phase = "waiting_trigger"', SOURCE)
        self.assertIn('self.test_phase = "ready"', SOURCE)
        self.assertIn("Run the propeller-free lift/kill test first", SOURCE)

    def test_single_gate_airframe_is_configurable(self):
        """A negative offboard channel means 'this airframe has one switch'."""
        self.assertIn("OFFBOARD_GATE_ENABLED = OFFBOARD_CHANNEL >= 0", SOURCE)
        self.assertIn(
            "return (self.rl_switch, not self.rl_switch, not self.rl_switch, False)",
            SOURCE)
        # The channel numbers are 0-based indices here and 1-based in CATMO.
        # If this note goes, the next person re-derives it from a moving robot.
        self.assertIn("0-BASED INDICES", SOURCE)

    def test_single_gate_still_requires_a_full_cycle(self):
        """One switch must be cycled low->high->low->high, never a single flip."""
        body = method_body(SOURCE, "_action_test_timer")
        # waiting_low -> waiting_prepare -> waiting_trigger -> ready -> running.
        # With one switch these alternate, so the sequence cannot be short-cut
        # by leaving the switch HIGH when the node starts.
        for phase in ("waiting_prepare", "waiting_trigger", "ready"):
            self.assertIn('self.test_phase = "%s"' % phase, body)
        self.assertIn("raised, lowered, half, abort = self._action_test_gates()", body)

    def test_fail_closed_paths_are_never_conditional_on_the_gate_scheme(self):
        """The deadman is not configurable. Only the ratchet is.

        rc_lost, rc_failsafe, implausible pulse widths and the RC watchdog must
        drop the gates regardless of how many switches the airframe has. If any
        of these grows a reference to OFFBOARD_GATE_ENABLED, a config flag has
        become able to disable the only thing that stops tilt and drive.
        """
        for name in ("rc_listener_callback", "_rc_watchdog", "_channel_high"):
            body = method_body(SOURCE, name)
            self.assertNotIn(
                "OFFBOARD_GATE_ENABLED", body,
                "%s must not depend on the gate scheme" % name)
        watchdog = method_body(SOURCE, "_rc_watchdog")
        self.assertIn("self.rl_switch = False", watchdog)
        self.assertIn("self.offboard_switch = False", watchdog)
        listener = method_body(SOURCE, "rc_listener_callback")
        self.assertIn("if msg.rc_lost or msg.rc_failsafe:", listener)
        self.assertIn("self.rl_switch = False", listener)

    def test_ctrl_c_stops_before_teardown(self):
        """rclpy.spin raises KeyboardInterrupt; the stop must be in a finally.

        Without this the node dies with its last non-zero tilt and drive
        command still in flight, and a RoboClaw holds its last command
        forever. Stopping is not cleanup here, it is the stop.
        """
        main = method_body(SOURCE, "main") if "\n    def main" in SOURCE else SOURCE.split("def main(")[1]
        self.assertIn("except KeyboardInterrupt", main)
        self.assertIn("finally:", main)
        self.assertIn('node.emergency_stop("shutdown")', main)
        # The stop must come BEFORE teardown, or it publishes into a dead node.
        self.assertLess(main.index("emergency_stop"), main.index("destroy_node"))

    def test_kill_switch_stops_tilt_and_drive(self):
        """PX4's kill covers rotors only; this carries it to the RoboClaws."""
        body = method_body(SOURCE, "rc_listener_callback")
        self.assertIn("kill_engaged(msg.values, msg.rc_lost, msg.rc_failsafe)", body)
        self.assertIn('self.emergency_stop("kill switch engaged")', body)
        self.assertIn("self.rl_switch = False", body)

    def test_emergency_stop_repeats_and_never_raises(self):
        """A dropped zero is a motor that never stops, so send it more than once."""
        body = method_body(SOURCE, "emergency_stop")
        self.assertIn("for _ in range(3)", body)
        self.assertIn("self._publish_tilt_vel(0.0)", body)
        self.assertIn("self._publish_drive_vel(0.0, 0.0)", body)
        # Every publish is guarded: this must finish even mid-teardown.
        self.assertGreaterEqual(body.count("except Exception"), 3)

    def test_ground_mode_cannot_reach_the_rotors(self):
        """Ground mode never even CREATES the rotor publisher."""
        self.assertIn('VALID_MODES = {"policy", "ground", "action_test", "sensor_test", "shadow"}', SOURCE)
        self.assertIn('POLICY_MODES = {"policy", "ground"}', SOURCE)
        self.assertIn('if self.mode != "ground":', SOURCE)
        # Slice to the start of the non-ground path, not the first `return` --
        # the ground block contains an early return inside the handoff branch.
        ground = SOURCE.split('if self.mode == "ground":')[1].split("self._switch_to_offboard()")[0]
        self.assertNotIn("_publish_actuator_motors", ground)
        self.assertNotIn("_switch_to_offboard", ground)
        self.assertIn("_publish_policy_command(include_rotors=False)", ground)

    def test_sensor_mode_has_no_command_publishers(self):
        self.assertIn('self.mode not in {"shadow", "sensor_test"}', SOURCE)
        # Motive's rigid body on this rig is "M4", overridable with
        # ATMO_MOCAP_POSE_TOPIC. The default matters: a wrong body name gives
        # no topic and no error.
        self.assertIn('"/vrpn_mocap/M4/pose"', SOURCE)
        self.assertIn("ATMO_MOCAP_POSE_TOPIC", SOURCE)
        self.assertIn('px4_topics.resolve(self, "vehicle_visual_odometry")', SOURCE)

    def test_px4_topics_are_never_hardcoded(self):
        """No bare /fmu/ literal may remain in the runtime.

        PX4 1.16+ versions some topic names per message, so a hardcoded name is
        correct only for the firmware it was written against. It also fails
        silently -- the subscription never fires and the node looks healthy --
        which is how /fmu/out/vehicle_status went unnoticed until it was
        measured against a real flight controller.

        Every name belongs in atmo/px4_topics.py, where it has a fallback and
        an ATMO_PX4_TOPIC_<NAME> override.
        """
        leaked = re.findall(r'"(/fmu/[^"]*)"', SOURCE)
        self.assertEqual(
            leaked, [],
            "hardcoded PX4 topics in rl_controller_hardware.py: %s. "
            "Route them through atmo/px4_topics.py." % leaked,
        )

    def test_versioned_topic_names_are_used_where_the_firmware_needs_them(self):
        # Measured on Cube Orange / PX4 1.17.0; see docs/px4_topics.md.
        self.assertIn('"vehicle_status": "/fmu/out/vehicle_status_v1"', TOPICS_SOURCE)
        self.assertIn('"battery_status": "/fmu/out/battery_status_v1"', TOPICS_SOURCE)
        # ...and NOT where it does not. These are unversioned on 1.17, and
        # "fixing" them by adding _v1 would break the working path.
        self.assertIn('"vehicle_odometry": "/fmu/out/vehicle_odometry"', TOPICS_SOURCE)
        self.assertIn('"input_rc": "/fmu/out/input_rc"', TOPICS_SOURCE)
        self.assertIn('"actuator_motors": "/fmu/in/actuator_motors"', TOPICS_SOURCE)

    def test_bag_records_the_names_this_firmware_actually_publishes(self):
        # A stale bag topic name is recorded silently: you discover after the
        # flight that the data was never captured.
        self.assertIn("'/fmu/out/vehicle_status_v1'", LAUNCH_SOURCE)
        self.assertNotIn("'/fmu/out/vehicle_status'", LAUNCH_SOURCE)
        self.assertNotIn("'/fmu/out/esc_status'", LAUNCH_SOURCE)


if __name__ == "__main__":
    unittest.main()
