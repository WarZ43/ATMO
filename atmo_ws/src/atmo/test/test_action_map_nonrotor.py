"""Pin the non-rotor policy action slots to physical motion.

Slots 4, 5 and 6 are tilt, drive and turn. Each one travels a long chain
before it reaches a motor:

    policy action -> LandingActionAdapter -> ActuatorCommand
                  -> /tilt_vel or /drive_vel
                  -> tilt_controller_hardware / drive_controller_hardware
                  -> Forward/Backward on a specific RoboClaw motor
                  -> a physical wheel or arm, in a specific direction

There are FOUR sign conventions in that chain -- the adapter negates raw[5],
the drive node negates again in update(), the two wheel sides are mounted
mirrored, and the tilt encoder counts negative away from fly. Tonight's
session found real inversions in two of them. A test that only checks
"something moved" cannot see any of this, which is exactly how the m4
bring-up recorded a passing gate that was inverted.

These are pure-Python: they assert the mapping, not the motion. The physical
half is scripts/action_map_check.py, which prints its prediction before it
commands anything.

Directions below are MEASURED on the vehicle 2026-08-14, not assumed:

    ForwardM1  -> RIGHT wheel drives vehicle FORWARD
    BackwardM1 -> RIGHT wheel drives vehicle BACKWARD
    ForwardM2  -> LEFT  wheel drives vehicle BACKWARD
    BackwardM2 -> LEFT  wheel drives vehicle FORWARD
    ForwardM2 on the TILT board -> angle INCREASES, away from fly config
"""

import unittest


# --- the measured hardware truth table -------------------------------------
# (board, roboclaw_call) -> what the vehicle physically does
PHYSICAL = {
    ("drive", "ForwardM1"): "right wheel forward",
    ("drive", "BackwardM1"): "right wheel backward",
    ("drive", "ForwardM2"): "left wheel backward",
    ("drive", "BackwardM2"): "left wheel forward",
    ("tilt", "ForwardM2"): "tilt angle increases",
    ("tilt", "BackwardM2"): "tilt angle decreases",
}


def drive_node_calls(drive_speed, turn_speed):
    """Reproduce drive_controller_hardware.update() -> RoboClaw calls.

    Mirrors the real code, including its negation and its per-side asymmetry:

        lin_vel = -map_speed(drive_speed)
        ang_vel = -map_speed(turn_speed)
        move_right_wheel(lin_vel + ang_vel)   # >0 -> BackwardM1, else ForwardM1
        move_left_wheel(lin_vel - ang_vel)    # >0 -> ForwardM2,  else BackwardM2
    """
    def map_speed(v):
        return int(127 * v)

    lin = -map_speed(drive_speed)
    ang = -map_speed(turn_speed)
    right, left = lin + ang, lin - ang
    calls = []
    if right != 0:
        calls.append("BackwardM1" if right > 0 else "ForwardM1")
    if left != 0:
        calls.append("ForwardM2" if left > 0 else "BackwardM2")
    return calls


def tilt_node_call(tilt_speed):
    """Reproduce tilt_controller_hardware.spin_motor() -> RoboClaw call."""
    if tilt_speed == 0:
        return None
    return "ForwardM2" if tilt_speed > 0 else "BackwardM2"


class DriveSlotTest(unittest.TestCase):
    def test_positive_drive_speed_moves_the_vehicle_forward(self):
        calls = drive_node_calls(drive_speed=1.0, turn_speed=0.0)
        motions = {PHYSICAL[("drive", c)] for c in calls}
        self.assertEqual(motions, {"right wheel forward", "left wheel forward"},
                         "positive drive_speed must drive both sides FORWARD")

    def test_negative_drive_speed_moves_the_vehicle_backward(self):
        calls = drive_node_calls(drive_speed=-1.0, turn_speed=0.0)
        motions = {PHYSICAL[("drive", c)] for c in calls}
        self.assertEqual(motions, {"right wheel backward", "left wheel backward"})

    def test_positive_turn_opposes_the_two_sides(self):
        """A turn must spin: the sides go opposite ways, or it is a drive."""
        calls = drive_node_calls(drive_speed=0.0, turn_speed=1.0)
        motions = {PHYSICAL[("drive", c)] for c in calls}
        self.assertEqual(motions, {"right wheel forward", "left wheel backward"})

    def test_negative_turn_is_the_mirror_of_positive(self):
        pos = {PHYSICAL[("drive", c)] for c in drive_node_calls(0.0, 1.0)}
        neg = {PHYSICAL[("drive", c)] for c in drive_node_calls(0.0, -1.0)}
        self.assertNotEqual(pos, neg)
        self.assertEqual(neg, {"right wheel backward", "left wheel forward"})

    def test_drive_and_turn_are_not_the_same_action(self):
        """The failure this exists to catch: drive and turn swapped."""
        drive = {PHYSICAL[("drive", c)] for c in drive_node_calls(1.0, 0.0)}
        turn = {PHYSICAL[("drive", c)] for c in drive_node_calls(0.0, 1.0)}
        self.assertNotEqual(drive, turn)

    def test_raw_forward_on_both_motors_would_spin_not_drive(self):
        """Measured on the vehicle, and the reason the asymmetry is correct."""
        motions = {PHYSICAL[("drive", "ForwardM1")], PHYSICAL[("drive", "ForwardM2")]}
        self.assertEqual(motions, {"right wheel forward", "left wheel backward"})


class TiltSlotTest(unittest.TestCase):
    def test_positive_tilt_speed_increases_the_angle(self):
        self.assertEqual(PHYSICAL[("tilt", tilt_node_call(1.0))],
                         "tilt angle increases")

    def test_negative_tilt_speed_decreases_the_angle(self):
        self.assertEqual(PHYSICAL[("tilt", tilt_node_call(-1.0))],
                         "tilt angle decreases")

    def test_zero_commands_nothing(self):
        self.assertIsNone(tilt_node_call(0.0))

    def test_tilt_is_not_inverted(self):
        """The pre-2026-08-14 bug: tilt_speed<0 was mapped to ForwardM2."""
        self.assertNotEqual(tilt_node_call(1.0), tilt_node_call(-1.0))
        self.assertEqual(tilt_node_call(1.0), "ForwardM2")


class AdapterSlotToNodeInputTest(unittest.TestCase):
    """Close the last link: policy action vector -> ActuatorCommand fields.

    The adapter NEGATES slot 5 (`drive_speed = -raw[5]`) and passes slot 6
    through unchanged. That negation is easy to lose in a refactor and would
    reverse the vehicle without breaking anything else.
    """

    def setUp(self):
        import numpy as np
        from atmo.rl_landing_stage1_runtime import LandingActionAdapter, LandingStage1Config
        self.np = np
        self.adapter = LandingActionAdapter(LandingStage1Config())

    def _cmd(self, tilt=0.0, drive=0.0, turn=0.0):
        action = self.np.zeros(7, dtype=self.np.float32)
        action[4], action[5], action[6] = tilt, drive, turn
        return self.adapter.pre_physics_step(action)

    def test_slot5_is_negated_into_drive_speed(self):
        self.assertLess(self._cmd(drive=1.0).drive_speed, 0.0)
        self.assertGreater(self._cmd(drive=-1.0).drive_speed, 0.0)

    def test_slot6_passes_through_into_turn_speed(self):
        self.assertGreater(self._cmd(turn=1.0).turn_speed, 0.0)
        self.assertLess(self._cmd(turn=-1.0).turn_speed, 0.0)

    def test_drive_and_turn_slots_are_independent(self):
        drive_only = self._cmd(drive=1.0)
        turn_only = self._cmd(turn=1.0)
        self.assertEqual(drive_only.turn_speed, 0.0)
        self.assertEqual(turn_only.drive_speed, 0.0)

    def test_slot5_positive_drives_the_vehicle_backward(self):
        """Documents the end-to-end sign, so a change to it is deliberate.

        raw[5]=+1 -> drive_speed<0 -> both wheels BACKWARD. If the training
        convention says +1 should mean forward, the negation is the thing to
        change, and this test is what will tell you.
        """
        drive_speed = self._cmd(drive=1.0).drive_speed
        motions = {PHYSICAL[("drive", c)] for c in drive_node_calls(drive_speed, 0.0)}
        self.assertEqual(motions, {"right wheel backward", "left wheel backward"})


if __name__ == "__main__":
    unittest.main()
