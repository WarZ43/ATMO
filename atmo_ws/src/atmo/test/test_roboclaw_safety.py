"""The RoboClaw gate must fail closed on every unknown.

These motors are outside PX4's kill entirely, and the boards latch their last
command indefinitely, so this gate plus the zero-on-exit path are the only
software stops that exist. Every case here is a way the vehicle moved, or
could have moved, with nobody commanding it.
"""

import unittest

from atmo.roboclaw_safety import gate_open, kill_engaged, stop_all


def channels(rl=1934, kill=944, length=18):
    """An RC frame with the measured endpoints for this vehicle.

    `kill` defaults to the RELEASED value: 944 us, which is the lever
    physically UP. Killed is 2084 us, lever physically DOWN.
    """
    values = [1514] * length
    if 7 < length:
        values[7] = rl
    if 12 < length:
        values[12] = kill
    return values


class FakeRoboclaw:
    def __init__(self, supported=None, failing=()):
        self.calls = []
        self.failing = set(failing)
        self._supported = supported

    def __getattr__(self, name):
        if self._supported is not None and name not in self._supported:
            raise AttributeError(name)

        def call(*args):
            if name in self.failing:
                raise IOError("simulated serial failure")
            self.calls.append((name, args))
        return call


class GateFailsClosedTest(unittest.TestCase):
    def test_open_only_when_gate_high_and_kill_disengaged(self):
        self.assertTrue(gate_open(channels(rl=1934, kill=944)))

    def test_gate_low_closes(self):
        self.assertFalse(gate_open(channels(rl=1094)))

    def test_kill_polarity_follows_the_configured_sense(self):
        """Polarity is configurable; the fail-closed cases are not.

        Active-HIGH is correct for this airframe, in MICROSECONDS:
        2084 us = killed, 944 us = released. The lever is reversed relative
        to the channel -- lever down is 2084 us -- which is what produced an
        inversion in this file once already.
        """
        import atmo.roboclaw_safety as rs
        original = rs.KILL_ACTIVE_HIGH
        try:
            rs.KILL_ACTIVE_HIGH = True
            self.assertTrue(rs.kill_engaged(channels(kill=2084)))
            self.assertFalse(rs.kill_engaged(channels(kill=944)))
            rs.KILL_ACTIVE_HIGH = False
            self.assertTrue(rs.kill_engaged(channels(kill=944)))
            self.assertFalse(rs.kill_engaged(channels(kill=2084)))
        finally:
            rs.KILL_ACTIVE_HIGH = original

    def test_fail_closed_cases_ignore_polarity(self):
        """Lost RC and implausible widths kill under EITHER polarity."""
        import atmo.roboclaw_safety as rs
        original = rs.KILL_ACTIVE_HIGH
        try:
            for setting in (True, False):
                rs.KILL_ACTIVE_HIGH = setting
                self.assertTrue(rs.kill_engaged(channels(), rc_lost=True))
                self.assertTrue(rs.kill_engaged(channels(), rc_failsafe=True))
                self.assertTrue(rs.kill_engaged(channels(kill=0)))
                self.assertTrue(rs.kill_engaged([1514] * 4))
        finally:
            rs.KILL_ACTIVE_HIGH = original

    def test_rc_lost_or_failsafe_closes(self):
        self.assertFalse(gate_open(channels(), rc_lost=True))
        self.assertFalse(gate_open(channels(), rc_failsafe=True))
        self.assertTrue(kill_engaged(channels(), rc_lost=True))

    def test_implausible_pulse_width_closes(self):
        """A dead or unmapped channel is not a switch position."""
        self.assertFalse(gate_open(channels(rl=0)))
        self.assertFalse(gate_open(channels(kill=0)))
        self.assertTrue(kill_engaged(channels(kill=0)))
        self.assertFalse(gate_open(channels(rl=1934, kill=0)))

    def test_missing_channels_close(self):
        """A frame too short to contain the gate must not read as raised."""
        self.assertFalse(gate_open([1514] * 4))
        self.assertTrue(kill_engaged([1514] * 4))


class StopAllTest(unittest.TestCase):
    def test_sends_zero_by_every_route(self):
        rc = FakeRoboclaw()
        self.assertTrue(stop_all(rc, 0x80))
        names = [name for name, _ in rc.calls]
        for expected in ("ForwardM1", "BackwardM1", "ForwardM2", "BackwardM2"):
            self.assertIn(expected, names)
        for _, args in rc.calls:
            self.assertEqual(args[1], 0, "stop must command zero")

    def test_one_failing_opcode_does_not_block_the_others(self):
        """The board is latched; a single unsupported opcode must not win."""
        rc = FakeRoboclaw(failing=("ForwardM1", "SpeedM1"))
        self.assertTrue(stop_all(rc, 0x80))
        self.assertIn("BackwardM2", [name for name, _ in rc.calls])

    def test_reports_failure_when_nothing_is_acknowledged(self):
        rc = FakeRoboclaw(supported=())
        self.assertFalse(stop_all(rc, 0x80))


if __name__ == "__main__":
    unittest.main()
