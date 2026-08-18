"""The signed phase-event timer, and why drive/flight must read zero.

This is the fifth task observation. The runtime built only the four phase
one-hots, giving 528 against the contract's 529, which deliberately blocked
closed-loop policy mode.

Semantics are ported from the training task (combined_task._task_observation);
these tests exist so a future edit cannot quietly diverge from it again. In
particular DRIVE and FLIGHT read zero ON PURPOSE -- their scheduled event is
the transition itself, and announcing it would reveal the route before the
mode one-hot does. That reads like a bug and is not one.
"""

import json
import pathlib
import unittest

import numpy as np

from atmo.rl_combined_runtime import (
    CombinedStage1Config, DRIVE, TAKEOFF, FLIGHT, LANDING)
import atmo.rl_combined_runtime as runtime


CONTRACT = json.loads(
    (pathlib.Path(__file__).parents[1] / "atmo" / "contracts"
     / "atmo_combined_v1.json").read_text(encoding="utf-8"))


class _Stub:
    """Just enough of the builder to exercise _phase_event_time."""

    def __init__(self, mode, phase_time, landing_duration=3.0, clip=5.0):
        self.mode = mode
        self._pt = phase_time
        self.landing_duration = landing_duration
        self.cfg = CombinedStage1Config()
        self.cfg.phase_event_time_clip_s = clip

    def _phase_time(self):
        return self._pt

    value = property(
        lambda self: runtime.CombinedObservationBuilder._phase_event_time(self))


class PhaseEventTimeTest(unittest.TestCase):
    def test_drive_and_flight_read_zero(self):
        """Deliberate: announcing their transition would leak the route."""
        self.assertEqual(_Stub(DRIVE, 2.5).value, 0.0)
        self.assertEqual(_Stub(FLIGHT, 2.5).value, 0.0)

    def test_takeoff_reports_phase_time(self):
        self.assertAlmostEqual(_Stub(TAKEOFF, 1.25).value, 1.25)

    def test_landing_is_signed_around_touchdown(self):
        """Negative BEFORE the expected touchdown, positive after."""
        self.assertAlmostEqual(_Stub(LANDING, 1.0, landing_duration=3.0).value, -2.0)
        self.assertAlmostEqual(_Stub(LANDING, 3.0, landing_duration=3.0).value, 0.0)
        self.assertAlmostEqual(_Stub(LANDING, 4.5, landing_duration=3.0).value, 1.5)

    def test_clipped_both_ways(self):
        self.assertEqual(_Stub(TAKEOFF, 900.0).value, 5.0)
        self.assertEqual(_Stub(LANDING, 0.0, landing_duration=900.0).value, -5.0)

    def test_non_finite_becomes_zero(self):
        self.assertEqual(_Stub(TAKEOFF, float("nan")).value, 0.0)
        self.assertEqual(_Stub(TAKEOFF, float("inf")).value, 5.0)

    def test_clip_matches_the_training_task(self):
        self.assertEqual(CombinedStage1Config().phase_event_time_clip_s, 5.0)


class ContractWidthTest(unittest.TestCase):
    def test_runtime_width_matches_the_contract(self):
        cfg = CombinedStage1Config()
        self.assertEqual(cfg.task_observation_dim, CONTRACT["task_observation_dim"])

    def test_total_observation_dim_matches_the_contract(self):
        cfg = CombinedStage1Config()
        total = (CONTRACT["history_obs_dim"] * CONTRACT["observation_history_length"]
                 + CONTRACT["current_obs_dim"]
                 + CONTRACT["action_dim"] * CONTRACT["action_history_length"]
                 + CONTRACT["task_observation_dim"])
        self.assertEqual(cfg.observation_dim, total)
        self.assertEqual(cfg.observation_dim, 529)

    def test_the_fifth_task_observation_is_the_timer(self):
        self.assertEqual(CONTRACT["task_observation"],
                         ["drive", "takeoff", "flight", "landing", "phase_event_time_s"])


if __name__ == "__main__":
    unittest.main()
