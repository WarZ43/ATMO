# Copyright 2026 ATMO contributors
# Licensed under the Apache License, Version 2.0

"""HOVER-ONLY: hold station in FLIGHT at the measured pose, indefinitely.

The flight-side counterpart to drive_only, and the default for the shadow
profile. Every named route moves the reference, so none of them shows what the
policy does when asked to stay exactly where it is -- the one case where the
reference error starts at zero, so any command observed is the policy reacting
to the vehicle rather than tracking a moving setpoint.

The properties that matter:

  1. The reference is the measured pose, z INCLUDED -- no climb, no descent.
  2. It never moves, and FLIGHT never hands over to LANDING, however long the
     run lasts.
  3. It is mutually exclusive with drive_only, loudly rather than silently.
"""

import os
import unittest
from unittest.mock import patch

import numpy as np

from atmo.rl_combined_runtime import (
    CombinedObservationBuilder,
    CombinedStage1Config,
    FLIGHT,
    LANDING_ROUTE,
)

POSE = np.array((1.5, -2.25, 0.83), dtype=np.float32)
LEVEL = (1.0, 0.0, 0.0, 0.0)


def _anchored_builder():
    with patch.dict(os.environ, {"ATMO_RL_HOVER_ONLY": "1"}):
        builder = CombinedObservationBuilder(CombinedStage1Config())
    builder.update_training_frame_state(POSE, LEVEL, np.zeros(3), np.zeros(3))
    builder.anchor_fixed_vertical_route()
    return builder


class HoverOnlyTest(unittest.TestCase):
    def test_holds_the_measured_pose_including_z(self):
        builder = _anchored_builder()
        self.assertTrue(builder.hover_only)
        self.assertEqual(builder.route, LANDING_ROUTE)
        self.assertEqual(builder.mode, FLIGHT)
        # The default landing target is 0.20 m; hover-only must NOT use it.
        np.testing.assert_allclose(builder.landing_position, POSE, atol=1e-6)
        np.testing.assert_allclose(builder.landing_start, POSE, atol=1e-6)

    def test_reference_never_moves_and_never_lands(self):
        builder = _anchored_builder()
        for elapsed in (0.0, 5.0, 60.0, 600.0):
            builder.phase_elapsed_s = elapsed
            reference = builder._reference_state()[0]
            np.testing.assert_allclose(reference, POSE, atol=1e-6)
            self.assertIsNone(builder._transition_mode())
            self.assertEqual(builder.mode, FLIGHT)

    def test_rejects_combination_with_drive_only(self):
        env = {"ATMO_RL_HOVER_ONLY": "1", "ATMO_RL_DRIVE_ONLY": "1"}
        with patch.dict(os.environ, env):
            with self.assertRaises(ValueError):
                CombinedObservationBuilder(CombinedStage1Config())


if __name__ == "__main__":
    unittest.main()
