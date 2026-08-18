# Copyright 2026 ATMO contributors
# Licensed under the Apache License, Version 2.0

"""The FULL route: takeoff, hover flight_duration seconds, land at the start.

The seam this file pins is the hover-to-landing handover, which no single
training episode contains. The properties that matter:

  1. The mode sequence is DRIVE -> TAKEOFF -> FLIGHT -> LANDING -> DRIVE.
  2. The reference is CONTINUOUS at the handover -- landing_start is the
     reference hover point, never the measured position.
  3. The landing target is the anchored ground z (the vehicle took off from
     its wheels; that pose is the ground truth).
  4. takeoff_to_flight is NOT terminal on this route; landing_to_drive is.
"""

import time
import unittest
from unittest.mock import patch

import numpy as np

from atmo.rl_combined_runtime import (
    CombinedObservationBuilder,
    CombinedStage1Config,
    DRIVE,
    FLIGHT,
    LANDING,
    TAKEOFF,
)


def make_config():
    return CombinedStage1Config(
        randomize_reset=False,
        randomize_motor_dynamics=False,
        observation_noise=False,
        observation_delay_min_steps=0,
        observation_delay_max_steps=0,
    )


def make_full_builder(hover_s="3.0", ground_z_ned=-0.2):
    # anchor_fixed_vertical_route reads ATMO_RL_HOVER_S, so the whole
    # setup must run inside the env patch, not just the constructor.
    with patch.dict(
        "os.environ",
        {
            "ATMO_RL_ROUTE": "full",
            "ATMO_RL_HOVER_S": hover_s,
            "ATMO_RL_DETERMINISTIC_TRAJECTORY": "1",
        },
    ):
        builder = CombinedObservationBuilder(make_config())
        builder.update_px4_state(
            position=(2.0, 3.0, ground_z_ned),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        builder.reset_policy_context()
        builder.anchor_fixed_vertical_route()
    return builder


def advance_phase(builder, seconds):
    """Move the wall-clock phase origin back so _phase_time() reads seconds."""
    builder.phase_elapsed_s = seconds
    builder.phase_wall_start_time = time.monotonic()


class TestFullRoute(unittest.TestCase):
    def test_full_route_walks_all_four_modes_in_order(self):
        builder = make_full_builder()
        self.assertEqual(builder.mode, DRIVE)
        self.assertTrue(builder.full_mission)

        advance_phase(builder, builder.drive_duration)
        self.assertEqual(builder._transition_mode(), "drive_to_takeoff")
        self.assertEqual(builder.mode, TAKEOFF)

        advance_phase(builder, builder.takeoff_duration)
        self.assertEqual(builder._transition_mode(), "takeoff_to_flight")
        self.assertEqual(builder.mode, FLIGHT)

        # Mid-hover: no transition yet.
        advance_phase(builder, builder.flight_duration * 0.5)
        self.assertIsNone(builder._transition_mode())
        self.assertEqual(builder.mode, FLIGHT)

        advance_phase(builder, builder.flight_duration)
        self.assertEqual(builder._transition_mode(), "flight_to_landing")
        self.assertEqual(builder.mode, LANDING)

        # Touchdown: at the landing reference, slow, past landing_duration.
        builder.update_px4_state(
            position=(2.0, 3.0, -0.2),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        advance_phase(builder, builder.landing_duration)
        self.assertEqual(builder._transition_mode(), "landing_to_drive")
        self.assertEqual(builder.mode, DRIVE)

    def test_hover_duration_comes_from_env(self):
        builder = make_full_builder(hover_s="5.5")
        self.assertAlmostEqual(builder.flight_duration, 5.5)

    def test_reference_is_continuous_at_the_landing_handover(self):
        builder = make_full_builder()
        advance_phase(builder, builder.drive_duration)
        builder._transition_mode()
        advance_phase(builder, builder.takeoff_duration)
        builder._transition_mode()

        # End of hover, still FLIGHT: reference sits at the hover point.
        advance_phase(builder, builder.flight_duration)
        ref_before, vel_before, _ = builder._reference_state()

        # Perturb the MEASURED position; the handover must not follow it.
        builder.update_px4_state(
            position=(2.4, 3.3, -1.5),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        self.assertEqual(builder._transition_mode(), "flight_to_landing")
        ref_after, vel_after, _ = builder._reference_state()

        np.testing.assert_allclose(ref_after, ref_before, atol=1e-5)
        np.testing.assert_allclose(vel_after, vel_before, atol=1e-5)
        np.testing.assert_allclose(builder.landing_start, ref_before, atol=1e-5)

    def test_landing_target_is_the_anchored_ground_z(self):
        builder = make_full_builder(ground_z_ned=-0.23)
        anchored_z = float(builder.position[2])  # training frame, z up
        advance_phase(builder, builder.drive_duration)
        builder._transition_mode()
        advance_phase(builder, builder.takeoff_duration)
        builder._transition_mode()
        advance_phase(builder, builder.flight_duration)
        builder._transition_mode()
        self.assertEqual(builder.mode, LANDING)
        self.assertAlmostEqual(float(builder.landing_position[2]), anchored_z, places=5)
        # x/y: land where it took off (zero drive velocity route).
        np.testing.assert_allclose(builder.landing_position[:2], builder.takeoff_end[:2], atol=1e-5)

    def test_single_takeoff_route_still_parks_in_flight(self):
        with patch.dict(
            "os.environ",
            {"ATMO_RL_ROUTE": "takeoff", "ATMO_RL_DETERMINISTIC_TRAJECTORY": "1"},
        ):
            builder = CombinedObservationBuilder(make_config())
        builder.update_px4_state(
            position=(0.0, 0.0, -0.2),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        builder.reset_policy_context()
        builder.anchor_fixed_vertical_route()
        advance_phase(builder, builder.drive_duration)
        builder._transition_mode()
        advance_phase(builder, builder.takeoff_duration)
        builder._transition_mode()
        self.assertEqual(builder.mode, FLIGHT)
        advance_phase(builder, 1000.0)
        self.assertIsNone(builder._transition_mode())
        self.assertEqual(builder.mode, FLIGHT)


if __name__ == "__main__":
    unittest.main()
