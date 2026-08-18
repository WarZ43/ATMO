"""Frame conversions for the OptiTrack bridge.

These are the functions where a sign error becomes a crash. None of this
replaces moving the vehicle by hand and watching the sign -- see
`scripts/hardware_optitrack_check.py`. What it does do is pin the arithmetic so
a refactor cannot quietly change a convention that a bench session confirmed.
"""

import math
import unittest

import numpy as np

from atmo.mocap_frames import (
    px4_position_atmo_legacy,
    px4_quaternion_atmo_legacy,
    px4_quaternion_composed,
    quat_conjugate,
    quat_multiply,
    rotate_by_quat_inverse,
    to_z_up,
    z_up_to_ned_position,
    z_up_to_ned_velocity,
)

IDENTITY = np.array((1.0, 0.0, 0.0, 0.0))


def yaw_quat(angle):
    return np.array((math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)))


class TestQuaternionAlgebra(unittest.TestCase):
    def test_identity_is_the_unit(self):
        q = yaw_quat(0.7)
        np.testing.assert_allclose(quat_multiply(IDENTITY, q), q, atol=1e-12)

    def test_conjugate_inverts(self):
        q = yaw_quat(1.1)
        np.testing.assert_allclose(
            quat_multiply(q, quat_conjugate(q)), IDENTITY, atol=1e-12
        )

    def test_rotating_into_the_body_frame_undoes_a_yaw(self):
        # A vehicle yawed +90 deg, moving along world +x, is moving along its
        # own -y. Getting this backwards inverts the body-frame twist.
        q = yaw_quat(math.pi / 2.0)
        body = rotate_by_quat_inverse(q, np.array((1.0, 0.0, 0.0)))
        np.testing.assert_allclose(body, (0.0, -1.0, 0.0), atol=1e-9)

    def test_body_frame_rotation_preserves_length(self):
        q = yaw_quat(0.37)
        vector = np.array((0.3, -1.2, 4.0))
        self.assertAlmostEqual(
            float(np.linalg.norm(rotate_by_quat_inverse(q, vector))),
            float(np.linalg.norm(vector)),
            places=9,
        )


class TestSourceFrame(unittest.TestCase):
    def test_z_up_passes_through_untouched(self):
        position = np.array((1.0, 2.0, 3.0))
        quaternion = yaw_quat(0.4)
        out_p, out_q = to_z_up(position, quaternion, "z_up")
        np.testing.assert_allclose(out_p, position)
        np.testing.assert_allclose(out_q, quaternion)

    def test_y_up_remaps_the_vertical_axis(self):
        # Motive y-up: the streamed y is up, and becomes world z.
        out_p, _ = to_z_up(np.array((1.0, 2.0, 3.0)), IDENTITY, "y_up")
        np.testing.assert_allclose(out_p, (1.0, -3.0, 2.0))

    def test_y_up_remap_is_right_handed(self):
        # A right-handed remap preserves the sign of the scalar triple product.
        basis = [np.array(v, dtype=float) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
        mapped = [to_z_up(v, IDENTITY, "y_up")[0] for v in basis]
        self.assertGreater(float(np.dot(mapped[0], np.cross(mapped[1], mapped[2]))), 0.0)

    def test_y_up_preserves_quaternion_norm(self):
        _, out_q = to_z_up(np.zeros(3), yaw_quat(0.9), "y_up")
        self.assertAlmostEqual(float(np.linalg.norm(out_q)), 1.0, places=12)

    def test_unknown_frame_is_rejected(self):
        with self.assertRaises(ValueError):
            to_z_up(np.zeros(3), IDENTITY, "enu")


class TestNed(unittest.TestCase):
    def test_position_mapping(self):
        self.assertEqual(z_up_to_ned_position((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))

    def test_velocity_uses_the_same_axes_as_position(self):
        self.assertEqual(
            z_up_to_ned_velocity((1.0, 2.0, 3.0)), z_up_to_ned_position((1.0, 2.0, 3.0))
        )

    def test_up_becomes_negative_down(self):
        # The single most consequential sign in the whole chain.
        self.assertLess(z_up_to_ned_position((0.0, 0.0, 1.5))[2], 0.0)


class TestPx4QuaternionConventions(unittest.TestCase):
    def test_both_mappings_preserve_norm(self):
        raw = np.array((0.5, 0.5, 0.5, 0.5))
        for mapping in (px4_quaternion_composed, px4_quaternion_atmo_legacy):
            self.assertAlmostEqual(
                float(np.linalg.norm(np.array(mapping(raw)))), 1.0, places=12
            )

    def test_the_two_mappings_are_not_the_same_rotation(self):
        # Recorded deliberately. relay_mocap.py mapped the RAW streamed
        # quaternion straight to NED as (w, -z, x, -y); composing
        # y_up -> z_up -> NED gives (w, x, z, -y). They disagree, so one of them
        # is wrong for any given Motive rigid-body definition. Which one is a
        # MEASUREMENT (Stage C), not a preference -- this test exists so nobody
        # "tidies up" the difference without taking that measurement.
        raw = np.array((0.6, 0.1, -0.2, 0.77))
        z_up = to_z_up(np.zeros(3), raw, "y_up")[1]
        composed = np.array(px4_quaternion_composed(z_up))
        legacy = np.array(px4_quaternion_atmo_legacy(raw))
        self.assertFalse(np.allclose(composed, legacy, atol=1e-9))

    def test_identity_stays_identity_in_both(self):
        for mapping in (px4_quaternion_composed, px4_quaternion_atmo_legacy):
            np.testing.assert_allclose(np.abs(np.array(mapping(IDENTITY))), IDENTITY)

    def test_legacy_position_matches_the_composed_position(self):
        # The POSITION halves of the two chains DO agree, which is why the
        # discrepancy above is specifically about the rotation.
        raw = np.array((1.0, 2.0, 3.0))
        composed = z_up_to_ned_position(to_z_up(raw, IDENTITY, "y_up")[0])
        np.testing.assert_allclose(px4_position_atmo_legacy(raw), composed)


if __name__ == "__main__":
    unittest.main()
