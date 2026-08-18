"""NumpyActor must match torch's semantics exactly, including the clamp.

The export script verifies against a real checkpoint on a machine with torch.
These tests cover what can be checked anywhere: the arithmetic, the guards, and
the provenance fields.
"""

import os
import shutil
import tempfile
import unittest

import numpy as np

from atmo.numpy_actor import (
    NORMALIZER_CLAMP,
    NumpyActor,
    is_numpy_archive,
    try_load_numpy_actor,
)

OBS = 12
ACT = 3
WIDTHS = (256, 128, 64)


def write_archive(path, obs=OBS, act=ACT, seed=0, **overrides):
    rng = np.random.RandomState(seed)
    sizes = [obs, WIDTHS[0], WIDTHS[1], WIDTHS[2], act]
    payload = {}
    for index in range(4):
        payload["weight_%d" % index] = rng.standard_normal(
            (sizes[index + 1], sizes[index])
        ) * 0.1
        payload["bias_%d" % index] = rng.standard_normal(sizes[index + 1]) * 0.1
    payload["observation_mean"] = rng.standard_normal(obs)
    payload["observation_variance"] = np.abs(rng.standard_normal(obs)) + 0.5
    payload["checkpoint_sha256"] = np.frombuffer(
        ("a" * 64).encode("ascii"), dtype=np.uint8
    )
    payload["checkpoint_name"] = np.frombuffer(
        b"unit_test.pth", dtype=np.uint8
    )
    payload.update(overrides)
    np.savez(path, **payload)
    return payload


def reference_forward(payload, observation):
    """The semantics NumpyActor must reproduce, written out independently."""
    value = (observation - payload["observation_mean"]) / np.sqrt(
        payload["observation_variance"] + 1e-5
    )
    value = np.clip(value, -5.0, 5.0)
    for index in range(4):
        value = payload["weight_%d" % index] @ value + payload["bias_%d" % index]
        if index < 3:
            value = np.where(value > 0.0, value, np.expm1(np.minimum(value, 0.0)))
    return value


class ArchiveCase(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.mkdtemp()
        self.path = os.path.join(self.directory, "policy.npz")

    def tearDown(self):
        shutil.rmtree(self.directory, ignore_errors=True)


class TestForwardPass(ArchiveCase):
    def test_matches_an_independent_implementation(self):
        payload = write_archive(self.path)
        actor = NumpyActor(self.path, OBS, ACT)
        rng = np.random.RandomState(7)
        for _ in range(32):
            observation = rng.standard_normal(OBS).astype(np.float32)
            np.testing.assert_allclose(
                actor(observation),
                reference_forward(payload, observation.astype(np.float64)),
                rtol=1e-6,
                atol=1e-6,
            )

    def test_the_clamp_is_applied(self):
        # Two observations far outside the clamp in the same direction must
        # produce the SAME action. Without the clamp they diverge.
        write_archive(self.path)
        actor = NumpyActor(self.path, OBS, ACT)
        far = np.full(OBS, 1.0e6, dtype=np.float32)
        farther = np.full(OBS, 1.0e9, dtype=np.float32)
        np.testing.assert_allclose(actor(far), actor(farther), rtol=0.0, atol=0.0)

    def test_output_is_float32(self):
        write_archive(self.path)
        actor = NumpyActor(self.path, OBS, ACT)
        self.assertEqual(actor(np.zeros(OBS, dtype=np.float32)).dtype, np.float32)

    def test_clamp_constant_matches_rl_games(self):
        self.assertEqual(NORMALIZER_CLAMP, 5.0)


class TestGuards(ArchiveCase):
    def test_wrong_observation_width_is_rejected_at_load(self):
        write_archive(self.path)
        with self.assertRaises(RuntimeError) as caught:
            NumpyActor(self.path, OBS + 1, ACT)
        self.assertIn("observations", str(caught.exception))

    def test_wrong_action_width_is_rejected_at_load(self):
        write_archive(self.path)
        with self.assertRaises(RuntimeError) as caught:
            NumpyActor(self.path, OBS, ACT + 1)
        self.assertIn("actions", str(caught.exception))

    def test_missing_layer_is_named(self):
        payload = write_archive(self.path)
        del payload["weight_2"]
        np.savez(self.path, **payload)
        with self.assertRaises(RuntimeError) as caught:
            NumpyActor(self.path, OBS, ACT)
        self.assertIn("weight_2", str(caught.exception))

    def test_negative_variance_is_rejected(self):
        write_archive(self.path, observation_variance=-np.ones(OBS))
        with self.assertRaises(RuntimeError) as caught:
            NumpyActor(self.path, OBS, ACT)
        self.assertIn("negative", str(caught.exception))

    def test_non_finite_weights_are_rejected(self):
        payload = write_archive(self.path)
        payload["weight_1"][0, 0] = np.nan
        np.savez(self.path, **payload)
        with self.assertRaises(RuntimeError) as caught:
            NumpyActor(self.path, OBS, ACT)
        self.assertIn("non-finite", str(caught.exception))

    def test_wrong_observation_shape_is_rejected_at_call(self):
        write_archive(self.path)
        actor = NumpyActor(self.path, OBS, ACT)
        with self.assertRaises(ValueError):
            actor(np.zeros(OBS + 3, dtype=np.float32))


class TestProvenance(ArchiveCase):
    def test_checkpoint_identity_is_carried(self):
        write_archive(self.path)
        actor = NumpyActor(self.path, OBS, ACT)
        self.assertEqual(actor.checkpoint_name, "unit_test.pth")
        self.assertEqual(actor.checkpoint_sha256, "a" * 64)
        self.assertIn("unit_test.pth", actor.describe())

    def test_missing_provenance_reads_as_unknown_not_a_crash(self):
        payload = write_archive(self.path)
        del payload["checkpoint_sha256"]
        del payload["checkpoint_name"]
        np.savez(self.path, **payload)
        actor = NumpyActor(self.path, OBS, ACT)
        self.assertEqual(actor.checkpoint_name, "unknown")


class TestSelection(ArchiveCase):
    def test_extension_selects_the_numpy_path(self):
        self.assertTrue(is_numpy_archive("/tmp/policy.npz"))
        self.assertFalse(is_numpy_archive("/tmp/policy.pth"))

    def test_try_load_declines_a_pth(self):
        self.assertIsNone(try_load_numpy_actor("/tmp/policy.pth", OBS, ACT))


if __name__ == "__main__":
    unittest.main()
