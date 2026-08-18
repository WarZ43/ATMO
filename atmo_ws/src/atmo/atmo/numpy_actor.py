"""Numpy-only inference for the rl_games actor, so the robot needs no PyTorch.

The trained actor is a four-layer MLP (obs -> 256 -> 128 -> 64 -> action) with
ELU activations, behind rl_games' RunningMeanStd and its +-5 clamp. For ATMO
that is 529 -> 256 -> 128 -> 64 -> 7, about 170k multiply-adds per step at
50 Hz -- roughly 8 MFLOP/s. Installing JetPack-pinned CUDA wheels, and
re-pinning them on every image change, to compute something numpy finishes in a
fraction of a millisecond is not a proportionate trade, and numpy is already a
dependency of this package.

Reading a .pth still needs torch, because the checkpoint is a pickle of torch
tensors. That stays a one-time job on a machine that has torch:
`scripts/export_policy_npz.py` does the conversion and verifies the numpy
forward pass against torch before it writes anything.

Ported from m4-direct-rl `src/m4_policy_runtime/m4_policy_runtime/policy.py`.
The semantics are deliberately identical: epsilon 1e-5, the +-5 clamp, ELU on
every layer but the output.
"""

from typing import Any, Optional  # noqa: F401  (used by the type comments)

import numpy as np

# rl_games' RunningMeanStd epsilon and the clamp the actor was trained behind.
NORMALIZER_EPSILON = 1.0e-5
NORMALIZER_CLAMP = 5.0
LAYER_COUNT = 4


class NumpyActor(object):
    """The exported actor, evaluated with numpy alone."""

    def __init__(self, path, observation_dim, action_dim):
        # type: (Any, int, int) -> None
        archive = np.load(str(path))
        expected = ["weight_%d" % index for index in range(LAYER_COUNT)]
        missing = [name for name in expected if name not in archive]
        if missing:
            raise RuntimeError("Policy archive is missing %s" % missing)
        self.weights = [
            np.asarray(archive["weight_%d" % index], dtype=np.float64)
            for index in range(LAYER_COUNT)
        ]
        self.biases = [
            np.asarray(archive["bias_%d" % index], dtype=np.float64)
            for index in range(LAYER_COUNT)
        ]
        self.observation_mean = np.asarray(
            archive["observation_mean"], dtype=np.float64
        ).reshape(-1)
        self.observation_variance = np.asarray(
            archive["observation_variance"], dtype=np.float64
        ).reshape(-1)
        self.observation_dim = observation_dim
        self.action_dim = action_dim

        # Provenance of the .pth this came from, so a stale archive is
        # identifiable rather than inferred from its filename.
        self.checkpoint_sha256 = _decode(archive, "checkpoint_sha256", "ascii")
        self.checkpoint_name = _decode(archive, "checkpoint_name", "utf-8")

        if self.weights[0].shape[1] != observation_dim:
            raise RuntimeError(
                "Policy archive expects %d observations, the contract packs %d"
                % (self.weights[0].shape[1], observation_dim)
            )
        if self.weights[-1].shape[0] != action_dim:
            raise RuntimeError(
                "Policy archive produces %d actions, the contract expects %d"
                % (self.weights[-1].shape[0], action_dim)
            )
        if self.observation_mean.size != observation_dim:
            raise RuntimeError("Normalizer mean does not match the observation width")
        if self.observation_variance.size != observation_dim:
            raise RuntimeError(
                "Normalizer variance does not match the observation width"
            )
        if np.any(self.observation_variance < 0.0):
            raise RuntimeError("Normalizer variance contains negative values")
        for array in self.weights + self.biases + [self.observation_mean]:
            if not np.all(np.isfinite(array)):
                raise RuntimeError("Policy archive contains non-finite values")

    def __call__(self, observation):
        # type: (np.ndarray) -> np.ndarray
        observation = np.asarray(observation)
        if observation.shape != (self.observation_dim,):
            raise ValueError(
                "Observation shape %s does not match the actor (%d,)"
                % (observation.shape, self.observation_dim)
            )
        value = (observation.astype(np.float64) - self.observation_mean) / np.sqrt(
            self.observation_variance + NORMALIZER_EPSILON
        )
        value = np.clip(value, -NORMALIZER_CLAMP, NORMALIZER_CLAMP)
        for index in range(LAYER_COUNT):
            value = self.weights[index] @ value + self.biases[index]
            if index < LAYER_COUNT - 1:
                # ELU, matching torch: x for x > 0, exp(x) - 1 otherwise.
                value = np.where(value > 0.0, value, np.expm1(np.minimum(value, 0.0)))
        action = value.astype(np.float32)
        if action.shape != (self.action_dim,):
            raise ValueError("Actor output shape does not match the contract")
        if not np.all(np.isfinite(action)):
            raise ValueError("Actor output contains non-finite values")
        return action

    def describe(self):
        # type: () -> str
        return "NumpyActor(obs=%d, act=%d, from=%s, sha256=%s)" % (
            self.observation_dim,
            self.action_dim,
            self.checkpoint_name,
            self.checkpoint_sha256[:12],
        )


def _decode(archive, key, encoding):
    # type: (Any, str, str) -> str
    if key not in archive:
        return "unknown"
    try:
        return bytes(archive[key]).decode(encoding)
    except Exception:
        return "unknown"


def is_numpy_archive(path):
    # type: (Any) -> bool
    """True when this path should be loaded by NumpyActor rather than torch."""
    return str(path).endswith(".npz")


def try_load_numpy_actor(path, observation_dim, action_dim):
    # type: (Any, int, int) -> Optional[NumpyActor]
    """Load a .npz actor, or return None if this is not a numpy archive."""
    if not is_numpy_archive(path):
        return None
    return NumpyActor(path, observation_dim, action_dim)
