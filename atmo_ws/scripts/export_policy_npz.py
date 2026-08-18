#!/usr/bin/env python3
"""Convert an rl_games .pth checkpoint into a numpy .npz the robot can run.

Run this WHERE TORCH ALREADY IS -- the training machine or any workstation with
PyTorch. The robot then needs numpy and nothing else.

The reason is proportion. The trained actor is a four-layer MLP, about 170k
multiply-adds per inference at 50 Hz for ATMO's 529 -> 7 network. Installing
PyTorch on a Jetson means JetPack-pinned CUDA wheels measured in gigabytes, a
version-matching exercise every time the image changes, and a large surface of
code between a bench session and its results. numpy computes the same
arithmetic in well under a millisecond and is already a dependency.

The conversion is VERIFIED, not assumed: after extracting the tensors this runs
both implementations over random observations and refuses to write the archive
unless they agree to tolerance. A silently wrong weight ordering produces a
policy that looks healthy and flies into the ground.

    python3 scripts/export_policy_npz.py checkpoint.pth policy.npz
    python3 scripts/export_policy_npz.py checkpoint.pth policy.npz --samples 512

Then copy the .npz to the robot and set ATMO_RL_POLICY_PATH to it; the runtime
picks the numpy actor from the file extension.
"""

import argparse
import hashlib
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(WORKSPACE, "src", "atmo"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", help="rl_games .pth input")
    parser.add_argument("output", help=".npz to write")
    parser.add_argument(
        "--contract",
        default=os.path.join(
            WORKSPACE, "src", "atmo", "atmo", "contracts", "atmo_combined_v1.json"
        ),
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=256,
        help="Random observations used to verify the two implementations agree.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1.0e-4,
        help="Largest permitted RELATIVE difference between torch and numpy "
        "(difference divided by max(|action|, 1)). float32 versus float64 "
        "accumulation lands around 1e-6; a structural error lands near 1.",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.checkpoint):
        parser.error("no such checkpoint: %s" % args.checkpoint)

    from atmo.numpy_actor import NumpyActor
    from atmo.policy_contract import DeploymentContract
    from atmo.rl_landing_stage1_runtime import PolicyRunner

    contract = DeploymentContract.load(args.contract)
    print("Contract: %s" % contract.describe())
    observation_dim = contract.observation_dim
    action_dim = contract.action_dim

    class _Cfg(object):
        pass

    cfg = _Cfg()
    cfg.policy_path = _PathLike(args.checkpoint)
    cfg.observation_dim = observation_dim
    cfg.action_dim = action_dim

    print("Loading %s (%d obs -> %d act)" % (args.checkpoint, observation_dim, action_dim))
    runner = PolicyRunner(cfg)
    if not runner.load():
        print("\nFAIL: %s" % runner.error, file=sys.stderr)
        return 1
    model = runner.model
    if model is None or not hasattr(model, "actor_mlp"):
        print(
            "\nFAIL: this checkpoint loaded as TorchScript, which has no named\n"
            "actor_mlp/mu layers to extract. Export from the rl_games state-dict\n"
            "checkpoint instead of a scripted module.",
            file=sys.stderr,
        )
        return 1
    if runner.obs_mean is None or runner.obs_var is None:
        print(
            "\nFAIL: no observation normalizer found in the checkpoint.\n"
            "The actor was trained with normalize_input, so running it without\n"
            "the normalizer produces garbage. Refusing to export a policy that\n"
            "would silently skip it.",
            file=sys.stderr,
        )
        return 1

    payload = {}
    layers = (model.actor_mlp[0], model.actor_mlp[2], model.actor_mlp[4], model.mu)
    for index, layer in enumerate(layers):
        payload["weight_%d" % index] = layer.weight.detach().cpu().numpy()
        payload["bias_%d" % index] = layer.bias.detach().cpu().numpy()
    payload["observation_mean"] = runner.obs_mean.detach().cpu().numpy().reshape(-1)
    payload["observation_variance"] = runner.obs_var.detach().cpu().numpy().reshape(-1)

    # Provenance, so a stale archive is identifiable rather than trusted by its
    # filename. The runtime logs this hash on load.
    with open(args.checkpoint, "rb") as stream:
        digest = hashlib.sha256(stream.read()).hexdigest()
    payload["checkpoint_sha256"] = np.frombuffer(digest.encode("ascii"), dtype=np.uint8)
    payload["checkpoint_name"] = np.frombuffer(
        os.path.basename(args.checkpoint).encode("utf-8"), dtype=np.uint8
    )
    print("  source sha256 %s..." % digest[:16])
    for name in sorted(payload):
        print("  %-22s %s" % (name, tuple(payload[name].shape)))

    # Write to a scratch path first so verification cannot be skipped by a
    # half-written file being picked up later. The name must END in .npz:
    # np.savez appends the extension otherwise, and the verifier then opens a
    # path that was never written.
    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    stem = os.path.splitext(os.path.basename(args.output))[0]
    scratch = os.path.join(output_dir, "%s.unverified.npz" % stem)
    np.savez(scratch, **payload)

    print("\nVerifying against torch over %d random observations ..." % args.samples)
    numpy_actor = NumpyActor(scratch, observation_dim, action_dim)
    rng = np.random.RandomState(0)
    worst = 0.0
    worst_relative = 0.0
    largest_action = 0.0
    for index in range(args.samples):
        # Cover both the linear region and the +-5 clamp: real observations
        # include normalized values well outside unit scale.
        scale = 1.0 if index % 2 else 20.0
        observation = (rng.standard_normal(observation_dim) * scale).astype(np.float32)
        reference = runner.action(observation)
        candidate = numpy_actor(observation)
        difference = np.abs(reference - candidate)
        magnitude = np.maximum(np.abs(reference), 1.0)
        worst = max(worst, float(np.max(difference)))
        worst_relative = max(worst_relative, float(np.max(difference / magnitude)))
        largest_action = max(largest_action, float(np.max(np.abs(reference))))
    print("  worst absolute difference: %.3e" % worst)
    print("  worst relative difference: %.3e" % worst_relative)
    print("  largest action seen:       %.3f" % largest_action)

    # Gate on the RELATIVE difference. torch runs this network in float32 and
    # NumpyActor accumulates in float64, so across a 529-wide dot product the
    # two disagree at float32 epsilon times the accumulation length -- order
    # 1e-5 absolute on outputs of order 10, which is not a defect and not
    # something a tighter absolute bound can distinguish from one. A real
    # structural error (wrong layer, transposed weight, wrong normalizer)
    # produces O(1) disagreement and fails this by orders of magnitude.
    if not np.isfinite(worst_relative) or worst_relative > args.tolerance:
        os.remove(scratch)
        print(
            "\nFAIL: numpy and torch disagree relatively by %.3e, over the %.1e "
            "tolerance. That is far above float32-versus-float64 noise, so "
            "suspect the layer selection rather than the arithmetic. Nothing "
            "was written." % (worst_relative, args.tolerance),
            file=sys.stderr,
        )
        return 1

    if os.path.exists(args.output):
        os.remove(args.output)
    os.rename(scratch, args.output)
    print("\nWrote %s (%.2f MB)" % (args.output, os.path.getsize(args.output) / 1e6))
    print("Source checkpoint sha256: %s" % digest)
    print(
        "numpy agrees with torch to %.1e relative (%.1e absolute); the residual "
        "is float32 accumulation in torch, and numpy is the more exact of the "
        "two." % (worst_relative, worst)
    )
    print("Copy it to the robot and set ATMO_RL_POLICY_PATH. The runtime logs this")
    print("hash on load; check it against the checkpoint before a session.")
    return 0


class _PathLike(str):
    """PolicyRunner calls cfg.policy_path.exists(); give a str that answers."""

    def exists(self):
        return os.path.isfile(str(self))


if __name__ == "__main__":
    raise SystemExit(main())
