"""Load and validate the exported IsaacLab deployment contract.

The contract is the single written agreement between training and deployment.
It is generated on the training machine by
`M4/export_atmo_deployment_contract.py` from the live `ATMO_SPEC`, so a renamed
term or a changed width shows up here as a named failure instead of passing a
stale value through into a silently wrong observation.

Why this exists at all, from the m4-direct-rl bring-up: an observation packed
one element short of what the network expects is not a crash you can read. It
is a shape error at load if you are lucky, and a garbage flight if the widths
happen to agree for the wrong reason. Every dimension the runtime derives is
cross-checked here against the contract before the policy is allowed to run.

Two lessons are encoded as explicit checks:

* `observation_delay_steps` must enumerate the whole inclusive set, not just
  its endpoints. The runtime sizes its frame buffer from the maximum, and an
  endpoints-only list of `[0, 2]` reads as "delay 1 is invalid" while sizing
  correctly -- so the buffer is right and the validation is wrong, which is the
  hardest version of this bug to see.
* `observation_dim` must equal the sum of its parts. Deriving it twice and
  comparing is the only thing that catches a task-observation width drifting
  between the training spec and the runtime config.
"""

import json
from typing import Any, Dict, List, Optional  # noqa: F401  (type comments)

REQUIRED_KEYS = (
    "policy_name",
    "policy_hz",
    "action_dim",
    "history_obs_dim",
    "current_obs_dim",
    "task_observation_dim",
    "observation_history_length",
    "action_history_length",
    "observation_dim",
    "forward_yaw_offset",
)


class ContractError(RuntimeError):
    """Raised when the contract is malformed or disagrees with the runtime."""


class DeploymentContract(object):
    """The exported training contract, with its internal consistency checked."""

    def __init__(self, data):
        # type: (Dict[str, Any]) -> None
        missing = [key for key in REQUIRED_KEYS if key not in data]
        if missing:
            raise ContractError("Contract is missing required keys: %s" % missing)
        self.data = data
        self.policy_name = str(data["policy_name"])
        self.policy_hz = float(data["policy_hz"])
        self.publish_hz = float(data.get("publish_hz", data["policy_hz"]))
        self.action_dim = int(data["action_dim"])
        self.history_obs_dim = int(data["history_obs_dim"])
        self.current_obs_dim = int(data["current_obs_dim"])
        self.task_observation_dim = int(data["task_observation_dim"])
        self.observation_history_length = int(data["observation_history_length"])
        self.action_history_length = int(data["action_history_length"])
        self.observation_dim = int(data["observation_dim"])
        self.forward_yaw_offset = float(data["forward_yaw_offset"])
        wheel_command = data.get("wheel_command", {})
        self.wheel_turn_scale = (
            float(wheel_command["turn_scale"])
            if "turn_scale" in wheel_command
            else None
        )
        self.motor_tau_seconds = [
            float(value) for value in data.get("motor_tau_seconds", [])
        ]
        self.observation_delay_steps = [
            int(value) for value in data.get("observation_delay_steps", [0])
        ]
        self.checkpoint = data.get("checkpoint")
        self.checkpoint_sha256 = data.get("checkpoint_sha256")
        self._validate()

    # -- construction ----------------------------------------------------

    @classmethod
    def load(cls, path):
        # type: (Any) -> DeploymentContract
        with open(str(path), "r") as stream:
            return cls(json.load(stream))

    # -- validation ------------------------------------------------------

    def _validate(self):
        # type: () -> None
        for name in (
            "action_dim",
            "history_obs_dim",
            "current_obs_dim",
            "observation_history_length",
            "action_history_length",
            "observation_dim",
        ):
            if getattr(self, name) <= 0:
                raise ContractError("Contract %s must be positive" % name)
        if self.task_observation_dim < 0:
            raise ContractError("Contract task_observation_dim must not be negative")
        if self.policy_hz <= 0.0:
            raise ContractError("Contract policy_hz must be positive")

        derived = self.derived_observation_dim()
        if derived != self.observation_dim:
            raise ContractError(
                "Contract observation_dim is %d but its parts sum to %d "
                "(%d history frames x %d + %d action frames x %d + %d current "
                "+ %d task)"
                % (
                    self.observation_dim,
                    derived,
                    self.observation_history_length,
                    self.history_obs_dim,
                    self.action_history_length,
                    self.action_dim,
                    self.current_obs_dim,
                    self.task_observation_dim,
                )
            )

        self._validate_delay_steps()
        self._validate_terms()

    def _validate_delay_steps(self):
        # type: () -> None
        delays = self.observation_delay_steps
        if not delays:
            raise ContractError("Contract observation_delay_steps must not be empty")
        if sorted(delays) != delays:
            raise ContractError(
                "Contract observation_delay_steps must be sorted ascending: %s"
                % delays
            )
        if any(value < 0 for value in delays):
            raise ContractError("Contract observation_delay_steps must not be negative")
        expected = list(range(delays[0], delays[-1] + 1))
        if delays != expected:
            raise ContractError(
                "Contract observation_delay_steps %s is not the whole inclusive "
                "set %s. The runtime sizes its frame buffer from the maximum, so "
                "an endpoints-only list validates delays it should accept as "
                "invalid. Export the full set." % (delays, expected)
            )

    def _validate_terms(self):
        # type: () -> None
        history = self.data.get("history_observation_terms")
        if history is not None:
            total = sum(int(term["size"]) for term in history)
            if total != self.history_obs_dim:
                raise ContractError(
                    "History observation terms sum to %d, contract history_obs_dim "
                    "is %d" % (total, self.history_obs_dim)
                )
        current = self.data.get("current_observation_terms")
        if current is not None:
            total = sum(int(term["size"]) for term in current)
            if total != self.current_obs_dim:
                raise ContractError(
                    "Current observation terms sum to %d, contract current_obs_dim "
                    "is %d" % (total, self.current_obs_dim)
                )
        actions = self.data.get("action_terms")
        if actions is not None:
            total = sum(int(term["size"]) for term in actions)
            if total != self.action_dim:
                raise ContractError(
                    "Action terms sum to %d, contract action_dim is %d"
                    % (total, self.action_dim)
                )
        task = self.data.get("task_observation")
        if task is not None and len(task) != self.task_observation_dim:
            raise ContractError(
                "Contract lists %d task observation names but "
                "task_observation_dim is %d"
                % (len(task), self.task_observation_dim)
            )

    # -- derived ---------------------------------------------------------

    def derived_observation_dim(self):
        # type: () -> int
        return (
            self.observation_history_length * self.history_obs_dim
            + self.action_history_length * self.action_dim
            + self.current_obs_dim
            + self.task_observation_dim
        )

    def max_observation_delay_steps(self):
        # type: () -> int
        return self.observation_delay_steps[-1]

    # -- runtime cross-check ---------------------------------------------

    def check_runtime(self, cfg):
        # type: (Any) -> List[str]
        """Compare a runtime config against the contract.

        Returns the list of disagreements, most important first, rather than
        raising -- the caller decides whether a given mode may proceed. A
        shadow run with a mismatch is still informative; a policy run is not.
        """
        problems = []  # type: List[str]
        for name in (
            "action_dim",
            "history_obs_dim",
            "current_obs_dim",
            "observation_history_length",
            "action_history_length",
            "observation_dim",
        ):
            expected = getattr(self, name)
            actual = getattr(cfg, name, None)
            if actual is None:
                continue
            if int(actual) != int(expected):
                problems.append(
                    "%s: runtime %d, contract %d" % (name, int(actual), int(expected))
                )
        task = getattr(cfg, "task_observation_dim", None)
        if task is not None and int(task) != self.task_observation_dim:
            names = self.data.get("task_observation") or []
            problems.append(
                "task_observation_dim: runtime %d, contract %d%s"
                % (
                    int(task),
                    self.task_observation_dim,
                    (" (contract terms: %s)" % ", ".join(names)) if names else "",
                )
            )
        offset = getattr(cfg, "forward_yaw_offset", None)
        if offset is not None and abs(float(offset) - self.forward_yaw_offset) > 1e-6:
            problems.append(
                "forward_yaw_offset: runtime %.6f, contract %.6f"
                % (float(offset), self.forward_yaw_offset)
            )
        turn_scale = getattr(cfg, "wheel_turn_scale", None)
        if self.wheel_turn_scale is not None and turn_scale is not None:
            if abs(float(turn_scale) - self.wheel_turn_scale) > 1e-6:
                problems.append(
                    "wheel_turn_scale: runtime %.6f, contract %.6f"
                    % (float(turn_scale), self.wheel_turn_scale)
                )
        delay = getattr(cfg, "observation_delay_max_steps", None)
        if delay is not None and int(delay) > self.max_observation_delay_steps():
            problems.append(
                "observation_delay_max_steps: runtime %d exceeds the trained "
                "maximum %d" % (int(delay), self.max_observation_delay_steps())
            )
        return problems

    def describe(self):
        # type: () -> str
        return (
            "%s: obs=%d (%dx%d history + %dx%d actions + %d current + %d task), "
            "act=%d, %.1f Hz, forward_yaw_offset=%.4f"
            % (
                self.policy_name,
                self.observation_dim,
                self.observation_history_length,
                self.history_obs_dim,
                self.action_history_length,
                self.action_dim,
                self.current_obs_dim,
                self.task_observation_dim,
                self.action_dim,
                self.policy_hz,
                self.forward_yaw_offset,
            )
        )


DEFAULT_CONTRACT_NAME = "atmo_combined_v1.json"


def find_contract(explicit=None):
    # type: (Optional[str]) -> Optional[str]
    """Locate the contract: an explicit path, then ATMO_RL_CONTRACT, then the
    copy shipped inside this package.

    The package-local copy is what makes this work identically from a source
    tree and from an installed one -- after `colcon build` the module lives
    under install/, where a path relative to the workspace does not resolve.
    """
    import os

    if explicit:
        return explicit if os.path.isfile(explicit) else None
    from_env = os.getenv("ATMO_RL_CONTRACT")
    if from_env:
        return from_env if os.path.isfile(from_env) else None
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, "contracts", DEFAULT_CONTRACT_NAME)
    return candidate if os.path.isfile(candidate) else None
