"""The contract validator: it exists to fail loudly, so test that it does."""

import json
import os
import unittest

from atmo.policy_contract import (
    ContractError,
    DeploymentContract,
    find_contract,
)

CONTRACT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "atmo",
    "contracts",
    "atmo_combined_v1.json",
)


def load_data():
    with open(CONTRACT_PATH, "r") as stream:
        return json.load(stream)


class Cfg(object):
    """Stands in for CombinedStage1Config with only the checked attributes."""

    def __init__(self, **kwargs):
        self.action_dim = 7
        self.history_obs_dim = 19
        self.current_obs_dim = 64
        self.observation_history_length = 15
        self.action_history_length = 25
        self.task_observation_dim = 5
        self.observation_dim = 529
        self.forward_yaw_offset = 3.141592653589793
        self.wheel_turn_scale = 0.5
        for key, value in kwargs.items():
            setattr(self, key, value)


class TestShippedContract(unittest.TestCase):
    def test_the_shipped_contract_is_present_and_valid(self):
        self.assertTrue(os.path.isfile(CONTRACT_PATH), CONTRACT_PATH)
        contract = DeploymentContract.load(CONTRACT_PATH)
        self.assertEqual(contract.policy_name, "atmo_combined")
        self.assertEqual(contract.action_dim, 7)
        self.assertEqual(contract.observation_dim, 529)
        self.assertEqual(contract.derived_observation_dim(), 529)

    def test_find_contract_locates_the_package_copy(self):
        # Must resolve without ATMO_RL_CONTRACT set, because that is how it
        # will be found on the robot after colcon build.
        previous = os.environ.pop("ATMO_RL_CONTRACT", None)
        try:
            found = find_contract()
            self.assertIsNotNone(found)
            self.assertTrue(os.path.isfile(found))
        finally:
            if previous is not None:
                os.environ["ATMO_RL_CONTRACT"] = previous

    def test_forward_yaw_offset_is_pi_for_atmo(self):
        # ATMO's forward axis is body -x, so this is pi rather than the 0.0 the
        # M4TII contract carries. A runtime that assumes zero rotates every
        # heading-frame observation by 180 degrees.
        contract = DeploymentContract.load(CONTRACT_PATH)
        self.assertAlmostEqual(contract.forward_yaw_offset, 3.141592653589793)


class TestValidation(unittest.TestCase):
    def test_missing_key_is_named(self):
        data = load_data()
        del data["action_dim"]
        with self.assertRaises(ContractError) as caught:
            DeploymentContract(data)
        self.assertIn("action_dim", str(caught.exception))

    def test_observation_dim_must_equal_the_sum_of_its_parts(self):
        data = load_data()
        data["observation_dim"] = 528
        with self.assertRaises(ContractError) as caught:
            DeploymentContract(data)
        self.assertIn("528", str(caught.exception))
        self.assertIn("529", str(caught.exception))

    def test_delay_steps_must_be_the_whole_inclusive_set(self):
        # The endpoints-only bug: [0, 2] sizes the buffer correctly from the
        # maximum while rejecting delay 1 as invalid.
        data = load_data()
        data["observation_delay_steps"] = [0, 2]
        with self.assertRaises(ContractError) as caught:
            DeploymentContract(data)
        self.assertIn("inclusive", str(caught.exception))

    def test_delay_steps_must_be_sorted(self):
        data = load_data()
        data["observation_delay_steps"] = [1, 0]
        with self.assertRaises(ContractError):
            DeploymentContract(data)

    def test_term_sizes_must_agree_with_the_declared_widths(self):
        data = load_data()
        data["history_observation_terms"][0]["size"] = 4
        with self.assertRaises(ContractError) as caught:
            DeploymentContract(data)
        self.assertIn("History observation terms", str(caught.exception))

    def test_task_observation_names_must_match_their_width(self):
        data = load_data()
        data["task_observation"] = ["drive", "takeoff"]
        with self.assertRaises(ContractError):
            DeploymentContract(data)


class TestRuntimeCrossCheck(unittest.TestCase):
    def setUp(self):
        self.contract = DeploymentContract.load(CONTRACT_PATH)

    def test_matching_runtime_reports_nothing(self):
        self.assertEqual(self.contract.check_runtime(Cfg()), [])

    def test_task_observation_width_mismatch_is_reported(self):
        # This is the live disagreement between the exported ATMO_SPEC (5, the
        # mode one-hot plus the signed phase-event timer) and
        # rl_combined_runtime.CombinedStage1Config (4, the one-hot alone).
        cfg = Cfg(task_observation_dim=4, observation_dim=528)
        problems = self.contract.check_runtime(cfg)
        self.assertTrue(any("task_observation_dim" in p for p in problems))
        self.assertTrue(any("observation_dim" in p for p in problems))
        # The message must name the missing term, not just the number.
        self.assertTrue(any("phase_event_time_s" in p for p in problems))

    def test_forward_yaw_offset_mismatch_is_reported(self):
        problems = self.contract.check_runtime(Cfg(forward_yaw_offset=0.0))
        self.assertTrue(any("forward_yaw_offset" in p for p in problems))

    def test_wheel_turn_scale_mismatch_is_reported(self):
        problems = self.contract.check_runtime(Cfg(wheel_turn_scale=1.0))
        self.assertTrue(any("wheel_turn_scale" in p for p in problems))

    def test_delay_beyond_the_trained_maximum_is_reported(self):
        cfg = Cfg()
        cfg.observation_delay_max_steps = 5
        problems = self.contract.check_runtime(cfg)
        self.assertTrue(any("observation_delay_max_steps" in p for p in problems))

    def test_absent_runtime_attributes_are_skipped_not_failed(self):
        class Sparse(object):
            action_dim = 7

        self.assertEqual(self.contract.check_runtime(Sparse()), [])


if __name__ == "__main__":
    unittest.main()
