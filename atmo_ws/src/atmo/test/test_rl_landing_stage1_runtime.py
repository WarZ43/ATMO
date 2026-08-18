import unittest
from collections import OrderedDict
import time
from unittest.mock import patch
from pathlib import Path
import tempfile

import numpy as np

try:
    import torch
except Exception:
    torch = None

from atmo.rl_landing_stage1_runtime import (
    LandingActionAdapter,
    LandingObservationBuilder,
    LandingStage1Config,
    PolicyRunner,
    _RlGamesActor,
)
from atmo.rl_combined_runtime import CombinedObservationBuilder, CombinedStage1Config


class LandingStage1ContractTest(unittest.TestCase):
    def make_config(self):
        return LandingStage1Config(
            randomize_reset=False,
            randomize_motor_dynamics=False,
            observation_noise=False,
        )

    def test_observation_contract_and_packing(self):
        cfg = self.make_config()
        self.assertEqual(cfg.action_dim, 7)
        self.assertEqual(cfg.history_obs_dim, 19)
        self.assertEqual(cfg.current_obs_dim, 64)
        self.assertEqual(cfg.observation_history_length, 15)
        self.assertEqual(cfg.action_history_length, 25)
        self.assertEqual(cfg.observation_dim, 524)

        observations = LandingObservationBuilder(cfg)
        observations.update_px4_state(
            position=(0.0, 0.0, -2.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        observations.set_tilt_angle(0.3)
        observation = observations.observation()
        self.assertEqual(observation.shape, (524,))
        self.assertTrue(np.isfinite(observation).all())
        np.testing.assert_allclose(observations.position, (0.0, 0.0, 2.0), atol=1e-6)
        self.assertAlmostEqual(float(observation[18]), 0.3, places=6)

        delayed_cfg = LandingStage1Config(
            randomize_reset=False,
            randomize_motor_dynamics=False,
            observation_noise=False,
            observation_delay_min_steps=1,
            observation_delay_max_steps=1,
        )
        delayed = LandingObservationBuilder(delayed_cfg)
        delayed.update_px4_state(
            position=(1.0, 0.0, -2.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        delayed.observation()
        delayed.update_px4_state(
            position=(2.0, 0.0, -2.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        delayed_observation = delayed.observation()
        # PX4 (1, 0, -2) becomes training-world (0, 1, 2), then the ATMO
        # forward-yaw offset of pi maps it into the policy heading frame.
        np.testing.assert_allclose(delayed_observation[:3], (-1.0, 0.0, 2.0), atol=1e-6)

    def test_heading_frame_rotates_vectors_and_strips_yaw(self):
        cfg = self.make_config()
        builder = LandingObservationBuilder(cfg)
        yaw = np.pi / 2.0
        builder.update_px4_state(
            position=(0.0, 0.0, -2.0),
            quat_wxyz=(np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(1.0, 0.0, 0.0),
        )
        history = builder._history_observation()
        # Vehicle yaw pi/2 plus ATMO's pi forward offset gives 3pi/2.
        np.testing.assert_allclose(history[0:3], (0.0, 0.0, 2.0), atol=1e-6)
        np.testing.assert_allclose(history[12:15], (0.0, 0.0, 0.0), atol=1e-6)
        np.testing.assert_allclose(history[15:18], (-1.0, 0.0, 0.0), atol=1e-6)

    def test_stage1_randomization_and_reference(self):
        def make_randomized_builder():
            cfg = LandingStage1Config(
                randomize_reset=True,
                randomize_motor_dynamics=True,
                observation_noise=False,
            )
            builder = LandingObservationBuilder(cfg)
            builder.update_px4_state(
                position=(0.0, 0.0, -2.0),
                quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                linear_velocity=(0.0, 0.0, 0.0),
                angular_velocity=(0.0, 0.0, 0.0),
            )
            return cfg, builder, LandingActionAdapter(cfg)

        cfg_a, builder_a, adapter_a = make_randomized_builder()
        cfg_b, builder_b, adapter_b = make_randomized_builder()
        self.assertTrue(np.all(builder_a.cfg.virtual_observation_offset[:2] <= 15.0))
        self.assertTrue(np.all(builder_a.cfg.virtual_observation_offset[:2] >= -15.0))
        self.assertGreaterEqual(float(builder_a.cfg.virtual_observation_offset[2]), 0.8)
        self.assertLessEqual(float(builder_a.cfg.virtual_observation_offset[2]), 1.2)
        self.assertIn(builder_a.observation_delay_steps, (0, 1))
        self.assertGreaterEqual(adapter_a.motor_tau, 0.125)
        self.assertLessEqual(adapter_a.motor_tau, 0.175)
        np.testing.assert_allclose(
            builder_a.cfg.virtual_observation_offset,
            builder_b.cfg.virtual_observation_offset,
            atol=1e-7,
        )
        self.assertEqual(builder_a.observation_delay_steps, builder_b.observation_delay_steps)
        self.assertAlmostEqual(adapter_a.motor_tau, adapter_b.motor_tau, places=7)

        builder_a.update_px4_state(
            position=(0.0, 0.0, -2.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        builder_a.reference_start_time -= builder_a.reference_accel_duration + 0.5 * builder_a.reference_cruise_duration
        _, cruise_velocity, cruise_accel = builder_a._reference_state()
        np.testing.assert_allclose(cruise_velocity, builder_a.reference_cruise_velocity, atol=1e-5)
        np.testing.assert_allclose(cruise_accel, np.zeros(3), atol=1e-6)

    def test_action_adapter_maps_physical_outputs(self):
        cfg = self.make_config()
        adapter = LandingActionAdapter(cfg)

        action = np.zeros(7, dtype=np.float32)
        action[1] = 0.1
        action[5:] = (0.1, 0.2)
        command = adapter.pre_physics_step(action)
        np.testing.assert_allclose(command.rotors_unfiltered, (0.55, 0.45, 0.45, 0.55), atol=1e-6)
        np.testing.assert_allclose(command.wheel_efforts, (-0.2, 0.0, 0.0, -0.2), atol=1e-6)

        action[5:] = (1.0, 1.0)
        saturated = adapter.pre_physics_step(action)
        self.assertTrue(np.all(np.abs(saturated.wheel_efforts) <= cfg.wheel_effort_limit))

    def test_tilt_history_keeps_continuous_action_after_physical_quantization(self):
        cfg = self.make_config()
        command = LandingActionAdapter(cfg).pre_physics_step(
            np.array((0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0), dtype=np.float32)
        )
        self.assertAlmostEqual(float(command.tilt_velocity), 0.0, places=6)
        self.assertAlmostEqual(float(command.semantic_action[4]), 0.3, places=6)

    def test_combined_fixed_vertical_routes_anchor_to_measured_state(self):
        for route, expected_mode in (("takeoff", 0), ("landing", 2)):
            with self.subTest(route=route), patch.dict(
                "os.environ",
                {"ATMO_RL_ROUTE": route, "ATMO_RL_DETERMINISTIC_TRAJECTORY": "1"},
            ):
                cfg = CombinedStage1Config(
                    randomize_reset=False,
                    randomize_motor_dynamics=False,
                    observation_noise=False,
                    observation_delay_min_steps=0,
                    observation_delay_max_steps=0,
                )
                builder = CombinedObservationBuilder(cfg)
                builder.update_px4_state(
                    position=(2.0, 3.0, -0.2 if route == "takeoff" else -1.2),
                    quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                    linear_velocity=(0.0, 0.0, 0.0),
                    angular_velocity=(0.0, 0.0, 0.0),
                )
                builder.reset_policy_context()
                builder.anchor_fixed_vertical_route()
                observation = builder.observation()

                # 529, not 528: the task observation is four phase one-hots
                # PLUS the signed phase-event timer. See test_phase_event_time.
                self.assertEqual(observation.shape, (529,))
                np.testing.assert_allclose(observation[-5:-1], np.eye(4, dtype=np.float32)[expected_mode])
                reference_position, reference_velocity, _ = builder._reference_state()
                np.testing.assert_allclose(reference_position, builder.position, atol=1e-6)
                np.testing.assert_allclose(reference_velocity, np.zeros(3), atol=1e-6)
                if route == "takeoff":
                    self.assertAlmostEqual(float(builder.takeoff_end[2] - builder.position[2]), 1.0)
                else:
                    self.assertAlmostEqual(float(builder.landing_position[2]), 0.20)

    def test_combined_physical_gates_match_training_modes(self):
        cfg = CombinedStage1Config(
            randomize_reset=False,
            randomize_motor_dynamics=False,
            observation_noise=False,
            observation_delay_min_steps=0,
            observation_delay_max_steps=0,
        )
        with patch.dict("os.environ", {"ATMO_RL_ROUTE": "takeoff"}):
            builder = CombinedObservationBuilder(cfg)
        builder.update_px4_state(
            position=(0.0, 0.0, -0.2),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
        builder.reset_policy_context()
        builder.anchor_fixed_vertical_route()

        self.assertEqual(builder.mode, 0)
        self.assertAlmostEqual(builder.rotor_thrust_gate(), 0.0)
        self.assertAlmostEqual(builder.wheel_speed_gate(), 1.0)

        builder.mode = 1
        builder.phase_elapsed_s = -cfg.takeoff_prep_duration_s
        builder.phase_wall_start_time = time.monotonic()
        builder.tilt_angle = cfg.takeoff_thrust_zero_tilt_rad
        self.assertAlmostEqual(builder.rotor_thrust_gate(), 0.0)
        self.assertAlmostEqual(builder.wheel_speed_gate(), 1.0)
        builder.tilt_angle = cfg.takeoff_thrust_full_tilt_rad
        self.assertAlmostEqual(builder.rotor_thrust_gate(), 1.0)
        builder.phase_elapsed_s = 0.1
        self.assertAlmostEqual(builder.wheel_speed_gate(), 0.0)

        builder.mode = 2
        self.assertAlmostEqual(builder.rotor_thrust_gate(), 1.0)
        self.assertAlmostEqual(builder.wheel_speed_gate(), 0.0)
        builder.mode = 3
        self.assertAlmostEqual(builder.rotor_thrust_gate(), 1.0)
        self.assertAlmostEqual(builder.wheel_speed_gate(), 1.0)

    @unittest.skipIf(torch is None, "PyTorch is not installed")
    def test_rl_games_loader_selects_actor_when_critic_keys_come_first(self):
        observation_dim = self.make_config().observation_dim
        expected = _RlGamesActor(observation_dim, 7).eval()
        checkpoint_model = OrderedDict()
        layer_shapes = (
            ("0", (256, observation_dim), (256,)),
            ("2", (128, 256), (128,)),
            ("4", (64, 128), (64,)),
        )
        for index, weight_shape, bias_shape in layer_shapes:
            checkpoint_model[f"a2c_network.critic_mlp.{index}.weight"] = torch.randn(*weight_shape)
            checkpoint_model[f"a2c_network.critic_mlp.{index}.bias"] = torch.randn(*bias_shape)
        checkpoint_model["a2c_network.critic.value.weight"] = torch.randn(1, 64)
        checkpoint_model["a2c_network.critic.value.bias"] = torch.randn(1)
        for key, value in expected.state_dict().items():
            checkpoint_model[f"a2c_network.{key}"] = value.detach().clone()

        checkpoint = {
            "model": checkpoint_model,
            "running_mean_std": {
                "running_mean": torch.zeros(observation_dim),
                "running_var": torch.ones(observation_dim),
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.pth"
            torch.save(checkpoint, path)
            runner = PolicyRunner(LandingStage1Config(policy_path=path))
            self.assertTrue(runner.load(), runner.error)
            sample = np.linspace(-0.5, 0.5, observation_dim, dtype=np.float32)
            expected_output = expected(torch.from_numpy(sample).unsqueeze(0)).detach().numpy()[0]
            np.testing.assert_allclose(runner.action(sample), expected_output, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
