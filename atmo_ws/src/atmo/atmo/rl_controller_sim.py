"""ROS 2 sim node for the ATMO combined-task RL policy."""

import math
import os
import time

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from custom_msgs.msg import TiltVel
from px4_msgs.msg import (
    ActuatorMotors,
    OffboardControlMode,
    TiltAngle,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleCommand,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)
from std_msgs.msg import Float32MultiArray
from atmo.rl_combined_runtime import (
    ActuatorCommand,
    CombinedObservationBuilder,
    CombinedStage1Config,
    LandingActionAdapter,
    PolicyRunner,
)


QUEUE_SIZE = int(os.getenv("ATMO_RL_QUEUE_SIZE", "10"))
ACTION_LABELS = ("lift", "roll", "pitch", "yaw", "tilt", "drive", "turn")
PX4_FORCE_ARM_MAGIC = 21196.0


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None or value == "":
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    if value is None or value == "":
        return default
    return value


def _normalize_mode(value: str) -> str:
    mode = value.strip().lower().replace("-", "_")
    if mode in {
        "zero_g_test",
        "zero_gravity_test",
        "zero_g",
        "zero_gravity",
        "action_test",
        "actuator_test",
        "test",
    }:
        return "zero_g_test"
    if mode in {"rl_test", "rl", "policy", "policy_test"}:
        return "rl_test"
    if mode in {"drive_test", "drive", "wheel_test", "wheels", "ground_drive_test"}:
        return "drive_test"
    return mode


def _normalize_tilt_control_mode(value: str) -> str:
    mode = value.strip().lower().replace("-", "_")
    if mode in {"direct", "actuator", "actuator_position", "position"}:
        return "direct"
    if mode in {
        "topic_velocity",
        "tilt_topic",
        "tilt_vel_topic",
        "tilt_angle_topic",
        "velocity",
        "tilt_vel",
        "tilt_velocity",
        "velocity_controller",
    }:
        return "topic_velocity"
    if mode in {"actuator_velocity", "actuator_vel", "direct_velocity"}:
        return "actuator_velocity"
    return "direct"


def _run_mode() -> str:
    mode = os.getenv("ATMO_RL_MODE")
    if mode:
        return _normalize_mode(mode)
    if _env_bool("ATMO_RL_TEST_MODE"):
        return "zero_g_test"
    return "rl_test"


def _fixed_action_from_env(mode: str):
    if mode not in {"zero_g_test", "drive_test"}:
        return None

    action = np.zeros(7, dtype=np.float32)
    if mode == "drive_test":
        forward = _env_float("ATMO_RL_TEST_FORWARD", _env_float("ATMO_RL_TEST_DRIVE", 0.0))
        turn = _env_float("ATMO_RL_TEST_LR", _env_float("ATMO_RL_TEST_TURN", 0.0))
        action[0] = -1.0
        action[5] = -forward
        action[6] = turn
        return np.clip(action, -1.0, 1.0)

    lift_force = np.clip(
        _env_float("ATMO_RL_TEST_LIFT", _env_float("ATMO_RL_TEST_THRUST", 0.0)),
        0.0,
        1.0,
    )
    action[0] = 2.0 * lift_force - 1.0
    action[1] = _env_float("ATMO_RL_TEST_ROLL", 0.0)
    action[2] = _env_float("ATMO_RL_TEST_PITCH", 0.0)
    action[3] = _env_float("ATMO_RL_TEST_YAW", 0.0)
    action[4] = _env_float("ATMO_RL_TEST_TILT", 0.0)
    action[5] = _env_float("ATMO_RL_TEST_DRIVE", 0.0)
    action[6] = _env_float("ATMO_RL_TEST_TURN", 0.0)
    return np.clip(action, -1.0, 1.0)


def _drive_test_tilt_angle() -> float:
    return float(
        np.clip(
            _env_float("ATMO_RL_TEST_TILT_ANGLE", _env_float("ATMO_RL_TEST_TILT_RAD", math.pi / 2.0)),
            0.0,
            math.pi / 2.0,
        )
    )


def _drive_test_tilt_tolerance() -> float:
    return max(_env_float("ATMO_RL_TEST_TILT_TOLERANCE", 0.02), 0.0)


class RLCombinedSim(Node):
    """Run the trained 50 Hz policy while keeping PX4 offboard alive at 143 Hz."""

    def __init__(self):
        super().__init__("rl_combined_sim")

        self.run_mode = _run_mode()
        if self.run_mode not in {"rl_test", "zero_g_test", "drive_test"}:
            self.get_logger().warn(
                f"Unknown ATMO_RL_MODE='{self.run_mode}', using rl_test"
            )
            self.run_mode = "rl_test"

        self.cfg = CombinedStage1Config()
        self.cfg.motor_tau_min_s = 0.0
        self.cfg.motor_tau_max_s = 0.0
        self.adapter = LandingActionAdapter(self.cfg)
        self.observations = CombinedObservationBuilder(self.cfg)
        self.policy = PolicyRunner(self.cfg)
        self.fixed_action = _fixed_action_from_env(self.run_mode)
        self.drive_test_tilt_angle = _drive_test_tilt_angle()
        self.drive_test_tilt_tolerance = _drive_test_tilt_tolerance()
        self.drive_test_commanded_tilt = None
        self.log_actions = _env_bool("ATMO_RL_LOG_ACTIONS")
        self.log_observations = _env_bool("ATMO_RL_LOG_OBS", _env_bool("ATMO_RL_LOG_OBSERVATIONS"))
        self.log_interval_s = max(_env_float("ATMO_RL_LOG_INTERVAL_S", 5.0), 0.02)
        self.rotor_command_model = "linear_force_to_gazebo_speed"
        self.gazebo_rotor_max_velocity = _env_float(
            "ATMO_RL_GAZEBO_ROTOR_MAX_VELOCITY", 1401.0
        )
        self.gazebo_rotor_zero_position_armed = _env_float(
            "ATMO_RL_GAZEBO_ROTOR_ZERO_POSITION_ARMED", 210.0
        )
        self.gazebo_rotor_input_scaling = _env_float(
            "ATMO_RL_GAZEBO_ROTOR_INPUT_SCALING", 1401.0
        )
        self.gazebo_rotor_thrust_scale = _env_float(
            "ATMO_RL_SIM_ROTOR_THRUST_SCALE", 1.0
        )
        self.tilt_actuator_rate = _env_float(
            "ATMO_RL_SIM_TILT_ACTUATOR_RATE", math.pi / 8.0
        )
        self.tilt_control_mode = _normalize_tilt_control_mode(
            _env_str("ATMO_RL_SIM_TILT_CONTROL_MODE", "direct")
        )
        self.use_direct_tilt_actuator = self.tilt_control_mode == "direct"
        self.use_actuator_tilt_velocity = self.tilt_control_mode == "actuator_velocity"
        self.use_topic_tilt_velocity = self.tilt_control_mode == "topic_velocity"
        self.tilt_actuator_upper = _env_float(
            "ATMO_RL_SIM_TILT_ACTUATOR_UPPER", math.radians(85.0)
        )
        self.cfg.tilt_upper = float(
            np.clip(
                min(float(self.cfg.tilt_upper), float(self.tilt_actuator_upper)),
                self.cfg.tilt_lower,
                math.pi / 2.0,
            )
        )
        self.tilt_actuator_upper = self.cfg.tilt_upper
        self.drive_test_tilt_angle = float(
            np.clip(self.drive_test_tilt_angle, self.cfg.tilt_lower, self.cfg.tilt_upper)
        )
        self.publish_tilt_vel = self.use_topic_tilt_velocity
        self.initial_tilt_angle = float(
            np.clip(
                _env_float("ATMO_RL_INITIAL_TILT_ANGLE", 0.0),
                self.cfg.tilt_lower,
                self.cfg.tilt_upper,
            )
        )
        self.adapter.set_tilt_angle(self.initial_tilt_angle)
        self.observations.set_tilt_angle(self.initial_tilt_angle)
        self.actuator_gap_warn_s = max(_env_float("ATMO_RL_ACTUATOR_GAP_WARN_S", 0.10), 0.0)
        self.tilt_jump_warn_rad = max(_env_float("ATMO_RL_TILT_JUMP_WARN_RAD", 0.05), 0.0)
        self.policy_loaded = self.fixed_action is not None or self.policy.load()
        self.policy_start_gate_file = os.getenv("ATMO_RL_START_GATE_FILE", "").strip()

        self.get_logger().info(f"ATMO RL sim mode: {self.run_mode}")
        if self.fixed_action is not None:
            if self.run_mode == "drive_test":
                self.get_logger().warn(
                    "ATMO_RL drive test mode is active: "
                    f"forward={-float(self.fixed_action[5]):+.3f}, "
                    f"lr={float(self.fixed_action[6]):+.3f}, "
                    f"tilt_angle={self.drive_test_tilt_angle:.3f}, "
                    f"policy_action={np.array2string(self.fixed_action, precision=3, suppress_small=True)}"
                )
            else:
                self.get_logger().warn(
                    "ATMO_RL zero-gravity action test mode is active: "
                    f"policy_action={np.array2string(self.fixed_action, precision=3, suppress_small=True)}"
                )
        elif self.policy_loaded:
            self.get_logger().info(f"Loaded RL policy from {self.cfg.policy_path}")
            self.get_logger().info("RL action contract uses clipped rl_games outputs")
            self.get_logger().info("ATMO_RL rl_test full action output is active")
        else:
            self.get_logger().warn(
                f"RL policy is not active: {self.policy.error}. "
                "Drop the .pth at that path or set ATMO_RL_POLICY_PATH."
            )
        self.get_logger().info(
            "ATMO RL action contract: physical rotor mix, wheel effort channels, "
            f"obs_dim={self.cfg.observation_dim}, action_dim={self.cfg.action_dim}"
        )
        self.get_logger().info(
            "ATMO RL rotor command model: "
            f"model={self.rotor_command_model}, "
            f"motor_alpha={self.adapter.motor_alpha:.3f}, "
            f"sim_thrust_scale={self.gazebo_rotor_thrust_scale:.3f}, "
            f"max_rot_velocity={self.gazebo_rotor_max_velocity:.1f}, "
            f"input_scaling={self.gazebo_rotor_input_scaling:.1f}, "
            f"zero_position_armed={self.gazebo_rotor_zero_position_armed:.1f}"
        )
        self.get_logger().info(
            "ATMO RL morph tilt control: "
            f"mode={self.tilt_control_mode}, "
            f"direct_position_channels={self.use_direct_tilt_actuator}, "
            f"actuator_velocity_channels={self.use_actuator_tilt_velocity}, "
            f"topic_velocity_position_channels={self.use_topic_tilt_velocity}, "
            f"actuator_slew={self.tilt_actuator_rate:.3f} rad/s, "
            f"actuator upper={math.degrees(self.tilt_actuator_upper):.1f} deg; "
            f"publish_tilt_vel={self.publish_tilt_vel}; "
            f"initial_tilt={self.initial_tilt_angle:.3f}; "
            f"gap_warn_s={self.actuator_gap_warn_s:.3f}; "
            f"tilt_jump_warn_rad={self.tilt_jump_warn_rad:.3f}"
        )
        if self.policy_start_gate_file:
            self.get_logger().info(
                f"ATMO RL policy start is gated by {self.policy_start_gate_file}"
            )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QUEUE_SIZE
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QUEUE_SIZE
        )
        self.actuator_motors_publisher = self.create_publisher(
            ActuatorMotors, "/fmu/in/actuator_motors", QUEUE_SIZE
        )
        self.motor_speed_publisher = self.create_publisher(
            Float32MultiArray, "/motor_speed", QUEUE_SIZE
        )
        self.tilt_vel_publisher = self.create_publisher(TiltVel, "/tilt_vel", QUEUE_SIZE)

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_groundtruth",
            self.vehicle_local_position_groundtruth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude_groundtruth",
            self.vehicle_attitude_groundtruth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleAngularVelocity,
            "/fmu/out/vehicle_angular_velocity_groundtruth",
            self.vehicle_angular_velocity_groundtruth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TiltAngle,
            "/fmu/in/tilt_angle",
            self.tilt_angle_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.vehicle_command_ack_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleControlMode,
            "/fmu/out/vehicle_control_mode",
            self.vehicle_control_mode_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_sensor_data,
        )

        self.position = np.zeros(3, dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.angular_velocity = np.zeros(3, dtype=np.float32)
        self.measured_tilt_angle = self.initial_tilt_angle
        self.actuator_tilt_angle = self.initial_tilt_angle
        self.offboard = False
        self.px4_armed = False
        self.px4_offboard = False
        self.px4_nav_state = None
        self.policy_started = self.run_mode != "rl_test"
        self.offboard_setpoint_counter = 0
        self.last_policy_time = 0.0
        self.last_mode_request_time = 0.0
        self.last_state_log_time = 0.0
        self.last_warn_time = 0.0
        self.last_policy_duration = 0.0
        self.policy_tick_count = 0
        self.last_action_log_time = 0.0
        self.last_observation_log_time = 0.0
        self.last_actuator_publish_time = 0.0
        self.last_tilt_target = None
        self.last_tilt_normalized = None
        self.last_publisher_warn_time = 0.0

        self.timer = self.create_timer(self.cfg.publish_dt, self.timer_callback)

    def timer_callback(self):
        if not self.policy_loaded:
            self._retry_policy_load()
            return
        if not self.observations.state_seen:
            self._warn_waiting_for_state()
            return

        self._warn_competing_publishers_if_due()
        self._publish_offboard_control_mode_direct_actuator()
        if self.run_mode == "rl_test" and not self._px4_ready_for_policy():
            idle_command = self._idle_direct_actuator_command()
            self._publish_actuator_motors(idle_command)
            self._publish_tilt_vel_if_enabled(idle_command.tilt_velocity)
            self._request_offboard_and_arm()
            self._log_px4_state_until_ready()
            return

        if self.run_mode == "rl_test" and not self._policy_start_gate_open():
            idle_command = self._idle_direct_actuator_command()
            self._publish_actuator_motors(idle_command)
            self._publish_tilt_vel_if_enabled(idle_command.tilt_velocity)
            self._request_offboard_and_arm()
            self._log_policy_start_gate_until_ready()
            return

        if self.run_mode == "rl_test" and not self.policy_started:
            self._start_policy_after_px4_ready()

        self._update_policy_if_due()
        self._publish_actuator_motors(self.adapter.last_command)
        self._publish_tilt_vel_if_enabled(self.adapter.last_command.tilt_velocity)
        self._request_offboard_and_arm()
        self._log_px4_state_until_ready()

    def _px4_ready_for_policy(self):
        return self.px4_armed and self.px4_offboard

    def _policy_start_gate_open(self):
        if not self.policy_start_gate_file:
            return True
        return os.path.exists(self.policy_start_gate_file)

    def _idle_direct_actuator_command(self):
        semantic_action = np.zeros(7, dtype=np.float32)
        semantic_action[0] = float(np.clip(self.cfg.neutral_rotor, 0.0, 1.0))
        raw_action = np.zeros(7, dtype=np.float32)
        raw_action[0] = 2.0 * semantic_action[0] - 1.0
        return ActuatorCommand(
            rotors=np.zeros(4, dtype=np.float32),
            rotors_unfiltered=np.zeros(4, dtype=np.float32),
            tilt_angle=float(
                np.clip(
                    self.actuator_tilt_angle,
                    self.cfg.tilt_lower,
                    self.cfg.tilt_upper,
                )
            ),
            tilt_velocity=0.0,
            wheel_efforts=np.zeros(4, dtype=np.float32),
            drive_speed=0.0,
            turn_speed=0.0,
            semantic_action=semantic_action,
            raw_action=raw_action,
        )

    def _start_policy_after_px4_ready(self):
        self.policy_started = True
        self.adapter = LandingActionAdapter(self.cfg)
        self.adapter.set_tilt_angle(self.initial_tilt_angle)
        self.observations.reset_policy_context()
        self._set_observation_tilt()
        self.last_policy_time = 0.0
        self.last_policy_duration = 0.0
        self.policy_tick_count = 0
        self.last_action_log_time = 0.0
        self.last_observation_log_time = 0.0
        self.get_logger().info(
            "PX4 is armed/offboard; starting RL policy with fresh reference and neutral action history"
        )
        self.get_logger().info(
            "Combined trajectory: "
            f"route={self.observations.route_name}, vertical={self.observations.vertical_trajectory}, "
            f"observation_delay_steps={self.observations.observation_delay_steps}, "
            f"mode={self.observations.last_debug.get('combined_mode', 'reset')}, "
            f"durations=[drive={self.observations.drive_duration:.3f}, "
            f"takeoff={self.observations.takeoff_duration:.3f}, "
            f"flight={self.observations.flight_duration:.3f}, "
            f"landing={self.observations.landing_duration:.3f}], "
            f"spawn={np.array2string(self.observations.spawn, precision=3)}, "
            f"liftoff={np.array2string(self.observations.liftoff, precision=3)}, "
            f"takeoff_end={np.array2string(self.observations.takeoff_end, precision=3)}, "
            f"landing_start={np.array2string(self.observations.landing_start, precision=3)}, "
            f"landing_end={np.array2string(self.observations.landing_position, precision=3)}"
        )

    def _update_policy_if_due(self):
        now = time.monotonic()
        if now - self.last_policy_time < self.cfg.policy_dt:
            return

        self.adapter.set_tilt_angle(self.actuator_tilt_angle)
        self._set_observation_tilt()
        self.policy_tick_count += 1
        obs = self.observations.observation()
        transition = self.observations.last_debug.get("combined_transition", "none")
        if transition != "none":
            self.get_logger().info(
                f"Combined mode transition: {transition}, "
                f"mode={self.observations.last_debug['combined_mode']}, "
                f"phase_s={self.observations.last_debug['combined_phase_time']:.3f}"
            )
        self._maybe_log_observation(obs)
        self.last_policy_time = now
        if self.fixed_action is not None:
            command = self.adapter.pre_physics_step(self.fixed_action)
            if self.run_mode == "drive_test":
                command = self._drive_test_command(command)
            self._set_observation_tilt()
            self.observations.append_action(command.semantic_action)
            self._maybe_log_action(self.fixed_action, command)
            return

        start = time.perf_counter()
        action = self.policy.action(obs)
        self.last_policy_duration = time.perf_counter() - start

        if action is None:
            return
        command = self.adapter.pre_physics_step(action)
        self._set_observation_tilt()
        self.observations.append_action(command.semantic_action)
        self._maybe_log_action(action, command)

    def _drive_test_command(self, command):
        measured_tilt = float(
            np.clip(self.actuator_tilt_angle, self.cfg.tilt_lower, self.cfg.tilt_upper)
        )
        target_tilt = self.drive_test_tilt_angle
        if self.drive_test_commanded_tilt is None:
            self.drive_test_commanded_tilt = measured_tilt

        step = self.cfg.max_tilt_velocity * self.cfg.policy_dt
        command_error = target_tilt - self.drive_test_commanded_tilt
        if abs(command_error) <= max(step, self.drive_test_tilt_tolerance):
            commanded_tilt = target_tilt
        else:
            commanded_tilt = self.drive_test_commanded_tilt + math.copysign(step, command_error)
        commanded_tilt = float(np.clip(commanded_tilt, self.cfg.tilt_lower, self.cfg.tilt_upper))
        self.drive_test_commanded_tilt = commanded_tilt

        measured_error = target_tilt - measured_tilt
        tilt_velocity = 0.0
        if abs(measured_error) > self.drive_test_tilt_tolerance:
            tilt_velocity = math.copysign(self.cfg.max_tilt_velocity, measured_error)

        semantic_action = np.asarray(command.semantic_action, dtype=np.float32).copy()
        semantic_action[4] = float(np.clip(tilt_velocity / max(self.cfg.max_tilt_velocity, 1e-6), -1.0, 1.0))
        fixed = ActuatorCommand(
            rotors=np.asarray(command.rotors, dtype=np.float32).copy(),
            rotors_unfiltered=np.asarray(command.rotors_unfiltered, dtype=np.float32).copy(),
            tilt_angle=commanded_tilt,
            tilt_velocity=tilt_velocity,
            wheel_efforts=np.asarray(command.wheel_efforts, dtype=np.float32).copy(),
            drive_speed=command.drive_speed,
            turn_speed=command.turn_speed,
            semantic_action=semantic_action,
            raw_action=np.asarray(command.raw_action, dtype=np.float32).copy(),
        )
        self.adapter.set_tilt_angle(commanded_tilt)
        self.adapter.last_command = fixed
        return fixed

    def _maybe_log_action(self, action, command):
        if not self.log_actions:
            return

        now = time.monotonic()
        if now - self.last_action_log_time < self.log_interval_s:
            return
        self.last_action_log_time = now

        log_stamp = self._log_stamp()
        network_raw = "neutral" if action is None else np.array2string(
            np.asarray(action, dtype=np.float32), precision=3, suppress_small=True
        )
        action_values = (
            command.raw_action if action is None else np.asarray(action, dtype=np.float32)
        )
        action_space = ", ".join(
            f"{label}={float(value):+.3f}"
            for label, value in zip(ACTION_LABELS, action_values)
        )
        semantic_space = ", ".join(
            f"{label}={float(value):+.3f}"
            for label, value in zip(ACTION_LABELS, command.semantic_action)
        )
        rotor_actuators = self._rotor_actuator_controls(command)
        rotor_motor_speeds = self._rotor_motor_speeds(command)
        self.get_logger().info(
            "RL policy output sample: "
            f"{log_stamp}, tick={self.policy_tick_count}, "
            f"policy_duration_ms={1000.0 * self.last_policy_duration:.2f}, "
            f"policy_action=[{action_space}], "
            f"network_raw={network_raw}, "
            f"adapter_raw={np.array2string(command.raw_action, precision=3, suppress_small=True)}, "
            f"semantic_cmd=[{semantic_space}], "
            f"rotor_force_pre_ema={np.array2string(command.rotors_unfiltered, precision=3, suppress_small=True)}, "
            f"rotor_force_cmd={np.array2string(command.rotors, precision=3, suppress_small=True)}, "
            f"motor_alpha={self.adapter.motor_alpha:.3f}, "
            f"rotor_px4_cmd={np.array2string(rotor_actuators, precision=3, suppress_small=True)}, "
            f"rotor_motor_speed_cmd={np.array2string(rotor_motor_speeds, precision=1, suppress_small=True)}, "
            f"tilt_cmd={command.tilt_angle:.3f}, "
            f"actuator_tilt={self.actuator_tilt_angle:.3f}, "
            f"tilt_vel_rad_s={command.tilt_velocity:.3f}, "
            f"tilt_velocity_actuator_cmd={self._tilt_velocity_control(command.tilt_velocity):+.3f}, "
            f"drive_speed={command.drive_speed:+.3f}, "
            f"turn_speed={command.turn_speed:+.3f}, "
            f"wheel_efforts={np.array2string(command.wheel_efforts, precision=3, suppress_small=True)}"
        )

    def _maybe_log_observation(self, obs):
        if not self.log_observations:
            return

        now = time.monotonic()
        if now - self.last_observation_log_time < self.log_interval_s:
            return
        self.last_observation_log_time = now

        log_stamp = self._log_stamp()
        debug = self.observations.last_debug

        def arr(name):
            value = debug.get(name)
            if value is None:
                return "n/a"
            return np.array2string(np.asarray(value, dtype=np.float32), precision=3, suppress_small=True)

        obs_head = np.array2string(obs[:24], precision=3, suppress_small=True)
        obs_tail = np.array2string(obs[-24:], precision=3, suppress_small=True)
        action_history_head = np.array2string(
            self.observations.action_history[0], precision=3, suppress_small=True
        )
        self.get_logger().info(
            "RL observation sample: "
            f"{log_stamp}, tick={self.policy_tick_count}, "
            f"obs_dim={obs.shape[0]}, obs_head={obs_head}, obs_tail={obs_tail}, "
            f"route={debug.get('combined_route', 'n/a')}, "
            f"mode={debug.get('combined_mode', 'n/a')}, "
            f"phase_s={float(debug.get('combined_phase_time', 0.0)):.3f}, "
            f"transition={debug.get('combined_transition', 'none')}, "
            f"action_history_newest={action_history_head}, "
            f"pos={arr('position')}, vel={arr('linear_velocity')}, "
            f"ang_vel={arr('angular_velocity_w')}, tilt={float(debug.get('tilt_angle', 0.0)):.3f}, "
            f"ref_err={arr('reference_pos_error')}, "
            f"ref_vel={arr('reference_velocity')}, vel_err={arr('reference_velocity_error')}, "
            f"ref_accel={arr('reference_accel')}, "
            f"yaw_error={arr('yaw_error')}, "
            f"wrench=[{float(debug.get('wrench_min', 0.0)):.3f},"
            f"{float(debug.get('wrench_max', 0.0)):.3f}], "
            f"allocation=[{float(debug.get('allocation_min', 0.0)):.3f},"
            f"{float(debug.get('allocation_max', 0.0)):.3f}], "
            f"obs_norm={float(debug.get('obs_norm', float(np.linalg.norm(obs)))):.3f}, "
            f"obs_range=[{float(debug.get('obs_min', float(np.min(obs)))):.3f},"
            f"{float(debug.get('obs_max', float(np.max(obs)))):.3f}], "
            f"t_ref={float(debug.get('reference_time', 0.0)):.3f}"
        )

    def _request_offboard_and_arm(self):
        if self.px4_armed and self.px4_offboard:
            self.offboard = True
            return

        if self.offboard_setpoint_counter < 20:
            self.offboard_setpoint_counter += 1
            return

        now = time.monotonic()
        if now - self.last_mode_request_time < 1.0:
            return
        self.last_mode_request_time = now

        if not self.px4_offboard:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if not self.px4_armed:
            self._arm()

    def _log_px4_state_until_ready(self):
        if self.px4_armed and self.px4_offboard:
            if not self.offboard:
                self.get_logger().info("PX4 confirmed armed offboard/direct-actuator mode")
            self.offboard = True
            return

        now = time.monotonic()
        if now - self.last_state_log_time > 2.0:
            self.last_state_log_time = now
            self.get_logger().warn(
                "Waiting for PX4 to accept RL offboard control: "
                f"armed={self.px4_armed}, offboard={self.px4_offboard}, "
                f"nav_state={self.px4_nav_state}"
            )

    def _log_policy_start_gate_until_ready(self):
        now = time.monotonic()
        if now - self.last_state_log_time > 2.0:
            self.last_state_log_time = now
            self.get_logger().warn(
                f"PX4 ready; holding RL policy until gate exists: {self.policy_start_gate_file}"
            )

    def _warn_competing_publishers_if_due(self):
        now = time.monotonic()
        if now - self.last_publisher_warn_time < 5.0:
            return
        self.last_publisher_warn_time = now

        actuator_publishers = self.count_publishers("/fmu/in/actuator_motors")
        tilt_angle_publishers = self.count_publishers("/fmu/in/tilt_angle")
        tilt_topic_problem = (
            tilt_angle_publishers > 0
            if not self.use_topic_tilt_velocity
            else tilt_angle_publishers != 1
        )
        if actuator_publishers > 1 or tilt_topic_problem:
            self.get_logger().warn(
                "RL sim topic contention check: "
                f"{self._log_stamp()}, "
                f"/fmu/in/actuator_motors_publishers={actuator_publishers}, "
                f"/fmu/in/tilt_angle_publishers={tilt_angle_publishers}, "
                f"tilt_control_mode={self.tilt_control_mode}. "
                "Only rl_controller_sim should publish actuator_motors during RL; "
                "topic_velocity tilt mode expects exactly one tilt_angle publisher."
            )

    def _publish_actuator_motors(self, command):
        msg = ActuatorMotors()
        for idx in range(12):
            msg.control[idx] = float("nan")

        motor_speeds = self._rotor_motor_speeds(command)
        rotor_controls = self._rotor_actuator_controls(command)
        for idx in range(4):
            msg.control[idx] = float(rotor_controls[idx])

        if self.use_direct_tilt_actuator:
            tilt_angle = self._slew_actuator_tilt(command.tilt_angle)
            tilt_normalized = float(np.clip(tilt_angle / (math.pi / 2.0), 0.0, 1.0))
            self._maybe_log_tilt_actuator_edge(command.tilt_angle, tilt_angle, tilt_normalized)
            msg.control[4] = tilt_normalized
            msg.control[5] = tilt_normalized
        elif self.use_topic_tilt_velocity:
            tilt_angle = float(
                np.clip(self.actuator_tilt_angle, self.cfg.tilt_lower, self.cfg.tilt_upper)
            )
            tilt_normalized = float(np.clip(tilt_angle / (math.pi / 2.0), 0.0, 1.0))
            self._maybe_log_tilt_actuator_edge(command.tilt_angle, tilt_angle, tilt_normalized)
            msg.control[4] = tilt_normalized
            msg.control[5] = tilt_normalized
        elif self.use_actuator_tilt_velocity:
            self.actuator_tilt_angle = float(
                np.clip(command.tilt_angle, self.cfg.tilt_lower, self.cfg.tilt_upper)
            )
            tilt_velocity = self._limited_tilt_velocity(command.tilt_velocity)
            tilt_velocity_normalized = self._tilt_velocity_control(tilt_velocity)
            msg.control[4] = tilt_velocity_normalized
            msg.control[5] = tilt_velocity_normalized

        for idx in range(4):
            msg.control[6 + idx] = float(command.wheel_efforts[idx])

        msg.timestamp = self._timestamp_us()
        self.actuator_motors_publisher.publish(msg)
        self._publish_motor_speeds(motor_speeds)

    def _maybe_log_tilt_actuator_edge(self, target_tilt, actuator_tilt, tilt_normalized):
        now = time.monotonic()
        dt_s = 0.0 if self.last_actuator_publish_time <= 0.0 else now - self.last_actuator_publish_time
        target_delta = (
            0.0
            if self.last_tilt_target is None
            else float(target_tilt) - float(self.last_tilt_target)
        )
        normalized_delta = (
            0.0
            if self.last_tilt_normalized is None
            else float(tilt_normalized) - float(self.last_tilt_normalized)
        )

        gap_warn = (
            self.actuator_gap_warn_s > 0.0
            and self.last_actuator_publish_time > 0.0
            and dt_s > self.actuator_gap_warn_s
        )
        target_jump_warn = (
            self.tilt_jump_warn_rad > 0.0
            and self.last_tilt_target is not None
            and abs(target_delta) > self.tilt_jump_warn_rad
        )
        actuator_jump_warn = (
            self.tilt_jump_warn_rad > 0.0
            and self.last_tilt_normalized is not None
            and abs(normalized_delta) * (math.pi / 2.0) > self.tilt_jump_warn_rad
        )

        if gap_warn or target_jump_warn or actuator_jump_warn:
            self.get_logger().warn(
                "RL tilt actuator edge: "
                f"{self._log_stamp()}, tick={self.policy_tick_count}, "
                f"gap_s={dt_s:.3f}, target_tilt={float(target_tilt):.3f}, "
                f"actuator_tilt={float(actuator_tilt):.3f}, "
                f"tilt_norm={float(tilt_normalized):.3f}, "
                f"target_delta={target_delta:+.3f}, "
                f"norm_delta={normalized_delta:+.3f}, "
                f"gap_warn={gap_warn}, target_jump_warn={target_jump_warn}, "
                f"actuator_jump_warn={actuator_jump_warn}"
            )

        self.last_actuator_publish_time = now
        self.last_tilt_target = float(target_tilt)
        self.last_tilt_normalized = float(tilt_normalized)

    def _slew_actuator_tilt(self, target_tilt):
        actuator_upper = min(float(self.tilt_actuator_upper), float(self.cfg.tilt_upper))
        target = float(np.clip(target_tilt, self.cfg.tilt_lower, actuator_upper))
        current = float(np.clip(self.actuator_tilt_angle, self.cfg.tilt_lower, actuator_upper))
        max_step = max(float(self.tilt_actuator_rate), 0.0) * float(self.cfg.publish_dt)
        error = target - current
        if abs(error) <= max_step:
            current = target
        elif max_step > 0.0:
            current += math.copysign(max_step, error)
        self.actuator_tilt_angle = float(np.clip(current, self.cfg.tilt_lower, actuator_upper))
        return self.actuator_tilt_angle

    def _rotor_actuator_controls(self, command):
        thrust_scale = max(float(self.gazebo_rotor_thrust_scale), 0.0)
        rotors = np.clip(np.asarray(command.rotors, dtype=np.float32) * thrust_scale, 0.0, 1.0)
        return self._motor_speed_fraction_to_px4_control(np.sqrt(rotors))

    def _rotor_motor_speeds(self, command):
        thrust_scale = max(float(self.gazebo_rotor_thrust_scale), 0.0)
        rotors = np.clip(np.asarray(command.rotors, dtype=np.float32) * thrust_scale, 0.0, 1.0)
        speed_fraction = np.sqrt(rotors)
        max_speed = max(float(self.gazebo_rotor_max_velocity), 1e-6)
        return (speed_fraction * max_speed).astype(np.float32)

    def _publish_motor_speeds(self, motor_speeds):
        msg = Float32MultiArray()
        msg.data = [float(value) for value in motor_speeds[:4]]
        self.motor_speed_publisher.publish(msg)

    def _tilt_velocity_control(self, tilt_vel):
        max_tilt_velocity = max(float(self.cfg.max_tilt_velocity), 1e-6)
        return float(np.clip(float(tilt_vel) / max_tilt_velocity, -1.0, 1.0))

    def _motor_speed_fraction_to_px4_control(self, speed_fraction):
        """Map Gazebo motor speed fraction to the PX4 actuator input.

        Training's rotor action is normalized thrust. Gazebo's M4 motor plugin
        turns actuator input into motor speed, then computes force from speed
        squared. The caller converts thrust to speed with sqrt before using
        this SDF channel mapping.
        """
        scale = max(float(self.gazebo_rotor_input_scaling), 1e-6)
        zero = float(self.gazebo_rotor_zero_position_armed)
        max_speed = max(float(self.gazebo_rotor_max_velocity), 1e-6)
        desired_speed = np.clip(speed_fraction, 0.0, 1.0) * max_speed
        controls = (desired_speed - zero) / scale
        return np.clip(controls, -1.0, 1.0).astype(np.float32)

    def _limited_tilt_velocity(self, tilt_vel):
        max_tilt_velocity = max(float(self.cfg.max_tilt_velocity), 1e-6)
        tilt_lower = float(self.cfg.tilt_lower)
        tilt_upper = min(float(self.tilt_actuator_upper), float(self.cfg.tilt_upper))
        stop_tolerance = max(float(self.cfg.publish_dt) * max_tilt_velocity * 0.5, 1e-4)
        command_tilt = float(
            np.clip(
                self.actuator_tilt_angle,
                self.cfg.tilt_lower,
                tilt_upper,
            )
        )

        if command_tilt <= tilt_lower + stop_tolerance and tilt_vel < 0.0:
            return 0.0
        if command_tilt >= tilt_upper - stop_tolerance and tilt_vel > 0.0:
            return 0.0
        if abs(float(tilt_vel)) < 1e-9:
            return 0.0
        return math.copysign(max_tilt_velocity, float(tilt_vel))

    def _publish_tilt_vel(self, tilt_vel):
        msg = TiltVel()
        max_tilt_velocity = max(float(self.cfg.max_tilt_velocity), 1e-6)
        limited_tilt_vel = self._limited_tilt_velocity(tilt_vel)
        msg.value = float(np.clip(limited_tilt_vel / max_tilt_velocity, -1.0, 1.0))
        msg.timestamp = self._timestamp_us()
        self.tilt_vel_publisher.publish(msg)

    def _publish_tilt_vel_if_enabled(self, tilt_vel):
        if self.publish_tilt_vel:
            self._publish_tilt_vel(tilt_vel)

    def _publish_offboard_control_mode_direct_actuator(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = True
        msg.timestamp = self._timestamp_us()
        self.offboard_control_mode_publisher.publish(msg)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0, from_external=True):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = bool(from_external)
        msg.timestamp = self._timestamp_us()
        self.vehicle_command_publisher.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
            PX4_FORCE_ARM_MAGIC,
            from_external=False,
        )

    def _retry_policy_load(self):
        now = time.monotonic()
        if now - self.last_warn_time > 5.0:
            self.last_warn_time = now
            if self.policy.load():
                self.policy_loaded = True
                self.get_logger().info(f"Loaded RL policy from {self.cfg.policy_path}")
                return
            self.get_logger().warn(
                f"RL policy is unavailable: {self.policy.error}. "
                f"Expected: {self.cfg.policy_path}"
            )

    def _warn_waiting_for_state(self):
        now = time.monotonic()
        if now - self.last_warn_time > 5.0:
            self.last_warn_time = now
            self.get_logger().warn("Waiting for PX4 groundtruth state before arming RL")

    def _timestamp_us(self):
        return int(Clock().now().nanoseconds / 1000)

    def _log_stamp(self):
        wall = time.time()
        wall_seconds = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(wall))
        wall_ms = min(int((wall - math.floor(wall)) * 1000.0), 999)
        return (
            f"wall={wall_seconds}.{wall_ms:03d}, "
            f"mono_s={time.monotonic():.3f}, "
            f"ros_us={self._timestamp_us()}"
        )

    def _refresh_observation_state(self):
        self.observations.update_px4_state(
            self.position, self.quaternion, self.velocity, self.angular_velocity
        )
        self._set_observation_tilt()

    def _set_observation_tilt(self):
        tilt = self.measured_tilt_angle if self.use_topic_tilt_velocity else self.actuator_tilt_angle
        self.observations.set_tilt_angle(tilt)

    def vehicle_local_position_groundtruth_callback(self, msg):
        self.position[:] = [msg.x, msg.y, msg.z]
        self.velocity[:] = [msg.vx, msg.vy, msg.vz]
        self._refresh_observation_state()

    def vehicle_attitude_groundtruth_callback(self, msg):
        self.quaternion[:] = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        self._refresh_observation_state()

    def vehicle_angular_velocity_groundtruth_callback(self, msg):
        self.angular_velocity[:] = [msg.xyz[0], msg.xyz[1], msg.xyz[2]]
        self._refresh_observation_state()

    def tilt_angle_callback(self, msg):
        measured_tilt = float(np.clip(msg.value, self.cfg.tilt_lower, self.cfg.tilt_upper))
        self.measured_tilt_angle = measured_tilt
        if self.use_topic_tilt_velocity:
            self.actuator_tilt_angle = measured_tilt
            self.adapter.set_tilt_angle(measured_tilt)
            self.observations.set_tilt_angle(measured_tilt)

    def vehicle_command_ack_callback(self, msg):
        interesting = {
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: "ARM_DISARM",
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: "SET_MODE",
        }
        name = interesting.get(msg.command)
        if name is not None:
            self.get_logger().info(
                f"PX4 command ack {name}: result={msg.result}, "
                f"result_param1={msg.result_param1}, result_param2={msg.result_param2}"
            )

    def vehicle_control_mode_callback(self, msg):
        self.px4_offboard = bool(msg.flag_control_offboard_enabled)

    def vehicle_status_callback(self, msg):
        self.px4_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.px4_nav_state = int(msg.nav_state)


def main(args=None):
    rclpy.init(args=args)
    node = RLCombinedSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
