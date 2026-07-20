"""ROS 2 hardware policy, fixed-action, and observation-shadow node."""

import json
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from custom_msgs.msg import DriveVel, TiltVel
from px4_msgs.msg import (
    ActuatorMotors,
    InputRc,
    OffboardControlMode,
    TiltAngle,
    VehicleCommand,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleOdometry,
    VehicleStatus,
)
from atmo.rl_combined_runtime import (
    CombinedObservationBuilder,
    CombinedStage1Config,
    LandingActionAdapter,
    PolicyRunner,
)


QUEUE_SIZE = int(os.getenv("ATMO_RL_QUEUE_SIZE", "10"))
RC_MAX = int(os.getenv("ATMO_RL_RC_MAX", "1934"))
RC_MARGIN = int(os.getenv("ATMO_RL_RC_MARGIN", "100"))
OFFBOARD_CHANNEL = int(os.getenv("ATMO_RL_OFFBOARD_CHANNEL", "8"))
RL_CHANNEL = int(os.getenv("ATMO_RL_CHANNEL", "7"))
ODOMETRY_TIMEOUT_S = 0.5
VALID_MODES = {"policy", "action_test", "sensor_test", "shadow"}
ACTION_NAMES = ("lift", "roll", "pitch", "yaw", "tilt", "drive", "turn")
ROTOR_ACTIONS = {"lift", "roll", "pitch", "yaw"}


class RLCombinedHardware(Node):
    """Run or inspect the deployment path on the companion computer."""

    def __init__(self):
        super().__init__("rl_combined_hardware")

        self.mode = os.getenv("ATMO_RL_HARDWARE_MODE", "policy").strip().lower()
        if self.mode not in VALID_MODES:
            raise ValueError(
                f"ATMO_RL_HARDWARE_MODE must be one of {sorted(VALID_MODES)}, got {self.mode!r}"
            )
        self.route = os.getenv("ATMO_RL_ROUTE", "landing").strip().lower()
        if self.route not in {"takeoff", "landing"}:
            raise ValueError("ATMO_RL_ROUTE must be 'takeoff' or 'landing'")
        os.environ["ATMO_RL_ROUTE"] = self.route
        self.cfg = CombinedStage1Config(
            randomize_reset=False,
            randomize_motor_dynamics=False,
            observation_noise=False,
        )
        self.adapter = LandingActionAdapter(self.cfg)
        self.observations = CombinedObservationBuilder(self.cfg)
        self.test_action = None
        self.action_test = os.getenv("ATMO_RL_ACTION_TEST", "lift").strip().lower()
        self.action_test_duration = float(os.getenv("ATMO_RL_ACTION_TEST_DURATION", "5.0"))
        kill_test_passed = os.getenv("ATMO_RL_KILL_TEST_PASSED", "0").lower() in {
            "1", "true", "yes", "on"
        }
        if self.mode == "action_test":
            if self.action_test not in ACTION_NAMES:
                raise ValueError(f"ATMO_RL_ACTION_TEST must be one of {ACTION_NAMES}")
            magnitude = float(os.getenv("ATMO_RL_ACTION_MAGNITUDE", "0.1"))
            sign = os.getenv("ATMO_RL_ACTION_SIGN", "positive").strip().lower()
            baseline = float(os.getenv("ATMO_RL_ROTOR_BASELINE", "-0.8"))
            max_magnitude = 1.0 if self.action_test == "tilt" else 0.25
            if not 0.0 < magnitude <= max_magnitude:
                raise ValueError(
                    f"ATMO_RL_ACTION_MAGNITUDE must be in (0, {max_magnitude}] "
                    f"for {self.action_test}"
                )
            if self.action_test == "tilt" and magnitude < 0.5:
                raise ValueError("Tilt actions below 0.5 round to zero; use magnitude 1.0")
            if sign not in {"positive", "negative"}:
                raise ValueError("ATMO_RL_ACTION_SIGN must be positive or negative")
            if not -1.0 <= baseline <= -0.5:
                raise ValueError("ATMO_RL_ROTOR_BASELINE must be in [-1.0, -0.5]")
            if self.action_test in {"roll", "pitch", "yaw"} and not kill_test_passed:
                raise ValueError(
                    "Run the propeller-free lift/kill test first, then set "
                    "kill_test_passed:=true for differential rotor tests"
                )
            self.test_action = np.zeros(self.cfg.action_dim, dtype=np.float32)
            if self.action_test in ROTOR_ACTIONS:
                self.test_action[0] = baseline
            value = magnitude if sign == "positive" else -magnitude
            if self.action_test == "lift":
                self.test_action[0] = float(np.clip(baseline + value, -1.0, -0.5))
            else:
                self.test_action[ACTION_NAMES.index(self.action_test)] = value

        self.policy = None
        self.policy_loaded = False
        if self.mode in {"policy", "shadow"}:
            self.policy = PolicyRunner(self.cfg)
            self.policy_loaded = self.policy.load()

        if self.policy_loaded:
            self.get_logger().info(f"Loaded RL policy from {self.cfg.policy_path}")
        elif self.mode == "policy":
            self.get_logger().warn(
                f"RL policy is not active: {self.policy.error}. "
                "Drop the .pth at that path or set ATMO_RL_POLICY_PATH."
            )
        self.get_logger().info(
            "ATMO RL hardware config: "
            f"mode={self.mode}, route={self.route}, "
            f"policy_hz={self.cfg.policy_hz:.1f}, publish_hz={self.cfg.publish_hz:.1f}, "
            f"obs_dim={self.cfg.observation_dim}, action_dim={self.cfg.action_dim}, "
            f"motor_alpha={self.adapter.motor_alpha:.3f}"
        )
        if self.route == "takeoff":
            self.get_logger().info(
                "Combined hardware trajectory: DRIVE hold, TAKEOFF rise 1.0 m, then FLIGHT hold"
            )
        else:
            self.get_logger().info(
                "Combined hardware trajectory: FLIGHT hold, LANDING to z=0.200 m, then DRIVE hold"
            )
        self.get_logger().info(
            f"RC gates: offboard channel index {OFFBOARD_CHANNEL}, "
            f"RL channel index {RL_CHANNEL}. Motor kill must be configured in PX4."
        )
        if self.test_action is not None:
            self.get_logger().info(
                f"ACTION TEST {self.action_test}: raw [lift, roll, pitch, yaw, tilt, drive, turn]="
                f"{np.array2string(self.test_action, precision=3)}. "
                "Start with both gates low; raise both, lower RL, then raise RL to run."
            )
            if self.action_test == "lift":
                self.get_logger().warn(
                    "PROPELLERS MUST BE REMOVED. This first lift test must prove that the "
                    "physical kill switch stops all four motors."
                )

        self.vehicle_command_publisher = None
        self.offboard_control_mode_publisher = None
        self.actuator_motors_publisher = None
        self.tilt_vel_publisher = None
        self.drive_vel_publisher = None
        self.manual_override_publisher = None
        if self.mode not in {"shadow", "sensor_test"}:
            self.vehicle_command_publisher = self.create_publisher(
                VehicleCommand, "/fmu/in/vehicle_command", QUEUE_SIZE
            )
            self.offboard_control_mode_publisher = self.create_publisher(
                OffboardControlMode, "/fmu/in/offboard_control_mode", QUEUE_SIZE
            )
            self.actuator_motors_publisher = self.create_publisher(
                ActuatorMotors, "/fmu/in/actuator_motors", QUEUE_SIZE
            )
            self.tilt_vel_publisher = self.create_publisher(TiltVel, "/tilt_vel", QUEUE_SIZE)
            self.drive_vel_publisher = self.create_publisher(DriveVel, "/drive_vel", QUEUE_SIZE)
            self.manual_override_publisher = self.create_publisher(
                Bool, "/atmo/rl/manual_override", QUEUE_SIZE
            )

        self.create_subscription(
            InputRc,
            "/fmu/out/input_rc",
            self.rc_listener_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/vrpn_mocap/m4_base/pose",
            self.mocap_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/in/vehicle_visual_odometry",
            self.visual_odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
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

        self.offboard_switch = False
        self.rl_switch = False
        self.rl_active = False
        self.offboard = False
        self.px4_armed = False
        self.px4_offboard = False
        self.px4_nav_state = None
        self.offboard_setpoint_counter = 0
        self.last_policy_time = 0.0
        self.last_mode_request_time = 0.0
        self.last_handoff_request_time = 0.0
        self.last_odometry_time = 0.0
        self.sensor_times = {}
        self.sensor_counts = {}
        self.test_phase = "waiting_low"
        self.test_started_at = 0.0
        self.last_warn_time = 0.0
        self.last_sample_log_time = 0.0
        self.handoff_latched = False
        self.handoff_complete = False
        self.terminal_transition = None
        self.raw_px4_position = np.zeros(3, dtype=np.float32)
        self.raw_px4_quaternion = np.array((1.0, 0.0, 0.0, 0.0), dtype=np.float32)
        self.raw_px4_velocity = np.zeros(3, dtype=np.float32)
        self.raw_px4_angular_velocity = np.zeros(3, dtype=np.float32)
        self.raw_mocap_pose = np.zeros(7, dtype=np.float32)
        self.raw_visual_odometry = np.zeros(7, dtype=np.float32)
        self.raw_rc_values = []
        self.shadow_log = None
        self.shadow_log_path = None
        if self.mode == "shadow":
            default_name = f"/tmp/atmo_rl_{self.route}_shadow.jsonl"
            self.shadow_log_path = Path(
                os.getenv("ATMO_RL_SHADOW_LOG", default_name)
            ).expanduser()
            self.shadow_log_path.parent.mkdir(parents=True, exist_ok=True)
            self.shadow_log = self.shadow_log_path.open("w", encoding="utf-8", buffering=1)
            self.shadow_log.write(json.dumps({
                "type": "metadata",
                "route": self.route,
                "observation_dim": self.cfg.observation_dim,
                "action_dim": self.cfg.action_dim,
                "policy_path": str(self.cfg.policy_path),
                "policy_loaded": self.policy_loaded,
            }) + "\n")
            self.get_logger().warn(
                "SHADOW MODE: no command publishers were created; raise only the RL switch "
                f"to reset/start the fixed reference. Log: {self.shadow_log_path}"
            )
            if not self.policy_loaded:
                self.get_logger().warn(
                    f"Policy output will be null in the shadow log: {self.policy.error}"
                )

        self.timer = self.create_timer(self.cfg.publish_dt, self.timer_callback)

    def timer_callback(self):
        if self.mode == "sensor_test":
            self._sensor_test_timer()
            return
        if self.mode == "shadow":
            self._shadow_timer_callback()
            return
        if self.mode == "action_test":
            self._action_test_timer()
            return

        if self.handoff_latched:
            if not self.offboard_switch and not self.rl_switch:
                self._clear_terminal_latch()
            else:
                self._terminal_handoff_timer()
            return

        requested = self.offboard_switch and self.rl_switch
        if not requested:
            self._leave_rl_idle()
            return
        if self.mode == "policy" and not self.policy_loaded:
            self._warn_missing_policy()
            return
        if not self.observations.state_seen:
            self._warn_waiting_for_state()
            return
        if time.monotonic() - self.last_odometry_time > ODOMETRY_TIMEOUT_S:
            self._warn_waiting_for_state("PX4 odometry is stale; RL remains inactive")
            self._leave_rl_idle()
            return

        self._switch_to_offboard()
        if not self.offboard:
            return
        if not self.rl_active:
            self._start_policy_session()

        self._publish_offboard_control_mode_direct_actuator()
        self._publish_manual_override(False)
        self._update_command_if_due()
        if self.handoff_latched:
            self._terminal_handoff_timer()
            return
        self._publish_actuator_motors(self.adapter.last_command.rotors)
        self._publish_tilt_vel(self.adapter.last_command.tilt_velocity)
        self._publish_drive_vel(
            self.adapter.last_command.drive_speed, self.adapter.last_command.turn_speed
        )

    def _update_command_if_due(self):
        now = time.monotonic()
        if now - self.last_policy_time < self.cfg.policy_dt:
            return

        obs = self.observations.observation()
        transition = self.observations.last_debug.get("combined_transition", "none")
        if transition != "none":
            self.get_logger().info(
                f"Combined hardware transition: {transition}, "
                f"mode={self.observations.last_debug['combined_mode']}"
            )
        action = self.policy.action(obs)
        self.last_policy_time = now
        if action is None:
            return

        command = self.adapter.pre_physics_step(action)
        self.observations.set_tilt_angle(command.tilt_angle)
        self.observations.append_action(command.semantic_action)
        if self.mode == "policy" and transition in {"takeoff_to_flight", "landing_to_drive"}:
            self._begin_terminal_handoff(transition)

    def _action_test_timer(self):
        both_high = self.offboard_switch and self.rl_switch
        if self.test_phase == "waiting_low":
            self._publish_safe_output()
            if not self.offboard_switch and not self.rl_switch:
                self.test_phase = "waiting_prepare"
        elif self.test_phase == "waiting_prepare":
            self._publish_safe_output()
            if both_high:
                self.test_phase = "waiting_trigger"
                self.get_logger().warn("Action test prepared; lower only the RL switch")
        elif self.test_phase == "waiting_trigger":
            self._publish_safe_output()
            if self.offboard_switch and not self.rl_switch:
                self.test_phase = "ready"
                self.get_logger().warn("Action test ready; raise the RL switch to apply output")
            elif not self.offboard_switch:
                self.test_phase = "waiting_low"
        elif self.test_phase == "ready":
            self._publish_safe_output()
            if both_high:
                self.test_phase = "starting" if self.action_test in ROTOR_ACTIONS else "running"
                self.test_started_at = time.monotonic()
            elif not self.offboard_switch:
                self.test_phase = "waiting_low"
        elif self.test_phase == "starting":
            if not both_high:
                self._finish_action_test("gate released")
                return
            self._publish_actuator_motors(np.zeros(4, dtype=np.float32))
            self._switch_to_offboard()
            if self.offboard:
                self.test_phase = "running"
                self.test_started_at = time.monotonic()
                self.adapter = LandingActionAdapter(self.cfg)
                self.adapter.motor_alpha = 1.0
                self.get_logger().warn(f"Running {self.action_test} action test")
        elif self.test_phase == "running":
            if not both_high:
                self._finish_action_test("gate released")
                return
            if time.monotonic() - self.test_started_at >= self.action_test_duration:
                self._finish_action_test("time limit reached")
                return
            command = self.adapter.pre_physics_step(self.test_action)
            if self.action_test in ROTOR_ACTIONS:
                self._publish_offboard_control_mode_direct_actuator()
                self._publish_actuator_motors(command.rotors)
            elif self.action_test == "tilt":
                self._publish_tilt_vel(command.tilt_velocity)
            else:
                self._publish_drive_vel(command.drive_speed, command.turn_speed)
            if time.monotonic() - self.last_sample_log_time >= 0.25:
                self.last_sample_log_time = time.monotonic()
                self._log_action_test(command)
        else:
            self._publish_safe_output()
            if not self.offboard_switch and not self.rl_switch:
                self.test_phase = "waiting_prepare"

    def _finish_action_test(self, reason):
        self._publish_safe_output()
        if self.action_test in ROTOR_ACTIONS and self.px4_armed:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.offboard = False
        self.test_phase = "complete"
        self.get_logger().warn(
            f"Action test stopped ({reason}); cycle both gates low before another test"
        )

    def _publish_safe_output(self):
        if self.actuator_motors_publisher is not None and self.px4_offboard:
            self._publish_offboard_control_mode_direct_actuator()
            self._publish_actuator_motors(np.zeros(4, dtype=np.float32))
        self._publish_tilt_vel(0.0)
        self._publish_drive_vel(0.0, 0.0)

    def _log_action_test(self, command):
        ages = self._sensor_ages()
        self.get_logger().info(
            f"Action test {self.action_test}: raw={np.array2string(command.raw_action, precision=3)}, "
            f"semantic={np.array2string(command.semantic_action, precision=3)}, "
            f"rotors={np.array2string(command.rotors, precision=3)}, "
            f"tilt_vel={command.tilt_velocity:+.3f}, measured_tilt={self.observations.tilt_angle:+.3f}, "
            f"drive={command.drive_speed:+.3f}, turn={command.turn_speed:+.3f}, "
            f"armed={self.px4_armed}, offboard={self.px4_offboard}, sensor_ages={ages}"
        )

    def _sensor_test_timer(self):
        now = time.monotonic()
        if now - self.last_sample_log_time < 1.0:
            return
        self.last_sample_log_time = now
        offboard_rc = (
            self.raw_rc_values[OFFBOARD_CHANNEL]
            if OFFBOARD_CHANNEL < len(self.raw_rc_values)
            else None
        )
        rl_rc = self.raw_rc_values[RL_CHANNEL] if RL_CHANNEL < len(self.raw_rc_values) else None
        self.get_logger().info(
            f"Sensor connectivity: counts={self.sensor_counts}, ages_s={self._sensor_ages()}, "
            f"px4_ned_pos={np.array2string(self.raw_px4_position, precision=3)}, "
            f"px4_ned_vel={np.array2string(self.raw_px4_velocity, precision=3)}, "
            f"mocap_xyz_qxyzw={np.array2string(self.raw_mocap_pose, precision=3)}, "
            f"visual_odom_ned_xyz_qwxyz={np.array2string(self.raw_visual_odometry, precision=3)}, "
            f"tilt_rad={self.observations.tilt_angle:+.3f}, armed={self.px4_armed}, "
            f"offboard={self.px4_offboard}, nav_state={self.px4_nav_state}, "
            f"rc_gates=({self.offboard_switch},{self.rl_switch}), "
            f"rc_values=({offboard_rc},{rl_rc})"
        )

    def _mark_sensor(self, name):
        self.sensor_times[name] = time.monotonic()
        self.sensor_counts[name] = self.sensor_counts.get(name, 0) + 1

    def _sensor_ages(self):
        now = time.monotonic()
        return {name: round(now - stamp, 3) for name, stamp in self.sensor_times.items()}

    def _shadow_timer_callback(self):
        if not self.rl_switch:
            if self.rl_active:
                self.get_logger().info("Shadow trigger released; observation logging paused")
            self.rl_active = False
            return
        if not self.observations.state_seen:
            self._warn_waiting_for_state("Shadow mode is waiting for PX4 odometry")
            return
        if time.monotonic() - self.last_odometry_time > ODOMETRY_TIMEOUT_S:
            self._warn_waiting_for_state("PX4 odometry is stale; shadow logging paused")
            self.rl_active = False
            return
        if not self.rl_active:
            self._start_policy_session()

        now = time.monotonic()
        if now - self.last_policy_time < self.cfg.policy_dt:
            return
        self.last_policy_time = now
        observation = self.observations.observation()
        transition = self.observations.last_debug.get("combined_transition", "none")
        if transition != "none":
            self.get_logger().info(
                f"Combined shadow transition: {transition}, "
                f"mode={self.observations.last_debug['combined_mode']}"
            )
        action = self.policy.action(observation) if self.policy_loaded else None
        command = None
        if action is not None:
            self.adapter.set_tilt_angle(self.observations.tilt_angle)
            command = self.adapter.pre_physics_step(action)
            self.observations.append_action(command.semantic_action)
        self._write_shadow_sample(observation, action, command)

    def _write_shadow_sample(self, observation, action, command):
        debug = self.observations.last_debug
        sample = {
            "type": "sample",
            "wall_time_s": time.time(),
            "monotonic_time_s": time.monotonic(),
            "reference_time_s": float(debug.get("reference_time", 0.0)),
            "combined_mode": debug.get("combined_mode"),
            "combined_transition": debug.get("combined_transition", "none"),
            "raw_px4_position_ned": self.raw_px4_position.tolist(),
            "raw_px4_quaternion_wxyz": self.raw_px4_quaternion.tolist(),
            "raw_px4_velocity_ned": self.raw_px4_velocity.tolist(),
            "raw_px4_angular_velocity_frd": self.raw_px4_angular_velocity.tolist(),
            "training_position_enu": self.observations.position.tolist(),
            "training_quaternion_wxyz": self.observations.quat_wxyz.tolist(),
            "training_velocity_enu": self.observations.linear_velocity.tolist(),
            "training_angular_velocity_world": self.observations.angular_velocity_w.tolist(),
            "tilt_angle_rad": float(self.observations.tilt_angle),
            "reference_position": np.asarray(debug.get("reference_position", np.zeros(3))).tolist(),
            "reference_velocity": np.asarray(debug.get("reference_velocity", np.zeros(3))).tolist(),
            "reference_acceleration": np.asarray(debug.get("reference_accel", np.zeros(3))).tolist(),
            "position_error": np.asarray(debug.get("reference_pos_error", np.zeros(3))).tolist(),
            "velocity_error": np.asarray(debug.get("reference_velocity_error", np.zeros(3))).tolist(),
            "yaw_error_rad": float(debug.get("yaw_error", 0.0)),
            "policy_action": None if action is None else np.asarray(action).tolist(),
            "semantic_action": None if command is None else command.semantic_action.tolist(),
            "rotor_pre_filter_preview": None if command is None else command.rotors_unfiltered.tolist(),
            "rotor_command_preview": None if command is None else command.rotors.tolist(),
            "tilt_velocity_preview_rad_s": None if command is None else float(command.tilt_velocity),
            "drive_speed_preview": None if command is None else float(command.drive_speed),
            "turn_speed_preview": None if command is None else float(command.turn_speed),
            "sensor_ages_s": self._sensor_ages(),
            "offboard_switch": self.offboard_switch,
            "rl_switch": self.rl_switch,
            "px4_armed": self.px4_armed,
            "px4_offboard": self.px4_offboard,
            "px4_nav_state": self.px4_nav_state,
            "observation": observation.tolist(),
        }
        self.shadow_log.write(json.dumps(sample, separators=(",", ":")) + "\n")

        now = time.monotonic()
        if now - self.last_sample_log_time >= 1.0:
            self.last_sample_log_time = now
            self.get_logger().info(
                "Shadow sample: "
                f"t_ref={sample['reference_time_s']:.2f}, "
                f"pos={np.array2string(self.observations.position, precision=3)}, "
                f"vel={np.array2string(self.observations.linear_velocity, precision=3)}, "
                f"ref={np.array2string(np.asarray(sample['reference_position']), precision=3)}, "
                f"pos_err={np.array2string(np.asarray(sample['position_error']), precision=3)}, "
                f"yaw_err={sample['yaw_error_rad']:+.3f}, "
                f"obs_norm={float(np.linalg.norm(observation)):.3f}"
            )

    def _start_policy_session(self):
        measured_tilt = float(self.observations.tilt_angle)
        self.rl_active = True
        self.adapter = LandingActionAdapter(self.cfg)
        self.observations.reset_policy_context()
        self.observations.anchor_fixed_vertical_route()
        self.adapter.set_tilt_angle(measured_tilt)
        self.observations.set_tilt_angle(measured_tilt)
        self.last_policy_time = 0.0
        if self.shadow_log is not None:
            self.shadow_log.write(json.dumps({
                "type": "session_start",
                "wall_time_s": time.time(),
                "training_start_position": self.observations.position.tolist(),
                "tilt_angle_rad": measured_tilt,
                "route": self.route,
            }) + "\n")
        self.get_logger().info(
            f"{self.mode} session engaged; starting fresh fixed reference and action history"
        )

    def _begin_terminal_handoff(self, transition):
        self.handoff_latched = True
        self.handoff_complete = False
        self.terminal_transition = transition
        self.last_handoff_request_time = 0.0
        destination = "PX4 Position mode" if self.route == "takeoff" else "disarmed ground control"
        self.get_logger().warn(
            f"Combined route complete at {transition}; beginning handoff to {destination}. "
            "Cycle both RL and Offboard switches low before another session."
        )

    def _terminal_handoff_timer(self):
        self._publish_tilt_vel(0.0)
        self._publish_drive_vel(0.0, 0.0)
        self._publish_manual_override(True)

        if self.route == "takeoff":
            position_control = self.px4_nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL
            if not self.px4_offboard and position_control:
                if not self.handoff_complete:
                    self.handoff_complete = True
                    self.offboard = False
                    self.get_logger().warn("Takeoff handoff complete: PX4 Position mode owns flight control")
                return
            if self.px4_offboard:
                self._publish_offboard_control_mode_direct_actuator()
                self._publish_actuator_motors(self.adapter.last_command.rotors)
            self._request_handoff_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0)
            return

        if not self.px4_armed:
            if not self.handoff_complete:
                self.handoff_complete = True
                self.offboard = False
                self.get_logger().warn("Landing handoff complete: PX4 is disarmed")
            return
        if self.px4_offboard:
            self._publish_offboard_control_mode_direct_actuator()
            self._publish_actuator_motors(np.zeros(4, dtype=np.float32))
        self._request_handoff_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def _request_handoff_command(self, command, param1=0.0, param2=0.0):
        now = time.monotonic()
        if now - self.last_handoff_request_time < 1.0:
            return
        self.last_handoff_request_time = now
        self._publish_vehicle_command(command, param1, param2)

    def _clear_terminal_latch(self):
        self.get_logger().info("Terminal handoff latch cleared; controls may be engaged again")
        self.handoff_latched = False
        self.handoff_complete = False
        self.terminal_transition = None
        self.last_handoff_request_time = 0.0
        self._leave_rl_idle()

    def _leave_rl_idle(self):
        if self.rl_active or self.offboard:
            self.get_logger().warn("RL switch released; stopping policy outputs")
        self.rl_active = False
        self.offboard = False
        self.offboard_setpoint_counter = 0
        self._publish_tilt_vel(0.0)
        self._publish_drive_vel(0.0, 0.0)
        self._publish_manual_override(False)

    def _switch_to_offboard(self):
        self._publish_offboard_control_mode_direct_actuator()
        if self.px4_armed and self.px4_offboard:
            if not self.offboard:
                self.get_logger().info("PX4 confirmed armed offboard/direct-actuator mode")
            self.offboard = True
            return

        self.offboard = False
        if self.offboard_setpoint_counter < 10:
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
        self.get_logger().info(
            "Waiting for PX4 confirmation: "
            f"armed={self.px4_armed}, offboard={self.px4_offboard}, "
            f"nav_state={self.px4_nav_state}"
        )

    def _publish_actuator_motors(self, rotors):
        msg = ActuatorMotors()
        for idx in range(12):
            msg.control[idx] = float("nan")
        for idx in range(4):
            msg.control[idx] = float(np.clip(rotors[idx], 0.0, 1.0))
        msg.timestamp = self._timestamp_us()
        self.actuator_motors_publisher.publish(msg)

    def _publish_tilt_vel(self, tilt_vel):
        msg = TiltVel()
        max_tilt_velocity = max(float(self.cfg.max_tilt_velocity), 1e-6)
        msg.value = float(np.clip(tilt_vel / max_tilt_velocity, -1.0, 1.0))
        msg.timestamp = self._timestamp_us()
        self.tilt_vel_publisher.publish(msg)

    def _publish_drive_vel(self, drive_speed, turn_speed):
        msg = DriveVel()
        msg.drivespeed = float(np.clip(drive_speed, -1.0, 1.0))
        msg.turnspeed = float(np.clip(turn_speed, -1.0, 1.0))
        msg.timestamp = self._timestamp_us()
        self.drive_vel_publisher.publish(msg)

    def _publish_manual_override(self, enabled):
        if self.manual_override_publisher is None:
            return
        msg = Bool()
        msg.data = bool(enabled)
        self.manual_override_publisher.publish(msg)

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

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self.vehicle_command_publisher.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _warn_missing_policy(self):
        now = time.monotonic()
        if now - self.last_warn_time > 5.0:
            self.last_warn_time = now
            self.get_logger().warn(
                f"Waiting for policy file. Expected: {self.cfg.policy_path}"
            )

    def _warn_waiting_for_state(self, message="Waiting for PX4 odometry before arming RL"):
        now = time.monotonic()
        if now - self.last_warn_time > 5.0:
            self.last_warn_time = now
            self.get_logger().warn(message)

    def _timestamp_us(self):
        return int(Clock().now().nanoseconds / 1000)

    def rc_listener_callback(self, msg):
        self._mark_sensor("rc")
        self.raw_rc_values = list(msg.values)
        self.offboard_switch = self._channel_high(msg, OFFBOARD_CHANNEL)
        self.rl_switch = self._channel_high(msg, RL_CHANNEL)

    def _channel_high(self, msg, channel):
        if channel < 0 or channel >= len(msg.values):
            return False
        return int(msg.values[channel]) >= RC_MAX - RC_MARGIN

    def vehicle_odometry_callback(self, msg):
        self._mark_sensor("px4_odometry")
        values = np.asarray(
            (*msg.position, *msg.q, *msg.velocity, *msg.angular_velocity),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(values)):
            self._warn_waiting_for_state("Ignoring non-finite PX4 odometry")
            return
        self.raw_px4_position[:] = msg.position
        self.raw_px4_quaternion[:] = msg.q
        self.raw_px4_velocity[:] = msg.velocity
        self.raw_px4_angular_velocity[:] = msg.angular_velocity
        self.observations.update_px4_state(
            msg.position,
            msg.q,
            msg.velocity,
            msg.angular_velocity,
        )
        self.last_odometry_time = time.monotonic()

    def mocap_callback(self, msg):
        values = np.asarray(
            (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ),
            dtype=np.float64,
        )
        if np.all(np.isfinite(values)):
            self.raw_mocap_pose[:] = values
            self._mark_sensor("mocap")

    def visual_odometry_callback(self, msg):
        values = np.asarray((*msg.position, *msg.q), dtype=np.float64)
        if np.all(np.isfinite(values)):
            self.raw_visual_odometry[:] = values
            self._mark_sensor("visual_odometry")

    def tilt_angle_callback(self, msg):
        self._mark_sensor("tilt")
        self.adapter.set_tilt_angle(msg.value)
        self.observations.set_tilt_angle(msg.value)

    def vehicle_command_ack_callback(self, msg):
        self._mark_sensor("command_ack")
        names = {
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: "ARM_DISARM",
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: "SET_MODE",
        }
        name = names.get(msg.command)
        if name is not None:
            self.get_logger().info(f"PX4 command ack {name}: result={msg.result}")

    def vehicle_control_mode_callback(self, msg):
        self._mark_sensor("control_mode")
        self.px4_offboard = bool(msg.flag_control_offboard_enabled)

    def vehicle_status_callback(self, msg):
        self._mark_sensor("vehicle_status")
        self.px4_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.px4_nav_state = int(msg.nav_state)

    def destroy_node(self):
        if self.shadow_log is not None:
            self.shadow_log.close()
            self.shadow_log = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RLCombinedHardware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
