"""ROS 2 hardware policy, fixed-action, and observation-shadow node."""

import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

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
from atmo.policy_contract import DeploymentContract, find_contract
from atmo import px4_topics
from atmo.roboclaw_safety import kill_engaged
from atmo.rl_combined_runtime import (
    CombinedObservationBuilder,
    CombinedStage1Config,
    LandingActionAdapter,
    PolicyRunner,
)
from atmo.rl_landing_stage1_runtime import quat_wxyz_to_rotmat

QUEUE_SIZE = int(os.getenv("ATMO_RL_QUEUE_SIZE", "10"))
RC_MAX = int(os.getenv("ATMO_RL_RC_MAX", "1934"))
RC_MARGIN = int(os.getenv("ATMO_RL_RC_MARGIN", "100"))
OFFBOARD_CHANNEL = int(os.getenv("ATMO_RL_OFFBOARD_CHANNEL", "8"))
RL_CHANNEL = int(os.getenv("ATMO_RL_CHANNEL", "7"))
# These are 0-BASED INDICES into InputRc.values, not 1-based channel numbers.
# The distinction has bitten this project once already: the CATMO tree on the
# vehicle stores 1-based numbers and subtracts one at every use
# (`msg.values[offboard_channel - 1]`), so its `offboard_channel = 8` and this
# module's `OFFBOARD_CHANNEL = 8` name channels one apart. Carlo's 8 is ch8,
# index 7. This module's 8 is ch9, index 8. Do not "fix" one to match the
# other without checking which convention the file you are editing uses.
#
# Set ATMO_RL_OFFBOARD_CHANNEL to -1 on airframes that have only ONE gate
# switch. Measured on this vehicle 2026-08-14: the transmitter is a T14SG
# carrying exactly two mapped switches -- kill (ch13, mirrored to ch17) and
# one free switch on ch8 -- while the channel map it inherited was written for
# a T18SZ. There is no second switch to gate on, and ch9 has never had one.
OFFBOARD_GATE_ENABLED = OFFBOARD_CHANNEL >= 0
ODOMETRY_TIMEOUT_S = 0.5
# The RC gates are a deadman: they expire rather than latching at the last
# value seen, and an implausible pulse width reads as LOW, not as a switch
# position. PX4's kill covers the rotors; only this covers tilt and drive.
RC_TIMEOUT_S = float(os.getenv("ATMO_RL_RC_TIMEOUT_S", "0.5"))
RC_PLAUSIBLE_MIN_US = int(os.getenv("ATMO_RL_RC_PLAUSIBLE_MIN_US", "900"))
# "ground" is the policy loop with the ROTORS CUT: the policy runs closed loop
# on tilt and the wheels, and nothing is ever published to actuator_motors, PX4
# is never switched to offboard, and the vehicle is never armed. It is the
# ground half of a policy run -- everything except the parts that could fly it,
# and it needs no mocap because the reference is synthetic.
#
# It exists as its own mode rather than a flag on "policy" so that reaching
# rotors requires typing a different word, not clearing a flag.
VALID_MODES = {"policy", "ground", "action_test", "sensor_test", "shadow"}
# Modes that run the policy pipeline and actuate.
POLICY_MODES = {"policy", "ground"}
# Replace PX4's position/velocity with a stationary origin. Defaults ON for
# ground mode, because ground runs without mocap or GPS and PX4's EKF then
# drifts without bound while still reporting finite numbers. Never defaults on
# for `policy`: flying on a fabricated position is not a degraded run, it is a
# crash. Force either way with ATMO_RL_VIRTUAL_POSE=1/0.
_MODE = os.getenv("ATMO_RL_HARDWARE_MODE", "policy").strip().lower()
VIRTUAL_POSE = os.getenv("ATMO_RL_VIRTUAL_POSE", "1" if _MODE == "ground" else "0").lower() in {
    "1",
    "true",
    "yes",
    "on",
}
# Where the vehicle's POSITION comes from.
#
#   mocap  -- straight from the mocap pose, bypassing PX4's estimator entirely
#   px4    -- PX4's EKF via vehicle_odometry
#   (virtual pose, above, is the no-reference fallback)
#
# Direct mocap is preferred over letting the EKF fuse it. Measured here
# 2026-08-14: GPS denied and unfused, the EKF reported (-4428, -699, -72) m
# and 24 m/s while the vehicle sat still, and those values are FINITE so
# nothing downstream rejects them. Fusing mocap should fix that, but it puts
# an estimator with its own failure modes, its own convergence time and its
# own frame conventions between the measurement and the policy. The mocap IS
# the measurement; read it.
POSE_SOURCE = os.getenv("ATMO_RL_POSE_SOURCE", "px4").strip().lower()
if POSE_SOURCE not in {"px4", "mocap"}:
    raise ValueError("ATMO_RL_POSE_SOURCE must be 'px4' or 'mocap'")
# Consume the BRIDGE's odometry, not the raw VRPN pose. mocap_bridge.py exists
# for exactly this ("anything that wants mocap without going through the EKF
# reads this"): it resolves the y-up/z-up frame question in one place, filters
# a real velocity instead of leaving it to be differentiated here, and reports
# rate and dropouts. Subscribing to the raw pose would duplicate all three and
# put the frame convention in two places that can disagree.
MOCAP_ODOM_TOPIC = os.getenv("ATMO_RL_MOCAP_ODOM_TOPIC", "/atmo/groundtruth_odom")
# Ideal skid-steer constants for the virtual odometry. Yaw is damped for the
# same reason the m4 node damps it: an undamped ideal yaw response lets the
# policy unwind a heading error instantly, which is less informative than a
# loop that has to work at it.
VIRTUAL_MAX_SPEED = float(os.getenv("ATMO_RL_VIRTUAL_MAX_SPEED", "1.0"))  # m/s at drive=1
VIRTUAL_MAX_YAW_RATE = float(os.getenv("ATMO_RL_VIRTUAL_MAX_YAW_RATE", "1.5"))  # rad/s at turn=1
VIRTUAL_YAW_DAMPING = float(os.getenv("ATMO_RL_VIRTUAL_YAW_DAMPING", "0.1"))
VIRTUAL_GROUND_Z = float(os.getenv("ATMO_RL_VIRTUAL_GROUND_Z", "0.0"))
ACTION_NAMES = ("lift", "roll", "pitch", "yaw", "tilt", "drive", "turn")
ROTOR_ACTIONS = {"lift", "roll", "pitch", "yaw"}
# Bench convenience for NON-ROTOR action tests only: skip the RL-switch
# ratchet and run as soon as RC is live with the kill released. The kill
# switch and every fail-closed RC path still stop the test instantly --
# autostart removes the START choreography, never the STOP. Rotor tests
# ignore this flag unconditionally.
ACTION_AUTOSTART = os.getenv("ATMO_RL_ACTION_AUTOSTART", "0").lower() in ("1", "true", "yes", "on")


class RLCombinedHardware(Node):
    """Run or inspect the deployment path on the companion computer."""

    def _check_contract(self):
        """Cross-check the runtime config against the exported training contract.

        A policy run on a mismatched observation layout is not a degraded run,
        it is a meaningless one: the network reads whatever happens to be at
        each index. Shadow and the test modes are still informative with a
        mismatch, so they warn and continue; `policy` refuses.

        Set ATMO_RL_SKIP_CONTRACT_CHECK=1 to override, which should only ever
        be a deliberate bench decision.
        """
        path = find_contract()
        if path is None:
            self.get_logger().warn(
                "No deployment contract found. Export one from the training "
                "machine with M4/export_atmo_deployment_contract.py and set "
                "ATMO_RL_CONTRACT. Running unchecked."
            )
            return
        try:
            contract = DeploymentContract.load(path)
        except Exception as exc:
            self.get_logger().error("Contract at %s is invalid: %s" % (path, exc))
            if self.mode in POLICY_MODES:
                raise
            return
        self.get_logger().info("Contract: %s" % contract.describe())
        problems = contract.check_runtime(self.cfg)
        if not problems:
            self.get_logger().info("Runtime agrees with the contract.")
            return
        message = "Runtime DISAGREES with the contract:\n    " + "\n    ".join(problems)
        skip = os.getenv("ATMO_RL_SKIP_CONTRACT_CHECK", "0").lower() in {"1", "true", "yes", "on"}
        if self.mode in POLICY_MODES and not skip:
            raise RuntimeError(
                message + "\n\nRefusing to run a closed-loop policy against a mismatched "
                "observation layout. Re-export the contract if training moved, "
                "or fix the runtime config. Override with "
                "ATMO_RL_SKIP_CONTRACT_CHECK=1 only as a deliberate bench decision."
            )
        self.get_logger().warn(message)

    def __init__(self):
        super().__init__("rl_combined_hardware")

        self.mode = os.getenv("ATMO_RL_HARDWARE_MODE", "policy").strip().lower()
        if self.mode not in VALID_MODES:
            raise ValueError(f"ATMO_RL_HARDWARE_MODE must be one of {sorted(VALID_MODES)}, got {self.mode!r}")
        self.route = os.getenv("ATMO_RL_ROUTE", "landing").strip().lower()
        if self.route not in {"takeoff", "landing", "full"}:
            raise ValueError("ATMO_RL_ROUTE must be 'takeoff', 'landing' or 'full'")
        os.environ["ATMO_RL_ROUTE"] = self.route
        self.cfg = CombinedStage1Config(
            randomize_reset=False,
            randomize_motor_dynamics=False,
            observation_noise=False,
        )
        self.adapter = LandingActionAdapter(self.cfg)
        self.observations = CombinedObservationBuilder(self.cfg)
        self.test_action = None
        self._virtual_pose_announced = False
        self._mocap_pose_announced = False
        self._mocap_last_position = np.zeros(3, dtype=np.float64)
        self._mocap_velocity = np.zeros(3, dtype=np.float64)
        self._mocap_last_time = 0.0
        self._mocap_have_previous = False
        self._virtual_position = np.zeros(3, dtype=np.float64)
        self._virtual_yaw = 0.0
        self._virtual_last_time = 0.0
        self.action_test = os.getenv("ATMO_RL_ACTION_TEST", "lift").strip().lower()
        self.action_test_duration = float(os.getenv("ATMO_RL_ACTION_TEST_DURATION", "5.0"))
        kill_test_passed = True
        if self.mode == "action_test":
            if self.action_test not in ACTION_NAMES:
                raise ValueError(f"ATMO_RL_ACTION_TEST must be one of {ACTION_NAMES}")
            magnitude = float(os.getenv("ATMO_RL_ACTION_MAGNITUDE", "0.1"))
            sign = os.getenv("ATMO_RL_ACTION_SIGN", "positive").strip().lower()
            baseline = float(os.getenv("ATMO_RL_ROTOR_BASELINE", "-0.8"))
            max_magnitude = 1.0 if self.action_test == "tilt" else 0.25
            if not 0.0 < magnitude <= max_magnitude:
                raise ValueError(f"ATMO_RL_ACTION_MAGNITUDE must be in (0, {max_magnitude}] " f"for {self.action_test}")
            if self.action_test == "tilt" and magnitude < 0.5:
                raise ValueError("Tilt actions below 0.5 round to zero; use magnitude 1.0")
            if sign not in {"positive", "negative"}:
                raise ValueError("ATMO_RL_ACTION_SIGN must be positive or negative")
            if not -1.0 <= baseline <= -0.5:
                raise ValueError("ATMO_RL_ROTOR_BASELINE must be in [-1.0, -0.5]")

            self.test_action = np.zeros(self.cfg.action_dim, dtype=np.float32)
            if self.action_test in ROTOR_ACTIONS:
                self.test_action[0] = baseline
            value = magnitude if sign == "positive" else -magnitude
            if self.action_test == "lift":
                self.test_action[0] = float(np.clip(baseline + value, -1.0, -0.5))
            else:
                self.test_action[ACTION_NAMES.index(self.action_test)] = value

        self._check_contract()

        self.policy = None
        self.policy_loaded = False
        # Ground runs the policy for real, so it must load it. Missing this
        # left self.policy as None while the POLICY_MODES branches below
        # dereferenced it, and the node died in __init__ with an
        # AttributeError before it ever reached spin().
        if self.mode in POLICY_MODES | {"shadow"}:
            self.policy = PolicyRunner(self.cfg)
            self.policy_loaded = self.policy.load()
            if self.policy_loaded and getattr(self.policy, "numpy_actor", None):
                # Log the provenance of the export so a stale .npz is
                # identifiable at the bench rather than trusted by filename.
                self.get_logger().info(self.policy.numpy_actor.describe())
            warning = getattr(self.policy, "warning", None)

            if warning:
                self.get_logger().error(warning)
                if self.mode in POLICY_MODES:
                    raise RuntimeError(warning)

        if self.policy_loaded:
            self.get_logger().info(f"Loaded RL policy from {self.cfg.policy_path}")
        elif self.mode in POLICY_MODES:
            # self.policy can still be None if a mode reaches here without
            # constructing one; report that plainly rather than raising an
            # AttributeError out of __init__.
            error = getattr(self.policy, "error", "no policy runner was created")
            self.get_logger().warn(
                f"RL policy is not active: {error}. " "Drop the .npz at that path or set ATMO_RL_POLICY_PATH."
            )
        self.get_logger().info(
            "ATMO RL hardware config: "
            f"mode={self.mode}, route={self.route}, "
            f"policy_hz={self.cfg.policy_hz:.1f}, publish_hz={self.cfg.publish_hz:.1f}, "
            f"obs_dim={self.cfg.observation_dim}, action_dim={self.cfg.action_dim}, "
            f"motor_alpha={self.adapter.motor_alpha:.3f}"
        )
        if self.route == "full":
            self.get_logger().info(
                "Combined hardware trajectory: DRIVE hold, TAKEOFF rise 1.0 m, "
                "FLIGHT hover, LANDING back to the anchored ground z, then DRIVE hold"
            )
        elif self.route == "takeoff":
            self.get_logger().info("Combined hardware trajectory: DRIVE hold, TAKEOFF rise 1.0 m, then FLIGHT hold")
        else:
            self.get_logger().info("Combined hardware trajectory: FLIGHT hold, LANDING to z=0.200 m, then DRIVE hold")
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
        self.policy_action_publisher = None
        self.drive_vel_publisher = None
        self.manual_override_publisher = None
        if self.mode not in {"shadow", "sensor_test"}:
            self.vehicle_command_publisher = self.create_publisher(
                VehicleCommand, px4_topics.topic("vehicle_command"), QUEUE_SIZE
            )
            self.offboard_control_mode_publisher = self.create_publisher(
                OffboardControlMode, px4_topics.topic("offboard_control_mode"), QUEUE_SIZE
            )
            if self.mode != "ground":
                # Ground mode never creates the rotor publisher at all. Cutting
                # it here rather than declining to publish means no later code
                # path, and no bug in one, can reach the rotors.
                self.actuator_motors_publisher = self.create_publisher(
                    ActuatorMotors, px4_topics.topic("actuator_motors"), QUEUE_SIZE
                )
            self.tilt_vel_publisher = self.create_publisher(TiltVel, "/tilt_vel", QUEUE_SIZE)
            self.drive_vel_publisher = self.create_publisher(DriveVel, "/drive_vel", QUEUE_SIZE)
            self.manual_override_publisher = self.create_publisher(Bool, "/atmo/rl/manual_override", QUEUE_SIZE)

        self.policy_action_publisher = self.create_publisher(Float32MultiArray, "/atmo/rl/policy_action", QUEUE_SIZE)
        self.observation_state_publisher = self.create_publisher(
            Float32MultiArray, "/atmo/rl/observation_state", QUEUE_SIZE
        )
        # The four per-rotor commands EXACTLY as handed to PX4: post-mixer,
        # post-gate, post-clip. /fmu/in/actuator_motors carries the same values
        # but only exists when the PX4 publisher is up, and its 12-wide NaN
        # padding makes it awkward to plot. This is the loggable mirror.
        self.actuator_commands_publisher = self.create_publisher(
            Float32MultiArray, "/atmo/rl/actuator_commands", QUEUE_SIZE
        )
        self.create_subscription(
            InputRc,
            px4_topics.resolve(self, "input_rc"),
            self.rc_listener_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            os.getenv("ATMO_MOCAP_POSE_TOPIC", "/vrpn_mocap/M4/pose"),
            self.mocap_callback,
            qos_profile_sensor_data,
        )
        # mocap_bridge's odometry. This is the pose source when
        # ATMO_RL_POSE_SOURCE=mocap; the raw pose above is logged only.
        self.create_subscription(
            Odometry,
            MOCAP_ODOM_TOPIC,
            self.mocap_odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry,
            px4_topics.resolve(self, "vehicle_visual_odometry"),
            self.visual_odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry,
            px4_topics.resolve(self, "vehicle_odometry"),
            self.vehicle_odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TiltAngle,
            px4_topics.resolve(self, "tilt_angle"),
            self.tilt_angle_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleCommandAck,
            px4_topics.resolve(self, "vehicle_command_ack"),
            self.vehicle_command_ack_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleControlMode,
            px4_topics.resolve(self, "vehicle_control_mode"),
            self.vehicle_control_mode_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            px4_topics.resolve(self, "vehicle_status"),
            self.vehicle_status_callback,
            qos_profile_sensor_data,
        )

        px4_topics.warn_missing(
            self,
            (
                "input_rc",
                "vehicle_odometry",
                "vehicle_status",
                "vehicle_control_mode",
                "vehicle_command_ack",
            ),
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
        self.rc_deadman_ok = False
        self.autostart = ACTION_AUTOSTART and self.mode == "action_test" and self.action_test not in ROTOR_ACTIONS
        if ACTION_AUTOSTART and self.action_test in ROTOR_ACTIONS:
            self.get_logger().warn("ATMO_RL_ACTION_AUTOSTART ignored: rotor tests keep the ratchet")
        self.test_phase = "ready" if self.autostart else "waiting_low"
        if self.autostart:
            self.get_logger().warn(
                "ACTION AUTOSTART: runs as soon as RC is live and the kill is " "released. Kill switch stops it."
            )
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
            self.shadow_log_path = Path(os.getenv("ATMO_RL_SHADOW_LOG", default_name)).expanduser()
            self.shadow_log_path.parent.mkdir(parents=True, exist_ok=True)
            self.shadow_log = self.shadow_log_path.open("w", encoding="utf-8", buffering=1)
            self.shadow_log.write(
                json.dumps(
                    {
                        "type": "metadata",
                        "route": self.route,
                        "observation_dim": self.cfg.observation_dim,
                        "action_dim": self.cfg.action_dim,
                        "policy_path": str(self.cfg.policy_path),
                        "policy_loaded": self.policy_loaded,
                    }
                )
                + "\n"
            )
            self.get_logger().warn(
                "SHADOW MODE: no command publishers were created; raise only the RL switch "
                f"to reset/start the fixed reference. Log: {self.shadow_log_path}"
            )
            if not self.policy_loaded:
                self.get_logger().warn(f"Policy output will be null in the shadow log: {self.policy.error}")

        self.timer = self.create_timer(self.cfg.publish_dt, self.timer_callback)
        # Runs regardless of mode: even shadow and the test modes decide what
        # they are allowed to do from these gates.
        self.rc_watchdog_timer = self.create_timer(0.1, self._rc_watchdog)

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
            if self.gates_released:
                self._clear_terminal_latch()
            else:
                self._terminal_handoff_timer()
            return

        requested = self.gates_requested
        if not requested:
            self._leave_rl_idle()
            return
        if self.mode in POLICY_MODES and not self.policy_loaded:
            self._warn_missing_policy()
            return
        if not self.observations.state_seen:
            self._warn_waiting_for_state()
            return
        if time.monotonic() - self.last_odometry_time > ODOMETRY_TIMEOUT_S:
            self._warn_waiting_for_state("PX4 odometry is stale; RL remains inactive")
            self._leave_rl_idle()
            return

        # GROUND MODE: never touch PX4. No offboard, no arming, no rotor
        # publication -- the rotors are cut at the source rather than zeroed,
        # so a bug in the policy output cannot reach them.
        if self.mode == "ground":
            if not self.rl_active:
                self._start_policy_session()
                if not self.rl_active:
                    return  # engagement blocked (telemetry gate); retry next tick
            self._publish_manual_override(False)
            self._update_command_if_due()
            if self.handoff_latched:
                self._terminal_handoff_timer()
                return
            self._publish_policy_command(include_rotors=False)
            return

        self._switch_to_offboard()
        if not self.offboard:
            return
        if not self.rl_active:
            self._start_policy_session()
            if not self.rl_active:
                return  # engagement blocked (telemetry gate); retry next tick

        self._publish_offboard_control_mode_direct_actuator()
        self._publish_manual_override(False)
        self._update_command_if_due()
        if self.handoff_latched:
            self._terminal_handoff_timer()
            return
        self._publish_policy_command(include_rotors=True)

    def _update_command_if_due(self):
        now = time.monotonic()
        if now - self.last_policy_time < self.cfg.policy_dt:
            return

        self._publish_observation_state()
        obs = self.observations.observation()
        transition = self.observations.last_debug.get("combined_transition", "none")
        if transition != "none":
            self.get_logger().info(
                f"Combined hardware transition: {transition}, " f"mode={self.observations.last_debug['combined_mode']}"
            )
        action = self.policy.action(obs)
        self.last_policy_time = now
        if action is None:
            return

        command = self.adapter.pre_physics_step(action)
        if self.policy_action_publisher is not None:
            msg = Float32MultiArray()
            msg.data = [float(v) for v in list(command.raw_action) + list(command.semantic_action)]
            self.policy_action_publisher.publish(msg)
        self.observations.set_tilt_angle(command.tilt_angle)
        self.observations.append_action(command.semantic_action)
        # On the single routes the first completed leg is terminal. On the
        # FULL route takeoff_to_flight is mid-mission (hover comes next) and
        # only landing_to_drive ends it.
        terminal = (
            {"landing_to_drive"}
            if self.route == "full"
            else {"takeoff_to_flight", "landing_to_drive"}
        )
        # ATMO_RL_TERMINAL_HANDOFF=0 keeps the session running through the
        # post-landing drive hold (like ground mode) instead of disarming at
        # the terminal transition. Bench use: the rotor thrust gate already
        # forces physical thrust to 0 in DRIVE, so the rotors idle while the
        # arm tucks. Flight default is unchanged: disarm on terminal.
        if (
            self.mode == "policy"
            and transition in terminal
            and os.getenv("ATMO_RL_TERMINAL_HANDOFF", "1") != "0"
        ):
            self._begin_terminal_handoff(transition)

    @property
    def gates_requested(self):
        """True when the operator is asking for output, under either scheme."""
        if OFFBOARD_GATE_ENABLED:
            return self.offboard_switch and self.rl_switch
        return self.rl_switch

    @property
    def gates_released(self):
        """True when every gate this airframe has is LOW."""
        if OFFBOARD_GATE_ENABLED:
            return not self.offboard_switch and not self.rl_switch
        return not self.rl_switch

    def _action_test_gates(self):
        """Transition predicates for the action-test ratchet.

        Two-gate airframes cycle the offboard and RL switches against each
        other, so no single switch position can start a motor.

        Airframes with only one gate switch (ATMO_RL_OFFBOARD_CHANNEL < 0)
        cycle the RL switch TWICE instead -- low, high, low, high. That keeps
        the property that actually matters: the sequence has to begin from
        LOW, so a switch left HIGH when the node starts cannot run anything,
        and a single flip is never enough.

        What is deliberately NOT conditional on this: every fail-closed path.
        rc_lost, rc_failsafe, implausible pulse width and the RC watchdog all
        drop `rl_switch`, and `raised` is false whenever `rl_switch` is false
        under either configuration. The deadman is unchanged.

        Returns (raised, lowered, half, abort).
        """
        if self.autostart:
            # Start condition is RC LIVENESS, not switch position; every
            # fail-closed path drops rc_deadman_ok and ends the test.
            ok = self.rc_deadman_ok
            return (ok, not ok, False, not ok)
        if OFFBOARD_GATE_ENABLED:
            return (
                self.offboard_switch and self.rl_switch,
                not self.offboard_switch and not self.rl_switch,
                self.offboard_switch and not self.rl_switch,
                not self.offboard_switch,
            )
        return (self.rl_switch, not self.rl_switch, not self.rl_switch, False)

    def _action_test_timer(self):
        # Autostart has no switch choreography: any time RC liveness is up
        # and the machine sits in a ratchet state (including after an early
        # abort while RC was still coming up), jump straight to ready.
        if (
            self.autostart
            and self.rc_deadman_ok
            and self.test_phase in ("waiting_low", "waiting_prepare", "waiting_trigger")
        ):
            self.test_phase = "ready"
        raised, lowered, half, abort = self._action_test_gates()
        both_high = raised
        if self.test_phase == "waiting_low":
            self._publish_safe_output()
            if lowered:
                self.test_phase = "waiting_prepare"
        elif self.test_phase == "waiting_prepare":
            self._publish_safe_output()
            if raised:
                self.test_phase = "waiting_trigger"
                self.get_logger().warn("Action test prepared; lower the RL switch")
        elif self.test_phase == "waiting_trigger":
            self._publish_safe_output()
            if half:
                self.test_phase = "ready"
                self.get_logger().warn("Action test ready; raise the RL switch to apply output")
            elif abort:
                self.test_phase = "waiting_low"
        elif self.test_phase == "ready":
            self._publish_safe_output()
            if raised:
                self.test_phase = "starting" if self.action_test in ROTOR_ACTIONS else "running"
                self.test_started_at = time.monotonic()
            elif abort:
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
            if lowered:
                self.test_phase = "waiting_prepare"

    def _finish_action_test(self, reason):
        self._publish_safe_output()
        if self.action_test in ROTOR_ACTIONS and self.px4_armed:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.offboard = False
        self.test_phase = "complete"
        self.get_logger().warn(f"Action test stopped ({reason}); cycle both gates low before another test")

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
        offboard_rc = self.raw_rc_values[OFFBOARD_CHANNEL] if OFFBOARD_CHANNEL < len(self.raw_rc_values) else None
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
            if not self.rl_active:
                return  # engagement blocked (telemetry gate); retry next tick

        now = time.monotonic()
        if now - self.last_policy_time < self.cfg.policy_dt:
            return
        self.last_policy_time = now
        self._publish_observation_state()
        observation = self.observations.observation()
        transition = self.observations.last_debug.get("combined_transition", "none")
        if transition != "none":
            self.get_logger().info(
                f"Combined shadow transition: {transition}, " f"mode={self.observations.last_debug['combined_mode']}"
            )
        action = self.policy.action(observation) if self.policy_loaded else None
        command = None
        if action is not None:
            self.adapter.set_tilt_angle(self.observations.tilt_angle)
            command = self.adapter.pre_physics_step(action)
            self.observations.append_action(command.semantic_action)
            msg = Float32MultiArray()
            msg.data = [float(value) for value in list(command.raw_action) + list(command.semantic_action)]
            self.policy_action_publisher.publish(msg)
            # Deliberately here and not in _publish_policy_command(): shadow and
            # sensor create no command publishers, so the PX4 path never runs
            # and the rotor commands would never be logged for exactly the
            # profile that exists to inspect them. This publishes what the
            # policy WOULD send, gated identically.
            self._publish_actuator_commands(command.rotors * self.observations.rotor_thrust_gate())

        self._write_shadow_sample(observation, action, command)

    def _publish_observation_state(self):
        """Publish the compact training-frame state used to build this observation."""
        w, x, y, z = (float(value) for value in self.observations.quat_wxyz)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        msg = Float32MultiArray()
        msg.data = [
            float(value)
            for value in (
                list(self.observations.position)
                + list(self.observations.quat_wxyz)
                + [roll, pitch, yaw]
                + list(self.observations.linear_velocity)
                + list(self.observations.angular_velocity_w)
                + [self.observations.tilt_angle]
            )
        ]
        self.observation_state_publisher.publish(msg)

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

    def _telemetry_matched(self):
        """Refuse engagement until every critical publisher is fully matched.

        Telemetry gate (2026-08-18 post-mortem): under the discovery server,
        endpoint matching crosses the WiFi even for same-host nodes, and a
        half-completed match leaves a RELIABLE publisher silently sending to
        nobody (bags 202503/205315: recorder discovered but never matched ->
        0/partial actuator_motors captured; bag 204231 lost entirely).
        get_subscription_count() reflects COMPLETED matches, so require every
        critical publisher to see all its consumers before the policy may
        engage.  actuator_motors: agent + rosbag = 2; tilt_vel: tilt node +
        rosbag = 2; policy_action: rosbag = 1 (env-overridable).
        """
        required = int(os.getenv("ATMO_RL_MIN_MATCHED_SUBS", "2"))
        checks = []
        if self.actuator_motors_publisher is not None:
            checks.append(("actuator_motors", self.actuator_motors_publisher.get_subscription_count(), required))
        if self.tilt_vel_publisher is not None:
            checks.append(("tilt_vel", self.tilt_vel_publisher.get_subscription_count(), required))
        if self.policy_action_publisher is not None:
            checks.append(
                (
                    "policy_action",
                    self.policy_action_publisher.get_subscription_count(),
                    int(os.getenv("ATMO_RL_MIN_MATCHED_ACTION_SUBS", "1")),
                )
            )
        unmatched = [(n, c, r) for n, c, r in checks if c < r]
        if unmatched:
            self.get_logger().error(
                "ENGAGEMENT BLOCKED: publishers not fully matched %s (matched < "
                "required; is the recorder up? did discovery complete?). Set "
                "ATMO_RL_MIN_MATCHED_SUBS to override." % (unmatched,)
            )
            return False
        return True

    def _start_policy_session(self):
        if not self._telemetry_matched():
            return  # refuse this tick; caller retries while gates stay up
        measured_tilt = float(self.observations.tilt_angle)
        self.rl_active = True
        self.adapter = LandingActionAdapter(self.cfg)
        self.observations.reset_policy_context()
        self.observations.anchor_fixed_vertical_route()
        self.adapter.set_tilt_angle(measured_tilt)
        self.observations.set_tilt_angle(measured_tilt)
        self.last_policy_time = 0.0
        if self.shadow_log is not None:
            self.shadow_log.write(
                json.dumps(
                    {
                        "type": "session_start",
                        "wall_time_s": time.time(),
                        "training_start_position": self.observations.position.tolist(),
                        "tilt_angle_rad": measured_tilt,
                        "route": self.route,
                    }
                )
                + "\n"
            )
        self.get_logger().info(f"{self.mode} session engaged; starting fresh fixed reference and action history")

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

    def _publish_policy_command(self, include_rotors: bool) -> None:
        command = self.adapter.last_command
        if include_rotors:
            rotor_gate = self.observations.rotor_thrust_gate()
            self._publish_actuator_motors(command.rotors * rotor_gate)
        wheel_gate = self.observations.wheel_speed_gate()
        self._publish_tilt_vel(command.tilt_velocity)
        self._publish_drive_vel(
            command.drive_speed * wheel_gate,
            command.turn_speed * wheel_gate,
        )

    def _publish_actuator_commands(self, rotors):
        """Mirror the per-rotor commands onto a plain ROS topic for logging.

        Clipped identically to `_publish_actuator_motors`, so what is logged is
        what PX4 was given -- not the pre-clip value.
        """
        msg = Float32MultiArray()
        msg.data = [float(np.clip(rotors[idx], 0.0, 1.0)) for idx in range(4)]
        self.actuator_commands_publisher.publish(msg)

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
            self.get_logger().warn(f"Waiting for policy file. Expected: {self.cfg.policy_path}")

    def _warn_waiting_for_state(self, message="Waiting for PX4 odometry before arming RL"):
        now = time.monotonic()
        if now - self.last_warn_time > 5.0:
            self.last_warn_time = now
            self.get_logger().warn(message)

    def _timestamp_us(self):
        return int(Clock().now().nanoseconds / 1000)

    def _virtual_odometry(self):
        """Dead-reckon a pose from the commands we are issuing.

        A FROZEN pose is not good enough for anything that drives. The m4
        bring-up measured this directly (2026-08-13): a policy commanding
        wheels into a world that never moves sees no yaw response and winds
        its yaw differential to the rail. The loop has to close, even if only
        in software.

        So integrate an ideal no-slip skid-steer from the drive and turn
        commands the adapter last produced:

            v       = drive_speed * VIRTUAL_MAX_SPEED
            yaw_dot = turn_speed  * VIRTUAL_MAX_YAW_RATE * VIRTUAL_YAW_DAMPING

        Yaw is deliberately damped, as in the m4 node: command a differential
        and the yaw unwinds at a fraction of the ideal rate, which keeps the
        loop stable without pretending the kinematics are real.

        THE KINEMATICS HERE ARE IDEAL. No slip, no friction, no lag. A stable
        run says the policy, reference, observation packing and wheel mapping
        close a loop. It says NOTHING about ground dynamics.

        DO NOT RUN THIS WITH THE WHEELS ON THE FLOOR. The vehicle would really
        move while this reports the ideal pose, and the two diverge from the
        first step.
        """
        now = time.monotonic()
        dt = now - self._virtual_last_time if self._virtual_last_time else 0.0
        self._virtual_last_time = now
        # Clamp dt so a stall cannot teleport the virtual vehicle.
        dt = min(max(dt, 0.0), 0.1)

        command = getattr(self.adapter, "last_command", None)
        drive = float(getattr(command, "drive_speed", 0.0) or 0.0)
        turn = float(getattr(command, "turn_speed", 0.0) or 0.0)

        v = drive * VIRTUAL_MAX_SPEED
        yaw_rate = turn * VIRTUAL_MAX_YAW_RATE * VIRTUAL_YAW_DAMPING
        self._virtual_yaw += yaw_rate * dt
        self._virtual_position[0] += v * math.cos(self._virtual_yaw) * dt
        self._virtual_position[1] += v * math.sin(self._virtual_yaw) * dt
        # Ground running: altitude is whatever the vehicle is sitting at.
        self._virtual_position[2] = VIRTUAL_GROUND_Z
        velocity = np.array(
            (v * math.cos(self._virtual_yaw), v * math.sin(self._virtual_yaw), 0.0),
            dtype=np.float64,
        )
        return self._virtual_position.copy(), velocity

    def emergency_stop(self, reason):
        """Command zero everywhere, as fast and as often as possible.

        Called on Ctrl-C, on shutdown, and whenever the kill switch engages.
        Publishes repeatedly because a single message can be lost and a
        RoboClaw holds its last command forever -- a dropped zero is a motor
        that never stops. Everything here is guarded: this must run to the end
        even if half the node is already torn down.
        """
        try:
            self.get_logger().error("EMERGENCY STOP (%s): commanding zero" % reason)
        except Exception:  # noqa: BLE001
            pass
        self.offboard_switch = False
        self.rl_switch = False
        self.rl_active = False
        for _ in range(3):
            for publish in (
                lambda: self._publish_tilt_vel(0.0),
                lambda: self._publish_drive_vel(0.0, 0.0),
                lambda: self._publish_manual_override(True),
            ):
                try:
                    publish()
                except Exception:  # noqa: BLE001
                    pass
            try:
                if self.actuator_motors_publisher is not None and self.px4_offboard:
                    self._publish_actuator_motors(np.zeros(4, dtype=np.float32))
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.02)

    def rc_listener_callback(self, msg):
        self._mark_sensor("rc")
        self.raw_rc_values = list(msg.values)
        # PX4's kill cuts the ROTORS. It does not reach the tilt and drive
        # RoboClaws at all, so it has to be carried across here: kill engaged
        # means stop commanding them, immediately, in every mode.
        if kill_engaged(msg.values, msg.rc_lost, msg.rc_failsafe):
            if self.rl_switch or self.offboard_switch or self.rl_active:
                self.emergency_stop("kill switch engaged")
            self.offboard_switch = False
            self.rl_switch = False
            self.rc_deadman_ok = False
            return
        # Fail closed. These gates are a deadman, so anything meaning "we do
        # not currently know where the switch is" must read LOW rather than
        # leaving the last value we happened to see latched high.
        if msg.rc_lost or msg.rc_failsafe:
            if self.offboard_switch or self.rl_switch:
                self.get_logger().warn("RC lost/failsafe reported by PX4; dropping both gates")
            self.offboard_switch = False
            self.rl_switch = False
            self.rc_deadman_ok = False
            return
        self.offboard_switch = self._channel_high(msg, OFFBOARD_CHANNEL)
        self.rl_switch = self._channel_high(msg, RL_CHANNEL)
        self.rc_deadman_ok = True

    def _channel_high(self, msg, channel):
        if channel < 0 or channel >= len(msg.values):
            return False
        value = int(msg.values[channel])
        # A channel reading below any plausible pulse width is a dead or
        # unmapped channel, not a switch position. Treat it as LOW.
        if value < RC_PLAUSIBLE_MIN_US:
            return False
        return value >= RC_MAX - RC_MARGIN

    def _rc_watchdog(self):
        """Expire the gates when the RC stream stops.

        Without this the switches keep whatever value arrived last, so pulling
        the transmitter's power leaves the companion believing both gates are
        still raised. PX4's own kill is independent and covers the rotors, but
        NOT the tilt and drive RoboClaws -- those are commanded only from here,
        so this is the only thing that stops them on RC loss.
        """
        stamp = self.sensor_times.get("rc")
        if stamp is None:
            return
        if time.monotonic() - stamp <= RC_TIMEOUT_S:
            return
        if self.offboard_switch or self.rl_switch:
            self.get_logger().error("No RC for %.1f s; dropping both gates" % RC_TIMEOUT_S)
        self.offboard_switch = False
        self.rl_switch = False
        # Autostart liveness expires with the stream too -- a dead agent must
        # stop an autostarted test exactly like a dead transmitter does.
        self.rc_deadman_ok = False

    def vehicle_odometry_callback(self, msg):
        self._mark_sensor("px4_odometry")
        values = np.asarray(
            (*msg.position, *msg.q, *msg.velocity, *msg.angular_velocity),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(values)):
            self._warn_waiting_for_state("Ignoring non-finite PX4 odometry")
            return
        self.raw_px4_angular_velocity[:] = msg.angular_velocity
        if POSE_SOURCE == "mocap":
            # Mocap owns the state. Keep the gyro (read above) and drop the
            # rest -- letting the EKF also write here would race the mocap.
            return
        position = np.asarray(msg.position, dtype=np.float64)
        velocity = np.asarray(msg.velocity, dtype=np.float64)
        if VIRTUAL_POSE:
            position, velocity = self._virtual_odometry()
            # Ground running with no position reference. PX4's EKF has GPS
            # denied and no mocap, so it drifts without bound -- measured on
            # this vehicle 2026-08-14, stationary on the bench: position
            # (-4428, -699, -72) m and velocity 24 m/s. Those are FINITE, so
            # nothing upstream rejects them, and they would go straight into
            # the observation and out as wheel and tilt commands.
            #
            # Attitude and angular velocity come from the IMU and are sound,
            # so keep them. Substitute a stationary origin for the two
            # channels the EKF cannot know, which is the honest statement of
            # what is actually observable here.
            if not self._virtual_pose_announced:
                self._virtual_pose_announced = True
                self.get_logger().warn(
                    "VIRTUAL ODOMETRY ACTIVE: PX4 position/velocity replaced "
                    "with an ideal dead-reckoned skid-steer from the commands "
                    "being issued. Attitude is still real. Kinematics are "
                    "IDEAL -- no slip, no friction. This validates that the "
                    "loop closes, NOT ground dynamics. DO NOT RUN WITH THE "
                    "WHEELS ON THE FLOOR: the vehicle would really move while "
                    "this reports the ideal pose, and they diverge at once."
                )
        self.raw_px4_position[:] = position
        self.raw_px4_quaternion[:] = msg.q
        self.raw_px4_velocity[:] = velocity
        self.raw_px4_angular_velocity[:] = msg.angular_velocity
        self.observations.update_px4_state(
            position,
            msg.q,
            velocity,
            msg.angular_velocity,
        )
        self.last_odometry_time = time.monotonic()

    def mocap_callback(self, msg):
        """Raw VRPN pose. Recorded for the log; NOT the pose source.

        With POSE_SOURCE=mocap the state comes from mocap_bridge's odometry
        (see mocap_odom_callback). This stays so the raw stream is still in
        the rosbag alongside the bridge's interpretation of it -- when a frame
        convention turns out to be wrong, the difference between these two is
        what tells you.
        """
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

    def mocap_odom_callback(self, msg):
        """mocap_bridge odometry -> the observation, with no EKF in between."""
        if POSE_SOURCE != "mocap":
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = np.array((p.x, p.y, p.z), dtype=np.float64)
        quaternion = np.array((q.w, q.x, q.y, q.z), dtype=np.float64)
        # The bridge publishes twist in the BODY frame (matching the Gazebo
        # convention); the observation wants world. Rotate it rather than
        # assuming they agree -- this is exactly the class of mismatch that
        # produced the m4 hip-frame inversion.
        linear_body = np.array(
            (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z), dtype=np.float64
        )
        angular_body = np.array(
            (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z), dtype=np.float64
        )
        if not (np.all(np.isfinite(position)) and np.all(np.isfinite(quaternion)) and np.all(np.isfinite(linear_body))):
            self._warn_waiting_for_state("Ignoring non-finite mocap odometry")
            return
        velocity_world = quat_wxyz_to_rotmat(quaternion) @ linear_body

        # The bridge already publishes the training convention -- z-up world,
        # body-frame twist -- so no frame conversion belongs here. Calling
        # update_px4_state() instead (as this did until 2026-08-20) applied
        # PX4's NED->ENU and FRD->FLU on top of it: height inverted, x swapped
        # with y, pitch and yaw senses flipped. ANALYSIS_HANDOFF S.13.
        self.observations.update_training_frame_state(position, quaternion, velocity_world, angular_body)
        self.last_odometry_time = time.monotonic()
        self._mark_sensor("mocap_odom")
        if not self._mocap_pose_announced:
            self._mocap_pose_announced = True
            self.get_logger().warn(
                "POSE SOURCE = MOCAP: state comes from %s via mocap_bridge. "
                "PX4's EKF is NOT in the loop. Frame conventions are the "
                "bridge's -- verify them by moving the vehicle in each axis "
                "before trusting a run." % MOCAP_ODOM_TOPIC
            )

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
    # rclpy.spin() raises KeyboardInterrupt on Ctrl-C, and without this every
    # line after it is skipped -- the node would die with the last non-zero
    # tilt and drive command still in flight, and a RoboClaw latches its last
    # command indefinitely. Stopping is not cleanup here; it is the stop.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
