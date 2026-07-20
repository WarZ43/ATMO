"""IsaacLab stage-1 landing contract used by the ATMO ROS nodes.

The policy contract is deliberately small and explicit.  It mirrors the active
ATMO vehicle spec: 7 mapped actions, 15 delayed state-and-tilt frames, 25 mapped
action frames, and 64 current task/vehicle values (524 values total).
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
except Exception:  # pragma: no cover - deployment images may not have torch yet.
    torch = None
    nn = None


POLICY_NAME = "atmo_landing_stage1"
DEFAULT_POLICY_PATH = Path(__file__).resolve().parent / "policies" / f"{POLICY_NAME}.pth"

_NED_TO_ENU = np.array(((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)), dtype=np.float32)
_FRD_TO_FLU = np.diag((1.0, -1.0, -1.0)).astype(np.float32)

_PHYSICAL_ROTOR_CONTROL_MIX = np.array(
    ((1.0, 1.0, -1.0), (-1.0, -1.0, -1.0), (-1.0, 1.0, 1.0), (1.0, -1.0, 1.0)),
    dtype=np.float32,
)

# Link origins copied from the active M4 URDF/SDF.  The arm joints rotate about
# body X, so these values reproduce the IsaacLab rotor geometry as tilt changes.
_ARM_ORIGINS_B = np.array(
    ((-0.00013437, 0.068076, 0.094), (-0.00013437, -0.067924, 0.094)),
    dtype=np.float32,
)
# IsaacLab's wrench geometry uses body COM positions.  The active M4 URDF
# places the base-link COM away from its link frame origin.
_BASE_COM_OFFSET_B = np.array((-0.018276, 0.00049378, 0.068096), dtype=np.float32)
_ROTOR_ARM_INDEX = np.array((1, 0, 0, 1), dtype=np.int64)  # rotor0..rotor3
_ROTOR_OFFSETS_ARM = np.array(
    ((0.16491, -0.13673, 0.069563), (-0.16509, 0.13673, 0.069563),
     (0.16491, 0.13673, 0.069563), (-0.16509, -0.13673, 0.069562)),
    dtype=np.float32,
)
_ROTOR_TILT_SIGN = np.array((1.0, -1.0, -1.0, 1.0), dtype=np.float32)
_ROTOR_SPIN_DIRECTION = np.array((-1.0, -1.0, 1.0, 1.0), dtype=np.float32)


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value in (None, ""):
        return float(default)
    try:
        return float(value)
    except ValueError:
        return float(default)


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value in (None, ""):
        return int(default)
    try:
        return int(value)
    except ValueError:
        return int(default)


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value in (None, ""):
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return default if value in (None, "") else value


def _env_path(name: str, default: Path) -> Path:
    value = os.getenv(name)
    return Path(value).expanduser() if value else default


def quat_wxyz_to_rotmat(quat: Iterable[float]) -> np.ndarray:
    q = np.asarray(list(quat), dtype=np.float32)
    if q.shape != (4,):
        return np.eye(3, dtype=np.float32)
    norm = float(np.linalg.norm(q))
    if norm < 1e-6:
        return np.eye(3, dtype=np.float32)
    w, x, y, z = q / norm
    return np.array(
        ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
         (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
         (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))),
        dtype=np.float32,
    )


def rotmat_to_quat_wxyz(rotmat: np.ndarray) -> np.ndarray:
    r = np.asarray(rotmat, dtype=np.float32).reshape(3, 3)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        values = ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
        quat = np.array((0.25 * s, *values), dtype=np.float32)
    else:
        diagonal = np.diag(r)
        index = int(np.argmax(diagonal))
        if index == 0:
            s = math.sqrt(max(1.0 + r[0, 0] - r[1, 1] - r[2, 2], 1e-12)) * 2.0
            quat = np.array(((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s), dtype=np.float32)
        elif index == 1:
            s = math.sqrt(max(1.0 + r[1, 1] - r[0, 0] - r[2, 2], 1e-12)) * 2.0
            quat = np.array(((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s), dtype=np.float32)
        else:
            s = math.sqrt(max(1.0 + r[2, 2] - r[0, 0] - r[1, 1], 1e-12)) * 2.0
            quat = np.array(((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s), dtype=np.float32)
    norm = float(np.linalg.norm(quat))
    return quat / norm if norm > 1e-6 else np.array((1.0, 0.0, 0.0, 0.0), dtype=np.float32)


def _px4_ned_to_training_vec(vec: Iterable[float]) -> np.ndarray:
    return _NED_TO_ENU @ np.asarray(list(vec), dtype=np.float32)[:3]


def _px4_frd_to_training_body_vec(vec: Iterable[float]) -> np.ndarray:
    return _FRD_TO_FLU @ np.asarray(list(vec), dtype=np.float32)[:3]


def _px4_attitude_to_training_quat(quat_wxyz: Iterable[float]) -> np.ndarray:
    rot_ned_frd = quat_wxyz_to_rotmat(quat_wxyz)
    return rotmat_to_quat_wxyz(_NED_TO_ENU @ rot_ned_frd @ _FRD_TO_FLU)


def _action_vector(action: np.ndarray, dim: int) -> np.ndarray:
    out = np.zeros(dim, dtype=np.float32)
    source = np.asarray(action, dtype=np.float32).reshape(-1)
    out[: min(dim, source.size)] = source[:dim]
    return np.nan_to_num(out, nan=0.0, posinf=1.0, neginf=-1.0)


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class LandingStage1Config:
    policy_path: Path = _env_path("ATMO_RL_POLICY_PATH", DEFAULT_POLICY_PATH)
    policy_hz: float = _env_float("ATMO_RL_POLICY_HZ", 50.0)
    publish_hz: float = _env_float("ATMO_RL_PUBLISH_HZ", 143.0)
    target_x: float = _env_float("ATMO_RL_TARGET_X", 0.0)
    target_y: float = _env_float("ATMO_RL_TARGET_Y", 0.0)
    target_z: float = _env_float("ATMO_RL_TARGET_Z", 0.24)
    trajectory_duration_scale: float = 1.875
    vertical_reference_speed_min: float = 0.4
    vertical_reference_speed_max: float = 0.7
    xy_reference_max_speed: float = 1.0
    xy_reference_min_duration_s: float = 0.75
    initial_virtual_xy_range: float = 15.0
    initial_virtual_z_range: Tuple[float, float] = (0.8, 1.2)
    randomize_reset: bool = _env_bool("ATMO_RL_RANDOMIZE_RESET", False)
    randomize_motor_dynamics: bool = _env_bool("ATMO_RL_RANDOMIZE_MOTOR_DYNAMICS", True)
    observation_noise: bool = _env_bool("ATMO_RL_OBSERVATION_NOISE", True)
    observation_delay_min_steps: int = _env_int("ATMO_RL_OBSERVATION_DELAY_MIN_STEPS", 0)
    observation_delay_max_steps: int = _env_int("ATMO_RL_OBSERVATION_DELAY_MAX_STEPS", 1)
    random_seed: int = _env_int("ATMO_RL_RANDOM_SEED", 42)
    action_dim: int = 7
    history_obs_dim: int = 19
    current_obs_dim: int = 64
    observation_history_length: int = 15
    action_history_length: int = 25
    motor_tau_min_s: float = 0.125
    motor_tau_max_s: float = 0.175
    neutral_rotor: float = 0.5
    max_tilt_velocity: float = math.pi / 8.0
    tilt_action_direction: float = 1.0
    tilt_lower: float = 0.0
    tilt_upper: float = math.pi / 2.0
    wheel_effort_limit: float = 1.0
    forward_yaw_offset: float = math.pi
    rotor_kT: float = 28.15
    rotor_kM: float = 0.018
    rotor_mix_scale: Tuple[float, float, float] = (0.5, 0.5, 0.5)
    wrench_allocation_damping: float = 1e-6
    wrench_allocation_clip: float = 100.0
    pos_noise_scale: float = 0.005
    rot_noise_scale: float = 0.005
    lin_vel_noise_scale: float = 0.035
    ang_vel_noise_scale: float = 0.035
    tilt_noise_scale: float = 0.018
    virtual_observation_offset: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    reference_start_vx: float = _env_float("ATMO_RL_REFERENCE_START_VX", 0.0)
    reference_start_vy: float = _env_float("ATMO_RL_REFERENCE_START_VY", 0.0)
    reference_end_vx: float = _env_float("ATMO_RL_REFERENCE_END_VX", 0.0)
    reference_end_vy: float = _env_float("ATMO_RL_REFERENCE_END_VY", 0.0)
    reference_vertical_speed: float = _env_float("ATMO_RL_REFERENCE_VERTICAL_SPEED", 0.55)

    @property
    def policy_dt(self) -> float:
        return 1.0 / self.policy_hz

    @property
    def publish_dt(self) -> float:
        return 1.0 / self.publish_hz

    @property
    def observation_dim(self) -> int:
        return self.observation_history_length * self.history_obs_dim + self.action_history_length * self.action_dim + self.current_obs_dim

    @property
    def target_position(self) -> np.ndarray:
        return np.array((self.target_x, self.target_y, self.target_z), dtype=np.float32)

    @property
    def nominal_total_kT(self) -> float:
        return 4.0 * self.rotor_kT


@dataclass
class ActuatorCommand:
    rotors: np.ndarray
    rotors_unfiltered: np.ndarray
    tilt_angle: float
    tilt_velocity: float
    wheel_efforts: np.ndarray
    drive_speed: float
    turn_speed: float
    semantic_action: np.ndarray
    raw_action: np.ndarray


class LandingActionAdapter:
    """Exact stage-1 action mapping from vehicle_adapters.py."""

    def __init__(self, cfg: LandingStage1Config):
        self.cfg = cfg
        self.filtered_rotors = np.full(4, cfg.neutral_rotor, dtype=np.float32)
        self.tilt_angle = 0.0
        self.rng = np.random.default_rng(cfg.random_seed)
        self.motor_tau = cfg.motor_tau_min_s
        self.motor_alpha = 1.0
        self._last_command: Optional[ActuatorCommand] = None
        self.reset()

    def reset(self) -> None:
        self.filtered_rotors.fill(float(np.clip(self.cfg.neutral_rotor, 0.0, 1.0)))
        if self.cfg.randomize_motor_dynamics:
            self.motor_tau = float(self.rng.uniform(self.cfg.motor_tau_min_s, self.cfg.motor_tau_max_s))
        else:
            self.motor_tau = 0.15
        self.motor_alpha = 1.0 - math.exp(-self.cfg.policy_dt / max(self.motor_tau, 1e-6))
        self.tilt_angle = float(np.clip(self.tilt_angle, self.cfg.tilt_lower, self.cfg.tilt_upper))
        self._last_command = None

    @property
    def last_command(self) -> ActuatorCommand:
        if self._last_command is not None:
            return self._last_command
        semantic = np.zeros(self.cfg.action_dim, dtype=np.float32)
        semantic[0] = float(np.mean(self.filtered_rotors))
        raw = semantic.copy()
        raw[0] = 2.0 * semantic[0] - 1.0
        return ActuatorCommand(
            rotors=self.filtered_rotors.copy(),
            rotors_unfiltered=self.filtered_rotors.copy(),
            tilt_angle=self.tilt_angle,
            tilt_velocity=0.0,
            wheel_efforts=np.zeros(4, dtype=np.float32),
            drive_speed=0.0,
            turn_speed=0.0,
            semantic_action=semantic,
            raw_action=raw,
        )

    @last_command.setter
    def last_command(self, command: ActuatorCommand) -> None:
        self._last_command = command

    def set_tilt_angle(self, tilt_angle: float) -> None:
        if math.isfinite(tilt_angle):
            self.tilt_angle = float(np.clip(tilt_angle, self.cfg.tilt_lower, self.cfg.tilt_upper))

    def pre_physics_step(self, policy_action: np.ndarray) -> ActuatorCommand:
        raw = np.clip(_action_vector(policy_action, self.cfg.action_dim), -1.0, 1.0)
        semantic = raw.copy()
        semantic[0] = 0.5 * (raw[0] + 1.0)
        rotor_matrix = np.vstack((
            np.ones(4, dtype=np.float32),
            self.cfg.rotor_mix_scale[0] * _PHYSICAL_ROTOR_CONTROL_MIX[:, 0],
            self.cfg.rotor_mix_scale[1] * _PHYSICAL_ROTOR_CONTROL_MIX[:, 1],
            self.cfg.rotor_mix_scale[2] * _PHYSICAL_ROTOR_CONTROL_MIX[:, 2],
        ))
        normalized_rotors = np.clip(np.matmul(rotor_matrix.T, semantic[:4]), 0.0, 1.0).astype(np.float32)
        self.filtered_rotors = self.motor_alpha * normalized_rotors + (1.0 - self.motor_alpha) * self.filtered_rotors

        tilt_action = float(np.round(raw[4]))
        tilt_velocity = self.cfg.tilt_action_direction * self.cfg.max_tilt_velocity * tilt_action
        requested_tilt = self.tilt_angle + tilt_velocity * self.cfg.policy_dt
        if self.cfg.tilt_action_direction < 0.0 and requested_tilt > self.tilt_angle:
            # IsaacLab takeoff untucks toward hover and never asks the morph
            # joint to retuck during the flight portion of the episode.
            tilt_velocity = 0.0
            tilt_action = 0.0
            requested_tilt = self.tilt_angle
        if requested_tilt < self.cfg.tilt_lower or requested_tilt > self.cfg.tilt_upper:
            tilt_velocity = 0.0
            tilt_action = 0.0
        self.tilt_angle = float(np.clip(self.tilt_angle + tilt_velocity * self.cfg.policy_dt, self.cfg.tilt_lower, self.cfg.tilt_upper))
        semantic[4] = tilt_action

        drive = float(raw[5])
        turn = float(raw[6])
        # Isaac joint order is [wheel1, wheel3, wheel2, wheel0]. Its
        # differential mix is [drive-turn, drive+turn, drive-turn, drive+turn];
        # the mirrored right-side axes invert wheel3/wheel0 in PX4 channel
        # order [wheel0, wheel1, wheel2, wheel3].
        wheel_efforts = self.cfg.wheel_effort_limit * np.clip(
            np.array((-(drive + turn), drive - turn, drive - turn, -(drive + turn)), dtype=np.float32),
            -1.0,
            1.0,
        )
        command = ActuatorCommand(
            rotors=np.clip(self.filtered_rotors.copy(), 0.0, 1.0),
            rotors_unfiltered=normalized_rotors,
            tilt_angle=self.tilt_angle,
            tilt_velocity=tilt_velocity,
            wheel_efforts=wheel_efforts,
            drive_speed=float(np.clip(-raw[5], -1.0, 1.0)),
            turn_speed=float(np.clip(raw[6], -1.0, 1.0)),
            semantic_action=semantic,
            raw_action=raw,
        )
        self._last_command = command
        return command



def _rotate_x(vector: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array((vector[0], c * vector[1] - s * vector[2], s * vector[1] + c * vector[2]), dtype=np.float32)


class LandingObservationBuilder:
    """Build the exact 524-value stage-1 observation."""

    def __init__(self, cfg: LandingStage1Config, fixed_trajectory: bool = False):
        self.cfg = cfg
        self.fixed_trajectory = fixed_trajectory
        self.position = np.zeros(3, dtype=np.float32)
        self.quat_wxyz = np.array((1.0, 0.0, 0.0, 0.0), dtype=np.float32)
        self.linear_velocity = np.zeros(3, dtype=np.float32)
        self.angular_velocity_w = np.zeros(3, dtype=np.float32)
        self.tilt_angle = 0.0
        self.state_seen = False
        self.reference_initialized = False
        self.reference_start_position = np.zeros(3, dtype=np.float32)
        self.reference_target_position = cfg.target_position.copy()
        self.reference_start_velocity = np.zeros(3, dtype=np.float32)
        self.reference_end_velocity = np.zeros(3, dtype=np.float32)
        self.reference_duration = cfg.xy_reference_min_duration_s
        self.reference_accel_duration = 0.0
        self.reference_decel_duration = 0.0
        self.reference_cruise_duration = 0.0
        self.reference_cruise_velocity = np.zeros(3, dtype=np.float32)
        self.reference_accel_end = np.zeros(3, dtype=np.float32)
        self.reference_decel_start = np.zeros(3, dtype=np.float32)
        self.reference_heading = 0.0
        self.reference_start_time = time.monotonic()
        self.rng = np.random.default_rng(cfg.random_seed)
        self.observation_delay_steps = cfg.observation_delay_min_steps
        self.observation_buffer = np.zeros((cfg.observation_history_length + cfg.observation_delay_max_steps + 1, cfg.history_obs_dim), dtype=np.float32)
        self.current_buffer = np.zeros((cfg.observation_history_length + cfg.observation_delay_max_steps + 1, cfg.current_obs_dim), dtype=np.float32)
        self.observation_history_index = 0
        self.observation_history_initialized = False
        self.action_history = np.full((cfg.action_history_length, cfg.action_dim), -1.0, dtype=np.float32)
        self.last_debug: Dict[str, Any] = {}

    def update_px4_state(self, position, quat_wxyz, linear_velocity, angular_velocity) -> None:
        self.position = _px4_ned_to_training_vec(position)
        self.quat_wxyz = _px4_attitude_to_training_quat(quat_wxyz)
        self.linear_velocity = _px4_ned_to_training_vec(linear_velocity)
        angular_velocity_b = _px4_frd_to_training_body_vec(angular_velocity)
        self.angular_velocity_w = quat_wxyz_to_rotmat(self.quat_wxyz) @ angular_velocity_b
        self.state_seen = True
        if not self.reference_initialized:
            self._reset_reference()

    def set_tilt_angle(self, tilt_angle: float) -> None:
        if math.isfinite(tilt_angle):
            self.tilt_angle = float(np.clip(tilt_angle, self.cfg.tilt_lower, self.cfg.tilt_upper))

    def reset_policy_context(self) -> None:
        self.reference_initialized = False
        self.tilt_angle = 0.0
        self.action_history.fill(-1.0)
        self.observation_history_initialized = False
        self.last_debug = {}
        self._reset_reference()

    def append_action(self, semantic_action: np.ndarray) -> None:
        self.action_history[1:] = self.action_history[:-1].copy()
        self.action_history[0] = np.asarray(semantic_action, dtype=np.float32)[: self.cfg.action_dim]

    def observation(self) -> np.ndarray:
        history_frame = self._history_observation()
        current_frame = self._current_observation()
        if not self.observation_history_initialized:
            self.observation_buffer[:] = history_frame
            self.current_buffer[:] = current_frame
            self.observation_history_initialized = True
        self.observation_history_index = (self.observation_history_index - 1) % self.observation_buffer.shape[0]
        self.observation_buffer[self.observation_history_index] = history_frame
        self.current_buffer[self.observation_history_index] = current_frame

        history_indices = (
            self.observation_history_index
            + 1
            + self.observation_delay_steps
            + np.arange(self.cfg.observation_history_length)
        ) % self.observation_buffer.shape[0]
        current_index = (self.observation_history_index + self.observation_delay_steps) % self.observation_buffer.shape[0]
        observation = np.concatenate((self.observation_buffer[history_indices].reshape(-1), self.action_history.reshape(-1), self.current_buffer[current_index])).astype(np.float32)
        observation = np.clip(np.nan_to_num(observation, nan=0.0, posinf=100.0, neginf=-100.0), -100.0, 100.0)
        self.last_debug["obs_norm"] = float(np.linalg.norm(observation))
        self.last_debug["obs_min"] = float(np.min(observation))
        self.last_debug["obs_max"] = float(np.max(observation))
        return observation

    def _noise(self, value: np.ndarray, scale: float) -> np.ndarray:
        if not self.cfg.observation_noise or scale <= 0.0:
            return value.astype(np.float32, copy=True)
        return (value + self.rng.uniform(-scale, scale, value.shape)).astype(np.float32)

    def _history_observation(self) -> np.ndarray:
        rotmat = quat_wxyz_to_rotmat(self.quat_wxyz).reshape(-1)
        return np.concatenate((
            self._noise(self.position + self.cfg.virtual_observation_offset, self.cfg.pos_noise_scale),
            self._noise(rotmat, self.cfg.rot_noise_scale),
            self._noise(self.linear_velocity, self.cfg.lin_vel_noise_scale),
            self._noise(self.angular_velocity_w, self.cfg.ang_vel_noise_scale),
            self._noise(np.array((self.tilt_angle,), dtype=np.float32), self.cfg.tilt_noise_scale),
        )).astype(np.float32)

    def _current_observation(self) -> np.ndarray:
        reference_position, reference_velocity, reference_accel = self._reference_state()
        pos_error = reference_position - self.position
        velocity_error = reference_velocity - self.linear_velocity
        rotmat = quat_wxyz_to_rotmat(self.quat_wxyz)
        yaw = math.atan2(float(rotmat[1, 0]), float(rotmat[0, 0]))
        yaw_error = _wrap_to_pi(self.reference_heading - _wrap_to_pi(yaw + self.cfg.forward_yaw_offset))
        wrench = self._thrust_wrench_forward_matrix_world()
        allocation = self._thrust_wrench_allocation_matrix(wrench)
        thrust_center = self._thrust_center_xy()
        morph_trig = np.array((math.sin(self.tilt_angle), math.cos(self.tilt_angle)), dtype=np.float32)
        current = np.concatenate((
            reference_accel,
            self._noise(pos_error, self.cfg.pos_noise_scale),
            self._noise(velocity_error, self.cfg.lin_vel_noise_scale),
            np.zeros(1, dtype=np.float32),
            np.array((yaw_error,), dtype=np.float32),
            np.array((-self.angular_velocity_w[2],), dtype=np.float32),
            wrench,
            allocation,
            thrust_center,
            morph_trig,
        )).astype(np.float32)
        if current.size != self.cfg.current_obs_dim:
            raise RuntimeError(f"Landing current observation has {current.size} values, expected {self.cfg.current_obs_dim}")
        self.last_debug.update({
            "position": self.position.copy(), "linear_velocity": self.linear_velocity.copy(),
            "angular_velocity_w": self.angular_velocity_w.copy(), "reference_position": reference_position.copy(),
            "reference_velocity": reference_velocity.copy(), "reference_velocity_error": velocity_error.copy(),
            "reference_accel": reference_accel.copy(), "reference_pos_error": pos_error.copy(),
            "yaw_error": yaw_error, "wrench_min": float(np.min(wrench)), "wrench_max": float(np.max(wrench)),
            "allocation_min": float(np.min(allocation)), "allocation_max": float(np.max(allocation)),
            "tilt_angle": float(self.tilt_angle), "reference_time": self._reference_elapsed(),
        })
        return current

    def _reset_reference(self) -> None:
        self.reference_start_position = self.position.copy()
        self.reference_target_position = self.cfg.target_position.copy()
        if self.fixed_trajectory:
            self.reference_target_position[:2] = self.reference_start_position[:2]
        delta = self.reference_target_position - self.reference_start_position
        if self.fixed_trajectory:
            self.reference_start_velocity = np.zeros(3, dtype=np.float32)
            self.reference_end_velocity = np.zeros(3, dtype=np.float32)
        else:
            self.reference_start_velocity = np.array(
                (self.cfg.reference_start_vx, self.cfg.reference_start_vy, 0.0), dtype=np.float32
            )
            self.reference_end_velocity = np.array(
                (self.cfg.reference_end_vx, self.cfg.reference_end_vy, 0.0), dtype=np.float32
            )
        vertical_speed = max(float(self.cfg.reference_vertical_speed), 1e-3)
        duration = max(abs(float(delta[2])) / vertical_speed, float(np.linalg.norm(delta[:2])) / max(self.cfg.xy_reference_max_speed, 1e-3), self.cfg.xy_reference_min_duration_s)
        self.reference_duration = duration * self.cfg.trajectory_duration_scale
        self.reference_accel_duration = 0.2 * self.reference_duration + 0.2
        self.reference_decel_duration = 0.2 * self.reference_duration + 0.5
        self.reference_cruise_duration = max(
            self.reference_duration - self.reference_accel_duration - self.reference_decel_duration,
            self.cfg.policy_dt,
        )
        self.reference_cruise_velocity = (
            delta
            - 0.5 * self.reference_accel_duration * self.reference_start_velocity
            - 0.5 * self.reference_decel_duration * self.reference_end_velocity
        ) / (
            0.5 * self.reference_accel_duration
            + self.reference_cruise_duration
            + 0.5 * self.reference_decel_duration
        )
        self.reference_accel_end = self.reference_start_position + 0.5 * self.reference_accel_duration * (
            self.reference_start_velocity + self.reference_cruise_velocity
        )
        self.reference_decel_start = self.reference_accel_end + self.reference_cruise_duration * self.reference_cruise_velocity
        velocity = self.reference_end_velocity if np.linalg.norm(self.reference_end_velocity[:2]) > 1e-6 else self.reference_start_velocity
        self.reference_heading = math.atan2(float(velocity[1]), float(velocity[0])) if np.linalg.norm(velocity[:2]) > 1e-6 else math.atan2(float(delta[1]), float(delta[0]))
        self.reference_start_time = time.monotonic()
        self.reference_initialized = True
        self.observation_delay_steps = self.cfg.observation_delay_min_steps
        if self.cfg.randomize_reset and self.cfg.observation_delay_max_steps > self.cfg.observation_delay_min_steps:
            self.observation_delay_steps = int(self.rng.integers(self.cfg.observation_delay_min_steps, self.cfg.observation_delay_max_steps + 1))
        virtual_xy = np.zeros(2, dtype=np.float32)
        virtual_z = 0.0
        if self.cfg.randomize_reset:
            virtual_xy = self.rng.uniform(-self.cfg.initial_virtual_xy_range, self.cfg.initial_virtual_xy_range, 2).astype(np.float32)
            virtual_z = float(self.rng.uniform(*self.cfg.initial_virtual_z_range))
        self.cfg.virtual_observation_offset = np.array((virtual_xy[0], virtual_xy[1], virtual_z), dtype=np.float32)

    def _reference_elapsed(self) -> float:
        return max(time.monotonic() - self.reference_start_time, 0.0)

    @staticmethod
    def _seventh_order_segment(start, end, start_velocity, end_velocity, time_s, duration):
        tau = np.clip(time_s / max(duration, 1e-6), 0.0, 1.0)
        tau2, tau3 = tau * tau, tau * tau * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau
        tau6 = tau5 * tau
        tau7 = tau6 * tau
        shape = 35*tau4 - 84*tau5 + 70*tau6 - 20*tau7
        shape_rate = 140*tau3 - 420*tau4 + 420*tau5 - 140*tau6
        shape_accel = 420*tau2 - 1680*tau3 + 2100*tau4 - 840*tau5
        start_shape = tau - 20*tau4 + 45*tau5 - 36*tau6 + 10*tau7
        start_rate = 1 - 80*tau3 + 225*tau4 - 216*tau5 + 70*tau6
        start_accel = -240*tau2 + 900*tau3 - 1080*tau4 + 420*tau5
        end_shape = -15*tau4 + 39*tau5 - 34*tau6 + 10*tau7
        end_rate = -60*tau3 + 195*tau4 - 204*tau5 + 70*tau6
        end_accel = -180*tau2 + 780*tau3 - 1020*tau4 + 420*tau5
        delta = end - start
        return (
            start + delta * shape + start_velocity * duration * start_shape + end_velocity * duration * end_shape,
            delta * (shape_rate / max(duration, 1e-6)) + start_velocity * start_rate + end_velocity * end_rate,
            delta * (shape_accel / max(duration * duration, 1e-6)) + start_velocity * (start_accel / max(duration, 1e-6)) + end_velocity * (end_accel / max(duration, 1e-6)),
        )

    def _reference_state(self):
        t = self._reference_elapsed()
        if t >= self.reference_duration:
            return self.reference_target_position + self.reference_end_velocity * (t - self.reference_duration), self.reference_end_velocity.copy(), np.zeros(3, dtype=np.float32)
        if t <= self.reference_accel_duration:
            return self._seventh_order_segment(
                self.reference_start_position,
                self.reference_accel_end,
                self.reference_start_velocity,
                self.reference_cruise_velocity,
                t,
                self.reference_accel_duration,
            )
        if t <= self.reference_accel_duration + self.reference_cruise_duration:
            cruise_time = t - self.reference_accel_duration
            return (
                self.reference_accel_end + self.reference_cruise_velocity * cruise_time,
                self.reference_cruise_velocity.copy(),
                np.zeros(3, dtype=np.float32),
            )
        decel_time = t - self.reference_accel_duration - self.reference_cruise_duration
        return self._seventh_order_segment(
            self.reference_decel_start,
            self.reference_target_position,
            self.reference_cruise_velocity,
            self.reference_end_velocity,
            decel_time,
            self.reference_decel_duration,
        )

    def _rotor_geometry(self) -> Tuple[np.ndarray, np.ndarray]:
        positions = np.zeros((4, 3), dtype=np.float32)
        axes = np.zeros((4, 3), dtype=np.float32)
        for index in range(4):
            angle = float(_ROTOR_TILT_SIGN[index] * self.tilt_angle)
            positions[index] = (
                _ARM_ORIGINS_B[_ROTOR_ARM_INDEX[index]]
                + _rotate_x(_ROTOR_OFFSETS_ARM[index], angle)
                - _BASE_COM_OFFSET_B
            )
            axes[index] = _rotate_x(np.array((0.0, 0.0, 1.0), dtype=np.float32), angle)
        return positions, axes

    def _thrust_wrench_forward_matrix_world(self) -> np.ndarray:
        rotor_positions_b, axes_b = self._rotor_geometry()
        rotmat = quat_wxyz_to_rotmat(self.quat_wxyz)
        rotor_positions_w = rotor_positions_b @ rotmat.T
        axes_w = axes_b @ rotmat.T
        control_mix = np.vstack((np.ones(4, dtype=np.float32), 0.5 * _PHYSICAL_ROTOR_CONTROL_MIX.T)).astype(np.float32)
        force_w = self.cfg.rotor_kT * control_mix[:, :, None] * axes_w[None, :, :]
        moment_w = _ROTOR_SPIN_DIRECTION[None, :, None] * self.cfg.rotor_kM * self.cfg.rotor_kT * control_mix[:, :, None] * axes_w[None, :, :]
        torque_w = np.cross(rotor_positions_w[None, :, :], force_w) + moment_w
        force = np.sum(force_w, axis=1) / max(self.cfg.nominal_total_kT, 1e-6)
        torque_scale = max(self.cfg.nominal_total_kT * float(np.mean(np.linalg.norm(rotor_positions_b[:, :2], axis=1))), 1e-3)
        torque = np.sum(torque_w, axis=1) / torque_scale
        return np.concatenate((force, torque), axis=1).reshape(-1).astype(np.float32)

    def _thrust_wrench_allocation_matrix(self, wrench: np.ndarray) -> np.ndarray:
        matrix = wrench.reshape(4, 6)
        gram = matrix @ matrix.T + self.cfg.wrench_allocation_damping * np.eye(4, dtype=np.float32)
        return np.clip(np.linalg.solve(gram, matrix), -self.cfg.wrench_allocation_clip, self.cfg.wrench_allocation_clip).reshape(-1).astype(np.float32)

    def _thrust_center_xy(self) -> np.ndarray:
        positions, _ = self._rotor_geometry()
        return (np.mean(positions, axis=0)[:2]).astype(np.float32)


class _RlGamesActor(nn.Module if nn is not None else object):
    def __init__(self, obs_dim: int, action_dim: int):
        super().__init__()
        if nn is None:
            raise RuntimeError("PyTorch is not available")
        self.actor_mlp = nn.Sequential(nn.Linear(obs_dim, 256), nn.ELU(), nn.Linear(256, 128), nn.ELU(), nn.Linear(128, 64), nn.ELU())
        self.mu = nn.Linear(64, action_dim)

    def forward(self, obs: Any) -> Any:
        return self.mu(self.actor_mlp(obs))


class PolicyRunner:
    """Load the current rl_games actor and its observation normalizer."""

    def __init__(self, cfg: LandingStage1Config, device: str = "cpu"):
        self.cfg, self.device = cfg, device
        self.model = None
        self.obs_mean = None
        self.obs_var = None
        self.error: Optional[str] = None

    def load(self) -> bool:
        if torch is None:
            self.error = "PyTorch is not installed"
            return False
        if not self.cfg.policy_path.exists():
            self.error = f"policy file not found: {self.cfg.policy_path}"
            return False
        try:
            self.model = torch.jit.load(str(self.cfg.policy_path), map_location=self.device).eval()
            return True
        except Exception:
            pass
        try:
            checkpoint = torch.load(str(self.cfg.policy_path), map_location=self.device, weights_only=False)
            model = _RlGamesActor(self.cfg.observation_dim, self.cfg.action_dim)
            state = checkpoint.get("model", checkpoint.get("state_dict", checkpoint)) if isinstance(checkpoint, dict) else checkpoint.state_dict()
            tensors = {
                key: value.detach().cpu()
                for key, value in self._flatten(state)
                if torch.is_tensor(value)
            }
            layers = (
                (model.actor_mlp[0], (256, self.cfg.observation_dim), ("actor_mlp",)),
                (model.actor_mlp[2], (128, 256), ("actor_mlp",)),
                (model.actor_mlp[4], (64, 128), ("actor_mlp",)),
                (model.mu, (self.cfg.action_dim, 64), (".mu", "mu_")),
            )
            used = set()
            for layer, shape, preferred_tokens in layers:
                weight = self._select_actor_tensor(tensors, shape, preferred_tokens, used)
                bias = self._select_actor_tensor(tensors, (shape[0],), preferred_tokens, used)
                if weight is None or bias is None:
                    raise RuntimeError(f"missing actor layer with shape {shape}")
                layer.weight.data.copy_(tensors[weight])
                layer.bias.data.copy_(tensors[bias])
                used.update((weight, bias))
            self.model = model.to(self.device).eval()
            flattened = list(self._flatten(checkpoint))
            self.obs_mean = self._find_normalizer(flattened, ("running_mean", "obs_mean", "mean"))
            self.obs_var = self._find_normalizer(flattened, ("running_var", "obs_var", "var", "variance"))
            if self.obs_mean is not None and self.obs_var is not None:
                self.obs_mean = self.obs_mean.to(self.device).reshape(1, -1).float()
                self.obs_var = self.obs_var.to(self.device).reshape(1, -1).float()
            return True
        except Exception as exc:
            self.error = f"could not load rl_games actor: {exc}"
            self.model = None
            return False

    def action(self, observation: np.ndarray) -> Optional[np.ndarray]:
        if self.model is None or torch is None:
            return None
        obs = torch.as_tensor(observation, dtype=torch.float32, device=self.device).unsqueeze(0)
        if self.obs_mean is not None and self.obs_var is not None:
            obs = (obs - self.obs_mean) / torch.sqrt(self.obs_var + 1e-5)
        with torch.no_grad():
            output = self.model(obs)
        if isinstance(output, dict):
            output = output.get("actions", output.get("mu", next(iter(output.values()))))
        elif isinstance(output, (tuple, list)):
            output = output[0]
        return output.squeeze(0).detach().cpu().numpy().astype(np.float32)

    @staticmethod
    def _flatten(value: Any, prefix: str = ""):
        if isinstance(value, dict):
            for key, child in value.items():
                yield from PolicyRunner._flatten(child, f"{prefix}.{key}" if prefix else str(key))
        else:
            yield prefix, value

    def _find_normalizer(self, items, names):
        for key, value in items:
            if torch.is_tensor(value) and value.numel() == self.cfg.observation_dim and any(name in key.lower() for name in names):
                return value.detach().cpu()
        return None

    @staticmethod
    def _select_actor_tensor(tensors, shape, preferred_tokens, used):
        candidates = [
            (key, value)
            for key, value in tensors.items()
            if key not in used and tuple(value.shape) == tuple(shape)
        ]
        for token in preferred_tokens:
            for key, _ in candidates:
                lowered = key.lower()
                if token.lower() in lowered and "critic" not in lowered and "value" not in lowered:
                    return key
        for key, _ in candidates:
            lowered = key.lower()
            if "critic" not in lowered and "value" not in lowered and "sigma" not in lowered:
                return key
        return None
