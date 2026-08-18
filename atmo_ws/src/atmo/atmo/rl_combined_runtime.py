"""Deployment runtime for the IsaacLab combined drive/takeoff/flight/landing policy."""

from dataclasses import dataclass
import argparse
import math
import os
from pathlib import Path
import time

import numpy as np

from atmo.rl_landing_stage1_runtime import (
    ActuatorCommand,
    LandingActionAdapter,
    LandingObservationBuilder,
    LandingStage1Config,
    PolicyRunner,
    quat_wxyz_to_rotmat,
)


DRIVE = 0
TAKEOFF = 1
FLIGHT = 2
LANDING = 3
TAKEOFF_ROUTE = 0
LANDING_ROUTE = 1
MODE_NAMES = ("drive", "takeoff", "flight", "landing")


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None or value == "":
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _env_float(name: str) -> float:
    return float(os.environ[name])


def sample_profile(route_name: str, deterministic: bool, seed: int) -> dict[str, float]:
    """Sample the boundary values used by CombinedTask.reset_initial_state()."""
    if route_name not in {"takeoff", "landing", "full"}:
        raise ValueError("route must be 'takeoff', 'landing' or 'full'")
    if route_name == "full":
        # The full mission starts on the takeoff route; its landing leg is
        # anchored at the hover handover, not sampled here.
        route_name = "takeoff"

    rng = np.random.default_rng(seed)
    vertical_fraction = float(np.clip(
        float(os.getenv("ATMO_RL_VERTICAL_TRAJECTORY_FRACTION", "0.60")), 0.0, 1.0
    ))
    vertical_trajectory = deterministic or bool(rng.random() < vertical_fraction)
    if deterministic:
        heading = 0.0
        drive_speed = 0.0
        drive_duration = 2.0
        takeoff_duration = 3.0
        flight_duration = 3.0
        landing_duration = 3.0
        distance = 0.0
        height = 1.0
        takeoff_heading = 0.0
        takeoff_end_speed = 0.0
        landing_heading = 0.0
        landing_start_speed = 0.0
        ground_speed = 0.0
    else:
        heading = float(rng.uniform(-math.pi, math.pi))
        drive_speed = float(rng.uniform(0.0, 1.0))
        drive_duration = float(rng.uniform(1.0, 4.0))
        takeoff_duration = float(rng.uniform(2.5, 4.0))
        flight_duration = float(rng.uniform(3.0, 5.0))
        landing_duration = float(rng.uniform(2.5, 4.0))
        distance = float(rng.uniform(2.0, 4.0))
        height = float(rng.uniform(1.0, 2.0))
        takeoff_heading = heading + float(rng.uniform(-math.pi / 12.0, math.pi / 12.0))
        takeoff_end_speed = float(rng.uniform(0.0, 1.0))
        landing_heading = heading + float(rng.uniform(-math.pi / 12.0, math.pi / 12.0))
        landing_start_speed = float(rng.uniform(0.0, 1.0))
        ground_speed = float(rng.uniform(0.0, 1.0))

    if vertical_trajectory:
        drive_speed = 0.0
        distance = 0.0
        takeoff_end_speed = 0.0
        landing_start_speed = 0.0
        ground_speed = 0.0

    direction = np.array((math.cos(heading), math.sin(heading), 0.0))
    takeoff_direction = np.array((math.cos(takeoff_heading), math.sin(takeoff_heading), 0.0))
    landing_direction = np.array((math.cos(landing_heading), math.sin(landing_heading), 0.0))
    spawn = np.array((0.0, 0.0, 0.20))
    drive_start_velocity = drive_speed * direction
    drive_velocity = drive_start_velocity
    liftoff = spawn + drive_velocity * drive_duration
    takeoff_end = liftoff + distance * takeoff_direction + np.array((0.0, 0.0, height))
    takeoff_end_velocity = takeoff_end_speed * takeoff_direction
    landing_start = -0.5 * distance * landing_direction + np.array((0.0, 0.0, height))
    landing_position = 0.5 * distance * landing_direction + np.array((0.0, 0.0, 0.20))
    landing_start_velocity = landing_start_speed * direction
    ground_velocity = ground_speed * landing_direction

    initial_position = spawn if route_name == "takeoff" else landing_start - landing_start_velocity * flight_duration
    initial_velocity = drive_start_velocity if route_name == "takeoff" else landing_start_velocity
    drive_yaw = heading if route_name == "takeoff" else landing_heading
    flight_yaw = takeoff_heading if route_name == "takeoff" else landing_heading

    values = {
        "drive_duration": drive_duration,
        "takeoff_duration": takeoff_duration,
        "flight_duration": flight_duration,
        "landing_duration": landing_duration,
        "drive_heading": drive_yaw,
        "flight_heading": flight_yaw,
        "vertical_trajectory": float(vertical_trajectory),
    }
    for name, vector in (
        ("spawn", spawn),
        ("drive_start_velocity", drive_start_velocity),
        ("drive_velocity", drive_velocity),
        ("liftoff", liftoff),
        ("takeoff_end", takeoff_end),
        ("takeoff_end_velocity", takeoff_end_velocity),
        ("landing_start", landing_start),
        ("landing_position", landing_position),
        ("landing_start_velocity", landing_start_velocity),
        ("ground_velocity", ground_velocity),
        ("initial_position", initial_position),
        ("initial_velocity", initial_velocity),
    ):
        for axis, value in zip("xyz", vector):
            values[f"{name}_{axis}"] = float(value)
    return values


@dataclass
class CombinedStage1Config(LandingStage1Config):
    policy_path: Path = Path(os.path.expanduser(os.getenv("ATMO_RL_POLICY_PATH", "~/policies/atmo_combined_stage1.pth")))
    # Four phase one-hots PLUS the signed phase-event timer. The contract has
    # always said 5 (['drive','takeoff','flight','landing','phase_event_time_s'])
    # and this runtime built only the 4 one-hots, which is the 528-vs-529
    # mismatch that blocked closed-loop policy mode.
    task_observation_dim: int = 5
    # Matches CombinedTaskCfg.phase_event_time_clip_s in the training task.
    phase_event_time_clip_s: float = 5.0
    tilt_action_direction: float = 1.0
    drive_zero_thrust: bool = True
    wheels_ground_only: bool = True
    takeoff_thrust_full_tilt_rad: float = math.radians(45.0)
    takeoff_thrust_zero_tilt_rad: float = math.radians(65.0)
    takeoff_prep_duration_s: float = 2.0

    @property
    def observation_dim(self) -> int:
        return super().observation_dim + self.task_observation_dim


class CombinedObservationBuilder(LandingObservationBuilder):
    """Build the 529-value combined observation and its forced three-state route."""

    def __init__(self, cfg: CombinedStage1Config):
        self.route_name = os.getenv("ATMO_RL_ROUTE", "takeoff").strip().lower()
        if self.route_name not in {"takeoff", "landing", "full"}:
            raise ValueError("ATMO_RL_ROUTE must be 'takeoff', 'landing' or 'full'")
        # FULL: takeoff, hover flight_duration seconds, then land where it
        # took off. Starts on the takeoff route; _transition_mode hands over
        # to the landing branch at the end of the hover. Training never
        # chained the two routes in one episode (12 s, one route each), so
        # the hover-to-landing seam is a composition the policy has not seen
        # -- same caveat as m4's mission cycle, and the reason the seam keeps
        # the reference continuous rather than re-anchoring on measurement.
        self.full_mission = self.route_name == "full"
        self.full_ground_z = 0.0
        self.route = LANDING_ROUTE if self.route_name == "landing" else TAKEOFF_ROUTE
        self.mode = DRIVE if self.route == TAKEOFF_ROUTE else FLIGHT
        # DRIVE-ONLY: pin the route to DRIVE and never transition out of it.
        #
        # A ground test on either full route is close to meaningless. The
        # landing route starts in FLIGHT, where the wheel actions are a
        # don't-care -- nothing in training constrains them because the wheels
        # are not touching anything -- and applying those to real wheels is
        # what produced the erratic driving observed 2026-08-14. The takeoff
        # route starts in DRIVE but leaves it after drive_duration.
        #
        # DRIVE is the only phase where wheel commands mean what they say, so
        # a ground test should never leave it. Mirrors M4_DRIVE_ONLY in
        # m4-direct-rl.
        self.drive_only = os.getenv("ATMO_RL_DRIVE_ONLY", "0").lower() in {
            "1", "true", "yes", "on"}
        # Reference ground speed while drive-only, m/s. 0.0 = hold station,
        # which is the honest default: it asks the policy to stay put and
        # makes any drift its own doing.
        self.drive_only_speed = float(os.getenv("ATMO_RL_DRIVE_ONLY_SPEED", "0.0"))
        if self.drive_only:
            self.route = TAKEOFF_ROUTE
            self.route_name = "takeoff"
            self.mode = DRIVE
        self.phase_elapsed_s = 0.0
        self.phase_wall_start_time = time.monotonic()
        super().__init__(cfg)

    def reset_policy_context(self) -> None:
        super().reset_policy_context()
        self.action_history.fill(-1.0)
        self.action_history[:, 1:4] = 0.0
        self.action_history[:, 5:7] = 0.0
        if self.mode != DRIVE:
            self.action_history[:, 0] = 2.0 * self.cfg.neutral_rotor - 1.0
        self.action_history[0, 0] = 0.5 * (self.action_history[0, 0] + 1.0)

    def anchor_fixed_vertical_route(self, takeoff_height: float = 1.0, landing_height: float = 0.20) -> None:
        """Anchor the deterministic hardware route to the current measured state."""
        position = self.position.copy()
        rotmat = quat_wxyz_to_rotmat(self.quat_wxyz)
        yaw = math.atan2(float(rotmat[1, 0]), float(rotmat[0, 0]))
        heading = math.atan2(
            math.sin(yaw + self.cfg.forward_yaw_offset),
            math.cos(yaw + self.cfg.forward_yaw_offset),
        )
        zero = np.zeros(3, dtype=np.float32)

        self.drive_heading = heading
        self.flight_heading = heading
        self.reference_heading = heading
        self.vertical_trajectory = True
        self.drive_duration = 2.0
        self.takeoff_duration = 3.0
        self.flight_duration = 3.0
        self.landing_duration = 3.0
        self.drive_start_velocity = zero.copy()
        self.drive_velocity = zero.copy()
        self.takeoff_end_velocity = zero.copy()
        self.landing_start_velocity = zero.copy()
        self.ground_velocity = zero.copy()

        if self.full_mission:
            # Hover time between takeoff_to_flight and the landing handover.
            self.flight_duration = float(os.getenv("ATMO_RL_HOVER_S", "3.0"))
            # The landed z is the anchored z: the vehicle starts on its
            # wheels, so the measured pose IS the ground truth the landing
            # route would otherwise need --ground-z for.
            self.full_ground_z = float(position[2])

        if self.drive_only:
            # Hold DRIVE forever, with a reference that translates along the
            # measured heading at drive_only_speed. drive_duration is set far
            # beyond any run so _reference_state never clamps the ramp.
            self.spawn = position.copy()
            self.liftoff = position.copy()
            self.takeoff_end = position.copy()
            self.drive_duration = 1.0e6
            self.drive_velocity = np.array(
                (self.drive_only_speed * math.cos(heading),
                 self.drive_only_speed * math.sin(heading),
                 0.0), dtype=np.float32)
            self.drive_start_velocity = self.drive_velocity.copy()
            self.mode = DRIVE
        elif self.route == TAKEOFF_ROUTE:
            self.spawn = position.copy()
            self.liftoff = position.copy()
            self.takeoff_end = position + np.array((0.0, 0.0, max(float(takeoff_height), 0.0)), dtype=np.float32)
            self.mode = DRIVE
        else:
            self.landing_start = position.copy()
            self.landing_position = np.array(
                (position[0], position[1], float(landing_height)), dtype=np.float32
            )
            self.mode = FLIGHT

        self.phase_elapsed_s = 0.0
        self.phase_wall_start_time = time.monotonic()
        self.reference_start_time = time.monotonic()
        self.reference_initialized = True
        self.observation_history_initialized = False
        self.last_debug = {}

    def observation(self) -> np.ndarray:
        transition = self._transition_mode()
        base = super().observation()
        mode = np.zeros(4, dtype=np.float32)
        mode[self.mode] = 1.0
        task = np.concatenate((mode, [self._phase_event_time()])).astype(np.float32)
        observation = np.concatenate((base, task)).astype(np.float32)
        if observation.size != self.cfg.observation_dim:
            raise RuntimeError(f"Combined observation has {observation.size} values, expected {self.cfg.observation_dim}")
        self.last_debug.update({
            "combined_mode": MODE_NAMES[self.mode],
            "combined_route": self.route_name,
            "combined_phase_time": self._phase_time(),
            "combined_transition": transition or "none",
            "obs_norm": float(np.linalg.norm(observation)),
            "obs_min": float(np.min(observation)),
            "obs_max": float(np.max(observation)),
        })
        return observation

    def rotor_thrust_gate(self) -> float:
        """Return physical rotor authority for the current combined mode."""
        if not self.cfg.drive_zero_thrust:
            return 1.0
        if self.mode == DRIVE:
            return 0.0
        if self.mode != TAKEOFF:
            return 1.0
        tilt = float(np.clip(self.tilt_angle, 0.0, math.pi / 2.0))
        return float(np.clip(
            (self.cfg.takeoff_thrust_zero_tilt_rad - tilt)
            / max(self.cfg.takeoff_thrust_zero_tilt_rad - self.cfg.takeoff_thrust_full_tilt_rad, 1e-6),
            0.0,
            1.0,
        ))

    def wheel_speed_gate(self) -> float:
        """Return physical wheel authority, preserving only grounded motion."""
        if not self.cfg.wheels_ground_only:
            return 1.0
        if self.mode in (DRIVE, LANDING):
            return 1.0
        return 1.0 if self.mode == TAKEOFF and self._phase_time() < 0.0 else 0.0

    def _phase_event_time(self) -> float:
        """Signed seconds to the next scheduled phase event, clipped.

        Ported from the training task (combined_task._task_observation) so the
        deployed observation matches the one the policy was trained on:

            DRIVE, FLIGHT  -> 0.0
            TAKEOFF        -> phase_time
            LANDING        -> phase_time - landing_duration   (negative before
                                                               touchdown)
            clamped to +/- phase_event_time_clip_s

        Drive and flight read ZERO deliberately. Their scheduled event is the
        transition itself, and announcing it would reveal the route before the
        mode one-hot does -- a leak the task avoids on purpose. Do not "fix"
        them to report their own phase time.
        """
        phase_time = float(self._phase_time())
        if self.mode == TAKEOFF:
            value = phase_time
        elif self.mode == LANDING:
            value = phase_time - float(self.landing_duration)
        else:
            value = 0.0
        # Match training exactly: nan_to_num(nan=0.0) THEN clamp. That maps NaN
        # to 0 but +/-inf to +/-clip, which is not the same as zeroing both.
        clip = float(self.cfg.phase_event_time_clip_s)
        return float(np.clip(np.nan_to_num(value, nan=0.0), -clip, clip))

    def _read_vector(self, name: str) -> np.ndarray:
        return np.array([_env_float(f"ATMO_RL_COMBINED_{name.upper()}_{axis.upper()}") for axis in "xyz"], dtype=np.float32)

    def _reset_reference(self) -> None:
        prefix = "ATMO_RL_COMBINED_"
        required = prefix + "DRIVE_DURATION"
        if required not in os.environ:
            profile = sample_profile(
                self.route_name,
                _env_bool("ATMO_RL_DETERMINISTIC_TRAJECTORY"),
                self.cfg.random_seed,
            )
            for key, value in profile.items():
                os.environ[prefix + key.upper()] = f"{value:.17g}"

        self.spawn = self._read_vector("spawn")
        self.drive_start_velocity = self._read_vector("drive_start_velocity")
        self.drive_velocity = self._read_vector("drive_velocity")
        self.liftoff = self._read_vector("liftoff")
        self.takeoff_end = self._read_vector("takeoff_end")
        self.takeoff_end_velocity = self._read_vector("takeoff_end_velocity")
        self.landing_start = self._read_vector("landing_start")
        self.landing_position = self._read_vector("landing_position")
        self.landing_start_velocity = self._read_vector("landing_start_velocity")
        self.ground_velocity = self._read_vector("ground_velocity")
        self.drive_duration = _env_float(prefix + "DRIVE_DURATION")
        self.takeoff_duration = _env_float(prefix + "TAKEOFF_DURATION")
        self.flight_duration = _env_float(prefix + "FLIGHT_DURATION")
        self.landing_duration = _env_float(prefix + "LANDING_DURATION")
        self.drive_heading = _env_float(prefix + "DRIVE_HEADING")
        self.flight_heading = _env_float(prefix + "FLIGHT_HEADING")
        self.vertical_trajectory = bool(round(_env_float(prefix + "VERTICAL_TRAJECTORY")))
        self.reference_heading = self.drive_heading if self.route == TAKEOFF_ROUTE else self.flight_heading
        self.mode = DRIVE if self.route == TAKEOFF_ROUTE else FLIGHT
        self.phase_elapsed_s = 0.0
        self.phase_wall_start_time = time.monotonic()
        self.reference_start_time = time.monotonic()
        self.reference_initialized = True
        self.cfg.virtual_observation_offset = np.zeros(3, dtype=np.float32)
        self.observation_delay_steps = self.cfg.observation_delay_min_steps
        if self.cfg.observation_delay_max_steps > self.cfg.observation_delay_min_steps:
            self.observation_delay_steps = int(self.rng.integers(
                self.cfg.observation_delay_min_steps,
                self.cfg.observation_delay_max_steps + 1,
            ))

    def _phase_time(self) -> float:
        return self.phase_elapsed_s + max(time.monotonic() - self.phase_wall_start_time, 0.0)

    def _reference_elapsed(self) -> float:
        return self._phase_time()

    def _trajectory_segment(self, start, end, start_velocity, end_velocity, time_s, duration):
        duration = max(duration, self.cfg.policy_dt)
        accel_duration = 0.2 * duration + 0.2
        decel_duration = 0.2 * duration + 0.5
        cruise_duration = max(duration - accel_duration - decel_duration, self.cfg.policy_dt)
        cruise_velocity = (
            end - start - 0.5 * accel_duration * start_velocity - 0.5 * decel_duration * end_velocity
        ) / (0.5 * accel_duration + cruise_duration + 0.5 * decel_duration)
        accel_end = start + 0.5 * accel_duration * (start_velocity + cruise_velocity)
        decel_start = accel_end + cruise_duration * cruise_velocity
        if time_s <= accel_duration:
            return self._seventh_order_segment(start, accel_end, start_velocity, cruise_velocity, time_s, accel_duration)
        if time_s <= accel_duration + cruise_duration:
            cruise_time = time_s - accel_duration
            return accel_end + cruise_velocity * cruise_time, cruise_velocity.copy(), np.zeros(3, dtype=np.float32)
        return self._seventh_order_segment(
            decel_start,
            end,
            cruise_velocity,
            end_velocity,
            time_s - accel_duration - cruise_duration,
            decel_duration,
        )

    def _transition_mode(self) -> str | None:
        if self.drive_only:
            # Never leaves DRIVE. This is the whole point of the mode.
            return None
        if self.route == TAKEOFF_ROUTE and self.mode == FLIGHT:
            if not self.full_mission:
                return None
            # FULL route hover complete: hand over to the landing branch.
            # landing_start is the REFERENCE hover point, not the measured
            # position -- anchoring transitions to the plan rather than the
            # measurement is the m4 lesson (`be7da62`/`94c7ff7`): re-anchoring
            # on a perturbed pose folds the tracking error into the route.
            phase_time = self._phase_time()
            if phase_time < self.flight_duration:
                return None
            overrun = max(phase_time - self.flight_duration, 0.0)
            self.route = LANDING_ROUTE
            self.landing_start = (
                self.takeoff_end + self.takeoff_end_velocity * self.flight_duration
            ).astype(np.float32)
            self.landing_position = np.array(
                (self.landing_start[0], self.landing_start[1], self.full_ground_z),
                dtype=np.float32,
            )
            self.landing_start_velocity = self.takeoff_end_velocity.copy()
            self.mode = LANDING
            self.phase_elapsed_s = overrun
            self.phase_wall_start_time = time.monotonic()
            return "flight_to_landing"
        if self.route == LANDING_ROUTE and self.mode == DRIVE:
            return None
        phase_time = self._phase_time()
        if self.route == TAKEOFF_ROUTE and self.mode == DRIVE:
            overrun = max(phase_time - self.drive_duration, 0.0)
            if phase_time >= self.drive_duration:
                shift = self.drive_velocity * overrun
                self.liftoff += shift
                self.takeoff_end += shift
                prep_shift = self.drive_velocity * self.cfg.takeoff_prep_duration_s
                self.liftoff += prep_shift
                self.takeoff_end += prep_shift
                self.mode = TAKEOFF
                self.reference_heading = self.flight_heading
                self.phase_elapsed_s = -self.cfg.takeoff_prep_duration_s
                self.phase_wall_start_time = time.monotonic()
                return "drive_to_takeoff"
        elif self.route == TAKEOFF_ROUTE and self.mode == TAKEOFF:
            if phase_time >= self.takeoff_duration:
                overrun = max(phase_time - self.takeoff_duration, 0.0)
                self.mode = FLIGHT
                self.phase_elapsed_s = overrun
                self.phase_wall_start_time = time.monotonic()
                return "takeoff_to_flight"
        elif self.route == LANDING_ROUTE and self.mode == FLIGHT:
            overrun = max(phase_time - self.flight_duration, 0.0)
            if phase_time >= self.flight_duration:
                shift = self.landing_start_velocity * overrun
                self.landing_start += shift
                self.landing_position += shift
                self.mode = LANDING
                self.phase_elapsed_s = 0.0
                self.phase_wall_start_time = time.monotonic()
                return "flight_to_landing"
        elif self.route == LANDING_ROUTE and self.mode == LANDING:
            overrun = max(phase_time - self.landing_duration, 0.0)
            capture = self.landing_position + self.ground_velocity * overrun
            ground_contact = self.position[2] <= capture[2] + 0.03 and abs(float(self.linear_velocity[2])) <= 0.30
            if phase_time >= self.landing_duration and ground_contact:
                self.mode = DRIVE
                self.reference_heading = self.drive_heading
                self.phase_elapsed_s = overrun
                self.phase_wall_start_time = time.monotonic()
                return "landing_to_drive"
        return None

    def _reference_state(self):
        t = self._phase_time()
        zero = np.zeros(3, dtype=np.float32)
        if self.mode == DRIVE:
            if self.route == LANDING_ROUTE:
                return self.landing_position + self.ground_velocity * t, self.ground_velocity.copy(), zero
            drive_time = min(t, self.drive_duration)
            position, velocity, accel = self._seventh_order_segment(
                self.spawn, self.liftoff, self.drive_start_velocity, self.drive_velocity, drive_time, self.drive_duration
            )
            if t > self.drive_duration:
                return position + self.drive_velocity * (t - self.drive_duration), self.drive_velocity.copy(), zero
            return position, velocity, accel
        if self.mode == TAKEOFF:
            if t < 0.0:
                return self.liftoff + self.drive_velocity * t, self.drive_velocity.copy(), zero
            segment_time = min(t, self.takeoff_duration)
            position, velocity, accel = self._trajectory_segment(
                self.liftoff,
                self.takeoff_end,
                self.drive_velocity,
                self.takeoff_end_velocity,
                segment_time,
                self.takeoff_duration,
            )
            if t > self.takeoff_duration:
                return position + self.takeoff_end_velocity * (t - self.takeoff_duration), self.takeoff_end_velocity.copy(), zero
            return position, velocity, accel
        if self.mode == FLIGHT:
            if self.route == TAKEOFF_ROUTE:
                return self.takeoff_end + self.takeoff_end_velocity * t, self.takeoff_end_velocity.copy(), zero
            bounded_time = min(t, self.flight_duration)
            position = self.landing_start - self.landing_start_velocity * (self.flight_duration - bounded_time)
            position += self.landing_start_velocity * max(t - self.flight_duration, 0.0)
            return position, self.landing_start_velocity.copy(), zero
        segment_time = min(t, self.landing_duration)
        position, velocity, accel = self._trajectory_segment(
            self.landing_start,
            self.landing_position,
            self.landing_start_velocity,
            self.ground_velocity,
            segment_time,
            self.landing_duration,
        )
        if t > self.landing_duration:
            return position + self.ground_velocity * (t - self.landing_duration), self.ground_velocity.copy(), zero
        return position, velocity, accel


def _print_shell_profile(route_name: str) -> None:
    deterministic = _env_bool("ATMO_RL_DETERMINISTIC_TRAJECTORY")
    seed_value = os.getenv("ATMO_RL_RANDOM_SEED")
    seed = int(seed_value) if seed_value not in (None, "") else int.from_bytes(os.urandom(8), "little")
    profile = sample_profile(route_name, deterministic, seed)
    for key, value in profile.items():
        print(f"export ATMO_RL_COMBINED_{key.upper()}={value:.17g}")
    initial_heading = profile["drive_heading"] if route_name == "takeoff" else profile["flight_heading"]
    for axis in "xyz":
        print(f"export ATMO_RL_DROP_{axis.upper()}={profile[f'initial_position_{axis}']:.17g}")
        print(f"export ATMO_RL_DROP_V{axis.upper()}={profile[f'initial_velocity_{axis}']:.17g}")
    print(f"export ATMO_RL_DROP_YAW={initial_heading - math.pi:.17g}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--shell-profile", choices=("takeoff", "landing"), required=True)
    args = parser.parse_args()
    _print_shell_profile(args.shell_profile)


__all__ = [
    "ActuatorCommand",
    "CombinedStage1Config",
    "CombinedObservationBuilder",
    "LandingActionAdapter",
    "PolicyRunner",
    "sample_profile",
]
