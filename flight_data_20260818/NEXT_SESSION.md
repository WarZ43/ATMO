# ATMO next session — deploy list (written 2026-08-19 ~00:50, Jetson was off)

## STATUS UPDATE (2026-08-19, pre-presentation)

DONE in the laptop repo (atmo_ws commit 3b870c8, symlink-install so live):
- Item 1 (engagement telemetry gate): implemented as
  `_telemetry_matched()` in rl_controller_hardware.py; blocked engagement
  returns before ANY command publication and retries next tick.
- Item 2 (recorder): `/fmu/in/tilt_angle` AND the new
  `/atmo/rl/policy_action` added to record_atmo_bag.sh.
- NEW: raw policy actions published on `/atmo/rl/policy_action`
  (14 floats @ 50 Hz: 7 raw network outputs + 7 semantic) — the 8/18
  analysis had to replay the network offline because intent was never
  logged. Gate requires the recorder matched on it before engagement.

STILL TO DO ON THE JETSON: rsync the repo, `colcon build
--packages-select atmo` there (laptop colcon build fails on
setuptools-80 `develop --uninstall`; the Jetson's older setuptools
should be fine), items 3-7 below unchanged. Note: measured tilt for
8/18 was recovered from the tilt node's launch.log prints — see
ANALYSIS_HANDOFF §7.


## 1. Engagement gate: policy must NOT run unless telemetry is verifiably recorded

In `~/ATMO_rl/atmo_ws/src/atmo/atmo/rl_controller_hardware.py`, in the
engagement path (where "session engaged" is logged / `gates_requested` first
honored), add a matched-subscriber check BEFORE allowing engagement:

```python
# Telemetry gate (2026-08-18 post-mortem): under the discovery server,
# endpoint matching crosses the WiFi even for same-host nodes, and a half-
# completed match leaves a RELIABLE publisher silently sending to nobody
# (bags 202503/205315: recorder discovered but never matched -> 0/partial
# actuator_motors captured). get_subscription_count() reflects COMPLETED
# matches, so require every critical publisher to see all its consumers
# before the policy may engage.
#   actuator_motors: agent + rosbag = 2 (policy mode)
#   tilt_vel:        tilt node + rosbag = 2
required = int(os.getenv("ATMO_RL_MIN_MATCHED_SUBS", "2"))
checks = []
if self.actuator_motors_publisher is not None:
    checks.append(("actuator_motors",
                   self.actuator_motors_publisher.get_subscription_count()))
checks.append(("tilt_vel", self.tilt_vel_publisher.get_subscription_count()))
unmatched = [(n, c) for n, c in checks if c < required]
if unmatched:
    self.get_logger().error(
        "ENGAGEMENT BLOCKED: publishers not fully matched %s (need >=%d "
        "matched subscribers each; is the recorder up? did discovery "
        "complete?). Set ATMO_RL_MIN_MATCHED_SUBS to override." %
        (unmatched, required))
    return   # refuse engagement this tick; retry next
```

Rebuild: `colcon build --packages-select atmo`.

## 2. Recorder: add measured tilt angle (Jetson copy)

`scripts/record_atmo_bag.sh` DEFAULT_TOPICS: add `/fmu/in/tilt_angle`
(laptop copy already patched — copy the same line or rsync the file).

## 3. Discovery: stop routing Jetson-local matching over WiFi

Run a second Fast DDS discovery server ON the Jetson and list BOTH:
`ROS_DISCOVERY_SERVER="192.168.0.4:11811;127.0.0.1:11811"` (server list
syntax; Jetson-local matches then never cross the air). Verify with a
kill-the-WiFi test: local topics must keep matching.

## 4. Before ANY flight: restrained single-axis tests (props on)

roll+/-, pitch+/-, yaw+/- via `atmo_session.sh action`. Expected responses
per the CORRECTED mixer (roll and yaw columns negated vs training spec —
only the yaw half is deployed as of 8/18; the ROLL negation is designed,
validated against Ioannis' dynamics.py S-matrix + flight IMU data, but NOT
YET DEPLOYED).

**READ ANALYSIS_HANDOFF §13 FIRST.** Net result after the 2026-08-20 pass:
**the roll column IS inverted — deploy the designed roll negation.** That is
§3's original conclusion, and log_417 now supports it model-free: the policy's
roll command climbed +0.15 -> +0.37 while roll ran -5 -> -100 deg (positive
feedback, +cmd gives -38 rad/s^2 measured against the FC gyro), while PITCH
departed and RECOVERED in the same 1.7 s window (+cmd gives +30 rad/s^2). Roll
unstable, pitch stable, same policy, same seconds. Yaw is not testable from 417
(+1.1 rad/s^2 per unit at that tilt) - 413 is its evidence.

**Know which motor is which before running these** (ANALYSIS_HANDOFF §12,
`analysis/rotor_identity.py`): control[0] front-right CCW, [1] rear-left CCW,
[2] front-left CW, [3] rear-right CW, from PX4's own CA table and Ioannis'
S rows agreeing independently. The roll+yaw inversion is *exactly* a
left-right mirror of the rotor numbering (deployed mix == correct mix with
rows permuted by [2,3,0,1]), so the designed column flip and a motor-index
map `perm = [2,3,0,1]` are the SAME fix — deploy one, never both. The
restrained test is what tells you the mirror is real, so run it before
deciding which form to ship.

## 5. Hardware

- Inspect FRONT-RIGHT prop (rotor0/control[0]): flight data shows ~50%
  thrust, ~4x drag torque after the carabiner strike (log_417 fits).
- Bench: thrust curve (T vs u) and kM (healthy value ~0.044, model 0.018).

## 6. Data to grab from the Jetson

- `~/ATMO_rl` full repo copy (only ATMO/CATMO were archived; the replay
  needs `rl_combined_runtime.py`).
- Push `bringup` commits 7f557a4 + f291c15.

## 7. Sim/training updates queued

- ATMO_KM 0.018 -> ~0.044 in vehicle_specs.py.
- Fix the roll+yaw sign columns in ATMO_SPEC control_mix (match
  Ioannis dynamics.py, then retrain/re-export).
- COM / mass model (root of the over-trained nose-up trim; 417 pitched up
  22° on the thrust ramp). Findings 2026-08-19:
    * Trained URDF: COM 7.7 mm forward of the thrust center — and the WHOLE
      offset comes from the base link's CAD-exported inertial origin (base
      COM 18.1 mm forward of the tilt-hinge line; arms/rotors symmetric).
    * Ioannis' flown model: 3.0 mm (his dynamics provably place the base COM
      at the body origin — gravity vector in dynamics.py has no m_base×r
      moment term — so his r_BA=6.6 mm IS his measured base-COM-to-hinge
      distance, battery included: his base is 2.33 kg vs URDF 2.087).
    * Matt's hands-on estimate of the real vehicle: ~6 mm. Band: 3-8 mm.
  ACTIONS:
    1. Knife-edge measurement (no tools needed): balance the vehicle in fly
       config, battery installed, across a straightedge perpendicular to the
       roll axis; measure balance line -> tilt-hinge axis. ~1-2 mm accuracy.
    2. Retrain with URDF base COM set so total COM sits ~6 mm forward of the
       thrust center, and DOMAIN-RANDOMIZE base COM x by +/-5 mm — the
       exaggerated feedforward trim exists because the sim COM was one exact
       wrong value; randomization forces feedback trim. Also close the mass
       gap per assembly (base+battery 2.33, arm+wheels 1.537, rotor 0.021).
    3. Post-flight checklist: from the first stable hover, steady pitch trim
       cmd × pitch effectiveness (~28 N·m/unit) / weight = measured COM
       offset to ~1 mm. Compute it automatically; it should converge to the
       knife-edge number.
- Fix the mocap ingestion frame — ANALYSIS_HANDOFF §13. This is now a SEPARATE
  fix from the mixer, not a paired one. Measured with the deployed builder over
  the real logged messages, across three runs: the mocap body frame is the FC
  body frame rotated 178.6-179.9 deg about y, and `update_px4_state` then applies
  `_NED_TO_ENU`/`_FRD_TO_FLU` on top of data that is already z-up/body-frame.
  Two consequences are exact and are defects on their own terms: the observed
  HEIGHT is negated (slope -1.000 on a run with 5.39 m of real climb) and
  observed x and y are SWAPPED (+1.000 cross, no same-axis correlation). Fix
  those. Verify with `analysis/frame_proof.py`: height slope +1.000, x/y
  unswapped.
- Consider homing tilt to fly BEFORE spool-up in the mission profile: the
  8/18 attempts took off in/through high tilt where pitch authority ~ 0.
