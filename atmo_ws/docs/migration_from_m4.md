# Migration from m4-direct-rl

Record of what was carried over from the White M4 deployment repo
(`~/Documents/m4-direct-rl`) into this workspace, what deliberately was not, and
what the port found on the way in.

## Why the platform changed

The White M4 stack worked end to end in simulation and reached Stage B on
hardware. It was stopped by an actuator limit, not a software one: its eight
Dynamixel servos could not hold the arms against rotor thrust.

ATMO does not have that problem. Its tilt is a single coupled degree of freedom
driven by a RoboClaw geared motor with an encoder and a limit switch, and its
wheels are RoboClaw-driven. Holding torque is a mechanical property of the drive
rather than a servo current limit.

That difference is also why most of the M4 *actuator* work does not transfer.

## What did NOT carry over, and why

| Not ported | Reason |
| --- | --- |
| `m4_dynamixel_transport` | The whole reason for the platform change. ATMO has no Dynamixels. |
| The hip-frame conversion chain | M4 had four hips at `pi/2 - logical`; ATMO has one tilt DOF read from an encoder table. Different problem entirely. |
| 8-DOF hip/leg kinematics, `hip_actions.py`, `leg_actions.py` | No such joints. |
| The M4TII Gazebo model and `m4_gazebo_backend` | ATMO's sim already exists and is working. |
| The C++ command core and hardware backend | ATMO's runtime is Python end to end, and it works. Rewriting it would replace tested code with untested code for no gain. |
| Position-mode goal leashing, joint census tooling | Dynamixel-specific. |

The general lesson is worth stating: the M4 repo's *value* to ATMO is its
bring-up methodology and its OptiTrack knowledge, not its actuator code.

## What was carried over

### OptiTrack

- **`atmo/mocap_bridge.py`** — new node, superseding `relay_mocap.py`. Derives
  filtered world-linear and body-angular twist, publishes
  `/atmo/groundtruth_odom` (`nav_msgs/Odometry`, pose z-up world, twist body
  frame) alongside the PX4 vision relay, offers `source_frame` y-up/z-up, and
  reports rate, gaps and staleness.
- **`atmo/mocap_frames.py`** — the frame arithmetic, ROS-free so it can be
  tested anywhere.
- **`scripts/hardware_optitrack_check.py`** — Stage C gates: pose signs, twist
  signs, rate and dropout, static noise. Prints expectations before measuring.
- **`docs/optitrack_bringup.md`** and **`docs/optitrack_session_checklist.md`**
  — the setup, the failure modes, and the run sheet. This is where the DDS
  discovery knowledge lives.
- **`scripts/fastdds_super_client.xml`**, **`scripts/fastdds_wifi_only.xml`** —
  reused unchanged. The whitelist one is kept only as a record that it did not
  work on this lab network.

### Session orchestration

- **`atmo_session.sh`** — tmux session builder with profiles (shadow, sensor,
  action, policy), replacing `startup_robot.sh`'s bare panes and `sleep 10`.
- **`stop_atmo_session.sh`** — ordered teardown: manual override, zero commands,
  disarm, stop the stack so `on_shutdown()` reaches the RoboClaw, then finalize
  the bag last.
- **`scripts/atmo_env.sh`** — per-window ROS environment with the override block
  at the end.
- **`scripts/check_host.sh`** — preflight, run on both machines and compared.
- **`scripts/record_atmo_bag.sh`** — waits for the custom message types to
  resolve before recording.
- **`scripts/atmo_operator.sh`** — the procedure on screen, next to the stop
  command.

`rl_control.launch.py` gained a `record` argument so its built-in recorder can
be turned off when the session script runs its own.

### Policy loading

- **`atmo/numpy_actor.py`** — numpy-only inference, so the robot needs no torch.
  Wired into the existing `PolicyRunner` by file extension rather than as a
  parallel path.
- **`scripts/export_policy_npz.py`** — `.pth` → `.npz` on a machine with torch,
  verified against torch over random observations before anything is written,
  and stamped with the source checkpoint's sha256.

### Contract

- **`atmo/policy_contract.py`** — loads and validates the exported training
  contract, and cross-checks the runtime config against it.
- **`atmo/contracts/atmo_combined_v1.json`** — generated from the live
  `ATMO_SPEC` by `M4/export_atmo_deployment_contract.py`. It lives inside the
  Python package so it resolves identically from a source tree and after
  `colcon build`.

The runtime now refuses to start a closed-loop `policy` run against a
mismatched observation layout. Shadow and the test modes warn and continue,
because they are still informative with a mismatch.

## What the port found

Four things, in descending order of how much they matter.

### 1. The task observation is one element narrower than training expects

`rl_combined_runtime.CombinedStage1Config.task_observation_dim` is **4** — the
mode one-hot alone — giving `observation_dim` 528. The exported `ATMO_SPEC`
contract says **5** and 529: the one-hot plus a signed phase-event timer
(negative through takeoff's prep window, the offset from expected touchdown
during landing, zero in drive and flight, clipped to ±5 s).

The m4 repo records the same change as a post-training addition, so ATMO's
runtime is simply on the older layout.

**This is not fixed here, deliberately.** Which value is right depends on which
checkpoint you actually train and deploy — changing the runtime to 5 against a
policy trained at 4 breaks it just as thoroughly as the reverse. The contract
check names the mismatch precisely and blocks a policy run until it is
reconciled. Reconcile it by deciding which side moves, not by silencing the
check.

### 2. The mocap relay told PX4 the vehicle was stationary

`relay_mocap.py` published `velocity = [0, 0, 0]` with `velocity_frame` set to
NED. PX4 reads that as a measurement, not as an absence, and fuses it against
every real motion. NaN is how a field is declined.

Fixed in both the legacy relay and the new bridge.

### 3. The PX4 quaternion mapping is unresolved

`relay_mocap.py` maps the raw streamed quaternion to NED as `(w, -z, x, -y)`.
Composing y-up → z-up → NED, as the m4 bridge does, gives `(w, x, z, -y)`.
These are different rotations. The position halves of the two chains *do* agree
— there is a test pinning that — so the disagreement is specifically attitude.

The bridge defaults to the ATMO mapping because it is the one that has flown
here, and exposes the other as `px4_quaternion:=composed`. A test asserts they
differ, so nobody tidies the difference away without measuring. Settle it in
Stage C.

### 4. A missing normalizer was silent

`PolicyRunner.action()` skipped normalization entirely when the checkpoint had
no `RunningMeanStd` tensors. The actor is trained with `normalize_input`, so
that produces garbage that still looks like a working policy. The runner now
records a warning, the node logs it as an error, `policy` mode refuses to run,
and the exporter refuses to write.

## Still open

- **The exporter's randomization values are hardcoded.**
  `export_atmo_deployment_contract.py` writes `motor_tau_seconds [0.125, 0.175]`
  and `observation_delay_steps [0, 1]` as literals rather than reading them from
  the training config. The m4 exporter was upgraded to parse its sources by
  name, so a renamed setting fails the export instead of passing a stale value
  through. Worth doing here before the next retrain — and note the m4 project
  widened both of these (tau to `[0.08, 0.25]`, delays to `[0, 1, 2]`) for
  deployment robustness, which ATMO's training may or may not have adopted.
- **The tilt encoder table needs recalibration.** `tilt_controller_hardware.py`
  says so in its own comment: the table was collected for a different encoder
  part. A wrong tilt angle corrupts the observation the same way the m4
  hip-frame inversion did. Gate A3 in `docs/hardware_bringup.md`.
- **`ROS_DOMAIN_ID` on the robot.** `atmo_env.sh` sets 42, matching the m4
  convention. If ATMO's flight stack expects something else, change it in one
  place — but change it on both machines.
- **No trained checkpoint is on this workstation.** `atmo/policies/` is empty.
  The existing checkpoint is treated as a plumbing fixture only: good for shape
  and dimension checks, not for any behavioural conclusion.
- **The robot is not on Foxy.** Measured 2026-08-14 over SSH: the Jetson (`m4`,
  192.168.0.44) runs **Humble, Python 3.10.12, numpy 2.2.6**, and has no torch.
  Every ATMO shell script hardcoded `/opt/ros/foxy/setup.bash` at line 1 and
  would have failed immediately. They now source `scripts/atmo_env.sh`, which
  detects the distro (override with `ATMO_ROS_DISTRO`).

  Two consequences worth carrying:

  - **numpy 2.x removed `ndarray.ptp()`** (and `.itemset()`, `.newbyteorder()`,
    `np.float_`, `np.NaN`, `np.in1d`, `np.trapz` …). Two `.ptp()` calls in
    `hardware_optitrack_check.py` were fixed to `np.ptp(...)`. Anything else
    ported from an older codebase should be grepped for these before it runs.
  - **No torch on the robot**, so the `.npz` numpy actor is not an optimisation
    here — it is the only way a policy runs at all.

  Everything added is written to 3.8 syntax (`# type:` comments rather than
  annotations), which is harmless on 3.10 and keeps the door open if any part
  of this ever runs on the Foxy machine the upstream scripts assumed.

## Testing

```bash
cd src/atmo && python -m pytest test/ -q
```

The three new suites (`test_policy_contract.py`, `test_numpy_actor.py`,
`test_mocap_frames.py`, 44 tests) run anywhere — no ROS, no torch, no robot.
The ROS nodes themselves still need `colcon build` on the target, which is the
real check for anything touching rclpy.
