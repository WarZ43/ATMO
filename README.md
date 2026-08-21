# ATMO

Aerially Transforming Morphobot — reinforcement-learning bring-up and flight
stack for the Caltech ATMO vehicle.

This fork carries the RL deployment workspace (`atmo_ws/`) built on top of the
published ATMO platform. Upstream is
[mandralis/ATMO](https://github.com/mandralis/ATMO), the reference
implementation for the *Nature Communications Engineering* paper; the paper and
citation are at the bottom of this file.

---

## What this is

A ROS 2 (Humble) workspace that runs a trained policy on the real vehicle:

- **`atmo_ws/src/atmo/`** — the nodes. Policy runtime (`rl_controller_hardware.py`,
  `rl_combined_runtime.py`), tilt and drive controllers over RoboClaw, mocap
  bridge and frame handling, PX4 topic indirection (`px4_topics.py`), and the
  RC safety gates (`roboclaw_safety.py`). Legacy MPC controllers are kept but
  unused by the RL path.
- **`atmo_ws/scripts/`** — operator tooling: RC and OptiTrack checks, host
  environment check, actuator sweep, bag recorder, policy export, e-stop.
- **`atmo_ws/atmo_session.sh`** — the launcher. Builds a tmux session for one of
  five profiles.
- **`interface.sh`** — starts the uXRCE-DDS agent that bridges PX4 to ROS 2.
- **`flight_data_20260818/`** — captured arena-day data (bags, ulogs, params)
  and the offline analysis pipeline under `analysis/`.

The Jetson runs the policy through a **numpy actor** — no torch on the vehicle.
Checkpoints are exported to `.npz` off-robot with
`scripts/export_policy_npz.py`.

---

## Vehicle configuration

These were established by measurement. Re-measure them after a board swap or a
replug; do not re-derive them from scratch.

| | |
| --- | --- |
| Companion | Jetson (`m4@m4`), Ubuntu 22.04 / ROS 2 Humble, Python 3.10, numpy 2.x |
| Flight controller | Cube Orange, PX4 v1.17.0 (git `8874d533`), NuttX 11.0.0 |
| PX4 to ROS transport | TELEM2 → CP2102 USB-UART → Jetson, **460800 baud**, DDS domain **0** |
| MAVLink shell | Cube micro-USB, or off-vehicle over the TELEM1 SiK radio (57600) |
| Transmitter | Futaba T14SG, S.Bus, 18 channels |

Three values have to agree or you get an agent that looks healthy and publishes
almost nothing: the **device** (PX4's `UXRCE_DDS_CFG` = 102 = TELEM2), the
**baud** (`SER_TEL2_BAUD` = 460800), and the **domain** (`UXRCE_DDS_DOM_ID` =
0 = `ROS_DOMAIN_ID`). At 921600 the session flaps and creates ~6 of ~67 topics.

`SER_TEL1_BAUD` is 57600 and `SER_TEL2_BAUD` is 460800 — different rates on
purpose. `SER_TEL2_BAUD` was changed from 921600 on 2026-08-14 and saved to
flash; the old `~/ATMO/ATMO/interface.sh` still hardcodes 921600 and no longer
works.

### RC channels

Only two switches on this transmitter are actually mapped:

```
ch8   1094 <-> 1934   free switch  -> the RL gate  (0-based index 7)
ch13  944  <-> 2084   kill, mirrored onto ch17
ch3   throttle, reversed
```

Everything else reads a constant. In particular `RC_MAP_ARM_SW = 15` and
`RC_MAP_FLTMODE = 14` point at dead channels — the inherited map was written
for a T18SZ — which is why the RC arm switch and mode switch do nothing.

**Kill polarity is reversed relative to the lever**: lever UP = 944 µs = not
killed; lever DOWN = 2084 µs = killed. In microseconds it is active-HIGH
(`ATMO_KILL_ACTIVE_HIGH=1`). Always say which axis you mean — conflating lever
and pulse width caused a real sign inversion once already.

This airframe is **single-gate** (`ATMO_RL_OFFBOARD_CHANNEL=-1`): the RL switch
alone carries the gate, and the action-test timer cycles it twice instead of
using a second switch. The deadman is unconditional either way — `rc_lost`,
`rc_failsafe`, implausible pulse widths and a 0.5 s watchdog all drop the gates,
and a test asserts none of those paths can be disabled by config.

### Actuators

```
by-path 2.1  TILT   M2 = tilt motor (encoder), M1 unused
by-path 2.2  DRIVE  M1 = RIGHT wheel, M2 = LEFT wheel, no encoders
```

Use `by-path`, not `by-id` — both RoboClaws enumerate identically and share one
symlink. `by-path` names the USB *socket*, so it is stable across reboots and
wrong after a replug. Read-only identification does not work; only commanding
one motor at a time distinguishes the boards.

The wheels are mounted mirrored, so `drive_controller_hardware.py`'s
`move_left_wheel(+) = ForwardM2` / `move_right_wheel(+) = BackwardM1` asymmetry
is correct. Do not "fix" it.

Tilt: `Forward` = away from fly config (downhill), `Backward` = toward fly
config (uphill). Zero is the **fly** configuration.

---

## Known hardware behaviour

Things that will look like software bugs and are not.

- **RoboClaw power-up lockout.** Every battery cycle starts both boards in a
  signal lockout (`0x23040000` drive / `0x23840000` tilt): commands ACK, rails
  are live, nothing moves, no sound. Clear it by turning the transmitter on and
  **cycling the kill lever once**. Read `ReadError` first whenever an actuator
  is silent — `0x20xxxxxx` is lockout, `0x40`/`0x80` is a real fault.
- **No automatic tilt homing exists** in either code tree. Home by driving
  `BackwardM2` at duty ~60 into the fly hard stop, then `SetEncM2(addr, 0)`
  there. Do not home at duty 100 — it wedges the arm, and escaping then needs
  ~80, which reads as a dead motor at any lower duty.
- **Never ladder the duty up from cold.** Sub-breakaway duties heat the motor
  without moving it (~0.4 %/°C of torque lost), so each attempt arrives weaker
  than the last. Command breakaway at the full known-good duty on the first
  try; if the encoder has not moved within 0.7 s, stop and let it cool. A duty
  ladder overheated the tilt motor on 2026-08-17.
- **The tilt limit switch does not zero the encoder.** `SetPinFunctions`
  returns `ack=False` on firmware v4.4.9 with this driver, so the "M2 goes to
  zero at home" comment in both trees describes a mechanism that was probably
  never configured.
- **`ReadCurrents` on the tilt board is unusable** — it reports −0.61 A on an
  empty terminal and is flat across duty. Use the encoder and `ReadSpeedM2`.
- **The drive board died 2026-08-17** (regen through the 12 V rail). Wheels are
  off by default; `--drive on` re-enables them.
- **Uncommanded tilt drift** of ~16° has been observed between commands on a
  supposedly non-backdrivable worm gear. Commanded tilt position is not
  reliably held between commands, which the policy assumes it is. Open.
- **Hitting the 90° stop while powered does not trap you** — the inhibit is a
  power-ON latch. Drive straight back out (`BackwardM2`, duty 80) and never
  power cycle while sitting on the switch.

### Safety rules

- **No remote tilt actuation.** All tilt motion is operator-performed, hands on
  the kill. Remote reads of the tilt board are fine. Remote command loops react
  in seconds where an operator reacts instantly.
- **No tilt-actuating session without a verified homing first.**
- **No rotor work until the pack voltage is checked.** The packs on hand are 6S
  and the ESCs are not rated for 6S — a 6S spin attempt kills ESCs instantly.
  Confirm the ESC rating and that they feed directly from the flight battery,
  not the 12 V regulator.

---

## Running a session

```bash
cd ~/ATMO_rl && ./interface.sh
```

Start the agent **first**, every time — especially after a Jetson reboot. Then:

```bash
cd ~/ATMO_rl/atmo_ws && source scripts/atmo_env.sh && bash scripts/check_host.sh
```

```bash
ros2 topic list | grep -c fmu
```

Expect ~67. A healthy agent start is **one** `create_participant`, then a burst
of `create_topic`, then quiet. Repeated `session re-established` means a baud
mismatch or a marginal link.

Before trusting anything read over DDS, check that exactly one agent is running
at the right baud, that no foreign nodes are up, that both ACM ports are free,
and that no user-level autostart unit exists:

```bash
pgrep -a MicroXRCEAgent; ros2 node list; fuser /dev/ttyACM0 /dev/ttyACM1; systemctl --user list-units --type=service --all | grep -i atmo
```

The `--user` matters. A user unit from a third workspace once autostarted
controllers at every boot, holding a RoboClaw open and spinning wheels off an
RC switch — invisible to a bare `systemctl list-units`.

Launch a profile:

```bash
./atmo_session.sh sensor
```

**Ctrl-C in the stack window is the stop.** Every node zeroes its actuators in a
`finally`. The kill switch and dropping the RL gate are equally valid. Do not
`tmux kill-session` while a bag is recording — it writes the `.db3` but never
`metadata.yaml`, and rosbag cannot reopen that.

### Profiles

| Profile | What it does |
| --- | --- |
| `sensor` | Connectivity only. No command publishers. |
| `shadow` | Policy runs, no command publishers created at all. Writes a JSONL observation log. |
| `action` | Single-axis action test: `lift`, `roll`, `pitch`, `yaw`, `tilt`, `drive`, `turn`. |
| `ground` | Closed-loop policy on tilt and wheels only. Rotors cut, no mocap. |
| `policy` | Full closed-loop run. Refuses to run on a virtual pose. |

Useful flags: `--route takeoff|landing|full`, `--policy PATH`,
`--action-test NAME --action-sign positive|negative --action-magnitude V`,
`--kill-test-passed`, `--drive on|off`, `--px4-relay on|off`.

### Build and test

```bash
cd ~/ATMO_rl/atmo_ws && colcon build --symlink-install
```

`--symlink-install` means pure-Python edits are live without a rebuild.

```bash
bash scripts/run_unit_tests.sh
```

The unit suite needs no ROS and no torch. Run it via the script — **not** with
the workspace sourced. It strips `PYTHONPATH`, because with build/install on
the path the `atmo` package appears three times and pytest's collection hook
loses the newest module.

`px4_msgs` in this tree is matched to the flight controller (PX4 v1.17.0, git
`8874d533`). Do not swap it for upstream.

---

## Bring-up gates

Staged so a failure at stage N cannot be caused by something stage N−1 would
have caught.

**The one rule: a test that shows you the number before you predict it is not a
measurement.** Every check script prints its expectations first. Read them,
commit to a prediction, then move the hardware.

- **Stage 0 — RC.** First test of every session. Channel identification, gate
  behaviour (including *reads low with the transmitter off* — that is the
  deadman), and kill polarity.
- **Stage A — actuator mapping.** Props off, restrained, one channel at a time.
  **A1 is the lift-and-kill proof**: all four motors spin, and the physical kill
  stops all four. Nothing else in Stage A runs until A1 passes; declare it
  afterwards with `--kill-test-passed`.
- **Stage B — sensors and failure behaviour.** Move the unpowered vehicle by
  hand; message ages, PX4 NED pose and velocity signs, tilt angle and RC gates
  all read correctly. PX4's NED yaw is opposite the policy world's z-up yaw —
  expect that, do not "fix" it here.
- **Stage C — OptiTrack.** C1 pose signs, C2 twist signs and body-frame, C3
  rate with worst gap under 40 ms, C4 static noise inside the training
  envelope, C5 PX4 relay fused with `EKF2_EV_DELAY` set from C3. Mocap gates
  **do not carry over between sessions** — the network changes and so does
  Motive.
- **Stage D — integrated dry runs**, restrained.
- **Stage E — flight**, in order with a full stop between each: tethered hover,
  free hover, takeoff route, landing route, full combined profile. Landing
  stays out of the first flights; its completion logic leans on proxies for a
  contact sensor the vehicle does not have.

Record everything — every session records a bag. Write the numbers down; a
measurement nobody recorded gets re-taken.

---

## OptiTrack

The VRPN client runs on the **laptop** (`scripts/run_mocap_laptop.sh`), not the
robot; the bridge converts to `/atmo/groundtruth_odom` and optionally relays to
`/fmu/in/vehicle_visual_odometry`. Use `scripts/hardware_optitrack_check.py`
for the pose, twist, rate and noise gates, and `scripts/fake_vrpn_publisher.py`
to exercise the path without the rig.

The pose frame was verified by motion on 2026-08-18: z-up confirmed, x/y
consistent. An apparent "x reversed" was the vehicle sitting ~120° off Motive's
+X — decompose by the vehicle's heading before concluding a sign is wrong. The
pose check's printed expectations assume yaw ≈ 0.

Two WiFi settings are mandatory on the companion, learned the hard way: pin the
BSSID (no roaming) and disable powersave. A scan storm took the radio
off-channel every ~7 s and punched 2–6 s holes in the mocap stream while the
signal looked perfect.

When running against a DDS discovery server, `interface.sh` must join it too —
it now exports `ATMO_DDS_DISCOVERY_SERVER` into the agent itself. An agent on
plain multicast and nodes on the server form two parallel worlds, with no error
anywhere.

---

## Flight data and analysis

`flight_data_20260818/` holds the 2026-08-18 arena session: rosbags, PX4 ulogs
395–417, and `mav.parm`. The reproducible entry point is
`analysis/smatrix_validation.py`, which regenerates every figure and merged CSV.

Headline result: the two post-flip attitude departures trace to a single
uncalibrated actuator sign table — roll and yaw columns inverted, inherited
from sim through training into deployment, and uncheckable by any test that
shared the same spec — amplified by taking off through high tilt, where control
authority differs radically. Against the flight data the S(φ) dynamics model
validates quantitatively on pitch and yaw (slopes ≈ 1) and on the lift axis
(vz r = 0.83); the roll axis carries an anomaly whose data-predicted cause is a
prop damaged during the second attempt. Yaw kM is ~2.4× the modeled value.

Two recording failures degraded that dataset and are fixed in the tree:

1. **Endpoint matching.** Under the discovery server, matching crosses the WiFi
   even for same-host nodes, and a half-completed match leaves a RELIABLE
   publisher silently sending to nobody. `_telemetry_matched()` in
   `rl_controller_hardware.py` now blocks engagement until every critical
   publisher sees all its consumers, and returns before any command is
   published.
2. **Intent was never logged.** Raw policy actions now publish on
   `/atmo/rl/policy_action` (14 floats at 50 Hz: 7 raw network outputs plus 7
   semantic), and the recorder captures it along with `/fmu/in/tilt_angle`.

---

## Gotchas worth knowing before you debug

- **PX4 1.17 renamed some topics per-message, not wholesale**:
  `vehicle_status` → `vehicle_status_v1`, likewise `battery_status`,
  `vehicle_local_position`, `home_position`. A wrong name does not crash — the
  subscription simply never fires and the node runs looking healthy while blind.
  All names go through `atmo/px4_topics.py`, and a test forbids bare `/fmu/`
  literals.
- **`/fmu/in/tilt_angle` and `/fmu/out/actuator_armed` are absent from the
  firmware's `dds_topics.yaml`.** PX4 never learns the arm tilt, and kill state
  cannot be read from ROS. One firmware rebuild covers both.
- The `ActuatorArmed` field is **`kill`**, not `manual_lockdown` — the vendored
  message still declares the latter, but this firmware does not report it.
- **Zero velocity is a measurement, not an absence.** Publishing
  `velocity = [0,0,0]` told the EKF the vehicle was stationary. NaN is how you
  decline.
- **numpy 2.x removed** `ndarray.ptp()`, `.itemset()`, `np.float_`, `np.NaN`,
  `np.in1d`, `np.trapz`. Grep for these in anything ported in.
- **Do not `pip install --user setuptools`** — a user-site setuptools newer than
  Ubuntu's packaging module breaks `colcon build`.
- Channel numbering differs between the two code trees on this robot: this one
  is 0-based (`values[OFFBOARD_CHANNEL]`), the CATMO tree is 1-based. Both
  write `8` and mean channels one apart.
- **When a result contradicts a written note, prefer the measurement.**

---

## Fresh Jetson install, in order

1. Ubuntu 22.04, user `m4`, hostname `m4` — every path and ssh target assumes
   `/home/m4`.
2. WiFi to the robot router. Pin the BSSID, disable powersave, turn autoconnect
   off on every other profile, and disable scan MAC randomization. Acceptance
   test: pull a 60 MB file from the laptop. Seconds is good; minutes means stop
   and diagnose before building anything on top.
3. `openssh-server git tmux curl python3-pip build-essential cmake`, and
   `usermod -aG dialout m4` for the serial ports.
4. ROS 2 Humble `ros-base` plus `ros-dev-tools` and
   `python3-colcon-common-extensions`. No desktop needed on the robot.
5. `pip install numpy pytest`.
6. Micro XRCE-DDS Agent v2.4.3 from source, so `MicroXRCEAgent` is on PATH.
7. Clone this repo to `~/ATMO_rl`, `colcon build --symlink-install`, run the
   unit suite.
8. Copy the policy `.npz` to `~/policies/` and verify its md5 against the
   source.
9. `~/.bashrc`: source Humble, `export ATMO_ROS_DOMAIN_ID=0` and
   `export ATMO_RL_OFFBOARD_CHANNEL=-1`. Do **not** set `ROS_DOMAIN_ID`,
   `CYCLONEDDS_URI` or `ROS_DISCOVERY_SERVER` globally —
   `scripts/atmo_env.sh` handles those per shell.

Deliberately not installed: torch (numpy actor on the vehicle, export happens
off-robot), acados (legacy MPC only), CUDA/TensorRT, `vrpn_mocap` (laptop by
design), and any autostart unit.

After a carrier-board change, re-measure the RoboClaw `by-path` names, the
CP2102 device path, the WiFi interface name, and the Jetson's IP.

---

## Contact

📧 [imandralis@caltech.edu](mailto:imandralis@caltech.edu) — questions about the
published platform. Open an issue here for anything specific to this fork.

## Citation

[![DOI](https://zenodo.org/badge/DOI/https://doi.org/10.1038/s44172-025-00413-6.svg)](https://doi.org/10.1038/s44172-025-00413-6)

Paper: <https://rdcu.be/eio6G> · [Video](https://www.youtube.com/watch?v=5w7pl7xQGKM)

```bibtex
@article{Mandralis2025,
  title = {ATMO: an aerially transforming morphobot for dynamic ground-aerial transition},
  volume = {4},
  ISSN = {2731-3395},
  url = {http://dx.doi.org/10.1038/s44172-025-00413-6},
  DOI = {10.1038/s44172-025-00413-6},
  number = {1},
  journal = {Communications Engineering},
  publisher = {Springer Science and Business Media LLC},
  author = {Mandralis, Ioannis and Nemovi, Reza and Ramezani, Alireza and Murray, Richard M. and Gharib, Morteza},
  year = {2025},
  month = apr
}
```
