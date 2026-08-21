# Session state: ATMO bring-up

Written as a handoff so a fresh conversation can continue without re-deriving
anything. Covers the measured vehicle configuration, what works, what is
deliberately configured rather than fixed, and what is still open.

Last updated **2026-08-17**.

---

## Where things stand in one line

The DDS link, RC, and the read-only half of Stage 0 all work. Nothing has been
commanded to move. No policy checkpoint exists on the robot.

---

## The vehicle, measured

Do not re-derive these. They were each established the hard way.

```
Jetson        m4@m4, wlan0 192.168.0.44, also on Tailscale
              Ubuntu / ROS 2 HUMBLE, Python 3.10.12, numpy 2.2.6, NO torch
              ~/.bashrc sources Humble and sets ATMO_ROS_DOMAIN_ID=0.
              It sets no ROS_DOMAIN_ID/CYCLONEDDS_URI/ROS_DISCOVERY_SERVER
              of its own -- keep it that way.

Flight ctrl   Cube Orange, PX4 v1.17.0, git 8874d533, branch main
              Built 2025-11-26, NuttX 11.0.0

Transport     TELEM2 -> CP2102 USB-UART -> Jetson /dev/ttyUSB0
              UXRCE_DDS_CFG    = 102 (TELEM2)
              UXRCE_DDS_DOM_ID = 0        <- ROS_DOMAIN_ID must equal this
              SER_TEL2_BAUD    = 460800   <- 921600 FLAPPED, see below
              TELEM1 -> SiK radio, NetID 25 (MAVLink, untouched)

MAVLink       Cube's micro-USB -> Jetson, appears as
              /dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00
              python3 ~/tools/mavlink_shell.py <that path>     (no baud needed)
              TELEM2 runs XRCE, not MAVLink, so no shell is available there.
              But a GROUND-SIDE SiK EXISTS (found 2026-08-14) and TELEM1 is
              MAVLink, so the shell can run off-vehicle and free this port
              entirely -- see docs/mavlink_shell_over_sik.md. Unverified.
              The Jetson has 4 USB ports, all normally full -- unplug a
              RoboClaw to free one. Never the WiFi dongle; that is your SSH.

RC            Futaba T14SG + S.Bus receiver, 18 channels, working
              endpoints  min 1094   centre 1514   max 1934  (on ch8)
              gate rule  >= RC_MAX - RC_MARGIN = 1834
              index 7 = RL gate = ch8. MEASURED 2026-08-14, toggles
              1094 <-> 1934. This is the only free switch on the aircraft.
              index 8 = ch9 = DEAD, 1514 constant. There is no offboard
              switch. ATMO_RL_OFFBOARD_CHANNEL=-1 -- see "Gates" below.

              ONLY TWO SWITCHES ARE MAPPED. Measured, whole-transmitter sweep:
                ch8   1094 <-> 1934   free switch  -> the RL gate
                ch13  944 <-> 2084    kill, mirrored onto ch17
                ch3   throttle, reversed (stick down reads 1934), live
                everything else is constant and unmapped

              PX4 expects switches that do not exist on this transmitter:
                RC_MAP_KILL_SW  = 13   ch13  works
                RC_MAP_FLTMODE  = 14   ch14  DEAD, 944 constant
                RC_MAP_ARM_SW   = 15   ch15  DEAD, 1514 constant  <- blocks A1
              The inherited channel map was written for a Futaba T18SZ
              (see the comment in CATMO Node_tilt_controller.py). When the
              transmitter became a T14SG only the kill switch was carried
              across. That is why arm and flight mode do nothing.

              CHANNEL NUMBERING DIFFERS BETWEEN THE TWO TREES ON THIS ROBOT:
                ATMO_rl   0-based index    values[OFFBOARD_CHANNEL]
                CATMO     1-based channel  values[offboard_channel - 1]
              Both write `8` and mean channels one apart. Carlo's 8 is ch8
              (index 7); ATMO_rl's 8 is ch9 (index 8). ATMO_rl was looking one
              channel right of the only switch that exists.

Serial by-path (stable across reboots; by-id COLLIDES, both RoboClaws share a
descriptor):
              platform-3610000.usb-usb-0:2.1:1.0   RoboClaw A
              platform-3610000.usb-usb-0:2.2:1.0   RoboClaw B
              platform-3610000.usb-usb-0:2.4:1.0-port0   CP2102 (TELEM2)
              WHICH RoboClaw is tilt and which is drive is NOT yet established.
```

## Workspace layout

```
~/ATMO_rl              the working tree. branch `bringup`, tracking
                       WarZ43/ATMO branch `m4-lessons-port`.
                       Carries the FC-matched px4_msgs as its own commit.
~/ATMO/ATMO            the OLD tree: upstream mandralis/ATMO at 83b03ac.
                       Branch `jetson-local` preserves the vehicle's
                       uncommitted state (FC-matched px4_msgs + bench tuning).
                       Keep it. Do not work in it.
~/ATMO/px4_msgs_saved  an older px4_msgs snapshot. Superseded; not in use.
~/tools/mavlink_shell.py
```

Build: `cd ~/ATMO_rl/atmo_ws && colcon build --symlink-install`.
`--symlink-install` means pure-Python edits take effect without rebuilding.

## Startup, in order

```bash
# 1. agent  (defaults: /dev/ttyUSB0, 460800, domain 0)
cd ~/ATMO_rl && ./interface.sh
#    healthy = ONE create_participant, then a burst of create_topic, then quiet
#    repeated `session re-established` = baud mismatch or marginal link

# 2. anything else
cd ~/ATMO_rl/atmo_ws && source scripts/atmo_env.sh
ros2 topic list | grep -c fmu        # expect ~67
```

`pgrep -a MicroXRCEAgent` must show exactly ONE, at the right baud. A stale
agent at the old baud silently produces ~6 topics instead of ~67.

**That check is necessary and not sufficient.** It greps for the agent and so
cannot see foreign *nodes*. On 2026-08-14 a `systemctl --user` unit
(`atmo-startup.service`) had been launching `offboard_control` and
`tilt_control` from a third workspace, `~/CATMO/ws_athenaIII`, at every boot —
holding a RoboClaw open and spinning two drive wheels off an RC switch. Add:

```bash
ros2 node list                       # must be EMPTY before Stage 0a
fuser /dev/ttyACM0 /dev/ttyACM1      # both must be free
systemctl --user list-units --type=service --all | grep -i atmo
```

The `--user` matters: user units are invisible to a bare `systemctl
list-units`, which is why disabling the *system* `startup_robot.service` in an
earlier session looked like it had fixed this and had not. See
`docs/vehicle_changes_2026-08-14.md`.

**This actually happened, 2026-08-14.** Two agents were running at once:

```
2626 MicroXRCEAgent serial --dev /dev/serial/by-id/usb-Silicon_Labs_CP2102_...-port0 -b 921600
2807 MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 460800
```

Those are almost certainly the SAME CP2102 under two names, opened at two baud
rates, corrupting each other's framing. 2626 is the old tree's
`~/ATMO/ATMO/interface.sh`, which still hardcodes 921600. Check `pgrep` FIRST
every session, before trusting anything read over DDS. Resolution of this
instance was not observed -- re-check before relying on any earlier reading.

---

## What works

- **DDS end to end.** ~67 `/fmu/` topics, stable session at 460800.
- **RC at the flight controller and in ROS.** 18 channels, no lost frames.
- **Stage 0a monitor.** `hardware_rc_check.py --mode monitor` shows channels
  plus armed / nav_state / failsafe / preflight.
- **The unit suite.** No ROS or torch needed. Run it as
  `bash scripts/run_unit_tests.sh` (99 passed, 1 skipped as of 2026-08-17) —
  NOT with the workspace sourced. With build/install on PYTHONPATH the
  `atmo` package is on the path three times and pytest's launch_testing
  collection hook loses the newest module (`atmo.mocap_frames` failed to
  import while every copy on disk had it). The script strips PYTHONPATH,
  which is also the honest environment for a suite that needs no ROS.

## What is NOT done

Stage 0a still owes three gates (`docs/hardware_bringup.md`):

1. ~~`--mode identify`~~ **DONE 2026-08-14.** index 7 = ch8, toggles
   1094 ↔ 1934, and was left LOW. Kill = ch13 (+ ch17), confirmed against
   `RC_MAP_KILL_SW = 13`. Record which physical switch ch8 is — still not
   written down anywhere.
2. `--mode gates` — the RL gate reaches HIGH and low, **and reads low with the
   transmitter switched off**. That is the deadman and the only test of the
   fail-closed path. **This airframe is single-gate** (`ATMO_RL_OFFBOARD_CHANNEL=-1`),
   so the RL switch alone carries it.
3. ~~Kill polarity~~ **MEASURED 2026-08-14, in the mavlink shell over SiK:**

   ```
   lever physically UP    ->  ch13 =  944 us  ->  NOT killed
   lever physically DOWN  ->  ch13 = 2084 us  ->  KILLED
   ```

   **THE CHANNEL IS REVERSED RELATIVE TO THE LEVER**, and conflating the two
   caused a real inversion in `roboclaw_safety.kill_engaged`. "Kill high is no
   kill" describes the LEVER; in microseconds the same statement is "2084 is
   killed", i.e. active-HIGH, which is also PX4's usual convention. The
   active-low reading that came from it called a released switch engaged,
   which blocked preflight, and would have called an engaged switch released.
   Always say which axis you mean.

   **The field is `kill`, not `manual_lockdown`.** PX4 1.17.0 (git 8874d533)
   reports `armed / prearmed / ready_to_arm / lockdown / kill / termination /
   in_esc_calibration_mode` and has no `manual_lockdown` — even though the
   px4_msgs `ActuatorArmed.msg` vendored here still declares it. Every doc in
   this repo named the wrong field until now. `px4_kill_state()` tries `kill`,
   then `manual_lockdown`, then `lockdown`.

   Polarity is `ATMO_KILL_ACTIVE_HIGH`, default 1 (2084 us = killed).
   Independent of it, `kill_engaged` fails closed on lost RC, RC failsafe, an
   implausible pulse width, and a frame too short to contain the channel.

   Flight termination is **not** latched on this build: `Flight termination
   active` appears with `Kill engaged` and clears on `Kill disengaged`.

Nothing has been commanded to move. No motor has turned.

---

## Bugs found and fixed this session

Every one measured, not inferred. Do not re-litigate without new data.

1. **`parameters.py` could not be imported at all**, two independent causes:
   `T_max = 4*params_['kT']` computed 70 lines before `kT` exists (KeyError),
   and `getenv("ATMO") + "/..."` with `$ATMO` unset (TypeError). Every node
   reading `params_` went down with it. Fixed at the cause.
2. **The RC gates were not a deadman.** They ignored `rc_lost`/`rc_failsafe`,
   accepted any pulse width, and never expired — so powering the transmitter
   off left the companion believing both gates were raised. PX4's kill covers
   the rotors; the tilt and drive RoboClaws are outside it entirely, so these
   gates are the only thing that stops them. Now fail closed + 0.5 s watchdog.
3. **PX4 1.17 renamed some topics.** `vehicle_status` → `vehicle_status_v1`,
   likewise `battery_status`, `vehicle_local_position`, `home_position`.
   Per-message, not a blanket rename. Exactly one runtime subscription was
   broken. All names now go through `atmo/px4_topics.py`; a test forbids bare
   `/fmu/` literals.
4. **The mocap relay told PX4 the vehicle was stationary** — `velocity =
   [0,0,0]` is a measurement to the EKF, not an absence. NaN is how you decline.
5. **numpy 2.x removed `ndarray.ptp()`** (and `.itemset()`, `np.float_`,
   `np.NaN`, `np.in1d`, `np.trapz`). Grep for these in anything ported.
6. **All nine ATMO shell scripts hardcoded `/opt/ros/foxy`.** The robot is
   Humble. They now source `scripts/atmo_env.sh`, which detects the distro.
7. **RoboClaw `by-id` collides** — both boards enumerate identically, so a
   udev rule keyed on serial cannot work. Use `by-path`.
8. **`atmo_env.sh` forced `set -u` onto the caller**, so every later
   `source .../setup.bash` died on `AMENT_TRACE_SETUP_FILES: unbound variable`
   and the overlay silently failed to load.

## Stage A actuator map — MEASURED 2026-08-14

Established by commanding one motor at a time and watching. Directions were
predicted in writing before each run and then checked, per "the one rule".

```
BOARDS   by-path 2.1  = TILT    M2 = tilt motor (has the encoder), M1 unused
         by-path 2.2  = DRIVE   M1 = RIGHT wheel, M2 = LEFT wheel, no encoders

         This DISAGREES with ~/CATMO/atmo_ports.env (tilt=2.2, drive=2.1).
         His file is not wrong, it is stale: the boards were moved between
         USB sockets during this session. by-path names the SOCKET, not the
         board, which is what makes it stable across reboots and wrong after
         a replug. Re-measure whenever those plugs move.

DRIVE    ForwardM1  -> RIGHT wheel drives vehicle FORWARD
         BackwardM1 -> RIGHT wheel drives vehicle BACKWARD
         ForwardM2  -> LEFT  wheel drives vehicle BACKWARD
         BackwardM2 -> LEFT  wheel drives vehicle FORWARD

         The sides are MIRRORED. Verified by prediction:
           both raw Forward          -> SPINS          (predicted, confirmed)
           right Fwd + left Backward -> STRAIGHT FWD   (predicted, confirmed)

         So drive_controller_hardware.py is CORRECT as written. Its
         move_left_wheel(+)=ForwardM2 / move_right_wheel(+)=BackwardM1
         asymmetry exists because the motors are mounted mirrored, and the
         negation in update() (lin_vel = -map_speed(drive_speed)) makes
         positive drive_speed = forward and positive turn_speed = a proper
         differential spin. Do not "fix" the asymmetry.

TILT     Forward  = AWAY from fly config, downhill, breaks away at duty ~45
         Backward = TOWARD fly config, uphill, needs duty ~100
         Full sweep verified: 0 -> 31.7 deg and back to 1.1 deg, smooth,
         angle tracking monotonically.
```

### Homing the tilt — there is no automatic homing anywhere

Neither tree has a homing routine. Carlo's `SetEncM2(0)` at node start is an
*instruction to the operator* ("always start robot in fly configuration"), not
something the code enforces, and his other two paths are the limit switch
(which does not work, below) and an RC-triggered re-zero.

Procedure that works, measured:

```
1. Drive BackwardM2 at duty ~60 until the speed collapses -> the FLY hard stop
2. SetEncM2(address, 0) there.  FLY = 0 counts = 0 deg.
3. Forward from there increases the angle.
```

**Do not home at duty 100.** It wedges the arm into the stop, and escaping the
wedge then needs duty ~80 — duty 55 will not move it and reads as a dead
motor. This cost an hour of misdiagnosis.

### Three things on this hardware that do not work

- **The limit switch does not zero the encoder at home.** Driven into the fly
  stop, the count stayed at 9503 instead of resetting. `SetPinFunctions`
  returns `ack=False` against v4.4.9 with this driver, so the "M2 goes to zero
  at home" comment in *both* trees describes a mechanism that was very likely
  never configured. It is the hardware guard everyone assumes exists.
- **`ReadCurrents` on the tilt board is unusable.** It reports **-0.61 A** on
  a terminal with no motor attached, and reads flat across duty 20/40/60. Every
  current-based inference in this session was wrong because of it. Use the
  encoder and `ReadSpeedM2`.
- **The left wheel is slower than the right at equal duty.** Nothing closes a
  loop on wheel speed — the drive board has no encoders — so a straight `drive`
  command will curve, and it will look like a policy problem. Quantify it with
  a measured straight-line run before blaming anything upstream.

## 2026-08-17 (evening): actuator session — wheels pass, tilt blocked by hardware

**RoboClaw kill lockout, decoded and reproducible.** Every battery power-up
starts BOTH RoboClaws in a signal lockout (error 0x23040000 drive /
0x23840000 tilt) — commands ACK, 11.7 V on the rails, zero motion, zero
sound. The clear recipe: transmitter ON, then CYCLE the RC kill lever once.
Verified live: the 0x20xxxxxx bits drop on the flip. This is the deadman
failing closed, not a fault. Check `ReadError` FIRST when actuators are
silent; it distinguishes lockout (0x20xxxxxx) from a driver fault (0x40/0x80)
in one read.

**Wheels verified** (suspended, props off): right wheel moves at duty 30,
left needs ~40 and stalls against resistance sooner — consistent with the
known left-side weakness. Both directions confirmed by eye.
LATER THE SAME SESSION: the left-side resistance was traced to a mechanical
cause and fixed, so the 1.35x left trim was removed
(`ATMO_DRIVE_LEFT_SCALE` default back to 1.0). Wheels also spin faster
post-fix; speed was never benchmarked, so no calibration is invalidated.
Re-trim only from a measured straight-line run if the vehicle curves.

**Tilt: electrically open, then mechanically suspect.** The tilt M2 channel
reports a driver fault (0x…80) the moment it is commanded and clears at
rest — signature of an open motor circuit. Cause per Taoran: hardware kill
switches at the tilt travel ends cut the motor circuit, and their firmware
assumes on power-up that the last motion was UPWARD (i.e. it sits at the
top switch), inhibiting that direction. Power cycling in DRIVE (on the
drive-end switch) makes the assumption wrong and creates a DEAD ZONE:
up blocked by the wrong assumption, down into the stop — stuck both ways.
Escape: manual control via Basicmicro Motion Studio, below the interlocks.
Deployed countermeasure (379fd74): `ATMO_TILT_MAX_DEG` default 85 and
`ATMO_TILT_HOME` default `drive` — software never commands the arm onto the
drive-end switch again, and every boot assumes worst case until the arm
measurably descends.

**Worm gear noise: RESOLVED as lack of grease** (operator-confirmed the
noise predates this session; nothing was damaged today — the motor never
turned under the inhibit, encoder never moved a count). Fix: grease the
worm drive with a proper gear grease (not thin oil) before the next tilt
session.

**Encoder state: WRONG.** An automated homing attempt zeroed the tilt
encoder near the drive posture believing it was at fly. After the
mechanical inspection, re-home at the fly stop and re-zero before trusting
any angle.

## 2026-08-17 (night): drive RoboClaw dead — root cause is the power architecture

**Symptom chain, in the order it presented:** wheels stutter (rapid stop/
start) under Motion Studio duty commands; status showed USB disconnects AND
overcurrent; board LEDs restart whenever the wheels run under drivetrain
load. Acquitted one by one, each by measurement: RC kill line (no change),
current limits (stutter survives Max Current = 50 A), settings (survives
factory defaults), USB cable (same cable clean on the tilt board), wiring
(resistance fine, nothing warm), mechanical load (free-spin clean at all
duty), the motors (all three identical). The decisive swap: **the wheel
motors run clean on the tilt board.** The drive board is defective — it
reboots under motor load.

**Root cause (and why it will recur if unfixed): all three motors are
powered THROUGH the 12 V regulator.** A regulator cannot absorb reverse
current, so every deceleration/reversal regenerates the wheels' kinetic
energy into a rail with no sink; the board's input stage eats the spike,
cumulatively. DEMONSTRATED, not theorized: hand-spinning the wheels lights
the regulator's LED with the system unpowered — the back-feed path through
the drive board's body diodes onto the shared rail, visible to the eye.
The worm-gear tilt cannot be back-driven, which is why its board survived.
Likely finishing blows: energetic hand-spins during the drivetrain repair
(raw generator into unpowered silicon) and battery cuts during motion.

**THE FIX, for whoever does it: motor power direct from the battery.**
- Both RoboClaws' main power straight off the pack, through the existing
  main power/kill switch, with an inline ~15-20 A fuse. 16 AWG. One clean
  crimped/soldered distribution point, photographed.
- The 12 V regulator keeps ELECTRONICS ONLY. Motors never live behind a
  regulator: the battery is the regen sink, by physics, forever.
- Patch alternatives (series diode + bulk cap + TVS) were considered and
  rejected: they embed energy-sizing assumptions a future operator will
  unknowingly violate.

**Standing rules from this:**
1. NEVER hand-spin the wheels faster than a crawl while motors are
   connected -- a spun motor is a generator wired into the electronics.
2. Never cut battery power while anything is moving.
3. A replacement 2x7A must not be installed on the old architecture.

**Fallback demo if no replacement board arrives in time:** the flight
mission barely uses wheels (drive phase holds station at zero speed), so
takeoff-hover-land survives; the closed-loop driving demo does not.

## 2026-08-17 (late night): tilt VERIFIED through the full stack — with lessons

**The tilt action test passed**: policy node -> tilt node -> RoboClaw -> arm,
autostart mode (`ATMO_RL_ACTION_AUTOSTART=1` on both nodes: gate is RC
liveness + kill released, no RL-switch choreography; every fail-closed path
kept). Commanded 0.393 rad/s, measured angle tracked it cleanly.

**But the run OVERSHOT to the physical 90 deg kill switch** while the node
read 57 deg: the January calibration table's 26325-count span is stale —
the true span, measured by motion (fly stop to drive-end switch), is
~22200 counts. The 85 deg bound was watching a fictional angle. FIXED in
e93d1eb (table scaled to the measured span; `ATMO_TILT_SPAN_COUNTS`
overrides after future drivetrain work). The table's absolute shape still
deserves a protractor pass.

**Dead-zone escape, learned the useful way:** hitting the switch while the
board stays POWERED does not trap you — the inhibit is a power-ON latch.
Drive back out immediately (BackwardM2, duty 80 works right at the stop)
and NEVER power cycle while on the switch.

**Uphill breakaway is posture-dependent and larger than documented:** duty
80 breaks away at the stop but NOT mid-travel (~74-87 deg, where gravity
load peaks) — mid-travel uphill needs ~100. Downhill moves at 50. The
homing recipe that works: up at 100, drop to 65 ~3000 counts before the
stop (no wedge), stall-detect, zero. Open question for flight: whether the
tilt node's duty mapping can break away uphill mid-morph under gravity
(rotor thrust unloads the arm in flight, so the regime differs — watch it
in the first restrained morph).

**State at session end:** arm parked at 45 deg on a TRUE fly-zeroed
encoder; safe to power off (no switch contact). Wheels disabled in code
(dead board). Tilt is a verified, correctly-calibrated axis.

## Gates: this airframe is single-gate

`ATMO_RL_OFFBOARD_CHANNEL=-1`. The two-gate ratchet assumed a second switch
that this transmitter does not have and never had — ch9 has been 1514 for
every frame of every run. With `-1`, `_action_test_timer` cycles the RL switch
**twice** instead (low → high → low → high), which keeps the property that
matters: the sequence starts from LOW, so a switch left HIGH when the node
starts cannot run anything and one flip is never enough.

**What did not change: the deadman.** `rc_lost`, `rc_failsafe`, implausible
pulse width and the 0.5 s watchdog all still drop the gates unconditionally.
A test asserts none of those paths reference `OFFBOARD_GATE_ENABLED`, so the
config flag cannot switch off the only thing that stops tilt and drive.

Set it in the environment before launching, alongside the rest:

```bash
export ATMO_RL_OFFBOARD_CHANNEL=-1
```

If a future airframe has two switches, unset it and the two-gate ratchet
returns with no code change.

## Open items, roughly by cost of getting them wrong

- **`task_observation_dim` is 4 in the runtime, 5 in the exported contract**
  (528 vs 529). The 5th is the signed phase-event timer. The contract check
  **blocks closed-loop `policy` mode** until reconciled — deliberately. Which
  side moves depends on the checkpoint you actually train. `shadow` and the
  test modes warn and continue.
- **No policy checkpoint exists** anywhere on the robot or the workstation.
  Whatever is used must be exported to `.npz` (`scripts/export_policy_npz.py`,
  run where torch is) — the Jetson has no torch, so `.npz` is the only path.
- **16° of UNCOMMANDED tilt drift. Blocks A3 sign-off.** Measured 2026-08-14:
  a `Forward` leg ended at raw −7233 (31.7°); 1.5 s later, after `stop_all`,
  the next leg started at raw −3705 (15.9°). The arm moved ~3528 counts with
  nothing commanding it. A non-backdrivable worm gear that cannot be moved by
  hand should not do that. Either the arm is not actually held when unpowered,
  or the encoder loses count. Until this is understood, commanded tilt
  position is not held between commands, which the policy assumes it is.
- ~~**The tilt encoder→angle table needs recalibration.**~~ **FIXED
  2026-08-14** by porting Carlo's corrections. It was wrong four ways at once,
  and the net effect was that `get_current_tilt_angle()` returned a **constant
  90°** for every count ever measured:
  1. *Sign* — the encoder counts negative away from fly; the raw count clamps
     below the table's first entry, so `interp` returns a constant. Negate it.
  2. *Table orientation* — collected from the opposite mechanical end. Reflect
     both axes.
  3. *Origin* — zero is the FLY configuration, not drive.
  4. *Command sign* — `spin_motor` mapped `tilt_speed < 0` to `ForwardM2`,
     inverting every tilt command the policy would issue.
  Verified: raw −9699 → 43.0° against an observed 30–40°, where the old code
  said 90.0°. `ATMO_TILT_DEBUG=1` prints raw count, lookup count and angle.
  The table's absolute scale still deserves a protractor pass.
- ~~**Which RoboClaw is tilt and which is drive is unverified.**~~ **SETTLED
  2026-08-14 by motion:** 2.1 = tilt, 2.2 = drive. See the Stage A map above.
  Note that read-only identification does NOT work — both boards report
  `USB Roboclaw 2x7A v4.4.9`, both encoders read 0 at rest, and with both
  plugged in there is exactly ONE `by-id` symlink, so the second board has
  none at all. Only commanding one motor at a time distinguishes them.
- **`RC_MAP_ARM_SW = 15` points at a dead channel.** ch15 reads 1514 always,
  so the RC arm switch cannot arm. A1 needs the vehicle armed for the
  lift-and-kill proof, so this blocks A1 independently of `preflight ok`.
  Either map arm onto a real channel in PX4, or arm by MAVLink command.
- **`RC_MAP_FLTMODE = 14` likewise.** ch14 is 944 constant; `nav_state` is
  stuck at `stab` because there is no mode switch to move it.
- ~~**`nav_state = offboard` while disarmed**, with nothing commanding it.~~
  **RESOLVED 2026-08-14.** Something *was* commanding it:
  `/offboard_controller_node`, autostarted from `~/CATMO/ws_athenaIII` by the
  user unit `atmo-startup.service`. Disabled; `nav_state` should be re-checked
  now that it is gone. This is the same root cause as the unexplained wheel
  motion — see `docs/vehicle_changes_2026-08-14.md`.
- **`preflight ok: False`** — expected indoors, no position estimate. Blocks
  arming, so it blocks A1. Resolves when mocap feeds the EKF (Stage C).
- **`/fmu/in/tilt_angle` and `/fmu/out/actuator_armed` are absent from the
  firmware's `dds_topics.yaml`.** So PX4 never learns the arm tilt, and kill
  state never reaches a rosbag. One firmware rebuild covers both.
- **The PX4 quaternion convention for mocap is unresolved** — `atmo_legacy` vs
  `composed` are different rotations. Settle in Stage C by motion.
- **OptiTrack has not been touched.** Bridge, docs and Stage C gates exist and
  are untested against a real rig.

---

## 2026-08-17: policy verified off-robot; the WiFi dongle is the enemy

**The policy exists and passes.** `~/policies/atmo_combined_stage1_policy.npz`
on the Jetson (copied to the laptop and verified there): NumpyActor obs=529
act=7, from `last_atmo_combined_stage1_ep_1800_rew_890.5606.pth`, sha256
`53e4b221...` embedded. 2000-random-obs forward pass all finite, output range
±~2.4. The 528-vs-529 contract mismatch is RESOLVED in code: contract (529/5),
`CombinedStage1Config` (529/5) and this policy all agree, so closed-loop
`policy`/`ground` modes are unblocked. Workspace builds clean on the laptop
(`PYTHONNOUSERSITE=1` needed there — a user-site setuptools breaks
ament_cmake_python; do not `pip install --user setuptools` on any machine that
builds this). Unit suite 99 passed / 1 skipped.

**Headless closed-loop drive sim** (laptop, real policy + real
CombinedObservationBuilder/LandingActionAdapter + point-mass skid-steer
plant): rotor gate held at exactly 0 physical thrust in DRIVE across every
run; all actions finite/bounded over ~2400 steps; no divergence under offset,
yaw kick, or 0.25 m/s cruise. Two residuals the harness cannot adjudicate
(invented plant gains/frames), carried to the bench as the FIRST two
questions of the next hardware session:

1. A steady ~0.2 m/s backward creep in drive-only hold — my plant's sign
   convention or a real drive-sign issue. `ground --drive-speed 0.0` settles
   it: any creep on real wheels is the policy's.
2. Tilt convention: seeded at 0 the policy drives tilt to 1.571 and holds;
   seeded at 1.571 it drives to 0. One is "tuck to drive", the other is
   wrong. `action_map_check.py --slot tilt` plus the encoder frame
   (0 = fly at the homed hard stop) decides which.

**WiFi RESOLVED (2026-08-17, evening): it was never just the dongle.** Three
stacked problems, each measured:

1. The RTL8188EUS dongle (2.4 GHz only, floor bitrates at -47 dBm) — replaced
   with a TP-Link Archer T2U Plus (RTL8811AU), driver built from
   morrownr/8821au-20210708 on the Jetson (headers preinstalled). 5 GHz at
   434 Mbit/s, 60 MB in 1.9 s.
2. The laptop's fast-discovery-server dies silently between sessions — an
   empty `ss -lun | grep 11811` while both sides publish into the void.
   Check it FIRST when topics vanish.
3. **A scan storm: the radio left the channel every ~7 s** for a full 39-channel
   sweep, punching 2-6 s holes in the stream (up to 22 dropouts per 2-minute
   window) while signal and bitrate looked perfect. Zero deauths — the
   neighboring group's "IMSS attacks our router" theory does not apply here.
   Fixed by the combination (applied in this order, storm ended after the
   last step + NetworkManager restart):
   - pin the profile to the 5 GHz BSSID:
     `nmcli c modify dont_trust_sean_devey 802-11-wireless.bssid AC:15:A2:6C:24:7C`
   - `802-11-wireless.powersave 2` (disable)
   - autoconnect OFF on the stale "Caltech Secure" and "linksys_SES_57619"
     profiles
   - `/etc/NetworkManager/conf.d/90-no-scan-rand.conf` with
     `[device] wifi.scan-rand-mac-address=no`, then
     `systemctl restart NetworkManager`
   After: 0 scans in 45 s, still pinned to 5745 MHz.

The mocap_bridge ALSO crashed on its own rate-warning path during this
(rclpy caches log severity per source line; picking info-vs-warn into a
variable and calling from one line throws the first time the rate crosses
the threshold — i.e. the bridge died precisely when the link degraded).
Fixed in e7f2b0b; the endurance test found it.

**Final link state (2026-08-17, accepted):** after all fixes, the bridge
output shows ~5 gaps of ~100 ms per 2-minute window (mean 120 Hz, p95 10 ms).
Kernel-vs-userspace attribution was not completed. ACCEPTED for flight
testing: the policy's freshness cut is 250 ms and the EKF tolerates
occasional 100 ms vision gaps; the catastrophic modes (multi-second wedges,
scan-storm holes, bridge crash on degradation) are all fixed and verified.
One more fix in this pile: **split the laptop and Jetson across the router's
two 5 GHz radios** (laptop BSSID AC:15:A2:6C:24:7B @5220, Jetson
AC:15:A2:6C:24:7C @5745) — with both on one radio, the AP relays every
packet across the same channel twice and its queue produced ~100 ms bursts;
split-radio ping went from 16 spikes/60 s to 1. Also: never `modprobe -r
8821au` on a live system (locks up / needs power cycle); reboot instead.
mocap_bridge got gc.freeze() + a distant gen-2 threshold (f599fb1) after
~90 ms GC stalls every ~25 s were measured on the Jetson.

**The original dongle post-mortem, kept for the record:** Two on-network
drops, both starting minutes after DDS traffic began, both self-recovering
when load stopped; the laptop on the same router never blinked. Signal
excellent (-47 dBm) but rates pinned at the floor (6.5/1 Mbit/s), and a
60 MB pull moved 311 KB in 65 s (~40 kbit/s). The link cannot carry the
120 Hz mocap stream, and it can never pass the sub-40 ms worst-gap gate.
Fix in progress: transplant the M.2 Key E WiFi module (RTL8822CE, dual-band,
in-kernel rtw88_8822ce driver) from the new Orin Nano devkit into the old
Jetson's empty Key E slot. Backup: the new devkit, freshly flashed —
see `docs/jetson_fresh_install.md` for the full from-scratch install order
and the list of things that must be re-measured after any board swap
(RoboClaw by-path identities above all).

Mocap laptop side is proven live: Motive streams rigid body **M4** (not
m4_base — pass `--mocap-body M4`), z-up, 120 Hz measured on
`/vrpn_mocap/M4/pose`, discovery server at the laptop's 192.168.0.4:11811.

## A note on where this code came from

Most of the tooling was ported from `~/Documents/m4-direct-rl` (the White M4
deployment repo, stopped because its Dynamixels could not hold the arms against
rotor thrust). The methodology transfers; the answers frequently did not.
Domain 42, ROS Foxy, and udev-by-serial were all imported and all wrong here,
each costing a debugging round.

Where the docs say "must equal `UXRCE_DDS_DOM_ID`" rather than naming a number,
that is why. Prefer that form.

## Shared-vehicle changes

This vehicle has other users. Everything altered on it -- PX4 params,
systemd, git state, physical connections -- is listed in
`docs/vehicle_changes_2026-08-14.md`, with how to revert each. The one
that breaks the previous workflow: `SER_TEL2_BAUD` is now 460800, and
`~/ATMO/ATMO/interface.sh` still says 921600.

## Reading order for a fresh session

1. This file.
2. `docs/px4_topics.md` — the measured topic inventory and transport config.
3. `docs/hardware_bringup.md` — the staged gates, Stage 0 onward.
4. `docs/mavlink_shell_over_sik.md` — the USB port budget, and how to get a
   shell without spending a Jetson port. Read before Stage A.
5. `docs/migration_from_m4.md` — what came from where, and what did not.
6. `docs/optitrack_bringup.md` + `docs/optitrack_session_checklist.md` — when
   the mocap rig is available.

## STANDING RULE (2026-08-17, operator-imposed): NO REMOTE TILT ACTUATION

Claude/remote sessions are FORBIDDEN from commanding the tilt motor. All
tilt motion is operator-performed (Motion Studio or operator-run scripts,
hands on the kill). Reason: remote command loops react in seconds where an
operator reacts instantly; sustained stall at duty during remote breakaway
attempts overheated the tilt motor on 2026-08-17 (hot to the touch;
recoverable). Remote READS of the tilt board (status, encoder, voltage)
remain fine. Any future automated tilt motion must carry a hard rule:
if the encoder has not moved within 0.7 s of a command, cut to zero and
back off -- never hold a stalled motor at duty.

## 2026-08-18 (arena night, final): mission ran, frames verified, morning list

**THE FULL MISSION RAN CLOSED-LOOP ON THE VEHICLE** (ground profile, route
full, real policy, real mocap, tilt actuating, rotors cut, wheels dead):
engaged -> drive_to_takeoff (+2.0s) -> takeoff_to_flight (+5.0s) ->
flight_to_landing (+3.0s) -> landing_to_drive (+3.0s), all on schedule,
rosbag in bags/atmo_ground_*. Post-landing drive hold pushed tilt to the
85 deg bound and was correctly blocked. CAVEAT: run was on an UNHOMED
encoder frame (assume-top) -- angles offset; rule below.

**C1 pose frame VERIFIED by motion (laptop-local bridge + check, vehicle
unpowered):** z-up confirmed (lift -> z up; suspended rest z ~0.85 m),
x/y consistent -- an apparent "x reversed" was the vehicle heading ~120 deg
from Motive +X (nets x -1.23 / y +0.84 decompose exactly). Yaw-CCW
confirmation still owed. The pose check's printed expectations assume yaw
~= 0; at any other heading, decompose by the vehicle's heading first.

**The evening's failure chain, each solved:** dead agent after every Jetson
reboot (start it FIRST); TELEM2 GROUND WIRE broken at the solder joint
(session forms, data dies -- 37 kB/s tx at FC, trickle at Jetson;
resoldered); agent/nodes in different DDS worlds (interface.sh now carries
ATMO_DDS_DISCOVERY_SERVER into the agent); RoboClaw power-up lockout after
every battery cycle (kill-lever cycle clears); tilt motor OVERHEATED by
sustained remote stall attempts (recoverable; see the standing rule -- no
remote tilt actuation, and never hold a stalled motor at duty).

**Worm gear drag is elevated vs 8/14** (duty 60-70 then, 100+ now, worse
mid-travel). Suspects, in order: reassembly alignment from the repair
(center distance / preload), shock from the 90 deg overrun, fresh grease.
Cold-morning check: loosen the worm mount a quarter turn, feel the shaft
through a slow full cycle (periodic tight spot = bent shaft; uniform =
depth/preload), re-snug while running slowly.

**MORNING CHECKLIST (no OptiTrack until 19:00):**
1. Fresh battery. Power up -> transmitter on -> CYCLE KILL LEVER.
2. Agent first (interface.sh; exports the discovery server itself now).
3. Worm alignment check, then Motion Studio homing to FLY, SetEncM2(0)
   there. OPERATOR ONLY. Short bursts, cold motor.
4. Kill test (lift action test), then the ARMING sequence -- how does this
   vehicle arm with no position estimate? (Carlo's PX4 notes; never tested.)
5. Rotor spin-up props off: motor map + spin directions off convention.
6. 19:00 arena: C2 twist, yaw-CCW confirm, then the flight decision.

RULE: no tilt-actuating session without a verified homing first.

## 2026-08-18 addendum: tilt stall root cause CONFIRMED by the operator

Cold motor + full duty (126) up: moved easily, 3 s operator-run test.
The evening's "increasing resistance" was THERMAL SELF-SABOTAGE: an
incremental duty ladder (60 -> 80 -> 100 -> 126) heats the motor at
sub-breakaway duties without moving it, sapping ~0.4%/degC of torque, so
each rung arrives weaker than the last would have from cold. RULE: command
breakaway at the full known-good duty on the FIRST attempt from cold;
never ladder up; if no encoder motion within 0.7 s, stop and let it cool
-- retrying hot only digs the hole. Worm drag vs 8/14 may still be partly
real (alignment check remains on the morning list) but the dramatic
"nothing moves at any duty" episodes were heat.

## 2026-08-18 (00:30): REAL-WORLD DATA CAPTURED; rotor plan for the morning

**The 5/5/5 mission ran with tilt actuating and is RECORDED**:
bags/atmo_ground_20260818_002029 (.db3 + metadata.yaml, closed cleanly).
Takeoff 5 s / hover 5 s / land 5 s via the new ATMO_RL_*_S envs, real
policy, real mocap, arm started at 85 deg. Operator: "the transformation
was incredibly clean despite the bad position." Two aborted earlier bags
(001219, 001827) are junk from mis-set switches; ignore them.

Confirmed cause of the aborted first attempt: THE AGENT WAS DEAD (again,
post-reboot). The rule is now unavoidable: agent first, verify
/fmu/out/input_rc flows, THEN launch anything.

## ROTOR SEQUENCE (morning, props OFF, in this order)

0. **POWER COMPATIBILITY GATE, before anything spins: the current packs are
   6S and the ESCs are NOT rated for 6S.** A 6S spin attempt kills ESCs
   instantly. Read the ESC rating (label/datasheet), source the correct
   pack, and confirm the ESCs feed DIRECTLY from the flight battery (not
   the 12 V regulator). NO rotor work until the right voltage exists.
1. Kill test (lift action test, first half): armed, rotors at idle, kill
   lever -> rotors die instantly. Mandatory gate.
2. Arming path: never tested. RC arm switch maps to a dead channel; the
   vehicle flies WITHOUT the EKF, so PX4 must arm with no position
   estimate -- Carlo's PX4 notes hold the config answer. The stack already
   sends the MAVLink arm command; untested whether PX4 accepts it.
3. Motor map by touch: barely-above-idle, finger/screwdriver on each bell
   to feel which corner answers which command. Map is convention-only
   until this passes.
4. Spin directions vs the expected CW/CCW pattern (tape flag or visual).
5. Only after 1-4: props on, restrained, then the 19:00 arena slot and the
   flight decision.

## 2026-08-18 (arena day, evening): THE VEHICLE FLEW; mixer handedness UNRESOLVED

**Transport root cause found and fixed.** The all-day ~1 s stream stalls
(RC + odometry freezing together, ~975 ms each, several per minute) were
TELEM2 serial saturation: payload had crept to 38.4 kB/s against 460800
baud (~46 kB/s raw), and `uxrce_dds_client status` showed cycle max
1.01 s -- the FC-side client blocking when the TX buffer filled. NOT the
resoldered cable (gaps identical after a fresh recrimp; wiggle correlation
was coincidence). Fixed: `SER_TEL2_BAUD = 921600` (param saved on FC) and
the agent started with `ATMO_XRCE_BAUD=921600`. Soak after: ZERO gaps in
2.5 min, and a full mission under load with none. interface.sh default is
still 460800 -- update it or always pass the env.

**Arming without position: answered.** `policy` profile with
`--px4-relay on` (the default) feeds the EKF mocap it can't fuse
(quaternion convention still unresolved) -> `pre_flight_checks_pass:
False`, `local_position_invalid`, ARM_DISARM result=1 forever. With
`--px4-relay off` PX4 arms cleanly on the stack's MAVLink command
(result=0) and flies direct-actuator offboard with no position estimate.
The EKF is not in the policy loop (pose comes from mocap_bridge), so
relay-off is the flight configuration until the EV convention is settled.

**Props-off full-mission bench run (policy profile, virtual pose,
--props-off-bench): PASS.** Armed, all transitions on schedule, rotors
under policy command, auto-disarm at terminal. The arming path and
direct-actuator pipeline are proven end to end.

**Flights (all on the real mocap, relay off, props on, tethered):**
- 200413 (pre yaw flip): ~9.5 s airborne, ~0.4 m of the commanded 1 m,
  mild yaw bursts (~4 rad/s) concentrated around the landing phase;
  survivable, landed under the terminal handoff.
- Operator + team then observed all four motors spinning OPPOSITE the
  spec handedness; yaw column + spin vector were flipped in
  rl_landing_stage1_runtime (deployment-side only).
- 202503 (post flip): lifted off, pitched up within ~2 s, snagged the
  tether carabiner. No yaw runaway (operator-observed).
- 205315 (post flip): lifted off, rolled to +81 deg inside the first
  second, killed instantly. The 40 s at z~4.9 m afterwards in the bag is
  the vehicle being hoisted on the tether, not flight.

**THE OPEN QUESTION (first gate of next session): is the yaw flip right?**
Two instant attitude departures post-flip vs one flyable flight pre-flip.
The flip also negates _ROTOR_SPIN_DIRECTION which feeds the observation
wrench features -- if wrong, it corrupts roll/pitch stabilization exactly
like what was seen. Against that: the team's direct observation of motor
spin directions, and the pre-flip flight's own yaw bursts. DECIDE BY
MEASUREMENT: restrained roll/pitch/yaw action tests (props on), noting
which side physically dips per sign, THEN the ulog reconstruction below.
Do not fly, and do not touch the mixer signs again, before that.

**Data preserved on the laptop** at ~/ATMO/flight_data_20260818/: all
2026-08-18 rosbags (294 MB) incl. the three flight attempts and last
night's 5/5/5 ground run. PX4 ulogs still on the FC microSD (one per
armed session; FC clock may stamp them 1969 -- match by order/size).
Rosbag caveat: /fmu/in/actuator_motors recording was flaky (2009 msgs in
200413, 0 in 202503, 533 in 205315) -- use the ulogs for rotor commands.
The policy's raw 7-dim action is NOT logged anywhere; reconstruct via
(a) inverting the mixer + motor filter from ulog rotor commands, or
(b) offline replay: rebuild observations from the recorded mocap/odometry
and forward-pass the .npz (validate against 200413's recorded rotors
first). TODO: publish /atmo/rl/action from the node (3-line change).

**Altitude anomaly noted for the replay:** 200413 reached only ~0.4 m of
the 1.0 m command. (205315's apparent 4.9 m was the tether hoist.)

**Assorted fixes that should not be relearned:** the tilt node's boot
SetEncM2(0) wiped verified homing until ATMO_TILT_PRESERVE_ENC=1 (commit
7f557a4); mission phase envs did not survive tmux until forwarded by
atmo_session.sh; the agent MUST carry ATMO_DDS_DISCOVERY_SERVER or the
stack sees mocap but no RC (silent gate failure); Motive at the arena was
VRPN at 169.254.243.238 via link-local on the wired NIC (laptop needs a
169.254/16 alias); mocap 120 Hz held clean through the WiFi at the arena
when measured idle, but the 202503 flight window shows repeated 400-800 ms
holes in the recorded stream -- re-verify the link in the final minute
before every attempt.
