# Hardware bring-up test suite

Staged gates for taking a trained ATMO policy from a checkpoint to flight. Each
stage has a precondition, a procedure and a pass criterion. Stages are ordered
so that a failure at stage N cannot be caused by something stage N-1 would have
caught.

The methodology is carried over from the White M4 bring-up
(`~/Documents/m4-direct-rl/docs/hardware_bringup.md`). The stage contents are
ATMO's, because the actuators are different — ATMO tilts on a RoboClaw-driven
geared motor with an encoder and a limit switch, and drives on RoboClaw wheels.
There are no Dynamixels here, which is the reason for the platform change: the
White M4's servos could not hold the arms against rotor thrust.

`src/atmo/HARDWARE_TESTS.md` is the original launch-argument reference for the
action tests. This document is the order to run them in and what must be true
first.

## The one rule

**A test that shows you the number before you predict it is not a measurement.**
Every script here prints its expectations first. Read them, commit, then move
the hardware. The m4 bring-up repeatedly recorded a "PASS" that only confirmed
that motion occurred, not that its direction matched the printed word — and
then treated that prose as evidence against a real tick-level measurement.

## Running a stage

```bash
bash scripts/check_host.sh          # every session, first thing
./atmo_session.sh <profile> [...]   # builds the tmux session
# STOP: Ctrl-C in the stack window. Also: the kill switch, or drop the RL gate.
```

**Ctrl-C in the stack window is the stop.** Every node zeroes its actuators in
a `finally` before teardown, so SIGINT is the safe path. The kill switch and
dropping the RL gate stop motion too, and all three are equivalent for safety.

Still do not `tmux kill-session` while the bag is running: it writes the `.db3`
but never `metadata.yaml`, and rosbag cannot reopen that. Ctrl-C the bag window
first if you want the data.

---

## Stage 0 — RC channels, gates, and kill polarity

**First test of every session, before anything else, every time.**

Stage 0 splits in two, and only the second half needs the vehicle to be capable
of moving:

- **0a — observation.** Entirely read-only. `hardware_rc_check.py` creates no
  publishers, so it cannot move, arm, or command anything. Safe to be the very
  first thing run on a vehicle you do not yet trust.
- **0b — the kill proof.** Requires motors turning, so it is fused with A1.

### 0a — read-only

Precondition: propellers OFF, vehicle restrained, transmitter on, PX4 powered,
uXRCE-DDS agent running (`./interface.sh`).

```bash
python3 scripts/hardware_rc_check.py --mode monitor    # live channels + PX4 state
python3 scripts/hardware_rc_check.py --mode identify   # which index is which switch
python3 scripts/hardware_rc_check.py --mode gates      # the two channels the runtime gates on
python3 scripts/hardware_rc_check.py --mode kill       # guided polarity measurement
```

**Kill polarity must be measured in the mavlink shell on this firmware.**
`/fmu/out/actuator_armed` is not published over DDS here, so `--mode kill`
can only infer from `vehicle_status`, and that updates on transitions rather
than continuously. Use the shell, which reads uORB directly and works while
disarmed:

```
listener actuator_armed 20
```

Toggle the switch and record which PHYSICAL position gives
`manual_lockdown: true`. See docs/px4_topics.md.

**Pass, all four:**

1. RC arrives, with the channel count the transmitter should be sending.
2. Each physical switch is matched to a channel index, and the offboard and RL
   gates (`ATMO_RL_OFFBOARD_CHANNEL`, `ATMO_RL_CHANNEL`) are the ones you
   intend. The gate rule is `>= RC_MAX - RC_MARGIN`, i.e. 1834 us by default.
3. Both gates reach HIGH and low, **and both read low when you switch the
   transmitter off.** That last one is the deadman.
4. PX4 reports killed in exactly one of the two kill-switch positions, and you
   have written down which physical position that is.

Two failure modes this exists to catch, both real on the m4 vehicle:

- **Inverted polarity.** PX4 read channel HIGH as KILLED while the code called
  that position "released". Every rotor test looked silently dead.
- **Latched flight termination.** `nav_state = TERMINATION` does not clear by
  toggling the switch or by re-arming — only a flight-controller reboot clears
  it. The `kill` mode reports both positions as killed and says so.

Do not adopt a different polarity in code to match an assumption. Adopt PX4's;
it is the shared vehicle configuration.

### The kill switch does not cover everything

PX4's kill cuts the **rotors**. The tilt and drive RoboClaws are commanded only
from the companion, so they are outside it. The RC gates in
`rl_controller_hardware` are what stops those, which is why they must fail
closed — they now drop on `rc_lost`, on `rc_failsafe`, on an implausible pulse
width, and on the stream going silent for `ATMO_RL_RC_TIMEOUT_S` (0.5 s). Gate
4 above tests exactly that path, and it is the only test of it.

Physically confirm the tilt and drive stop when you power the transmitter off,
during A3 and A4 when those actuators are live.

---

## Stage A — actuator mapping

Props off, vehicle restrained, one channel at a time. Everything here uses
`hardware_mode:=action_test`.

### A1 — lift and the kill proof

```bash
./atmo_session.sh action --action-test lift --action-magnitude 0.1
```

The lift test uses a low raw collective baseline. **While the motors are
turning, activate the physical kill switch and confirm all four stop.**

**Pass:** all four motors spin, and the kill stops all four.

Nothing else in Stage A may run until this passes. After it does, declare it on
subsequent launches with `--kill-test-passed`.

### A2 — roll, pitch, yaw

```bash
./atmo_session.sh action --action-test roll --action-sign positive --kill-test-passed
```

Repeat for `pitch` and `yaw`, and both signs of each.

**Pass:** the correct diagonal pair spins up for each axis and sign.

Note on rotor identity: if the ESCs are PWM without telemetry, `esc_status` is
empty and per-corner identification by ear is unreliable at speed. The m4
bring-up accepted its motor map on convention and gated it behind the shadow
check. A zero-cost identity test that does work: at barely above idle, props
off, rest a screwdriver against each motor can — only the spinning corner
buzzes.

### A3 — tilt

**Status 2026-08-14: motion and calibration PASS; sign-off BLOCKED by drift.**

Board is `by-path ...:2.1:1.0`, motor **M2**. Home first — there is no
automatic homing in any tree:

```bash
# 1. BackwardM2 at duty ~60 until the speed collapses = FLY hard stop
# 2. SetEncM2(address, 0) there.  FLY = 0 counts = 0 deg.
# DO NOT home at duty 100: it wedges the arm, and escaping then needs ~80,
# which reads as a dead motor at any lower duty.
```

Measured: `Forward` = away from fly, downhill, breaks away ~duty 45.
`Backward` = toward fly, uphill, needs ~duty 100. Full sweep 0 -> 31.7 deg and
back to 1.1 deg, smooth, angle monotonic.

```bash
./atmo_session.sh action --action-test tilt --action-sign positive
```

**Pass:** positive tilt action drives the arms toward the posture the contract
says it should, and negative reverses it.

**Blocker:** ~16 deg of uncommanded drift between commands (raw -7233 -> -3705
in 1.5 s with nothing commanding). Commanded tilt position is not held.

Three ATMO-specific things to check here, all visible in
`atmo/tilt_controller_hardware.py`:

- **The encoder-to-angle table is flagged as needing recalibration in the
  source.** It maps encoder counts to angle by interpolation over a table
  collected for a *different* encoder part. Verify at least three points
  against a physical protractor before trusting any tilt reading. A wrong tilt
  angle corrupts the observation the same way the m4 hip-frame inversion did.
- **`SetEncM2(address, 0)` runs at node startup**, so the node assumes the
  vehicle always starts in drive configuration. Starting it with the arms
  part-tilted zeroes the encoder at the wrong place. Confirm the startup
  posture every time, or use the limit switch to home.
- **The travel guard is one-sided.** `limit_tilt` only blocks motion in one
  direction near one end. Confirm by hand what stops the other end.

### A4 — drive and turn

```bash
./atmo_session.sh action --action-test drive --action-sign positive
./atmo_session.sh action --action-test turn --action-sign positive
```

**Pass:** all four wheels turn the same way for `drive`; the left and right
pairs oppose for `turn`. Record which physical side is which — the contract's
wheel basis assumes front-left, front-right, rear-left, rear-right and ATMO's
URDF mirrors the right-side joints.

**Status 2026-08-14: PASS.** Board `by-path ...:2.2:1.0`, M1 = RIGHT, M2 =
LEFT, no encoders. The sides are mirrored:

```
ForwardM1  -> RIGHT forward      ForwardM2  -> LEFT backward
BackwardM1 -> RIGHT backward     BackwardM2 -> LEFT forward
```

Verified by prediction-then-test: both raw `Forward` was predicted to SPIN and
did; right `Forward` + left `Backward` was predicted to drive STRAIGHT and did.
`drive_controller_hardware.py` is correct as written — do not "fix" its
`ForwardM2`/`BackwardM1` asymmetry, it exists because the motors are mirrored.

**Caveat:** the left side runs slower than the right at equal duty, and nothing
closes a loop on wheel speed (no encoders on this board), so a straight `drive`
command will curve. Expect it, and do not attribute it to the policy.

**Stage A pass:** every action slot moves the right hardware in the right
direction, both signs, and the kill switch has been proven.

---

## Stage B — sensors and failure behaviour

### B1 — connectivity and conventions

```bash
./atmo_session.sh sensor
```

Move the unpowered vehicle by hand.

**Pass:** message counts advance, ages stay small, PX4 NED pose and velocity
track the hand motion with the right signs, the tilt angle reads the physical
arm position, and the RC gates read the switch positions you set.

PX4's NED yaw is opposite the policy world's z-up yaw. Expect that; do not
"fix" it here.

### B2 — safety cuts

Measure, do not assume:

- **Kill latency.** Time from the switch to commands stopping. Budget 500 ms.
- **Command timeout.** Stop the policy stream and confirm the rotor output
  goes to zero and the tilt and drive stop.
- **Manual override.** Publish `/atmo/rl/manual_override` true and confirm the
  tilt controller hands back to the RC trim lever while the policy is running.

A note on judging a kill test: a kill cuts torque, so an unsupported arm sags
afterwards. Gating on the joint *not* moving would only pass if torque were
still applied — which is the opposite of a working kill. Gate on the disable
latency and on continued motion being explainable by the last command.

### B3 — loop timing

**Pass:** the actuator publish rate holds its nominal value with a p95 period
inside one policy step, and the command-to-actuator path is inside the trained
observation-delay envelope (the contract's `observation_delay_steps`, currently
`[0, 1]` — 0 to 20 ms at 50 Hz).

Separate latency from plant dynamics. A slow *joint* response is a mechanical
property; a slow *pipeline* is a deployment defect. Only the second one belongs
in a delay budget. On the m4 vehicle the two were conflated for a session, and
the answer turned out to be servo stiction rather than latency — which would
have been retrained for, wrongly.

**Stage B pass:** conventions confirmed, every safety cut measured, timing
inside the trained envelope.

---

## Stage C — OptiTrack

See `docs/optitrack_bringup.md` for setup and
`docs/optitrack_session_checklist.md` for the run sheet. Gates:

- **C1** pose signs, all six
- **C2** twist signs, and twist genuinely body-frame
- **C3** rate and worst gap under 40 ms
- **C4** static noise inside the training envelope
- **C5** PX4 relay fused, `EKF2_EV_DELAY` set from C3, quaternion convention
  settled

**Stage C pass:** all five, measured this session. Mocap gates do not carry over
between sessions — the network changes, and so does Motive.

---

## Stage D — integrated dry runs

Restrained, props off unless stated.

### D1 — shadow on live mocap

```bash
./atmo_session.sh shadow --mocap-body <body> --mocap-frame <frame>
```

The stack runs end to end and creates **no** command publishers at all.

**Pass:** the contract check passes; policy output is finite across the whole
run; collective sits near hover rather than saturated; and attitude commands
**oppose** a hand-held tilt. A shadow run whose collective sits at 1.6× hover is
telling you the observation is wrong, which is exactly how the m4 hip-frame
inversion was found.

This is the mandatory gate before props go on.

### D2 — fixed action

```bash
./atmo_session.sh action --action-test <axis> ...
```

Re-run the Stage A axes with the full stack up, so the mapping is confirmed
through the same path a policy run uses.

### D3 — restrained low-authority spin-up

Props on, vehicle firmly restrained, low collective. Confirm the vehicle pushes
in the commanded direction and the kill switch still works with the full stack
running.

**Stage D pass:** shadow clean on live mocap, mapping re-confirmed through the
policy path, restrained spin-up behaves.

---

## Stage E — flight progression

In order, with a full stop and review between each:

1. Tethered hover, hover route only
2. Free hover
3. Takeoff route
4. Landing route
5. Full combined profile

Landing stays out of the first flights. Its completion logic depends on
proxies for a contact sensor the vehicle does not have, and those proxies are
the least-tested part of the runtime.

---

## Session hygiene

- Run `scripts/check_host.sh` on both machines and compare.
- Record everything. Every session records a bag; the shadow profile also
  writes a JSONL observation log.
- Stop with Ctrl-C in the stack window. The kill switch and dropping the RL
  gate are equally valid; all three zero the actuators.
- Write the numbers down. The blanks in the session checklist exist because a
  measurement nobody recorded gets re-taken.
- When a result contradicts a previous conclusion, prefer the measurement.
  Several m4 sessions were spent defending a written note against a tick-level
  reading that was right all along.
