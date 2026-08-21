# ATMO 2026-08-18 flight-data analysis — handoff

Everything below was derived offline on the laptop from
`~/ATMO/flight_data_20260818/` (bags, ulogs, `mav.parm`) after the arena
session ended. The single reproducible entry point is
`analysis/smatrix_validation.py`; every figure and merged CSV regenerates
from it. Auxiliary one-off analyses (lift model, EMA comparison, tau/phi
sweeps, z-alignment) are documented here with their results; their code
lives in the conversation-driven runs but all key numbers are recorded
below.

---

## 1. Data sources and run identification

**Operator account (2026-08-19), which supersedes the reconstruction below
where they disagree: four powered attempts — #1 yaw runaway, #2 started from
FLY config and rolled, #2b tilt frozen at 85° (drive), #3 started from DRIVE
and rolled at φ ≈ 15°.**

| # | run | ulog | rosbag | what it is |
|---|---|---|---|---|
| 1 | yaw runaway | `log_413.ulg` | `..._200125` | pre-flip mixer, ON GROUND (wheels), yaw spun to −10 rad/s in 1 s |
| — | quiescent | `log_414.ulg` | `..._200413` | ~15 s armed, **never left the ground** (mocap z flat to ±0.02 m, max rise 0.16 m at the very end) |
| 2 | roll, fly config | `log_415.ulg` | `..._202503` | **φ = 0 (FLY), not 85°**; roll event; 0.42 s of powered flight, rose 0.21 m, all of it after the kill |
| 2b | tilt frozen | `log_416.ulg` | `..._204231` **CORRUPT** | 6.4 s at φ = 85° (drive). Bag killed before checkpoint: no metadata.yaml, `.db3` fails `pragma integrity_check` — **unrecoverable, ulog-only** (no mocap, no hoist detection) |
| 3 | roll at φ≈15 | `log_417.ulg` | `..._205315` | started from DRIVE, swept to φ ≈ 15° from fly, rolled/flipped, killed at 1.70 s, hoisted at 1.80 s. **The only run with powered-airborne data** (0.70 s, climb 0.87 m) |

- ulog↔bag identity was established by cross-correlating bag-recorded
  `/fmu/in/actuator_motors` with the ulog's (log_414 ↔ bag 200413:
  **corr 0.999**). log_406 (only 460800-baud log) anchors "today" in the
  FC's log list; 407–417 are the evening's 11 arms.
- FC clock is unset (Y2K stamps) — never trust ulog dates; match by
  content.
- Time alignment bag↔ulog: actuator cross-correlation where the bag has a
  dense actuator record; otherwise **roll-signature alignment** (|roll|
  cross-correlation between mocap attitude and ulog attitude). For
  log_417: ulog spinup = bag t 16.78 s. The naive "first bag actuator
  message = spinup" anchor is WRONG when the recorder matched late (see
  §7) — it slid the window ~2 s early and produced a false "never left
  the ground" conclusion (retracted, §8).

## 2. The model

Ioannis' MPC dynamics (`ATMO/atmo_ws/src/atmo/atmo/mpc/dynamics.py`,
`S_func` + `M_func`, parameters from his `parameters.py`) — the only
physically-flown per-rotor model of this vehicle:

- Torque rows per motor at tilt φ (evaluated symbolically, §3 of
  `smatrix_validation.py:s_torque`): roll arm A(φ), pitch arms B1/B2(φ),
  yaw C1/C2(φ). At φ=0: roll `(−,+,+,−)`, pitch `(+,−,+,−)`,
  yaw `(+,+,−,−)`.
- Inertia diagonals Ixx/Iyy/Izz(φ) from `M_func` (0.118/0.093/0.187 kg m²
  at φ=0).
- Euler coupling ω×Iω included in all predictions.
- **Thrust/effort (final form, validated §5):** rotor SPEED follows the
  published command with a first-order lag τ=0.15 s (EMA), thrust goes as
  speed²: `T_i = kT_eff · (EMA_0.15(u_i))²` with **kT_eff = 59 N** at
  u=1. Ioannis' linear `kT = 28.15` is this curve's near-hover tangent
  (both put hover at per-rotor u ≈ 0.5–0.57). Note the subtlety: the lag
  applies to SPEED, and squaring makes the effective thrust response
  faster than lagging thrust directly — this reconciles the torque-side
  finding that *extra* lag on the (already adapter-filtered) commands
  only degrades fits (τ sweep: r_roll 0.56@0 → 0.33@0.15 → sign-flip
  @0.5 when lag applied to thrust).
- Commands are ZEROED from the kill instant (thr collapse after >0.5) —
  PX4's kill stops motors while the node keeps publishing.

Angle convention: φ measured FROM FLY (0 = rotors up, 90 = drive).
Operator/video reports are typically FROM DRIVE. This bit us once
("75°" = 15° from fly). Every figure states its convention.

## 3. Mixer verdict (the crash cause)

The deployed `_PHYSICAL_ROTOR_CONTROL_MIX` (from the training spec
`~/M4/vehicle_specs.py` ATMO_SPEC, propagated sim→contract→hardware) has
**ROLL and YAW columns inverted** vs the vehicle; PITCH is correct.
Three independent proofs:

1. Ioannis' S_func signs vs the spec, motor-by-motor (roll and yaw
   exactly negated, pitch identical).
2. IMU torque-response correlations in the departures under the spec:
   roll −0.40/−0.76, pitch +0.71/+0.91, yaw-old −0.77/−0.84. Under
   Ioannis' signs: all positive.
3. log_413's real yaw runaway: policy correction fed the spin (one-second
   wind-up to −10 rad/s), and post-saturation braking matches the
   corrected convention at corr +0.80.

4. **Control effectiveness ∂τ/∂cmd** (`mixer_proof.png`, 2026-08-19) — the
   one to put in front of a skeptic, and a derivative rather than a
   correlation. *(An earlier draft of this proof correlated the roll command
   against total roll torque; that was wrong — the total sums all four
   channels, so it largely measured the lift ramp, and correlating a command
   that never crosses zero is meaningless anyway. Superseded, do not reuse.)*

   The question a sign table has to answer is: **if the policy raises its roll
   command by one unit, which way does roll torque move?**

   ```
   B[k,a] = d tau_a / d cmd_k = sum_i S[i,a] * (dT_i/du_i) * M[k,i]
   ```

   with `M` the mixer DEPLOYED that day, `S(phi)` Ioannis' torque rows (which
   never see the mixer), and `dT_i/du_i = 2 (kT_eff/kT) * speed_i` from the
   squared-thrust model. **diag(B) must be positive on every axis** — that is
   the entire definition of a correct sign table. B depends only on the mixer,
   the geometry and the operating point: not on what the policy chose to
   command, not on vehicle motion, and therefore not on ground contact.

   | run | mixer era | ∂τ_roll/∂cmd | ∂τ_pitch/∂cmd | ∂τ_yaw/∂cmd |
   |---|---|---|---|---|
   | 413 | old | **−9.96** | −0.30 (no authority) | **−13.49** |
   | 414 | old | **−22.35** | −0.66 (no authority) | **−30.30** |
   | 415 | new, φ=0 | **−35.22** | **+28.18** | **+3.09** |
   | 416 | new, φ=85 | **−22.93** | −0.67 (no authority) | **+31.18** |
   | 417 | new | **−22.46** | **+13.98** | **+12.85** |

   Medians in N·m per unit command over powered samples. **Roll is negative at
   every powered sample of every run and its range never crosses zero** — this
   is not a statistical statement.

   Three things make it hard to argue with:
   - **Pitch is a known-correct column** and reads positive wherever it has
     authority, so the negative roll is not a frame-convention error in the
     test itself. At φ=85 pitch moves 0.3–0.7 N·m per unit command against
     roll's 10–23: no authority, reported as such rather than scored.
   - **Yaw flips sign exactly where the fix was deployed** — negative on
     413/414 (old mix), positive on 415/416/417 (flipped column). The test
     independently recovers a sign change we know happened, mid-session, on
     the same kind of column. Roll never flips.
   - **Model-free corroboration** on log_417, the only run with powered-
     airborne data (70 samples): corr(command, measured IMU angular accel) is
     **roll −0.56**, pitch +0.42, yaw +0.28. Cross-axis torque is not removed,
     so this corroborates B rather than replacing it — but it is the IMU, with
     no model in the loop, agreeing that the airframe accelerated opposite to
     the roll command.

   log_415 is the cleanest single exhibit: at φ = 0 all three axes have real
   authority, and roll is the only negative one (−35 against +28 and +3).

   **On smoothing** (challenged 2026-08-19, and rightly): `actuator_motors`
   logs at **9 Hz**; the pipeline interpolates to 100 Hz and EMAs with
   τ=0.15 s, so every smooth curve here has almost no real bandwidth — and
   the three B(t) traces are near-copies of each other **by construction**,
   not coincidence: with mixer entries ±1 and four similar rotor speeds,
   `B[a,a](t) ≈ arm_a(φ) × Σ_i speed_i(t)` — one shared time factor (total
   rotor speed) times a per-axis geometric arm. `mixer_proof.png` therefore
   plots the two factors separately — shared speed factor with the **raw 9 Hz
   samples overlaid as dots** (the smooth line passes through them), the B
   diagonals with their raw-sample (no-EMA) counterparts, and the arm(φ)
   panel, which is where the sign verdict actually lives: pure geometry ×
   deployed mixer, independent of rotor speed and untouched by any smoothing.
   For 415 (φ const): roll arm −2.9, pitch +2.3, yaw +0.26 — constants; for
   417 the arms sweep with φ and roll stays negative for the whole sweep.

5. **Deployed-network replay** (`analysis/policy_replay.py`,
   `log_417/replay_proof.png`, 2026-08-19) — the non-circular version of
   "what did the policy ask for". The raw policy actions were never logged;
   proof 4's semantic commands were recovered by INVERTING the mixer, which a
   skeptic can call circular. So the replay runs the actual deployed network
   (`atmo_ws/policies/atmo_combined_stage1_policy.npz`, ep_1800, sha
   53e4b221 — the Jetson deploy) over the actual flight observations, using
   the deployed runtime code imported as-is (obs packing, frames, normalizer,
   adapter, gates — nothing reimplemented), with `time.monotonic` bent onto
   the log clock. State: bag mocap odometry (the ulogs carry no
   vehicle_odometry, itself evidence the flight ran POSE_SOURCE=mocap).
   Tilt: MEASURED, from the tilt node's launch.log prints (see §7 update).

   Self-validation first: the replayed pipeline republishes motors through
   the deployed adapter+gates; against the ulog's logged actuator_motors the
   best engagement (interior optimum, spinup−6.0 s = 2 s drive + 2 s prep +
   gate ramp, exactly the deployed timing) gives pattern r = 0.64, per-rotor
   0.59–0.73. Not perfect — mocap-rate limits and the unlogged engagement
   instant cap it — but the replay demonstrably tracks the real run.

   The result, log_417 powered window, **no mixer inversion anywhere in the
   chain**: corr(network's own raw roll action, measured IMU roll accel) =
   **−0.67**, while pitch = **+0.34** and yaw = **+0.26**. The network pushed
   roll one way; the vehicle went the other; the two known-correct axes went
   the way they were told. The replayed roll also agrees with proof 4's
   reconstruction at +0.66, retroactively validating it.

Deployment status at end of day: **yaw column flipped and installed;
ROLL negation designed but NOT deployed.** Restrained single-axis tests
are the gate before it flies (NEXT_SESSION.md).

## 4. Per-run torque-model results (final pipeline settings)

**Numbers below are the 2026-08-19 second pass** (event-cut windows, corrected
tilt configs). Fits are corr r and regression slope (measured vs predicted angular
acceleration, FC-filtered `xyz_derivative`), in each run's stated fit
window, squared+EMA thrust model, post-kill commands zeroed:

| run | φ model | roll | pitch | yaw | reading |
|---|---|---|---|---|---|
| 413 | 85° const (on ground) | ~0 (ground-locked) | slope 1.5, r 0.72 | slope 0.06–0.15, r 0.60–0.74 | yaw direction/timing right; magnitude suppressed by wheel/ground friction; at 85° the model's yaw authority is ~10× fly-config (why the runaway was so violent) |
| 414 | 81–85° | ~0 | ~0 | ~0 | genuinely quiescent — no angular signal to validate against; the "control" run |
| 415 | ~~85° const~~ **φ = 0, FLY config** | slope 0.75, r 0.74 | slope 0.39, r 0.88 | slope **0.98**, r 0.75 | superseded row — see §8.8. At the correct tilt all three axes agree at once and the "carabiner anti-correlation" disappears |
| 416 | 85° const (tilt frozen) | slope −0.10, r −0.49 | slope −1.1, r −0.21 | slope 0.30, **r 0.92** | never analysed before 2026-08-19; bag lost so ulog-only. Same 85° signature as 413: yaw timing excellent, magnitude ~0.3× |
| 417 | ~~35→5°~~ **sweep 85→15° from fly** | slope −0.20, r −0.21 (ANOMALOUS) | slope 0.31, r 0.73 | slope 0.61, r 0.45 | started from drive, rolled at ≈15° (§8.9); roll is the damaged axis (§6) and the only powered-airborne data in the session (§11.3) |

Sensitivity notes:
- Correlation is nearly BLIND to φ (scale cancels); the discriminating
  objective is slopes-nearest-unity. Grid search (φ_start × φ_end ×
  τ, 417 pre-kill window) optimum: φ linear 35→5° from fly, τ=0(thrust-lag
  formulation), slopes (0.71, 1.05, 1.03). Consistent with the video
  (15° from fly ≈ mid-sweep) and with climb physics.
- Thrust exponent is NOT identifiable from torque data (differentials too
  small; sum_r flat 1.49–1.53 for e=1..2.5). It was identified from the
  lift axis instead (§5).

## 5. Lift-axis validation (log_417, the operator's model)

Chain: u → EMA(τ=0.15 s) speed → thrust = kT_eff·speed² → tilt φ=15° in
body → projected through measured roll/pitch → vertical accel → integrate.

- Fit window is PRE-PULL only: the safety pilot pulls the tether when it
  looks dangerous; the post-kill climb at 2.5 m/s (motors dead) is the
  rescue pull, NOT thrust, and contaminates any full-window fit.
- Result: **vz r = 0.83, rms 0.37 m/s, kT_eff = 59 N**; implied hover
  per-rotor u = 0.48. Model liftoff t=0.90 vs measured climb onset ~0.65
  — residual consistent with ground effect.
- Figure: `analysis/log_417/lift_model_prepull.png` (also
  `lift_model.png` full-window, kept to show the tether contamination).

## 6. The roll anomaly (open, with a named suspect)

log_417's roll axis disagrees with every model variant while pitch/yaw
agree. Per-rotor effectiveness fit (417 vs 413 as healthy control):
**rotor0 = front-right = control[0]: ~50% thrust, ~4× drag torque**
vs its own pre-strike values — the signature of a blade damaged in the
415 carabiner strike. Corroborated by |f| (accelerometer) sagging
11.8→6.5 m/s² while commands rose, and by 417's inflated apparent kM
(0.107 vs 0.043–0.047 in 413/415). **Action: physically inspect the
front-right prop; bench thrust/kM.** Tether loading is the secondary
contributor on this axis.

Healthy-vehicle yaw constant from the clean spin: kM_eff ≈ 0.044
(model had 0.018 — update ATMO_KM before retraining), yaw drag
b ≈ 0.022 N·m·s, possible standing yaw trim ~−0.7 N·m (bias term —
verify on bench before believing).

## 7. Measurement caveats that shaped (and mis-shaped) the analysis

- ~~**Measured tilt was never recorded**~~ **WRONG (found 2026-08-19): it
  was, in text.** The tilt node PRINTS "tilt angle is: X deg" into the ros
  launch logs (`ros_logs_20260818.tgz`, unpacked to `analysis/roslogs/`), 327
  lines during log_417's session alone, and launch.log timestamps are wall
  clock = bag clock, so the bag→ulog offsets place them on the flight
  timeline. `policy_replay.py:load_measured_tilt` parses them. **Timing caveat (found
  2026-08-19 late)**: the launch.log line stamps are FLUSH times, not sample
  times — stdout buffers and dumps at shutdown (417's 327 lines share one
  instant). The VALUES are real (417: 85.55°→17.58°, confirming the from-fly
  convention and the ~17.6° resting tilt at the departure); timing must be
  reconstructed from the bag's /tilt_vel active window (417: −2.2→+1.6 s
  relative to spinup, ≈17.9°/s slew ⇒ liftoff at 0.96 s happened at φ≈29°,
  75°-roll crossing at φ≈21°). The bag topic
  `/fmu/in/tilt_angle` is still missing (recorder fix stands), and the
  φ(t)-inference machinery in §4 predates this find — the smatrix pipeline
  should be rerun with measured tilt. The commanded `/tilt_vel` is ±1
  bang-bang — integrating it is unreliable and its bag values are normalized
  (×π/8 for rad/s).
- **Bag capture of `actuator_motors`/`vehicle_command_ack` was flaky**
  (2009 / 0 / 533 msgs across runs): one-way DDS endpoint matching —
  under the single laptop discovery server, even Jetson-local matching
  crosses the arena WiFi; a hole during the handshake leaves a RELIABLE
  publisher sending to nobody. QoS rows in the db3s prove the recorder
  discovered the publishers. Fixes: engagement gate on
  `get_subscription_count()` + a second, Jetson-local discovery server
  (NEXT_SESSION.md).
- Mocap yaw near the ground produces PHANTOM spins (tens of rad/s of
  marker-swap garbage — bags 195615/200125 "spins" with rotors unpowered,
  and log_414's "landing spin": FC gyro flat at ≤0.2 rad/s throughout).
  The policy reacts to these at full authority → obs pipeline needs a
  gyro-vs-mocap sanity gate.
- No RPM: `esc_status` absent (PWM ESCs). `actuator_outputs`,
  `vehicle_acceleration`, `hover_thrust_estimate` are in the ulogs and
  underused.
- The FC's `xyz_derivative` (onboard-filtered angular accel) is much
  cleaner than offline differentiation; rate-domain (integrated
  prediction vs ω) is cleaner still. Both are in `model_clean.png`.

## 8. Corrections log (claims made and retracted during the analysis)

Kept so nobody re-inherits a dead hypothesis:

1. "Policy commanded into the spin in 414's landing" — RETRACTED: that
   spin was mocap phantom; gyro flat. (The REAL spin is log_413.)
2. "Left-right mirrored rotor positions" — RETRACTED: positions match
   PX4 CA and the URDF exactly; the inversion is in the mix SIGNS
   (roll+yaw), tangled with frame conventions.
3. "log_417 = ground pivot, never left the ground" — RETRACTED: caused by
   the late-recorder alignment error; roll-signature alignment shows a
   climb to 0.8 m pre-kill, 2.3 m post (partly pull).
4. "φ=75 (from fly) at the flip" — corrected to 15° from fly (video was
   75° FROM DRIVE); then refined to a 35→5° sweep by grid search.
5. "417 flew at fly config from spinup" (accelerometer argument) —
   over-claimed; on-ground accel reads normal force, not thrust
   direction. The z-thrust growth is consistent with the sweep model.
6. Extra motor lag τ=0.15–0.5 on thrust — rejected by data; the correct
   home for τ=0.15 is on SPEED inside the squared model (§2).
7. kM_eff 6× (early fit) — artifact of assuming φ=0 for a φ=85° run,
   plus 417's damaged prop; healthy value ≈ 0.044 at fly config.

8. **"415 flew at φ=85 (drive) and its pitch axis is the carabiner"** —
   RETRACTED on the operator's account: 415 started from **FLY config
   (φ = 0)**. Re-running it there fixes all three axes at once — roll slope
   0.75 r 0.74, pitch slope 0.39 r 0.88, yaw slope **0.98** r 0.75 — where
   the φ=85 assumption gave yaw slope 0.04 and a −0.87 pitch anti-correlation.
   The tilt assumption, not the carabiner, was doing most of that damage.
9. **"417 swept 35→5° from fly"** (grid search) — superseded: it started from
   DRIVE and rolled at ≈15°, i.e. an **85→15° sweep**. The grid search was
   blind to this because correlation is nearly blind to φ (§4).
10. **"414 flew ~9.5 s at ~0.4 m"** — mocap disagrees: z is flat within
    ±0.02 m for 15 s and peaks at 0.16 m only at the very end. Treat 414 as an
    on-ground run.
11. **"Hub stiffness K_β ≈ 0.3–0.5 N·m/rad, found independently by 413 and
    415"** — RETRACTED within the hour it was written: those fits ran through
    **ground contact**, where the floor supplies whatever roll torque the
    constraint needs. Under a proper powered-airborne gate neither run has a
    single admissible sample (§11.3).
12. **log_416 was never analysed at all** until 2026-08-19 — the ulog was
    present the whole time; only its bag is lost.

## 9. Reproduction and file map

```
analysis/smatrix_validation.py     # the pipeline: source ROS + workspace, run it
analysis/policy_replay.py          # deployed-network replay (S.3 proof 5)
analysis/roslogs/                  # unpacked ros launch logs (measured tilt!)
analysis/log_41{3,4,5,7}/
  merged.csv                       # t, phi, attitude, pose, rotors, rates,
                                   # measured+predicted accel (with and without
                                   # the gyro term), rotor-momentum torque,
                                   # rotor speeds, reconstructed policy
                                   # semantic actions (era-correct mixer).
                                   # TRUNCATED at the kill/hoist cut (S.11).
  timeline.png  scatter.png        # accel overlay + fit scatter (offline-derivative)
  wheel_gyro.png                   # rotor-speed model + rotor-momentum torque (S.11)
  mixer_proof.png                  # commanded vs produced torque per axis: the
                                   # roll-inversion proof (S.3 proof 4)
  model_clean.png                  # FC-filtered accel + rate-domain (presentation)
  policy.png                       # what the policy commanded vs vehicle state
analysis/log_417/
  lift_model_prepull.png           # the validated lift model (r=0.83)
  lift_model.png                   # full-window version showing tether pull
  ema_comparison.png               # raw vs lagged commands, per-axis r
  z_timeline.png  z_aligned.png    # session altitude + corrected flight window
MANIFEST.md                        # per-bag/per-ulog identification
NEXT_SESSION.md                    # deploy list: gates, recorder, discovery,
                                   # restrained tests, prop inspection, sim updates
mav.parm                           # full FC parameter dump (2026-08-18 evening)
```

Run: `source /opt/ros/humble/setup.bash && source ~/ATMO/atmo_ws/install/setup.bash
&& python3 analysis/smatrix_validation.py` (needs pyulog, matplotlib>=3.8
in user site; bag deserialization needs the laptop's px4_msgs — which
does NOT match the Jetson's for VehicleStatus; ActuatorMotors/TiltVel/
Odometry are fine).

## 10. One-paragraph summary

The 8/18 crashes trace to a single uncalibrated actuator sign table
(roll+yaw columns inverted, inherited sim→training→deployment and
uncheckable by any test that shared the spec), amplified by taking off
in/through high tilt where the vehicle's control authority is radically
different, on a tether that both saved it and confounded the data. Against
flight data, Ioannis' S(φ) dynamics model is quantitatively validated on
pitch and yaw (slopes ≈ 1) and on the lift axis (vz r=0.83 under the
squared-thrust/speed-lag model); the roll axis carries a physical anomaly
whose data-predicted cause is a front-right prop damaged in the second
attempt. Yaw kM is ~2.4× the modeled value. Every open item has a
physical test queued in NEXT_SESSION.md, and the recorder/discovery
failures that degraded the dataset are root-caused with fixes staged.

---

## 11. Event-cut windows and the rotor-momentum ("wheel") model
*(added 2026-08-19, second pass on `smatrix_validation.py`)*

### 11.1 Every window now ends at a detected event

Two events are found per run and every figure, the merged CSV, and the fit
window are cut at whichever comes first:

- **kill** — the commanded-thrust collapse after spin-up (already used to zero
  the commands; now also an end-of-evidence marker, cut at kill + 0.5 s).
- **hoist** — the safety pilot's tether pull: first sustained (≥0.15 s) mocap
  climb faster than 0.8 m/s while commanded thrust is below hover (0.40).
  Past it the airframe is on a string, not flying.

Both are drawn as dotted verticals (red = kill, blue = hoist) on every timeline.

This was not cosmetic. `actuator_motors` in these ulogs is ~10 Hz and **the logs
re-arm inside the same file** — log_415 spins up again at 1.45 s, log_417 at
2.76 s — so the old fixed windows were splicing two separate flights into one
fit. Cutting changed the published numbers:

| run | window | roll | pitch | yaw | vs. §4 |
|---|---|---|---|---|---|
| 413 | kill 1.09 → cut 1.59 (fit clipped 6.5→1.59) | ~0 (ground-locked) | slope 1.46, r **0.90** | slope 0.19, r **0.82** | pitch r 0.76→0.90 |
| 414 | kill 16.99 → cut 17.49 | ~0 | ~0 | ~0 | unchanged (quiescent) |
| 415 | kill 0.40 → cut 0.90 (fit clipped 1.2→0.90) | slope 0.42, r **0.74** | r −0.87 (carabiner) | r 0.54 | roll r 0.51→0.74 |
| 417 | kill 1.70, hoist 1.80 → cut 1.80 | r −0.22 (still anomalous) | r 0.73 | slope 1.06 | roll anomaly survives |

log_415's real flight is only 0.42 s long; the rest of that ulog is a re-arm.

**Alignment change:** the first-actuator anchor is now automatically replaced by
the roll-signature cross-correlation whenever it (or nothing) is what the bag
supports — this is the §1 correction, made structural instead of manual. For
log_417 it moves the bag→ulog offset **2.06 s later** than the anchor, which is
exactly the error that produced retracted claim §8.3. It also recovers mocap
pose for 413 and 415, which previously had none.

### 11.2 Rotor speed, rotor momentum, and the hub (flapping) moment

The squared-thrust model already says the lagged command *is* the rotor speed
(`speed = EMA(u, τ=0.15 s)`, thrust ∝ speed²), so the rotors can be given
rotational dynamics with no new fitted quantity except their full-throttle
speed Ω_max (unmeasured — no `esc_status`, PWM ESCs — nominal 730 rad/s; every
moment below is exactly linear in it). Geometry read back out of Ioannis'
`S_func`: spin senses σ = (+,+,−,−) from the ±kM·cos φ yaw terms, tilt
directions e = (+,−,−,+) from the rA1x·sin φ terms, rotor axis
**n_i(φ) = (0, e_i sin φ, cos φ)** — which at φ=90° puts all four axes
horizontal along ±y, i.e. they become the wheel axles, as they must. Tilt is
therefore a rotation of each disc about body x, and the disc in-plane axes are
e1 = (1,0,0), e2 = (0, cos φ, −e_i sin φ).

Two distinct effects, both in the CSV and in `wheel_gyro.png`:

**(a) Rotor angular momentum** (`rotor_gyro_torque`): −ω×h, the geometric
precession from tilting `−I_r φ̇ Σ σΩ dn/dφ`, and the spin-up reaction
`−I_r Σ σΩ̇ n`. Two structural results here, both worth keeping:

- **Tilting and spinning up the rotors cannot produce roll torque at all** —
  n and dn/dφ have no x component by construction, so those two terms load only
  pitch and yaw. The flat zero traces in `wheel_gyro.png` are that identity, not
  a bug. Only −ω×h reaches roll, and only when h ≠ 0.
- With four healthy rotors Σσ = 0 and Σσe = 0, so **h cancels exactly**. It is
  non-zero on log_417 only because the front-right rotor is damaged (§6,
  modelled as a 0.707 speed factor = 50 % thrust).

Magnitude: **0.009 N·m rms → 0.078 rad/s² on 417's roll, against 24 rad/s²
measured (0.3 %)**. Linear in Ω_max and in the imbalance, so it cannot close a
300× gap at any plausible value. Ruled out as the roll anomaly.

**(b) Hub / flapping moment** (`flap_regressors`) — the larger effect, and the
one that actually reaches roll. The disc, not the hub, is the gyro: a shaft
angular rate flaps the blades, and because the flapping response lags the
excitation by 90° of azimuth, a **pitch rate tilts the disc sideways** and the
hub transmits that as a **roll** moment. Quasi-steady, per rotor, with shaft
rates p_d = ω·e1 and q_d = ω·e2:

```
beta_1 = (16/gamma)(p_d/Omega) + sigma (q_d/Omega)     about e1
beta_2 = (16/gamma)(q_d/Omega) - sigma (p_d/Omega)     about e2
tau    = -K_beta (beta_1 e1 + beta_2 e2)
```

The two halves behave completely differently across four rotors:

- the **damping half** (16/γ · rate/Ω) carries the same sign on every rotor, so
  the four **add**. This is a roll-rate-proportional moment the pure-thrust
  S matrix has no representation of whatsoever, and it is 40–200× bigger than
  anything in (a): 0.10–0.16 N·m rms → **2.9–3.6 rad/s²** on 413/415.
- the **precession half** (σ · rate/Ω) flips with spin direction and therefore
  cancels on a matched set — 0.001–0.026 N·m, i.e. two orders down. Same
  cancellation as (a), same escape hatch (rotor imbalance), same verdict.

`K_beta` is unmeasured for these props, so rather than guess it the script fits
it per run from the **roll residual** and asks what stiffness roll would have to
demand:

| run | fitted K_β [N·m/rad] | reading |
|---|---|---|
| 413 (ground) | **+0.28** | plausible, healthy props |
| 415 (departure 1) | **+0.47** | plausible, agrees with 413 |
| 417 (departure 2) | **+12.4** | **25× the healthy consensus** |
| 414 | −26 | meaningless: quiescent run, 0.08 rad/s² of roll signal to fit |

413 and 415 agreeing at ≈0.3–0.5 N·m/rad is the useful result: that is a
physically sane hub stiffness, and the two healthy runs find it independently.
The prediction therefore runs at a **fixed nominal K_β = 0.4** (`fit_flap=False`)
so runs stay comparable, with the per-run fit printed as a diagnostic.

**But K_β is not identifiable from this dataset**, and the first answer this
section gave was wrong — see §11.3.

### 11.3 Ground contact: why the hub-stiffness "measurement" was void

The first pass fitted K_β on 413 (+0.28) and 415 (+0.47), called their
agreement a measurement, and reported it. It was not one: **a wheel or a leg on
the floor supplies whatever roll torque the constraint demands**, so an
on-ground sample carries no information about the airframe's free-flight
moments. Anything fitted through it — hub stiffness, drag, damping — is fitting
ground reaction and will happily look consistent while meaning nothing.

The script now derives ground contact from mocap (`airborne` = rise > 0.15 m
above this run's own floor; absolute z is not comparable between bags) and
fits nothing aerodynamic except on **powered-airborne** samples — airborne AND
before the kill, since coasting rotors on a tether are no better than the
floor. What survives that gate:

| run | airborne | max climb | powered-airborne | K_β |
|---|---|---|---|---|
| 413 | 0 % | 0.00 m | 0.00 s | not identifiable |
| 414 | 1 % | 0.16 m | 0.00 s | not identifiable |
| 415 | 48 % | 0.21 m | **0.00 s** — every airborne sample is post-kill | not identifiable |
| 416 | — (bag lost) | — | — | not identifiable |
| 417 | 45 % | 0.87 m | 0.70 s | +12.3 N·m/rad |

**Exactly one run in the whole session has powered-airborne roll data, and it
is the run with the damaged prop.** So there is no clean measurement of the hub
moment here at any stiffness, and 417's +12.3 has nothing to be compared
against. The prediction runs at a nominal K_β = 0.4 for sizing only
(`fit_flap=False`), the per-run fit is printed as a diagnostic, and the honest
statement is that the flapping moment is **sized but unmeasured**: 0.10–0.16
N·m rms → a few rad/s² if K_β is O(0.4), which is large enough to matter to the
sim and cannot be pinned down without either a bench test or a flight that
actually leaves the ground under power.

That last point is the real finding of this pass. Four of the five runs never
flew: the dataset cannot answer roll-axis questions, and no amount of modelling
will change that. **The next session needs a run that climbs and stays climbing
under power**, or the roll anomaly stays open regardless of which physics gets
added to the model.

---

## 12. Which physical rotor is 0, 1, 2, 3 — and which way each one spins

Written 2026-08-20, because §3 says "roll and yaw columns are inverted" without
ever saying *which motor is which*, and every remaining question on this axis
(the damaged prop, the restrained single-axis tests, whether to fix the mixer or
the wiring) needs that table. Regenerate with `analysis/rotor_identity.py`.

### The table

PX4's FRD body frame: **+x forward, +y right, +z down**, thrust along `-z`.
Spin is stated as seen **from above**.

| control[i] | position (m, FRD) | corner | spin | yaw torque per unit thrust |
|---|---|---|---|---|
| **0** | (+0.16, +0.21) | **front-right** | CCW | + (nose right) |
| **1** | (−0.16, −0.21) | **rear-left** | CCW | + |
| **2** | (+0.16, −0.21) | **front-left** | CW | − |
| **3** | (−0.16, +0.21) | **rear-right** | CW | − |

This is the standard PX4 quad-X numbering, and it is the same numbering the
White M4 uses (`m4-direct-rl/docs/session_state.md`, "Motor indexing fixed
pre-emptively").

Two independent sources give it, neither of which knows about the RL stack:

* **`mav.parm`**, the FC's own `CA_ROTOR{0..3}_{PX,PY,PZ,KM}` — what the flight
  controller allocated with on the day.
* **Ioannis' `S_func`**, read backwards. In FRD with thrust along `−z`,
  `tau = r x F` makes the roll entry `−kT·y`, the pitch entry `+kT·x` and the
  yaw entry `+kT·kM·sigma`, so his four torque rows *are* the four positions
  and spins: (+0.172, +0.205), (−0.158, −0.205), (+0.172, −0.205),
  (−0.158, +0.205) with sigma = (+, +, −, −). Corner and spin pairing agree
  with `mav.parm` on all four rotors; his arms are the unrounded ones and carry
  the 14 mm front/rear asymmetry that `mav.parm` rounds away.

Positive yaw torque in FRD is nose-right, i.e. clockwise from above, which is
the *reaction* to a rotor turning counter-clockwise — hence CCW for the pair
that carries `+kM`. The absolute sense comes from Ioannis' frame (his altitudes
are negative throughout `parameters.py`, so NED is not in doubt); PX4's own
`KM` sign convention was not verified against PX4 source here, so `mav.parm`
corroborates the **pairing and the positions**, not the absolute handedness.

### The mapping is MEASURED, not derived

Superseding the derivation above as the authority: the operator physically
identified the rotors on the vehicle — **control[0] front-right, [1] rear-left,
[2] front-left, [3] rear-right, props-in ("innie")** — which is exactly what
`mav.parm` and Ioannis' S rows both predict, and it is the standard PX4 quad-X.
Props-in means the front pair's leading edges sweep toward the centreline:
front-right CCW, front-left CW, which is the `+kM / -kM` pairing above.

**So there is no rotor mislabelling anywhere.** The training regime that
produced the deployed 8/17–8/18 policy had the correct rotor mapping. An
earlier draft of this section proposed that the deployed mixer was the correct
one with rows permuted by (2,3,0,1) — a left–right mirror of the numbering.
That permutation is real arithmetic (the two matrices *are* related by it) but
it is **excluded by the physical measurement**, and it is not the explanation.
The explanation is a frame, not a permutation: see §13.

---

## 13. Why the axes looked flipped: two frame defects, and what each axis did

Written 2026-08-20. Supersedes §3's attribution of the crash to the mixer. The
mixer is not the fault; two frame errors are, and the axes differ because the
two errors touch different axes.

### What is actually wrong

**D1 — the mocap rigid body is 180 degrees out.** The Motive body is defined /
mounted 180 deg yawed relative to the flight controller and the rotor
numbering, and nothing downstream removes it. **Measured**, not assumed: the
bridge's body frame is the FC's body frame rotated **178.6-179.9 deg about y**
on three runs (414/415/417), residual 1.0-3.2 deg, `det < 0` confirming z-up.
Motive is configured z-up and `atmo_session.sh` passes `source_frame=z_up`, so
`to_z_up` is a no-op and **no code touches the pose between Motive and D2** —
the 180 deg can only be the rigid-body definition.

**D2 — the PX4 conversion is applied to data that is not PX4's.**
`update_px4_state` (`rl_landing_stage1_runtime.py:457`) applies `_NED_TO_ENU`
(world) and `_FRD_TO_FLU` (body) to every field. That is the correct
transform for `/fmu/out/vehicle_odometry`, and the wrong one for
`mocap_odom_callback`, which passes the bridge's already-z-up, already-body-frame
odometry. One function, two callers, one of them in the wrong frame.

**D3 — latent, currently inert.** The PX4 vision relay maps the raw stream with
`px4_*_atmo_legacy`, which assumes y-up; on this z-up rig it sends height into
the East channel (measured, correlation 1.000). Nothing consumes it: log_417's
`cs_ev_pos`, `cs_ev_yaw` and `cs_ev_vel` are 0 for the whole run and every
`vehicle_local_position` validity flag is 0. The EKF is not in the loop and
never even ingested the mocap. Fix or disable it before anyone enables EV
fusion; it explains nothing about 8/18.

### How D1 and D2 compose

Writing every body frame as `(FC body) . R`:

```
   training expects     Rx(pi)     z-up, x forward
   D1 delivers          Ry(pi)     MEASURED
   D2 then adds Rx(pi)  Rz(pi)     what the policy received
```

| scenario | observation frame | axes inverted |
|---|---|---|
| **as flown** | `Rz(pi)` | **roll, yaw** |
| fix the mount only | `I` | pitch, yaw |
| remove the conversion only | `Ry(pi)` | roll, pitch |
| **fix both** | `Rx(pi)` | **none** |

Neither defect can be fixed alone — each single fix leaves two axes inverted
instead of two. They must land in the same change.

Now add the mixer's yaw column, which was flipped mid-session on 8/18:

| | axes inverted |
|---|---|
| as flown, pre-8/18 yaw column | roll, yaw |
| as flown, post-8/18 yaw column | **roll only** |
| both frames fixed, yaw column still flipped | **yaw only** |

### What each axis did, and why

**ROLL — inverted, and it is the one that crashed 417.**
D1 flips roll; D2 does not touch roll; so the flip survives to the rotors. The
loop is positive feedback, and the flight shows it directly. Over the 1.7 s
powered window the policy's own roll command climbs monotonically **+0.15 ->
+0.37** while roll runs **−5 -> −100 deg**, and never recovers; kill came at
−141 deg. Model-free plant sign, network output against the FC's own filtered
gyro: **corr −0.60, slope −38.1 rad/s² per unit** — a positive roll command
produced negative roll acceleration, so pushing harder drove it further in.
This is also what §3's `∂τ/∂cmd` roll < 0 was seeing: **the sign table was
reporting the mount**, in a test that compares an asset-frame command against
PX4-frame torque.

**PITCH — correct, by accident.**
D1 flips pitch and D2 flips it back. Two inversions, so the loop is sound — and
it visibly is: in the same 1.7 s window pitch departs to **−22.2 deg** and is
back through zero **0.4 s later** (+30.3 rad/s² per unit, corr +0.25). This is
why every mixer-side test called pitch "the known-correct column": it is
correct, and D2 was quietly repairing the mount for that one axis. Nothing in
§3 could have seen D1 through it.

**YAW — inverted by D2 alone, then masked by the mixer flip.**
D1 does not touch yaw; D2 does. Pre-8/18 that left one inversion, and 413 is
exactly that failure: a gyro-confirmed yaw runaway with the policy's correction
feeding the spin (§3 proof 3). The 8/18 yaw-column flip added a second
inversion, and no yaw runaway appears in 415, 416 or 417. **The flip cancelled
D2; it did not fix the mixer.** 417 says nothing further about yaw — at that
tilt the yaw authority is +1.1 rad/s² per unit, essentially none.

**POSITION — separately and unambiguously wrong.**
D2's `_NED_TO_ENU` is `(x, y, z) -> (y, x, -z)`: it **swaps x with y** and
**negates height** (slope −1.000 on a run with 5.39 m of real climb). So the
policy was told a climb was a descent, and shown longitudinal errors as lateral
ones — a position loop that answers an x error with roll. Independent of every
sign above, and fixed by the same change.

### Fix order

1. **Take the 180 deg out in exactly one place** — preferably by redefining the
   Motive rigid body, since that is where the physical truth lives. Fixes roll.
2. **In the same change**, remove `_NED_TO_ENU` / `_FRD_TO_FLU` from the mocap
   path (split `update_px4_state` into a PX4 entry point and a
   training-frame entry point). Fixes yaw, height and the x/y swap.
3. **In the same change**, revert the 8/18 yaw column. Step 2 removes what it
   was cancelling; leaving it flipped makes yaw the newly broken axis.
4. **Do not deploy the designed roll negation.** Step 1 fixes roll; doing both
   inverts it again.
5. Disable the PX4 vision relay, or fix `px4_*_atmo_legacy` for a z-up rig.
6. **Restrained single-axis test before flight.** Everything above except the
   height and x/y swap rests on 1.7 s of powered flight and one identified
   frame; the bench test settles all three axes in minutes.

### Confidence

Measured: the 180 deg mount (3 runs), what D2 does to each field, the height
inversion, the x/y swap, the body-rate flip, the roll runaway and its plant
sign, the pitch recovery, the EKF's non-participation. Inferred: that training's
body frame is `Rx(pi)` relative to the FC — the ATMO USD is not in this repo, so
this comes from the flight behaviour plus the requirement that the composition
be a proper rotation. If that inference is wrong the per-axis assignment moves,
which is the other reason step 6 exists.

### The frame map: every conversion in the chain, in order

Three paths leave the mocap stream, and they need different frames. Listed with
the file that owns each one, so there is a single place to check.

**Path A — mocap to the policy observation** (this is the one that broke)

| # | where | what it does | verdict |
|---|---|---|---|
| A0 | Motive rigid-body definition + physical mount | sets the body axes. **The rig is mounted 180 deg yawed from the FC and the rotor numbering.** | **UNCOMPENSATED — defect** |
| A1 | Motive world config | **z-up, explicitly configured by the operator** | fine |
| A2 | `mocap_frames.to_z_up` :40 | with `source_frame=z_up` this **returns the pose unchanged** — a no-op. `atmo_session.sh:90` defaults `MOCAP_FRAME=z_up`, which is what the 8/18 runs used. | no-op on this path |
| A3 | `mocap_bridge` publishes `/atmo/groundtruth_odom` | pose passed through, **twist derived in the BODY frame** | correct |
| A4 | `rl_controller_hardware.mocap_odom_callback` :1278 | `vel_w = R(q) @ vel_body`; angular velocity stays body | correct |
| A5 | `update_px4_state` :457 | `_NED_TO_ENU @ p`, `_NED_TO_ENU @ R @ _FRD_TO_FLU`, `_FRD_TO_FLU @ w` | **WRONG HERE — the input is not PX4 NED/FRD** |
| A6 | observation packing | rotate into the heading frame by `heading_yaw + forward_yaw_offset` (pi) | correct, but built on A5's output |

Because A2 is a no-op, **no code touches the pose between Motive and A5.** The
180 degrees measured in CLAIM 1 is therefore entirely the rigid-body
definition and the physical mounting — there is nothing else it could be.

**Trap worth knowing:** the node's own default is `source_frame=y_up`
(`mocap_bridge.py:70`); only `atmo_session.sh` overrides it to `z_up`. Launch
`ros2 run atmo mocap_bridge` by hand against this z-up rig and A2 stops being a
no-op — it applies a spurious +90 deg about x, putting height into `-y`.

**Path B — mocap to PX4's EKF** (a different consumer wanting a different frame)

| # | where | what it does | verdict |
|---|---|---|---|
| B1 | `mocap_frames.px4_position_atmo_legacy` :81 | RAW `(x,y,z) -> (x, z, -y)` | **WRONG for a z-up rig — this mapping assumes a y-up stream** |
| B2 | `px4_quaternion_atmo_legacy` :70 | RAW `(w,x,y,z) -> (w, -z, x, -y)` | same assumption; `mocap_frames.py` already flags that it is not the composition of y_up -> z_up -> NED and that "which is correct is a measurement" |

Measured on bag 195615, bridge position against the relayed
`/fmu/in/vehicle_visual_odometry`, correlations of 1.000:

```
   bridge x  ->  EV North     bridge z  ->  EV East     bridge y  ->  EV Down (negated)
```

For a z-up world the correct NED is `N = x, E = -y, D = -z`. What was actually
sent puts **height into the East channel** and lateral position into Down. This
is the y-up mapping applied to a z-up stream — a second, independent frame
defect, on the EKF path rather than the policy path. It is consistent with the
FC's own position estimate in 417 having a standard deviation of **607 m**: the
EKF was fed a 90-degree-rotated world and never tracked anything. It did not
crash the flights only because they ran `POSE_SOURCE=mocap`, with the EKF out of
the loop.

**Three entries are defects: A0, A5 and B1/B2.** Everything else is either a
no-op or a single legitimate conversion. A0 and A5 do not cancel:


```
   what training expects        (FC body) . Rx(pi)      z-up, x forward
   A0 the mount delivers        (FC body) . Ry(pi)      MEASURED, 178.6-179.9 deg, 3 runs
   A5 then adds a body Rx(pi)   (FC body) . Rz(pi)      what the policy received

   mount alone       vs expected  =  Rz(pi)  ->  flips ROLL and PITCH
   mount + A5        vs expected  =  Ry(pi)  ->  flips ROLL and YAW
```

Roll is flipped by the mount and A5 never touches roll, so it stays flipped.
Pitch is flipped by the mount and flipped back by A5, so it looks fine. Yaw is
untouched by the mount and flipped by A5. That is the whole mechanism, and it
predicts 417's roll runaway, 417's pitch recovery in the same window, and 413's
yaw runaway under the pre-8/18 column.

A5 also carries the two position defects, which are pure `_NED_TO_ENU`:
`(x, y, z) -> (y, x, -z)` swaps x with y and negates height.

### The answer, per axis — from the flight, not from the frames

**Retraction.** An earlier version of this section argued from frame algebra that
roll had no defect (that `∂τ/∂cmd` roll < 0 was merely the asset frame showing
through) and that pitch was the broken axis. **Log 417 says the opposite, and the
flight wins.** The measurement below needs no frame argument at all.

#### Roll ran away under power. Pitch recovered. Same policy, same 1.7 s.

The deployed network's own commands (replay, pattern r = 0.64) against the FC's
attitude, through the powered window:

| t − spinup | roll cmd | roll deg | pitch cmd | pitch deg |
|---:|---:|---:|---:|---:|
| 0.80 | +0.16 | −0.4 | +0.11 | −9.2 |
| 1.00 | +0.15 | −5.2 | −0.07 | −22.2 |
| 1.10 | +0.21 | −12.5 | −0.18 | −16.8 |
| 1.20 | +0.32 | −22.8 | −0.28 | −7.0 |
| 1.30 | +0.37 | −39.5 | −0.13 | −0.4 |
| 1.40 | +0.36 | −65.9 | −0.06 | +0.7 |
| 1.50 | +0.20 | −100.1 | −0.07 | +3.7 |
| 1.60 | −0.02 | −141.0 | −0.28 | +18.8 |

**Pitch is a working loop.** It departs to −22.2 deg, the policy answers, and it
is back through zero 0.4 s later. That is a stabilised axis behaving exactly as
it should.

**Roll is positive feedback.** The command climbs monotonically +0.15 → +0.37
while the roll goes −5 → −100 deg. The policy pushed harder the further it went,
and it never recovered. Kill came at −141 deg.

#### The plant sign, measured with no model in the loop

Correlating the same commands against the FC's own filtered angular
acceleration (`vehicle_angular_velocity.xyz_derivative`), 85 samples:

| axis | corr(cmd, FC angular accel) | slope |
|---|---:|---:|
| roll | **−0.60** | **−38.1 rad/s² per unit** |
| pitch | +0.25 | +30.3 rad/s² per unit |
| yaw | +0.11 | +1.1 rad/s² per unit |

A positive roll command produced a **negative** roll acceleration, at 38 rad/s²
per unit — so commanding +0.37 into an already-negative roll drove it further
negative. That is the runaway in one number, and it is model-free: it uses only
the network's output and the FC's gyro. It also independently reproduces §3
proof 4's signs and magnitudes (−22.46 roll, +13.98 pitch for this run), which
were derived through Ioannis' S matrix.

Yaw's +1.1 rad/s² per unit is the tilt: at the 85 → 15 deg sweep this run flew,
yaw authority is near zero for most of it, which is why 417 says nothing about
the yaw column and 413 (on the ground, high tilt) is where yaw showed itself.

#### So, per axis

| axis | verdict | evidence |
|---|---|---|
| **roll** | **inverted — a real defect** | positive feedback in flight; +cmd gives −38 rad/s²; runaway −5 → −100 deg in 0.5 s with the command still rising |
| **pitch** | **correct** | departs and recovers inside the same window; +cmd gives +30 rad/s² |
| **yaw** | unresolved by 417 (no authority); 413 is the evidence, and it ran away under the pre-8/18 column | +1.1 rad/s² per unit here |

**This restores §3's original conclusion and the slides': the roll column is
inverted and the designed roll negation should be deployed.** The frame work in
this section stands as measurement — the mocap mount, the double conversion, the
negated height, the swapped x/y are all real and independently confirmed — but
the inference I drew from it, that the mixer's roll sign was merely the asset
frame showing through, is refuted by the flight. Somewhere in that chain an
assumption is wrong; the most likely candidate is my identification of the
training asset's body frame with the mocap frame, which no file in this repo can
confirm (the ATMO USD is not here). **Where the frame algebra and the flight
disagree, use the flight.**

#### What this does not excuse

The height inversion and the x/y swap (CLAIMS 3 and 4) are exact, and they are
still defects regardless of the roll column: the policy was told a 5.39 m climb
was a descent, and shown longitudinal errors as lateral ones. They are a
separate fix from the mixer and both are needed.

### How the frames are measured, and why not by Euler slopes

A per-axis slope of −1 is consistent with several different frame relations, so
the fit here is on a **body-frame vertical**: for each stream, take the world
z axis expressed in the body, `v = R^T e_z`, and least-squares fit
`v_other = M · g_fc` over the run. Body-frame vectors do not depend on either
world's heading, so this isolates the body frame relation and the z convention,
with the heading excluded by construction. `M` is fitted **unconstrained**, so
an improper result is visible rather than projected away: `det M < 0` means that
stream's world z points opposite to the FC's (NED z is down), i.e. z-up.

Fitting full attitude instead does not work, and that is a finding in itself —
see the drift measurement below.

### CLAIM 1: the mocap body frame is the FC body frame rotated 180 deg about y

| run | M diagonal | det | recovered rotation | residual median / p90 |
|---|---|---:|---|---:|
| 414 | (+1.04, −0.86, +1.00) | −0.90 | **179.9 deg about y** | 1.0 / 2.5 deg |
| 415 | (+0.46, −0.32, +0.99) | −0.15 | **179.1 deg about y** | 1.8 / 8.4 deg |
| 417 | (+0.98, −0.93, +0.99) | −0.91 | **178.6 deg about y** | 3.2 / 5.8 deg |

Three runs, 982 / 3669 / 5761 samples, agreeing within 1.4 degrees, off-diagonal
terms ≤ 0.11. `det < 0` on all three: the bridge's world z points **up**. So the
mount is measured, not asserted, and the operator's account is confirmed —
"180 degrees rotated" in plan view, on a z-up stream, **is** 180 degrees about
body y once the z-up/z-down difference is included (`Rx(pi)·Rz(pi) = Ry(pi)`).

**And 180 degrees about y is exactly the asset frame.** Training is z-up with
`forward_yaw_offset = pi` — x backward — which is precisely `FC ∘ Ry(pi)`. The
mocap stream arrived already in the frame the policy was trained in.

log_413 cannot contribute: 0 of its 15456 samples have FC lean above 3 degrees,
so there is no attitude to fit. Its bag is still usable for the position claims.

### CLAIM 2: `update_px4_state` then destroys that frame

Same fit, applied to what the deployed builder holds after ingestion:

| run | M diagonal | det | recovered rotation | residual |
|---|---|---:|---|---:|
| 414 | (−1.04, −0.86, +1.00) | +0.90 | 180.0 deg about **z** | 1.0 / 2.5 deg |
| 415 | (−0.46, −0.32, +0.99) | +0.15 | 173.3 deg about **z** | 1.8 / 8.4 deg |
| 417 | (−0.98, −0.93, +0.99) | +0.91 | 176.6 deg about **z** | 3.2 / 5.8 deg |

`det` flips sign: the observation's world z points **down**. The body rotation
moves from y to z. Relative to what training wants, the observation is the asset
frame rotated a further 180 degrees about body x, in a world with z inverted —
which is exactly `_FRD_TO_FLU` (body) and `_NED_TO_ENU` (world) applied to data
that was already in the training convention
(`rl_landing_stage1_runtime.py:457`).

### CLAIM 3: height is negated. CLAIM 4: x and y are swapped

No fitting needed; these are exact, on every run including 413:

```
obs z vs bridge z    slope -1.000   r -1.000     (417 bridge z spans 5.39 m)
obs x vs bridge y    slope +1.000   r +1.000
obs y vs bridge x    slope +1.000   r +1.000
obs x vs bridge x    slope -0.07     r +0.29     (no relationship)
```

`_NED_TO_ENU` as coded is `((0,1,0),(1,0,0),(0,0,-1))`: it **swaps x and y** and
negates z. On 417 the vehicle really climbed — 5.39 m of range in the bridge's z
— and the observation reported that as 5.39 m of descent.

### CLAIM 5: the body rates carry the same body flip

```
obs p vs bridge p    slope +1.000   r +1.000
obs q vs bridge q    slope -1.000   r -1.000
obs r vs bridge r    slope -1.000   r -1.000
```

Identical on all four runs. `(p, −q, −r)` is the 180-about-x body rotation of
CLAIM 2, confirming it independently of the attitude fit.

### CLAIM 6: mocap and FC headings drift, so no yaw-offset argument is safe

`bridge yaw − FC yaw`, over the fitted samples:

| run | median | p10..p90 | drift |
|---|---:|---:|---:|
| 414 | −344 deg | −357 .. −245 | +5.3 deg/s |
| 415 | −148 deg | −266 .. −134 | −0.8 deg/s |
| 417 | −6443 deg | −7887 .. −2164 | −133 deg/s |

417's is the documented mocap phantom yaw (§7), and none of the three is a
stable offset. This is why CLAIM 1 and 2 are fitted on a heading-independent
quantity, and why the earlier attempt in this analysis to read the mount off a
yaw offset alone gave −90 in some runs and −180 in others.

### What this changes

**Correction to an earlier draft of this section.** It claimed the mount and the
double conversion cancel, leaving the attitude accidentally correct. The
measurement says the opposite: the bridge stream was **already** in the asset
frame (CLAIM 1), and the conversion **broke** it (CLAIM 2). Relative to what
training wants, the observation carried:

| quantity | state |
|---|---|
| roll, roll rate | correct |
| pitch, pitch rate | **inverted** |
| yaw, yaw rate | **inverted** |
| height | **inverted** |
| x, y position | **swapped** |

### 417's roll departure, and what it is not

The departure is powered, not a post-kill artefact: roll leaves zero at
t = 1.0 s after spin-up and reaches −141 deg by 1.6 s, with the kill at 1.70 s.
Everything past that (peak −150, pitch −85) is the fall and the tether hoist and
should not be read as vehicle behaviour.

Two explanations offered earlier are withdrawn:

* **The damaged front-right prop is not the cause.** §6 read `control[0]` at
  ~50% thrust in 417 as a blade damaged in the 415 carabiner strike; the
  operator reports the props were replaced, so that inference needs a different
  cause and cannot carry the roll departure.
* **Roll-dominant geometry is not sufficient either.** It is true that at the
  tilt 417 flew, roll authority runs 1-3x pitch and Ixx falls to a third of its
  flight value (table below), so any moment shows up mostly as roll. But that
  explains an *amplitude*, not a *runaway*: it does not explain a command that
  rises monotonically as the angle diverges, and it does not explain why pitch
  recovered in the same window.

| tilt (from fly) | roll arm | pitch arm | Ixx | Iyy | roll:pitch accel |
|---:|---:|---:|---:|---:|---:|
| 0 deg | 5.78 | 4.83 | 0.118 | 0.093 | 0.94 |
| 15 | 5.56 | 4.53 | 0.113 | 0.094 | 1.02 |
| 29 | 5.25 | 3.98 | 0.103 | 0.096 | 1.22 |
| 60 | 4.30 | 1.98 | 0.071 | 0.102 | 3.14 |
| 85 | 3.43 | −0.08 | 0.038 | 0.104 | 111 |

The geometry does say the departure would be violent once started, and that the
window for catching it is short: with Ixx at 0.038-0.11 and 38 rad/s² per unit
of command available, roll is by far the twitchiest axis in a tilted posture.

### What the frames do bear on: the yaw column

413's yaw runaway is a different case, and there the loop-sign argument has
independent support: §3 proof 3 shows the policy's correction feeding the spin
under the pre-8/18 mixer, and 413's spin is gyro-confirmed rather than mocap
phantom. Combined with CLAIM 2 (the observation's yaw is inverted relative to
the asset frame) and §12 (the mixer is correct in that frame), the pre-flip yaw
loop had exactly one inversion in it, which is the condition for that runaway.
After the flip it had two, and no yaw runaway appears in 415/416/417.

That makes the 8/18 yaw flip a **cancellation of an observation error, not a fix
to the mixer** — a hypothesis with one supporting run, not a proved result. Its
practical consequence is a warning rather than a conclusion: **correcting
`update_px4_state` while leaving the yaw column flipped puts two wrongs back to
one.** The restrained single-axis test resolves it in minutes and should be run
before either change flies.

Roll gets no such support from the data, in either direction. §3's `∂τ/∂cmd`
roll < 0 is what a correct asset-frame mixer produces under CLAIM 1's measured
`Ry(pi)`, so it is not evidence of a defect — but neither is anything here
evidence that the roll column is right. It is untested, and the restrained test
is what tests it.

### Fix order

The restrained single-axis test comes FIRST now, because the frame work has
narrowed the mixer question rather than answered it, and because 417 turned out
not to test it.

1. **Restrained single-axis test, props on** (NEXT_SESSION §4). It measures roll,
   pitch and yaw response against the operator's own eyes, in the one frame not
   in dispute. Run it before changing any sign.
2. Remove the unconditional `_NED_TO_ENU` / `_FRD_TO_FLU` from the mocap path in
   `update_px4_state`. It is correct only for a genuine PX4 NED/FRD source; the
   bridge is neither. This fixes the height inversion and the x/y swap, which
   are exact measurements (CLAIMS 3 and 4) and need no further evidence.
3. **In the same change**, decide the yaw column from what step 1 measured — not
   from §3. If the pre-8/18 column was correct in the asset frame, the current
   flipped column must go back, or the yaw runaway returns.
4. Leave the roll column alone unless step 1 says otherwise.
5. Re-run `analysis/frame_proof.py`. The target is CLAIM 2 reproducing CLAIM 1 —
   `det < 0`, 180 degrees about y — with height slope +1.000 and x/y unswapped.
6. Inspect and bench the front-right prop (§6) before reading anything else into
   417's roll.

Whether the mocap mount should be corrected in Motive instead is a separate
decision: as measured it lands the stream in the training frame, so correcting
it would *require* a compensating rotation in the runtime. Whichever is chosen,
it belongs in exactly one place, written down.
