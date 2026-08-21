# ATMO flight data, 2026-08-18 (arena day)

## bags/  — all rosbags from 2026-08-18 (Jetson ~/ATMO_rl/atmo_ws/bags)
Key runs:
- atmo_ground_20260818_002029: last night's clean 5/5/5 ground mission (real mocap, unhomed frame)
- atmo_policy_20260818_200413: PRE-FLIP FLIGHT. ~9.5 s airborne, ~0.4 m alt, yaw bursts ~4 rad/s
  near landing. 2009 actuator_motors msgs recorded — the cross-correlation anchor for ulog matching.
- atmo_policy_20260818_202503: post-flip attempt 1 — pitch-up into tether carabiner ~2 s after
  engage (engage at bag t≈219 s). actuator_motors NOT recorded (recorder fault).
  Bag also shows 400-800 ms mocap stream holes through the flight window.
- atmo_policy_20260818_204231: aborted attempt; bag file corrupted (killed mid-write).
- atmo_policy_20260818_205315: post-flip attempt 2 — +81 deg roll in first second, instant kill
  (attempt at bag t≈17-18 s; the "flight" to z≈4.9 m afterwards is the tether hoist).
  actuator_motors undersampled (533 msgs).
- atmo_policy_20260818_1956xx/2001xx: arm-refused sessions (px4-relay on -> EKF invalid).
  High-rate "spins" in these bags are mocap artifacts/handling, NOT flight (rotors never armed).

## ulogs/ — PX4 logs 395-417 pulled over USB MAVLink (FC clock unset -> Y2K stamps)
- 395-405: PRE-8/14 era (old baud 921600, other users).
- 406 (4.0 s, SER_TEL2_BAUD=460800): today's A1 kill test — the ONLY log of the 460800 era,
  anchors the boundary. Everything after is tonight.
- 407-417: tonight's post-baud-fix armed sessions in order (11 arms incl. aborts).
  ~19-21 s runs = bench mission(s) + pre-flip flight; short runs (5.5/6.4/8.0/8.7/9.2 s) =
  the killed attempts and arm-abort cycles. Definitive per-run identity: cross-correlate
  actuator_motors against bag 200413's recorded rotors, then order falls out.

## ros_logs/ — launch logs (thin; node stdout lived in tmux)

## Open analysis tasks
1. Match ulogs 407-417 to runs (correlation anchor above).
2. Reconstruct policy semantic actions from ulog actuator_motors (invert mixer + motor filter).
3. Observation-replay: rebuild obs from bag pose/odometry, forward-pass the .npz, validate on
   200413, then read roll/pitch/yaw commands during both post-flip departures.
4. MIXER QUESTION (blocks flight): restrained roll/pitch/yaw action tests, props on. Two instant
   attitude departures post-flip vs one flyable pre-flip flight; team observed motors spinning
   opposite spec. Code state: flip IS deployed (Jetson bringup 7f557a4).
