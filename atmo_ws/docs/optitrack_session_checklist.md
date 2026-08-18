# OptiTrack session run sheet

Linear. Commands and blanks, no rationale — that is in
`docs/optitrack_bringup.md`. Work down this sheet; when something fails, go
there.

Fill the blanks in as you go. A number you did not write down is a number you
will re-measure next session.

```
Date ____________  Operators ______________________________
Laptop WiFi IP ______________  Robot IP ______________
Motive server IP ______________  Rigid body name ______________
Motive up axis  [ ] y-up  [ ] z-up      Motive rate ________ Hz
```

## 1. Rigid body

- [ ] Markers mounted, not on a tilting arm (they occlude through the morph)
- [ ] Rigid body defined and tracking in Motive
- [ ] Pivot offset applied in Motive, not in code
- [ ] Body name recorded above

## 2. Motive streaming

- [ ] VRPN Streaming Engine ON
- [ ] Up axis and rate recorded above

## 3. Environment — BOTH machines

```bash
source scripts/atmo_env.sh
bash scripts/check_host.sh
ros2 daemon stop
```

- [ ] `ROS_DOMAIN_ID` matches on both: ________
- [ ] `RMW_IMPLEMENTATION` matches on both: ________
- [ ] `CYCLONEDDS_URI` unset on both
- [ ] `ROS_DISCOVERY_SERVER` unset on both (or set identically, step 5)

## 4. Wired link to Motive — laptop

```bash
ping <motive-server-ip>
nc -vz <motive-server-ip> 3883
```

- [ ] Both succeed

## 5. Discovery

```bash
# robot
ros2 multicast receive
# laptop
ros2 multicast send
```

- [ ] Arrives → continue on plain discovery
- [ ] Does not arrive → discovery server:

```bash
# laptop
fast-discovery-server -i 0 -p 11811
# both machines, then ros2 daemon stop
export ROS_DISCOVERY_SERVER=<laptop-wifi-ip>:11811
# introspection terminals only
export FASTRTPS_DEFAULT_PROFILES_FILE=<workspace>/scripts/fastdds_super_client.xml
```

Remember to update the laptop IP inside `fastdds_super_client.xml`.

## 6. VRPN client — laptop

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=<motive-server-ip> port:=3883
ros2 topic hz /vrpn_mocap/<body>/pose
```

- [ ] Topic present, rate ________ Hz

## 7. Multi-machine gate — on the ROBOT

```bash
ros2 topic list | grep vrpn
ros2 topic hz /vrpn_mocap/<body>/pose
```

- [ ] Topic present on the robot, rate ________ Hz

Do not continue past this line until the robot sees the rate. This is the step
most often skipped and the one that costs the session.

## 8. Bridge — on the robot

```bash
ros2 run atmo mocap_bridge --ros-args \
  -p body:=<body> -p source_frame:=<y_up|z_up> \
  -p expected_rate_hz:=<rate> -p px4_relay:=false
```

- [ ] Bridge reports a steady rate, 0 gaps
- [ ] `ros2 topic hz /atmo/groundtruth_odom` = ________ Hz

## 9. C1 — pose signs

```bash
python3 scripts/hardware_optitrack_check.py --mode pose
```

Read the printed expectations FIRST, then move the vehicle.

- [ ] +x forward   [ ] +y left   [ ] +z up
- [ ] yaw CCW positive   [ ] roll right-down positive   [ ] pitch nose-up positive
- [ ] resting z small and positive: ________ m

## 10. C2 — twist signs and body frame

```bash
python3 scripts/hardware_optitrack_check.py --mode twist
```

- [ ] linear signs correct
- [ ] angular signs correct
- [ ] Held at a fixed tilt, body-frame linear twist CHANGES with the tilt for
      the same world motion (if it does not, the twist is world-frame)

## 11. C3 — rate and latency

```bash
python3 scripts/hardware_optitrack_check.py --mode rate --duration 30
```

- [ ] mean rate ________ Hz
- [ ] p95 gap ________ ms
- [ ] worst gap ________ ms  (must be under 40 ms)
- [ ] dropouts ________ (must be 0)

If the worst gap exceeds 40 ms the fix is topological — 5 GHz, router placement,
contention — not a parameter.

Set `EKF2_EV_DELAY` from the measured transport latency: ________ ms

## 12. C4 — static noise

```bash
python3 scripts/hardware_optitrack_check.py --mode static
```

- [ ] position sigma ________ m    (limit 0.005)
- [ ] velocity sigma ________ m/s  (limit 0.035)

## 13. PX4 relay — only after C1 and C2 passed

```bash
ros2 run atmo mocap_bridge --ros-args -p body:=<body> \
  -p source_frame:=<y_up|z_up> -p px4_relay:=true
```

In the mavlink shell:

```bash
uorb top
listener vehicle_visual_odometry
listener vehicle_local_position
```

- [ ] `EKF2_EV_CTRL` set
- [ ] `EKF2_EV_DELAY` = ________ ms
- [ ] `vehicle_local_position` valid, and tracks hand motion
- [ ] Attitude tracks hand pitch and roll correctly
      → decides `px4_quaternion`: [ ] atmo_legacy  [ ] composed

## 14. Shadow run on live mocap

```bash
./atmo_session.sh shadow --mocap-body <body> --mocap-frame <y_up|z_up>
```

- [ ] Contract check passes (or its warning is understood)
- [ ] Policy output finite, collective near hover
- [ ] Attitude commands OPPOSE a hand-held tilt
- [ ] Shadow log written to `shadow_logs/`

## Stop conditions

Stop the session and do not proceed if any of these happen:

- The rigid body drops tracking anywhere in the working volume
- Worst gap over 40 ms and it cannot be fixed topologically
- Pose or twist signs disagree with the expectations and the disagreement is
  not explained by a `source_frame` change
- The EKF rejects or fights the vision estimate
- The contract check fails and the mismatch is not understood

## Realistic pacing

The m4 rig's first OptiTrack session got through C1 and C2 only, and most of it
went on the discovery failure in step 5. Budget a full session for steps 1–10
and do not plan to fly on the same day.
