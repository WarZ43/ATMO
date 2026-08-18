# OptiTrack bring-up

How to get Motive's rigid-body pose onto ATMO as `/atmo/groundtruth_odom` and,
optionally, into the PX4 EKF as `/fmu/in/vehicle_visual_odometry`.

`docs/hardware_bringup.md` Stage C is the gate list: what must be measured and
what counts as a pass. This document is the setup that has to work before any
of those gates can run, plus the failures that setup produces and how to tell
them apart. Read them together — get the stream up with this document, then
prove it correct with Stage C.

`docs/optitrack_session_checklist.md` is the linear run sheet: the same
procedure as commands and blanks to fill in, with no rationale. Work from that
at the bench and come here when something fails.

Almost everything below was learned on the White M4 rig with the same Motive
installation and the same laptop. It is carried over because the network and
frame problems are properties of the lab, not of the airframe.

## Topology

```
OptiTrack cameras ──wired──> Motive server ──wired eth──> LAPTOP ──wifi──> router ──wifi──> ATMO
                                                          (Ubuntu)                            │
                                                       vrpn_mocap node                  mocap_bridge
                                                  /vrpn_mocap/<body>/pose                     │
                                                                                /atmo/groundtruth_odom
                                                                          /fmu/in/vehicle_visual_odometry
                                                                                              ↓
                                                                                            EKF2
```

Two properties of this topology drive everything below.

**The laptop is dual-homed.** Wired to the mocap subnet, WiFi to the router.
DDS announces a participant on every interface it can see unless told not to,
so the robot can discover the VRPN publisher and then be handed a locator on
the mocap subnet it has no route to. This is the single most likely thing to go
wrong, and it presents as "the topic is listed but `hz` says nothing".

**The only mocap link to the vehicle is WiFi.** Stage C3's gate is under two
policy steps — 40 ms at 50 Hz — *including network transport*. A contended
2.4 GHz link will not hold that. If C3 fails, the fix is topological (move to
5 GHz, move the router, reduce contention), not a parameter.

## Where the converter runs

`mocap_bridge` runs **on the robot**, not the laptop. The laptop publishes only
the raw VRPN pose. Two reasons: the robot is the machine that must fail safe if
the link degrades, so it should be the one measuring staleness; and the bridge's
velocity filter is tuned against the arrival times it actually sees.

## ROS 2 environment

Both machines must agree on all three of these. `scripts/atmo_env.sh` sets them
per-window, and `scripts/check_host.sh` prints them for comparison.

```bash
export ROS_DOMAIN_ID=0        # must equal PX4's UXRCE_DDS_DOM_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
unset ROS_DISCOVERY_SERVER
```

The domain must equal the flight controller's `UXRCE_DDS_DOM_ID` parameter,
which decides where the uXRCE bridge publishes. It defaults to 0 and this
vehicle is measured at 0. Get it wrong and you see zero `/fmu/` topics with
a perfectly healthy-looking agent.

**This is where an entire m4 session was lost.** The robot's `~/.bashrc`
sourced a different autonomy stack that set `ROS_DOMAIN_ID=0`, CycloneDDS and a
`CYCLONEDDS_URI`. Every network-layer test passed and the two machines still
could not see each other. A leftover `ROS_DISCOVERY_SERVER` from earlier
debugging then kept Fast DDS off multicast discovery even after the domain was
fixed. If ATMO's onboard computer sources anything at login, put the override
block at the **end** of `~/.bashrc` so it wins, and accept that the other stack
loses new-shell visibility — that is the intent.

Run `ros2 daemon stop` after changing any of these. A stale daemon reports the
old graph and reads exactly like a broken network.

## Motive

Record these before leaving the lab, because the bridge is configured from them:

- **Rigid body name** — becomes the topic segment. The default here is
  `m4_base`, inherited from the ATMO relay. Pass the real one with
  `-p body:=<name>`.
- **Streaming up axis** — decides `-p source_frame:=y_up|z_up`. Motive's
  default is y-up; the m4 rig was configured z-up. Do not assume.
- **Streaming rate** — sets `-p expected_rate_hz:=`. The bridge derives its
  velocity low-pass alpha from this, so streaming at 120 Hz while the bridge
  assumes 100 silently mistunes the filter. That surfaces later as a C2 twist
  noise failure rather than as anything resembling a rate problem.
- **Pivot offset** — if the marker plate is not at the body origin, apply the
  offset in Motive rather than in code, so every consumer agrees.

## Laptop: VRPN client

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=<motive-server-ip> port:=3883
```

`server` and `port` are the only arguments. `server:=` **must** be passed — the
`localhost` default connects to nothing and presents identically to Motive not
streaming.

```bash
ros2 topic list | grep vrpn
ros2 topic hz /vrpn_mocap/m4_base/pose
```

If the client connected but no topic appears, Motive is not tracking the body.
Check that the rigid body is selected and visible in Motive — this is not a
network problem.

Before the ROS layer, prove the wired side:

```bash
ping <motive-server-ip>
nc -vz <motive-server-ip> 3883
```

## Robot: the bridge

```bash
ros2 run atmo mocap_bridge --ros-args \
  -p body:=m4_base \
  -p source_frame:=y_up \
  -p expected_rate_hz:=120.0 \
  -p px4_relay:=false
```

Start with `px4_relay:=false`. Prove the frame with Stage C1 and C2 **before**
anything reaches the EKF, so a frame error cannot present as an estimator
problem.

The bridge logs a rate and gap summary every five seconds. A stream that
degrades rather than stops is the failure mode that looks like bad control.

### Frame validation before the relay

```bash
python3 scripts/hardware_optitrack_check.py --mode pose
python3 scripts/hardware_optitrack_check.py --mode twist
python3 scripts/hardware_optitrack_check.py --mode rate --duration 30
python3 scripts/hardware_optitrack_check.py --mode static
```

Each mode prints the expected signs **before** you move anything. That ordering
is deliberate: a test that shows the number first invites you to rationalise it.

### PX4 relay

Only after C1 and C2 pass:

```bash
ros2 run atmo mocap_bridge --ros-args -p px4_relay:=true
```

Then confirm the EKF is actually fusing it — by measurement, not by name:

```bash
# In the mavlink shell
uorb top
listener vehicle_visual_odometry
listener vehicle_local_position
```

`EKF2_EV_CTRL` must be set for external vision to be used at all, and
`EKF2_EV_DELAY` should be set from the C3 measurement.

### The PX4 quaternion is an open question

`relay_mocap.py` mapped the raw streamed quaternion to NED as `(w, -z, x, -y)`.
Composing y-up → z-up → NED, as the m4 bridge does, gives `(w, x, z, -y)`.
**These are not the same rotation.** The position halves of the two chains do
agree — there is a test pinning that — so the disagreement is specifically
about attitude.

`mocap_bridge` defaults to `px4_quaternion:=atmo_legacy`, the mapping that has
flown on this airframe. `px4_quaternion:=composed` selects the other. Which is
correct depends on how the rigid body was defined in Motive, so it is a
measurement, not a preference. Settle it in Stage C with the vehicle pitched and
rolled by hand while watching `listener vehicle_attitude`, and do not fly a
changed value that has not been confirmed by motion.

## Discovery fallback

Many access points block or rate-limit multicast, which breaks DDS discovery
outright. Test early — on the robot `ros2 multicast receive`, on the laptop
`ros2 multicast send`.

**A raw multicast test passing does not mean DDS discovery works.** It is plain
UDP and does not exercise locator announcement, which is the actual dual-homing
failure. Do not conclude anything from it passing; only from it failing.

If it fails, run a discovery server on the laptop rather than fighting the
network:

```bash
fast-discovery-server -i 0 -p 11811
```

Then on both machines, followed by `ros2 daemon stop`:

```bash
export ROS_DISCOVERY_SERVER=<laptop-wifi-ip>:11811
```

This sidesteps dual-homing entirely, because discovery becomes unicast to an
address named explicitly. When a session is time-constrained, going straight to
the discovery server is the pragmatic choice.

The tool is `fast-discovery-server`, not `fastdds discovery --port` — that flag
spelling is rejected — and it needs ROS sourced or it fails on
`libfastrtps.so.2.6`.

### The CLI goes blind under a discovery server

**A plain discovery-server CLIENT learns only the endpoints it needs, so
`ros2 node list` and `ros2 topic list` come back empty while data flows
perfectly.** This cost an evening on the m4 rig: every introspection command
reported nothing, which reads identically to a dead stream.

CLI tools must join as a **SUPER_CLIENT**, which is a profile setting rather
than an environment variable. `scripts/fastdds_super_client.xml` is committed so
`git pull` carries it to the robot — it is needed on both machines, and a file
in `~` does not travel.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=<workspace>/scripts/fastdds_super_client.xml
```

Update the laptop's WiFi address inside the XML at the start of each session;
campus DHCP reassigns it. Unset `ROS_DISCOVERY_SERVER` in shells using this
profile — the server address is in the XML, and setting both invites a conflict.

### The interface whitelist did not work here

`scripts/fastdds_wifi_only.xml` is kept for reference but **was measured not to
work on this lab network**. It was tried three times and broke the stream each
time, including the laptop's own topic, because `useBuiltinTransports=false`
also disables shared memory and forces same-host participants onto one NIC.
Adding an explicit SHM descriptor alongside the whitelisted UDP did not rescue
it. Use the discovery server instead.

## Order of operations

1. Motive streaming, rigid body named and tracking.
2. Wired link to Motive: `ping`, then `nc -vz <ip> 3883`.
3. Environment exported on both machines, `ros2 daemon stop`.
4. Multicast test both directions, or go straight to the discovery server.
5. VRPN client up; rate verified **on the laptop**.
6. Same topic and rate verified **on the robot**. This is the multi-machine gate
   and the one most often skipped.
7. `mocap_bridge` with `px4_relay:=false`.
8. `hardware_optitrack_check.py` — pose signs, twist, rate, static noise.
   Stage C1 through C4.
9. `px4_relay:=true`, `EKF2_*` parameters, `vehicle_local_position` valid.

## Symptom to cause

| Symptom | Cause |
| --- | --- |
| No `/vrpn_mocap/*` topics on the laptop | Motive not streaming, or the rigid body not tracking |
| `nc -vz` to 3883 fails | Wrong wired subnet, or the VRPN Streaming Engine is off |
| Topic listed on the robot, `hz` silent | Dual-homing; use the discovery server |
| `[TRANSPORT Error] All whitelist interfaces were filtered out` | The address in the whitelist XML is not on this machine — DHCP moved the WiFi IP. That shell has no transport at all |
| Neither machine sees the other | Multicast blocked; use the discovery server |
| Topics appear and then vanish | Mismatched `ROS_DOMAIN_ID` between terminals |
| `ros2 topic list` empty while data flows | Discovery-server client without the SUPER_CLIENT profile |
| `ros2 topic list` disagrees with reality | Stale daemon; `ros2 daemon stop` |
| Rate good on the laptop, gappy on the robot | WiFi contention |
| Height negative or nonsensical | `source_frame` set against Motive's actual up axis |
| Twist noisy beyond the C2 limits under rotation | Rigid body pivot offset from the body origin |
| Twist correct at zero yaw, wrong when rotated | Body-versus-world frame error |
| EKF ignores the relay entirely | Wrong `/fmu/in/` topic, or `EKF2_EV_CTRL` unset |
| EKF fights every real motion | Velocity published as zeros rather than NaN — see below |
| Everything plausible but the EKF rejects it | `px4_msgs` version mismatch against the flight controller |
| Rigid body lost during the drive-to-flight morph | Markers on a moving arm, or posture-dependent occlusion |

### The zeros-versus-NaN trap

`relay_mocap.py` published `velocity = [0, 0, 0]` with `velocity_frame` set.
PX4 does not read that as "no velocity supplied" — it reads it as a measurement
that the vehicle is stationary, and fuses it against every real motion. NaN is
how a field is declined. Both the legacy relay and `mocap_bridge` now send NaN
unless `publish_velocity:=true` is set, in which case a real derivative goes out
instead.

## Open on arrival

Cheap to settle on site, expensive to guess at.

1. **The rigid body name in Motive.** Defaults here to `m4_base`.
2. **Motive's streaming up axis**, which decides `source_frame`, and its actual
   rate, which `expected_rate_hz` must match.
3. **The PX4 quaternion convention** — `atmo_legacy` versus `composed`, above.
4. **Which `/fmu/in/` topic EKF2 fuses.** Settle with `uorb top`, not by name.
5. **The marker plate offset** from the body origin, applied in Motive.
6. **Whether the onboard computer runs anything at boot** that contends for the
   PX4 actuator path or the RoboClaw serial port. Confirm it is disabled.
