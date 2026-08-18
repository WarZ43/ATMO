# PX4 uXRCE-DDS topic inventory

Measured on the vehicle, 2026-08-14. **Topic names are not stable across PX4
versions** — 1.16 introduced message versioning on the DDS interface, and some
topics gained a `_v1` suffix while others did not. It is per-message, not a
blanket rename, so it cannot be guessed.

A wrong name here does not crash. The subscription simply never fires, and the
node runs looking healthy while blind on that signal. `atmo/px4_topics.py`
holds every name in one place, tries fallbacks, and warns at startup about
anything not on the graph.

## This vehicle

```
HW arch:      CUBEPILOT_CUBEORANGE
PX4 version:  1.17.0   git 8874d533   branch main
Build:        2025-11-26
Transport:    TELEM2 -> CP2102 -> Jetson /dev/ttyUSB0
UXRCE_DDS_CFG:     102 (TELEM2)
UXRCE_DDS_DOM_ID:  0
SER_TEL2_BAUD:     460800 (921600 flapped -- see "Session stability")
```

`ROS_DOMAIN_ID` on the Jetson must equal `UXRCE_DDS_DOM_ID`, so **0** here.
`atmo_env.sh` and `interface.sh` both default to 0; override with
`ATMO_ROS_DOMAIN_ID` if a future vehicle differs. Check it on the vehicle
rather than assuming, because a mismatch gives zero `/fmu/` topics from an
agent that looks healthy.

## What ATMO uses, and whether it exists

| ATMO topic | on this firmware | note |
| --- | --- | --- |
| `/fmu/in/actuator_motors` | yes | |
| `/fmu/in/offboard_control_mode` | yes | |
| `/fmu/in/vehicle_command` | yes | |
| `/fmu/in/vehicle_visual_odometry` | yes | mocap relay target |
| `/fmu/in/trajectory_setpoint` | yes | MPC only |
| `/fmu/in/vehicle_thrust_setpoint` | yes | MPC only |
| `/fmu/in/tilt_angle` | **NO** | see below |
| `/fmu/out/input_rc` | yes | |
| `/fmu/out/vehicle_odometry` | yes | |
| `/fmu/out/vehicle_command_ack` | yes | |
| `/fmu/out/vehicle_control_mode` | yes | |
| `/fmu/out/vehicle_status` | **renamed** | `/fmu/out/vehicle_status_v1` |
| `/fmu/out/battery_status` | **renamed** | `/fmu/out/battery_status_v1` |
| `/fmu/out/esc_status` | **NO** | not published at all |
| `/fmu/out/actuator_armed` | **NO** | carries the kill state -- see below |
| `/fmu/out/*_groundtruth` | no | simulator only, irrelevant on hardware |

Only **one** runtime subscription was actually broken: `vehicle_status`. The
rest of the RL hardware path was already correct.

### Versioned on 1.17

```
/fmu/in/arming_check_reply_v1            /fmu/out/airspeed_validated_v1
/fmu/in/config_overrides_request_v1      /fmu/out/arming_check_request_v1
/fmu/in/register_ext_component_request_v1 /fmu/out/battery_status_v1
/fmu/in/vehicle_attitude_setpoint_v1     /fmu/out/home_position_v1
                                         /fmu/out/register_ext_component_reply_v1
                                         /fmu/out/vehicle_local_position_v1
                                         /fmu/out/vehicle_status_v1
```

Everything else in the list is unversioned. Notably `vehicle_odometry`,
`vehicle_attitude`, `vehicle_control_mode`, `input_rc` and `actuator_motors`
are all plain — which is why most of the stack worked.

## `tilt_angle` never reaches PX4

`TiltAngle` is ATMO's own message, and `/fmu/in/tilt_angle` is **not in this
firmware's `dds_topics.yaml`**. So:

- Node-to-node it works. `TiltControllerBase` publishes it, the RL runtime
  subscribes, and the tilt angle reaches the policy observation normally.
- **PX4 itself never receives it.** Anything in the firmware that expects to
  know the arm tilt — control allocation on a tiltrotor airframe, for one —
  is not getting it.

Whether that matters depends on how this airframe is configured in PX4. It is
recorded here because it is invisible otherwise: nothing errors, the topic just
has no subscriber on the flight controller side. Adding it needs a
`dds_topics.yaml` edit and a firmware rebuild.

## `actuator_armed` is absent, so kill state cannot be read from ROS

**Field name warning, measured 2026-08-14.** On this firmware `listener
actuator_armed` reports `lockdown` and `kill`; there is **no**
`manual_lockdown`, despite the vendored `ActuatorArmed.msg` declaring one.
Read `kill`. Everything below that says `manual_lockdown` predates that
measurement; the field it means is `kill`.

`ActuatorArmed.manual_lockdown` is the direct "is the kill switch engaged"
signal, and `/fmu/out/actuator_armed` is not in this firmware's
`dds_topics.yaml`. `hardware_rc_check.py --mode kill` therefore falls back to
inferring from `vehicle_status`, using `latest_arming_reason == KILL_SWITCH`
and `nav_state == TERMINATION`.

**That fallback only updates on a transition**, so it cannot establish kill
polarity while the vehicle is disarmed -- which is exactly when you want to
establish it.

Measure it in the mavlink shell instead, which reads the uORB topic directly
and works disarmed:

```
listener actuator_armed 20
```

Toggle the switch and watch `manual_lockdown`. Record which PHYSICAL position
gives `true`. On the White M4 this was inverted from what the software assumed
and every rotor test looked silently dead.

Adding `actuator_armed` to `dds_topics.yaml` and rebuilding the firmware would
let the ROS tooling gate on it directly. Worth doing before flight testing, so
the kill state is in the rosbag rather than only on a console.

## Session stability

The agent showed repeated `session re-established` at 921600 with only ~6 of
~67 topics created, because the session reset before topic creation finished.
Symptoms of an over-fast or marginal serial link:

```
create_client -> establish_session -> re-established -> re-established -> ...
```

A healthy start is **one** `create_participant` followed by a burst of
`create_topic` / `create_datawriter`, then quiet.

CONFIRMED on this vehicle: 921600 flapped, **460800 is stable** and gives
the full ~67 topics. Both ends were changed together, and `interface.sh`
defaults to 460800 accordingly.
PX4's payload was ~34.8 kB/s ≈ 350 kbaud, so 460800 has headroom and 230400
may not. Set it in `nsh` with `param set` **and `param save`**, then reboot —
without the save it does not survive.

## Re-taking this inventory

After any firmware change:

```bash
ros2 topic list | grep fmu | sort
```

Then update `atmo/px4_topics.py`. Every name is overridable without a code
edit — `ATMO_PX4_TOPIC_VEHICLE_STATUS=/fmu/out/vehicle_status`, and so on —
which is the fast path when you are at the bench and just need it to run.

`px4_topics.warn_missing()` runs at node startup and lists anything absent, so
a firmware change reports itself instead of presenting as a dead signal.
