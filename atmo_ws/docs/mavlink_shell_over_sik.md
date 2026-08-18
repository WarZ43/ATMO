# The mavlink shell without a Jetson USB port

Why the shell does not have to run on the vehicle, and what it costs to move it
off. Written 2026-08-14, immediately after the finding, before any of it was
proven on the SiK link.

**Status: the SiK route is UNVERIFIED. Do not unplug the Cube's micro-USB until
step 3 below has actually printed data.**

---

## The problem this solves

The Jetson has 4 USB ports and 5 things that want one:

```
1  WiFi dongle          <- this is your SSH. never unplug it.
2  CP2102 -> TELEM2     <- the uXRCE-DDS link
3  RoboClaw A
4  RoboClaw B
5  Cube micro-USB       <- the mavlink shell
```

Five into four. Something is always unplugged, and the previous session's
handoff resolved that by leaving a RoboClaw out — which is fine for Stage 0a
and impossible for Stage A, where both RoboClaws must be live.

That framing made the conflict look like a dependency between the shell and the
RoboClaws. It is not. They are unrelated subsystems that happen to compete for a
port. The shell needs a MAVLink path to the Cube and does not care which host
provides it.

## The route

TELEM1 already carries MAVLink to a SiK radio, NetID 25, and has been untouched
all along. **A ground-side SiK exists** — found 2026-08-14, not mentioned in any
prior handoff. Plug it into the ground station and run `mavlink_shell.py` there.

That consumes zero Jetson ports. With the Cube's micro-USB unplugged the budget
becomes 4 into 4: WiFi, CP2102, RoboClaw A, RoboClaw B — everything Stage A
needs, with the shell still available.

`listener actuator_armed 20` is a few bytes per line, comfortably inside SiK's
air rate. Bandwidth is not the risk here; linking is.

### Correcting the previous handoff

`session_state.md` says the Cube's micro-USB is "the ONLY practical shell". That
is narrower than it reads. It means **TELEM2 speaks XRCE-DDS, not MAVLink**, so
no shell is available over that port. It does not mean the Cube's micro-USB is
the only route to a shell. TELEM1/SiK is a second route.

## Bring it up in this order

The ordering is the point. Do not reorder it.

1. **Ground station sees the radio.** A COM port (Windows) or `/dev/ttyUSB*`
   (Linux) appears when the ground SiK is plugged in.
2. **`pymavlink` installed and `mavlink_shell.py` present** on the ground
   station. The only copy is `~/tools/mavlink_shell.py` on the Jetson; scp it.
3. **Prove the shell, with the Cube STILL CONNECTED.** You want an `nsh>`
   prompt, and `listener actuator_armed 20` returning real data. Keeping the
   Cube plugged in means a failure here costs nothing — you still have the
   working path.
4. **Only now unplug the Cube** and connect the second RoboClaw.

Step 3 is the whole safety property. Removing a working path before the
replacement is proven is how you end up with neither.

If it enumerates but will not link, check the ground radio matches the air side:
**NetID 25**, and both ends on the same air rate. Mission Planner or
`sik_uploader` can read the ground radio's params.

## Ground station: Windows vs Ubuntu

Measured on the Windows workstation, 2026-08-14: the ground SiK is a **CP2102**
(`USB\VID_10C4&PID_EA60`) and it came up **Code 28, `CM_PROB_FAILED_INSTALL`** —
the CP210x VCP driver has never been installed there. The chip enumerates; there
is nothing bound to it, so no COM port. Not a bad radio and not a bad cable.

(Ignore the FTDI `COM5` entry on that machine. `VID_0403+PID_6014`, status
`Unknown` — a different device, not currently plugged in, unrelated.)

On Linux `cp210x` is an in-tree kernel module, so the same radio is
`/dev/ttyUSB*` with no install step. **Decision: the ground station is the
Ubuntu dual-boot.** OptiTrack needs Ubuntu later anyway, so the ground station
and the mocap host converge on one machine.

Two Ubuntu-side gotchas, neither specific to this project:

- Be in `dialout`, or every open needs sudo:
  `sudo usermod -aG dialout $USER`, then log out and back in.
- ModemManager probes new serial devices and can hold the port. If the shell
  behaves strangely on first connect: `sudo systemctl stop ModemManager`.

## What this does not change

Kill polarity still has to be **measured**, and still cannot be measured by
`--mode kill` on this firmware — `/fmu/out/actuator_armed` is absent from this
build's `dds_topics.yaml`, so the script can only infer from `vehicle_status`,
which updates on transitions rather than continuously. The shell reads uORB
directly and works while disarmed. Moving the shell to the ground station
changes where you type, not what you have to establish:

```
listener actuator_armed 20
```

Toggle, and record which PHYSICAL switch position gives `manual_lockdown: true`.
Adopt PX4's polarity; do not rewrite code to match an assumption.
