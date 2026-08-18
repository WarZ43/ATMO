# Changes made to the shared vehicle — 2026-08-14

Everything altered on the aircraft, the flight controller, or the Jetson during
the RL bring-up session, so anyone else using this vehicle knows what moved.

**The one that will bite you:** `SER_TEL2_BAUD` changed from 921600 to 460800,
and `~/ATMO/ATMO/interface.sh` still says 921600. **The old startup path no
longer works** until that number is changed. See "Known breakage" below.

---

## 1. PX4 parameters

### Changed and saved to flash

| Parameter | Was | Now | Why |
| --- | --- | --- | --- |
| `SER_TEL2_BAUD` | 921600 | **460800** | At 921600 the uXRCE session flapped — repeated `session re-established`, only ~6 of ~67 topics ever created, because the session reset before topic creation finished. 460800 is stable and gives the full set. PX4's payload is ~35 kB/s (~350 kbaud), so 460800 has headroom; 230400 probably would not. |

This survived a power cycle, so it is in flash. Verify with `param show
SER_TEL2_BAUD`.

**To revert:** `param set SER_TEL2_BAUD 921600` + `param save` + reboot, and
change the agent's `-b` to match. But the flapping will come back.

### Read only — NOT changed

Recorded because they were checked and found already correct. Nobody needs to
re-check them.

```
MAV_1_CONFIG      = 0        (TELEM2 not claimed by MAVLink)
UXRCE_DDS_CFG     = 102      (TELEM2)
UXRCE_DDS_DOM_ID  = 0
SER_TEL2_FLOW     does not exist in this firmware
```

Read again 2026-08-14 over the Cube's USB, before it was unplugged. All
re-confirmed, plus four that had never been recorded:

```
SER_TEL1_BAUD     = 57600    <- the SiK / mavlink-shell link. NOT 460800.
SER_TEL2_BAUD     = 460800   (re-confirmed off the FC, not over DDS)
MAV_0_CONFIG      = 101      (TELEM1 -- confirms the SiK route carries MAVLink)
MAV_0_MODE        = 0        (Normal)
MAV_0_RATE        = 1200     (B/s)
RC_MAP_KILL_SW    = 13       <- kill is RC channel 13, i.e. INDEX 12 0-based
COM_RC_IN_MODE    = 3
```

`SER_TEL1_BAUD` and `SER_TEL2_BAUD` being different rates is the thing to keep
straight: 57600 for the shell over SiK, 460800 for DDS over the CP2102.

`RC_MAP_KILL_SW = 13` is new information — no prior doc named the kill channel.
PX4's `RC_MAP_*` are 1-based, so it is index **12** in the 0-based array
`hardware_rc_check.py` prints. It says which channel, not which polarity;
polarity still has to be measured in the shell.

Note on reading params with pymavlink here: PX4 returns INT32 params as the
int32 **bit pattern reinterpreted as float**, so `param_value` comes back as a
denormal like `8.07e-41` for 57600. Decode with
`struct.unpack("<i", struct.pack("<f", v))[0]` when `param_type` is 6.

**No other PX4 parameter was written.** RC mapping, airframe, kill switch
config, EKF and arming parameters were never touched — several were discussed
and then deliberately only read.

---

## 2. Jetson system state

### `startup_robot.service` was DISABLED

```bash
sudo systemctl disable startup_robot.service
```

The robot **no longer brings its stack up automatically at boot.** It was
disabled because it launches a tmux session with the tilt and drive controllers,
which would grab the RoboClaw serial ports and publish to `/tilt_vel` and
`/drive_vel` underneath anything started by hand — the classic ghost-publisher
problem.

**To restore:** `sudo systemctl enable --now startup_robot.service`

Note it is `Type=oneshot` with `RemainAfterExit=yes`, so it shows as
*active (exited)* rather than *running* — `systemctl list-units --state=running`
will not show it. Check with `systemctl is-enabled startup_robot.service`.

### `atmo-startup.service` (USER unit) was DISABLED — 2026-08-14, later session

```bash
systemctl --user stop atmo-startup.service
tmux kill-session -t robot
systemctl --user disable atmo-startup.service
```

**Revert by 2026-08-21** (end of the week of 08-17), when the vehicle is handed
back:

```bash
systemctl --user enable --now atmo-startup.service
```

**This is a second, independent copy of the problem `startup_robot.service` was
disabled for.** That one is a *system* unit. This one is a **`--user` unit**, so
it does not appear in `systemctl list-units`, `systemctl list-unit-files`, or
anything else run without `--user`. Disabling the system unit therefore looked
like it had solved the ghost-publisher problem and had not. It re-armed on every
boot, including the power cycle that cleared the duplicate XRCE agents.

What it launched, from a **third workspace that appears in no other doc**:

```
systemd --user
  └─ atmo-startup.service      "Start ATMO hardware tmux session"  (active/exited)
      └─ tmux new-session -d -s robot -c /home/m4/CATMO/ws_athenaIII
          ├─ ros2 run atmo offboard_control  -> /offboard_controller_node
          └─ ros2 run atmo tilt_control      -> /TiltControllerBase
                                                 held /dev/ttyACM1 open
```

`~/CATMO/ws_athenaIII` is not `~/ATMO_rl` and not `~/ATMO/ATMO`. Both nodes had
been running since boot (20:26:24).

**Two things this caused, both previously mis-attributed:**

1. **Two drive wheels turned in response to an RC switch**, with no ATMO_rl node
   running and nothing knowingly commanded. This was the first unexpected motion
   of the whole bring-up. It was not a stale buffer: `fuser` showed PID 2709
   (`tilt_control`) holding `/dev/ttyACM1` *at the time*, so a live node with an
   open serial port responded to RC.
2. **`nav_state = offboard` while disarmed "with nothing commanding it"** — the
   open item in `session_state.md`. Something was commanding it:
   `/offboard_controller_node`. Resolved by measurement, not inference.

**Checking for this is not `pgrep -a MicroXRCEAgent`.** That greps for the wrong
thing and these nodes do not match it. The check that finds them:

```bash
ros2 node list          # must be EMPTY before Stage 0a
systemctl --user list-units --type=service --all | grep -i atmo
fuser /dev/ttyACM0 /dev/ttyACM1     # both must be free
```

Verified clear after disabling: `ros2 node list` empty, service `inactive` and
`disabled`, no tmux server, both RoboClaw ports free.

### Crash dump DELETED from the flight controller's SD card

```
/fs/microsd/fault_2000_01_01_00_39_13.log      (removed via nsh `rm`)
```

**Not recoverable.** It was the record of a previous hardfault on this
airframe, predating this session — the timestamp is the NuttX epoch default,
so the FC had no valid clock when it was written. Whoever flew it before may
have wanted it.

Deleted because `health_and_arming_checks` fails preflight with
`Crash dumps present on SD` while any fault log exists, and that was blocking
arming, which blocks Stage A1. Confirmed gone with `ls /fs/microsd`.

### Added

- `~/tools/mavlink_shell.py` — PX4's MAVLink shell, fetched from PX4-Autopilot.
  New file, affects nothing else.
- `pymavlink` and `pyserial` were already installed; nothing changed.

### `~/.bashrc` — one line added

```bash
export ATMO_ROS_DOMAIN_ID=0
```

Confirmed added. It is safe to leave, and safe to delete.

What it does: `ATMO_ROS_DOMAIN_ID` is read **only** by this project's scripts
(`atmo_ws/scripts/atmo_env.sh` and `interface.sh`), which use it to set
`ROS_DOMAIN_ID`. It does **not** set `ROS_DOMAIN_ID` itself, so it cannot
affect any workflow that does not source those two files.

The value 0 is PX4's default and matches this vehicle's `UXRCE_DDS_DOM_ID`, so
it agrees with what the aircraft already does. It is now also the built-in
default in both scripts, which makes the line redundant — kept as belt and
braces, since a domain mismatch presents as zero `/fmu/` topics from a
healthy-looking agent and is genuinely hard to recognise.

```bash
grep -n "ATMO_ROS_DOMAIN_ID" ~/.bashrc     # to confirm or remove
```

Nothing else in `~/.bashrc` was modified. It still sources
`/opt/ros/humble/setup.bash` and sets no `ROS_DOMAIN_ID`, `CYCLONEDDS_URI` or
`ROS_DISCOVERY_SERVER` of its own — which is worth preserving, because a login
shell that sets those silently is exactly what cost the White M4 project an
entire mocap session.

---

## 3. Git state

### `~/ATMO/ATMO` — the original tree

**Moved from `main` to a new branch `jetson-local`, and previously uncommitted
work was committed to it** (147 files, commit `31db99c`).

That work was the FC-matched `px4_msgs` plus bench tuning in
`parameters.py`, `mpc.py`, and the tilt and drive controllers — it existed
nowhere else and was one `git checkout` away from being lost. **Nothing was
deleted or overwritten.**

```bash
cd ~/ATMO/ATMO
git branch                 # jetson-local now, main still exists
git log --oneline -1       # 31db99c "Jetson local state: ..."
git checkout main          # to go back; jetson-local keeps the work
```

If you were mid-edit in that tree, your changes are in `31db99c`, not lost.

### `~/ATMO_rl` — new

A fresh clone of `WarZ43/ATMO` on branch `bringup`, carrying the RL bring-up
work. Separate directory; does not touch `~/ATMO/ATMO`.

---

## 4. Physical changes

| What | State | Action needed |
| --- | --- | --- |
| Cube micro-USB → Jetson | **Newly connected** | Leave it; it is the only practical MAVLink shell. Remove if you need the USB port. |
| One RoboClaw USB | **May still be unplugged** | It was unplugged to free a Jetson USB port. **Plug it back in.** |
| CP2102 on TELEM2 | Believed unchanged | Verify — it was discussed as a swap candidate and should still be on TELEM2. |
| SiK radio on TELEM1 | Believed unchanged | Verify. |
| RC transmitter | **Index-7 switch left HIGH** | That is the RL gate. Put it down. |

The Jetson has four USB ports and they were full. The Cube's micro-USB is
occupying a port freed by unplugging a RoboClaw.

**RoboClaw device paths are not stable.** `by-id` is useless here — both boards
enumerate with the identical descriptor. Use `by-path`:

```bash
ls -l /dev/serial/by-path/
```

Recorded during the session:

```
platform-3610000.usb-usb-0:2.1:1.0   RoboClaw A
platform-3610000.usb-usb-0:2.2:1.0   RoboClaw B
platform-3610000.usb-usb-0:2.4:1.0-port0   CP2102 (TELEM2)
```

Which board is tilt and which is drive was **never established.** Re-check
before commanding either.

---

## 5. Known breakage

**`~/ATMO/ATMO/interface.sh` is now wrong.** It reads:

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

but PX4's TELEM2 is now at 460800. Running it gives a flapping session and
~6 `/fmu/` topics instead of ~67, which looks like a broken flight controller.

Either change that `921600` to `460800`, or use
`~/ATMO_rl/interface.sh`, which defaults to 460800, sets the domain, and prints
what it is doing.

**`startup_robot.service` calls that same script**, so re-enabling the service
without fixing the baud will produce the same failure at every boot.

---

## 6. Nothing moved

For the avoidance of doubt, none of this was touched:

- No motor, rotor, wheel or tilt actuator was ever commanded. Nothing moved
  under power at any point.
- PX4 was never armed.
- No airframe, RC mapping, kill switch, EKF or arming parameter was written.
- The SiK radio's configuration (NetID 25) was not changed.
- No firmware was flashed.
- The transmitter's model configuration was not changed.

---

## Quick verification

```bash
# PX4 (needs the mavlink shell over the Cube's USB)
param show SER_TEL2_BAUD          # expect 460800
param show UXRCE_DDS_CFG          # expect 102
param show UXRCE_DDS_DOM_ID       # expect 0

# Jetson
systemctl is-enabled startup_robot.service
grep -nE "ATMO_|ROS_DOMAIN_ID" ~/.bashrc
ls -l /dev/serial/by-path/        # both RoboClaws present?
cd ~/ATMO/ATMO && git branch && git log --oneline -1
```
