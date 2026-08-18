#!/usr/bin/env python3
"""Operator gate and homing sequence for an ATMO shadow (or policy) run.

The order is the safety property, and it is the same shape as the m4
bring-up's operator gate:

  1. PREFLIGHT, read-only. Confirm RC is live, the kill switch is RELEASED,
     and the RL gate is DOWN. Refuse to continue otherwise -- starting with
     the gate already raised means the first thing that happens after homing
     is motion nobody asked for.
  2. Operator presses Enter.
  3. HOME THE TILT to the FLY hard stop and zero the encoder there. Homing
     is the only way the encoder means anything: the count lives in board RAM,
     survives Jetson reboots, and is redefined wherever the node last started.
  4. Operator presses Enter again, then raises the RL switch. Only then does
     anything run.

Homing goes to FLY, NEVER to drive, since 2026-08-17: the drive end (~90 deg)
carries a hardware kill switch that cuts the tilt motor circuit, and a power
cycle on that switch creates an electronic DEAD ZONE (the interlock assumes
the last motion was upward and inhibits the only direction that leads out;
escape needs Basicmicro Motion Studio). The fly end is safe to touch and safe
to power off on. Homing here therefore only ever commands the arm UPWARD,
toward fly. The tilt node reads the resulting frame via ATMO_TILT_HOME=fly.

WHAT SHADOW DOES NOT DO: it does not move the vehicle. `_shadow_timer_callback`
reads the observation, runs the policy, computes the command and writes it to
a log; it never publishes an actuator command. Pass --mode policy if you want
the computed command actually applied.

    python3 scripts/shadow_test.py                     # observation only
    python3 scripts/shadow_test.py --mode policy       # ACTUATES
    python3 scripts/shadow_test.py --skip-homing       # tilt already homed
"""

import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "atmo"))

from atmo.mpc.roboclaw_3 import Roboclaw                     # noqa: E402
from atmo.roboclaw_safety import (                           # noqa: E402
    DEFAULT_ADDRESS, KILL_CHANNEL, RL_CHANNEL, kill_engaged, stop_all)

TILT_PORT = os.getenv("ATMO_TILT_ROBOCLAW",
                      "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0")
POLICY = os.getenv("ATMO_RL_POLICY_PATH",
                   os.path.expanduser("~/policies/atmo_combined_stage1_policy.npz"))
# Measured: escaping a hard stop needs ~80; 100 wedges the arm into one.
HOMING_DUTY = int(os.getenv("ATMO_TILT_HOMING_DUTY", "70"))
HOMING_TIMEOUT_S = 30.0


def read_rc(timeout=15.0):
    """One InputRc frame via `ros2 topic echo`, as values[]."""
    import re
    try:
        out = subprocess.run(
            ["ros2", "topic", "echo", "--once", "/fmu/out/input_rc"],
            capture_output=True, text=True, timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return None
    m = re.search(r"values:\s*\n((?:\s*-\s*\d+\n)+)", out)
    if not m:
        return None
    return [int(v) for v in re.findall(r"-\s*(\d+)", m.group(1))]


def preflight():
    print("=" * 68)
    print("PREFLIGHT -- read only, nothing is commanded")
    print("=" * 68)
    values = read_rc()
    if not values:
        print("  RC        NO FRAME. Is the agent running and the transmitter on?")
        return False
    kill = values[KILL_CHANNEL] if len(values) > KILL_CHANNEL else None
    gate = values[RL_CHANNEL] if len(values) > RL_CHANNEL else None
    killed = kill_engaged(values)
    gate_up = gate is not None and gate >= 1834

    print("  RC frame        %d channels" % len(values))
    print("  kill  ch%-2d      %s us   -> %s"
          % (KILL_CHANNEL + 1, kill, "ENGAGED (killed)" if killed else "released"))
    print("  RL gate ch%-2d    %s us   -> %s"
          % (RL_CHANNEL + 1, gate, "RAISED" if gate_up else "down"))
    print()

    ok = True
    if killed:
        print("  REFUSING: the kill switch is engaged. Release it "
              "(HIGH is released on this vehicle).")
        ok = False
    if gate_up:
        print("  REFUSING: the RL gate is already RAISED. Put it DOWN before "
              "homing, or the run starts the moment homing finishes.")
        ok = False
    if ok:
        print("  OK: kill released, RL gate down.")
    return ok


def home_tilt_to_fly():
    """Drive UP to the FLY hard stop and zero the encoder there.

    Direction is BackwardM2 = toward fly = upward, and only ever upward:
    the drive end carries the kill switch and its dead zone (see the module
    docstring). Duty 60-70 toward fly from mid-travel is the measured
    working range (2026-08-14); duty 100 wedges the arm into the stop.
    """
    print("=" * 68)
    print("HOMING the tilt UP to the FLY hard stop (never toward drive)")
    print("=" * 68)
    rc = Roboclaw(TILT_PORT, 115200)
    if not rc.Open():
        print("  could not open %s" % TILT_PORT)
        return False

    def enc():
        r = rc.ReadEncM2(DEFAULT_ADDRESS)
        return r[1] if r and r[0] else None

    def speed():
        r = rc.ReadSpeedM2(DEFAULT_ADDRESS)
        return r[1] if r and r[0] else None

    print("  start enc = %s, driving BackwardM2 (toward FLY) at duty %d"
          % (enc(), HOMING_DUTY))
    # Three consecutive updates with no movement IS the hard stop.
    #
    # This used to require the arm to move first before it would accept a
    # stall, which meant homing could never succeed when the arm was ALREADY
    # sitting on the stop -- the common case after a previous run. Not moving
    # is the condition we are looking for, whether or not we got there just
    # now.
    #
    # The tradeoff, stated: a motor that cannot move for some OTHER reason
    # (wedged, unpowered, a dead driver) also reads as "at the stop", and the
    # encoder gets zeroed wherever it happens to be. That shows up immediately
    # as a wrong tilt angle, so check the reported angle after homing rather
    # than assuming this succeeded.
    STILL_COUNTS = 5      # encoder counts; below this is "not moving"
    STILL_UPDATES = 3
    still = 0
    last = enc()
    reached = False
    try:
        rc.BackwardM2(DEFAULT_ADDRESS, HOMING_DUTY)
        t0 = time.time()
        while time.time() - t0 < HOMING_TIMEOUT_S:
            time.sleep(0.3)
            s, c = speed(), enc()
            delta = None if (c is None or last is None) else abs(c - last)
            print("    enc=%-9s speed=%-7s delta=%s" % (c, s, delta))
            if delta is not None and delta < STILL_COUNTS:
                still += 1
                if still >= STILL_UPDATES:
                    print("    no movement for %d updates -> at the FLY hard stop"
                          % STILL_UPDATES)
                    reached = True
                    break
            else:
                still = 0
            last = c
        else:
            print("    TIMEOUT: still moving after %.0fs, never reached a stop"
                  % HOMING_TIMEOUT_S)
    finally:
        stop_all(rc, DEFAULT_ADDRESS)

    if not reached:
        try:
            rc._port.close()
        except Exception:  # noqa: BLE001
            pass
        return False

    # "Not moving" is also what an INHIBITED motor looks like (kill-switch
    # lockout 0x20xxxxxx, driver fault 0x40/0x80). Zeroing there would plant
    # the fly zero at some arbitrary angle. The status register tells the two
    # apart, so refuse rather than mis-zero.
    ok_err, err = rc.ReadError(DEFAULT_ADDRESS)
    if ok_err and (err & 0x200000C0):
        print("  REFUSING to zero: board status 0x%08X shows an inhibit or "
              "driver fault, not a hard stop. Kill lockout? Cycle the kill "
              "lever; driver fault? Check the tilt motor circuit." % err)
        try:
            rc._port.close()
        except Exception:  # noqa: BLE001
            pass
        return False

    time.sleep(0.6)
    print("  SetEncM2(0) here -> FLY = 0 counts = 0 deg")
    ack = rc.SetEncM2(DEFAULT_ADDRESS, 0)
    time.sleep(0.3)
    print("  ack=%s, encoder now reads %s" % (ack, enc()))
    try:
        rc._port.close()
    except Exception:  # noqa: BLE001
        pass
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="ground",
                    choices=("shadow", "ground", "policy"))
    ap.add_argument("--skip-homing", action="store_true")
    ap.add_argument("--policy", default=POLICY)
    args = ap.parse_args(argv)

    if not os.path.exists(args.policy):
        print("Policy not found: %s" % args.policy, file=sys.stderr)
        return 2
    print("policy: %s" % args.policy)
    print("mode:   %s%s" % (args.mode,
                            "  (COMPUTES ONLY, DOES NOT ACTUATE)"
                            if args.mode == "shadow"
                            else "  *** MOVES TILT AND WHEELS, ROTORS CUT ***"
                            if args.mode == "ground"
                            else "  *** FULL POLICY INCLUDING ROTORS ***"))
    print()

    if not preflight():
        return 1

    try:
        input("Press Enter to HOME THE TILT, or Ctrl-C to abort: ")
    except (KeyboardInterrupt, EOFError):
        print("\naborted")
        return 1

    if not args.skip_homing:
        if not home_tilt_to_fly():
            print("\nHoming failed. Not starting.")
            return 1
    else:
        print("skipping homing at operator request")

    print()
    print("Re-checking the gate after homing:")
    if not preflight():
        return 1

    print("=" * 68)
    print("READY. The stack will start with the gate DOWN and do nothing.")
    print("Raise the RL switch to begin; lower it to stop.")
    if args.mode in ("ground", "policy"):
        print("Raising the gate WILL MOVE THE VEHICLE.")
    if args.mode == "policy":
        print("MODE IS POLICY: THE ROTORS ARE LIVE.")
    print("=" * 68)
    try:
        input("Press Enter to start the stack, or Ctrl-C to abort: ")
    except (KeyboardInterrupt, EOFError):
        print("\naborted")
        return 1

    env = dict(os.environ)
    env["ATMO_RL_POLICY_PATH"] = args.policy
    env["ATMO_TILT_HOME"] = "fly"
    cmd = ["./atmo_session.sh", args.mode, "--policy", args.policy]
    print("\n$ ATMO_TILT_HOME=fly %s" % " ".join(cmd))
    workspace = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    return subprocess.call(cmd, cwd=workspace, env=env)


if __name__ == "__main__":
    sys.exit(main())
