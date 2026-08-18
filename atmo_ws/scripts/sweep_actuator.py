#!/usr/bin/env python3
"""Move ONE RoboClaw motor, slowly, briefly, with a guaranteed stop.

This exists because the two boards are physically indistinguishable at rest --
both report `USB Roboclaw 2x7A v4.4.9`, both encoders read 0, both draw no
current -- so which one is tilt and which is drive can only be established by
moving one motor at a time and watching.

It does NOT go through ROS. That is deliberate: the action tests bring up both
hardware nodes at once, and this needs exactly one motor moving with nothing
else able to command anything.

Safety properties, in order of how much they matter:

  * The motor is stopped in a `finally`, so it stops on normal exit, on Ctrl-C,
    on SIGTERM, and on any exception.
  * The stop is `roboclaw_safety.stop_all`, which zeroes every motor on the
    board by every route, not just the one that was commanded.
  * Duration is bounded and short. A RoboClaw latches its last command forever
    -- there is no command timeout on these boards -- so an unbounded command
    is a motor nobody can stop except by pulling the battery.
  * It refuses to run if anything else holds the port.

If something is moving and this did not stop it:
    python3 scripts/estop_roboclaw.py
and if that does not work, DISCONNECT THE BATTERY.

Examples
--------
    # slowest useful sweep, 5 s, M1 on the board in USB socket 2.1
    python3 scripts/sweep_actuator.py --port 2.1 --motor M1 --seconds 5 --duty 10

    # the other direction
    python3 scripts/sweep_actuator.py --port 2.1 --motor M1 --direction backward
"""

import argparse
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "atmo"))

from atmo.mpc.roboclaw_3 import Roboclaw                     # noqa: E402
from atmo.roboclaw_safety import DEFAULT_ADDRESS, stop_all   # noqa: E402

BY_PATH = "/dev/serial/by-path/platform-3610000.usb-usb-0:%s:1.0"

# 127 is full scale. The drive node's own runaway was speed 3 and it visibly
# turned the wheels, so single digits are real motion, not a nudge.
DEFAULT_DUTY = 10
MAX_DUTY = 40


def resolve_port(value):
    return BY_PATH % value if value in ("2.1", "2.2") else value


def port_is_busy(dev):
    """True only if another process actually holds the port.

    fuser prints the device name with a trailing colon to STDERR on every
    call, whether or not anything holds the file, and prints PIDs to STDOUT.
    Testing stderr therefore reports every port as busy -- which made this
    guard refuse every sweep and silently never command a motor.
    """
    real = os.path.realpath(dev)
    try:
        out = subprocess.run(["fuser", real], capture_output=True, text=True)
    except FileNotFoundError:
        return False
    return out.returncode == 0 and bool(out.stdout.split())


def read_encoders(rc, address):
    out = {}
    for name in ("ReadEncM1", "ReadEncM2"):
        try:
            result = getattr(rc, name)(address)
            out[name] = result[1] if result and result[0] else None
        except Exception:  # noqa: BLE001
            out[name] = None
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True,
                        help="'2.1', '2.2', or a full device path")
    parser.add_argument("--motor", required=True, choices=("M1", "M2"))
    parser.add_argument("--direction", default="forward",
                        choices=("forward", "backward"))
    parser.add_argument("--duty", type=int, default=DEFAULT_DUTY,
                        help="0-127 (default %d). Keep it small." % DEFAULT_DUTY)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=DEFAULT_ADDRESS)
    args = parser.parse_args(argv)

    if not 0 < args.duty <= MAX_DUTY:
        parser.error("--duty must be in 1..%d for a bring-up sweep" % MAX_DUTY)
    if not 0 < args.seconds <= 15:
        parser.error("--seconds must be in 0..15")

    dev = resolve_port(args.port)
    if not os.path.exists(dev):
        print("No such device: %s" % dev, file=sys.stderr)
        return 2
    if port_is_busy(dev):
        print("%s is held by another process. Stop the ROS nodes first."
              % dev, file=sys.stderr)
        return 2

    rc = Roboclaw(dev, 115200)
    if not rc.Open():
        print("Could not open %s" % dev, file=sys.stderr)
        return 2

    command = "%sM%s" % (args.direction.capitalize(), args.motor[1])
    method = getattr(rc, command, None)
    if method is None:
        print("Board does not support %s" % command, file=sys.stderr)
        return 2

    before = read_encoders(rc, args.address)
    print("port      %s (%s)" % (args.port, os.path.realpath(dev)))
    print("command   %s(duty=%d) for %.1f s" % (command, args.duty, args.seconds))
    print("encoders  before: M1=%s M2=%s" % (before["ReadEncM1"], before["ReadEncM2"]))
    print()
    print("WATCH THE VEHICLE. Ctrl-C stops it immediately.")
    print()

    # SIGTERM must land in the same finally as everything else.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    moved = None
    try:
        method(args.address, args.duty)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            time.sleep(0.25)
            now = read_encoders(rc, args.address)
            print("  t=%4.1fs  M1=%-10s M2=%-10s" % (
                args.seconds - (deadline - time.monotonic()),
                now["ReadEncM1"], now["ReadEncM2"]))
            moved = now
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        ok = stop_all(rc, args.address)
        print()
        print("STOP sent: %s" % ("acknowledged" if ok else "*** NOT ACKNOWLEDGED ***"))
        after = read_encoders(rc, args.address)
        try:
            rc._port.close()
        except Exception:  # noqa: BLE001
            pass

    print("encoders  after:  M1=%s M2=%s" % (after["ReadEncM1"], after["ReadEncM2"]))
    for enc in ("ReadEncM1", "ReadEncM2"):
        a, b = before[enc], after[enc]
        if a is not None and b is not None and a != b:
            print("  %s CHANGED %s -> %s  (this motor has an encoder: TILT board)"
                  % (enc, a, b))
    if moved and all(before[e] == after[e] for e in before):
        print("  no encoder change: either a drive wheel (no encoder) or nothing moved")
    if not ok:
        print()
        print("STOP WAS NOT ACKNOWLEDGED. Run scripts/estop_roboclaw.py, then")
        print("DISCONNECT THE BATTERY if anything is still moving.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
