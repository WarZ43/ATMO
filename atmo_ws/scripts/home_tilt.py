#!/usr/bin/env python3
"""Operator-run tilt homing: BackwardM2 to the FLY hard stop, then zero.

Run this yourself with a hand on the kill lever. Ctrl-C stops the motor.

Rules enforced here (session_state.md 2026-08-18):
  - Full known-good duty from COLD on the first attempt. Never ladder up.
  - If the encoder has not moved within 0.7 s of the command, cut to zero
    and stop: the motor is stalled. Let it cool; do not retry hot.
  - Hard stop = the encoder was moving and its speed collapses. Zero there.

Usage:
    python3 scripts/home_tilt.py            # duty 126, zero on success
    python3 scripts/home_tilt.py --no-zero  # detect the stop, do not zero
"""

import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "atmo"))

from atmo.mpc.roboclaw_3 import Roboclaw                     # noqa: E402
from atmo.roboclaw_safety import DEFAULT_ADDRESS             # noqa: E402

TILT_PORT = os.environ.get(
    "ATMO_TILT_ROBOCLAW",
    "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0")

STALL_WINDOW_S = 0.7      # no motion within this after command = stall, stop
STALL_MIN_COUNTS = 20     # less than this is "not moving"
STOP_WINDOW_S = 0.5       # after motion: speed collapse window = hard stop
POLL_S = 0.1
TIMEOUT_S = 20.0


def read_enc(rc, address):
    result = rc.ReadEncM2(address)
    if not result or not result[0]:
        return None
    raw = result[1]
    return raw - (1 << 32) if raw > (1 << 31) else raw


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duty", type=int, default=126,
                        help="full known-good duty from cold (default 126)")
    parser.add_argument("--no-zero", action="store_true",
                        help="detect the hard stop but do not SetEncM2(0)")
    parser.add_argument("--address", type=lambda s: int(s, 0),
                        default=DEFAULT_ADDRESS)
    args = parser.parse_args(argv)

    if not os.path.exists(TILT_PORT):
        print("No tilt board at %s" % TILT_PORT, file=sys.stderr)
        return 2

    rc = Roboclaw(TILT_PORT, 115200)
    if not rc.Open():
        print("Could not open %s" % TILT_PORT, file=sys.stderr)
        return 2

    start = read_enc(rc, args.address)
    if start is None:
        print("Cannot read M2 encoder; refusing to move.", file=sys.stderr)
        return 2

    print("tilt board %s" % os.path.realpath(TILT_PORT))
    print("BackwardM2 duty=%d toward FLY. Encoder start=%d." % (args.duty, start))
    print("HAND ON THE KILL. Ctrl-C stops the motor.")

    signal.signal(signal.SIGTERM,
                  lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    homed = False
    try:
        rc.BackwardM2(args.address, args.duty)
        t0 = time.monotonic()
        last = start
        last_move_t = t0
        moved_total = 0
        while True:
            time.sleep(POLL_S)
            now = time.monotonic()
            enc = read_enc(rc, args.address)
            if enc is None:
                print("Encoder read failed; stopping.", file=sys.stderr)
                break
            delta = abs(enc - last)
            if delta >= STALL_MIN_COUNTS:
                last_move_t = now
                moved_total += delta
            last = enc
            if moved_total < STALL_MIN_COUNTS and now - t0 > STALL_WINDOW_S:
                print("STALL: no motion within %.1f s. Stopping. Let the "
                      "motor COOL before any retry." % STALL_WINDOW_S)
                break
            if moved_total >= STALL_MIN_COUNTS and now - last_move_t > STOP_WINDOW_S:
                print("Hard stop: moved %d counts, speed collapsed. enc=%d"
                      % (moved_total, enc))
                homed = True
                break
            if now - t0 > TIMEOUT_S:
                print("Timeout after %.0f s without a hard stop; stopping."
                      % TIMEOUT_S, file=sys.stderr)
                break
    except KeyboardInterrupt:
        print("\nOperator stop.")
    finally:
        try:
            rc.BackwardM2(args.address, 0)
            rc.ForwardM2(args.address, 0)
        except Exception:  # noqa: BLE001
            pass

    if homed and not args.no_zero:
        if rc.SetEncM2(args.address, 0):
            print("SetEncM2(0): FLY = 0 counts = 0 deg. HOMED.")
        else:
            print("SetEncM2(0) FAILED; frame is NOT homed.", file=sys.stderr)
            return 1
    elif homed:
        print("Hard stop found; --no-zero so the encoder was left alone.")
    else:
        print("NOT homed.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
