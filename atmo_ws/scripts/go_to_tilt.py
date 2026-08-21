#!/usr/bin/env python3
"""Operator-run tilt goto: move the arm to a target angle, FLY-homed frame.

ONLY VALID AFTER scripts/home_tilt.py has succeeded this power cycle:
the angle lookup assumes the encoder was zeroed at the FLY hard stop.

Run this yourself with a hand on the kill lever. Ctrl-C stops the motor.
Stall rule enforced: no encoder motion within 0.7 s -> cut to zero, stop,
let the motor cool. Never retry hot.

Usage:
    python3 scripts/go_to_tilt.py --deg 85     # go to ground/drive config
    python3 scripts/go_to_tilt.py --deg 0      # back toward fly (uphill)
"""

import argparse
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "atmo"))

from atmo.mpc.roboclaw_3 import Roboclaw                     # noqa: E402
from atmo.roboclaw_safety import DEFAULT_ADDRESS             # noqa: E402

TILT_PORT = os.environ.get(
    "ATMO_TILT_ROBOCLAW",
    "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0")

MAX_DEG = 85.0            # software bound; hardware kill switch sits at ~90
STALL_WINDOW_S = 1.0   # operator-raised from 0.7 (2026-08-18): guarantee a
                       # full second at duty before the stall cutoff fires
STALL_MIN_COUNTS = 20
POLL_S = 0.05
TIMEOUT_S = 45.0

# Calibration, identical math to tilt_controller_hardware.py, FLY-homed
# branch: table span-corrected to the measured mechanism span, reflected so
# 0 counts = 0 deg at fly, counts NEGATIVE toward drive (negate for lookup).
_CAL_ENC = np.array([0, 3696, 5551, 7647, 8886, 10118, 11062, 11982, 12846,
                     13957, 14885, 15629, 16549, 17037, 17749, 19029, 19645,
                     20989, 21453, 22357, 22989, 23517, 24013, 24605, 25437,
                     25973, 26325], dtype=float)
_CAL_ENC *= float(os.getenv("ATMO_TILT_SPAN_COUNTS", "22200")) / _CAL_ENC[-1]
_CAL_DEG = np.array([90, 86.6, 84.0, 78.6, 74.2, 70.5, 66.5, 63.5, 59.7,
                     55.3, 51.3, 48.0, 44.2, 41.8, 38.4, 32.8, 30.0, 24.2,
                     22.0, 18.0, 14.8, 12.9, 10.8, 8.2, 4.7, 2.8, 0.8])
ENC_FLY = _CAL_ENC[-1] - _CAL_ENC[::-1]
DEG_FLY = _CAL_DEG[::-1] - _CAL_DEG[-1]


def read_enc(rc, address):
    result = rc.ReadEncM2(address)
    if not result or not result[0]:
        return None
    raw = result[1]
    return raw - (1 << 32) if raw > (1 << 31) else raw


def angle_deg(enc):
    return float(np.interp(-enc, ENC_FLY, DEG_FLY))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deg", type=float, required=True,
                        help="target tilt angle, 0 (fly) .. %.0f (ground)" % MAX_DEG)
    parser.add_argument("--duty-down", type=int, default=126,
                        help="duty toward ground (downhill, default 126)")
    parser.add_argument("--duty-up", type=int, default=126,
                        help="duty toward fly (uphill; full known-good from "
                             "cold, default 126)")
    parser.add_argument("--address", type=lambda s: int(s, 0),
                        default=DEFAULT_ADDRESS)
    args = parser.parse_args(argv)

    if not 0.0 <= args.deg <= MAX_DEG:
        parser.error("--deg must be in 0..%.0f" % MAX_DEG)
    if not os.path.exists(TILT_PORT):
        print("No tilt board at %s" % TILT_PORT, file=sys.stderr)
        return 2

    rc = Roboclaw(TILT_PORT, 115200)
    if not rc.Open():
        print("Could not open %s" % TILT_PORT, file=sys.stderr)
        return 2

    enc = read_enc(rc, args.address)
    if enc is None:
        print("Cannot read M2 encoder; refusing to move.", file=sys.stderr)
        return 2
    here = angle_deg(enc)
    print("tilt board %s" % os.path.realpath(TILT_PORT))
    print("current %.1f deg (enc %d) -> target %.1f deg" % (here, enc, args.deg))
    if abs(here - args.deg) < 1.0:
        print("Already there.")
        return 0

    going_down = args.deg > here
    duty = args.duty_down if going_down else args.duty_up
    move = rc.ForwardM2 if going_down else rc.BackwardM2
    print("%s duty=%d (%s). HAND ON THE KILL. Ctrl-C stops the motor."
          % ("ForwardM2" if going_down else "BackwardM2", duty,
             "downhill to ground" if going_down else "uphill to fly"))

    signal.signal(signal.SIGTERM,
                  lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    reached = False
    try:
        move(args.address, duty)
        t0 = time.monotonic()
        last = enc
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
            here = angle_deg(enc)
            done = here >= args.deg if going_down else here <= args.deg
            if done:
                reached = True
                break
            if now - last_move_t > STALL_WINDOW_S:
                print("STALL: no motion within %.1f s at %.1f deg. Stopping. "
                      "Let the motor COOL before any retry." %
                      (STALL_WINDOW_S, here))
                break
            if now - t0 > TIMEOUT_S:
                print("Timeout after %.0f s at %.1f deg; stopping."
                      % (TIMEOUT_S, here), file=sys.stderr)
                break
    except KeyboardInterrupt:
        print("\nOperator stop.")
    finally:
        try:
            rc.ForwardM2(args.address, 0)
            rc.BackwardM2(args.address, 0)
        except Exception:  # noqa: BLE001
            pass

    enc = read_enc(rc, args.address)
    if enc is not None:
        print("Stopped at %.1f deg (enc %d)." % (angle_deg(enc), enc))
    return 0 if reached else 1


if __name__ == "__main__":
    sys.exit(main())
