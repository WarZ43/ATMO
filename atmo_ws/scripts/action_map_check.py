#!/usr/bin/env python3
"""Verify a non-rotor policy action slot against physical motion.

Runs one slot (tilt, drive or turn) at +1 or -1 through the real
LandingActionAdapter, PRINTS WHAT IT PREDICTS WILL HAPPEN, waits for you to
commit, and only then commands the motors.

The printing-first order is the point. A test that shows you the number after
the hardware moves confirms only that motion occurred, not that its direction
matched the prediction -- which is how the m4 bring-up recorded a passing gate
that was inverted. See docs/hardware_bringup.md, "The one rule".

Talks to the RoboClaws directly rather than through ROS, so exactly one
actuator group moves and nothing else can command anything. Rotor slots are
deliberately not supported: those go through PX4 and are gated behind A1.

    python3 scripts/action_map_check.py --slot drive --sign positive
    python3 scripts/action_map_check.py --slot tilt  --sign negative --seconds 3
    python3 scripts/action_map_check.py --slot turn  --sign positive --yes

Stops in a finally, on Ctrl-C, SIGTERM or any exception. If anything is still
moving afterwards: python3 scripts/estop_roboclaw.py, then the battery.
"""

import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "atmo"))

import numpy as np                                            # noqa: E402
from numpy import interp, array, deg2rad                      # noqa: E402
from atmo.mpc.roboclaw_3 import Roboclaw                      # noqa: E402
from atmo.roboclaw_safety import DEFAULT_ADDRESS, stop_all    # noqa: E402
from atmo.rl_landing_stage1_runtime import LandingActionAdapter   # noqa: E402
# The DEPLOYED config, not LandingStage1Config. They differ, and using the
# landing one made tilt=-1 look permanently dead.
from atmo.rl_combined_runtime import CombinedStage1Config     # noqa: E402

# Same reflected calibration as tilt_controller_hardware.
_CAL = array([0,3696,5551,7647,8886,10118,11062,11982,12846,13957,14885,15629,16549,17037,17749,19029,19645,20989,21453,22357,22989,23517,24013,24605,25437,25973,26325])
_RAWD = array([90,86.6,84.0,78.6,74.2,70.5,66.5,63.5,59.7,55.3,51.3,48.0,44.2,41.8,38.4,32.8,30.0,24.2,22.0,18.0,14.8,12.9,10.8,8.2,4.7,2.8,0.8])
_ENC = _CAL[-1] - _CAL[::-1]
_ANG = deg2rad(_RAWD[::-1] - _RAWD[-1])


def measured_tilt_angle():
    """Read the real arm angle, so the prediction reflects the vehicle.

    The adapter clamps tilt against tilt_lower/tilt_upper using its OWN
    internal angle, which starts at the lower bound. Seeded with 0 it reports
    "tilt = -1 does nothing" no matter where the arm actually is -- true at
    the fly stop, misleading anywhere else.
    """
    try:
        rc = Roboclaw(TILT_PORT, 115200)
        if not rc.Open():
            return None
        r = rc.ReadEncM2(DEFAULT_ADDRESS)
        try:
            rc._port.close()
        except Exception:  # noqa: BLE001
            pass
        if not r or not r[0]:
            return None
        return float(interp(-r[1], _ENC, _ANG))
    except Exception:  # noqa: BLE001
        return None

TILT_PORT = os.getenv("ATMO_TILT_ROBOCLAW",
                      "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0")
DRIVE_PORT = os.getenv("ATMO_DRIVE_ROBOCLAW",
                       "/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0")

# Measured on the vehicle 2026-08-14. See docs/session_state.md.
PHYSICAL = {
    ("drive", "ForwardM1"): "RIGHT wheel -> vehicle FORWARD",
    ("drive", "BackwardM1"): "RIGHT wheel -> vehicle BACKWARD",
    ("drive", "ForwardM2"): "LEFT wheel  -> vehicle BACKWARD",
    ("drive", "BackwardM2"): "LEFT wheel  -> vehicle FORWARD",
    ("tilt", "ForwardM2"): "TILT angle INCREASES (away from fly config)",
    ("tilt", "BackwardM2"): "TILT angle DECREASES (toward fly config)",
}
SLOT_INDEX = {"tilt": 4, "drive": 5, "turn": 6}


def plan(slot, sign, duty, seed_tilt=None):
    """Return (board, [(roboclaw_call, duty)], [prediction strings])."""
    adapter = LandingActionAdapter(CombinedStage1Config())
    if seed_tilt is not None:
        adapter.tilt_angle = float(seed_tilt)
    action = np.zeros(7, dtype=np.float32)
    action[SLOT_INDEX[slot]] = 1.0 if sign == "positive" else -1.0
    command = adapter.pre_physics_step(action)

    if slot == "tilt":
        v = float(command.tilt_velocity)
        if v == 0.0:
            return "tilt", [], ["adapter produced tilt_velocity=0 -- nothing to command"]
        call = "ForwardM2" if v > 0 else "BackwardM2"
        return "tilt", [(call, duty)], [PHYSICAL[("tilt", call)]]

    # drive board: reproduce drive_controller_hardware.update()
    lin = -int(127 * float(command.drive_speed))
    ang = -int(127 * float(command.turn_speed))
    right, left = lin + ang, lin - ang
    calls, preds = [], []
    if right:
        c = "BackwardM1" if right > 0 else "ForwardM1"
        calls.append((c, duty)); preds.append(PHYSICAL[("drive", c)])
    if left:
        c = "ForwardM2" if left > 0 else "BackwardM2"
        calls.append((c, duty)); preds.append(PHYSICAL[("drive", c)])
    return "drive", calls, preds


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slot", required=True, choices=sorted(SLOT_INDEX))
    ap.add_argument("--sign", default="positive", choices=("positive", "negative"))
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--duty", type=int, default=None,
                    help="0-127. Default 40 for drive/turn, 60 for tilt.")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = ap.parse_args(argv)

    if not 0 < args.seconds <= 15:
        ap.error("--seconds must be in 0..15")
    duty = args.duty if args.duty is not None else (60 if args.slot == "tilt" else 40)
    if not 0 < duty <= 110:
        ap.error("--duty must be in 1..110")

    seed = measured_tilt_angle()
    board, calls, preds = plan(args.slot, args.sign, duty, seed_tilt=seed)
    print("=" * 66)
    print("SLOT   %s = %+d   (policy action index %d)"
          % (args.slot, 1 if args.sign == "positive" else -1, SLOT_INDEX[args.slot]))
    if seed is None:
        print("TILT   could not read the arm angle; adapter seeded at its default")
    else:
        print("TILT   arm measured at %.1f deg (adapter seeded with this)"
              % np.degrees(seed))
    print("BOARD  %s" % board)
    print("DUTY   %d for %.1f s" % (duty, args.seconds))
    print()
    print("PREDICTION -- commit to this BEFORE you watch:")
    for p in preds:
        print("    %s" % p)
    if not calls:
        print("\nNothing to command.")
        return 0
    print()
    print("Commands: %s" % ", ".join(c for c, _ in calls))
    print("=" * 66)
    if not args.yes:
        try:
            if input("Watch the vehicle, then press Enter to run (Ctrl-C aborts): ") is None:
                return 1
        except (KeyboardInterrupt, EOFError):
            print("\naborted")
            return 1

    dev = TILT_PORT if board == "tilt" else DRIVE_PORT
    rc = Roboclaw(dev, 115200)
    if not rc.Open():
        print("Could not open %s" % dev, file=sys.stderr)
        return 2
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    ok = False
    try:
        for call, d in calls:
            getattr(rc, call)(DEFAULT_ADDRESS, d)
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            time.sleep(0.5)
            if board == "tilt":
                e = rc.ReadEncM2(DEFAULT_ADDRESS)
                print("   t=%.1f  enc=%s" % (time.time() - t0, e[1] if e[0] else "?"))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        ok = stop_all(rc, DEFAULT_ADDRESS)
        print("\nSTOP: %s" % ("acknowledged" if ok else "*** NOT ACKNOWLEDGED ***"))
        try:
            rc._port.close()
        except Exception:  # noqa: BLE001
            pass
    print()
    print("Did the vehicle do EXACTLY what was predicted above? If not, the")
    print("slot is inverted or swapped -- record it, do not adjust the")
    print("prediction to match what you saw.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
