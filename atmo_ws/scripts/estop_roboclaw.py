#!/usr/bin/env python3
"""Emergency stop for the RoboClaws. Needs no ROS, no nodes, no network.

A RoboClaw latches its last command indefinitely -- there is no command
timeout configured on this vehicle's boards. Once told to move they move until
told otherwise or until power is removed. Closing the port does not stop them.
Killing the node does not stop them. Dropping the RC gate does not stop them,
because the gate only prevents a LIVE node from sending commands.

Measured 2026-08-14: both boards kept driving after every node had exited and
both serial ports were free. What stopped them was exactly what this script
does -- opening the port and writing a zero.

Run it when something is moving and nothing is controlling it:

    python3 scripts/estop_roboclaw.py

If this fails or the motion continues, DISCONNECT THE BATTERY. That is the
only stop that does not depend on the board still listening.

`reset_roboclaw.sh` is NOT this. It resets the USB device, which does nothing
to a latched motor.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "atmo"))

from atmo.mpc.roboclaw_3 import Roboclaw          # noqa: E402
from atmo.roboclaw_safety import DEFAULT_ADDRESS, stop_all  # noqa: E402

BY_PATH = "/dev/serial/by-path"
# Every socket that has ever held a RoboClaw on this vehicle. Stopping a board
# matters more than knowing which one it is, so this does not care which is
# tilt and which is drive -- it stops both.
DEFAULT_PORTS = (
    "%s/platform-3610000.usb-usb-0:2.1:1.0" % BY_PATH,
    "%s/platform-3610000.usb-usb-0:2.2:1.0" % BY_PATH,
)


def main(argv):
    ports = argv[1:] or [
        p for p in (
            os.getenv("ATMO_TILT_ROBOCLAW"),
            os.getenv("ATMO_DRIVE_ROBOCLAW"),
        ) if p
    ] or list(DEFAULT_PORTS)

    stopped, failed = [], []
    for dev in ports:
        if not os.path.exists(dev):
            print("SKIP  %s (not present)" % dev)
            continue
        try:
            rc = Roboclaw(dev, 115200)
            if not rc.Open():
                print("FAIL  %s (could not open)" % dev)
                failed.append(dev)
                continue
            if stop_all(rc, DEFAULT_ADDRESS):
                print("STOP  %s" % dev)
                stopped.append(dev)
            else:
                print("FAIL  %s (no stop command acknowledged)" % dev)
                failed.append(dev)
            try:
                rc._port.close()
            except Exception:  # noqa: BLE001
                pass
        except Exception as exc:  # noqa: BLE001
            print("FAIL  %s (%s)" % (dev, exc))
            failed.append(dev)

    print()
    if failed or not stopped:
        print("NOT EVERYTHING WAS STOPPED. DISCONNECT THE BATTERY.")
        return 1
    print("Zero written to %d board(s). If anything is still moving, the board"
          % len(stopped))
    print("is not listening -- disconnect the battery.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
