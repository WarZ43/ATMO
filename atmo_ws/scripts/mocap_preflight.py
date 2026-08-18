#!/usr/bin/env python3
"""Is the mocap actually reaching the EKF? Read-only.

With ATMO_RL_POSE_SOURCE=mocap the chain is short, which is the point:

    OptiTrack -> /vrpn_mocap/<body>/pose -> the observation

PX4's EKF is NOT in it. That removes an estimator with its own convergence
time and frame conventions, and it removes a specific failure seen here on
2026-08-14: GPS denied and unfused, PX4 published vehicle_odometry at 92 Hz
reporting (-4428, -699, -72) m and 24 m/s while the vehicle sat still. Finite,
so nothing rejected it, and a rate check called it healthy.

A short chain still has to be checked, just differently. What matters now is
the mocap stream itself: is it arriving, fast enough, without gaps, and is the
vehicle where you think it is? Velocity is DIFFERENTIATED from this stream, so
dropouts and jitter become velocity spikes the policy will act on.

The EKF comparison below is reported for information only. It is expected to
disagree, and disagreement is not a failure.

    python3 scripts/mocap_preflight.py
    python3 scripts/mocap_preflight.py --body M4 --seconds 10
"""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped

# px4_msgs and the atmo package are only built on the VEHICLE. This script's
# job is the mocap stream, which lives on the LAPTOP, and PX4 appears here in
# one informational line. Requiring them would mean this could only run on the
# machine that cannot see the thing it checks.
try:
    from px4_msgs.msg import VehicleOdometry
    from atmo import px4_topics
    _ODOM_TOPIC = px4_topics.topic("vehicle_odometry")
except ImportError:
    VehicleOdometry = None
    _ODOM_TOPIC = None


# A vehicle in a mocap volume is within a few metres of its origin.
PLAUSIBLE_RADIUS_M = 50.0
# Velocity is differentiated from this stream, so the gaps matter more than
# the average rate: one 200 ms hole becomes a velocity spike the policy acts on.
MIN_RATE_HZ = float(os.getenv("ATMO_MOCAP_MIN_HZ", "50"))
MAX_GAP_S = float(os.getenv("ATMO_MOCAP_MAX_GAP_S", "0.10"))


class Preflight(Node):
    def __init__(self, topic):
        super().__init__("mocap_preflight")
        self.mocap = []
        self.stamps = []
        self.odom = []
        self.create_subscription(PoseStamped, topic, self._mocap, qos_profile_sensor_data)
        if VehicleOdometry is not None:
            self.create_subscription(
                VehicleOdometry, _ODOM_TOPIC, self._odom, qos_profile_sensor_data)

    def _mocap(self, msg):
        p = msg.pose.position
        self.mocap.append((p.x, p.y, p.z))
        self.stamps.append(time.monotonic())

    def _odom(self, msg):
        self.odom.append(tuple(float(v) for v in msg.position))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--body", default=os.getenv("ATMO_MOCAP_BODY", "M4"))
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args(argv)
    topic = "/vrpn_mocap/%s/pose" % args.body

    rclpy.init()
    node = Preflight(topic)
    print("watching %s for %.0fs. Keep the vehicle STILL." % (topic, args.seconds))
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    problems = []
    print()
    n = len(node.mocap)
    print("  mocap samples          %d" % n)
    if n < 2:
        problems.append("No mocap on %s. Is Motive streaming and the bridge up?" % topic)
    else:
        gaps = [b - a for a, b in zip(node.stamps, node.stamps[1:])]
        rate = (n - 1) / (node.stamps[-1] - node.stamps[0])
        worst = max(gaps)
        print("  rate                   %.1f Hz" % rate)
        print("  worst gap              %.0f ms" % (worst * 1000.0))
        if rate < MIN_RATE_HZ:
            problems.append("Mocap rate %.1f Hz is below %.0f Hz. Velocity is "
                            "differentiated from this stream." % (rate, MIN_RATE_HZ))
        if worst > MAX_GAP_S:
            problems.append("A %.0f ms gap in the pose stream. Velocity is "
                            "differentiated, so a dropout becomes a velocity "
                            "spike the policy will act on." % (worst * 1000.0))

        m = node.mocap[-1]
        radius = math.sqrt(sum(v * v for v in m))
        print("  mocap position         (%.2f, %.2f, %.2f) m   |r| = %.2f m" % (m + (radius,)))
        if not all(math.isfinite(v) for v in m):
            problems.append("Mocap position is not finite.")
        elif radius > PLAUSIBLE_RADIUS_M:
            problems.append("Mocap reports %.0f m from origin. Wrong rigid body, "
                            "or the volume is not calibrated." % radius)

        # Stationary vehicle: the spread IS the noise floor, and it propagates
        # into the differentiated velocity.
        spread = max(math.sqrt(sum((a - b) ** 2 for a, b in zip(s, m)))
                     for s in node.mocap[-min(n, 200):])
        print("  jitter (still)         %.1f mm" % (spread * 1000.0))
        if spread > 0.01:
            problems.append("%.0f mm of jitter while stationary. Differentiated, "
                            "that is %.2f m/s of noise." % (spread * 1000.0, spread / 0.008))

    if node.odom:
        o = node.odom[-1]
        print("  PX4 EKF (INFO ONLY)    (%.1f, %.1f, %.1f) m -- not used as the "
              "pose source; disagreement here is expected" % o)
    elif VehicleOdometry is None:
        print("  PX4 EKF                not checked (px4_msgs not built here, "
              "which is normal on the laptop)")

    print()
    if problems:
        print("NOT READY for a mocap run:")
        for p in problems:
            print("  - %s" % p)
    else:
        print("READY: mocap stream is live, fast, gap-free and plausible.")
    rclpy.shutdown()
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
