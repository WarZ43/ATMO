#!/usr/bin/env python3
"""Stage C: prove the mocap stream correct by motion, not by label.

Prints what each axis SHOULD do before you move anything, then measures what it
actually did. That ordering is deliberate: a test that shows you the number
first invites you to rationalise it. Write the expectation down, then move the
vehicle.

Ported from m4-direct-rl `scripts/hardware_optitrack_expectations.py`, adapted
to ATMO's topics. The m4 bring-up lost hours to a mocap chain where every
network-layer test passed and the frame was still wrong; the only thing that
settles a frame convention is moving the vehicle and watching the sign.

Modes:

    --mode pose      C1. Sign of each pose component under hand motion.
    --mode twist     C2. Sign of each twist component, and noise at rest.
    --mode rate      C3/C4. Rate, jitter, dropouts over a window.
    --mode static    Noise floor at rest, against the training envelope.

    python3 scripts/hardware_optitrack_check.py --mode pose
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

DEFAULT_TOPIC = "/atmo/groundtruth_odom"

# The static noise the policy was trained to tolerate. A stream noisier than
# this at rest is feeding the observation something training never saw.
POSITION_NOISE_LIMIT_M = 0.005
VELOCITY_NOISE_LIMIT_MPS = 0.035

POSE_EXPECTATIONS = """
EXPECTED, in the policy's z-up world frame. Read this BEFORE moving anything.

    move the vehicle FORWARD  (its own +x)   -> pos.x increases
    move the vehicle LEFT     (its own +y)   -> pos.y increases
    LIFT the vehicle                          -> pos.z increases
    yaw CCW seen from above                   -> yaw increases
    roll RIGHT wing down                      -> roll increases
    pitch NOSE UP                             -> pitch increases

At rest on the ground, pos.z should be small and POSITIVE. A negative or large
z means the Motive origin or the rigid-body pivot is not where you think.
"""

TWIST_EXPECTATIONS = """
EXPECTED. Twist is published in the BODY frame.

    push the vehicle along its own nose      -> twist.linear.x  positive
    push it along its own left               -> twist.linear.y  positive
    lift it                                   -> twist.linear.z  positive
    yaw it CCW seen from above                -> twist.angular.z positive

Hold the vehicle at a fixed tilt and translate it: the body-frame linear twist
must change with the tilt even though the world motion did not. If it does not,
the twist is being published in the world frame.
"""


class Listener(Node):
    def __init__(self, topic):
        super().__init__("atmo_optitrack_check")
        self.samples = []
        self.receive_times = []
        self.create_subscription(
            Odometry, topic, self._callback, qos_profile_sensor_data
        )

    def _callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        self.samples.append(
            (
                (position.x, position.y, position.z),
                (orientation.w, orientation.x, orientation.y, orientation.z),
                (linear.x, linear.y, linear.z),
                (angular.x, angular.y, angular.z),
            )
        )
        self.receive_times.append(time.time())


def euler_from_quaternion(q):
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def collect(node, seconds):
    node.samples = []
    node.receive_times = []
    deadline = time.time() + seconds
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    return node.samples, node.receive_times


def require_samples(samples, topic):
    if len(samples) < 2:
        print("\nFAIL: fewer than two samples on %s." % topic)
        print("Is mocap_bridge running, and is the rigid body tracked in Motive?")
        return False
    return True


def mode_pose(node, args):
    print(POSE_EXPECTATIONS)
    input("Press Enter, then move the vehicle through each axis in turn... ")
    samples, _ = collect(node, args.duration)
    if not require_samples(samples, args.topic):
        return 1
    positions = np.array([s[0] for s in samples])
    angles = np.array([euler_from_quaternion(s[1]) for s in samples])
    print("\nMEASURED over %.1f s, %d samples:\n" % (args.duration, len(samples)))
    for index, name in enumerate(("pos.x", "pos.y", "pos.z")):
        column = positions[:, index]
        print(
            "    %-8s min %+7.3f  max %+7.3f  range %6.3f  net %+7.3f"
            % (name, column.min(), column.max(), np.ptp(column), column[-1] - column[0])
        )
    for index, name in enumerate(("roll", "pitch", "yaw")):
        column = np.degrees(np.unwrap(angles[:, index]))
        print(
            "    %-8s min %+7.1f  max %+7.1f  range %6.1f  net %+7.1f   [deg]"
            % (name, column.min(), column.max(), np.ptp(column), column[-1] - column[0])
        )
    resting_z = float(positions[0, 2])
    print("\n    first-sample z = %+.3f m" % resting_z)
    if resting_z < 0.0:
        print("    WARNING: negative z at start. Check the Motive ground plane.")
    print("\nC1 passes only if every sign matched the expectation above.")
    return 0


def mode_twist(node, args):
    print(TWIST_EXPECTATIONS)
    input("Press Enter, then translate and rotate the vehicle by hand... ")
    samples, _ = collect(node, args.duration)
    if not require_samples(samples, args.topic):
        return 1
    linear = np.array([s[2] for s in samples])
    angular = np.array([s[3] for s in samples])
    print("\nMEASURED over %.1f s, %d samples:\n" % (args.duration, len(samples)))
    for index, name in enumerate(("lin.x", "lin.y", "lin.z")):
        column = linear[:, index]
        print(
            "    %-8s min %+7.3f  max %+7.3f  peak |v| %6.3f   [m/s]"
            % (name, column.min(), column.max(), np.abs(column).max())
        )
    for index, name in enumerate(("ang.x", "ang.y", "ang.z")):
        column = angular[:, index]
        print(
            "    %-8s min %+7.3f  max %+7.3f  peak |w| %6.3f   [rad/s]"
            % (name, column.min(), column.max(), np.abs(column).max())
        )
    print("\nC2 passes only if every sign matched the expectation above.")
    return 0


def mode_rate(node, args):
    print("Measuring stream rate over %.1f s. Leave the vehicle alone." % args.duration)
    samples, times = collect(node, args.duration)
    if not require_samples(samples, args.topic):
        return 1
    deltas = np.diff(np.array(times))
    rate = 1.0 / float(np.mean(deltas))
    p95 = float(np.percentile(deltas, 95)) * 1000.0
    worst = float(deltas.max()) * 1000.0
    dropouts = int(np.sum(deltas > args.stale_ms / 1000.0))
    print("\n    samples      %d" % len(samples))
    print("    mean rate    %.1f Hz (expected %.0f)" % (rate, args.expected_hz))
    print("    p95 gap      %.1f ms" % p95)
    print("    worst gap    %.1f ms" % worst)
    print("    dropouts     %d over %.0f ms" % (dropouts, args.stale_ms))
    ok = True
    if rate < 0.8 * args.expected_hz:
        print("\n    FAIL: rate is below 80%% of expected.")
        ok = False
    # Stage C3's budget is two policy steps, 40 ms at 50 Hz, INCLUDING network
    # transport. A contended 2.4 GHz link will not hold that.
    if worst > 40.0:
        print("\n    FAIL: worst gap exceeds the 40 ms two-step budget.")
        ok = False
    if dropouts:
        print("\n    FAIL: %d dropouts." % dropouts)
        ok = False
    print("\n%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def mode_static(node, args):
    print("Measuring the noise floor over %.1f s. Do NOT touch the vehicle." % args.duration)
    samples, _ = collect(node, args.duration)
    if not require_samples(samples, args.topic):
        return 1
    positions = np.array([s[0] for s in samples])
    linear = np.array([s[2] for s in samples])
    position_noise = float(np.max(np.std(positions, axis=0)))
    velocity_noise = float(np.max(np.std(linear, axis=0)))
    print("\n    position sigma  %.4f m    (limit %.4f)" % (position_noise, POSITION_NOISE_LIMIT_M))
    print("    velocity sigma  %.4f m/s  (limit %.4f)" % (velocity_noise, VELOCITY_NOISE_LIMIT_MPS))
    ok = (
        position_noise <= POSITION_NOISE_LIMIT_M
        and velocity_noise <= VELOCITY_NOISE_LIMIT_MPS
    )
    if not ok:
        print(
            "\n    FAIL: the stream is noisier at rest than the training envelope.\n"
            "    Check marker occlusion, Motive smoothing, and the filter_hz\n"
            "    parameter on mocap_bridge."
        )
    print("\n%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


MODES = {
    "pose": mode_pose,
    "twist": mode_twist,
    "rate": mode_rate,
    "static": mode_static,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=sorted(MODES), required=True)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--expected-hz", type=float, default=120.0)
    parser.add_argument("--stale-ms", type=float, default=40.0)
    args = parser.parse_args()

    rclpy.init()
    node = Listener(args.topic)
    try:
        return MODES[args.mode](node, args)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
