#!/usr/bin/env python3
"""Measure every frame conversion in the deployed state path, from the logs.

Question this answers: the mocap rig was mounted 180 deg yawed relative to the
flight controller and the rotor numbering, and the runtime applies PX4->training
frame conversions to data that is ALREADY in the training convention. Which of
those actually reached the policy, and with what sign on each axis?

Nothing here is modelled. It regresses the bridge's published attitude against
the FC's own attitude over a whole run, then pushes the bridge attitude through
the deployed conversion functions and regresses that too.

    source /opt/ros/humble/setup.bash && source ~/ATMO/atmo_ws/install/setup.bash
    python analysis/frame_chain.py
"""

from __future__ import annotations

import math
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/opt/ros/humble/local/lib/python3.10/dist-packages")
from rclpy.serialization import deserialize_message  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from px4_msgs.msg import VehicleOdometry  # noqa: E402

BASE = Path(__file__).resolve().parent.parent

# The deployed conversions, copied verbatim from
# atmo_ws/src/atmo/atmo/rl_landing_stage1_runtime.py:39-40.
NED_TO_ENU = np.array(((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))
FRD_TO_FLU = np.diag((1.0, -1.0, -1.0))

RUNS = {
    "413 yaw runaway": "atmo_policy_20260818_200125",
    "414 quiescent": "atmo_policy_20260818_200413",
    "415 roll, fly": "atmo_policy_20260818_202503",
    "417 roll at 15deg": "atmo_policy_20260818_205315",
    "195615 pre-run": "atmo_policy_20260818_195615",
}


def quat_to_rotmat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array((
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    ))


def rpy(R):
    return np.array((
        math.atan2(R[2, 1], R[2, 2]),
        math.asin(max(-1.0, min(1.0, -R[2, 0]))),
        math.atan2(R[1, 0], R[0, 0]),
    ))


def read(bagdir: Path, topic: str, msgtype):
    dbs = sorted(bagdir.glob("*.db3"))
    if not dbs:
        return [], []
    con = sqlite3.connect(str(dbs[0]))
    try:
        con.execute("pragma integrity_check").fetchone()
        tid = con.execute("select id from topics where name=?", (topic,)).fetchone()
        if tid is None:
            return [], []
        rows = con.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp", (tid[0],)
        ).fetchall()
    except sqlite3.DatabaseError:
        return [], []
    finally:
        con.close()
    t = np.array([r[0] for r in rows], dtype=np.float64) * 1e-9
    msgs = [deserialize_message(bytes(r[1]), msgtype) for r in rows]
    return t, msgs


def slope(a, b):
    """Least-squares slope of b on a, through the origin, plus correlation."""
    a = np.asarray(a)
    b = np.asarray(b)
    keep = np.isfinite(a) & np.isfinite(b)
    a, b = a[keep], b[keep]
    if a.size < 20 or np.dot(a, a) < 1e-9:
        return float("nan"), float("nan"), a.size
    s = float(np.dot(a, b) / np.dot(a, a))
    c = float(np.corrcoef(a, b)[0, 1]) if a.std() > 1e-9 and b.std() > 1e-9 else float("nan")
    return s, c, a.size


def analyse(label: str, bagname: str) -> None:
    bagdir = BASE / "bags" / bagname
    tb, bridge = read(bagdir, "/atmo/groundtruth_odom", Odometry)
    tf, fc = read(bagdir, "/fmu/out/vehicle_odometry", VehicleOdometry)
    if len(bridge) < 50 or len(fc) < 50:
        print("  %-20s no usable pair (bridge %d, fc %d)" % (label, len(bridge), len(fc)))
        return

    # FC attitude is NED->FRD; convert to the z-up/FLU convention the bridge
    # publishes in, so "the same physical attitude" means the same numbers.
    fc_rpy = []
    for m in fc:
        q = np.array(m.q, dtype=np.float64)          # w, x, y, z
        R_ned_frd = quat_to_rotmat(q)
        fc_rpy.append(rpy(NED_TO_ENU @ R_ned_frd @ FRD_TO_FLU))
    fc_rpy = np.array(fc_rpy)

    bridge_rpy = []
    obs_rpy = []
    for m in bridge:
        o = m.pose.pose.orientation
        q = np.array((o.w, o.x, o.y, o.z), dtype=np.float64)
        R = quat_to_rotmat(q)
        bridge_rpy.append(rpy(R))
        # What update_px4_state() does to it: the SAME pair of conversions,
        # applied a second time to data that is already z-up/FLU.
        obs_rpy.append(rpy(NED_TO_ENU @ R @ FRD_TO_FLU))
    bridge_rpy = np.array(bridge_rpy)
    obs_rpy = np.array(obs_rpy)

    # Resample the FC onto the bridge stamps.
    fc_on_b = np.stack([np.interp(tb, tf, fc_rpy[:, k]) for k in range(3)], 1)

    # Only use samples where there is real attitude to regress against.
    live = (np.abs(fc_on_b[:, 0]) > 0.02) | (np.abs(fc_on_b[:, 1]) > 0.02)
    print("\n  %s   (%d bridge samples, %d with attitude > 1.1 deg)"
          % (label, len(tb), int(live.sum())))
    for name, target in (("bridge (what mocap published)", bridge_rpy),
                         ("obs    (after update_px4_state)", obs_rpy)):
        out = []
        for k, axis in enumerate(("roll", "pitch")):
            s, c, n = slope(fc_on_b[live, k], target[live, k])
            out.append("%s %+.2f (r %+.2f)" % (axis, s, c))
        # Yaw is an offset, not a slope: report the circular mean difference.
        dy = np.unwrap(target[live, 2] - fc_on_b[live, 2])
        out.append("yaw offset %+.1f deg" % math.degrees(np.median(dy)))
        print("    %-32s %s" % (name, "  ".join(out)))


def main() -> int:
    print("Frame chain, measured from the bags")
    print("  FC attitude is converted NED/FRD -> ENU/FLU once, honestly, so it")
    print("  can be compared with the bridge's z-up publication.")
    print("  A slope of +1 means the two agree; -1 means that axis is inverted.")
    for label, bagname in RUNS.items():
        analyse(label, bagname)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
