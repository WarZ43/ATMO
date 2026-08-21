#!/usr/bin/env python3
"""Claim-by-claim proof of the 8/18 frame chain, from the logs only.

Every number below is produced by feeding the DEPLOYED observation builder
(`atmo.rl_combined_runtime.CombinedObservationBuilder`, imported, not
reimplemented) with the actual logged `/atmo/groundtruth_odom` messages, then
reading back what the builder holds -- i.e. what the policy saw -- and
comparing it against the flight controller's own attitude from
`/fmu/out/vehicle_odometry` in the same bag.

The central tool is a two-sided Procrustes fit. Any pair of attitude streams of
the same physical motion is related by

    R_obs(t) = Rw . R_fc(t) . Rb

with Rw a fixed WORLD rotation and Rb a fixed BODY rotation. Solving for both
turns a pile of per-axis slopes into two rotations with an axis, an angle and a
residual -- which is what a frame claim actually is. A per-axis slope of -1 is
consistent with several different rotations; an axis-angle is not.

    source /opt/ros/humble/setup.bash && source ~/ATMO/atmo_ws/install/setup.bash
    python analysis/frame_proof.py            # all runs
    python analysis/frame_proof.py 417        # one run
"""

from __future__ import annotations

import glob
import math
import os
import sqlite3
import sys

import numpy as np

BASE = "/home/warz42/ATMO/flight_data_20260818"
ATMO_SRC = "/home/warz42/ATMO/atmo_ws/src/atmo"
sys.path.insert(0, ATMO_SRC)
sys.path.insert(0, "/opt/ros/humble/local/lib/python3.10/dist-packages")
os.environ.setdefault("ATMO_RL_ROUTE", "takeoff")
os.environ.setdefault("ATMO_RL_MODE", "policy")

RUNS = {
    413: "atmo_policy_20260818_200125",
    414: "atmo_policy_20260818_200413",
    415: "atmo_policy_20260818_202503",
    417: "atmo_policy_20260818_205315",
}


def quat_to_rotmat(q):
    w, x, y, z = np.asarray(q, dtype=np.float64)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array((
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    ))


def axis_angle(R):
    """Rotation angle in degrees and unit axis, from a rotation matrix."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = math.degrees(math.acos(c))
    if abs(ang) < 1e-6 or abs(ang - 180.0) < 1e-6:
        # near 0 or pi the skew part vanishes; take the axis from R + I
        w, v = np.linalg.eigh((R + R.T) / 2.0 + np.eye(3))
        axis = v[:, int(np.argmax(w))]
    else:
        axis = np.array((R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]))
        axis = axis / np.linalg.norm(axis)
    return ang, axis


def name_axis(axis):
    labels = []
    for k, nm in enumerate("xyz"):
        if abs(axis[k]) > 0.9:
            return nm
        if abs(axis[k]) > 0.3:
            labels.append("%+.2f%s" % (axis[k], nm))
    return " ".join(labels) if labels else "?"


def polar(M):
    """Nearest rotation matrix to M (SVD projection, det forced +1)."""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def two_sided_procrustes(A, B, iters=40):
    """Fit R_a(t) ~ Rw . R_b(t) . Rbody over a stack of rotation matrices."""
    Rw = np.eye(3)
    Rb = np.eye(3)
    for _ in range(iters):
        Rw = polar(np.einsum("nij,nkj->ik", A, np.einsum("nij,jk->nik", B, Rb)))
        Rb = polar(np.einsum("nji,njk->ik", np.einsum("ij,njk->nik", Rw, B), A))
    pred = np.einsum("ij,njk,kl->nil", Rw, B, Rb)
    # axis_angle already returns degrees; the residual is the angle of the
    # rotation still separating prediction from measurement.
    resid = np.array([axis_angle(q.T @ a)[0] for q, a in zip(pred, A)])
    return Rw, Rb, resid


def read_topic(bagname, topic, msgtype):
    dbs = sorted(glob.glob("%s/bags/%s/*.db3" % (BASE, bagname)))
    if not dbs:
        return None, None
    from rclpy.serialization import deserialize_message
    try:
        con = sqlite3.connect("file:%s?mode=ro" % dbs[0], uri=True)
        tid = con.execute("select id from topics where name=?", (topic,)).fetchone()
        if tid is None:
            return None, None
        rows = con.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp",
            (tid[0],),
        ).fetchall()
    except sqlite3.DatabaseError:
        return None, None
    finally:
        try:
            con.close()
        except Exception:
            pass
    t = np.array([r[0] for r in rows], dtype=np.float64) * 1e-9
    return t, [deserialize_message(bytes(r[1]), msgtype) for r in rows]


def slope_r(a, b):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    keep = np.isfinite(a) & np.isfinite(b)
    a, b = a[keep], b[keep]
    if a.size < 20 or np.dot(a, a) < 1e-9 or a.std() < 1e-9 or b.std() < 1e-9:
        return float("nan"), float("nan")
    return float(np.dot(a, b) / np.dot(a, a)), float(np.corrcoef(a, b)[0, 1])


def analyse(lid):
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleOdometry
    import atmo.rl_combined_runtime as rlc

    bagname = RUNS[lid]
    tb, bridge = read_topic(bagname, "/atmo/groundtruth_odom", Odometry)
    tf, fc = read_topic(bagname, "/fmu/out/vehicle_odometry", VehicleOdometry)
    print("\n" + "=" * 74)
    print("log_%d   %s" % (lid, bagname))
    print("=" * 74)
    if tb is None or tf is None:
        print("  bag lacks one of the two topics -- nothing to prove here")
        return

    # ---- feed the DEPLOYED builder with the real logged messages ----------
    fixed = "--fixed" in sys.argv
    builder = rlc.CombinedObservationBuilder(rlc.CombinedStage1Config())
    obs_pos, obs_quat, obs_w = [], [], []
    bri_pos, bri_quat = [], []
    for m in bridge:
        p, o = m.pose.pose.position, m.pose.pose.orientation
        tw = m.twist.twist
        P = np.array((p.x, p.y, p.z))
        Q = np.array((o.w, o.x, o.y, o.z))
        Vb = np.array((tw.linear.x, tw.linear.y, tw.linear.z))
        Wb = np.array((tw.angular.x, tw.angular.y, tw.angular.z))
        if fixed:
            # The 2026-08-20 fixes: the bridge corrects the 180 deg mount, and
            # the mocap path no longer runs through PX4's conversion.
            from atmo.mocap_frames import apply_mount_yaw
            Q = np.asarray(apply_mount_yaw(Q, math.pi), dtype=np.float64)
            vel_w = quat_to_rotmat(Q) @ Vb
            builder.update_training_frame_state(P, Q, vel_w, Wb)
        else:
            # exactly what mocap_odom_callback did before
            vel_w = quat_to_rotmat(Q) @ Vb
            builder.update_px4_state(P, Q, vel_w, Wb)
        obs_pos.append(np.array(builder.position, dtype=np.float64))
        obs_quat.append(np.array(builder.quat_wxyz, dtype=np.float64))
        obs_w.append(np.array(builder.angular_velocity_w, dtype=np.float64))
        bri_pos.append(P)
        bri_quat.append(Q)
    obs_pos = np.array(obs_pos)
    bri_pos = np.array(bri_pos)
    R_obs = np.array([quat_to_rotmat(q) for q in obs_quat])
    R_bri = np.array([quat_to_rotmat(q) for q in bri_quat])

    # ---- FC attitude, nearest sample on the bridge stamps -----------------
    R_fc_all = np.array([quat_to_rotmat(np.array(m.q, dtype=np.float64)) for m in fc])
    idx = np.clip(np.searchsorted(tf, tb), 0, len(tf) - 1)
    R_fc = R_fc_all[idx]

    # Only fit where there is real attitude to fit: lean > 3 deg.
    lean_fc = np.degrees(np.arccos(np.clip(R_fc[:, 2, 2], -1, 1)))
    live = lean_fc > 3.0
    n = int(live.sum())
    print("  %d bridge samples, %d with FC lean > 3 deg" % (len(tb), n))
    if n < 200:
        print("  too few attitude samples to fit a frame -- skipping the rotations")
    else:
        # Heading-independent comparison. A body-frame gravity vector does not
        # depend on either world's heading, so fitting
        #     g_body_other = M . g_body_fc
        # measures the BODY frame relation alone -- the mount and the z
        # convention -- with the yaw drift between the two worlds excluded by
        # construction. Fitting full attitude instead does NOT work here: the
        # two headings drift against each other (see CLAIM 6), so no single
        # fixed pair of rotations describes the run.
        g_fc = R_fc[live].transpose(0, 2, 1) @ np.array((0.0, 0.0, 1.0))
        for label, R_other in (("CLAIM 1  bridge (what mocap published) vs the FC",
                                R_bri[live]),
                               ("CLAIM 2  obs (after update_px4_state) vs the FC",
                                R_obs[live])):
            v_other = R_other.transpose(0, 2, 1) @ np.array((0.0, 0.0, 1.0))
            # Unconstrained first, so an improper map is visible rather than
            # silently projected away. det < 0 means this source's world z
            # points the OTHER way from the FC's (NED z is down), i.e. z-up.
            M = np.linalg.lstsq(g_fc, v_other, rcond=None)[0].T
            det = float(np.linalg.det(M))
            zsign = "down (same as NED)" if det > 0 else "UP (opposite NED)"
            R_body = polar(M if det > 0 else -M)
            ang, ax = axis_angle(R_body)
            pred = g_fc @ M.T
            err = np.degrees(np.arccos(np.clip(
                np.einsum("ni,ni->n", pred, v_other)
                / (np.linalg.norm(pred, axis=1) * np.linalg.norm(v_other, axis=1)),
                -1, 1)))
            print("\n  %s" % label)
            print("     body-frame vertical fit   v_other = M . g_fc      (n=%d)" % n)
            print("       M diag %s   offdiag max %.2f   det %+.2f"
                  % (np.round(np.diag(M), 2), np.abs(M - np.diag(np.diag(M))).max(), det))
            print("       => this stream's world z points %s" % zsign)
            print("       => body frame = FC body rotated %.1f deg about %s"
                  % (ang, name_axis(ax)))
            print("       residual  median %.1f deg, p90 %.1f deg"
                  % (np.median(err), np.percentile(err, 90)))

        print("\n  CLAIM 6  the two headings DRIFT, so the mount cannot be read")
        print("           off a yaw offset alone")

        def yaw_of(M):
            return np.arctan2(M[:, 1, 0], M[:, 0, 0])
        dy = np.degrees(np.unwrap(yaw_of(R_bri[live]) - yaw_of(R_fc[live])))
        tt = tb[live] - tb[live][0]
        drift = np.polyfit(tt, dy, 1)[0] if np.ptp(tt) > 1.0 else float("nan")
        print("     bridge yaw - FC yaw: median %+.1f deg, p10..p90 %+.1f..%+.1f, "
              "drift %+.1f deg/s" % (np.median(dy), np.percentile(dy, 10),
                                     np.percentile(dy, 90), drift))

        def rpy(M):
            return (math.atan2(M[2, 1], M[2, 2]),
                    math.asin(np.clip(-M[2, 0], -1, 1)),
                    math.atan2(M[1, 0], M[0, 0]))
        e_obs = np.array([rpy(M) for M in R_obs[live]])
        e_bri = np.array([rpy(M) for M in R_bri[live]])
        e_fc = np.array([rpy(M) for M in R_fc[live]])
        print("\n     per-axis Euler, for continuity with the earlier tables")
        print("     (FC read as NED/FRD, i.e. z DOWN -- not converted)")
        for k, ax in enumerate(("roll", "pitch")):
            sb, rb = slope_r(e_fc[:, k], e_bri[:, k])
            so, ro = slope_r(e_fc[:, k], e_obs[:, k])
            print("       %-6s bridge/FC %+.2f (r %+.2f)   obs/FC %+.2f (r %+.2f)"
                  % (ax, sb, rb, so, ro))

    # ---- position claims: these need no fit at all -------------------------
    print("\n  CLAIM 3  the observation's height is the NEGATIVE of the true one")
    s, r = slope_r(bri_pos[:, 2], obs_pos[:, 2])
    rng = bri_pos[:, 2].max() - bri_pos[:, 2].min()
    print("     obs z vs bridge z   slope %+.3f  r %+.3f   (bridge z spans %.2f m)"
          % (s, r, rng))

    print("\n  CLAIM 4  x and y are swapped in the observation")
    for a, b, lbl in ((0, 1, "obs x vs bridge y"), (1, 0, "obs y vs bridge x"),
                      (0, 0, "obs x vs bridge x"), (1, 1, "obs y vs bridge y")):
        s, r = slope_r(bri_pos[:, b], obs_pos[:, a])
        print("     %-20s slope %+.3f  r %+.3f" % (lbl, s, r))

    print("\n  CLAIM 5  body rates survive into the observation")
    # builder stores world-frame angular velocity; rotate back with its own quat
    w_body_obs = np.array([quat_to_rotmat(q).T @ w for q, w in zip(obs_quat, obs_w)])
    w_body_in = np.array([(m.twist.twist.angular.x, m.twist.twist.angular.y,
                           m.twist.twist.angular.z) for m in bridge])
    for k, ax in enumerate(("p", "q", "r")):
        s, rr = slope_r(w_body_in[:, k], w_body_obs[:, k])
        print("     obs %s vs bridge %s   slope %+.3f  r %+.3f" % (ax, ax, s, rr))


def main():
    wanted = [int(a) for a in sys.argv[1:] if a.isdigit()] or sorted(RUNS)
    print("Frame chain, proved run by run against the deployed builder")
    if "--fixed" in sys.argv:
        print("  --fixed: mount correction ON, mocap path NOT converted."
              "\n  TARGET: CLAIM 2 must match CLAIM 1's z-up, and read 180 deg about x"
              "\n          (= the training frame); height slope +1.000; x/y unswapped.")
    for lid in wanted:
        try:
            analyse(lid)
        except Exception as exc:  # a corrupt bag must not kill the sweep
            print("\nlog_%d: FAILED (%s)" % (lid, exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
