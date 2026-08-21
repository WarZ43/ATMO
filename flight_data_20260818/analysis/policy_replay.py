#!/usr/bin/env python3
"""Replay the DEPLOYED policy network over the flight observations.

Why this exists (2026-08-19): the policy's raw actions were never logged --
only post-mixer motor commands were -- and reconstructing "what the policy
asked for" by inverting the mixer is circular for any mixer-sign question.
The only non-circular source of policy intent is the network itself, run over
the observation stream it saw. Everything needed survives on the laptop:

  policy   ~/ATMO/atmo_ws/policies/atmo_combined_stage1_policy.npz
           (ep_1800_rew_890.5606, sha256 53e4b221... = the Jetson deploy
           verified 2026-08-17; ~/Documents holds a later ep_2450 export --
           pass --policy to try it)
  runtime  ~/ATMO/atmo_ws/src/atmo/atmo/ rl_combined_runtime + landing stage1
           (imported directly, NOT reimplemented, so obs packing, frames,
           normalizer, adapter and gates are the deployed code paths)
  state    ulog vehicle_odometry (POSE_SOURCE=px4 path), and/or the bag's
           /atmo/groundtruth_odom (POSE_SOURCE=mocap path)
  tilt     measured tilt from the tilt_controller launch.log prints
           ("tilt angle is: X deg"), wall-clock aligned via the run's
           bag->ulog offset

Self-validation: the replay republishes motors through the deployed adapter
and gates; correlating them against the ulog's logged actuator_motors scores
how faithful the replay is, and which pose source / engagement time the
flight actually used. Only then do the raw actions mean anything.

Usage: source ROS + workspace, then
  python3 policy_replay.py 417            # scan engagement, both pose sources
  python3 policy_replay.py 417 --engage-at 14.2 --pose-source px4
"""
import argparse
import glob
import math
import os
import re
import sqlite3
import sys
import time as _time

import numpy as np

BASE = "/home/warz42/ATMO/flight_data_20260818"
ATMO_SRC = "/home/warz42/ATMO/atmo_ws/src/atmo"
POLICY = "/home/warz42/ATMO/atmo_ws/policies/atmo_combined_stage1_policy.npz"
ROSLOG_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roslogs")

# per-run: bag name, bag->ulog offset (roll-signature / xcorr values from
# smatrix_validation.py), ros launch-log session dir (wall clock == bag clock)
RUNS = {
    413: ("atmo_policy_20260818_200125", -1787106990.94, "2026-08-18-20-01-26-969290-m4-6661"),
    414: ("atmo_policy_20260818_200413", -1787106548.74, "2026-08-18-20-04-14-586893-m4-7429"),
    415: ("atmo_policy_20260818_202503", -1787106789.09, "2026-08-18-20-25-04-122586-m4-8580"),
    417: ("atmo_policy_20260818_205315", -1787111284.67, "2026-08-18-20-53-16-763102-m4-2894"),
}

sys.path.insert(0, ATMO_SRC)
os.environ.setdefault("ATMO_RL_ROUTE", "takeoff")
os.environ.setdefault("ATMO_RL_MODE", "policy")

from pyulog import ULog  # noqa: E402


# ---- a controllable clock, patched into the deployed runtime --------------
class FakeClock:
    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t


CLOCK = FakeClock()
_time.monotonic = CLOCK.monotonic  # rl_*_runtime call time.monotonic()


def load_ulog_odometry(lid):
    # NOTE: these ulogs carry NO vehicle_odometry (SDLOG profile), so the
    # POSE_SOURCE=px4 branch cannot be replayed from them -- which is itself
    # evidence the flight ran POSE_SOURCE=mocap, as rl_controller_hardware.py
    # recommends after the 2026-08-14 EKF blowup. Only the logged motors are
    # read here (the replay's validation target).
    u = ULog("%s/ulogs/log_%d.ulg" % (BASE, lid),
             message_name_filter_list=["actuator_motors"])
    am = next((d for d in u.data_list if d.name == "actuator_motors"), None)
    ta = am.data["timestamp"] / 1e6
    rot = np.stack([am.data["control[%d]" % i] for i in range(4)], 1)
    ok = np.isfinite(rot).all(1)
    return None, (ta[ok], rot[ok])


def load_bag_odometry(lid):
    """(t_ulog, pos, quat_wxyz, vel_body, angvel_body) from /atmo/groundtruth_odom."""
    bagname, offset, _ = RUNS[lid]
    dbs = glob.glob("%s/bags/%s/*.db3" % (BASE, bagname))
    if not dbs:
        return None
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    try:
        c = sqlite3.connect("file:%s?mode=ro" % dbs[0], uri=True)
        topics = {n: i for i, n in c.execute("select id, name from topics")}
        if "/atmo/groundtruth_odom" not in topics:
            return None
        rows = c.execute("select timestamp, data from messages where topic_id=?"
                         " order by timestamp",
                         (topics["/atmo/groundtruth_odom"],)).fetchall()
    except sqlite3.DatabaseError:
        return None
    t, P, Q, V, W = [], [], [], [], []
    for ts, blob in rows:
        try:
            m = deserialize_message(blob, Odometry)
        except Exception:
            continue
        t.append(ts / 1e9 + offset)
        p, o = m.pose.pose.position, m.pose.pose.orientation
        tw = m.twist.twist
        P.append((p.x, p.y, p.z))
        Q.append((o.w, o.x, o.y, o.z))
        V.append((tw.linear.x, tw.linear.y, tw.linear.z))
        W.append((tw.angular.x, tw.angular.y, tw.angular.z))
    return tuple(np.asarray(a) for a in (t, P, Q, V, W))


def load_measured_tilt(lid):
    """(t_ulog, tilt_rad) from the tilt node's launch.log prints, or None.

    The 2026-08-18 recorder never captured /fmu/in/tilt_angle, but the tilt
    node PRINTED the measured angle, and launch.log stamps are wall clock =
    bag clock, so the run's bag->ulog offset places them on the ulog timeline.
    Convention check: the runtime clips tilt to [0, pi/2] with 0 = fly.
    """
    _, offset, sess = RUNS[lid]
    path = os.path.join(ROSLOG_ROOT, sess, "launch.log")
    if not os.path.exists(path):
        return None
    ts, deg = [], []
    pat = re.compile(r"^(\d+\.\d+).*tilt angle is:\s*(-?\d+(?:\.\d+)?)\s*deg")
    with open(path) as f:
        for line in f:
            m = pat.match(line)
            if m:
                ts.append(float(m.group(1)) + offset)
                deg.append(float(m.group(2)))
    if not ts:
        return None
    return np.asarray(ts), np.radians(np.asarray(deg))


def replay(lid, t_engage, pose_source, policy_path, tilt_series, odom, t_end):
    """Run the deployed stack from t_engage; return per-step records."""
    # Fresh runtime objects per replay (the builder holds history state).
    import importlib
    import atmo.rl_landing_stage1_runtime as rls
    import atmo.rl_combined_runtime as rlc
    importlib.reload(rls)
    importlib.reload(rlc)
    rls.time.monotonic = CLOCK.monotonic
    rlc.time.monotonic = CLOCK.monotonic

    os.environ["ATMO_RL_POLICY_PATH"] = policy_path
    cfg = rlc.CombinedStage1Config()
    builder = rlc.CombinedObservationBuilder(cfg)
    adapter = rlc.LandingActionAdapter(cfg)
    from atmo.numpy_actor import NumpyActor
    actor = NumpyActor(policy_path, cfg.observation_dim, cfg.action_dim)

    (t_o, P, Q, V, W) = odom
    tilt_t, tilt_v = tilt_series if tilt_series is not None else (None, None)

    def feed_state(t):
        i = int(np.searchsorted(t_o, t))
        i = min(max(i, 0), len(t_o) - 1)
        if pose_source == "px4":
            builder.update_px4_state(P[i], Q[i], V[i], W[i])
        else:
            # the mocap callback rotates body lin vel to world, then calls the
            # same update; reproduce that exactly (rl_controller_hardware.py)
            vel_w = rls.quat_wxyz_to_rotmat(Q[i]) @ V[i]
            builder.update_px4_state(P[i], Q[i], vel_w, W[i])

    def feed_tilt(t):
        if tilt_t is None:
            return
        i = int(np.searchsorted(tilt_t, t))
        i = min(max(i, 0), len(tilt_t) - 1)
        adapter.set_tilt_angle(float(tilt_v[i]))
        builder.set_tilt_angle(float(tilt_v[i]))

    # ---- engagement, exactly as _engage() does it
    CLOCK.t = t_engage
    feed_state(t_engage)
    feed_tilt(t_engage)
    builder.reset_policy_context()
    adapter.reset()
    measured_tilt = float(builder.tilt_angle)
    builder.anchor_fixed_vertical_route()
    adapter.set_tilt_angle(measured_tilt)
    builder.set_tilt_angle(measured_tilt)

    rec = {k: [] for k in ("t", "action", "semantic", "rotors_pub", "mode", "gate",
                           "clip_frac", "obs_norm")}
    dt = cfg.policy_dt
    steps = int((t_end - t_engage) / dt)
    for k in range(steps):
        t = t_engage + k * dt
        CLOCK.t = t
        feed_state(t)
        feed_tilt(t)
        obs = builder.observation()
        # how far outside the training distribution is this observation?
        nv = (obs - actor.observation_mean) / np.sqrt(actor.observation_variance + 1e-5)
        clip_frac = float(np.mean(np.abs(nv) >= 5.0))
        sat_norm = float(np.mean(np.abs(np.clip(nv, -5, 5))))
        action = actor(obs)
        command = adapter.pre_physics_step(action)
        builder.set_tilt_angle(command.tilt_angle) if tilt_t is None else None
        builder.append_action(command.semantic_action)
        gate = builder.rotor_thrust_gate()
        rec["t"].append(t)
        rec["action"].append(action.copy())
        rec["semantic"].append(command.semantic_action.copy())
        rec["rotors_pub"].append(command.rotors * gate)
        rec["mode"].append(builder.mode)
        rec["gate"].append(gate)
        rec["clip_frac"].append(clip_frac)
        rec["obs_norm"].append(sat_norm)
    return {k: np.asarray(v) for k, v in rec.items()}


def score(rec, ta, rot):
    """Correlation of replayed vs logged motor commands, per rotor + pattern."""
    m = (ta >= rec["t"][0]) & (ta <= rec["t"][-1])
    if m.sum() < 6:
        return -1.0, np.zeros(4)
    R = np.stack([np.interp(ta[m], rec["t"], rec["rotors_pub"][:, i])
                  for i in range(4)], 1)
    L = rot[m]
    per = np.array([np.corrcoef(R[:, i], L[:, i])[0, 1]
                    if R[:, i].std() > 1e-6 and L[:, i].std() > 1e-6 else 0.0
                    for i in range(4)])
    # differential pattern (what the mixer question is actually about)
    Rd, Ld = R - R.mean(1, keepdims=True), L - L.mean(1, keepdims=True)
    pat = float(np.corrcoef(Rd.reshape(-1), Ld.reshape(-1))[0, 1]) \
        if Rd.std() > 1e-6 else 0.0
    return pat, per


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lid", type=int)
    ap.add_argument("--engage-at", type=float, default=None,
                    help="ulog time of engagement; default: scan")
    ap.add_argument("--pose-source", choices=("px4", "mocap", "both"), default="mocap")
    ap.add_argument("--policy", default=POLICY)
    ap.add_argument("--t-end", type=float, default=None)
    args = ap.parse_args()
    lid = args.lid

    odom_u, (ta, rot) = load_ulog_odometry(lid)
    odom_m = load_bag_odometry(lid)
    tilt = load_measured_tilt(lid)
    thr = rot.mean(1)
    t_spin = ta[np.argmax(thr > 0.1)]
    hi = int(np.argmax(thr > 0.5)) if (thr > 0.5).any() else 0
    t_kill = ta[hi + int(np.argmax(thr[hi:] < 0.02))] if (thr[hi:] < 0.02).any() else ta[-1]
    t_end = args.t_end if args.t_end else t_kill + 0.3
    print("run %d: spinup %.2f  kill %.2f  tilt log %s (%d pts)"
          % (lid, t_spin, t_kill, "yes" if tilt else "NO", 0 if tilt is None else len(tilt[0])))

    sources = [args.pose_source] if args.pose_source != "both" else ["px4", "mocap"]
    best = None
    for src in sources:
        odom = odom_u if src == "px4" else odom_m
        if odom is None:
            print("  %s: no odometry available" % src)
            continue
        engages = [args.engage_at] if args.engage_at is not None else \
            list(np.arange(t_spin - 12.0, t_spin - 1.0, 0.25))
        for te in engages:
            rec = replay(lid, te, src, args.policy, tilt, odom, t_end)
            pat, per = score(rec, ta, rot)
            if best is None or pat > best[0]:
                best = (pat, per, src, te, rec)
        print("  %s best so far: pattern r=%.3f" % (src, best[0]))

    pat, per, src, te, rec = best
    print("BEST: pose=%s engage@%.2f  pattern r=%.3f  per-rotor %s"
          % (src, te, pat, np.round(per, 2)))

    # the deliverable: the policy's OWN raw actions on the ulog timeline
    out = np.column_stack([rec["t"], rec["action"], rec["semantic"],
                           rec["rotors_pub"], rec["mode"], rec["gate"],
                           rec["clip_frac"], rec["obs_norm"]])
    hdr = ("t_ulog," + ",".join("raw_%s" % n for n in
           ("lift", "roll", "pitch", "yaw", "tilt", "wheel0", "wheel1"))
           + "," + ",".join("sem_%s" % n for n in
           ("lift", "roll", "pitch", "yaw", "tilt", "wheel0", "wheel1"))
           + ",pub0,pub1,pub2,pub3,mode,gate,clip_frac,obs_norm")
    outpath = "%s/analysis/log_%d/replay.csv" % (BASE, lid)
    np.savetxt(outpath, out, delimiter=",", header=hdr, comments="")
    print("wrote %s  (%d steps at 50 Hz)" % (outpath, len(rec["t"])))


if __name__ == "__main__":
    main()
