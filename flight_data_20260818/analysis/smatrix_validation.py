#!/usr/bin/env python3
"""Validate Ioannis' MPC S-matrix (with tilt angle) against flight IMU data.

Per armed run:
  1. Merge ulog (rotors, attitude, rates) + rosbag (mocap pose, tilt_vel) on
     the arming instant; integrate commanded tilt_vel from 85 deg for phi(t).
  2. Predict angular acceleration from Ioannis' S(phi) torque rows and
     M(phi) inertia diagonals + Euler coupling.
  3. Plot expected vs measured acceleration (timeline + scatter w/ fit).
  4. Reconstruct the policy's semantic actions through the era-correct mixer.

Outputs: analysis/<run>/merged.csv + PNGs.
"""
import glob
import math
import sqlite3
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

sys.path.insert(0, "/opt/ros/humble/local/lib/python3.10/dist-packages")
from rclpy.serialization import deserialize_message  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from custom_msgs.msg import TiltVel  # noqa: E402
from px4_msgs.msg import ActuatorMotors  # noqa: E402

BASE = "/home/warz42/ATMO/flight_data_20260818"
COL = {"roll": "#4269d0", "pitch": "#b45309", "yaw": "#2e7d4f", "aux": "#8f4bab"}

# ------- Ioannis parameters.py -------
kT, kM = 28.15, 0.018
m_base, m_arm, m_rotor = 2.33, 1.537, 0.021
Ibxx, Ibyy, Ibzz = 0.0067, 0.011, 0.0088
Iaxx, Iayy, Iazz = 0.008732, 0.036926, 0.043822
Irxx, Iryy, Irzz = 0.000022, 0.000022, 0.000043
rBAx, rBAy, rBAz = 0.0066, 0.0685, -0.021
rAGx, rAGy, rAGz = -0.00032, 0.16739, -0.02495
rA1x, rA1y, rA1z = 0.16491, 0.13673, -0.069563

OLD_MIX = np.array(((1, 1, -1), (-1, -1, -1), (-1, 1, 1), (1, -1, 1)), float)
NEW_MIX = np.array(((1, 1, 1), (-1, -1, 1), (-1, 1, -1), (1, -1, -1)), float)

# ---- rotor ("wheel") spin geometry, read back out of Ioannis' S_func -----
# Spin sense sigma_i: the sign of the kM*cos(phi) term in the yaw column
# (motors 1,2 spin one way, 3,4 the other).
SPIN = np.array([1.0, 1.0, -1.0, -1.0])
# Tilt direction e_i: the sign the rA1x*sin(phi) yaw term demands given each
# motor's x arm (from its B column). At phi=90 (drive) the four thrust axes
# become the four wheel AXLES, all horizontal along +/-y -- which is exactly
# how this vehicle rolls. Rotor axis: n_i(phi) = (0, e_i sin phi, cos phi).
TILT_Y = np.array([1.0, -1.0, -1.0, 1.0])
# Full-command rotor speed. NOT logged (PWM ESCs, no esc_status), so this is
# the single free parameter of the gyroscopic term and every gyro torque
# below scales linearly with it. 730 rad/s ~ 7000 rpm is the plausible
# full-throttle speed for a prop making kT_eff = 59 N; bench-measure it.
OMEGA_MAX = 730.0


# ---- rotor hub / flapping moment ----------------------------------------
# The disc, not the hub, is the gyro. A shaft angular rate makes the blades
# flap, and because the flapping response lags the excitation by 90 deg of
# azimuth, a PITCH rate tilts the disc SIDEWAYS -- a roll hub moment. Two
# halves, with very different behaviour across four rotors:
#   damping half   (16/gamma)(rate/Omega): same sign for every rotor, so the
#                  four ADD. This is the piece a pure-thrust S matrix has no
#                  representation of at all.
#   precession half     sigma*(rate/Omega): flips with spin direction, so it
#                  cancels for four matched rotors and survives only on an
#                  unbalanced set (log_417).
# LOCK and K_BETA are unmeasured for these props; K_BETA is fitted from the
# roll residual below (fit_flap) rather than guessed, and every moment here
# is exactly linear in it.
LOCK = 3.0          # Lock number gamma; small stiff props sit low
K_BETA = 0.4        # hub flapping stiffness [N.m/rad]. NOT measured: the
                    # 413/415 residual fits that first produced ~0.3-0.5 were
                    # taken through ground contact and are void (S.11.3).
                    # Nominal placeholder for sizing only -- bench it.
OMEGA_FLOOR = 0.05  # fraction of OMEGA_MAX below which the rotor is "dead"


def flap_regressors(spd, phi, w):
    """Roll/pitch/yaw hub-moment shape per unit K_BETA, split damping/precession.

    Tilt is a rotation of the disc about body x, so the disc frame is
      shaft n = (0, e sin phi, cos phi)
      in-plane e1 = (1, 0, 0),  e2 = n x e1 = (0, cos phi, -e sin phi)
    Shaft rates in that frame are p_d = w.e1, q_d = w.e2; quasi-steady flap
      beta_1 = (16/gamma)(p_d/Om) + sigma (q_d/Om)     [about e1]
      beta_2 = (16/gamma)(q_d/Om) - sigma (p_d/Om)     [about e2]
    and the hub opposes it: tau = -K_BETA (beta_1 e1 + beta_2 e2).
    Returns (damping_torque, precession_torque) per unit K_BETA.
    """
    c, s = math.cos(phi), math.sin(phi)
    live = spd > OMEGA_FLOOR
    Om = np.maximum(spd, OMEGA_FLOOR) * OMEGA_MAX
    e1 = np.array([1.0, 0.0, 0.0])
    e2 = np.stack([np.zeros(4), np.full(4, c), -TILT_Y * s], 1)     # (4,3)
    p_d = np.full(4, float(np.dot(w, e1)))
    q_d = e2 @ w
    damp = np.zeros(3)
    prec = np.zeros(3)
    for i in range(4):
        if not live[i]:
            continue
        damp += -((16.0 / LOCK) * (p_d[i] / Om[i])) * e1
        damp += -((16.0 / LOCK) * (q_d[i] / Om[i])) * e2[i]
        prec += -(SPIN[i] * q_d[i] / Om[i]) * e1
        prec += +(SPIN[i] * p_d[i] / Om[i]) * e2[i]
    return damp, prec


def rotor_gyro_torque(spd, spd_dot, phi, phidot, w):
    """Body torque from the four spinning rotors treated as gyros.

    spd      : (4,) rotor speed as a fraction of OMEGA_MAX (the EMA'd command)
    spd_dot  : (4,) its time derivative [1/s]
    phi      : tilt [rad], phidot: tilt rate [rad/s]
    w        : (3,) body rate [rad/s]

    Three pieces, all reactions of d/dt (rotor angular momentum):
      gyro : -w x h            body rotation precessing the spin axes
      prec : -Irzz*phidot*sum(sigma*Omega*dn/dphi)   the arms tilting them
      spin : -Irzz*sum(sigma*Omega_dot*n)            spin-up/-down reaction
    With four healthy rotors sum(sigma) = 0 and sum(sigma*e) = 0, so h
    cancels and the gyro term vanishes -- it only bites when the rotors are
    UNBALANCED (see the log_417 front-right prop, HANDOFF section 6).
    """
    c, s = math.cos(phi), math.sin(phi)
    n = np.stack([np.zeros(4), TILT_Y * s, np.full(4, c)], 1)      # (4,3)
    dn = np.stack([np.zeros(4), TILT_Y * c, np.full(4, -s)], 1)    # dn/dphi
    L = Irzz * OMEGA_MAX
    h = L * (SPIN * spd) @ n
    gyro = -np.cross(w, h)
    prec = -L * phidot * ((SPIN * spd) @ dn)
    spin = -L * ((SPIN * spd_dot) @ n)
    return gyro, prec, spin


def s_torque(phi):
    """4x3 torque rows of Ioannis' S_func at tilt phi (rows=motors u1..u4)."""
    c, s = np.cos(phi), np.sin(phi)
    A = kT * (rA1y + rBAy * c + rBAz * s)
    B1 = kT * (rA1x * c + rBAx * c - kM * s)
    B2 = kT * (rBAx * c - rA1x * c + kM * s)
    C1 = kT * (kM * c + rA1x * s + rBAx * s)
    C2 = kT * (kM * c + rA1x * s - rBAx * s)
    return np.array([
        (-A, B1,  C1),
        ( A, B2,  C2),
        ( A, B1, -C1),
        (-A, B2, -C2),
    ])


def inertia(phi):
    c, s = np.cos(phi), np.sin(phi)
    Ixx = (2*Iaxx + Ibxx + 4*Irxx + 2*m_arm*(rBAy**2+rBAz**2)
           + 4*m_rotor*(rA1y**2+rA1z**2+rBAy**2+rBAz**2)
           + 4*m_arm*(rAGy*rBAy+rAGz*rBAz)*c + 8*m_rotor*(rA1y*rBAy+rA1z*rBAz)*c
           + 4*m_arm*(rAGy*rBAz-rAGz*rBAy)*s + 8*m_rotor*(rA1y*rBAz-rA1z*rBAy)*s)
    Iyy = (2*Iazz + Ibyy + 4*Irzz + 2*m_arm*(rBAx**2+rBAz**2)
           + 4*m_rotor*(rA1x**2+rA1y**2+rBAx**2+rBAz**2)
           + 2*Iayy*c**2 - 2*Iazz*c**2 + 4*Iryy*c**2 - 4*Irzz*c**2
           + 4*m_arm*rAGx*rBAx - 4*m_rotor*rA1y**2*c**2 + 4*m_rotor*rA1z**2*c**2
           + 4*m_arm*rAGz*rBAz*c + 8*m_rotor*rA1z*rBAz*c)
    Izz = (2*Iayy + Ibzz + 4*Iryy + 2*m_arm*(rBAx**2+rBAy**2)
           + 4*m_rotor*(rA1x**2+rA1z**2+rBAx**2+rBAy**2)
           - 2*Iayy*c**2 + 2*Iazz*c**2 - 4*Iryy*c**2 + 4*Irzz*c**2
           + 4*m_arm*rAGx*rBAx + 4*m_rotor*rA1y**2*c**2 - 4*m_rotor*rA1z**2*c**2
           + 4*m_arm*rAGy*rBAy*c + 8*m_rotor*rA1y*rBAy*c)
    return np.array([Ixx, Iyy, Izz])


def bag_series(bagdir, topic, msgtype, extract):
    """Read one topic out of a rosbag2 sqlite file.

    Returns empty on a missing or CORRUPT bag rather than raising: bag
    204231 (flight 2b) was killed before checkpoint -- no metadata.yaml, and
    the .db3 fails `pragma integrity_check` outright -- and the ulog-only
    analysis of that run is still worth having.
    """
    dbs = glob.glob("%s/*.db3" % bagdir)
    if not dbs:
        return np.array([]), []
    try:
        c = sqlite3.connect("file:%s?mode=ro" % dbs[0], uri=True)
        topics = {name: tid for tid, name in c.execute("select id, name from topics")}
        if topic not in topics:
            return np.array([]), []
        rows = c.execute("select timestamp, data from messages where topic_id=? order by timestamp",
                         (topics[topic],)).fetchall()
    except sqlite3.DatabaseError as e:
        print("  bag unreadable (%s): %s -- ulog-only for this run" %
              (bagdir.rsplit("/", 1)[-1], e))
        return np.array([]), []
    ts, vals = [], []
    for t, blob in rows:
        try:
            m = deserialize_message(blob, msgtype)
        except Exception:
            continue
        ts.append(t / 1e9)
        vals.append(extract(m))
    return np.array(ts), vals


def bag_to_ulog_offset(bagdir, ta_u, rot_u):
    """Return offset such that t_ulog = t_bag + offset.

    Preferred: cross-correlate bag-recorded actuator_motors with the ulog's.
    Fallback: first sustained tilt_vel burst = rotor spinup + 2.0 s
    (drive_to_takeoff fires 2 s after engagement)."""
    ts, vals = bag_series(bagdir, "/fmu/in/actuator_motors", ActuatorMotors,
                          lambda m: float(np.nanmean(np.array(m.control[:4]))))
    if len(ts) > 1500:
        v = np.array(vals)
        okm = np.isfinite(v)
        ts, v = ts[okm], v[okm]
        gb = np.arange(ts[0], ts[-1], 0.02)
        sb = np.interp(gb, ts, v); sb -= sb.mean()
        mu = np.nanmean(rot_u, 1)
        gu = np.arange(ta_u[0], ta_u[-1], 0.02)
        su = np.interp(gu, ta_u, mu); su -= su.mean()
        best, bofs = -1e9, 0.0
        for k in range(-1500, 1500, 2):
            if k >= 0: a, b = su[k:], sb
            else: a, b = su, sb[-k:]
            n = min(len(a), len(b))
            if n < 300: continue
            r = float(np.dot(a[:n], b[:n]))
            if r > best:
                best, bofs = r, gu[max(k, 0)] - gb[max(-k, 0)]
        return bofs, "actuator xcorr"
    if len(ts) > 10:
        t_spin = ta_u[np.argmax(np.nanmean(rot_u, 1) > 0.1)]
        return t_spin - ts[0], "first-actuator anchor"
    # fallback: tilt burst
    from custom_msgs.msg import TiltVel as _TV
    tt, tv = bag_series(bagdir, "/tilt_vel", _TV, lambda m: m.value)
    tv = np.array(tv)
    t_spin = ta_u[np.argmax(np.nanmean(rot_u, 1) > 0.1)]
    nz = np.abs(tv) > 0.5
    if nz.any():
        t_burst = tt[np.argmax(nz)]
        return (t_spin + 2.0) - t_burst, "tilt-burst heuristic (+/-0.3 s)"
    return None, "none"


def roll_signature_offset(bagdir, tq, q):
    """Offset from |roll| cross-correlation, mocap attitude vs ulog attitude.

    HANDOFF section 1: where the recorder matched late, the first-actuator
    anchor slides the window ~2 s early and fabricates conclusions (it once
    produced "log_417 never left the ground"). The airframe's roll history is
    recorded honestly in BOTH clocks, so match on that instead.
    """
    ts, vals = bag_series(bagdir, "/atmo/groundtruth_odom", Odometry,
                          lambda m: (m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                                     m.pose.pose.orientation.y, m.pose.pose.orientation.z))
    if len(ts) < 200:
        return None
    Qb = np.array(vals)
    rb = np.abs(quat_to_rpy(Qb[:, 0], Qb[:, 1], Qb[:, 2], Qb[:, 3])[0])
    ru = np.abs(quat_to_rpy(q[:, 0], q[:, 1], q[:, 2], q[:, 3])[0])
    gb = np.arange(ts[0], ts[-1], 0.02)
    sb = np.interp(gb, ts, rb); sb -= sb.mean()
    gu = np.arange(tq[0], tq[-1], 0.02)
    su = np.interp(gu, tq, ru); su -= su.mean()
    best, bofs = -1e9, None
    for k in range(-len(sb) + 300, len(su) - 300, 2):
        a = su[k:] if k >= 0 else su
        b = sb if k >= 0 else sb[-k:]
        n = min(len(a), len(b))
        if n < 300:
            continue
        d = float(np.dot(a[:n], b[:n]) / n)
        if d > best:
            best, bofs = d, gu[max(k, 0)] - gb[max(-k, 0)]
    return bofs


def arming_time_ulog(u):
    vs = next((d for d in u.data_list if d.name == "vehicle_status"), None)
    t = vs.data["timestamp"] / 1e6
    a = vs.data["arming_state"]
    idx = np.argmax(a == 2)
    return t[idx] if a[idx] == 2 else None


def quat_to_rpy(w, x, y, z):
    roll = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    yaw = np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return roll, pitch, yaw


def run(lid, bagname, mixer, label, t_end, fit_window=None, kM_scale=1.0,
        phi_fixed_deg=None, phi_linear=None, thrust_scale=None, gyro=True,
        align="auto", fit_flap=False):
    import os
    outdir = "%s/analysis/log_%d" % (BASE, lid)
    os.makedirs(outdir, exist_ok=True)
    bagdir = "%s/bags/%s" % (BASE, bagname)

    u = ULog("%s/ulogs/log_%d.ulg" % (BASE, lid),
             message_name_filter_list=["actuator_motors", "vehicle_attitude",
                                       "vehicle_angular_velocity", "vehicle_status"])
    get = lambda n: next((d for d in u.data_list if d.name == n), None)
    am, att, av = get("actuator_motors"), get("vehicle_attitude"), get("vehicle_angular_velocity")

    ta = am.data["timestamp"] / 1e6
    rot = np.stack([am.data["control[%d]" % i] for i in range(4)], 1)
    ok = np.isfinite(rot).all(1)
    ta, rot = ta[ok], rot[ok]
    t_arm_u = ta[np.argmax(rot.mean(1) > 0.1)]
    tq = att.data["timestamp"] / 1e6
    q = np.stack([att.data["q[%d]" % i] for i in range(4)], 1)
    offset, method = bag_to_ulog_offset(bagdir, ta, rot)
    if align == "roll" or (align == "auto" and method in ("first-actuator anchor", "none")):
        ro = roll_signature_offset(bagdir, tq, q)
        if ro is not None:
            offset, method = ro, "roll signature"
    print("  align: %s (offset %s)" % (method, "%.2f" % offset if offset is not None else "-"))
    tw = av.data["timestamp"] / 1e6
    w = np.stack([av.data["xyz[%d]" % i] for i in range(3)], 1)

    # grid relative to arming
    g = np.arange(0.0, t_end, 0.01) + t_arm_u
    U = np.stack([np.interp(g, ta, rot[:, i]) for i in range(4)], 1)
    Q = np.stack([np.interp(g, tq, q[:, i]) for i in range(4)], 1)
    W = np.stack([np.interp(g, tw, w[:, i]) for i in range(3)], 1)
    Wd = np.gradient(W, 0.01, axis=0)
    kern = np.ones(15) / 15
    Wd = np.stack([np.convolve(Wd[:, i], kern, "same") for i in range(3)], 1)
    roll, pitch, yaw = quat_to_rpy(Q[:, 0], Q[:, 1], Q[:, 2], Q[:, 3])

    # tilt from bag tilt_vel integration (commanded), aligned via arming
    tvt, tvv = bag_series(bagdir, "/tilt_vel", TiltVel, lambda m: m.value)
    tvv = [v * math.pi / 8.0 for v in tvv]   # normalized [-1,1] -> rad/s
    phi = np.full(len(g), math.radians(85.0))
    if len(tvt) and offset is not None:
        tb = tvt + offset                    # into ulog clock
        order = np.argsort(tb)
        tb, vv = tb[order], np.array(tvv)[order]
        # integrate
        ph = np.zeros(len(tb))
        cur = math.radians(85.0)
        for i in range(len(tb)):
            if i > 0:
                cur = float(np.clip(cur + vv[i-1] * (tb[i] - tb[i-1]), 0.0, math.radians(85.0)))
            ph[i] = cur
        phi = np.interp(g, tb, ph)

    if phi_linear is not None:
        # Best-fit tilt motion from the phi/tau grid search (slopes-nearest-
        # unity objective): the arm was still swinging toward fly through the
        # flight. Consistent with video (75 deg from drive ~ mid-sweep).
        s_, e_ = phi_linear
        fw = fit_window if fit_window else (0.0, t_end)
        fr = np.clip((grel_for_phi := (g - t_arm_u) - fw[0]) / max(fw[1]-fw[0], 1e-6), 0, 1)
        phi = np.radians(s_ + (e_ - s_) * fr)
    elif phi_fixed_deg is not None:
        # Video evidence (2026-08-18): operator footage shows the arm at
        # ~75 deg at the log_417 departure; the commanded-velocity integral
        # is unreliable (bang-bang commands, worm-gear tracking unknown).
        phi = np.full(len(g), math.radians(phi_fixed_deg))

    # mocap pose (aligned the same way)
    pt, pv = bag_series(bagdir, "/atmo/groundtruth_odom", Odometry,
                        lambda m: (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
    pose = np.full((len(g), 3), np.nan)
    if len(pt) and offset is not None:
        pb = pt + offset
        P = np.array(pv)
        for i in range(3):
            pose[:, i] = np.interp(g, pb, P[:, i])

    # motor response lag: first-order EMA, tau = 0.15 s (physical spin-up lag
    # on top of the adapter's own command filter)
    # SQUARED THRUST MODEL (validated on the log_417 pre-pull lift fit,
    # vz r=0.83): rotor SPEED follows the command with a 0.15 s EMA, and
    # thrust goes as speed^2 with kT_eff = 59 N at u=1 (the linear
    # kT = 28.15 is its near-hover tangent). The S matrix consumes thrust
    # per unit kT, so the effective input is (59/28.15) * EMA(u)^2.
    MOTOR_TAU = 0.15
    KT_EFF_SQ = 59.0
    # PX4 kill physically stops the motors even while the node keeps
    # publishing: zero the commands from the first post-spinup collapse so
    # the model matches what the ESCs actually received.
    grel = g - t_arm_u
    thr = U.mean(1)
    hi = int(np.argmax(thr > 0.5)) if (thr > 0.5).any() else 0
    kill_i = None
    if (thr[hi:] < 0.02).any():
        kill_i = hi + int(np.argmax(thr[hi:] < 0.02))
        U = U.copy()
        U[kill_i:] = 0.0
    t_kill = grel[kill_i] if kill_i is not None else None

    # ---- hoist (safety-pilot tether pull) detection -----------------------
    # The pull is the only thing that can carry the airframe upward while the
    # rotors are dead, and it dominates every axis once it lands: past this
    # instant the vehicle is on a string, not flying, so nothing after it is
    # evidence about the model. Rule: first sustained (>=0.15 s) mocap climb
    # faster than HOIST_VZ while the commanded thrust cannot hold hover.
    HOIST_VZ, HOIST_THR = 0.8, 0.40
    t_hoist = None
    if np.isfinite(pose[:, 2]).sum() > 50:
        z = pose[:, 2].copy()
        okz = np.isfinite(z)
        z = np.interp(grel, grel[okz], z[okz])
        vz = np.convolve(np.gradient(z, 0.01), np.ones(25) / 25, "same")
        cand = (vz > HOIST_VZ) & (thr < HOIST_THR) & (grel > grel[hi])
        runlen = np.convolve(cand.astype(float), np.ones(15), "same")
        if (runlen >= 15).any():
            t_hoist = grel[int(np.argmax(runlen >= 15)) - 7]

    # Cut at whichever event comes first. Past the kill the motors are dead
    # (and these logs re-arm inside the same ulog -- log_415 spins up again at
    # 1.45 s, log_417 at 2.76 s -- so uncut windows silently splice two flights
    # together); past the hoist the airframe is on a string. KILL_TAIL keeps
    # just enough to see the collapse itself.
    KILL_TAIL = 0.5
    ends = [t_end]
    if t_kill is not None:
        ends.append(t_kill + KILL_TAIL)
    if t_hoist is not None:
        ends.append(max(t_hoist, 0.2))
    t_cut = min(ends)
    print("  events: kill %s  hoist %s  -> cut at %.2f s" % (
        "%.2f" % t_kill if t_kill is not None else "none",
        "%.2f" % t_hoist if t_hoist is not None else "none", t_cut))

    alpha = 1.0 - math.exp(-0.01 / MOTOR_TAU)
    Sp = np.copy(U)
    for i in range(1, len(Sp)):
        Sp[i] = alpha * U[i] + (1 - alpha) * Sp[i-1]
    if thrust_scale is not None:
        Sp = Sp * np.asarray(thrust_scale, float)   # per-rotor health factor
    Uf = (KT_EFF_SQ / kT) * Sp ** 2
    # Sp IS the rotor ("wheel") speed as a fraction of OMEGA_MAX: the squared
    # thrust model says thrust ~ speed^2, so the lagged command is the speed.
    Spd = np.gradient(Sp, 0.01, axis=0)
    phidot = np.gradient(phi, 0.01)

    # predicted angular acceleration from S(phi), I(phi), Euler coupling
    pred = np.zeros((len(g), 3))
    pred_nogyro = np.zeros((len(g), 3))
    tau_gyro = np.zeros((len(g), 3))
    tau_parts = np.zeros((len(g), 3, 3))   # [i, part(gyro/prec/spin), axis]
    flap = np.zeros((len(g), 2, 3))        # [i, part(damping/precession), axis]
    Ivec = np.zeros((len(g), 3))
    tau_cmd = np.zeros((len(g), 3))    # S(phi) torque from the actual commands
    for i in range(len(g)):
        S = s_torque(phi[i])
        S[:, 2] *= kM_scale
        tau = S.T @ Uf[i]
        tau_cmd[i] = tau
        I = inertia(phi[i])
        euler = np.array([
            (I[1]-I[2]) * W[i, 1] * W[i, 2],
            (I[2]-I[0]) * W[i, 2] * W[i, 0],
            (I[0]-I[1]) * W[i, 0] * W[i, 1],
        ])
        Ivec[i] = I
        flap[i] = np.stack(flap_regressors(Sp[i], phi[i], W[i]))
        gy, pr, sp = rotor_gyro_torque(Sp[i], Spd[i], phi[i], phidot[i], W[i])
        tau_parts[i] = np.stack([gy, pr, sp])
        tau_gyro[i] = gy + pr + sp
        pred_nogyro[i] = (tau + euler) / I
        pred[i] = (tau + euler + (tau_gyro[i] if gyro else 0.0)) / I

    # policy semantic reconstruction (era-correct mixer)
    M = np.vstack((np.ones(4), 0.5*mixer[:, 0], 0.5*mixer[:, 1], 0.5*mixer[:, 2]))
    sem = (np.linalg.inv(M.T) @ U.T).T

    # ---- FC-filtered angular acceleration (cleaner than differentiating)
    wd_fc_src = np.stack([av.data["xyz_derivative[%d]" % i] for i in range(3)], 1)
    WdF = np.stack([np.interp(g, tw, wd_fc_src[:, i]) for i in range(3)], 1)

    # ---- truncate everything at the hoist (nothing past it is evidence)
    keep = grel <= t_cut
    (grel, phi, phidot, roll, pitch, yaw, pose, U, Sp, Spd, W, Wd, WdF,
     pred, pred_nogyro, tau_gyro, tau_parts, flap, Ivec, tau_cmd, sem) = [
        a[keep] for a in (grel, phi, phidot, roll, pitch, yaw, pose, U, Sp,
                          Spd, W, Wd, WdF, pred, pred_nogyro, tau_gyro,
                          tau_parts, flap, Ivec, tau_cmd, sem)]

    # ---- ground contact --------------------------------------------------
    # A wheel or a leg on the floor supplies whatever roll/pitch torque the
    # constraint demands, so ON-GROUND samples carry no information about the
    # airframe's free-flight moments: anything fitted through them (hub
    # stiffness, drag coefficients) is really fitting ground reaction. The
    # floor is the median mocap z over the first 0.2 s after spin-up.
    # 0.15 m of sustained rise: comfortably more than wheel radius + mocap
    # noise, so a sample flagged airborne really has nothing touching the floor.
    # (Absolute z is NOT comparable between runs -- each bag's mocap origin
    # differs -- so everything here is rise above THIS run's own floor.)
    GROUND_Z = 0.15
    airborne = np.zeros(len(grel), bool)
    climb = 0.0
    if np.isfinite(pose[:, 2]).sum() > 50:
        z = pose[:, 2]
        z0 = float(np.nanmedian(z[grel < 0.2]))
        rise = np.nan_to_num(z - z0, nan=-1.0)
        climb = float(rise.max())
        airborne = rise > GROUND_Z
    free = airborne & (grel < (t_kill if t_kill is not None else 1e9))
    print("    airborne: %.0f%% of the cut window (%.2f s), max climb %.2f m; "
          "%.2f s of it under power" % (100.0 * airborne.mean(),
                                        airborne.sum() * 0.01, climb, free.sum() * 0.01)) 

    # ---- fit window (clamped to the cut) ---------------------------------
    fmask = np.ones(len(grel), bool)
    if fit_window is not None:
        fw_hi = min(fit_window[1], t_cut)
        if fw_hi < fit_window[1]:
            print("    (fit window clipped %.2f -> %.2f by the cut)" % (fit_window[1], fw_hi))
        fmask = (grel >= fit_window[0]) & (grel <= fw_hi)

    # ---- hub-moment stiffness fitted from the ROLL residual --------------
    # What K_BETA does the unexplained roll acceleration actually demand? If
    # the answer is a physically plausible stiffness the flapping moment is a
    # live explanation; if it is absurd, it is ruled out the same way the
    # rotor-momentum term was.
    flap_tot = flap.sum(1)
    resid = (WdF[:, 0] - pred[:, 0]) * Ivec[:, 0]        # unexplained roll torque
    reg = flap_tot[:, 0]
    # NEVER fit aerodynamics through the floor, or through dead motors: after
    # the kill the rotors are coasting and the airframe is on the tether.
    kmask = fmask & free
    if kmask.sum() < 20:
        k_fit, k_note = float("nan"), ("not identifiable (%d powered-airborne "
                                       "samples in window)" % kmask.sum())
    else:
        den = float(np.dot(reg[kmask], reg[kmask]))
        k_fit = float(np.dot(reg[kmask], resid[kmask]) / den) if den > 1e-12 else 0.0
        k_note = "from %d powered-airborne samples" % kmask.sum()
    # The prediction uses the FIXED nominal stiffness so runs stay comparable;
    # k_fit is reported as a diagnostic ("what would roll have to demand?").
    k_use = (k_fit if np.isfinite(k_fit) else 0.0) if fit_flap else K_BETA
    pred_flap = pred + (k_use * flap_tot) / Ivec
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))
    rcorr = lambda a, b: float(np.corrcoef(a, b)[0, 1]) if a.std() > 1e-9 else 0.0
    print("    hub moment: K_beta %s (nominal %.2f applied) -> roll "
          "torque %.4f N.m rms, %.2f rad/s2; roll r %.2f -> %.2f"
          % (("%+.2f N.m/rad %s" % (k_fit, k_note)) if np.isfinite(k_fit) else k_note,
             K_BETA, rms(k_use * flap_tot[fmask, 0]),
             rms((k_use * flap_tot[fmask, 0]) / Ivec[fmask, 0]),
             rcorr(pred[fmask, 0], WdF[fmask, 0]),
             rcorr(pred_flap[fmask, 0], WdF[fmask, 0])))
    print("      damping half %.4f N.m rms (adds over 4 rotors) | precession half "
          "%.4f (cancels when balanced)"
          % (rms(k_use * flap[fmask, 0, 0]), rms(k_use * flap[fmask, 1, 0])))
    pred_noflap = pred
    pred = pred_flap
    print("    rotor-gyro roll torque rms: ωxh %.4f  precession %.4f  spin-up %.4f N·m"
          " -> %.3f rad/s² vs measured %.2f"
          % (rms(tau_parts[:, 0, 0]), rms(tau_parts[:, 1, 0]), rms(tau_parts[:, 2, 0]),
             rms(pred_noflap[:, 0] - pred_nogyro[:, 0]), rms(WdF[:, 0])))
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))

    # ---- merged CSV
    hdr = ("t_s,phi_deg,roll_deg,pitch_deg,yaw_deg,pos_x,pos_y,pos_z,"
           "u0,u1,u2,u3,wx,wy,wz,wx_dot,wy_dot,wz_dot,"
           "pred_wx_dot,pred_wy_dot,pred_wz_dot,"
           "pred_nogyro_wx_dot,pred_nogyro_wy_dot,pred_nogyro_wz_dot,"
           "tau_gyro_x,tau_gyro_y,tau_gyro_z,"
           "flap_damp_x,flap_damp_y,flap_damp_z,"
           "flap_prec_x,flap_prec_y,flap_prec_z,"
           "tau_cmd_x,tau_cmd_y,tau_cmd_z,"
           "spd0,spd1,spd2,spd3,sem_lift,sem_roll,sem_pitch,sem_yaw")
    table = np.column_stack([grel, np.degrees(phi), roll, pitch, yaw, pose,
                             U, W, Wd, pred, pred_nogyro, tau_gyro,
                             k_use * flap[:, 0], k_use * flap[:, 1],
                             tau_cmd, Sp, sem])
    np.savetxt("%s/merged.csv" % outdir, table, delimiter=",", header=hdr, comments="")

    def mark(a):
        """Draw the two events every window is defined by."""
        if t_kill is not None and t_kill <= t_cut:
            a.axvline(t_kill, color="#c0392b", lw=1.1, ls=":", zorder=0)
        if t_hoist is not None and t_hoist <= t_cut:
            a.axvline(t_hoist, color="#2c7fb8", lw=1.1, ls=":", zorder=0)

    # ---- figure 1: timeline overlay
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle("%s — Ioannis S(φ) model vs measured IMU" % label, fontsize=13)
    axes[0].plot(grel, np.degrees(phi), color=COL["aux"], lw=2)
    axes[0].set_ylabel("tilt φ [deg]")
    mark(axes[0])
    for _t, _c, _lab in ((t_kill, "#c0392b", "kill"), (t_hoist, "#2c7fb8", "hoist")):
        if _t is not None and _t <= t_cut:
            axes[0].text(_t, 0.06, " %s %.2fs" % (_lab, _t), color=_c, fontsize=8,
                         transform=axes[0].get_xaxis_transform())
    axes[0].text(0.01, 0.85, "φ integrated from commanded /tilt_vel (85° start)",
                 transform=axes[0].transAxes, fontsize=8, color="#666")
    for ax_i, (axis, name) in enumerate(zip(range(3), ("roll", "pitch", "yaw"))):
        a = axes[ax_i+1]
        a.plot(grel, Wd[:, axis], color="#444", lw=1.2, label="measured")
        a.plot(grel, pred[:, axis], color=COL[name], lw=1.6, label="S(φ) + gyro" if gyro else "S(φ) predicted")
        if gyro:
            a.plot(grel, pred_nogyro[:, axis], color=COL[name], lw=1.0,
                   ls="--", alpha=0.55, label="S(φ) alone")
        mark(a)
        a.set_ylabel("%s\n[rad/s²]" % name)
        a.legend(loc="upper right", fontsize=8, frameon=False)
        a.grid(alpha=0.25, lw=0.5)
    axes[-1].set_xlabel("time since arming [s]")
    fig.tight_layout()
    fig.savefig("%s/timeline.png" % outdir, dpi=150)
    plt.close(fig)

    # ---- figure 2: scatter with fit
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    fig.suptitle("%s — predicted vs measured angular acceleration (fit window %s)"
                 % (label, str(fit_window)), fontsize=12)
    for axis, name in zip(range(3), ("roll", "pitch", "yaw")):
        a = axes[axis]
        p, m = pred[fmask, axis], Wd[fmask, axis]
        a.scatter(p, m, s=4, alpha=0.4, color=COL[name], edgecolors="none")
        pc, mc = p - p.mean(), m - m.mean()
        slope = float(np.dot(pc, mc) / max(np.dot(pc, pc), 1e-9))
        r = float(np.corrcoef(p, m)[0, 1]) if p.std() > 1e-9 else 0.0
        lim = max(np.abs(p).max(), np.abs(m).max(), 1e-3)
        a.plot([-lim, lim], [-lim, lim], color="#999", lw=1, ls="--", label="1:1")
        a.plot([-lim, lim], [-lim*slope, lim*slope], color="#333", lw=1.2,
               label="fit %.2f, r=%.2f" % (slope, r))
        a.set_title(name)
        a.set_xlabel("predicted [rad/s²]")
        if axis == 0:
            a.set_ylabel("measured [rad/s²]")
        a.legend(fontsize=8, frameon=False)
        a.grid(alpha=0.25, lw=0.5)
        pn = pred_nogyro[fmask, axis]
        pnc = pn - pn.mean()
        s0 = float(np.dot(pnc, mc) / max(np.dot(pnc, pnc), 1e-9))
        r0 = float(np.corrcoef(pn, m)[0, 1]) if pn.std() > 1e-9 else 0.0
        print("    %-5s: slope %5.2f  r %5.2f   (no gyro: slope %5.2f  r %5.2f)"
              % (name, slope, r, s0, r0))
    fig.tight_layout()
    fig.savefig("%s/scatter.png" % outdir, dpi=150)
    plt.close(fig)

    # ---- figure 3: what the policy was thinking
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle("%s — reconstructed policy actions vs vehicle state" % label, fontsize=13)
    axes[0].plot(grel, sem[:, 0], color=COL["aux"], lw=1.5, label="lift")
    axes[0].set_ylabel("lift [0..1]")
    axes[0].legend(fontsize=8, frameon=False)
    for nm, idx in (("roll", 1), ("pitch", 2), ("yaw", 3)):
        axes[1].plot(grel, sem[:, idx], color=COL[nm], lw=1.3, label=nm)
    axes[1].set_ylabel("attitude cmds [-1..1]")
    axes[1].legend(fontsize=8, frameon=False, ncol=3)
    axes[2].plot(grel, roll, color=COL["roll"], lw=1.3, label="roll")
    axes[2].plot(grel, pitch, color=COL["pitch"], lw=1.3, label="pitch")
    axes[2].plot(grel, yaw, color=COL["yaw"], lw=1.0, alpha=0.8, label="yaw")
    axes[2].set_ylabel("attitude [deg]")
    axes[2].set_xlabel("time since arming [s]")
    axes[2].legend(fontsize=8, frameon=False, ncol=3)
    for a in axes:
        a.grid(alpha=0.25, lw=0.5)
        mark(a)
    fig.tight_layout()
    fig.savefig("%s/policy.png" % outdir, dpi=150)
    plt.close(fig)

    # ---- figure 4: FC-filtered accel + rate domain
    Wpred = np.zeros_like(pred)
    Wpred[0] = W[0]
    for i in range(1, len(grel)):
        Wpred[i] = Wpred[i-1] + pred[i-1] * 0.01
    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex=True)
    fig.suptitle("%s — FC-filtered accel (left), integrated rates (right)" % label, fontsize=13)
    for axi, (name2, col2) in enumerate(zip(("roll", "pitch", "yaw"),
                                            (COL["roll"], COL["pitch"], COL["yaw"]))):
        a2 = axs[axi][0]
        a2.plot(grel, WdF[:, axi], color="#333", lw=1.4, label="FC ang. accel")
        a2.plot(grel, pred[:, axi], color=col2, lw=1.5,
                label="S(phi) + gyro" if gyro else "S(phi) predicted")
        if gyro:
            a2.plot(grel, pred_nogyro[:, axi], color=col2, lw=1.0, ls="--",
                    alpha=0.55, label="S(phi) alone")
        mark(a2)
        a2.set_ylabel("%s accel [rad/s2]" % name2)
        a2.legend(fontsize=8, frameon=False)
        a2.grid(alpha=0.25, lw=0.5)
        b2 = axs[axi][1]
        b2.plot(grel, W[:, axi], color="#333", lw=1.8, label="measured rate")
        b2.plot(grel, Wpred[:, axi], color=col2, lw=1.6, ls="--", label="integrated prediction")
        mark(b2)
        b2.set_ylabel("%s rate [rad/s]" % name2)
        b2.legend(fontsize=8, frameon=False)
        b2.grid(alpha=0.25, lw=0.5)
    axs[2][0].set_xlabel("time since spinup [s]")
    axs[2][1].set_xlabel("time since spinup [s]")
    fig.tight_layout()
    fig.savefig("%s/model_clean.png" % outdir, dpi=150)
    plt.close(fig)

    # ---- figure 5: rotor ("wheel") speeds and the gyroscopic roll term
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle("%s — rotor speed, rotor-momentum and hub-moment torques" % label, fontsize=13)
    for i in range(4):
        axs[0].plot(grel, Sp[:, i] * OMEGA_MAX, lw=1.3, label="rotor%d" % i)
    axs[0].set_ylabel("rotor speed\n[rad/s]")
    axs[0].legend(fontsize=8, frameon=False, ncol=4)
    axs[0].text(0.01, 0.05, "speed = EMA(u, τ=%.2fs) × Ω_max=%.0f rad/s (Ω_max unmeasured; "
                "gyro torque scales linearly with it)" % (MOTOR_TAU, OMEGA_MAX),
                transform=axs[0].transAxes, fontsize=7.5, color="#666")
    for pi, (nm, st) in enumerate((("-ω×h (gyroscopic)", "-"),
                                   ("geometric precession (φ̇)", "--"),
                                   ("spin-up reaction (Ω̇)", ":"))):
        axs[1].plot(grel, tau_parts[:, pi, 0], color=COL["roll"], lw=1.3, ls=st, label=nm)
    axs[1].plot(grel, tau_gyro[:, 0], color="#111", lw=1.0, alpha=0.7, label="total")
    axs[1].set_ylabel("ROLL torque from\nrotor momentum [N·m]")
    axs[1].legend(fontsize=8, frameon=False, ncol=2)
    for pi, (nm, st) in enumerate((("hub damping half (adds ×4)", "-"),
                                   ("hub precession half (cancels if balanced)", "--"))):
        axs[1].plot(grel, k_use * flap[:, pi, 0], lw=1.4, ls=st,
                    color="#c2185b", alpha=0.9 if pi == 0 else 0.6, label=nm)
    axs[1].legend(fontsize=7.5, frameon=False, ncol=2)
    axs[2].plot(grel, WdF[:, 0], color="#333", lw=1.4, label="measured roll accel (FC)")
    axs[2].plot(grel, pred_nogyro[:, 0], color=COL["roll"], lw=1.1, ls="--",
                alpha=0.6, label="S(φ) alone")
    axs[2].plot(grel, pred_noflap[:, 0], color=COL["roll"], lw=1.2, alpha=0.7,
                label="S(φ) + rotor momentum")
    axs[2].plot(grel, pred[:, 0], color="#c2185b", lw=1.7,
                label="+ hub moment (K_β=%+.2f N·m/rad)" % k_use)
    axs[2].set_ylabel("roll accel\n[rad/s²]")
    axs[2].set_xlabel("time since spinup [s]")
    axs[2].legend(fontsize=8, frameon=False)
    for a in axs:
        a.grid(alpha=0.25, lw=0.5)
        mark(a)
    fig.tight_layout()
    fig.savefig("%s/wheel_gyro.png" % outdir, dpi=150)
    plt.close(fig)

    # ---- figure 6: MIXER SIGN PROOF --------------------------------------
    # CONTROL EFFECTIVENESS, not correlation. The earlier version correlated
    # the policy's roll command against the TOTAL roll torque, which sums all
    # four channels -- so it was mostly measuring the lift ramp, and a command
    # that never crosses zero makes such a correlation meaningless anyway.
    #
    # The question is a derivative: if the policy raises its roll command by
    # one unit, which way does roll torque move? That is
    #     B[k,a] = d tau_a / d cmd_k = sum_i S[i,a] * dT_i/du_i * du_i/dcmd_k
    # with du_i/dcmd_k = M[k,i] (the DEPLOYED mixer) and, for the squared
    # thrust model, dT_i/du_i = 2 (kT_eff/kT) * speed_i.
    #
    # B's diagonal must be POSITIVE on every axis: commanding +roll must make
    # +roll torque. That is the whole definition of a correct sign table. The
    # value depends only on the mixer, the geometry and the operating point --
    # not on what the policy chose to do, not on vehicle motion, and not on
    # ground contact.
    Bdiag = np.zeros((len(grel), 3))
    Bfull = np.zeros((len(grel), 3, 3))
    for i in range(len(grel)):
        S = s_torque(phi[i])
        S[:, 2] *= kM_scale
        dTdu = 2.0 * (KT_EFF_SQ / kT) * Sp[i]          # per-motor thrust slope
        # M rows are [lift, roll, pitch, yaw] -> motors; drop the lift row
        B = np.array([[float(np.sum(S[:, a] * dTdu * M[k, :]))
                       for a in range(3)] for k in range(1, 4)])
        Bfull[i] = B
        Bdiag[i] = np.diag(B)

    print("    CONTROL EFFECTIVENESS  dτ/dcmd  [N·m per unit command]:")
    verdicts = {}
    live = Sp.mean(1) > 0.2                      # only where the motors are up
    vm = fmask & live
    if vm.sum() < 5:
        vm = live if live.any() else np.ones(len(grel), bool)
    for a, name in enumerate(("roll", "pitch", "yaw")):
        d = Bdiag[vm, a]
        frac_neg = float((d < 0).mean())
        med = float(np.median(d))
        AUTHORITY = 1.0
        if abs(med) < AUTHORITY:
            v = "no authority"
        elif frac_neg > 0.99:
            v = "INVERTED (negative at every powered sample)"
        elif frac_neg < 0.01:
            v = "correct"
        else:
            v = "sign changes (%.0f%% negative)" % (100 * frac_neg)
        verdicts[name] = (med, frac_neg, v)
        print("      %-5s median %+8.2f   range %+7.2f..%+7.2f   -> %s"
              % (name, med, d.min(), d.max(), v))

    # Model-free cross-check: B above is derived from Ioannis' S(phi), so on
    # the only run with powered-airborne data ask the IMU directly -- when the
    # policy pushed roll one way, which way did the airframe actually
    # accelerate? Cross-axis torque is not removed, so this is a corroboration,
    # not a substitute for B.
    if free.sum() >= 20:
        for a, (k, name) in enumerate(((1, "roll"), (2, "pitch"), (3, "yaw"))):
            c, w_ = sem[free, k], WdF[free, a]
            print("      [IMU, model-free, %d powered-airborne samples] %-5s: "
                  "corr(cmd, measured accel) %+.2f" % (free.sum(), name, rcorr(c, w_)))

    # The three B_diag(t) traces are near-copies of one another BY
    # CONSTRUCTION, not by coincidence and not from smoothing: with mixer
    # entries +/-1 and four similar rotor speeds,
    #     B[a,a](t) ~ arm_a(phi) * sum_i speed_i(t)
    # -- one shared time-varying factor (total rotor speed, itself built from
    # 9 Hz actuator_motors interpolated + EMA'd, so its wiggles are not real
    # bandwidth) times a per-axis GEOMETRIC arm. The figure therefore plots
    # the two factors separately: the verdict lives entirely in the arm.
    B_raw = np.zeros((len(ta), 3))          # at the raw 9 Hz samples, no EMA
    tr = ta - t_arm_u
    rmask = (tr >= 0) & (tr <= t_cut)
    phi_raw = np.interp(ta[rmask], grel + t_arm_u, phi)
    ur = rot[rmask]
    if thrust_scale is not None:
        ur = ur * np.asarray(thrust_scale, float)
    for j, (ph, uu) in enumerate(zip(phi_raw, ur)):
        S = s_torque(ph)
        S[:, 2] *= kM_scale
        dTdu = 2.0 * (KT_EFF_SQ / kT) * uu
        B_raw[j] = [float(np.sum(S[:, a] * dTdu * M[k + 1, :])) for k, a in
                    ((0, 0), (1, 1), (2, 2))]
    B_raw = B_raw[:int(rmask.sum())]
    tr = tr[rmask]

    arm = np.zeros((len(grel), 3))
    for i in range(len(grel)):
        S = s_torque(phi[i])
        S[:, 2] *= kM_scale
        arm[i] = [float(np.mean(S[:, a] * M[a + 1, :])) for a in range(3)]

    fig, axs = plt.subplots(3, 1, figsize=(11, 10),
                            gridspec_kw={"height_ratios": [1, 1.4, 0.9]}, sharex=True)
    fig.suptitle("%s — control effectiveness = shared speed factor × per-axis arm"
                 % label, fontsize=13)
    ax = axs[0]
    ax.plot(grel, Sp.sum(1), color="#666", lw=1.6,
            label="Σ rotor speeds (EMA of 9 Hz commands — smoothed by construction)")
    ax.plot(tr, ur.sum(1), "o", ms=3, color="#333", alpha=0.6,
            label="raw 9 Hz actuator samples")
    ax.set_ylabel("shared factor\nΣ speed [frac]")
    ax.legend(fontsize=8, frameon=False)
    ax.grid(alpha=0.25, lw=0.5)
    mark(ax)
    ax = axs[1]
    for a, name in enumerate(("roll", "pitch", "yaw")):
        ax.plot(grel, Bdiag[:, a], color=COL[name], lw=1.7,
                label="%s  (median %+.1f — %s)" % (name, verdicts[name][0],
                                                   verdicts[name][2]))
        ax.plot(tr, B_raw[:, a], "o", ms=3, color=COL[name], alpha=0.55)
    ax.axhline(0, color="#111", lw=1.1)
    ax.set_ylabel("∂τ/∂cmd [N·m]\n(dots = raw samples, no EMA)")
    ax.legend(fontsize=8, frameon=False)
    ax.grid(alpha=0.25, lw=0.5)
    mark(ax)
    ax = axs[2]
    for a, name in enumerate(("roll", "pitch", "yaw")):
        ax.plot(grel, arm[:, a], color=COL[name], lw=1.7, label=name)
    ax.axhline(0, color="#111", lw=1.1)
    ax.set_ylabel("per-axis arm(φ)\n[N·m per unit kT·u]")
    ax.set_xlabel("time since spinup [s]")
    ax.text(0.01, 0.05, "the sign verdict lives HERE: geometry × deployed mixer, "
            "independent of rotor speed and of any smoothing",
            transform=ax.transAxes, fontsize=8, color="#666")
    ax.legend(fontsize=8, frameon=False, ncol=3)
    ax.grid(alpha=0.25, lw=0.5)
    mark(ax)
    fig.tight_layout()
    fig.savefig("%s/mixer_proof.png" % outdir, dpi=150)
    plt.close(fig)

    print("%s: wrote merged.csv + 6 figures (phi %.0f..%.0f deg in window)"
          % (label, np.degrees(phi).min(), np.degrees(phi).max()))


if __name__ == "__main__":
    run(413, "atmo_policy_20260818_200125", OLD_MIX, "log_413 yaw runaway (pre-flip, on ground)", 8.0, fit_window=(0.0, 6.5))
    run(414, "atmo_policy_20260818_200413", OLD_MIX, "log_414 pre-flip flight", 18.0, fit_window=(9.0, 17.0))
    # Operator account (2026-08-19): three flights. #1 = 413 yaw runaway,
    # #2 = 415 which STARTED FROM FLY CONFIG (phi = 0, not the drive-config
    # phi=85 assumed all night) and had a ROLL event, #3 = 417 which started
    # from DRIVE and rolled at phi ~ 15 deg from fly.
    run(415, "atmo_policy_20260818_202503", NEW_MIX,
        "log_415 flight 2 (from FLY config, roll event)", 6.0,
        fit_window=(0.0, 1.2), phi_fixed_deg=0.0)
    # Flight #2b (operator, 2026-08-19): between 415 and 417, tilt never moved,
    # held at 85 deg (drive). ulog 416 + bag 204231 -- the pair nobody had
    # touched: it sits between 202503 and 205315 in both orderings.
    run(416, "atmo_policy_20260818_204231", NEW_MIX,
        "log_416 flight 2b (tilt frozen at 85 deg = drive)", 8.0,
        fit_window=(0.0, 3.0), phi_fixed_deg=85.0)
    run(417, "atmo_policy_20260818_205315", NEW_MIX,
        "log_417 flight 3 (from DRIVE, sweep 85->15 deg from fly, rolled at ~15)", 7.0,
        fit_window=(0.0, 1.68), phi_linear=(85.0, 15.0),
        # front-right rotor0 makes ~50% thrust after the 415 carabiner strike
        # (HANDOFF section 6) -> speed factor sqrt(0.5). This is also what makes the
        # gyro term non-zero: it breaks the four-rotor momentum cancellation.
        thrust_scale=(0.707, 1.0, 1.0, 1.0))
