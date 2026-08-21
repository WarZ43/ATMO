#!/usr/bin/env python3
"""Which physical rotor is control[0..3], which way does each spin, and what
the deployed mixer actually did with that.

Everything here is arithmetic on two sources that were written independently of
each other and of the RL stack:

  * ``mav.parm``  -- the CA_ROTORn_{PX,PY,PZ,KM} table the flight controller
    itself allocated with on 2026-08-18.
  * Ioannis' ``mpc/parameters.py`` geometry, read through the torque rows of
    his ``S_func`` (reproduced in ``smatrix_validation.py:s_torque``).

Neither knows about the training spec, so where they agree the answer is not a
convention -- it is the vehicle. Run it:

    python analysis/rotor_identity.py
"""

from __future__ import annotations

import itertools
import re
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parent.parent

# Ioannis' parameters.py, the same constants smatrix_validation.py uses.
kT, kM = 28.15, 0.018
rBAx, rBAy, rBAz = 0.0066, 0.0685, -0.021
rA1x, rA1y, rA1z = 0.16491, 0.13673, -0.069563

# The three mixers, rows = rotor index, columns = (roll, pitch, yaw).
OLD_MIX = np.array(((1, 1, -1), (-1, -1, -1), (-1, 1, 1), (1, -1, 1)), float)
YAW_ONLY = np.array(((1, 1, 1), (-1, -1, 1), (-1, 1, -1), (1, -1, -1)), float)
ROLL_AND_YAW = np.array(((-1, 1, 1), (1, -1, 1), (1, 1, -1), (-1, -1, -1)), float)


def s_torque(phi: float) -> np.ndarray:
    """4x3 torque rows of Ioannis' S_func at tilt phi (rows = u1..u4)."""
    c, s = np.cos(phi), np.sin(phi)
    A = kT * (rA1y + rBAy * c + rBAz * s)
    B1 = kT * (rA1x * c + rBAx * c - kM * s)
    B2 = kT * (rBAx * c - rA1x * c + kM * s)
    C1 = kT * (kM * c + rA1x * s + rBAx * s)
    C2 = kT * (kM * c + rA1x * s - rBAx * s)
    return np.array([(-A, B1, C1), (A, B2, C2), (A, B1, -C1), (-A, B2, -C2)])


def corner(x: float, y: float) -> str:
    """Name a rotor position in PX4's FRD body frame (+x forward, +y right)."""
    return ("front" if x > 0 else "rear") + "-" + ("right" if y > 0 else "left")


def read_ca_table(parm: Path) -> dict[int, dict[str, float]]:
    wanted = re.compile(r"^CA_ROTOR([0-3])_(PX|PY|PZ|KM|CT|AX|AY|AZ)\s+(-?[\d.]+)")
    table: dict[int, dict[str, float]] = {i: {} for i in range(4)}
    for line in parm.read_text().splitlines():
        m = wanted.match(line)
        if m:
            table[int(m.group(1))][m.group(2)] = float(m.group(3))
    return table


def main() -> int:
    ca = read_ca_table(BASE / "mav.parm")
    S0 = s_torque(0.0)

    print("PX4's own allocation table (mav.parm, FRD, thrust axis -z = up)")
    print("  idx   position (m)        corner        KM      spin seen from above")
    for i in range(4):
        r = ca[i]
        # PX4 pairs the rotors by the sign of KM; the pair that shares a sign
        # spins together. Which absolute sense that is comes from Ioannis'
        # yaw column below, not from PX4's sign convention.
        print("  [%d]  (%+.2f, %+.2f, %+.2f)  %-12s  %+.2f   %s"
              % (i, r["PX"], r["PY"], r["PZ"], corner(r["PX"], r["PY"]), r["KM"],
                 "CCW" if r["KM"] > 0 else "CW"))

    print("\nGeometry recovered from Ioannis' S rows, independently")
    print("  In FRD with thrust along -z, tau = r x F gives")
    print("      roll entry = -kT*y,  pitch entry = +kT*x,  yaw entry = +kT*kM*sigma")
    print("  idx   position (m)        corner        yaw sign   spin")
    for i, row in enumerate(S0):
        y = -row[0] / kT
        x = row[1] / kT
        sigma = np.sign(row[2])
        # Positive yaw torque in FRD is nose-right, i.e. clockwise seen from
        # above, which is the reaction to a rotor turning counter-clockwise.
        print("  [%d]  (%+.3f, %+.3f, %+.3f)  %-12s  %+d         %s"
              % (i, x, y, rA1z, corner(x, y), sigma, "CCW" if sigma > 0 else "CW"))

    agree = all(
        np.sign(ca[i]["PX"]) == np.sign(S0[i][1]) and np.sign(ca[i]["PY"]) == np.sign(-S0[i][0])
        and np.sign(ca[i]["KM"]) == np.sign(S0[i][2])
        for i in range(4)
    )
    print("\n  two sources agree on corner and spin pairing for all four:", agree)

    print("\nWhat the mixer has to be")
    print("  A sign table is correct iff every diagonal of d tau / d cmd is")
    print("  positive, which for unit mixer entries means mix = sign(S).")
    required = np.sign(S0)
    for name, mix in (("deployed pre-8/18", OLD_MIX),
                      ("deployed 8/18 (yaw flipped)", YAW_ONLY),
                      ("designed (roll+yaw flipped)", ROLL_AND_YAW)):
        wrong = [axis for axis, k in enumerate("rpy") if not np.array_equal(mix[:, axis], required[:, axis])]
        names = ["roll", "pitch", "yaw"]
        verdict = "correct" if not wrong else "inverted: " + ", ".join(names[a] for a in wrong)
        print("  %-28s %s" % (name, verdict))

    print("\nThe inversion is a rotor RELABELLING, exactly")
    for perm in itertools.permutations(range(4)):
        if np.array_equal(OLD_MIX, ROLL_AND_YAW[list(perm)]):
            print("  deployed mix == correct mix with rows permuted by", perm)
            print("  which pairs:")
            for i, j in enumerate(perm):
                print("     policy rotor %d  ->  physical %s (control[%d])"
                      % (i, corner(ca[j]["PX"], ca[j]["PY"]), j))
            mirrored = all(
                abs(ca[j]["PX"] - ca[i]["PX"]) < 1e-9 and abs(ca[j]["PY"] + ca[i]["PY"]) < 1e-9
                for i, j in enumerate(perm)
            )
            print("  every pair is the same x with y negated (a left-right mirror):", mirrored)
            print("  equivalent fix: keep the old mix, publish rotor i to control[perm[i]] =",
                  list(perm))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
