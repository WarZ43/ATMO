#!/usr/bin/env python3
"""Figures for frame_slide.pptx. Regenerate with:  python analysis/frame_figs.py"""
import csv, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from pyulog import ULog

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, "analysis", "frame")
# Deck palette (SURF Draft v2): Cambria/Calibri, navy ink, brick red, teal.
INK, MUTE, RED, BLUE, GREY = "#1B2A3A", "#8A94A0", "#C0392B", "#1C7293", "#DDE3EA"
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Calibri", "Carlito", "DejaVu Sans"],
    "font.size": 8,
    "axes.edgecolor": GREY, "axes.labelcolor": INK, "axes.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "xtick.color": MUTE, "ytick.color": MUTE, "text.color": INK,
    "xtick.major.size": 3, "ytick.major.size": 3, "xtick.major.width": 0.8,
    "ytick.major.width": 0.8, "figure.facecolor": "white",
    "savefig.facecolor": "white", "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})


def load():
    rows = list(csv.DictReader(open(os.path.join(BASE, "analysis/log_417/replay.csv"))))
    tr = np.array([float(r["t_ulog"]) for r in rows])
    cmd = np.array([[float(r["raw_roll"]), float(r["raw_pitch"])] for r in rows])
    u = ULog(os.path.join(BASE, "ulogs/log_417.ulg"))
    d = {x.name: x for x in u.data_list}
    am = d["actuator_motors"].data
    ta = np.array(am["timestamp"]) * 1e-6
    thr = np.stack([am["control[%d]" % i] for i in range(4)], 1).mean(1)
    t_spin = ta[np.argmax(thr > 0.1)]
    hi = int(np.argmax(thr > 0.5))
    t_kill = ta[hi + int(np.argmax(thr[hi:] < 0.02))]
    va = d["vehicle_attitude"].data
    tv = np.array(va["timestamp"]) * 1e-6
    q = np.stack([va["q[%d]" % i] for i in range(4)], 1)
    roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]),
                                 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
    av = d["vehicle_angular_velocity"].data
    tw = np.array(av["timestamp"]) * 1e-6
    dw = np.stack([av["xyz_derivative[%d]" % i] for i in range(3)], 1)
    return tr, cmd, tv, roll, pitch, tw, dw, t_spin, t_kill


def fig_roll_feedback():
    tr, cmd, tv, roll, pitch, tw, dw, t_spin, t_kill = load()
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(5.6, 2.35),
                                  gridspec_kw={"width_ratios": [1.6, 0.95], "wspace": 0.95})
    m = (tr >= t_spin - 0.1) & (tr <= t_kill)
    ax.plot(tr[m] - t_spin, cmd[m, 0], color=BLUE, lw=1.8, label="policy roll command")
    ax.set_ylim(-0.55, 0.55); ax.set_ylabel("roll command", color=BLUE)
    ax.tick_params(axis="y", colors=BLUE)
    ax.axhline(0, color=GREY, lw=0.8, zorder=0)
    axr = ax.twinx()
    mv = (tv >= t_spin - 0.1) & (tv <= t_kill)
    axr.plot(tv[mv] - t_spin, roll[mv], color=RED, lw=1.8, label="measured roll")
    axr.set_ylim(-160, 40)
    axr.set_ylabel("roll angle [deg]", color=RED, fontsize=7.2, labelpad=1)
    axr.tick_params(axis="y", colors=RED); axr.spines["top"].set_visible(False)
    ax.set_xlabel("seconds from spin-up"); ax.spines["top"].set_visible(False)
    ax.annotate("command rises\n+0.15 → +0.37", xy=(1.30, 0.37), xytext=(0.28, 0.40),
                fontsize=7.5, color=BLUE,
                arrowprops=dict(arrowstyle="->", color=BLUE, lw=0.9))
    axr.annotate("roll runs −5° → −100°", xy=(1.45, -70), xytext=(0.15, -105),
                 fontsize=7.5, color=RED,
                 arrowprops=dict(arrowstyle="->", color=RED, lw=0.9))
    ax.set_title("The policy pushed harder as it went", fontsize=8.5, color=INK, pad=4)

    D = np.stack([np.interp(tr, tw, dw[:, k]) for k in range(3)], 1)
    mm = (tr >= t_spin) & (tr <= t_kill)
    ax2.scatter(cmd[mm, 0], D[mm, 0], s=9, color=RED, alpha=0.55, linewidths=0)
    xs = np.linspace(cmd[mm, 0].min(), cmd[mm, 0].max(), 10)
    sl = np.dot(cmd[mm, 0], D[mm, 0]) / np.dot(cmd[mm, 0], cmd[mm, 0])
    ax2.plot(xs, sl * xs, color=INK, lw=1.3)
    ax2.axhline(0, color=GREY, lw=0.8); ax2.axvline(0, color=GREY, lw=0.8)
    ax2.set_xlabel("roll command")
    ax2.set_ylabel("roll accel [rad/s\u00b2]", fontsize=7.2, labelpad=1)
    ax2.set_title("+cmd → −accel", fontsize=8.5, color=INK, pad=4)
    ax2.text(0.96, 0.95, "slope %.1f\nr = \u22120.60" % sl, transform=ax2.transAxes,
             fontsize=7.5, color=INK, va="top", ha="right")
    for a in (ax2,):
        a.spines["top"].set_visible(False); a.spines["right"].set_visible(False)
    fig.subplots_adjust(left=0.095, right=0.895, top=0.86, bottom=0.19)
    fig.savefig(os.path.join(OUT, "roll_feedback.png"), dpi=260)
    plt.close(fig)


def _frame_glyph(ax, cx, title, sub, flip_x, flip_z, colour):
    """A boxed frame card: axis glyph on top, three lines of label beneath."""
    from matplotlib.patches import FancyBboxPatch
    ax.add_patch(FancyBboxPatch((cx - 1.45, -1.02), 2.90, 2.04,
                                boxstyle="round,pad=0.02,rounding_size=0.10",
                                fc="#F8F9FA", ec=GREY, lw=1.0))
    gy, L = 0.56, 0.33
    ax.add_patch(FancyArrowPatch((cx, gy), (cx + (-L if flip_x else L), gy),
                                 arrowstyle="-|>", mutation_scale=9, color=colour, lw=1.7))
    ax.add_patch(FancyArrowPatch((cx, gy), (cx, gy + (-L if flip_z else L)),
                                 arrowstyle="-|>", mutation_scale=9, color=colour, lw=1.7))
    ax.text(cx + (-L - 0.12 if flip_x else L + 0.12), gy, "x", fontsize=7.5, color=colour,
            va="center", ha="right" if flip_x else "left")
    ax.text(cx, gy + (-L - 0.14 if flip_z else L + 0.14), "z", fontsize=7.5, color=colour,
            ha="center", va="top" if flip_z else "bottom")
    ax.text(cx, -0.24, title, fontsize=8.2, color=INK, ha="center", weight="bold")
    ax.text(cx, -0.56, sub[0], fontsize=8.0, color=colour, ha="center")
    ax.text(cx, -0.86, sub[1], fontsize=7.0, color=MUTE, ha="center")


def fig_frames():
    fig, ax = plt.subplots(figsize=(5.6, 1.62))
    ax.set_xlim(0, 11.0); ax.set_ylim(-1.72, 1.15); ax.axis("off")
    _frame_glyph(ax, 1.60, "training expects", ("Rx(\u03c0)", "z-up, x forward"),
                 False, False, INK)
    _frame_glyph(ax, 5.50, "mocap delivers", ("Ry(\u03c0)", "rig faces backwards"),
                 True, False, RED)
    _frame_glyph(ax, 9.40, "policy received", ("Rz(\u03c0)", "z-DOWN"),
                 True, True, RED)
    for x0, x1, txt in ((3.08, 4.02, "\u2460 rigid body\nmounted 180\u00b0 out"),
                        (6.98, 7.92, "\u2461 PX4 conversion\non non-PX4 data")):
        ax.add_patch(FancyArrowPatch((x0, 0.56), (x1, 0.56), arrowstyle="-|>",
                                     mutation_scale=12, color=RED, lw=1.6))
        ax.text((x0 + x1) / 2, -1.14, txt, fontsize=7.2, color=RED, ha="center", va="top")
    fig.subplots_adjust(left=0.005, right=0.995, top=0.99, bottom=0.01)
    fig.savefig(os.path.join(OUT, "frames.png"), dpi=260)
    plt.close(fig)


def fig_mount():
    fig, ax = plt.subplots(figsize=(2.75, 1.72))
    runs = ["414", "415", "417"]
    ang = [179.9, 179.1, 178.6]
    res = [1.0, 1.8, 3.2]
    y = np.arange(3)
    ax.barh(y, ang, color=RED, height=0.5, alpha=0.85)
    ax.errorbar(ang, y, xerr=res, fmt="none", ecolor=INK, elinewidth=1.1, capsize=2.5)
    ax.axvline(180, color=INK, lw=1.2, ls="--")

    ax.set_yticks(y); ax.set_yticklabels(["log_" + r for r in runs], fontsize=7.5)
    ax.set_xlim(170, 185); ax.set_xlabel("mocap body vs FC body [deg about y]", fontsize=7.5)
    ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)

    fig.tight_layout(pad=0.35)
    fig.savefig(os.path.join(OUT, "mount.png"), dpi=260)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig_roll_feedback(); fig_frames(); fig_mount()
    print("wrote", OUT)
