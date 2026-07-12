#!/usr/bin/env python3
"""Regenerate the analysis figures (01-03) from the two test flights.

Requires numpy + matplotlib. Figures land next to this script.
"""
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from crash_detector import CrashDetector, MONITOR, IMPACT, CRASHED

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CRASH_LOG = os.path.join(REPO, "outputs", "Flight-4-Impact-Hard-Surface.01.csv")
NOCRASH_LOG = os.path.join(
    REPO, "tests", "flight-no-crash", "Impact Testing - Flight 3.02.csv")

# reference palette, light mode
SURFACE, INK, SEC, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, BASE = "#e1e0d9", "#c3c2b7"
BLUE, RED, VIOLET = "#2a78d6", "#e34948", "#4a3aa7"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica Neue", "Arial", "DejaVu Sans"],
    "text.color": INK, "axes.labelcolor": SEC,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.edgecolor": BASE, "axes.linewidth": 0.8,
    "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.grid": True, "axes.axisbelow": True,
    "axes.spines.top": False, "axes.spines.right": False,
    "font.size": 10,
})

COLS = {'t': 'time (us)',
        'ax': 'accSmooth[0] (g)', 'ay': 'accSmooth[1] (g)', 'az': 'accSmooth[2] (g)',
        'gx': 'gyroADC[0] (deg/s)', 'gy': 'gyroADC[1] (deg/s)', 'gz': 'gyroADC[2] (deg/s)'}


def load(path):
    with open(path) as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        idx = {k: header.index(name) for k, name in COLS.items()}
        data = {k: [] for k in idx}
        for row in reader:
            if len(row) < len(header):
                continue
            try:
                vals = {k: float(row[i]) for k, i in idx.items()}
            except ValueError:
                continue
            for k, v in vals.items():
                data[k].append(v)
    return {k: np.array(v) for k, v in data.items()}


def derive(d):
    """time (s, from 0), jerk magnitude (g/s), accel magnitude (g), gyro (deg/s)."""
    t = (d['t'] - d['t'][0]) / 1e6
    dt = np.diff(d['t']) / 1e6
    jerk = np.concatenate([[0], np.sqrt(
        (np.diff(d['ax']) / dt) ** 2 +
        (np.diff(d['ay']) / dt) ** 2 +
        (np.diff(d['az']) / dt) ** 2)])
    amag = np.sqrt(d['ax'] ** 2 + d['ay'] ** 2 + d['az'] ** 2)
    gmag = np.sqrt(d['gx'] ** 2 + d['gy'] ** 2 + d['gz'] ** 2)
    return t, jerk, amag, gmag


def states(d):
    det = CrashDetector()
    out = np.empty(len(d['t']), dtype=int)
    for i in range(len(d['t'])):
        out[i] = det.update(d['t'][i], d['ax'][i], d['ay'][i], d['az'][i],
                            d['gx'][i], d['gy'][i], d['gz'][i])
    return out


dc, dn = load(CRASH_LOG), load(NOCRASH_LOG)
tc, jc, ac, gc = derive(dc)
tn, jn, an, gn = derive(dn)

# ------------------------------------------------ 01: full-flight jerk
fig, axes = plt.subplots(2, 1, figsize=(11, 6.2), sharey=True)
fig.subplots_adjust(hspace=0.45, left=0.08, right=0.97, top=0.90, bottom=0.09)
for ax, t, j, color, title, note_t, note, va, dy in [
    (axes[0], tc, jc, RED, "Flight 4 — crash on hard surface",
     17.96, "crash: 1971 g/s\nlog ends 14 ms later", "top", -4),
    (axes[1], tn, jn, BLUE, "Flight 3 — full flight, normal landing",
     222.43, "landing: 611 g/s", "bottom", 14),
]:
    ax.plot(t, j, color=color, lw=0.8)
    ax.set_title(title, loc="left", fontsize=11, color=INK, fontweight="bold")
    ax.set_ylabel("jerk (g/s)")
    ax.set_xlim(0, t[-1] * 1.02)
    ax.axhline(500, color=MUTED, lw=1, ls=(0, (4, 3)))
    ax.text(t[-1] * 0.005, 560, "impact trigger 500 g/s", color=MUTED,
            fontsize=8.5, ha="left")
    ax.annotate(note, xy=(note_t, min(j.max(), 1971)), xytext=(-12, dy),
                textcoords="offset points", ha="right", va=va,
                fontsize=9, color=SEC)
axes[0].set_ylim(0, 2100)
axes[1].set_xlabel("flight time (s)", color=SEC)
fig.suptitle("Jerk |da/dt| over the whole flight — crash vs normal flight+landing",
             x=0.08, ha="left", fontsize=13, fontweight="bold", color=INK)
fig.savefig(os.path.join(HERE, "01_jerk_full_flights.png"), dpi=150)
plt.close(fig)

# ------------------------------------------------ 02: impact zoom grid
fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharey="row")
fig.subplots_adjust(hspace=0.35, wspace=0.10, left=0.08, right=0.97,
                    top=0.85, bottom=0.08)
win_c = (tc >= 17.90) & (tc <= 17.97)
win_n = (tn >= 222.38) & (tn <= 222.55)
t0c, t0n = 17.950, 222.432  # first jerk-trigger crossing in each flight
cols = [("Flight 4 — crash", RED, (tc[win_c] - t0c) * 1e3,
         jc[win_c], ac[win_c], gc[win_c]),
        ("Flight 3 — landing", BLUE, (tn[win_n] - t0n) * 1e3,
         jn[win_n], an[win_n], gn[win_n])]
rows = [("jerk (g/s)", 500, "trigger 500"),
        ("|accel| (g)", None, None),
        ("gyro (deg/s)", 300, "crash confirm 300")]
for ci, (title, color, tt, j, a, g) in enumerate(cols):
    for ri, series in enumerate([j, a, g]):
        ax = axes[ri][ci]
        ax.plot(tt, series, color=color, lw=2, marker="o", ms=3.5,
                markerfacecolor=color, markeredgecolor=SURFACE,
                markeredgewidth=0.6)
        ylabel, thr, thrname = rows[ri]
        if ci == 0:
            ax.set_ylabel(ylabel)
        if thr is not None:
            ax.axhline(thr, color=MUTED, lw=1, ls=(0, (4, 3)))
            if ci == 1:
                ax.text(tt[-1], thr * 1.12, thrname, color=MUTED,
                        fontsize=8.5, ha="right")
        ax.axvline(0, color=BASE, lw=0.8)
        if ri == 0:
            ax.set_title(title, loc="left", fontsize=11, color=INK,
                         fontweight="bold")
        if ri == 2:
            ax.set_xlabel("ms from jerk trigger", color=SEC)
axes[0][0].set_ylim(0, 2100)
axes[1][0].set_ylim(0, 8.2)
axes[2][0].set_ylim(0, 1850)
axes[2][0].annotate("tumble: 1735 deg/s", xy=(14, 1735), xytext=(-6, -14),
                    textcoords="offset points", ha="right", fontsize=9, color=SEC)
axes[2][1].annotate("stays level: max 84 deg/s", xy=(20, 84), xytext=(6, 60),
                    textcoords="offset points", ha="left", fontsize=9, color=SEC)
axes[1][1].annotate("7.07 g — as hard as the crash", xy=(4, 7.07),
                    xytext=(8, 2), textcoords="offset points", fontsize=9,
                    color=SEC)
fig.suptitle("Impact zoom: crash tumbles, landing stays level",
             x=0.08, y=0.97, ha="left", fontsize=13, fontweight="bold", color=INK)
fig.text(0.08, 0.925, "Same accel peak (7.7 vs 7.1 g) — but only the crash pairs "
         "a sustained jerk spike with violent rotation", fontsize=10, color=SEC)
fig.savefig(os.path.join(HERE, "02_impact_zoom.png"), dpi=150)
plt.close(fig)

# ------------------------------------------------ 03: state-machine trace
sc, sn = states(dc), states(dn)
fig, axes = plt.subplots(2, 2, figsize=(11, 6.4), sharey="row",
                         gridspec_kw={"height_ratios": [3, 1]})
fig.subplots_adjust(hspace=0.12, wspace=0.10, left=0.09, right=0.97,
                    top=0.84, bottom=0.10)
for ci, (t, jerk, gmag, st, color, title, t0, t1, tref) in enumerate([
    (tc, jc, gc, sc, RED, "Flight 4 — crash", 17.90, 17.97, 17.950),
    (tn, jn, gn, sn, BLUE, "Flight 3 — landing", 222.38, 222.56, 222.432),
]):
    m = (t >= t0) & (t <= t1)
    tt = (t[m] - tref) * 1e3
    ax = axes[0][ci]
    # series colors are fixed across panels (legend on the left applies to
    # both); the flight color only marks the CRASHED shading below
    ax.plot(tt, jerk[m], color=RED, lw=2, label="jerk (g/s)")
    ax.plot(tt, gmag[m], color=VIOLET, lw=2, label="gyro (deg/s)")
    ax.axhline(500, color=MUTED, lw=1, ls=(0, (4, 3)))
    ax.axhline(300, color=MUTED, lw=1, ls=(0, (1, 2)))
    ax.set_title(title, loc="left", fontsize=11, color=INK, fontweight="bold")
    if ci == 0:
        ax.set_ylabel("jerk / gyro")
        ax.text(tt[0], 545, "J_TRIG 500 g/s", color=MUTED, fontsize=8.5)
        ax.text(tt[0], 190, "G_CRASH 300 deg/s", color=MUTED, fontsize=8.5)
        ax.legend(loc="upper left", frameon=False, fontsize=9,
                  labelcolor=SEC, bbox_to_anchor=(0.0, 0.86))
    ax.set_ylim(0, 2100)
    ax.tick_params(labelbottom=False)

    axs = axes[1][ci]
    axs.step(tt, st[m], where="post", color=INK, lw=2)
    axs.set_ylim(-0.4, 2.4)
    axs.set_yticks([MONITOR, IMPACT, CRASHED])
    axs.set_xlabel("ms from jerk trigger", color=SEC)
    if ci == 0:
        axs.set_yticklabels(["MONITOR", "IMPACT", "CRASHED"], fontsize=8.5)
        axs.set_ylabel("state")
    axs.fill_between(tt, -0.4, 2.4, where=st[m] == IMPACT,
                     step="post", color=MUTED, alpha=0.15, lw=0)
    axs.fill_between(tt, -0.4, 2.4, where=st[m] == CRASHED,
                     step="post", color=color, alpha=0.15, lw=0)
axes[1][0].annotate("CRASHED at +4 ms\n(10 ms before power loss)",
                    xy=(4, 2), xytext=(-8, -6), textcoords="offset points",
                    ha="right", va="top", fontsize=9, color=SEC)
axes[1][1].annotate("window expires -> back to MONITOR",
                    xy=(52, 1), xytext=(4, 10), textcoords="offset points",
                    fontsize=9, color=SEC)
fig.suptitle("Detector state trace at the two impacts", x=0.09, y=0.97,
             ha="left", fontsize=13, fontweight="bold", color=INK)
fig.text(0.09, 0.90, "Both impacts open the IMPACT window; only the crash "
         "confirms with rotation and latches CRASHED", fontsize=10, color=SEC)
fig.savefig(os.path.join(HERE, "03_state_machine_trace.png"), dpi=150)
plt.close(fig)
print("wrote 01_jerk_full_flights.png, 02_impact_zoom.png, 03_state_machine_trace.png")
