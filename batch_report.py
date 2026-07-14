#!/usr/bin/env python3
"""Batch-convert BMI270 .bin recordings and run the crash detector on them.

Two subcommands, both pointed at the same directory:

  convert <dir>   For every *.bin in <dir>, run the convert.py pipeline
                  (IMULOG02..05 -> .csv + Foxglove .mcap, written next to
                  each .bin). Nothing is filtered; values are raw sensor
                  output with the datasheet CAS correction, exactly as
                  convert.py produces them.

  report  <dir>   For every *.csv in <dir> (the convert output), stream it
                  through crash-detection/crash_detector.py and render the
                  same three figures the crash-detection library makes --
                  full-flight jerk, impact zoom, detector state trace --
                  auto-located on whatever the detector flags (or the
                  peak-jerk region if no crash fires). One report folder
                  per recording, named after the file with the extension
                  stripped:  <dir>/<basename>/{01_jerk_full,02_impact_zoom,
                  03_state_trace}.png  plus a crash_report.txt summary.

Typical use:
    python3 batch_report.py convert data/
    python3 batch_report.py report  data/

convert needs the mcap/protobuf deps (as convert.py does); report needs
numpy + matplotlib. The two steps are independent so you can run report on
a directory whose CSVs already exist without the mcap deps installed.
"""
import argparse
import csv
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                                  # convert.py
sys.path.insert(0, os.path.join(HERE, "crash-detection"))  # crash_detector.py

# Sensortime LSB is 39.0625 us (25.6 kHz); used only as a fallback clock.
US_PER_TICK = 1e6 / 25600.0
CORE_COLS = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")


# --------------------------------------------------------------- convert
def convert_dir(path):
    """Run convert.py on every *.bin in `path`."""
    import convert  # imported lazily: only this subcommand needs mcap deps

    bins = sorted(glob.glob(os.path.join(path, "*.bin")))
    if not bins:
        raise SystemExit(f"no .bin files in {path}")
    ok = 0
    for b in bins:
        try:
            convert.main(b)
            ok += 1
        except SystemExit as e:            # bad magic / unknown format
            print(f"skip {os.path.basename(b)}: {e}")
        except Exception as e:             # keep going on a single bad file
            print(f"skip {os.path.basename(b)}: {type(e).__name__}: {e}")
    print(f"\nconverted {ok}/{len(bins)} .bin files in {path}")


# ---------------------------------------------------------------- report
def load_csv(path):
    """Read a convert.py CSV into numpy arrays keyed by CORE_COLS, plus a
    uniform microsecond time base reconstructed from the sample period.

    The per-sample host/sensortime stamps carry sub-period readout jitter;
    feeding that jitter straight into a derivative would swamp the jerk
    signal. The true spacing is one ODR period, so we estimate the period
    (median sensortime delta, host-time fallback) and lay samples on an
    even grid -- the same "difference against the nominal period" the
    logger's docs prescribe."""
    import numpy as np

    with open(path) as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        missing = [c for c in CORE_COLS if c not in header]
        if missing:
            raise ValueError(f"missing columns {missing}")
        idx = {c: header.index(c) for c in CORE_COLS}
        has_st = "sensortime" in header
        has_ht = "host_time_ns" in header
        st_i = header.index("sensortime") if has_st else None
        ht_i = header.index("host_time_ns") if has_ht else None
        cols = {c: [] for c in CORE_COLS}
        st, ht = [], []
        for row in reader:
            if len(row) < len(header):
                continue
            try:
                for c in CORE_COLS:
                    cols[c].append(float(row[idx[c]]))
                if has_st:
                    st.append(int(row[st_i]))
                if has_ht:
                    ht.append(int(row[ht_i]))
            except (ValueError, IndexError):
                continue

    d = {c: np.asarray(v, dtype=float) for c, v in cols.items()}
    n = len(d["ax_g"])
    period_us = _estimate_period_us(np.asarray(st), np.asarray(ht), n)
    d["t_us"] = np.arange(n, dtype=float) * period_us
    d["period_us"] = period_us
    d["n"] = n
    return d


def _estimate_period_us(st, ht, n):
    """Median inter-sample period in microseconds, hardware clock first."""
    import numpy as np

    if st.size > 2:
        ds = np.diff(st) & 0xFFFFFF          # 24-bit sensortime wraps
        ds = ds[(ds > 0) & (ds < 0x800000)]
        if ds.size:
            return float(np.median(ds)) * US_PER_TICK
    if ht.size > 2:
        dh = np.diff(ht) / 1000.0            # ns -> us
        dh = dh[dh > 0]
        if dh.size:
            return float(np.median(dh))
    return 1e6 / 800.0                        # last resort: assume 800 Hz


def derive(d):
    """(t_s, jerk g/s, |accel| g, |gyro| deg/s) on the uniform grid."""
    import numpy as np

    t = d["t_us"] / 1e6
    dt = d["period_us"] / 1e6
    jerk = np.concatenate([[0.0], np.sqrt(
        (np.diff(d["ax_g"]) / dt) ** 2 +
        (np.diff(d["ay_g"]) / dt) ** 2 +
        (np.diff(d["az_g"]) / dt) ** 2)])
    amag = np.sqrt(d["ax_g"] ** 2 + d["ay_g"] ** 2 + d["az_g"] ** 2)
    gmag = np.sqrt(d["gx_dps"] ** 2 + d["gy_dps"] ** 2 + d["gz_dps"] ** 2)
    return t, jerk, amag, gmag


def run_detector(d):
    """Stream the recording through CrashDetector; return (states, crash_t_s)."""
    import numpy as np
    from crash_detector import CrashDetector, CRASHED

    det = CrashDetector()
    n = d["n"]
    states = np.empty(n, dtype=int)
    t_us = d["t_us"]
    ax, ay, az = d["ax_g"], d["ay_g"], d["az_g"]
    gx, gy, gz = d["gx_dps"], d["gy_dps"], d["gz_dps"]
    for i in range(n):
        states[i] = det.update(t_us[i], ax[i], ay[i], az[i],
                               gx[i], gy[i], gz[i])
    crash_t = det.crash_t_us / 1e6 if det.crash_t_us is not None else None
    return states, crash_t


# reference palette / style, copied from crash-detection/make_figures.py
_STYLE = {
    "SURFACE": "#fcfcfb", "INK": "#0b0b0b", "SEC": "#52514e", "MUTED": "#898781",
    "GRID": "#e1e0d9", "BASE": "#c3c2b7",
    "BLUE": "#2a78d6", "RED": "#e34948", "VIOLET": "#4a3aa7",
}
J_TRIG, G_CRASH = 500.0, 300.0            # match CrashDetector defaults


def _apply_style():
    import matplotlib.pyplot as plt
    s = _STYLE
    plt.rcParams.update({
        "figure.facecolor": s["SURFACE"], "axes.facecolor": s["SURFACE"],
        "font.family": "sans-serif",
        "font.sans-serif": ["Helvetica Neue", "Arial", "DejaVu Sans"],
        "text.color": s["INK"], "axes.labelcolor": s["SEC"],
        "xtick.color": s["MUTED"], "ytick.color": s["MUTED"],
        "axes.edgecolor": s["BASE"], "axes.linewidth": 0.8,
        "grid.color": s["GRID"], "grid.linewidth": 0.6,
        "axes.grid": True, "axes.axisbelow": True,
        "axes.spines.top": False, "axes.spines.right": False,
        "font.size": 10,
    })


def make_report(csv_path, out_dir):
    """Render the three figures + a text summary for one recording.
    Returns a one-line status string."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from crash_detector import IMPACT, CRASHED

    s = _STYLE
    d = load_csv(csv_path)
    if d["n"] < 3:
        return f"{os.path.basename(csv_path)}: too few samples ({d['n']}), skipped"
    _apply_style()

    t, jerk, amag, gmag = derive(d)
    states, crash_t = run_detector(d)
    name = os.path.splitext(os.path.basename(csv_path))[0]
    os.makedirs(out_dir, exist_ok=True)

    # centre the zoom on the crash, else on the peak-jerk sample
    center_t = crash_t if crash_t is not None else float(t[int(np.argmax(jerk))])
    half_ms = 35.0
    win = (t >= center_t - half_ms / 1e3) & (t <= center_t + half_ms / 1e3)
    if win.sum() < 3:                          # short file: show all of it
        win = np.ones_like(t, dtype=bool)
    tt = (t[win] - center_t) * 1e3             # ms from centre

    crashed = crash_t is not None
    tag = (f"CRASH at {crash_t:.3f} s" if crashed
           else "no crash detected (peak-jerk region shown)")

    # ----- 01: full-flight jerk -----------------------------------
    fig, ax = plt.subplots(figsize=(11, 3.6))
    fig.subplots_adjust(left=0.08, right=0.97, top=0.82, bottom=0.16)
    ax.plot(t, jerk, color=(s["RED"] if crashed else s["BLUE"]), lw=0.8)
    ax.axhline(J_TRIG, color=s["MUTED"], lw=1, ls=(0, (4, 3)))
    ax.text(t[-1] * 0.005, J_TRIG * 1.05, f"impact trigger {J_TRIG:g} g/s",
            color=s["MUTED"], fontsize=8.5, ha="left")
    if crashed:
        ax.axvline(crash_t, color=s["RED"], lw=1.2, ls=(0, (1, 2)))
        ax.annotate("CRASHED", xy=(crash_t, jerk.max()), xytext=(-8, -4),
                    textcoords="offset points", ha="right", fontsize=9,
                    color=s["SEC"])
    ax.set_xlim(0, t[-1] * 1.02 if t[-1] > 0 else 1)
    ax.set_ylim(0, max(jerk.max() * 1.1, J_TRIG * 1.2))
    ax.set_xlabel("recording time (s)", color=s["SEC"])
    ax.set_ylabel("jerk (g/s)")
    fig.suptitle(f"{name} — jerk |da/dt| over the recording", x=0.08,
                 ha="left", fontsize=12, fontweight="bold", color=s["INK"])
    fig.text(0.08, 0.87, tag, fontsize=9.5, color=s["SEC"])
    fig.savefig(os.path.join(out_dir, "01_jerk_full.png"), dpi=150)
    plt.close(fig)

    # ----- 02: impact zoom (jerk / |accel| / gyro) ----------------
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    fig.subplots_adjust(hspace=0.14, left=0.11, right=0.96, top=0.90, bottom=0.08)
    color = s["RED"] if crashed else s["BLUE"]
    series = [("jerk (g/s)", jerk[win], J_TRIG, "trigger 500 g/s"),
              ("|accel| (g)", amag[win], None, None),
              ("|gyro| (deg/s)", gmag[win], G_CRASH, "crash confirm 300 deg/s")]
    for ax, (ylabel, y, thr, thrname) in zip(axes, series):
        ax.plot(tt, y, color=color, lw=1.6, marker="o", ms=2.5,
                markerfacecolor=color, markeredgecolor=s["SURFACE"],
                markeredgewidth=0.5)
        ax.set_ylabel(ylabel)
        if thr is not None:
            ax.axhline(thr, color=s["MUTED"], lw=1, ls=(0, (4, 3)))
            ax.text(tt[-1], thr * 1.05, thrname, color=s["MUTED"],
                    fontsize=8.5, ha="right")
        ax.axvline(0, color=s["BASE"], lw=0.8)
    axes[-1].set_xlabel(
        f"ms from {'crash' if crashed else 'peak jerk'}", color=s["SEC"])
    fig.suptitle(f"{name} — impact zoom", x=0.11, y=0.965, ha="left",
                 fontsize=12, fontweight="bold", color=s["INK"])
    fig.text(0.11, 0.925, tag, fontsize=9.5, color=s["SEC"])
    fig.savefig(os.path.join(out_dir, "02_impact_zoom.png"), dpi=150)
    plt.close(fig)

    # ----- 03: detector state trace -------------------------------
    fig, (axj, axs) = plt.subplots(2, 1, figsize=(8, 5.6), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1]})
    fig.subplots_adjust(hspace=0.12, left=0.13, right=0.96, top=0.86, bottom=0.12)
    axj.plot(tt, jerk[win], color=s["RED"], lw=1.8, label="jerk (g/s)")
    axj.plot(tt, gmag[win], color=s["VIOLET"], lw=1.8, label="gyro (deg/s)")
    axj.axhline(J_TRIG, color=s["MUTED"], lw=1, ls=(0, (4, 3)))
    axj.axhline(G_CRASH, color=s["MUTED"], lw=1, ls=(0, (1, 2)))
    axj.set_ylabel("jerk / gyro")
    axj.legend(loc="upper right", frameon=False, fontsize=9, labelcolor=s["SEC"])
    st = states[win]
    axs.step(tt, st, where="post", color=s["INK"], lw=2)
    axs.set_ylim(-0.4, 2.4)
    axs.set_yticks([0, 1, 2])
    axs.set_yticklabels(["MONITOR", "IMPACT", "CRASHED"], fontsize=8.5)
    axs.set_ylabel("state")
    axs.set_xlabel(f"ms from {'crash' if crashed else 'peak jerk'}", color=s["SEC"])
    axs.fill_between(tt, -0.4, 2.4, where=st == IMPACT, step="post",
                     color=s["MUTED"], alpha=0.15, lw=0)
    axs.fill_between(tt, -0.4, 2.4, where=st == CRASHED, step="post",
                     color=color, alpha=0.15, lw=0)
    fig.suptitle(f"{name} — detector state trace", x=0.13, y=0.965, ha="left",
                 fontsize=12, fontweight="bold", color=s["INK"])
    fig.text(0.13, 0.915, tag, fontsize=9.5, color=s["SEC"])
    fig.savefig(os.path.join(out_dir, "03_state_trace.png"), dpi=150)
    plt.close(fig)

    # ----- text summary -------------------------------------------
    dur = t[-1] if t.size else 0.0
    with open(os.path.join(out_dir, "crash_report.txt"), "w") as f:
        f.write(f"recording : {name}\n")
        f.write(f"samples   : {d['n']}\n")
        f.write(f"rate      : {1e6 / d['period_us']:.1f} Hz "
                f"({d['period_us']:.2f} us/sample)\n")
        f.write(f"duration  : {dur:.3f} s\n")
        f.write(f"peak jerk : {jerk.max():.0f} g/s (trigger {J_TRIG:g})\n")
        f.write(f"peak accel: {amag.max():.2f} g\n")
        f.write(f"peak gyro : {gmag.max():.0f} deg/s (confirm {G_CRASH:g})\n")
        f.write(f"crash     : {'YES at %.3f s' % crash_t if crashed else 'no'}\n")

    return (f"{name}: {'CRASH @ %.3fs' % crash_t if crashed else 'no crash'}"
            f"  (peak jerk {jerk.max():.0f} g/s, peak gyro {gmag.max():.0f} deg/s)")


def report_dir(path):
    csvs = sorted(glob.glob(os.path.join(path, "*.csv")))
    if not csvs:
        raise SystemExit(f"no .csv files in {path} (run `convert` first)")
    crashes = 0
    for c in csvs:
        base = os.path.splitext(os.path.basename(c))[0]
        out_dir = os.path.join(path, base)
        try:
            line = make_report(c, out_dir)
            print(line)
            if "CRASH @" in line:
                crashes += 1
        except Exception as e:
            print(f"{os.path.basename(c)}: FAILED {type(e).__name__}: {e}")
    print(f"\n{len(csvs)} recordings, {crashes} with a detected crash; "
          f"reports in per-recording folders under {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    pc = sub.add_parser("convert", help="all .bin -> .csv + .mcap in <dir>")
    pc.add_argument("dir")
    pr = sub.add_parser("report", help="run crash detector + figures on all .csv in <dir>")
    pr.add_argument("dir")
    args = ap.parse_args()

    if not os.path.isdir(args.dir):
        raise SystemExit(f"not a directory: {args.dir}")
    if args.cmd == "convert":
        convert_dir(args.dir)
    else:
        report_dir(args.dir)


if __name__ == "__main__":
    main()
