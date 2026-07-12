#!/usr/bin/env python3
"""FFT / lowpass-filter analysis of BMI270 recordings.

Produces four PNGs in this directory:
  01_flight_noise_fft.png   -- vibration spectrum of powered flight (motors on)
  02_lowpass_on_noise.png   -- what candidate lowpass cutoffs remove from that noise
  03_lowpass_on_impact.png  -- what those same cutoffs do to a real high-g impact edge
  04_flight_vs_drop_fft.png -- noisy (motors on) vs quiet (motors off) spectra

IMPORTANT framing: the flight capture contains NO crash -- it is motor and
airframe vibration plus maneuvering only. The only real high-g transient we
have is the drop-test impact (motors off). So "noisy vs not noisy" is
flight-vibration vs drop-test-quiet, and the impact plot shows a filter acting
on the one genuine impact edge in the dataset.

Both files come from the old direct-register logger, which oversampled ~3x with
monotonic sensortime. Every trace here is first reconstructed onto a true,
uniform 1600 Hz grid by interpolating against sensortime before any FFT.
"""
import os
import struct

import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
# Entire flight, exported from the trimmed mcap (flight3-cut2) to CSV.
FLIGHT_CSV = os.path.join(REPO, "outputs/bin/test3/flight3-cut2.csv")
DROP = os.path.join(REPO, "tests/drop-test/imu_20260707_105743.bin")

FS = 1600.0                 # true BMI270 ODR -> reconstruction grid
TICK_S = 39.0625e-6         # sensortime LSB
ACC_G = 16 / 32768.0        # +/-16g range -> g per LSB
CUTOFFS = [50, 100, 200, 400]   # candidate 4th-order Butterworth lowpass cutoffs (Hz)

# --- palette (dataviz reference; blue↔orange CVD-safe, blue ordinal ramp) ---
INK = "#0b0b0b"
MUTED = "#52514e"
GRID = "#e6e6e2"
CUT_COLORS = {50: "#86b6ef", 100: "#3987e5", 200: "#1c5cab", 400: "#0d366b"}
FLIGHT_C = "#eb6834"        # orange  -- noisy, motors on
DROP_C = "#2a78d6"          # blue    -- quiet, motors off

_REC = struct.Struct("<QIhhhhhh")   # IMULOG01: host_ns, sensortime, ax..gz


def _reconstruct(mag, st, t0, t1):
    """Shared: sensortime -> uniform FS grid, collapsing oversampled duplicates.

    st is the raw 24-bit sensortime column; mag is accel |magnitude| in g.
    t0/t1 in seconds from the first sample; pass None for the full span.
    """
    st = st.astype(np.float64)
    st = np.unwrap(st * (2 * np.pi / 2 ** 24)) / (2 * np.pi) * 2 ** 24
    ts = (st - st[0]) * TICK_S
    uts, inv = np.unique(ts, return_inverse=True)   # the old logger oversampled ~3x
    umag = np.zeros_like(uts)
    cnt = np.zeros_like(uts)
    np.add.at(umag, inv, mag)
    np.add.at(cnt, inv, 1.0)
    umag /= cnt
    if t0 is None:
        t0 = 0.0
    if t1 is None:
        t1 = ts[-1]
    grid = np.arange(t0, t1, 1.0 / FS)
    return grid, np.interp(grid, uts, umag)


def load_mag_bin(path, t0, t1):
    """accel |magnitude| in g from an IMULOG01 .bin, on a uniform FS grid."""
    with open(path, "rb") as f:
        body = f.read()[16:]
    n = len(body) // _REC.size
    dt = np.dtype([("ht", "<u8"), ("st", "<u4"),
                   ("ax", "<i2"), ("ay", "<i2"), ("az", "<i2"),
                   ("gx", "<i2"), ("gy", "<i2"), ("gz", "<i2")])
    a = np.frombuffer(body[:n * _REC.size], dtype=dt)
    mag = np.sqrt(a["ax"].astype(np.float64) ** 2
                  + a["ay"].astype(np.float64) ** 2
                  + a["az"].astype(np.float64) ** 2) * ACC_G
    return _reconstruct(mag, a["st"], t0, t1)


def load_mag_csv(path, t0=None, t1=None):
    """accel |magnitude| in g from a converter CSV (ax_g/ay_g/az_g already scaled)."""
    a = np.genfromtxt(path, delimiter=",", names=True)
    mag = np.sqrt(a["ax_g"] ** 2 + a["ay_g"] ** 2 + a["az_g"] ** 2)
    return _reconstruct(mag, a["sensortime"], t0, t1)


def welch_db(x, nperseg):
    """Welch PSD -> (freqs, dB relative to peak) of the AC (gravity-removed) part."""
    f, p = signal.welch(x - x.mean(), fs=FS, nperseg=nperseg,
                        window="hann", detrend="constant")
    p = p[1:]                          # drop DC bin
    f = f[1:]
    db = 10 * np.log10(p / p.max())
    return f, db, p


def lowpass(x, fc):
    b, a = signal.butter(4, fc / (FS / 2.0))
    return signal.filtfilt(b, a, x)


def style_ax(ax):
    ax.set_facecolor("white")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, which="both", color=GRID, lw=0.7)
    ax.set_axisbelow(True)


def save(fig, name):
    out = os.path.join(HERE, name)
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", os.path.relpath(out, REPO))


# ---------------------------------------------------------------------------
def main():
    print("loading + reconstructing to uniform 1600 Hz grid ...")
    ft, fmag = load_mag_csv(FLIGHT_CSV)                    # ENTIRE flight (cut2)
    qt, qmag = load_mag_bin(DROP, 0.2, 2.0)               # quiet held phase (motors off)
    it, imag = load_mag_bin(DROP, 4.63 - 0.05, 4.63 + 0.05)  # around impact
    print(f"  flight span {ft[-1]:.1f}s ({len(ft)} samples on grid)")

    ff, fdb, fp = welch_db(fmag, nperseg=4096)
    qf, qdb, qp = welch_db(qmag, nperseg=1024)

    # cumulative fraction of vibration energy vs frequency (flight)
    cum = np.cumsum(fp) / fp.sum()
    def frac_below(hz):
        return cum[np.searchsorted(ff, hz) - 1]

    # ---- 01: flight vibration FFT (two stacked panels, shared x -- no dual axis) ----
    # peak within the vibration band; below ~2 Hz is maneuvering/throttle, not vibration
    vib = ff >= 2.0
    fpk = ff[vib][np.argmax(fdb[vib])]
    sub2 = frac_below(2.0) * 100
    fig, (ax, axc) = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True,
                                  gridspec_kw=dict(height_ratios=[2.4, 1], hspace=0.12))
    style_ax(ax); style_ax(axc)
    ax.semilogx(ff, fdb, color=FLIGHT_C, lw=1.8)
    ax.set_ylim(-55, 3)
    ax.set_ylabel("spectral power\n(dB, rel. peak)", color=INK)
    ax.annotate(f"dominant vibration ~{fpk:.1f} Hz",
                xy=(fpk, fdb[vib].max()), xytext=(fpk * 1.7, -10),
                color=INK, fontsize=9,
                arrowprops=dict(arrowstyle="->", color=MUTED, lw=1))
    ax.set_title("Flight vibration spectrum  —  motors on, no crash in this data "
                 "(entire flight, 230 s)",
                 color=INK, fontsize=11, loc="left", pad=10)
    axc.semilogx(ff, cum * 100, color=DROP_C, lw=1.8)
    axc.set_ylim(0, 100)
    axc.set_xlim(ff[0], 800)
    axc.set_ylabel("cumulative\nenergy (%)", color=INK)
    axc.set_xlabel("frequency (Hz)", color=INK)
    axc.xaxis.set_major_formatter(ScalarFormatter())
    axc.set_xticks([1, 5, 10, 50, 100, 200, 500])
    for hz in (50, 100):
        y = frac_below(hz) * 100
        axc.plot([hz], [y], "o", color=DROP_C, ms=6)
        axc.annotate(f"{y:.0f}% < {hz} Hz", xy=(hz, y), xytext=(hz * 1.12, y - 22),
                     color=DROP_C, fontsize=8.5,
                     arrowprops=dict(arrowstyle="->", color=DROP_C, lw=0.8))
    fig.text(0.5, -0.02,
             f"Energy piles up at the bottom of the band — {sub2:.0f}% below 2 Hz is maneuvering/throttle, "
             "the rest a low vibration harmonic stack. All low-frequency; no crash in this data.",
             ha="center", color=MUTED, fontsize=8.5)
    save(fig, "01_flight_noise_fft.png")

    # ---- 02: lowpass cutoffs applied to the flight noise ----
    fig, ax = plt.subplots(figsize=(9, 4.6))
    style_ax(ax)
    ax.semilogx(ff, fdb, color=INK, lw=1.8, label="raw (no filter)")
    removed = {}
    ac = fmag - fmag.mean()
    raw_rms = ac.std()
    for fc in CUTOFFS:
        y = lowpass(fmag, fc)
        _, db_f, _ = welch_db(y, nperseg=4096)
        ax.semilogx(ff, db_f, color=CUT_COLORS[fc], lw=1.6,
                    label=f"{fc} Hz lowpass")
        removed[fc] = 100 * (1 - (y - y.mean()).std() / raw_rms)
    ax.set_xlim(ff[0], 800)
    ax.set_ylim(-70, 3)
    ax.set_xlabel("frequency (Hz)", color=INK)
    ax.set_ylabel("spectral power (dB, rel. raw peak)", color=INK)
    ax.xaxis.set_major_formatter(ScalarFormatter())
    ax.set_xticks([1, 5, 10, 50, 100, 200, 500])
    for fc in CUTOFFS:
        ax.axvline(fc, color=CUT_COLORS[fc], lw=0.8, ls=":", alpha=0.7)
    ax.legend(loc="upper right", frameon=False, fontsize=8.5, labelcolor=INK)
    note = "  vibration RMS removed:  " + "   ".join(
        f"{fc}Hz → {removed[fc]:.0f}%" for fc in CUTOFFS)
    ax.set_title("Lowpass cutoffs on the flight noise" + note,
                 color=INK, fontsize=10.5, loc="left", pad=10)
    fig.text(0.5, -0.02,
             "Even a 50 Hz cut removes only a slice of the vibration — most of the energy "
             "sits below every candidate cutoff, so lowpassing can't clean this up.",
             ha="center", color=MUTED, fontsize=8.5)
    save(fig, "02_lowpass_on_noise.png")

    # ---- 03: lowpass cutoffs on the real impact edge ----
    tms = (it - it[0]) * 1000 - 50        # center ~0 at the impact
    fig, ax = plt.subplots(figsize=(9, 4.6))
    style_ax(ax)
    ax.plot(tms, imag, color=INK, lw=2.0, label="raw (1600 Hz)")
    for fc in CUTOFFS:
        ax.plot(tms, lowpass(imag, fc), color=CUT_COLORS[fc], lw=1.6,
                label=f"{fc} Hz lowpass")
    ax.axhline(16, color="#e34948", lw=1, ls="--", alpha=0.7)
    ax.text(tms[0], 16.6, "16 g accel rail", color="#e34948", fontsize=8)
    ax.set_xlim(-40, 40)
    ax.set_xlabel("time relative to impact (ms)", color=INK)
    ax.set_ylabel("accel |magnitude| (g)", color=INK)
    ax.legend(loc="upper right", frameon=False, fontsize=8.5, labelcolor=INK)
    ax.set_title("Same cutoffs on the drop-test impact  —  the only real high-g edge we have",
                 color=INK, fontsize=11, loc="left", pad=10)
    fig.text(0.5, -0.02,
             "The impact is a sub-millisecond edge. Each lower cutoff flattens and smears the "
             "peak — the sharp rise a jerk detector would trigger on is what the filter discards.",
             ha="center", color=MUTED, fontsize=8.5)
    save(fig, "03_lowpass_on_impact.png")

    # ---- 04: noisy vs not-noisy spectra ----
    fig, ax = plt.subplots(figsize=(9, 4.6))
    style_ax(ax)
    # amplitude spectral density in g/sqrt(Hz), absolute (not peak-normalized)
    fF, fP = signal.welch(fmag - fmag.mean(), fs=FS, nperseg=4096, window="hann", detrend="constant")
    qF, qP = signal.welch(qmag - qmag.mean(), fs=FS, nperseg=1024, window="hann", detrend="constant")
    ax.loglog(fF[1:], np.sqrt(fP[1:]), color=FLIGHT_C, lw=1.8,
              label=f"flight, motors ON (noisy)  RMS ≈ {(fmag-fmag.mean()).std():.2f} g")
    ax.loglog(qF[1:], np.sqrt(qP[1:]), color=DROP_C, lw=1.8,
              label=f"drop test, motors OFF (quiet)  RMS ≈ {(qmag-qmag.mean()).std():.2f} g")
    ax.set_xlim(1, 800)
    ax.set_xlabel("frequency (Hz)", color=INK)
    ax.set_ylabel("amplitude spectral density (g/√Hz)", color=INK)
    ax.xaxis.set_major_formatter(ScalarFormatter())
    ax.set_xticks([1, 5, 10, 50, 100, 200, 500])
    ax.legend(loc="lower left", frameon=False, fontsize=8.5, labelcolor=INK)
    ax.set_title("Noisy vs not-noisy: how far motor vibration sits above the quiet floor",
                 color=INK, fontsize=11, loc="left", pad=10)
    fig.text(0.5, -0.02,
             "Motors-on vibration is ~1–2 orders of magnitude above the motors-off baseline "
             "across the whole low band — that gap is the vibration a crash detector must survive.",
             ha="center", color=MUTED, fontsize=8.5)
    save(fig, "04_flight_vs_drop_fft.png")

    print("\nsummary")
    print(f"  flight vibration AC RMS  : {(fmag-fmag.mean()).std():.3f} g (motors on)")
    print(f"  drop quiet AC RMS        : {(qmag-qmag.mean()).std():.3f} g (motors off)")
    print(f"  dominant flight freq     : {fpk:.1f} Hz")
    print(f"  energy < 50 Hz / <100 Hz : {frac_below(50)*100:.0f}% / {frac_below(100)*100:.0f}%")
    print("  vibration RMS removed by lowpass: "
          + ", ".join(f"{fc}Hz={removed[fc]:.0f}%" for fc in CUTOFFS))


if __name__ == "__main__":
    main()
