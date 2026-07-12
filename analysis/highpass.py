#!/usr/bin/env python3
"""Highpass experiment: does removing low-frequency motion separate impact from noise?

We now know 56% of the flight signal is sub-2 Hz bulk motion (maneuvering,
throttle), not vibration. A highpass should strip that away while KEEPING a
sharp impact edge (a fast rise is high-frequency). This asks whether that
improves crash detectability -- i.e. how far the surviving impact peak sits
above the surviving motor-vibration floor -- for each cutoff.

Writes 06_highpass_on_noise.png (flight spectrum per cutoff) and
07_highpass_on_impact.png (impact waveform per cutoff), and prints a
detectability ratio (impact peak / flight-noise RMS) per cutoff.
"""
import os

import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

from fft_analysis import (load_mag_csv, load_mag_bin, FLIGHT_CSV, DROP, FS,
                          INK, MUTED, GRID, style_ax, HERE, REPO)

HP_CUTS = [5, 20, 50, 100]                       # highpass cutoffs (Hz)
HP_COLORS = {5: "#86b6ef", 20: "#3987e5", 50: "#1c5cab", 100: "#0d366b"}


def highpass(x, fc):
    b, a = signal.butter(4, fc / (FS / 2.0), btype="highpass")
    return signal.filtfilt(b, a, x)


def welch(x, nperseg):
    f, p = signal.welch(x - x.mean(), fs=FS, nperseg=nperseg,
                        window="hann", detrend="constant")
    return f[1:], p[1:]


def main():
    print("loading + reconstructing ...")
    _, fmag = load_mag_csv(FLIGHT_CSV)                     # entire flight (motors on)
    it, imag = load_mag_bin(DROP, 4.63 - 0.05, 4.63 + 0.05)   # around impact

    fac = fmag - fmag.mean()
    raw_rms = fac.std()
    iac = imag - imag.mean()
    raw_peak = np.abs(iac).max()
    print(f"\nraw: flight noise RMS={raw_rms:.2f} g, impact peak={raw_peak:.1f} g, "
          f"ratio={raw_peak/raw_rms:.1f}")

    # ---- 06: highpass on the flight noise (spectrum) ----
    ff, fp = welch(fmag, 4096)
    fig, ax = plt.subplots(figsize=(9, 4.6))
    style_ax(ax)
    ax.semilogx(ff, 10 * np.log10(fp / fp.max()), color=INK, lw=1.8, label="raw (no filter)")
    resid = {}
    for fc in HP_CUTS:
        y = highpass(fmag, fc)
        _, p = welch(y, 4096)
        ax.semilogx(ff, 10 * np.log10(p / fp.max()), color=HP_COLORS[fc], lw=1.6,
                    label=f"{fc} Hz highpass")
        resid[fc] = (y - y.mean()).std()
        ax.axvline(fc, color=HP_COLORS[fc], lw=0.8, ls=":", alpha=0.7)
    ax.set_xlim(ff[0], 800)
    ax.set_ylim(-70, 3)
    ax.set_xlabel("frequency (Hz)", color=INK)
    ax.set_ylabel("spectral power (dB, rel. raw peak)", color=INK)
    ax.xaxis.set_major_formatter(ScalarFormatter())
    ax.set_xticks([1, 5, 10, 50, 100, 200, 500])
    ax.legend(loc="lower right", frameon=False, fontsize=8.5, labelcolor=INK)
    note = "   noise RMS remaining:  " + "  ".join(
        f"{fc}Hz→{resid[fc]:.2f}g" for fc in HP_CUTS) + f"  (raw {raw_rms:.2f}g)"
    ax.set_title("Highpass on the flight noise" + note, color=INK, fontsize=10, loc="left", pad=10)
    fig.text(0.5, -0.02,
             "Highpass kills the sub-2 Hz bulk-motion hump (left side collapses) but the motor "
             "vibration tones above ~100 Hz survive — that residual is the floor a crash must beat.",
             ha="center", color=MUTED, fontsize=8.5)
    fig.savefig(os.path.join(HERE, "06_highpass_on_noise.png"), dpi=150,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote analysis/06_highpass_on_noise.png")

    # ---- 07: highpass on the impact (time domain) ----
    tms = (it - it[0]) * 1000 - 50
    fig, ax = plt.subplots(figsize=(9, 4.6))
    style_ax(ax)
    ax.plot(tms, iac, color=INK, lw=2.0, label="raw (gravity removed)")
    peak = {}
    for fc in HP_CUTS:
        y = highpass(imag, fc)
        ax.plot(tms, y, color=HP_COLORS[fc], lw=1.6, label=f"{fc} Hz highpass")
        peak[fc] = np.abs(y).max()
    ax.set_xlim(-40, 40)
    ax.set_xlabel("time relative to impact (ms)", color=INK)
    ax.set_ylabel("accel |mag|, AC (g)", color=INK)
    ax.legend(loc="upper right", frameon=False, fontsize=8.5, labelcolor=INK)
    ax.set_title("Highpass on the drop-test impact  —  the sharp edge survives",
                 color=INK, fontsize=11, loc="left", pad=10)
    fig.text(0.5, -0.02,
             "Unlike a lowpass, a highpass keeps the fast rising edge (that's high-frequency) and "
             "just removes the slow baseline — the peak a jerk/amplitude detector needs stays intact.",
             ha="center", color=MUTED, fontsize=8.5)
    fig.savefig(os.path.join(HERE, "07_highpass_on_impact.png"), dpi=150,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote analysis/07_highpass_on_impact.png")

    print("\ndetectability = impact peak / flight-noise RMS (higher = crash stands out more)")
    print(f"  raw (DC..800Hz)     peak={raw_peak:5.1f}g  noise={raw_rms:.2f}g  ratio={raw_peak/raw_rms:4.1f}")
    for fc in HP_CUTS:
        print(f"  highpass {fc:3d} Hz     peak={peak[fc]:5.1f}g  noise={resid[fc]:.2f}g  "
              f"ratio={peak[fc]/resid[fc]:4.1f}")


if __name__ == "__main__":
    main()
