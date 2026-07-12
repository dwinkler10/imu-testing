#!/usr/bin/env python3
"""Spectrogram (frequency vs. time) showing why a lowpass can't isolate an impact.

Two panels on a shared log-frequency axis:
  left  -- entire flight (motors on): steady vibration = horizontal bands
  right -- drop test (motors off): the real impact = a vertical broadband streak

A lowpass keeps everything BELOW a horizontal cutoff line. The vibration bands
sit below it (so they survive the filter) and the impact streak crosses it
top-to-bottom (so cutting its upper part just removes the sharp edge while its
low-frequency content stays tangled with the vibration). No horizontal cut
separates the vertical streak from the horizontal bands -- that is the whole
point. Writes 05_spectrogram.png.
"""
import os

import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

from fft_analysis import (load_mag_csv, load_mag_bin, FLIGHT_CSV, DROP,
                          FS, INK, MUTED, HERE, REPO)

CUTLINE = 50            # illustrative lowpass cutoff to draw


def spec_db(x, nperseg, hop):
    # PSD density (g^2/Hz) is comparable across window sizes, so the two panels
    # can use different time/frequency resolution yet share one colour scale.
    f, t, S = signal.spectrogram(x - x.mean(), fs=FS, window="hann",
                                 nperseg=nperseg, noverlap=nperseg - hop,
                                 detrend=False, scaling="density")
    return f[1:], t, 10 * np.log10(S[1:] + 1e-12)   # drop DC row, to dB


def main():
    print("loading + reconstructing ...")
    _, fmag = load_mag_csv(FLIGHT_CSV)                 # entire flight (motors on)
    _, dmag = load_mag_bin(DROP, 0.0, None)            # full drop test (motors off)

    # flight: long window -> fine frequency resolution to resolve the low bands
    ff, ft, fS = spec_db(fmag, nperseg=1024, hop=256)
    # drop: short-ish window -> good time localization while still reaching the
    # low band (6 Hz bins) so the streak visibly spans into the vibration range
    df, dt, dS = spec_db(dmag, nperseg=256, hop=16)

    # shared absolute colour scale so the two panels are directly comparable
    vmax = max(fS.max(), dS.max())
    vmin = vmax - 55

    fig, (axl, axr) = plt.subplots(
        1, 2, figsize=(12, 5), sharey=True,
        gridspec_kw=dict(width_ratios=[1.7, 1], wspace=0.06))

    m = axl.pcolormesh(ft, ff, fS, cmap="magma", vmin=vmin, vmax=vmax, shading="auto")
    axr.pcolormesh(dt, df, dS, cmap="magma", vmin=vmin, vmax=vmax, shading="auto")

    for ax in (axl, axr):
        ax.set_yscale("log")
        ax.set_ylim(2, 800)
        ax.yaxis.set_major_formatter(ScalarFormatter())
        ax.set_yticks([2, 5, 10, 50, 100, 200, 500])
        ax.set_xlabel("time (s)", color=INK)
        ax.tick_params(colors=MUTED, labelsize=9)
        # the illustrative lowpass cutoff: a filter keeps everything below this
        ax.axhline(CUTLINE, color="#37c2d4", lw=1.6, ls="--")
    axl.set_ylabel("frequency (Hz)", color=INK)
    axr.set_xlim(4.2, 5.1)          # zoom onto the impact so the streak is visible

    # impact marker on the drop panel
    axr.annotate("impact — one instant,\nevery frequency at once",
                 xy=(4.63, 250), xytext=(4.66, 430), color="white", fontsize=9,
                 ha="left", arrowprops=dict(arrowstyle="->", color="white", lw=1.2))
    axl.text(0.98, 0.04, "horizontal bands = steady motor vibration",
             transform=axl.transAxes, ha="right", va="bottom", color="white",
             fontsize=9.5)
    axl.text(0.5, CUTLINE * 1.16, "below a 50 Hz lowpass = KEPT  (vibration survives the filter)",
             transform=axl.get_yaxis_transform(), color="#37c2d4", fontsize=8.5,
             va="bottom", ha="center")

    axl.set_title("Entire flight — motors ON (vibration, no crash)",
                  color=INK, fontsize=11, loc="left")
    axr.set_title("Drop test — motors OFF (real impact, zoomed)",
                  color=INK, fontsize=11, loc="left")

    cbar = fig.colorbar(m, ax=(axl, axr), fraction=0.03, pad=0.02)
    cbar.set_label("spectral power (dB, shared scale)", color=INK)
    cbar.ax.tick_params(colors=MUTED, labelsize=8)

    fig.suptitle("A lowpass is a horizontal cut; the impact is a vertical streak — "
                 "no cut isolates it", color=INK, fontsize=12.5, x=0.02, ha="left")
    fig.text(0.5, -0.03,
             "The impact deposits energy across the whole frequency column at one instant, "
             "overlapping the exact low bands the vibration occupies continuously. "
             "Any horizontal cutoff either keeps both or removes both.",
             ha="center", color=MUTED, fontsize=8.5)

    out = os.path.join(HERE, "05_spectrogram.png")
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", os.path.relpath(out, REPO))


if __name__ == "__main__":
    main()
