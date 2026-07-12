#!/usr/bin/env python3
"""Replay blackbox CSV logs through the CrashDetector, sample by sample.

    python3 simulate.py                # run both known test flights (pass/fail)
    python3 simulate.py some_log.csv   # replay any blackbox CSV, print transitions
    python3 simulate.py --sweep        # threshold-margin sweep over both flights

Streams rows exactly as the real-time loop would -- no lookahead.
"""
import csv
import os
import sys

from crash_detector import CrashDetector, STATE_NAMES, CRASHED

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CRASH_LOG = os.path.join(REPO, "outputs", "Flight-4-Impact-Hard-Surface.01.csv")
NOCRASH_LOG = os.path.join(
    REPO, "tests", "flight-no-crash", "Impact Testing - Flight 3.02.csv")

FIELDS = ("time (us)", "accSmooth[0] (g)", "accSmooth[1] (g)", "accSmooth[2] (g)",
          "gyroADC[0] (deg/s)", "gyroADC[1] (deg/s)", "gyroADC[2] (deg/s)")


def replay(path, **kw):
    """Yields (t_rel_s, old_state, new_state) transitions; returns detector."""
    det = CrashDetector(**kw)
    transitions = []
    t0 = None
    with open(path) as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        idx = [header.index(name) for name in FIELDS]
        for row in reader:
            if len(row) < len(header):
                continue
            try:
                t, ax, ay, az, gx, gy, gz = (float(row[i]) for i in idx)
            except ValueError:
                continue
            if t0 is None:
                t0 = t
            old = det.state
            new = det.update(t, ax, ay, az, gx, gy, gz)
            if new != old:
                transitions.append(((t - t0) / 1e6, old, new))
    return det, transitions


def report(path):
    det, transitions = replay(path)
    print(f"\n{os.path.basename(path)}")
    for t, old, new in transitions:
        print(f"  t={t:8.3f}s  {STATE_NAMES[old]} -> {STATE_NAMES[new]}")
    if not transitions:
        print("  (no state changes)")
    print(f"  final state: {STATE_NAMES[det.state]}")
    return det


def sweep():
    """Detection + false-positive check across the whole threshold grid."""
    print(f"{'J_trig':>7} {'G_crash':>8} {'N':>2} | {'crash det':>9} "
          f"{'latency ms':>10} | {'landing FP':>10}")
    failures = 0
    for j in (300, 400, 500, 600, 800):
        for g in (150, 200, 300, 400, 500):
            for n in (1, 2, 3):
                detc, tc = replay(CRASH_LOG, j_trig=j, g_crash=g, n_trig=n)
                detn, _ = replay(NOCRASH_LOG, j_trig=j, g_crash=g, n_trig=n)
                ok = detc.state == CRASHED
                # latency from first contact (t=17.948 s into the crash log)
                lat = f"{(tc[-1][0] - 17.948) * 1e3:.0f}" if ok else ""
                fp = detn.state == CRASHED
                flag = "  <-- FAILS" if (not ok or fp) else ""
                failures += bool(flag)
                print(f"{j:>7} {g:>8} {n:>2} | {str(ok):>9} {lat:>10} | "
                      f"{str(fp):>10}{flag}")
    print(f"\n{failures} failing combinations")
    sys.exit(1 if failures else 0)


def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == "--sweep":
            sweep()
        report(sys.argv[1])
        return

    ok = True
    det = report(CRASH_LOG)
    if det.state == CRASHED:
        print("  PASS: crash detected")
    else:
        ok = False
        print("  FAIL: crash NOT detected")

    det = report(NOCRASH_LOG)
    if det.state == CRASHED:
        ok = False
        print("  FAIL: false positive on no-crash flight")
    else:
        print("  PASS: no false positive (landing correctly rejected)")

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
