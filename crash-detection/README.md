# Crash detection

Real-time crash detector derived from the Flight-4 hard-surface crash and the
Flight-3 no-crash (normal landing) blackbox logs. Three states, O(1) per
sample, no sqrt — runs comfortably inside the 800 Hz logger loop and fires
milliseconds before the vehicle loses power.

## What the data shows

| signal at impact | Flight 4 (crash) | Flight 3 (landing) | Flight 3 (aerobatics) |
|---|---|---|---|
| peak accel | 7.66 g | 7.07 g | ~2 g |
| peak jerk | 1971 g/s, sustained >1300 g/s for ~12 ms | 611 g/s, one sample | ≤ 267 g/s |
| gyro within 10 ms | 455 → 1735 deg/s | ≤ 84 deg/s | up to 606 deg/s |

- **Accel alone fails**: a firm landing hits almost the same peak g as the crash.
- **Jerk alone is thin**: 611 vs 1971 g/s separates these two flights, but the
  margin rests on a single landing sample.
- **Rotation is the clean discriminator**: the crash tumbles (gyro explodes
  past 450 deg/s within 6 ms of contact) while the landing stays level
  (< 84 deg/s). Aggressive flight reaches 606 deg/s but never with a jerk
  spike — so the *conjunction* of jerk and rotation identifies a crash.

Timing constraint: the crash log ends 14 ms after first contact (battery
disconnect on impact), so detection must complete within ~10 ms. It does.

![jerk over both flights](01_jerk_full_flights.png)
![impact zoom](02_impact_zoom.png)

## The state machine

```
             jerk > J_TRIG (500 g/s)
  MONITOR ──────────────────────────► IMPACT
     ▲                                  │
     │  window (50 ms) expires          │  ≥ N_TRIG (2) jerk hits in window
     └──────────────────────────────────┤  AND gyro > G_CRASH (300 deg/s)
                                        ▼
                                     CRASHED   (latched — flush log / beacon)
```

- A hard landing opens the IMPACT window (its one 611 g/s sample) but the
  gyro never confirms, so it falls back to MONITOR 50 ms later.
- Aerobatics never open the window (in-flight jerk background ≤ 267 g/s).
- The crash opens the window at first contact and confirms 4 ms later.

![state trace](03_state_machine_trace.png)

## Measured results (`python3 simulate.py`)

- Flight 4: `MONITOR → IMPACT` at t=17.950 s, `→ CRASHED` at t=17.954 s —
  6 ms after first contact, **10 ms before the log dies from power loss**.
- Flight 3: one `MONITOR → IMPACT → MONITOR` round trip at the landing,
  zero false positives over the full 222 s flight.
- Threshold sweep (`J_TRIG` 300–800 g/s × `G_CRASH` 150–500 deg/s ×
  `N_TRIG` 1–3, all 75 combinations): crash detected in 4–10 ms, no false
  positive — the defaults sit mid-grid, so the result is not tuned to a knife
  edge. Both flights' impact signals exceed the thresholds by 1.5–5× while
  all non-crash activity stays 2–3.5× below them.

## Efficiency

Per sample: 3 subtractions, ~8 multiplies, 2–3 comparisons. Thresholds are
compared against *squared* magnitudes so no square roots are taken, and jerk
uses the actual inter-sample dt, so the detector is sample-rate independent
(validated at 500 Hz, intended for the 800 Hz logger). Pure Python does
~1.4 M samples/s (0.7 µs/sample) on a laptop; the arithmetic is trivial for
any MCU or the Pi logger loop.

## Files

- `crash_detector.py` — the detector (`CrashDetector.update()` per sample).
- `simulate.py` — replays the two test logs through the detector and
  pass/fails; give it any blackbox CSV path to replay other flights, or
  `--sweep` for the threshold-margin grid.
- `make_figures.py` — regenerates the figures below (needs numpy+matplotlib).
- `01…03*.png` — analysis figures.

## Integration sketch

```python
from crash_detector import CrashDetector, CRASHED

det = CrashDetector()
# in the logger poll loop, after converting to g / deg/s:
if det.update(t_us, ax, ay, az, gx, gy, gz) == CRASHED:
    flush_log_and_fire_beacon()   # runs once; state is latched
```

## Caveats / next data to collect

- One crash and one landing so far. The gyro margin (84 vs 455 deg/s) is
  wide, but more landings — especially sloppy/tilted ones — would firm up
  `G_CRASH`, and more crash types (soft-surface, clipped prop, mid-air) would
  confirm `J_TRIG`.
- Props striking ground on a tip-over after a *gentle* touchdown may look
  crash-like (jerk + rotation). If that case matters, gate on
  time-since-window-open or motor state.
