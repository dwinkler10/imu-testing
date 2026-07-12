"""Real-time crash detector: jerk-triggered, rotation-confirmed state machine.

Why this design (from Flight-4 crash vs Flight-3 landing data):

  * Peak accel does NOT separate the cases: the hard-surface crash peaked
    at 7.66 g but a firm normal landing peaked at 7.07 g.
  * Jerk (derivative of the accel vector) separates better: the crash
    sustained 1300-1971 g/s over ~12 ms, the landing showed a single
    611 g/s sample. But with one landing in the dataset that margin is
    too thin to trust alone.
  * Rotation is the clean discriminator: within 6 ms of the crash impact
    the gyro exploded past 450 deg/s (peaking 1735), while during the
    landing it never exceeded 84 deg/s. Aggressive flight reaches
    606 deg/s, but with jerk <= 267 g/s -- so neither signal alone is
    safe, and the two together are: crash = jerk AND rotation.

State machine (per IMU sample, O(1), no sqrt -- squared comparisons):

    MONITOR --jerk > J_TRIG--> IMPACT --window expires--> MONITOR
                               IMPACT --N jerk hits AND gyro > G_CRASH--> CRASHED

  A hard landing opens the IMPACT window but the gyro condition never
  fires, so it falls back to MONITOR. Aerobatics never open the window.
  CRASHED latches so the power-loss action (log flush, beacon) runs once.

Measured on the test data (simulate.py):
  * crash detected 6 ms after first contact, 10 ms before the log dies
    from power loss;
  * zero false positives across the full 222 s no-crash flight;
  * detection holds over the whole sweep J_TRIG 300-800 g/s,
    G_CRASH 150-500 deg/s, N_TRIG 1-3;
  * ~0.7 us per sample in pure Python -- three subtractions, six
    multiplies and a few compares -- comfortably real time at 800 Hz.
"""

MONITOR = 0   # normal flight, watching jerk
IMPACT = 1    # jerk spike seen, waiting for rotation confirmation
CRASHED = 2   # latched: impact + tumble

STATE_NAMES = {MONITOR: "MONITOR", IMPACT: "IMPACT", CRASHED: "CRASHED"}


class CrashDetector:
    """Streaming detector. Feed every IMU sample to update().

    Parameters (defaults sit mid-sweep, maximizing margin both ways):
      j_trig    g/s   jerk that opens/extends the confirmation window.
                      Flight background <= 267, landing peak 611, crash
                      1971 -- 500 is above all normal-flight jerk while
                      the crash crosses it on the first impact sample.
      g_crash   deg/s gyro magnitude that confirms loss of control.
                      Landing max 84, crash 455 within 6 ms of contact.
      n_trig    --    jerk hits required inside the window; 2 rejects a
                      single-sample spike (e.g. a hard landing's one
                      611 g/s sample) as an impact by itself.
      window_ms ms    confirmation window after the first jerk hit.
    """

    def __init__(self, j_trig=500.0, g_crash=300.0, n_trig=2, window_ms=50.0):
        self._j_trig2 = j_trig * j_trig
        self._g_crash2 = g_crash * g_crash
        self._n_trig = n_trig
        self._window_us = window_ms * 1000.0
        self.state = MONITOR
        self.crash_t_us = None
        self._prev = None          # (t_us, ax, ay, az)
        self._window_end = 0.0
        self._hits = 0

    def update(self, t_us, ax, ay, az, gx, gy, gz):
        """One sample: time in us, accel in g, gyro in deg/s.

        Any consistent units work -- thresholds just have to match.
        Returns the state after this sample; CRASHED is terminal.
        """
        if self.state == CRASHED:
            return CRASHED

        prev = self._prev
        self._prev = (t_us, ax, ay, az)
        if prev is None:
            return self.state
        dt_s = (t_us - prev[0]) * 1e-6
        if dt_s <= 0.0:
            return self.state

        inv = 1.0 / dt_s
        jx = (ax - prev[1]) * inv
        jy = (ay - prev[2]) * inv
        jz = (az - prev[3]) * inv
        jerk2 = jx * jx + jy * jy + jz * jz

        if self.state == MONITOR:
            if jerk2 > self._j_trig2:
                self.state = IMPACT
                self._window_end = t_us + self._window_us
                self._hits = 1
        else:  # IMPACT
            if t_us > self._window_end:
                self.state = MONITOR
                self._hits = 0
            else:
                if jerk2 > self._j_trig2:
                    self._hits = self._hits + 1
                if self._hits >= self._n_trig:
                    gyro2 = gx * gx + gy * gy + gz * gz
                    if gyro2 > self._g_crash2:
                        self.state = CRASHED
                        self.crash_t_us = t_us
        return self.state
