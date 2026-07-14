#!/usr/bin/env python3
"""BMI270 raw IMU logger for Raspberry Pi 4/5 (I2C), FIFO-drain capture.

This runs as a control daemon: on start it initialises the sensor and
then WAITS on a Unix socket (SOCK_PATH), idle, NOT recording. The
`imu-loggerd` CLI drives it:

    imu-loggerd start [-o file.bin] [-c config.json]     # start recording
    imu-loggerd stop                                     # stop + flush
    imu-loggerd status                                   # daemon state

So the systemd unit can be up and armed across a whole session while
recordings are started/stopped on demand, each to its own file.

Sensor configuration is read from a config.json (the one next to this
script by default, or the file passed to `start -c`) at the moment a
recording starts (see DEFAULT_CONFIG below for keys and defaults; values
may be hex strings like "0x0b" or plain ints). The defaults are accel
800 Hz +/-16g, gyro 800 Hz +/-2000dps, performance (aliasing-free)
filter mode -- a jerk-based crash detector needs full bandwidth and
untouched raw values, so no lowpass/notch filtering is applied here or
in convert.py. The config is written to the sensor's registers and
embedded in the recording (it stays constant for that recording).

Samples are captured through the chip's 2 KB FIFO (header mode) and
drained as aggressively as the bus allows: each loop iteration reads
the FIFO fill level and, the moment at least one frame is queued,
burst-reads the whole backlog in a single write+read transaction and
appends it to disk. The FIFO is NOT used to batch samples -- with
continuous draining the sensor-to-disk latency stays at one poll
cycle, which is the point: this logger feeds a crash-detection test.
The FIFO is a hardware safety net: when the scheduler stalls the
loop, samples queue up on-chip (2 KB holds ~157 acc+gyr frames =
~195 ms at 800 Hz) instead of being overwritten unread, which is
what the previous direct data-register polling did. Data is lost
only when a stall outlives the entire FIFO; the chip then overwrites
the OLDEST frames (keeping the newest -- the ones that matter for a
crash, streaming mode) and reports the loss in a skip frame, which
is counted and reported live.

Direct register polling was abandoned because it loses a sample for
every poll cycle that runs longer than one ODR period: on a Pi 4
(slower BCM2835 I2C, no multi-message combined transactions) the
cycle time sat right at the 800 Hz period and ~20% of samples were
lost. The FIFO burst read is a plain pointer-write + read supported
by every Pi's I2C controller, and its per-sample bus cost is lower
than per-register polling (13 bytes/frame, no wasted not-ready
polls), so the same code path is used on the Pi 4 and Pi 5.

Recordings are named boot_<idx>_<stamp>.bin, where <idx> is a
monotonic counter persisted in data/.boot_counter (so every start gets
a distinct, ordered name even with no RTC / a backward clock after an
unclean power cut). data/ is capped at MAX_BYTES: the oldest boot_*.bin
files are deleted at startup until the directory fits.

On-disk format (little-endian):
  header, 16 bytes: magic "IMULOG06", int8 gyro CAS factor_zx,
                    uint8 accel range [g], uint16 gyro range [dps],
                    uint32 sensortime ticks per sample
                    (ticks are 39.0625 us; 800 Hz -> 32 ticks)
  config block:     uint16 length, then that many bytes of UTF-8 JSON --
                    the fully-resolved sensor config (the register field
                    values from config.json merged over the defaults).
                    The config is read once at startup and is immutable
                    for the recording, so it is stored exactly once here
                    rather than sampled; the converter surfaces it as a
                    latched /config topic in the MCAP.
  records, 25 bytes each: uint64 host CLOCK_REALTIME [ns] taken once
                          per FIFO drain and back-computed one ODR
                          period per preceding frame in that drain,
                          uint32 sensortime (24-bit, 39.0625 us/LSB,
                          reconstructed on the exact ODR grid -- see
                          below), int16 ax, ay, az, gx, gy, gz (raw
                          LSB), uint8 saturation from the FIFO
                          frame-header tag bits: bits 0-2 all set
                          when any accel axis hit the rail on this
                          frame, bits 3-5 all set for any gyro axis.
                          (Per-sensor, not per-axis: the fh_ext tag
                          carries one bit per sensor. The bits are
                          broadcast to the per-axis positions so the
                          format and converters stay unchanged.)

Timing. The FIFO does not stamp individual frames, but frames are
produced exactly one ODR period apart on the sensortime grid, so per
-sample stamps are reconstructed instead of measured: a sensortime
control frame (the 24-bit counter latched as a drain empties the
FIFO) anchors the timeline; each record's sensortime is the anchor
advanced one period per frame (and per skip-frame drop), which by
construction lands on the exact power-of-two tick grid. Every later
sensortime frame re-checks the anchor and any mismatch is converted
into counted drops, so the reconstruction cannot drift silently.
Host timestamps carry up to one drain of jitter; the hardware
timeline is authoritative, as before.

Saturation comes from the FIFO frame-header fh_ext tag bits
(FIFO_CONFIG_1: INT1 tag = acc saturation, INT2 tag = gyr
saturation, datasheet 4.7.4/5.2.50), which are frame-locked to the
sample -- the exact guarantee the old direct-polling SATURATION
register read could not provide. The trade is per-axis resolution,
which the jerk-based detector does not use.

Frame consistency: the FIFO delivers whole frames; a frame only
partially fetched by a burst is retransmitted in full on the next
read (datasheet 4.7.2), so records cannot tear across drains.

A record cut off mid-write by a crash is simply a truncated tail; the
converter drops it. Convert with convert.py.
"""
import json
import os
import signal
import socket
import struct
import threading
import time

from smbus2 import SMBus, i2c_msg

I2C_BUS = 1
# Control socket. The daemon initialises the sensor and waits here; it
# records only when imu-loggerd sends a `start` command, and stops on
# `stop`. Override for local testing via $IMU_LOGGER_SOCK.
SOCK_PATH = os.environ.get("IMU_LOGGER_SOCK", "/run/imu-logger.sock")
ADDR = 0x68                    # SDO -> GND (0x69 if SDO -> VDDIO)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_BLOB = os.path.join(SCRIPT_DIR, "bmi270_config.bin")
CONFIG_PATH = os.path.join(SCRIPT_DIR, "config.json")
OUT_DIR = os.path.join(SCRIPT_DIR, "data")
COUNTER_PATH = os.path.join(OUT_DIR, ".boot_counter")
# Crash-resilience knob. On a hard power cut, at-risk data = up to one
# fsync interval written but not yet durable (there is no other
# buffering: each sample is written to the file the moment it is read
# off the chip).
FSYNC_INTERVAL_S = 0.05        # max written-but-unsynced data at risk
# Rolling storage budget for data/. Oldest recordings are deleted at
# startup until the directory fits. A ~20 min flight at 800 Hz is
# ~24 MB, so 2 GB keeps ~80 recordings before anything rolls off.
MAX_BYTES = 2 * 1024**3

# BMI270 registers (datasheet section 5.2)
CHIP_ID, INTERNAL_STATUS = 0x00, 0x21
SENSORTIME_0 = 0x18            # 24-bit free-running counter, 39.0625 us/LSB
FIFO_LENGTH_0, FIFO_DATA = 0x24, 0x26
FEAT_PAGE, GYR_CAS = 0x2F, 0x3C
ACC_CONF, ACC_RANGE, GYR_CONF, GYR_RANGE = 0x40, 0x41, 0x42, 0x43
FIFO_CONFIG_0, FIFO_CONFIG_1 = 0x48, 0x49
INIT_CTRL, INIT_ADDR_0, INIT_ADDR_1, INIT_DATA = 0x59, 0x5B, 0x5C, 0x5E
PWR_CONF, PWR_CTRL, CMD = 0x7C, 0x7D, 0x7E
CMD_FIFO_FLUSH = 0xB0

# FIFO (header mode, datasheet 4.7.1). Each regular frame is a 1-byte
# header -- fh_mode 0b10 in bits 7-6, enabled-sensor set in bits 5-2,
# saturation tags in bits 1-0 -- followed by 6 bytes per enabled sensor
# (gyro data BEFORE accel, the reverse of the data registers). Control
# frames carry fh_mode 0b01 and an opcode in bits 5-2: skip (1-byte
# payload: frames lost to overflow), sensortime (3 bytes, sent when a
# read empties the FIFO), input-config (4 bytes).
FIFO_SIZE = 2048
FH_MODE_MASK, FH_REGULAR, FH_CONTROL = 0xC0, 0x80, 0x40
FH_SKIP, FH_SENSORTIME, FH_CONFIG = 0x0, 0x1, 0x2
FH_EXT_ACC_SAT, FH_EXT_GYR_SAT = 0x01, 0x02   # INT1/INT2 tag bits

# Register field values -> physical units (datasheet 5.2.42 / 5.2.44)
ACC_RANGE_G = {0x00: 2, 0x01: 4, 0x02: 8, 0x03: 16}
GYR_RANGE_DPS = {0x00: 2000, 0x01: 1000, 0x02: 500, 0x03: 250, 0x04: 125}

# Sensor configuration, overridable via config.json. Values are the
# raw BMI270 register field values (ACC_CONF/ACC_RANGE/GYR_CONF/
# GYR_RANGE/PWR_CTRL, datasheet 5.2.41-5.2.44). Defaults: both sensors
# at 800 Hz (the fastest rate the poll loop sustains with margin),
# widest ranges, performance (aliasing-free) filter mode.
DEFAULT_CONFIG = {
    "acc_en": 0x01,           # PWR_CTRL.acc_en
    "acc_odr": 0x0B,          # ACC_CONF.acc_odr   (0x0b = 800 Hz)
    "acc_bwp": 0x02,          # ACC_CONF.acc_bwp   (0x02 = normal filter)
    "acc_filter_perf": 0x01,  # ACC_CONF.acc_filter_perf (1 = aliasing-free)
    "acc_range": 0x03,        # ACC_RANGE          (0x03 = +/-16g)
    "gyr_en": 0x01,           # PWR_CTRL.gyr_en
    "gyr_odr": 0x0B,          # GYR_CONF.gyr_odr   (0x0b = 800 Hz)
    "gyr_bwp": 0x02,          # GYR_CONF.gyr_bwp   (0x02 = normal filter)
    "gyr_noise_perf": 0x01,   # GYR_CONF.gyr_noise_perf (1 = low noise)
    "gyr_filter_perf": 0x01,  # GYR_CONF.gyr_filter_perf (1 = aliasing-free)
    "gyr_range": 0x00,        # GYR_RANGE          (0x00 = +/-2000 dps)
}

HEADER = struct.Struct("<8sbBHI")
RECORD = struct.Struct("<QIhhhhhhB")
AXES = struct.Struct("<hhhhhh")    # acc+gyr FIFO payload (gyro first)
AXES3 = struct.Struct("<hhh")      # single-sensor FIFO payload


def odr_hz(code):
    """ODR register code -> output data rate in Hz (25 Hz * 2^(code-6))."""
    return 25.0 * 2.0 ** (code - 6)


def odr_ticks(code):
    """ODR register code -> sensortime ticks per sample.

    Sensortime runs at 25.6 kHz (39.0625 us/tick) and every valid ODR
    is 25*2^k Hz, so the period is always an exact power-of-two tick
    count: 2^(16-code). 800 Hz (0x0b) -> 32 ticks.
    """
    return 1 << (16 - code)


def load_config(path=CONFIG_PATH):
    """Read config.json, merge over DEFAULT_CONFIG, and validate.

    Values may be ints or strings in any int() base-0 notation
    ("0x0b", "11"). Rejects unknown keys and register values the
    sample-timing model can't support, so a typo'd config fails the
    service visibly instead of silently recording bad data.
    """
    cfg = dict(DEFAULT_CONFIG)
    if os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        unknown = sorted(set(user) - set(DEFAULT_CONFIG))
        if unknown:
            raise SystemExit(f"config error: unknown key(s) {unknown}; "
                             f"valid keys: {sorted(DEFAULT_CONFIG)}")
        try:
            cfg.update({k: int(v, 0) if isinstance(v, str) else int(v)
                        for k, v in user.items()})
        except (TypeError, ValueError) as e:
            raise SystemExit(f"config error: values must be ints or int strings ({e})")
        print(f"config loaded from {path}")
    else:
        print(f"no {path}, using defaults")

    def check(cond, msg):
        if not cond:
            raise SystemExit(f"config error: {msg}")

    for k in ("acc_en", "acc_filter_perf", "gyr_en", "gyr_noise_perf",
              "gyr_filter_perf"):
        check(cfg[k] in (0, 1), f"{k} must be 0 or 1")
    check(cfg["acc_en"] or cfg["gyr_en"], "at least one sensor must be enabled")
    check(cfg["acc_range"] in ACC_RANGE_G, "acc_range must be 0x00..0x03")
    check(cfg["gyr_range"] in GYR_RANGE_DPS, "gyr_range must be 0x00..0x04")
    check(0x01 <= cfg["acc_odr"] <= 0x0C, "acc_odr must be 0x01..0x0c (max 1600 Hz)")
    check(0x06 <= cfg["gyr_odr"] <= 0x0D, "gyr_odr must be 0x06..0x0d (25..3200 Hz)")
    check(0 <= cfg["acc_bwp"] <= (3 if cfg["acc_filter_perf"] else 7),
          "acc_bwp must be 0x00..0x03 in filter-performance mode (0x00..0x07 otherwise)")
    check(0 <= cfg["gyr_bwp"] <= 2, "gyr_bwp must be 0x00..0x02")
    # One drdy bit triggers the record of both sensors' registers, so
    # the two must sample on a single common clock for the pairing to
    # be stable.
    check(not (cfg["acc_en"] and cfg["gyr_en"]) or cfg["acc_odr"] == cfg["gyr_odr"],
          "acc_odr and gyr_odr must match while both sensors are enabled")
    return cfg


def init_bmi270(bus):
    """One-time bring-up: soft reset + upload the 8 kB Bosch config blob.

    Done once when the daemon starts (the blob only needs reloading after
    a power-on/reset). The per-recording data-path registers are written
    separately by apply_config so a capture can pick a different config
    without re-uploading the blob."""
    if bus.read_byte_data(ADDR, CHIP_ID) != 0x24:
        raise SystemExit("BMI270 not found (bad chip id) -- check wiring/address")

    bus.write_byte_data(ADDR, CMD, 0xB6)          # soft reset
    time.sleep(0.01)
    bus.write_byte_data(ADDR, PWR_CONF, 0x00)     # disable adv. power save
    time.sleep(0.001)
    bus.write_byte_data(ADDR, INIT_CTRL, 0x00)    # prepare config load

    with open(CONFIG_BLOB, "rb") as f:
        blob = f.read()
    for i in range(0, len(blob), 128):            # chunked 8 kB upload
        idx = i // 2
        bus.write_byte_data(ADDR, INIT_ADDR_0, idx & 0x0F)
        bus.write_byte_data(ADDR, INIT_ADDR_1, (idx >> 4) & 0xFF)
        bus.i2c_rdwr(i2c_msg.write(ADDR, bytes([INIT_DATA]) + blob[i:i + 128]))

    bus.write_byte_data(ADDR, INIT_CTRL, 0x01)    # complete config load
    deadline = time.time() + 0.5
    while (bus.read_byte_data(ADDR, INTERNAL_STATUS) & 0x0F) != 0x01:
        if time.time() > deadline:
            raise SystemExit("BMI270 init failed (INTERNAL_STATUS != init_ok)")
        time.sleep(0.005)


def apply_config(bus, cfg):
    """Write the data-path registers for one recording and return the
    gyro cross-axis (CAS) factor. Called at the start of every capture,
    so each recording can use its own config (datasheet 5.2.41-5.2.44,
    5.2.85)."""
    pwr_ctrl = 0x08 | cfg["acc_en"] << 2 | cfg["gyr_en"] << 1   # + temp sensor
    acc_conf = cfg["acc_filter_perf"] << 7 | cfg["acc_bwp"] << 4 | cfg["acc_odr"]
    gyr_conf = (cfg["gyr_filter_perf"] << 7 | cfg["gyr_noise_perf"] << 6
                | cfg["gyr_bwp"] << 4 | cfg["gyr_odr"])
    bus.write_byte_data(ADDR, PWR_CTRL, pwr_ctrl)
    bus.write_byte_data(ADDR, ACC_CONF, acc_conf)
    bus.write_byte_data(ADDR, ACC_RANGE, cfg["acc_range"])
    bus.write_byte_data(ADDR, GYR_CONF, gyr_conf)
    bus.write_byte_data(ADDR, GYR_RANGE, cfg["gyr_range"])
    bus.write_byte_data(ADDR, PWR_CONF, 0x00)     # keep adv. power save off
    time.sleep(0.1)                               # gyro startup (45 ms typ.)

    bus.write_byte_data(ADDR, FEAT_PAGE, 0x00)    # gyro cross-axis factor for converter
    cas = bus.read_byte_data(ADDR, GYR_CAS) & 0x7F
    return cas - 128 if cas & 0x40 else cas       # sign-extend 7-bit


def setup_fifo(bus, acc_en, gyr_en):
    """Configure and flush the FIFO for one recording: header mode,
    streaming (overwrite-oldest on overflow -- for a crash the newest
    samples matter most), sensortime frame on empty reads, and the
    fh_ext saturation tags (INT1 tag <- acc_sat, INT2 tag <- gyr_sat,
    datasheet 5.2.50) so every frame carries frame-locked saturation.
    FIFO_DOWNS keeps its reset value (filtered data, no downsampling),
    so the FIFO frame rate is exactly the configured ODR."""
    bus.write_byte_data(ADDR, FIFO_CONFIG_0, 0x02)   # fifo_time_en, not stop_on_full
    bus.write_byte_data(ADDR, FIFO_CONFIG_1,
                        gyr_en << 7 | acc_en << 6 | 0x10   # header mode
                        | 0x03 << 2 | 0x02)   # int2 tag gyr_sat, int1 tag acc_sat
    bus.write_byte_data(ADDR, CMD, CMD_FIFO_FLUSH)


def parse_fifo_burst(buf, header_base, frame_len):
    """Split one FIFO burst into (records, skipped, sensortime).

    records:    [(sat_tag_bits, payload bytes)] in FIFO order, whole
                regular frames only -- a frame cut off at the end of
                the burst is dropped here and retransmitted in full by
                the chip on the next read (datasheet 4.7.2).
    skipped:    frames lost to FIFO overflow (skip-frame payload;
                always prepended by the chip, so it precedes the
                records it displaced).
    sensortime: 24-bit counter from the trailing sensortime frame if
                this burst emptied the FIFO, else None.

    Parsing stops at the first uninitialized (0x80 overread) or
    unexpected header; anything beyond it would be garbage."""
    recs, skipped, st = [], 0, None
    i, n = 0, len(buf)
    while i < n:
        h = buf[i]
        mode = h & FH_MODE_MASK
        if mode == FH_REGULAR:
            if h & 0xFC != header_base or i + frame_len > n:
                break                     # 0x80 overread / foreign / partial
            recs.append((h & 0x03, buf[i + 1:i + frame_len]))
            i += frame_len
        elif mode == FH_CONTROL:
            parm = (h >> 2) & 0x0F
            if parm == FH_SKIP:
                if i + 2 > n:
                    break
                skipped += buf[i + 1]
                i += 2
            elif parm == FH_SENSORTIME:
                if i + 4 <= n:
                    st = buf[i + 1] | buf[i + 2] << 8 | buf[i + 3] << 16
                break                     # always the last frame in a read
            elif parm == FH_CONFIG:
                i += 5                    # config-change marker, ignored
            else:
                break
        else:
            break
    return recs, skipped, st


def boot_index_of(name):
    """Parse the boot counter out of a boot_<idx>_<stamp>.bin filename;
    0 if it doesn't match (sorts such files oldest)."""
    try:
        return int(name.split("_")[1])
    except (IndexError, ValueError):
        return 0


def next_boot_index():
    """Monotonic per-recording counter, persisted in data/ so it
    survives reboots and never depends on the wall clock (the Pi has no
    RTC, and after an unclean power cut the clock can jump backward).
    Read, increment, write back atomically via a temp file + rename."""
    try:
        with open(COUNTER_PATH) as f:
            n = int(f.read().strip()) + 1
    except (FileNotFoundError, ValueError):
        n = 1
    tmp = COUNTER_PATH + ".tmp"
    with open(tmp, "w") as f:
        f.write(str(n))
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, COUNTER_PATH)
    return n


def prune_storage(budget=MAX_BYTES):
    """Delete oldest boot_*.bin recordings until data/ fits in `budget`
    bytes. Oldest = lowest boot index, which is clock-independent (a
    backward clock jump can't reorder them)."""
    files = []
    for name in os.listdir(OUT_DIR):
        if name.startswith("boot_") and name.endswith(".bin"):
            p = os.path.join(OUT_DIR, name)
            try:
                files.append((boot_index_of(name), os.path.getsize(p), p))
            except OSError:
                continue
    total = sum(sz for _, sz, _ in files)
    for _, sz, p in sorted(files):        # ascending index = oldest first
        if total <= budget:
            break
        try:
            os.remove(p)
            total -= sz
            print(f"rolling storage: removed {os.path.basename(p)}")
        except OSError:
            pass


def open_recording():
    """Prune to budget, then create a fresh recording file named
    boot_<idx>_<stamp>.bin. O_EXCL guarantees a brand-new file so a
    same-second/backward clock can never append onto an old recording;
    the monotonic idx makes that essentially impossible anyway. Returns
    (fd, path)."""
    os.makedirs(OUT_DIR, exist_ok=True)
    prune_storage()
    idx = next_boot_index()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for attempt in range(1000):
        tail = "" if attempt == 0 else f"_{attempt}"
        path = os.path.join(OUT_DIR, f"boot_{idx:06d}_{stamp}{tail}.bin")
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
            return fd, path
        except FileExistsError:
            continue
    raise SystemExit(f"could not create a unique recording file in {OUT_DIR}")


class Recorder:
    """Owns the recording. At most one recording at a time; the poll loop
    runs in a worker thread so the control socket stays responsive to a
    `stop` while a capture is in progress."""

    def __init__(self, bus):
        self.bus = bus
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.active = False
        self.path = None
        self.start_ns = None
        self.n = 0
        self.dropped = 0

    def _open_output(self, output):
        """(fd, path) for the recording. Default -> boot-counter name in
        OUT_DIR with rolling-storage prune; explicit -o -> that exact path,
        erroring if it already exists (O_EXCL)."""
        if not output:
            return open_recording()
        path = os.path.abspath(output)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            raise RuntimeError(f"output path already exists: {path}")
        return fd, path

    def start(self, cfg, output=None):
        with self._lock:
            if self.active:
                raise RuntimeError(f"already recording to {self.path}")
            odr_code = cfg["acc_odr"] if cfg["acc_en"] else cfg["gyr_odr"]
            ticks = odr_ticks(odr_code)
            fd, path = self._open_output(output)
            try:
                cas = apply_config(self.bus, cfg)
                setup_fifo(self.bus, cfg["acc_en"], cfg["gyr_en"])
                os.write(fd, HEADER.pack(
                    b"IMULOG06", cas, ACC_RANGE_G[cfg["acc_range"]],
                    GYR_RANGE_DPS[cfg["gyr_range"]], ticks))
                # cfg plus capture-mode provenance: fifo_capture=1 marks
                # FIFO-drain recordings (grid-reconstructed sensortime,
                # per-sensor saturation broadcast to the axis bits) apart
                # from older direct-polling files with the same magic. It
                # surfaces on the MCAP /config topic via the converter.
                cfg_json = json.dumps({**cfg, "fifo_capture": 1},
                                      sort_keys=True).encode()
                os.write(fd, struct.pack("<H", len(cfg_json)) + cfg_json)
                os.fsync(fd)
            except Exception:
                os.close(fd)
                raise
            self._stop.clear()
            self.n, self.dropped, self.active = 0, 0, True
            self.path, self.start_ns = path, time.time_ns()
            self._thread = threading.Thread(
                target=self._run,
                args=(fd, ticks, cfg["acc_en"], cfg["gyr_en"]), daemon=True)
            self._thread.start()
            return path, self.start_ns

    def _run(self, fd, ticks, acc_en, gyr_en):
        try:  # worker inherits normal priority; raise it for tight polling
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
        except (AttributeError, PermissionError, OSError):
            pass                              # AttributeError: non-Linux host
        bus = self.bus
        write, pack = os.write, RECORD.pack
        both = acc_en and gyr_en
        header_base = FH_REGULAR | gyr_en << 3 | acc_en << 2
        frame_len = 1 + 6 * (acc_en + gyr_en)
        period_ns = ticks * 625000 // 16      # 39062.5 ns/tick, exact
        grid_mask = 0xFFFFFF & ~(ticks - 1)   # floor onto the ODR tick grid
        st_next = None                        # grid sensortime of next record
        last_t = 0                            # host stamps kept monotonic
        next_sync = time.monotonic()
        try:
            while not self._stop.is_set():
                # Busy-poll the fill level (one cheap 2-byte read, its own
                # shadow group so the two bytes are consistent) and drain
                # the moment anything is queued: minimal buffering -- the
                # FIFO only accumulates while the loop is stalled.
                lo, hi = bus.read_i2c_block_data(ADDR, FIFO_LENGTH_0, 2)
                fill = lo | (hi & 0x3F) << 8
                if fill < frame_len:
                    continue
                # Overread by 4 bytes: when the burst empties the FIFO the
                # chip appends the sensortime control frame (header + 3
                # bytes) -- the timeline anchor. A frame that slipped in
                # after the fill read shows up as a partial frame instead;
                # the chip retransmits it in full on the next drain.
                rd = i2c_msg.read(ADDR, min(fill, FIFO_SIZE) + 4)
                bus.i2c_rdwr(i2c_msg.write(ADDR, [FIFO_DATA]), rd)
                t = time.time_ns()
                recs, skipped, st_frame = parse_fifo_burst(
                    bytes(rd), header_base, frame_len)
                if skipped:                   # overflow while stalled >195 ms
                    self.dropped += skipped
                    if st_next is not None:
                        st_next = (st_next + skipped * ticks) & 0xFFFFFF
                if not recs:
                    continue
                n_recs = len(recs)
                if st_next is None:
                    # First drain: anchor the grid. Samples land exactly on
                    # ODR ticks, so flooring the counter gives the last
                    # record's slot; the register read is a startup-only
                    # fallback if this drain did not empty the FIFO (its
                    # small error is corrected by the next re-anchor).
                    if st_frame is None:
                        b = bus.read_i2c_block_data(ADDR, SENSORTIME_0, 3)
                        st_frame = b[0] | b[1] << 8 | b[2] << 16
                    st_next = ((st_frame & grid_mask)
                               - (n_recs - 1) * ticks) & 0xFFFFFF
                for k, (tag, payload) in enumerate(recs):
                    if both:                  # FIFO order: gyro, then accel
                        gx, gy, gz, ax, ay, az = AXES.unpack(payload)
                    elif acc_en:
                        (ax, ay, az), gx, gy, gz = AXES3.unpack(payload), 0, 0, 0
                    else:
                        (gx, gy, gz), ax, ay, az = AXES3.unpack(payload), 0, 0, 0
                    sat = ((0x07 if tag & FH_EXT_ACC_SAT else 0)
                           | (0x38 if tag & FH_EXT_GYR_SAT else 0))
                    # Back-computed host stamp, clamped monotonic: a drain
                    # can land less than one backlog-worth after the
                    # previous one, which would step time backwards. The
                    # sensortime grid is the authoritative timeline.
                    tk = t - (n_recs - 1 - k) * period_ns
                    last_t = tk if tk > last_t else last_t + 1
                    write(fd, pack(last_t, st_next,
                                   ax, ay, az, gx, gy, gz, sat))
                    st_next = (st_next + ticks) & 0xFFFFFF
                self.n += n_recs
                if st_frame is not None:
                    # Re-anchor: the sensortime frame is latched as the
                    # drain empties, so its floor must equal the last
                    # record's slot (= st_next - ticks). A positive
                    # mismatch is unaccounted frames -> count as dropped
                    # and resync; the grid cannot drift silently.
                    drift = ((st_frame & grid_mask) - st_next + ticks) & 0xFFFFFF
                    if drift and drift < 0x800000:
                        self.dropped += drift // ticks
                        st_next = (st_next + drift) & 0xFFFFFF
                now = time.monotonic()
                if now >= next_sync:
                    os.fsync(fd)
                    next_sync = now + FSYNC_INTERVAL_S
        finally:
            os.fsync(fd)
            os.close(fd)

    def stop(self):
        with self._lock:
            if not self.active:
                raise RuntimeError("not recording")
            self._stop.set()
            thread = self._thread
        thread.join()
        with self._lock:
            self.active = False
            return {"path": self.path, "samples": self.n, "dropped": self.dropped}

    def status(self):
        with self._lock:
            return {"active": self.active, "path": self.path,
                    "samples": self.n, "dropped": self.dropped,
                    "start_ns": self.start_ns}


def handle_command(recorder, req):
    """Map one control request to a JSON-able response."""
    cmd = req.get("cmd")
    if cmd == "start":
        cfg = load_config(req.get("config") or CONFIG_PATH)
        path, start_ns = recorder.start(cfg, req.get("output"))
        ts = time.strftime("%Y-%m-%dT%H:%M:%S",
                           time.localtime(start_ns / 1e9))
        return {"ok": True, "path": path, "start_ns": start_ns, "timestamp": ts,
                "message": f"recording started {ts} -> {path}"}
    if cmd == "stop":
        s = recorder.stop()
        drop = f", ~{s['dropped']} lost to stalls" if s["dropped"] else ""
        return {"ok": True, "message":
                f"stopped: {s['samples']} samples -> {s['path']}{drop}", **s}
    if cmd == "status":
        s = recorder.status()
        state = "recording" if s["active"] else "idle"
        where = f" ({s['samples']} samples -> {s['path']})" if s["active"] else ""
        return {"ok": True, "message": f"{state}{where}", **s}
    return {"ok": False, "message": f"unknown command {cmd!r}"}


def serve(recorder):
    """Listen on SOCK_PATH; one newline-delimited JSON request/response per
    connection. Blocks (idle, ~0 CPU) until a client connects."""
    if os.path.exists(SOCK_PATH):
        os.unlink(SOCK_PATH)                      # clear a stale socket
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    os.chmod(SOCK_PATH, 0o660)
    srv.listen(4)
    print(f"imu-logger daemon ready on {SOCK_PATH} -- idle, not recording")
    try:
        while True:
            conn, _ = srv.accept()
            with conn:
                try:
                    buf = b""
                    while b"\n" not in buf:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        buf += chunk
                    req = json.loads(buf.decode() or "{}")
                    resp = handle_command(recorder, req)
                except (Exception, SystemExit) as e:   # SystemExit: bad config
                    resp = {"ok": False, "message": f"{type(e).__name__}: {e}"}
                try:
                    conn.sendall((json.dumps(resp) + "\n").encode())
                except OSError:
                    pass
    finally:
        srv.close()
        if os.path.exists(SOCK_PATH):
            os.unlink(SOCK_PATH)


def _on_sigterm(signum, frame):
    # systemd stop -> unwind serve()'s accept() the same way Ctrl-C does,
    # so an in-progress capture is flushed on the way out.
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, _on_sigterm)
    try:  # real-time priority; needs root, best effort otherwise
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
    except (AttributeError, PermissionError, OSError):
        print("warning: SCHED_FIFO unavailable, running at normal priority")

    with SMBus(I2C_BUS) as bus:
        init_bmi270(bus)                          # one-time blob upload
        recorder = Recorder(bus)
        try:
            serve(recorder)
        except KeyboardInterrupt:
            if recorder.active:                   # flush an in-progress capture
                s = recorder.stop()
                print(f"\nshutdown: stopped recording, "
                      f"{s['samples']} samples -> {s['path']}")


if __name__ == "__main__":
    main()
