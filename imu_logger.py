#!/usr/bin/env python3
"""BMI270 raw IMU logger for Raspberry Pi 5 (I2C), direct-register polling.

Sensor configuration is read from config.json next to this script
(see DEFAULT_CONFIG below for keys and defaults; values may be hex
strings like "0x0b" or plain ints). The defaults are accel 800 Hz
+/-16g, gyro 800 Hz +/-2000dps, performance (aliasing-free) filter
mode -- a jerk-based crash detector needs full bandwidth and untouched
raw values, not a smoothed or decimated signal, so no lowpass/notch
filtering is applied here or in convert.py. The config is read once at
startup; edit it and `sudo systemctl restart imu-logger` to begin a
new recording with the new settings.

Samples are polled directly from the data registers as fast as the
I2C bus allows -- no on-chip FIFO, no sleeps. Each loop iteration
burst-reads STATUS through SENSORTIME (0x03..0x1A) in one transaction
and keeps the sample only when STATUS.drdy_acc (drdy_gyr in a
gyro-only config) reports it as new; reading the data registers
clears the drdy bit, so each sample is recorded exactly once. Every
sample hits the OS page cache the moment it is read, which is the
point: this logger feeds a crash-detection test, so per-sample
latency from sensor to disk must be one poll cycle (~0.3 ms at 1 MHz
I2C), not a FIFO drain period.

The cost of direct polling is a race against the sample period: if
the scheduler stalls the loop for more than one ODR period, a sample
is overwritten unread. Missed samples are detected from the hardware
sensortime read in the same burst -- a delta of ~k ODR periods means
k-1 samples were lost -- counted, and reported live. One poll cycle
is ~0.3-0.4 ms on a Pi 5 at 1 MHz I2C, so 800 Hz (1.25 ms period)
leaves comfortable margin while 1600 Hz (625 us) does not and will
drop samples whenever the scheduler hiccups.

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
  records, 25 bytes each: uint64 host CLOCK_REALTIME [ns] read right
                          after the sample's I2C burst, uint32
                          sensortime (24-bit, 39.0625 us/LSB, latched
                          in the same burst), int16 ax, ay, az, gx,
                          gy, gz (raw LSB), uint8 saturation
                          (SATURATION register 0x4A, datasheet 5.2.51:
                          bit0 acc_x, bit1 acc_y, bit2 acc_z, bit3
                          gyr_x, bit4 gyr_y, bit5 gyr_z -- each set
                          when a raw pre-filter sample on that axis
                          hit the +/-full-scale rail)

Read consistency. acc/gyr/sensortime are read in one burst spanning
0x03..0x1A, which is entirely inside the register shadow group
(STATUS, DATA_x, SENSORTIME_x; datasheet 5.1): starting the burst
freezes the whole group, so those three are guaranteed to belong to
the same sample -- they cannot tear mid-read. SATURATION (0x4A) is
NOT in that shadow group, so it cannot be frozen together with the
data; it is instead read FIRST in the *same* i2c_rdwr transaction
(repeated start, single stop), one address-cycle (~tens of us) before
the data burst latches. It updates synchronously with the data
registers (5.2.51), so the only residual risk is a <=1-sample edge
misalignment of the flag if an ODR boundary happens to fall in that
tiny gap -- harmless, since saturation occurs in multi-sample runs.
Perfectly frame-locked saturation is only available via the FIFO
frame-header tag, which this direct-polling logger trades away for
sensor-to-disk latency.

Both timestamps are per-sample and taken at readout, so they carry up
to one poll cycle of jitter relative to the hardware-exact sample
instant. The true sample spacing is still exactly one ODR period;
the converter reconstructs the uniform timeline by rounding
sensortime deltas to whole periods.

A record cut off mid-write by a crash is simply a truncated tail; the
converter drops it. Convert with convert.py.
"""
import json
import os
import struct
import time

from smbus2 import SMBus, i2c_msg

I2C_BUS = 1
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
STATUS = 0x03                  # bit7 drdy_acc, bit6 drdy_gyr
SATURATION = 0x4A              # per-axis raw-saturation flags (5.2.51)
FEAT_PAGE, GYR_CAS = 0x2F, 0x3C
ACC_CONF, ACC_RANGE, GYR_CONF, GYR_RANGE = 0x40, 0x41, 0x42, 0x43
INIT_CTRL, INIT_ADDR_0, INIT_ADDR_1, INIT_DATA = 0x59, 0x5B, 0x5C, 0x5E
PWR_CONF, PWR_CTRL, CMD = 0x7C, 0x7D, 0x7E

# One poll = one burst read STATUS..SENSORTIME_2 (0x03..0x1A, 24 bytes).
# Offsets within the burst: 0 STATUS, 1-8 aux (unused), 9-14 acc x/y/z,
# 15-20 gyr x/y/z, 21-23 sensortime.
BURST_LEN = 24
ACC_OFS, ST_OFS = 9, 21

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
AXES = struct.Struct("<hhhhhh")


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


def init_bmi270(bus, cfg):
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

    # Compose the data-path registers from the user config
    # (datasheet 5.2.41-5.2.44, 5.2.85).
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


def main():
    try:  # real-time priority; needs root, best effort otherwise
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
    except PermissionError:
        print("warning: no permission for SCHED_FIFO, running at normal priority")

    cfg = load_config()
    odr_code = cfg["acc_odr"] if cfg["acc_en"] else cfg["gyr_odr"]
    ticks = odr_ticks(odr_code)
    rate_hz = odr_hz(odr_code)
    drdy_mask = 0x80 if cfg["acc_en"] else 0x40   # STATUS.drdy_acc / drdy_gyr
    print(f"sensors: acc {'on' if cfg['acc_en'] else 'off'} "
          f"(+/-{ACC_RANGE_G[cfg['acc_range']]}g), "
          f"gyr {'on' if cfg['gyr_en'] else 'off'} "
          f"(+/-{GYR_RANGE_DPS[cfg['gyr_range']]}dps), ODR {rate_hz:g} Hz "
          f"({ticks} sensortime ticks/sample); "
          f"polling data registers continuously, no FIFO")

    with SMBus(I2C_BUS) as bus:
        cas = init_bmi270(bus, cfg)

        fd, path = open_recording()
        os.write(fd, HEADER.pack(b"IMULOG06", cas, ACC_RANGE_G[cfg["acc_range"]],
                                 GYR_RANGE_DPS[cfg["gyr_range"]], ticks))
        # Config block: the resolved config is constant for this recording,
        # so store it once. length-prefixed JSON, right after the header.
        cfg_json = json.dumps(cfg, sort_keys=True).encode()
        os.write(fd, struct.pack("<H", len(cfg_json)) + cfg_json)
        os.fsync(fd)
        print(f"logging to {path} (Ctrl-C to stop)")

        write, pack, unpack_axes = os.write, RECORD.pack, AXES.unpack_from
        n, dropped, prev_st = 0, 0, None
        next_sync = time.monotonic()
        try:
            while True:
                # Busy-poll in ONE i2c transaction (repeated start, single
                # stop). SATURATION (0x4A, not in a shadow group) is read
                # FIRST so it is captured one address-cycle (~tens of us)
                # before the 0x03..0x1A data burst freezes its shadow
                # group -- that group (STATUS+DATA+SENSORTIME, datasheet
                # 5.1) is atomic, so acc/gyr/sensortime are same-frame,
                # and the sat-to-data gap is bounded to that address cycle.
                # The data read clears drdy, so a sample is kept once.
                sat = i2c_msg.read(ADDR, 1)
                data = i2c_msg.read(ADDR, BURST_LEN)
                bus.i2c_rdwr(i2c_msg.write(ADDR, [SATURATION]), sat,
                             i2c_msg.write(ADDR, [STATUS]), data)
                buf = bytes(data)
                if not buf[0] & drdy_mask:
                    continue
                t = time.time_ns()
                ax, ay, az, gx, gy, gz = unpack_axes(buf, ACC_OFS)
                st = buf[ST_OFS] | buf[ST_OFS + 1] << 8 | buf[ST_OFS + 2] << 16
                saturation = bytes(sat)[0] & 0x3F   # 6 per-axis bits (reg 0x4A)

                if prev_st is not None:
                    # Hardware sensortime deltas are k ODR periods plus
                    # sub-period readout jitter; round to whole periods.
                    periods = (((st - prev_st) & 0xFFFFFF) + ticks // 2) // ticks
                    if periods > 1:
                        dropped += periods - 1
                        print(f"warning: poll loop fell behind, "
                              f"~{periods - 1} sample(s) lost")
                prev_st = st

                write(fd, pack(t, st, ax, ay, az, gx, gy, gz, saturation))
                n += 1
                now = time.monotonic()
                if now >= next_sync:
                    os.fsync(fd)
                    next_sync = now + FSYNC_INTERVAL_S
        except KeyboardInterrupt:
            pass
        finally:
            os.fsync(fd)
            os.close(fd)
            suffix = f" (~{dropped} lost to poll-loop stalls)" if dropped else ""
            print(f"\n{n} samples written to {path}{suffix}")


if __name__ == "__main__":
    main()
