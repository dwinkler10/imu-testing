#!/usr/bin/env python3
"""BMI270 raw IMU logger for Raspberry Pi 5 (I2C), FIFO-based.

Sensor configuration is read from config.json next to this script
(see DEFAULT_CONFIG below for keys and defaults; values may be hex
strings like "0x0c" or plain ints). The defaults are accel 1600 Hz
+/-16g, gyro 1600 Hz +/-2000dps, performance (aliasing-free) filter
mode -- a jerk-based crash detector needs full bandwidth and untouched
raw values, not a smoothed or decimated signal, so no lowpass/notch
filtering is applied here or in convert.py. The config is read once at
startup; edit it and `sudo systemctl restart imu-logger` to begin a
new recording with the new settings.

Samples are drained from the BMI270's on-chip 2 KB FIFO (header mode,
datasheet section 4.7) instead of polled from the data registers.
Direct-register polling raced the host's scheduler against the fixed
sample period and lost ~4% of samples to timing jitter; the FIFO
buffers samples on-chip with sensortime-exact timing regardless of
when the host gets around to reading, and openly reports a count of
any *actual* loss via "skip frames" if the host ever falls behind (it
shouldn't -- even at the fastest rate the FIFO takes ~91 ms to fill
and this loop drains it every 10 ms). That skip-frame count replaces
the old sensortime-gap heuristic with a number the chip itself
guarantees.

Each regular FIFO frame also carries an accelerometer/gyroscope
saturation tag (FIFO_CONFIG_1 tag_int1/2_en = acc_sat/gyr_sat,
section 4.7.4), so clipping is flagged by the sensor's own filter
pipeline rather than inferred after the fact from output magnitude.
This tagging is a pure data-routing feature -- it does not require
the INT1/INT2 pins to be wired.

On-disk format (little-endian):
  header, 16 bytes: magic "IMULOG03", int8 gyro CAS factor_zx,
                    uint8 accel range [g], uint16 gyro range [dps],
                    uint32 sensortime ticks per sample
                    (ticks are 39.0625 us; 1600 Hz -> 16 ticks)
  records, 25 bytes each: uint64 host CLOCK_REALTIME [ns] (derived,
                          see below), uint32 sensortime (24-bit,
                          39.0625 us/LSB), int16 ax, ay, az, gx, gy,
                          gz (raw LSB), uint8 flags
                          (bit0 = acc_saturated, bit1 = gyr_saturated)

host_time_ns is not a per-sample clock read. One CLOCK_REALTIME read
is taken per drain, right after the I2C burst completes, and each
sample in the batch is back-dated from it by its exact position (one
ODR period apart, hardware-timed, no OS jitter). Because each drain
is stamped independently, host_time_ns can step back by a few ms
across drain boundaries -- use sensortime as the analysis timeline.
sensortime itself is a pure software counter advanced one ODR period
per frame (plus one per skip-frame-reported overflow drop); the
FIFO's own sensortime control frame is intentionally not used to pin
per-sample times because it records the readout instant, not the
sample instant, and pinning to it injects non-uniform spacing. The
result is a perfectly evenly spaced stream, which is what a later
jerk-based crash detector needs to differentiate cleanly.

A record cut off mid-write by a crash is simply a truncated tail; the
converter drops it. Convert with convert.py.
"""
import json
import os
import struct
import sys
import time

from smbus2 import SMBus, i2c_msg

I2C_BUS = 1
ADDR = 0x68                    # SDO -> GND (0x69 if SDO -> VDDIO)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_BLOB = os.path.join(SCRIPT_DIR, "bmi270_config.bin")
CONFIG_PATH = os.path.join(SCRIPT_DIR, "config.json")
OUT_DIR = os.path.join(SCRIPT_DIR, "data")
# Crash-resilience knobs. On a hard power cut, at-risk data =
# up to one poll interval still sitting in the FIFO SRAM (lost with
# the chip) plus up to one fsync interval written but not yet durable.
# A short poll interval keeps on-chip residency small (the FIFO is the
# one place buffering became unavoidable once we stopped direct-polling
# to fix sample drops); it stays far under the >=91 ms the 2 KB FIFO
# takes to fill even at the fastest configuration, so it never overflows.
FSYNC_INTERVAL_S = 0.05        # max written-but-unsynced data at risk
FIFO_POLL_INTERVAL_S = 0.01    # max on-chip residency at risk; ~10x fill margin
FIFO_CAP_BYTES = 2048

# BMI270 registers (datasheet section 5.2)
CHIP_ID, INTERNAL_STATUS = 0x00, 0x21
SENSORTIME_0 = 0x18
FIFO_LENGTH_0, FIFO_DATA = 0x24, 0x26
FEAT_PAGE, GYR_CAS = 0x2F, 0x3C
ACC_CONF, ACC_RANGE, GYR_CONF, GYR_RANGE = 0x40, 0x41, 0x42, 0x43
FIFO_CONFIG_0, FIFO_CONFIG_1 = 0x48, 0x49
INIT_CTRL, INIT_ADDR_0, INIT_ADDR_1, INIT_DATA = 0x59, 0x5B, 0x5C, 0x5E
PWR_CONF, PWR_CTRL, CMD = 0x7C, 0x7D, 0x7E

# Register field values -> physical units (datasheet 5.2.42 / 5.2.44)
ACC_RANGE_G = {0x00: 2, 0x01: 4, 0x02: 8, 0x03: 16}
GYR_RANGE_DPS = {0x00: 2000, 0x01: 1000, 0x02: 500, 0x03: 250, 0x04: 125}

# Sensor configuration, overridable via config.json. Values are the
# raw BMI270 register field values (ACC_CONF/ACC_RANGE/GYR_CONF/
# GYR_RANGE/PWR_CTRL, datasheet 5.2.41-5.2.44). Defaults: both sensors
# at 1600 Hz, widest ranges, performance (aliasing-free) filter mode.
DEFAULT_CONFIG = {
    "acc_en": 0x01,           # PWR_CTRL.acc_en
    "acc_odr": 0x0C,          # ACC_CONF.acc_odr   (0x0c = 1600 Hz)
    "acc_bwp": 0x02,          # ACC_CONF.acc_bwp   (0x02 = normal filter)
    "acc_filter_perf": 0x01,  # ACC_CONF.acc_filter_perf (1 = aliasing-free)
    "acc_range": 0x03,        # ACC_RANGE          (0x03 = +/-16g)
    "gyr_en": 0x01,           # PWR_CTRL.gyr_en
    "gyr_odr": 0x0C,          # GYR_CONF.gyr_odr   (0x0c = 1600 Hz)
    "gyr_bwp": 0x02,          # GYR_CONF.gyr_bwp   (0x02 = normal filter)
    "gyr_noise_perf": 0x01,   # GYR_CONF.gyr_noise_perf (1 = low noise)
    "gyr_filter_perf": 0x01,  # GYR_CONF.gyr_filter_perf (1 = aliasing-free)
    "gyr_range": 0x00,        # GYR_RANGE          (0x00 = +/-2000 dps)
}

HEADER = struct.Struct("<8sbBHI")
RECORD = struct.Struct("<QIhhhhhhB")


def odr_hz(code):
    """ODR register code -> output data rate in Hz (25 Hz * 2^(code-6))."""
    return 25.0 * 2.0 ** (code - 6)


def odr_ticks(code):
    """ODR register code -> sensortime ticks per sample.

    Sensortime runs at 25.6 kHz (39.0625 us/tick) and every valid ODR
    is 25*2^k Hz, so the period is always an exact power-of-two tick
    count: 2^(16-code). 1600 Hz (0x0c) -> 16 ticks.
    """
    return 1 << (16 - code)


def load_config(path=CONFIG_PATH):
    """Read config.json, merge over DEFAULT_CONFIG, and validate.

    Values may be ints or strings in any int() base-0 notation
    ("0x0c", "12"). Rejects unknown keys and register values the
    frame-timing model can't support, so a typo'd config fails the
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
    # The software sensortime counter advances one fixed ODR period per
    # FIFO frame, which requires every frame to carry every enabled
    # sensor -- i.e. a single common ODR (datasheet 4.7.3).
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
    bus.write_byte_data(ADDR, PWR_CONF, 0x02)     # adv. power save off
    time.sleep(0.1)                               # gyro startup (45 ms typ.)

    # FIFO: header mode, frames for the enabled sensors only (no aux),
    # don't stop on full (keep newest data if the host ever falls
    # behind), tag fh_ext<0> (INT1 slot) = acc_sat, fh_ext<1> (INT2
    # slot) = gyr_sat. Tagging is pure data routing, no pins needed.
    fifo_cfg1 = cfg["gyr_en"] << 7 | cfg["acc_en"] << 6 | 0x1E
    bus.write_byte_data(ADDR, FIFO_CONFIG_0, 0x02)   # fifo_time_en=1, fifo_stop_on_full=0
    bus.write_byte_data(ADDR, FIFO_CONFIG_1, fifo_cfg1)
    bus.write_byte_data(ADDR, CMD, 0xB0)             # fifo_flush: discard power-up garbage

    bus.write_byte_data(ADDR, FEAT_PAGE, 0x00)    # gyro cross-axis factor for converter
    cas = bus.read_byte_data(ADDR, GYR_CAS) & 0x7F
    return cas - 128 if cas & 0x40 else cas       # sign-extend 7-bit


def read_sensortime(bus):
    """Current value of the 24-bit sensortime counter (registers 0x18-0x1A)."""
    d = bus.read_i2c_block_data(ADDR, SENSORTIME_0, 3)
    return d[0] | d[1] << 8 | d[2] << 16


def parse_fifo_buffer(buf):
    """Pure parser for a raw FIFO_DATA burst (datasheet section 4.7.1).

    Returns (raw, skipped, anchor_st): raw is an ordered list of
    (ax, ay, az, gx, gy, gz, flags) tuples for each regular acc+gyr
    frame found (a disabled sensor's axes read as 0); skipped is the
    chip-reported frame-loss count from any skip frame; anchor_st is
    the sensortime of the last regular frame (from the trailing
    sensortime control frame), or None if the batch didn't end with
    one (e.g. buffer cut off mid-stream).

    Split out from drain_fifo() so the frame/header bit-parsing can
    be exercised directly against synthetic buffers, independent of
    any I2C hardware.
    """
    raw, skipped, i, anchor_st = [], 0, 0, None
    while i < len(buf):
        header = buf[i]
        if header == 0x80:                          # uninitialized frame = end of data
            break
        mode = header >> 6
        if mode == 0b10:                             # regular (data) frame
            parm = (header >> 2) & 0xF
            has_acc, has_gyr, has_aux = parm & 1, parm & 2, parm & 4
            plen = 6 * bool(has_acc) + 6 * bool(has_gyr) + 8 * bool(has_aux)
            if i + 1 + plen > len(buf):
                break                                # incomplete tail, chip will resend
            p = i + 1 + 8 * bool(has_aux)             # aux unused, skip its bytes
            gx, gy, gz = struct.unpack_from("<hhh", buf, p) if has_gyr else (0, 0, 0)
            p += 6 * bool(has_gyr)
            ax, ay, az = struct.unpack_from("<hhh", buf, p) if has_acc else (0, 0, 0)
            raw.append((ax, ay, az, gx, gy, gz, header & 0x03))
            i += 1 + plen
        elif mode == 0b01:                           # control frame
            opcode = (header >> 2) & 0xF
            if opcode == 0x0:                        # skip frame: 1 byte payload
                if i + 2 > len(buf):
                    break
                skipped += buf[i + 1]
                i += 2
            elif opcode == 0x1:                      # sensortime frame: 3 byte payload
                if i + 4 > len(buf):
                    break
                anchor_st = buf[i + 1] | buf[i + 2] << 8 | buf[i + 3] << 16
                i += 4
            elif opcode == 0x2:                      # fifo input config frame: 4 byte payload
                if i + 5 > len(buf):
                    break
                i += 5
            else:
                break                                 # reserved opcode, bail safely
        else:
            break                                     # reserved fh_mode, bail safely
    return raw, skipped, anchor_st


def drain_fifo(bus, next_st, ticks, frame_ns):
    """Read and parse every complete frame currently in the FIFO.

    next_st is the software-tracked sensortime the oldest unread frame
    is expected to carry; ticks/frame_ns are the configured ODR period
    in sensortime ticks and nanoseconds. Returns
    (records, skipped, new_next_st): records is a list of
    (host_time_ns, sensortime, ax, ay, az, gx, gy, gz, flags) tuples;
    skipped is the chip-reported dropped-frame count (0 in normal
    operation); new_next_st is next_st advanced past this batch.

    Sensortime is tracked purely in software (+ticks per frame), which
    is exact because the hardware ODR is exact -- see the timing note
    below where records are built. Frames are never discarded: a frame
    our read cuts short is simply not parsed, and the chip reverts its
    read pointer for a partially-read frame and resends it whole on the
    next drain (datasheet 4.7.2), so nothing is lost.
    """
    length = bus.read_i2c_block_data(ADDR, FIFO_LENGTH_0, 2)
    n = length[0] | (length[1] & 0x3F) << 8
    if n == 0:
        return [], 0, next_st

    # Over-read past the reported fill (which excludes control frames):
    # the extra often lets us reach empty and pick up the sensortime
    # frame for a free counter resync, and costs little since the tail
    # past real data reads back as 0x80 padding (parser stops on it).
    to_read = min(n + 64, FIFO_CAP_BYTES)
    write, read = i2c_msg.write(ADDR, [FIFO_DATA]), i2c_msg.read(ADDR, to_read)
    bus.i2c_rdwr(write, read)
    host_now = time.time_ns()

    raw, skipped, _anchor_st = parse_fifo_buffer(bytes(read))
    if not raw:
        return [], skipped, next_st
    n_frames = len(raw)

    # Pure software timing. Frames are exactly one ODR period apart
    # (hardware-timed, jitter-free), so sensortime is just a running
    # counter advanced +ticks per frame. A FIFO-overflow skip frame is
    # the ONLY real source of missing samples, so `skipped` is the only
    # thing that advances the counter faster than one frame at a time.
    #
    # The FIFO's own sensortime control frame is deliberately NOT used
    # to pin per-frame times: it stores the sensortime at *readout*, not
    # at sample generation (datasheet 4.7.1 -- "when the last byte of
    # the last sample frame was read"), so it drifts from the true
    # sample instant by a variable few ticks each drain. Pinning frames
    # to it injects exactly the non-uniform spacing (deltas != one ODR
    # period, even impossibly short deltas) that a jerk-based detector
    # must not see.
    oldest_st = (next_st + skipped * ticks) & 0xFFFFFF
    records = []
    for k, (ax, ay, az, gx, gy, gz, flags) in enumerate(raw):
        st = (oldest_st + ticks * k) & 0xFFFFFF
        t = host_now - (n_frames - 1 - k) * frame_ns
        records.append((t, st, ax, ay, az, gx, gy, gz, flags))
    return records, skipped, (oldest_st + ticks * n_frames) & 0xFFFFFF


def main():
    try:  # real-time priority; needs root, best effort otherwise
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
    except PermissionError:
        print("warning: no permission for SCHED_FIFO, running at normal priority")

    cfg = load_config()
    odr_code = cfg["acc_odr"] if cfg["acc_en"] else cfg["gyr_odr"]
    ticks = odr_ticks(odr_code)
    frame_ns = ticks * 78125 // 2                 # ticks * 39062.5 ns, exact
    rate_hz = odr_hz(odr_code)
    frame_bytes = 1 + 6 * cfg["acc_en"] + 6 * cfg["gyr_en"]
    fill_s = FIFO_CAP_BYTES / (frame_bytes * rate_hz)
    print(f"sensors: acc {'on' if cfg['acc_en'] else 'off'} "
          f"(+/-{ACC_RANGE_G[cfg['acc_range']]}g), "
          f"gyr {'on' if cfg['gyr_en'] else 'off'} "
          f"(+/-{GYR_RANGE_DPS[cfg['gyr_range']]}dps), ODR {rate_hz:g} Hz "
          f"({ticks} sensortime ticks/sample); FIFO fills in {fill_s * 1000:.0f} ms, "
          f"drained every {FIFO_POLL_INTERVAL_S * 1000:.0f} ms")

    with SMBus(I2C_BUS) as bus:
        cas = init_bmi270(bus, cfg)
        next_st = read_sensortime(bus)   # seed the software sensortime counter

        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR, time.strftime("imu_%Y%m%d_%H%M%S.bin"))
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        os.write(fd, HEADER.pack(b"IMULOG03", cas, ACC_RANGE_G[cfg["acc_range"]],
                                 GYR_RANGE_DPS[cfg["gyr_range"]], ticks))
        os.fsync(fd)
        print(f"logging to {path} (Ctrl-C to stop)")

        write, pack = os.write, RECORD.pack
        n, dropped, next_sync = 0, 0, time.monotonic()
        try:
            while True:
                records, skipped, next_st = drain_fifo(bus, next_st, ticks, frame_ns)
                if skipped:
                    dropped += skipped
                    print(f"warning: FIFO overflow, chip reports {skipped} frame(s) dropped")
                for rec in records:
                    write(fd, pack(*rec))
                n += len(records)
                now = time.monotonic()
                if now >= next_sync:
                    os.fsync(fd)
                    next_sync = now + FSYNC_INTERVAL_S
                time.sleep(FIFO_POLL_INTERVAL_S)
        except KeyboardInterrupt:
            pass
        finally:
            os.fsync(fd)
            os.close(fd)
            suffix = f" ({dropped} dropped by FIFO overflow)" if dropped else ""
            print(f"\n{n} samples written to {path}{suffix}")


if __name__ == "__main__":
    main()
