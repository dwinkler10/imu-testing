#!/usr/bin/env python3
"""Convert a .bin file from imu_logger.py to CSV and Foxglove-viewable MCAP
(protobuf-encoded crashlog.ImuSample messages, see proto/imu.proto).

Usage: python3 convert.py data/boot_000001_20260707_120000.bin
Writes .csv and .mcap next to the input file.

Reads the direct-polling logger's IMULOG06 format (and older
IMULOG02..05): the header carries the configured ranges and the
sample spacing in sensortime ticks. IMULOG06 additionally embeds the
fully-resolved sensor config (JSON, right after the header); it is
surfaced as a latched /config topic (crashlog.LoggerConfig),
re-emitted every CONFIG_TOPIC_INTERVAL_S so Foxglove shows it at any
seek point. The per-record flags byte is decoded per format:
  IMULOG06 -> same 6-bit SATURATION register as IMULOG05, plus the
              embedded config block.
  IMULOG05 -> BMI270 SATURATION register 0x4A, six per-axis bits
              (acc x/y/z, gyr x/y/z): a raw pre-filter sample on that
              axis hit the +/-full-scale rail.
  IMULOG04 -> logger-inferred "output at the int16 rail", aggregate
              (bit0 acc, bit1 gyr) -> mapped onto all three axes.
  IMULOG03 -> BMI270 FIFO-frame saturation tag, aggregate, same mapping.
  IMULOG02 -> no saturation info.

No filtering/smoothing is applied -- values are raw sensor output
(with only the mandatory CAS correction, see below) so a later
jerk-based crash detector sees untouched data.

Handles a crash-corrupted tail (partial last record is dropped) and
cross-checks for sample loss via sensortime gaps. IMULOG04 sensortime
is latched at readout, so deltas carry sub-period jitter; they are
rounded to whole ODR periods before gap-checking (exact for the older
formats, whose deltas are exact multiples).
Applies the gyro cross-axis (CAS) correction from datasheet section
4.6.10:
  gx_corrected = gx - factor_zx * gz / 2^9
"""
import csv
import json
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "proto"))
import imu_pb2  # noqa: E402
from mcap_protobuf.writer import Writer  # noqa: E402

HEADER = struct.Struct("<8sbBHI")
RECORD = struct.Struct("<QIhhhhhhB")   # host_t, st, ax,ay,az,gx,gy,gz, flags
# IMULOG06 embeds the resolved config as a latched /config topic; re-emit
# it this often (in host-time seconds) so Foxglove shows it at any seek.
CONFIG_TOPIC_INTERVAL_S = 10.0


def main(path):
    with open(path, "rb") as f:
        raw = f.read()

    magic, cas, acc_range, gyr_range, ticks = HEADER.unpack(raw[:HEADER.size])
    if magic == b"IMULOG02":
        ticks = 16                         # fixed 1600 Hz era, field was reserved
    elif magic not in (b"IMULOG03", b"IMULOG04", b"IMULOG05", b"IMULOG06"):
        raise SystemExit(f"unknown magic {magic!r} (expected IMULOG02..IMULOG06)")
    per_axis_sat = magic in (b"IMULOG05", b"IMULOG06")  # 6-bit SATURATION reg
    rate_hz = 25600 / ticks                # sensortime runs at 25.6 kHz
    off = HEADER.size
    registers = {}
    if magic == b"IMULOG06":               # length-prefixed JSON config block
        (clen,) = struct.unpack_from("<H", raw, off)
        off += 2
        registers = json.loads(raw[off:off + clen].decode())
        off += clen
    body = raw[off:]
    n = len(body) // RECORD.size
    if len(body) % RECORD.size:
        print(f"note: dropped {len(body) % RECORD.size} corrupted tail bytes")

    acc_scale = acc_range / 32768.0            # LSB -> g
    gyr_scale = gyr_range / 32768.0            # LSB -> dps

    base = path.rsplit(".", 1)[0]
    csv_f = open(base + ".csv", "w", newline="")
    cw = csv.writer(csv_f)
    cw.writerow(["host_time_ns", "sensortime",
                 "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
                 "acc_sat_x", "acc_sat_y", "acc_sat_z",
                 "gyr_sat_x", "gyr_sat_y", "gyr_sat_z"])

    mcap_f = open(base + ".mcap", "wb")
    mw = Writer(mcap_f)

    # Latched /config topic: build once (config is constant for the log),
    # re-emitted every CONFIG_TOPIC_INTERVAL_S below so Foxglove shows it
    # wherever the playhead is.
    cfg_msg = imu_pb2.LoggerConfig(
        format=magic.decode(errors="replace"), sample_rate_hz=rate_hz,
        acc_range_g=acc_range, gyr_range_dps=gyr_range, gyr_cas_factor_zx=cas)
    for k, v in registers.items():
        try:
            cfg_msg.registers[k] = int(v)
        except (TypeError, ValueError):
            pass
    config_interval_ns = int(CONFIG_TOPIC_INTERVAL_S * 1e9)
    next_config_t = None

    prev_st, gaps, dropped = None, 0, 0
    acc_sat_n, gyr_sat_n = 0, 0
    for i in range(n):
        t, st, ax, ay, az, gx, gy, gz, flags = RECORD.unpack_from(body, i * RECORD.size)
        if next_config_t is None or t >= next_config_t:   # latched /config
            mw.write_message("/config", cfg_msg, log_time=t, publish_time=t)
            next_config_t = t + config_interval_ns
        if per_axis_sat:                          # SATURATION reg 0x4A bits
            a = (bool(flags & 0x01), bool(flags & 0x02), bool(flags & 0x04))
            g = (bool(flags & 0x08), bool(flags & 0x10), bool(flags & 0x20))
        else:                                     # aggregate flag -> all axes
            a = (bool(flags & 0x01),) * 3
            g = (bool(flags & 0x02),) * 3
        acc_sat_n += any(a)
        gyr_sat_n += any(g)

        if prev_st is not None:
            delta = (st - prev_st) & 0xFFFFFF     # 24-bit counter wraps
            periods = (delta + ticks // 2) // ticks   # round off readout jitter
            if periods > 1:                       # one ODR period per sample
                gaps += 1
                dropped += periods - 1
        prev_st = st

        gx = gx - cas * gz / 512.0             # CAS correction (x-axis only)
        s = imu_pb2.ImuSample(host_time_ns=t, sensortime=st)
        s.accel_g.x, s.accel_g.y, s.accel_g.z = (
            ax * acc_scale, ay * acc_scale, az * acc_scale)
        s.gyro_dps.x, s.gyro_dps.y, s.gyro_dps.z = (
            gx * gyr_scale, gy * gyr_scale, gz * gyr_scale)
        s.acc_saturated.x, s.acc_saturated.y, s.acc_saturated.z = a
        s.gyr_saturated.x, s.gyr_saturated.y, s.gyr_saturated.z = g

        cw.writerow((t, st,
                     round(s.accel_g.x, 5), round(s.accel_g.y, 5),
                     round(s.accel_g.z, 5), round(s.gyro_dps.x, 4),
                     round(s.gyro_dps.y, 4), round(s.gyro_dps.z, 4),
                     int(a[0]), int(a[1]), int(a[2]),
                     int(g[0]), int(g[1]), int(g[2])))
        mw.write_message("/imu", s, log_time=t, publish_time=t)

    mw.finish()
    mcap_f.close()
    csv_f.close()
    print(f"{n} samples @ {rate_hz:g} Hz (+/-{acc_range}g, +/-{gyr_range}dps) "
          f"-> {base}.csv, {base}.mcap")
    if gaps:
        print(f"warning: {gaps} sensortime gaps (~{dropped} samples apparently missing)")
    if acc_sat_n or gyr_sat_n:
        print(f"saturation: accel {acc_sat_n}/{n} samples, gyro {gyr_sat_n}/{n} samples")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
