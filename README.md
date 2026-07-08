# BMI270 crash IMU logger (Raspberry Pi 5)

Logs **raw, unfiltered accel + gyro** with Raspberry Pi clock timestamps and
continuous flushing to disk. Sample rate, ranges, and filter mode are set in
[config.json](config.json); the defaults are **1600 Hz, ±16 g / ±2000 dps,
aliasing-free filter mode** — the widest ranges and full bandwidth the BMI270
offers. Values are stored untouched (only the datasheet-mandated gyro
cross-axis correction is applied offline) so a future jerk-based
crash-detection state machine sees the true signal, not a smoothed one.

Samples are drained from the BMI270's **on-chip FIFO** (header mode) rather
than polled from the data registers. Direct-register polling raced the Pi's
scheduler against the fixed 625 µs sample period and dropped ~4 % of samples to
timing jitter during flight; the FIFO buffers on-chip with hardware-exact
timing, so the record stream is gap-free and evenly spaced. Each frame also
carries the sensor's own **accel/gyro saturation flag** (§4.7.4), so clipping
is reported by the chip's filter pipeline instead of guessed from magnitude.

## Wiring — check pin 4!

The Pi 5 I2C1 pins are:

| BMI270 | Pi header pin | GPIO |
|--------|---------------|------|
| 3V3    | **1**         | —    |
| SDA    | **3**         | GPIO2 |
| SCL    | **5**         | GPIO3 |
| GND    | **6**         | —    |

**Pin 4 is 5 V.** The BMI270 is a 3.6 V-max part — if SCL (or anything) is on
pin 4, move it to pin 5 before powering up.

## Deploy to a Pi (one command)

From this directory on your workstation, with a freshly flashed Raspberry Pi
OS Lite Pi reachable over SSH:

```sh
./deploy.sh pi@raspberrypi.local
```

This packages `imu_logger.py`, `config.json`, the Bosch init blob, and the
systemd unit; then runs [install.sh](install.sh) on the Pi, which enables I2C
at 1 MHz (fast-mode+), apt-installs `python3-smbus2`, installs everything to
`/opt/imu-logger`, and enables + starts the `imu-logger` service (logging
from every boot — recommended for crash capture). If the I2C baudrate was
newly configured it asks for one `sudo reboot`; logging starts automatically
afterwards. Re-running `deploy.sh` updates the code but **preserves an edited
`config.json` on the Pi**.

Recordings land in `/opt/imu-logger/data/imu_YYYYMMDD_HHMMSS.bin` (~144 MB/h
at 1600 Hz — clear the card before a campaign).

## Configuration

`/opt/imu-logger/config.json` holds the raw BMI270 register field values
(hex strings or ints). Edit it, then start a new recording with the new
settings:

```sh
sudo nano /opt/imu-logger/config.json
sudo systemctl restart imu-logger
```

| key | register field | default | meaning |
|-----|----------------|---------|---------|
| `acc_en` / `gyr_en` | `PWR_CTRL` | `0x01` | sensor on/off |
| `acc_odr` / `gyr_odr` | `*_CONF.odr` | `0x0c` | sample rate: `0x0c`=1600 Hz, `0x0b`=800, `0x0a`=400, … (25·2ᵏ Hz) |
| `acc_bwp` / `gyr_bwp` | `*_CONF.bwp` | `0x02` | filter bandwidth (`0x02` = normal) |
| `acc_filter_perf` / `gyr_filter_perf` | `*_CONF.filter_perf` | `0x01` | 1 = aliasing-free performance filter |
| `gyr_noise_perf` | `GYR_CONF.noise_perf` | `0x01` | 1 = low-noise mode |
| `acc_range` | `ACC_RANGE` | `0x03` | `0x00`=±2g … `0x03`=±16g |
| `gyr_range` | `GYR_RANGE` | `0x00` | `0x00`=±2000dps … `0x04`=±125dps |

Missing keys fall back to the defaults above. The config is validated at
startup and the service fails visibly (see `systemctl status imu-logger`) on
unknown keys or unsupported values rather than recording bad data. One
constraint: **`acc_odr` must equal `gyr_odr` while both sensors are enabled**
— the frame-timing model requires a single common sample clock (a gyro-only
config may go up to `0x0d` = 3200 Hz).

## Run manually (bench tests)

```sh
sudo systemctl stop imu-logger           # free the sensor first
sudo python3 /opt/imu-logger/imu_logger.py   # sudo -> real-time priority; Ctrl-C to stop
```

## Convert to CSV + Foxglove MCAP

```sh
pip install mcap-protobuf-support protobuf   # converter deps (not needed on the Pi)
python3 convert.py data/imu_20260707_120000.bin
```

Produces `.csv` and `.mcap` next to the input. Messages are protobuf-encoded
`crashlog.ImuSample` ([proto/imu.proto](proto/imu.proto)); open the `.mcap`
in [Foxglove](https://foxglove.dev) and plot `/imu.accel_g.x` …
`/imu.gyro_dps.z`, plus the boolean `/imu.acc_saturated` /
`/imu.gyr_saturated` flags. The converter drops a crash-truncated partial last
record automatically, reports total per-axis saturation counts, and
cross-checks for sample loss via sensortime gaps (which should be zero — the
logger reports any real FIFO overflow live).

## Betaflight Blackbox (.BBL) → CSV + MCAP

`bbl_convert.py` converts Betaflight blackbox logs using Betaflight's
official decoder, then writes one MCAP per flight session:

```sh
git clone https://github.com/betaflight/blackbox-tools.git   # one-time
make -C blackbox-tools obj/blackbox_decode

python3 bbl_convert.py "~/Downloads/Blackbox - 51 Test - 4km.BBL"
```

A .BBL contains one log per arm; outputs land in `outputs/` as
`<name>.NN.csv` / `<name>.NN.mcap`. MCAP messages are protobuf-encoded
`crashlog.BlackboxFrame` ([proto/blackbox.proto](proto/blackbox.proto)) — in
Foxglove plot `/blackbox.gyro_dps.x`, `.acc_g.z`, `.motor[0]`,
`.battery.voltage_v`, etc. (units: deg/s, g, V, A). Columns the proto doesn't
know land in the `extras` map. The decoder is found via `$PATH`, the
`BLACKBOX_DECODE` env var, or `blackbox-tools/obj/` next to the script.

## Protobuf schemas

Message definitions live in [proto/](proto/): `common.proto` (Vector3),
`imu.proto` (`crashlog.ImuSample`), `blackbox.proto`
(`crashlog.BlackboxFrame`). The generated `*_pb2.py` files are committed;
after editing a `.proto`, regenerate with:

```sh
pip install grpcio-tools
python3 -m grpc_tools.protoc -Iproto --python_out=proto proto/*.proto
```

## Design notes

- **FIFO, header mode, acc+gyr only.** The logger drains the chip's 2 KB FIFO
  every 10 ms (`FIFO_POLL_INTERVAL_S`) — far under the ≥91 ms it takes to fill
  even at the fastest configuration, so it never overflows. Each frame is
  parsed to a 25-byte record (host time, sensortime, six int16 axes,
  saturation flags) written to disk immediately; `fsync` runs every 50 ms
  (`FSYNC_INTERVAL_S`).
- **Crash-resilience tradeoff.** Using the FIFO is the one place buffering
  became unavoidable — it was the price of eliminating the ~4 % flight sample
  drops. On a hard power cut, at-risk data ≈ one poll interval still in the
  FIFO SRAM (≤10 ms, lost with the chip) + one fsync interval written but not
  yet durable (≤50 ms). Shorten either constant to trade CPU/I-O for tighter
  crash capture. (In the reference drone crash the flight-controller log ended
  cleanly, i.e. battery power survived impact, so the FIFO residency is
  unlikely to eat the crash instant in practice.)
- **Timing.** Every frame is exactly one ODR period apart — hardware-exact,
  no OS jitter — so sensortime is tracked in software (+1 period per frame;
  16 ticks at 1600 Hz). The period is written into the file header
  (`IMULOG03`) so the converter scales and gap-checks correctly for any
  configured rate. `host_time_ns` (`CLOCK_REALTIME`) is captured per drain
  and each frame back-dated by its exact offset, giving uniform spacing for
  later differentiation into jerk — but it can step back a few ms across
  drain boundaries, so use `sensortime` as the analysis timeline.
- **Rates.** Both sensors default to 1600 Hz. The gyro alone could do
  3200 Hz (`gyr_odr: 0x0d` with `acc_en: 0`), but its filter bandwidth only
  rises 557 → 751 Hz for double the data — not worth it.
- **Saturation flags** come from `FIFO_CONFIG_1` frame tagging (`acc_sat` on
  the INT1 slot, `gyr_sat` on INT2), read out of each frame header's `fh_ext`
  bits — no interrupt pins need wiring. If a bench test taps the accel axis and
  the *gyr* flag lights instead, the two `fh_ext` bits are swapped for your
  part — flip bits 0/1 in `convert.py`.
- **Units.** Raw int16 are stored; the converter scales to g and dps and
  applies the datasheet's gyro cross-axis (CAS) correction (§4.6.10).
