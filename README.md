# BMI270 crash IMU logger (Raspberry Pi 5)

Logs **raw, unfiltered accel + gyro** with Raspberry Pi clock timestamps and
continuous flushing to disk. Sample rate, ranges, and filter mode are set in
[config.json](config.json); the defaults are **800 Hz, ±16 g / ±2000 dps,
aliasing-free filter mode** — the widest ranges the BMI270 offers. Values are
stored untouched (only the datasheet-mandated gyro cross-axis correction is
applied offline) so a jerk-based crash-detection state machine sees the true
signal, not a smoothed one.

Samples are **polled directly from the data registers** as fast as the I2C
bus allows — no on-chip FIFO buffering. Each sample is on disk (page cache)
within one poll cycle (~0.3 ms) of leaving the sensor, so a crash-detection
test sees data in true realtime instead of waiting out a FIFO drain interval.
The tradeoff: if the OS stalls the poll loop for more than one sample period,
that sample is overwritten unread. At 800 Hz (1.25 ms period) the loop has
comfortable margin; missed samples are detected via hardware **sensortime**
gaps, counted, and reported live and at exit (1600 Hz previously dropped ~4 %
this way — that's why 800 Hz is the recommended ceiling).

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

Recordings land in `/opt/imu-logger/data/imu_YYYYMMDD_HHMMSS.bin` (~72 MB/h
at 800 Hz — clear the card before a campaign).

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
| `acc_odr` / `gyr_odr` | `*_CONF.odr` | `0x0b` | sample rate: `0x0c`=1600 Hz, `0x0b`=800, `0x0a`=400, … (25·2ᵏ Hz) |
| `acc_bwp` / `gyr_bwp` | `*_CONF.bwp` | `0x02` | filter bandwidth (`0x02` = normal) |
| `acc_filter_perf` / `gyr_filter_perf` | `*_CONF.filter_perf` | `0x01` | 1 = aliasing-free performance filter |
| `gyr_noise_perf` | `GYR_CONF.noise_perf` | `0x01` | 1 = low-noise mode |
| `acc_range` | `ACC_RANGE` | `0x03` | `0x00`=±2g … `0x03`=±16g |
| `gyr_range` | `GYR_RANGE` | `0x00` | `0x00`=±2000dps … `0x04`=±125dps |

Missing keys fall back to the defaults above. The config is validated at
startup and the service fails visibly (see `systemctl status imu-logger`) on
unknown keys or unsupported values rather than recording bad data. One
constraint: **`acc_odr` must equal `gyr_odr` while both sensors are enabled**
— one data-ready bit triggers the record of both sensors' registers, so they
must share a sample clock. Rates above 800 Hz are accepted but the poll loop
(~0.3–0.4 ms/cycle) will lose samples whenever the scheduler hiccups; the
logger reports every loss live.

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
record automatically, reports total saturation counts, and cross-checks for
sample loss via sensortime gaps (the logger also reports every loss live).

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

- **Direct-register busy-poll, no FIFO.** Each loop iteration burst-reads
  `STATUS` through `SENSORTIME` (0x03–0x1A, 24 bytes) in one I2C transaction
  and keeps the sample only when the data-ready bit says it's new (the read
  clears the bit, so nothing is recorded twice). Each sample becomes a
  25-byte record (host time, sensortime, six int16 axes, clip flags) written
  to disk immediately; `fsync` runs every 50 ms (`FSYNC_INTERVAL_S`). The
  loop never sleeps, so it pins one core — acceptable on a quad-core Pi 5 in
  exchange for minimum sensor-to-disk latency.
- **Crash-resilience.** With no buffering anywhere between sensor and page
  cache, at-risk data on a hard power cut ≈ one fsync interval (≤50 ms)
  written but not yet durable, plus the ≲1 sample still inside the chip.
  Shorten `FSYNC_INTERVAL_S` to trade I/O for tighter crash capture.
- **Timing.** `sensortime` is the chip's 24-bit 39.0625 µs hardware counter,
  latched in the same burst as the data. It is stamped at *readout*, so
  consecutive deltas are one ODR period ± sub-period poll jitter; the
  converter rounds deltas to whole periods (written into the `IMULOG04`
  header) for gap-checking, and a jerk detector should difference against the
  nominal period, not raw per-sample deltas. `host_time_ns`
  (`CLOCK_REALTIME`) is read per sample right after the I2C burst.
- **Rates.** Both sensors default to 800 Hz — the fastest rate the ~0.3–0.4 ms
  poll cycle can service with margin. 1600 Hz historically lost ~4 % of
  samples to scheduler jitter; if you need it, the FIFO-based logger
  (`IMULOG03`, in git history) is the right tool.
- **Clip flags** are inferred by the logger: the flag bit is set when any
  axis of that sensor sits at the int16 rail (±32767/−32768). The FIFO-era
  hardware saturation tag (§4.7.4) only exists in FIFO frames, so it is not
  available in direct-read mode.
- **Units.** Raw int16 are stored; the converter scales to g and dps and
  applies the datasheet's gyro cross-axis (CAS) correction (§4.6.10).
