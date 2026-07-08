#!/usr/bin/env python3
"""Convert a Betaflight Blackbox log (.BBL/.BFL/.TXT) to CSV and MCAP.

Decoding is done by Betaflight's official blackbox_decode tool
(https://github.com/betaflight/blackbox-tools), which this script runs and
then converts each decoded flight session's CSV to a Foxglove-viewable MCAP
of protobuf-encoded crashlog.BlackboxFrame messages (see proto/blackbox.proto).

Usage: python3 bbl_convert.py "~/Downloads/Blackbox - 51 Test - 4km.BBL"

A .BBL file holds one log per arm; outputs are written to outputs/ next to
this script as <name>.01.csv/.mcap, <name>.02.csv/.mcap, ... Timestamps are
the flight controller's time-since-boot, so in Foxglove the timeline is
relative.
"""
import csv
import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "proto"))
import blackbox_pb2  # noqa: E402
from google.protobuf.descriptor import FieldDescriptor  # noqa: E402
from mcap_protobuf.writer import Writer  # noqa: E402

DECODE_ARGS = ["--unit-rotation", "deg/s", "--unit-acceleration", "g",
               "--unit-frame-time", "us"]

# grouping of known Betaflight columns into BlackboxFrame fields;
# unlisted numeric columns land in the frame's `extras` map
VEC3 = {"gyroADC": "gyro_dps", "gyroUnfilt": "gyro_raw", "accSmooth": "acc_g"}
VEC4 = {"setpoint": "setpoint", "rcCommand": "rc_command"}
ARRAY = {"motor": "motor", "eRPM": "erpm", "debug": "debug",
         "axisP": "pid_p", "axisI": "pid_i", "axisD": "pid_d", "axisF": "pid_f"}
SCALAR = {"time": "time_us", "loopIteration": "loop",
          "vbatLatest": ("battery", "voltage_v"),
          "amperageLatest": ("battery", "current_a"),
          "energyCumulative": ("battery", "mah"),
          "baroAlt": "baro_alt", "rssi": "rssi",
          "flightModeFlags": "flight_mode", "stateFlags": "state_flags",
          "failsafePhase": "failsafe", "rxSignalReceived": "rx_signal",
          "rxFlightChannelsValid": "rx_channels_valid"}
AXES3, AXES4 = "xyz", ("roll", "pitch", "yaw", "throttle")

FRAME_FIELDS = blackbox_pb2.BlackboxFrame.DESCRIPTOR.fields_by_name
INT_TYPES = {FieldDescriptor.TYPE_UINT32, FieldDescriptor.TYPE_UINT64,
             FieldDescriptor.TYPE_INT32, FieldDescriptor.TYPE_INT64}


def find_decoder():
    for cand in (os.environ.get("BLACKBOX_DECODE"),
                 shutil.which("blackbox_decode"),
                 os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "blackbox-tools", "obj", "blackbox_decode")):
        if cand and os.path.isfile(cand):
            return cand
    raise SystemExit(
        "blackbox_decode not found. Build it with:\n"
        "  git clone https://github.com/betaflight/blackbox-tools.git\n"
        "  make -C blackbox-tools obj/blackbox_decode\n"
        "or set the BLACKBOX_DECODE env var to its path.")


def column_paths(header):
    """Map each CSV column to its destination in BlackboxFrame."""
    paths = []
    for h in header:
        k = re.sub(r"_+", "_", re.sub(r"[^\w]", "_",
                   re.sub(r"\s*\(.*\)", "", h.strip()))).strip("_")
        m = re.fullmatch(r"(.+?)_(\d+)", k)
        base, idx = (m.group(1), int(m.group(2))) if m else (k, None)
        if idx is not None and base in VEC3 and idx < 3:
            paths.append((VEC3[base], AXES3[idx]))
        elif idx is not None and base in VEC4 and idx < 4:
            paths.append((VEC4[base], AXES4[idx]))
        elif idx is not None and base in ARRAY:
            paths.append((ARRAY[base],))          # repeated: append in order
        else:
            dest = SCALAR.get(k, k)
            paths.append(dest if isinstance(dest, tuple) else (dest,))
    return paths


def fill_frame(frame, paths, row):
    for path, raw in zip(paths, row):
        raw = raw.strip()
        if len(path) == 2:                        # vec3 / sticks / battery.*
            setattr(getattr(frame, path[0]), path[1], float(raw))
        else:
            name = path[0]
            fd = FRAME_FIELDS.get(name)
            if fd is None:                        # unknown column -> extras
                try:
                    frame.extras[name] = float(raw)
                except ValueError:
                    pass
            elif fd.label == fd.LABEL_REPEATED:
                getattr(frame, name).append(float(raw))
            elif fd.type == FieldDescriptor.TYPE_STRING:
                setattr(frame, name, raw)
            elif fd.type in INT_TYPES:
                setattr(frame, name, int(float(raw or 0)))
            else:
                setattr(frame, name, float(raw or 0))


def csv_to_mcap(csv_path):
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        paths = column_paths(next(reader))

        out = csv_path[:-4] + ".mcap"
        n = 0
        with open(out, "wb") as mf, Writer(mf) as mw:
            for row in reader:
                frame = blackbox_pb2.BlackboxFrame()
                fill_frame(frame, paths, row)
                t = frame.time_us * 1000          # us -> ns
                mw.write_message("/blackbox", frame, log_time=t, publish_time=t)
                n += 1
        print(f"  {os.path.basename(out)}  ({n} frames)")


def main(path):
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(path):
        raise SystemExit(f"no such file: {path}")
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(path))[0]

    # exit code is nonzero for benign warnings (e.g. a session with no
    # events), so judge success by whether CSVs were produced instead
    subprocess.run([find_decoder(), "--output-dir", out_dir, *DECODE_ARGS, path])

    csvs = sorted(f for f in os.listdir(out_dir)
                  if re.fullmatch(re.escape(base) + r"\.\d+\.csv", f))
    if not csvs:
        raise SystemExit("blackbox_decode produced no CSV output")

    print(f"\n{len(csvs)} flight session(s) decoded, converting to MCAP:")
    for f in csvs:
        csv_to_mcap(os.path.join(out_dir, f))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
