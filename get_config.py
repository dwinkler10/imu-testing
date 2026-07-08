#!/usr/bin/env python3
"""Download the BMI270 init config blob (8 kB) from Bosch's official
SensorAPI repo and save it as bmi270_config.bin next to this script.

The BMI270 requires this firmware blob to be uploaded at every power-on
(datasheet section 4.4). Run this once, with internet access.
"""
import os
import re
import urllib.request

URLS = [
    "https://raw.githubusercontent.com/boschsensortec/BMI270_SensorAPI/master/bmi270.c",
    "https://raw.githubusercontent.com/BoschSensortec/BMI270-Sensor-API/master/bmi270.c",
]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bmi270_config.bin")


def main():
    src = None
    for url in URLS:
        try:
            src = urllib.request.urlopen(url, timeout=30).read().decode()
            break
        except Exception as e:
            print(f"failed {url}: {e}")
    if src is None:
        raise SystemExit("could not download bmi270.c")

    m = re.search(r"bmi270_config_file\[\]\s*=\s*\{(.*?)\};", src, re.S)
    if not m:
        raise SystemExit("bmi270_config_file array not found in bmi270.c")

    blob = bytes(int(h, 16) for h in re.findall(r"0[xX][0-9a-fA-F]{2}", m.group(1)))
    if len(blob) != 8192:
        raise SystemExit(f"unexpected config size {len(blob)} (expected 8192)")

    with open(OUT, "wb") as f:
        f.write(blob)
    print(f"wrote {OUT} ({len(blob)} bytes)")


if __name__ == "__main__":
    main()
