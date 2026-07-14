#!/bin/sh
# Package the logger and deploy it to a Raspberry Pi 5 over SSH.
#
# Usage: ./deploy.sh <user@pi-host>       e.g. ./deploy.sh pi@raspberrypi.local
#
# Copies only what the vehicle needs (logger, sensor init blob, default
# config, systemd unit, installer) and runs install.sh on the Pi, which
# enables I2C at 1 MHz, installs python3-smbus2, installs everything to
# /opt/imu-logger, and enables + starts the imu-logger service. Re-running
# redeploys code but preserves an edited config.json on the Pi.
set -eu

[ $# -eq 1 ] || { echo "usage: $0 <user@pi-host>"; exit 1; }
HOST=$1
SRC=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

FILES="imu_logger.py imu-loggerd config.json bmi270_config.bin imu-logger.service install.sh"
for f in $FILES; do
    [ -f "$SRC/$f" ] || { echo "missing $SRC/$f"; exit 1; }
done

echo "== copying package to $HOST =="
tar -czf - -C "$SRC" $FILES | ssh "$HOST" \
    'rm -rf /tmp/imu-logger-pkg && mkdir -p /tmp/imu-logger-pkg && tar -xzf - -C /tmp/imu-logger-pkg'

echo "== running installer on $HOST (will sudo) =="
ssh -t "$HOST" 'sudo /tmp/imu-logger-pkg/install.sh'

echo
echo "done. the daemon is up and IDLE (armed, not recording). control it:"
echo "  sudo imu-loggerd status                 # daemon / recording state"
echo "  sudo imu-loggerd start                  # start recording (default file)"
echo "  sudo imu-loggerd start -o run1.bin -c my.json     # custom file/config"
echo "  sudo imu-loggerd stop                   # stop + flush the recording"
echo "  sudo systemctl status imu-logger        # is the daemon running?"
echo "  ls /opt/imu-logger/data/                # recordings"
