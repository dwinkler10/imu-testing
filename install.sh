#!/bin/sh
# Install the BMI270 crash IMU logger on a Raspberry Pi (RPi OS Lite).
# Run as root ON THE PI from an unpacked package directory; normally
# invoked for you by deploy.sh. Safe to re-run: an existing (possibly
# edited) /opt/imu-logger/config.json is preserved.
set -eu

SRC=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DEST=/opt/imu-logger
BOOTCFG=/boot/firmware/config.txt

[ "$(id -u)" -eq 0 ] || { echo "run as root: sudo $0"; exit 1; }

echo "== enabling I2C =="
raspi-config nonint do_i2c 0
NEED_REBOOT=0
if ! grep -q '^dtparam=i2c_arm_baudrate=1000000' "$BOOTCFG"; then
    echo 'dtparam=i2c_arm_baudrate=1000000' >> "$BOOTCFG"   # BMI270 supports I2C fast-mode+ (1 MHz)
    NEED_REBOOT=1
fi

echo "== installing dependencies =="
if ! dpkg -s python3-smbus2 >/dev/null 2>&1; then
    apt-get update
    apt-get install -y --no-install-recommends python3-smbus2
fi

echo "== installing files to $DEST =="
mkdir -p "$DEST/data"
install -m 755 "$SRC/imu_logger.py" "$DEST/"
install -m 755 "$SRC/imu-loggerd" /usr/local/bin/imu-loggerd   # control CLI, on PATH
install -m 644 "$SRC/bmi270_config.bin" "$DEST/"
if [ ! -f "$DEST/config.json" ]; then
    install -m 644 "$SRC/config.json" "$DEST/"
else
    echo "keeping existing $DEST/config.json"
fi
install -m 644 "$SRC/imu-logger.service" /etc/systemd/system/

echo "== enabling service =="
systemctl daemon-reload
systemctl enable imu-logger

if [ "$NEED_REBOOT" -eq 1 ]; then
    echo
    echo "I2C baudrate was just configured -- reboot to apply:"
    echo "    sudo reboot"
    echo "The daemon comes up idle after the reboot; start a recording with"
    echo "    sudo imu-loggerd start"
else
    systemctl restart imu-logger
    sleep 2
    systemctl --no-pager --lines=5 status imu-logger || true
    echo
    echo "daemon is up and IDLE. start recording with: sudo imu-loggerd start"
fi
