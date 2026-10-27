#!/usr/bin/env bash
# One-shot Raspberry Pi setup + hardware check for ramenman.
# Run from inside the repo:   bash setup.sh
set -e
cd "$(dirname "$0")"

echo "=== 1/4  Enabling I2C (for the IMU) ==="
sudo raspi-config nonint do_i2c 0

echo "=== 2/4  Installing system packages ==="
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-venv i2c-tools git

echo "=== 3/4  Creating Python environment ==="
if [ ! -d .venv ]; then
  python3 -m venv --system-site-packages .venv
fi
.venv/bin/pip install -q -r requirements.txt

echo
echo "=== 4/4  Hardware check ==="
echo "--- OS ---"
grep PRETTY_NAME /etc/os-release
if [ -f /etc/rpi-issue ]; then echo "Raspberry Pi OS: yes"; else echo "Raspberry Pi OS: NOT detected (plain Debian?)"; fi

echo "--- Cameras ---"
rpicam-hello --list-cameras 2>&1 || true

echo "--- I2C devices (IMU should show 68 or 69) ---"
sudo i2cdetect -y 1 || true

echo
echo "=================================================="
echo "Setup done. Send a screenshot of everything above."
echo "If I2C was just enabled for the first time, reboot:"
echo "    sudo reboot"
echo "Then test the sensors with:"
echo "    cd ~/ramenman && source .venv/bin/activate && python3 test_sensors.py"
echo "=================================================="
