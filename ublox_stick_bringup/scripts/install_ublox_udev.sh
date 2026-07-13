#!/bin/bash
# Install udev rule so ublox_dgnss can access the X20P USB interface (1546:01ab).
# Requires sudo.

set -euo pipefail

RULE_FILE=/etc/udev/rules.d/99-ublox-gnss.rules

sudo tee "$RULE_FILE" >/dev/null <<'EOF'
# UBLOX ZED-F9P/F9R (CDC-ACM)
ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev", ENV{ID_MM_DEVICE_IGNORE}="1"
# UBLOX ZED-X20P main interface (CDC-ACM) - required for ublox_dgnss
ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01ab", MODE="0666", GROUP="plugdev", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Installed $RULE_FILE"
echo "Unplug/replug the GPS USB cable, or run: sudo chmod 666 /dev/bus/usb/BUS/DEV"
