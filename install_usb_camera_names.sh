#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

sudo install -o root -g root -m 0644 usb-camera-names.rules /etc/udev/rules.d/99-usb-camera-names.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
sudo udevadm settle --timeout=10

ls -l /dev/camera_decxin /dev/camera_wdr5m
