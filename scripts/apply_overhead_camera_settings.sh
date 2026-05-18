#!/usr/bin/env bash
set -euo pipefail

OVERHEAD="/dev/v4l/by-id/usb-DHZJ-240229-XH_Integrated_Webcam_HD-video-index0"
CONTROLS="
brightness=0
contrast=3
saturation=69
hue=0
gamma=84
gain=1
power_line_frequency=1
sharpness=2
white_balance_temperature_auto=1
exposure_auto=3
backlight_compensation=32
"

if ! command -v v4l2-ctl >/dev/null 2>&1; then
    echo "ERROR: v4l2-ctl is required but was not found." >&2
    exit 1
fi

echo "Applying overhead camera settings to $OVERHEAD"
for control in $CONTROLS; do
    v4l2-ctl -d "$OVERHEAD" --set-ctrl="$control"
done

echo "Applied overhead camera settings:"
for control in $CONTROLS; do
    name="${control%%=*}"
    v4l2-ctl -d "$OVERHEAD" --get-ctrl="$name"
done
