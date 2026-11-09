#!/bin/zsh

# One-click launcher for the complete firefighter visor prototype.
# Starts the Pi camera/OLED service, relays the real ESP32-S3 sensors, and
# opens the color HUD full-screen on this Mac.

set -e

readonly SCRIPT_DIR=${0:A:h}
exec "$SCRIPT_DIR/Live_Camera.command"
