#!/bin/zsh

# Launch the Raspberry Pi camera with the lowest practical latency while
# retaining a sharp 1080p image. Double-click this file in Finder, or run:
#   ./Live_Camera.command

set -u

readonly SCRIPT_DIR=${0:A:h}
readonly WINDOW_TITLE="NATHACKS Firefighter Visor — Live HUD"
readonly USB_SSH_ALIAS="nathacks-pi"
readonly USB_STREAM_HOST="10.12.194.1"
readonly STREAM_PORT="8888"
readonly SENSOR_UDP_PORT="8892"
readonly RELAY_SCRIPT="$SCRIPT_DIR/pi_camera/reconnecting_h264_relay.py"
readonly SENSOR_RELAY_SCRIPT="$SCRIPT_DIR/mac_sensor_relay.py"
readonly LOCK_FILE="${TMPDIR:-/tmp}/nathacks-firefighter-view.lock"
readonly MAC_EPOCH="$(date +%s)"

RELAY_PID=""
SENSOR_RELAY_PID=""
FFPLAY_PID=""
PORT_FILE=""
HAVE_LOCK=false

cleanup() {
  local saved_status=$?
  trap - EXIT INT TERM HUP

  if [[ -n "${FFPLAY_PID:-}" ]] && kill -0 "$FFPLAY_PID" 2>/dev/null; then
    kill -TERM "$FFPLAY_PID" 2>/dev/null || true
    wait "$FFPLAY_PID" 2>/dev/null || true
  fi
  if [[ -n "${RELAY_PID:-}" ]] && kill -0 "$RELAY_PID" 2>/dev/null; then
    kill -TERM "$RELAY_PID" 2>/dev/null || true
    wait "$RELAY_PID" 2>/dev/null || true
  fi
  if [[ -n "${SENSOR_RELAY_PID:-}" ]] && kill -0 "$SENSOR_RELAY_PID" 2>/dev/null; then
    kill -TERM "$SENSOR_RELAY_PID" 2>/dev/null || true
    wait "$SENSOR_RELAY_PID" 2>/dev/null || true
  fi
  if [[ -n "${PORT_FILE:-}" ]]; then
    /bin/rm -f -- "$PORT_FILE"
  fi
  if [[ "$HAVE_LOCK" == true ]]; then
    /bin/rm -f -- "$LOCK_FILE"
  fi

  return "$saved_status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

pause_on_error() {
  print -u2 "\n$1"
  if [[ "${NATHACKS_NO_PAUSE:-0}" != 1 && -t 0 ]]; then
    print -u2 "Press Return to close this window."
    read -r
  fi
  exit 1
}

if [[ -x /opt/homebrew/bin/ffplay ]]; then
  FFPLAY=/opt/homebrew/bin/ffplay
elif command -v ffplay >/dev/null 2>&1; then
  FFPLAY=$(command -v ffplay)
else
  pause_on_error "ffplay is not installed. Install FFmpeg with: brew install ffmpeg"
fi

if ! command -v python3 >/dev/null 2>&1; then
  pause_on_error "Python 3 is required for automatic camera reconnection."
fi
readonly PYTHON3="$(command -v python3)"

if [[ ! -r "$RELAY_SCRIPT" ]]; then
  pause_on_error "The camera reconnect helper is missing: $RELAY_SCRIPT"
fi

if ! /usr/bin/shlock -f "$LOCK_FILE" -p $$; then
  print -u2 "Firefighter View is already running. Focus its video window, or press q there to stop it."
  exit 0
fi
HAVE_LOCK=true

SSH_OPTIONS=(
  -o BatchMode=yes
  -o ConnectTimeout=6
  -o ConnectionAttempts=1
)

if ssh "${SSH_OPTIONS[@]}" "$USB_SSH_ALIAS" true >/dev/null 2>&1; then
  PI_SSH_ALIAS=$USB_SSH_ALIAS
  STREAM_HOST=$USB_STREAM_HOST
else
  pause_on_error "The Pi is unreachable. Connect it directly to this Mac by USB-C; your Mac Wi-Fi will not be changed."
fi

print "Connecting to the camera on $PI_SSH_ALIAS over USB (Mac Wi-Fi is unchanged)..."
if ! ssh "${SSH_OPTIONS[@]}" "$PI_SSH_ALIAS" \
  "sudo date -u -s '@$MAC_EPOCH' >/dev/null && sudo systemctl start nathacks-camera.service && sudo systemctl is-active --quiet nathacks-camera.service"; then
  pause_on_error "The Pi camera service could not be started."
fi

print "Waiting for camera, OLED, and sensor services..."
SERVICE_READY=false
for _attempt in {1..60}; do
  if ssh "${SSH_OPTIONS[@]}" "$PI_SSH_ALIAS" \
    "systemctl is-active --quiet nathacks-camera.service && /usr/bin/ss -H -ltn 'sport = :${STREAM_PORT}' | /usr/bin/grep -q ." \
    >/dev/null 2>&1; then
    SERVICE_READY=true
    break
  fi
  sleep 0.5
done

if [[ "$SERVICE_READY" != true ]]; then
  pause_on_error "The Pi service started, but its firefighter camera stream did not become ready on USB port $STREAM_PORT."
fi

if ssh "${SSH_OPTIONS[@]}" "$PI_SSH_ALIAS" "test -e /dev/nathacks-oled" >/dev/null 2>&1; then
  print "ESP32-S3 connected: the same camera/HUD is being sent to the OLED and its sensors are returning to the Pi."
else
  print -u2 "Warning: the Pi camera is live, but the ESP32-S3/OLED is not connected."
  print -u2 "On the Freenove board, move USB-C to the RIGHT socket under BOOT/IO0; keep USB-A in the Pi."
fi

# The second ESP32-S3 remains attached directly to this Mac. Its IMU, air
# temperature, and MQ-7 fields are forwarded over the Pi's isolated USB
# network. The helper validates the exact USB identity and deliberately strips
# that board's legacy FLAME field so it cannot replace the glasses-mounted IR.
if [[ -r "$SENSOR_RELAY_SCRIPT" ]]; then
  "$PYTHON3" "$SENSOR_RELAY_SCRIPT" \
    --target "$STREAM_HOST" \
    --udp-port "$SENSOR_UDP_PORT" &
  SENSOR_RELAY_PID=$!
  sleep 0.2
  if kill -0 "$SENSOR_RELAY_PID" 2>/dev/null; then
    print "Separate sensor ESP relay started. Keep it still while the IMU calibrates."
    print "The MQ-7 gas card will show WARMUP for about 2-3 minutes."
  else
    wait "$SENSOR_RELAY_PID" 2>/dev/null || true
    SENSOR_RELAY_PID=""
    print -u2 "Warning: the separate sensor relay could not start; camera and OLED will continue."
  fi
else
  print -u2 "Warning: $SENSOR_RELAY_SCRIPT is missing; camera and OLED will continue without the separate sensors."
fi

PORT_FILE="$(mktemp "${TMPDIR:-/tmp}/nathacks-h264-relay.XXXXXX")" || pause_on_error "Could not create the local camera relay."
"$PYTHON3" "$RELAY_SCRIPT" \
  --upstream-host "$STREAM_HOST" \
  --upstream-port "$STREAM_PORT" \
  --port-file "$PORT_FILE" &
RELAY_PID=$!

RELAY_READY=false
for _attempt in {1..100}; do
  if [[ -s "$PORT_FILE" ]]; then
    RELAY_READY=true
    break
  fi
  if ! kill -0 "$RELAY_PID" 2>/dev/null; then
    wait "$RELAY_PID" 2>/dev/null || true
    pause_on_error "The local camera reconnect helper stopped unexpectedly."
  fi
  sleep 0.05
done

if [[ "$RELAY_READY" != true ]]; then
  pause_on_error "The local camera reconnect helper did not become ready."
fi

RELAY_PORT="$(<"$PORT_FILE")"
if [[ "$RELAY_PORT" != <-> ]] || (( RELAY_PORT < 1 || RELAY_PORT > 65535 )); then
  pause_on_error "The local camera reconnect helper returned an invalid port."
fi

print "Opening the live 1080p feed. Press f for full screen or q to quit."
print "If the Pi camera restarts, the picture will pause briefly and reconnect in the same window."
"$FFPLAY" \
  -x 1280 \
  -y 720 \
  -hide_banner \
  -loglevel warning \
  -fflags nobuffer \
  -flags low_delay \
  -framedrop \
  -fast \
  -probesize 32 \
  -analyzeduration 0 \
  -f h264 \
  -window_title "$WINDOW_TITLE" \
  "tcp://127.0.0.1:$RELAY_PORT" &
FFPLAY_PID=$!

wait "$FFPLAY_PID"
FFPLAY_STATUS=$?
FFPLAY_PID=""
exit "$FFPLAY_STATUS"
