# Raspberry Pi live camera

`Live_Camera.command` opens the Pi Camera Module 2 feed on this Mac. The Pi
runs `nathacks-camera.service`, which captures the camera once and produces
both a low-latency 1080p H.264 Mac stream and 128x64 monochrome frames for the
ESP32 OLED. The service keeps running when the Mac viewer closes.

The OLED/IR ESP32-S3 is attached directly to the Pi. A second verified ESP stays
attached to the Mac and supplies relative heading, air temperature, and MQ-7
gas telemetry. `Live_Camera.command` automatically relays those three fields to
the Pi over the fixed USB link at `10.12.194.1`; none of the launchers changes
the Mac's Wi-Fi route. The second board's legacy `FLAME` field is discarded so
only the glasses-mounted GPIO10 receiver drives fire guidance.

## Start the viewer

Connect the Pi directly to the Mac over USB-C. The launcher uses only the USB
gadget network and never changes the Mac's Wi-Fi connection. Then double-click
`Live_Camera.command` or run:

```sh
./Live_Camera.command
```

Press `f` in the video window for full screen and `q` to close it. A local
relay keeps that window open and reconnects automatically if the Pi camera
service restarts; it exits with the viewer, so quitting never starts an
automatic relaunch loop.

## Stream profile

- 1920x1080 at 30 frames per second
- 180-degree camera rotation
- 18 Mbps hardware H.264, High profile level 4.1
- 0.5-second keyframe interval with inline headers
- Fast denoising and mild sharpening
- Immediate output flushing and no-buffer playback

The Mac stream is converted from visible-light BGR video into a full-frame
thermal-style false-colour view using OpenCV histogram equalization and the
INFERNO colour map. HUD cards, text, the heading strip, and arrows are rendered
after colour mapping so they remain crisp. The Pi service loads its headless
OpenCV runtime from `/home/nathacks/camera-oled/vendor`; this visual effect is a
simulation and does not turn the camera into a temperature sensor.

## OLED output

- 128x64 monochrome, centre-cropped from a separate 256x144 camera stream
- The processed camera remains the main OLED image; one compact left or right
  edge arrow is drawn only while a turn is needed, with a black separation halo
- Target lock removes the turn arrow instead of covering vision with an up cue
- No OLED text, degrees, labels, brackets, cards, or secondary symbols; the
  processed image is a visual stand-in, not a real temperature measurement
- Detailed IR/heading/air/gas data and labels remain on the Mac interface
- With heading available, the Mac shows the original 36-sector scan strip:
  low IR is blue, stronger alignment becomes orange/red, and previous peaks
  remain as fading marks while head movement scrolls the strip
- A credible in-frame source gets a brighter localized fire patch on the Mac
  video only. A confirmed current source is labeled `FIRE`, and its horizontal
  position follows relative bearing
- The most recent confirmed source in the current IMU/service session is
  retained at its world-relative bearing as a fixed light-orange patch labeled
  `PREVIOUS FIRE DETECTED HERE`. It goes off-screen with that bearing and
  reappears when the wearer looks back; it is intentionally not restored across
  a service restart because relative-yaw zero may change
- The full-frame false-colour effect and both localized colour patches are
  Mac-only; the OLED renderer and its monochrome camera pixels are unchanged
- Mac wearer-facing labels use `FIRE`; internal protocol fields remain unchanged
- With no heading sensor, a filtered guided sweep alternates/searches
  left/right, reverses after passing a signal peak, and locks ahead without
  inventing degrees
- Guidance starts only after two consecutive normalized readings reach `1200`.
  A confirmed fire and history entry require three consecutive readings at or
  above `2800`; eight sustained low readings clear search guidance
- 15 frames per second over the Freenove ESP32-S3's CH343 USB-UART
  connection at 921600 baud. The udev rule assigns `/dev/nathacks-oled` only
  to this board's verified USB serial `5C94049272`.
- CRC-checked frames with automatic reconnect if the ESP32 is unplugged
- Live `VSRSTAT` evidence lets the bridge track accepted/displayed frame
  counters and protocol errors
- The same S3 returns its GPIO10 analog IR-receiver reading over the full-duplex
  USB link; no separate Mac sensor relay is required for that reading
- The ESP32 must be connected to a Pi USB-A port; the Pi-to-Mac USB-C cable is
  reserved for power, SSH, and the Mac video stream
- On the Freenove board, use USB-UART: antenna at top and USB sockets at bottom
  means the **right socket under BOOT**

## Sensor telemetry

See `../HARDWARE_WIRING.md` for the complete confirmed and guarded wiring map.

- The Pi-attached OLED board reports the glasses IR as `FLAME:<raw>` over its
  existing full-duplex link
- The verified Mac board reports records such as
  `HEADING:359,AIR:24.0,FLAME:3990,GAS:1270` at 115200 baud. The relay forwards
  only `HEADING`, `AIR`, and `GAS`
- Keep the sensor ESP still for its startup IMU calibration. `HEADING` is
  relative yaw from startup and is not magnetic/absolute north
- Directional IR sectors require a fresh heading; one receiver alone provides
  strength/centre lock and never invents a left/right bearing
- GPIO10 is treated as analog `AO`; calibrate emitter-off/covered and
  emitter-on/aimed endpoints before target detection is enabled
- The MQ-7 needs roughly 2-3 minutes of heater warm-up. The Mac card remains
  `WARMUP` until the Pi establishes a live baseline; a raw reading is not a
  certified safe/unsafe gas concentration
- The verified air channel reports degrees Celsius from the separate sensor ESP
- Missing or failed sensors are omitted from telemetry so the Mac and OLED show
  no false safe/zero value; the OLED leaves its camera pixels unchanged when
  it has no trustworthy direction
- The one-click launcher starts and stops `mac_sensor_relay.py` automatically,
  matching only USB VID:PID `303A:1001` and serial `7C:DF:A1:FB:16:FC`
- A second sensor board plugged directly into the Pi is also auto-detected as
  `/dev/ttyACM*` or an unused `/dev/ttyUSB*`
- Fake readings are never enabled by default; `--fake-sensors` is an explicit
  labeled demo option for bench testing

## First S3 provisioning

`nathacks-provision-s3.service` is a guarded one-shot provisioner. It accepts
only the Freenove CH343 USB-UART identity, exact board MAC and flash JEDEC ID,
verifies the ESP32-S3/16MB/security state, backs up all 16MB, checks hashes,
flashes and verifies the receiver, and then restarts the camera service. The
verified original recovery image can be reused for a guarded firmware-only
update instead of taking a redundant second backup.

The installed I2C/SSD1306 profile uses GPIO8/GPIO9 and address `0x3D`. U8g2 is
retained for its 1 KiB drawing buffer, while a direct STOP-delimited SSD1306
transport avoids an ESP32-S3 Wire re-entry stall during display startup.

For a non-invasive service check from the Mac:

```sh
ssh nathacks-pi 'systemctl is-active nathacks-camera.service'
```

## IR-emitter target calibration

The GPIO10 value is raw ADC data. Do not assume that a high number means a
strong target: many receiver modules are active-low. Calibration samples once
with the emitter off (or receiver covered), then with the emitter on and aimed
directly at the glasses. A valid span must be at least 64 ADC counts. The bridge
maps the installed ambient and target endpoints onto a
polarity-independent 0..4095 target-strength scale. Until both endpoints are
configured, the Mac shows the raw value and `CALIBRATION NEEDED` without
claiming a target lock, while the OLED adds no direction overlay.

The installed receiver was calibrated active-low at ambient `4095.0` and aimed
target `147.5`; `/etc/systemd/system/nathacks-camera.service` includes those
validated arguments.

## Target sighting history

Lock transitions are persisted in SQLite at
`/home/nathacks/camera-oled/data/target_history.sqlite3`. Each row records UTC
time, raw and normalized IR values, and optional relative heading/bearing. A
missing bearing means only that the target was detected ahead; it is not a map
coordinate. Stored bearings are informational because the MPU6050 yaw zero can
change after restart.
