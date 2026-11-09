# NATHACKS visor end-to-end test results

Tested on November 9, 2026 with the connected Raspberry Pi, Camera Module 2,
Freenove ESP32-S3-WROOM-1 N16R8, 128x64 SSD1306 OLED, GPIO10 IR receiver, and
the separate Mac-attached IMU/MQ-7/air-temperature ESP.

## Result: PASS

The complete live path is operational:

- Mac-to-Pi USB gadget SSH works at `10.12.194.1` without replacing the Mac's
  normal Wi-Fi default route.
- `nathacks-camera.service` is active with zero restarts.
- The 1920x1080 colour firefighter camera path advances normally.
- The OLED acknowledges on GPIO8/GPIO9 at I2C address `0x3D`.
- The ESP32 reports `READY VSR1`; direct SSD1306 initialization completes in
  1 ms.
- GPIO10 IR-receiver telemetry is fresh at 10 Hz (the protocol field remains
  `FLAME` for compatibility).
- The verified Mac ESP (`303A:1001`, serial `7C:DF:A1:FB:16:FC`) is relayed
  automatically. Live HUD values showed relative yaw `359`, air `24.0 C`, and
  MQ-7 raw `1270` in warm-up while its unrelated `FLAME` field was filtered.
- In the final live check, accepted OLED frames advanced `1192 -> 1236` and
  displayed frames advanced `1190 -> 1234` during the sample window.
- CRC, length, and partial-packet error counters remained `0/0/0`.
- The exact ESP32-S3 USB serial `5C94049272` now owns
  `/dev/nathacks-oled`; another CH343 board cannot take that name.
- The Mac viewer kept the same ffplay PID through both a controlled service
  restart and a full Pi reboot, paused briefly, and reconnected to each new
  H.264 process. The enabled camera service and exact-serial OLED link returned
  automatically after boot.

The final acceptance run passed installed IR calibration, normalized strength,
consistent scan/lock state, camera advancement, and OLED protocol integrity.
Automated validation passed `71/71` firefighter HUD/bridge/history tests,
`6/6` Mac sensor-relay tests, and `1/1` reconnect-relay test, plus Python
compilation and zsh syntax checks. The final sensitivity checks require two
consecutive normalized readings at or above `1200` before guidance starts,
three strong readings at or above `2800` before `FIRE` or history is created,
and eight low readings before search guidance clears.

The deployed Pi service loads OpenCV Headless 5.0.0 from its local vendor
directory. A captured live H.264 frame verified the full-frame, contrast-
equalized INFERNO false-colour camera look while HUD cards and text remained
crisp. This is a visible-light visualization rather than temperature data.

The Mac-only fire overlays were verified independently of the fading 36-sector
strip. A confirmed live source gets the brighter localized patch labeled
`FIRE`. The most recent source is retained for the current IMU/service session
at a world-relative bearing as a fixed light-orange patch labeled
`PREVIOUS FIRE DETECTED HERE`: it stays absent while off-camera and reappears
at the correct horizontal position when the wearer looks back. An overlapping
current source displays only `FIRE`; different in-frame current and previous
locations can display both labels. The session marker is not restored after a
service restart because relative-yaw zero can change. OLED output remains
text-free and unchanged.

The OLED renderer was also checked for the simplified wearer view: turn states
produce exactly one compact connected left or right edge arrow, target lock
leaves the processed camera unobstructed, there is no OLED text, and camera
pixels outside the small arrow and its contrast halo remain unchanged.

## Firmware and target evidence

Target identity:

- ESP32-S3 revision 0.2
- MAC `68:ee:8f:4e:ae:08`
- 16 MB flash, JEDEC manufacturer/device `46:4018`
- N16R8 board configuration: 16 MB flash and 8 MB OPI PSRAM
- flash encryption and secure boot disabled

All eight PlatformIO OLED profiles compile. The installed mirrored
I2C/SSD1306 firmware is 324,624 bytes with SHA-256:

```text
3513a44117a9afbd8d9119bdf760267f19a53e9c2d8cc6c8026e996a6d03cf6f
```

The firmware uses direct STOP-delimited SSD1306 commands and 32-byte frame
chunks for the installed I2C profile. This bypasses a U8g2 hardware-I2C
re-entry stall on the ESP32-S3 while retaining U8g2's framebuffer and drawing
functions.

## Recovery evidence

The original complete 16 MB flash image exists on both the Pi and this Mac.
Its SHA-256 is:

```text
cf3235cf5dc18c1d788d07a9b7e8fc987c4eaca15c897e9343e25d192d59797b
```

Mac copy:
`hardware_backups/freenove_s3_16mb_before_20261109T013319Z.bin`

The guarded provisioner validates staged hashes and partition bounds, exact
USB identity, chip/MAC, flash capacity/JEDEC identity, and security state before
writing. Every flashed region was read back and matched.

## Sensor limits

- GPIO10 calibration measured ambient/covered `4095.0` and an emitter-aimed
  median of `147.5` (observed minimum `125`). The receiver is active-low with a
  valid 3947.5-count span; these endpoints are installed in the Pi service.
- One forward-facing receiver supports signal strength and centre lock. It
  cannot produce a truthful left/right direction without heading-tagged scans
  or a second spatially separated receiver.
- The separate sensor ESP booted with `IMU_ID:0x70`, completed stationary gyro
  calibration, and supplied relative heading. This is not absolute north.
- MQ-7 telemetry is live but requires 2-3 minutes of heater warm-up and a local
  baseline; the raw ADC value is not a certified gas concentration.
- Ambient air telemetry is live at approximately `24.0 C` in the final check.

This remains a prototype and is not certified life-safety equipment.
