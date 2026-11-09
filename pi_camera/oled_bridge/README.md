# Camera + OLED bridge

These files capture the Pi camera once and produce both outputs:

- a live Mac viewer over the existing USB-only network at `10.12.194.1:8888`;
- 128x64 monochrome frames for the Freenove ESP32-S3 through its CH343
  USB-UART bridge at the stable Pi name `/dev/nathacks-oled` and 921600 baud.

The preferred bridge is deployed as `nathacks-camera.service` on the Pi. It
owns the camera and TCP ports, so do not start either alternate service at the
same time.

## Preferred profile

`picamera2_camera_oled_bridge.py` keeps the sharp 1920x1080 H.264 Mac feed and
uses the camera ISP's 256x144 low-resolution stream for the OLED. It requires the
Raspberry Pi OS package that is available in the configured apt repository:

```sh
sudo apt install --no-install-recommends python3-picamera2
```

The Pi currently has no default Internet route, so do not run that command until
the package can be transferred or Internet access is intentionally provided.

## No-install fallback

`raw_camera_oled_bridge.py` uses only Python's standard library and the already
installed `rpicam-vid`. It sends 1024x576 YUV420 to the Mac. This is smooth and
low latency, but uses more USB bandwidth and is less sharp than 1080p H.264.

## ESP32 wire format

Every OLED packet is exactly 1034 bytes:

```text
"VSR1" | sequence u16 LE | 1024 u16 LE | payload[1024] | CRC16 u16 LE
```

CRC-16/CCITT-FALSE covers the sequence, length, and payload. The payload is
unmirrored SSD1306 page order: `index=(y>>3)*128+x`, `bit=(y&7)`, `1=lit`.
Frames are capped at 15 FPS by the service templates (the receiver supports up
to 20 FPS). There is no ACK, so neither camera nor Mac video can be stalled by a
missing or slow ESP32.

## Validation without hardware changes

```sh
cd pi_camera/oled_bridge
python3 -m unittest -v test_bridge_common.py
python3 -m py_compile *.py
```

Both older subdirectory launchers now forward to the maintained root
`Live_Camera.command`. The raw service is retained only as source-level fallback;
the deployed H.264 service is the supported one-click path.
