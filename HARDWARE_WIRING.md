# Firefighter visor hardware map

This is the wiring that the current firmware can verify. Power down the ESP32
before moving wires. Never infer a connection from wire colour; follow the
labels printed on each board.

## Pi and ESP32-S3 USB

Connect the Freenove ESP32-S3 to a Raspberry Pi USB-A port through the
Freenove board's **right USB-C socket under BOOT/IO0** (antenna at the top,
USB sockets at the bottom). This is the CH343 USB-UART socket. The left native
USB/OTG socket is not used by this build.

The Pi remains connected to the Mac through the Pi's USB-C gadget cable. This
USB link has a fixed local address and does not replace the Mac's Wi-Fi route.

The separate assembled IMU/MQ-7/air-temperature ESP connects directly to the
Mac by USB-C. It enumerates as Espressif `303A:1001`, serial
`7C:DF:A1:FB:16:FC`; the launcher accepts only that identity. Its internal GPIO
wiring was not changed or inferred during this integration.

## Confirmed connections

| Device | Device pin | Freenove ESP32-S3 |
| --- | --- | --- |
| SSD1306 OLED | `VIN` | `3V3` |
| SSD1306 OLED | `GND` | `GND` |
| SSD1306 OLED | `DATA` | GPIO 8 (I2C SDA) |
| SSD1306 OLED | `CLK` | GPIO 9 (I2C SCL) |
| IR receiver module | `AO` | GPIO 10 |
| IR receiver module | `VCC` | `3V3` |
| IR receiver module | `GND` | `GND` |

Leave the OLED's `3Vo`, `CS`, `DC`, and `RST` disconnected in I2C mode. The
receiver `AO` connection is based on the user's GPIO 10 description. The
separate IR emitter is the simulated target. Receiver polarity must be measured
with the emitter off/covered and then on/aimed; do not infer detection from a
high raw number alone.

A single forward receiver can confirm increasing signal and centre alignment,
but cannot distinguish left from right. Left/right guidance requires the
relative heading supplied by the separate sensor ESP (so signal samples can be
tagged by yaw) or two receivers mounted with different viewing directions.

## Optional Pi-attached GY-521 / MPU6050

The GY-521 can share the OLED's I2C bus:

| GY-521 | Freenove ESP32-S3 |
| --- | --- |
| `VCC` | `3V3` |
| `GND` | `GND` |
| `SDA` | GPIO 8 |
| `SCL` | GPIO 9 |

The firmware probes addresses `0x68` and `0x69`. An MPU6050 can produce a
short-term relative yaw after stationary calibration, but it has no
magnetometer and therefore cannot provide an absolute magnetic heading.
Keep the glasses still for about 1.5 seconds after the ESP32 starts. Then sweep
the receiver past the emitter once: the HUD remembers the strongest 10-degree
sector and shows the approximate left/right degrees needed to turn back toward
it. Re-zero by restarting if gyro drift becomes noticeable.

## Connections deliberately not guessed

- **Optional Pi-attached MQ-2 gas module:** do not connect a 5 V module's `AO`
  directly to the ESP32. Use a verified divider before a chosen ADC pin. The
  exact module and chosen GPIO must be recorded before enabling it in firmware.
- **Separate sensor ESP internals:** it currently reports valid `HEADING`,
  `AIR`, and `GAS` fields, but do not move its IMU, MQ-7, or temperature-sensor
  wiring until the printed module labels and internal GPIOs are documented.
- **Ultrasonic/ToF sensors:** record each exact model, power voltage, and GPIO
  or I2C address. Unknown devices remain unavailable in the HUD rather than
  displaying a fabricated zero or safe state.

At boot the firmware prints every responding I2C address. Once the S3 appears
on the Pi, use that scan plus the module labels to finish the remaining map.

## Live result on this hardware

The connected OLED acknowledges at `0x3D` and the installed firmware completed
direct SSD1306 initialization in 1 ms. The Pi-to-S3 accepted-frame and
S3-to-OLED displayed-frame counters advance together at 15 fps with zero CRC,
length, or partial-packet errors. GPIO10 two-point calibration measured
ambient/covered `4095.0` and emitter-aimed `147.5`, proving an active-low
receiver with a 3947.5-count response. The Pi service now normalizes that range
to 0..4095 target strength. The separate Mac ESP now supplies relative yaw,
air near `24.0 C`, and MQ-7 raw values near `1270-1315`; the gas channel remains
in warm-up for roughly 2-3 minutes after connection. Its `FLAME` field is
filtered so it cannot replace the glasses receiver.
