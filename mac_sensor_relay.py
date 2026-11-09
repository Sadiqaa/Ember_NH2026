#!/usr/bin/env python3
"""Relay ESP32-S3 telemetry to the Pi over its USB-gadget network.

The sensor remains attached to the Mac. This process selects the verified
ESP32-S3 USB identity, removes its legacy FLAME field, and forwards its real
HEADING/AIR/GAS fields as UDP datagrams to the Pi.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import socket
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print(
        "Sensor relay needs pyserial. Install it with: python3 -m pip install pyserial",
        file=sys.stderr,
    )
    raise SystemExit(2)


DEFAULT_TARGET = "10.12.194.1"
DEFAULT_UDP_PORT = 8892
DEFAULT_BAUD = 115200
SENSOR_USB_VID = 0x303A
SENSOR_USB_PID = 0x1001
SENSOR_USB_SERIAL = "7C:DF:A1:FB:16:FC"
RECONNECT_DELAY_SECONDS = 1.0
# The sensor firmware can spend several seconds initialising/calibrating after
# the serial port opens. Closing it too quickly restarts some ESP32-S3 boards
# and traps them in a perpetual boot loop, so allow the full startup sequence.
PROBE_SECONDS = 30.0
MAX_RECORD_BYTES = 512
FORWARDED_FIELDS = ("HEADING", "AIR", "GAS")
FIELD_RANGES = {
    "HEADING": (0.0, 360.0),
    "AIR": (-100.0, 1000.0),
    "GAS": (0.0, 4095.0),
}


def log(message: str) -> None:
    print(f"[sensor] {message}", flush=True)


def _is_sensor_board(port: object) -> bool:
    """Match only the verified Mac-attached ESP32-S3 sensor board."""

    return (
        getattr(port, "vid", None) == SENSOR_USB_VID
        and getattr(port, "pid", None) == SENSOR_USB_PID
        and str(getattr(port, "serial_number", "")).upper()
        == SENSOR_USB_SERIAL
        and str(getattr(port, "device", "")).startswith("/dev/cu.")
    )


def discover_devices(last_working_device: str | None) -> list[str]:
    devices = sorted(
        str(port.device) for port in list_ports.comports() if _is_sensor_board(port)
    )
    if last_working_device in devices:
        devices.remove(last_working_device)
        devices.insert(0, last_working_device)
    return devices


def sanitize_record(raw_record: bytes) -> bytes | None:
    """Keep real IMU/air/MQ-7 data without replacing the visor IR channel.

    The separate sensor ESP still emits a legacy ``FLAME`` field. Fire-direction
    guidance belongs to the glasses-mounted GPIO10 receiver connected through
    the Pi, so that field must never cross this relay.
    """

    record = raw_record.rstrip(b"\r\n")
    if not record or len(record) > MAX_RECORD_BYTES or b"\n" in record or b"\r" in record:
        return None
    try:
        text = record.decode("ascii")
    except UnicodeDecodeError:
        return None

    values: dict[str, str] = {}
    for item in text.split(","):
        if ":" not in item:
            continue
        name, raw_value = item.split(":", 1)
        name = name.strip().upper()
        raw_value = raw_value.strip()
        if name not in FIELD_RANGES:
            continue
        try:
            numeric = float(raw_value)
        except ValueError:
            continue
        minimum, maximum = FIELD_RANGES[name]
        if not math.isfinite(numeric) or not minimum <= numeric <= maximum:
            continue
        if name == "HEADING" and numeric == 360.0:
            raw_value = "0"
        values[name] = raw_value

    # A heading is required to identify this board's intended telemetry and to
    # pair subsequent visor-IR samples with the wearer's relative yaw.
    if "HEADING" not in values:
        return None
    return ",".join(
        f"{name}:{values[name]}" for name in FORWARDED_FIELDS if name in values
    ).encode("ascii")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward verified ESP32 IMU/air/MQ-7 telemetry to the Pi."
    )
    parser.add_argument("--target", default=DEFAULT_TARGET, help="Pi USB-gadget IP")
    parser.add_argument(
        "--udp-port", type=int, default=DEFAULT_UDP_PORT, help="Pi telemetry UDP port"
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    args = parser.parse_args()

    if not 1 <= args.udp_port <= 65535:
        parser.error("--udp-port must be between 1 and 65535")
    if args.baud <= 0:
        parser.error("--baud must be positive")
    return args


def run(args: argparse.Namespace) -> int:
    stopping = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopping.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGHUP, request_stop)

    destination = (args.target, args.udp_port)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_working_device: str | None = None
    last_no_device_notice = 0.0
    last_send_error_notice = 0.0

    log(
        "forwarding verified ESP32-S3 "
        f"{SENSOR_USB_SERIAL} at {args.baud} baud to "
        f"{args.target}:{args.udp_port}"
    )

    try:
        while not stopping.is_set():
            devices = discover_devices(last_working_device)
            if not devices:
                now = time.monotonic()
                if now - last_no_device_notice >= 5.0:
                    log("waiting for the ESP32-S3 USB serial device")
                    last_no_device_notice = now
                stopping.wait(RECONNECT_DELAY_SECONDS)
                continue

            for device_path in devices:
                if stopping.is_set():
                    break

                try:
                    sensor = serial.Serial(
                        device_path,
                        baudrate=args.baud,
                        timeout=0.5,
                        write_timeout=0.5,
                    )
                except (OSError, serial.SerialException) as exc:
                    log(f"could not open {device_path}: {exc}")
                    continue

                saw_telemetry = False
                probe_deadline = time.monotonic() + PROBE_SECONDS
                log(f"connected to {device_path}")

                try:
                    while not stopping.is_set():
                        raw_record = sensor.read_until(b"\n", MAX_RECORD_BYTES + 1)
                        if not raw_record:
                            if saw_telemetry and not os.path.exists(device_path):
                                log(f"{device_path} disconnected")
                                break
                            if not saw_telemetry and time.monotonic() >= probe_deadline:
                                log(f"no HEADING telemetry on {device_path}; rescanning")
                                break
                            continue

                        record = sanitize_record(raw_record)
                        if record is None:
                            if not saw_telemetry and time.monotonic() >= probe_deadline:
                                log(f"no HEADING telemetry on {device_path}; rescanning")
                                break
                            continue

                        if not saw_telemetry:
                            saw_telemetry = True
                            last_working_device = device_path
                            log("telemetry detected")

                        try:
                            # A UDP datagram preserves the boundary of each raw sensor record.
                            udp_socket.sendto(record, destination)
                        except OSError as exc:
                            now = time.monotonic()
                            if now - last_send_error_notice >= 5.0:
                                log(f"UDP send failed (will keep retrying): {exc}")
                                last_send_error_notice = now
                except (OSError, serial.SerialException) as exc:
                    log(f"{device_path} disconnected: {exc}")
                finally:
                    sensor.close()

                # Re-discover immediately after the known sensor disconnects so a
                # newly assigned macOS device name is picked up without delay.
                if saw_telemetry:
                    break

            if not stopping.is_set():
                stopping.wait(RECONNECT_DELAY_SECONDS)
    finally:
        udp_socket.close()
        log("stopped")

    return 0


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
