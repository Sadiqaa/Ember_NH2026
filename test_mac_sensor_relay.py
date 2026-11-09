#!/usr/bin/env python3

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import mac_sensor_relay


def usb_port(
    device: str,
    *,
    vid: int = mac_sensor_relay.SENSOR_USB_VID,
    pid: int = mac_sensor_relay.SENSOR_USB_PID,
    serial_number: str = mac_sensor_relay.SENSOR_USB_SERIAL,
):
    return SimpleNamespace(
        device=device,
        vid=vid,
        pid=pid,
        serial_number=serial_number,
    )


class SensorRelayTests(unittest.TestCase):
    def test_discovery_accepts_only_verified_sensor_esp(self) -> None:
        ports = [
            usb_port("/dev/cu.usbmodem2101"),
            usb_port("/dev/cu.wrong-pid", pid=0x0001),
            usb_port("/dev/cu.wrong-serial", serial_number="OTHER"),
            usb_port("/dev/tty.usbmodem2101"),
            SimpleNamespace(
                device="/dev/cu.Bluetooth-Incoming-Port",
                vid=None,
                pid=None,
                serial_number=None,
            ),
        ]
        with patch.object(mac_sensor_relay.list_ports, "comports", return_value=ports):
            self.assertEqual(
                mac_sensor_relay.discover_devices(None),
                ["/dev/cu.usbmodem2101"],
            )

    def test_last_working_device_is_probed_first(self) -> None:
        ports = [
            usb_port("/dev/cu.usbmodem3101"),
            usb_port("/dev/cu.usbmodem2101"),
        ]
        with patch.object(mac_sensor_relay.list_ports, "comports", return_value=ports):
            self.assertEqual(
                mac_sensor_relay.discover_devices("/dev/cu.usbmodem3101"),
                ["/dev/cu.usbmodem3101", "/dev/cu.usbmodem2101"],
            )

    def test_sanitizer_keeps_sensor_fields_and_strips_external_flame(self) -> None:
        self.assertEqual(
            mac_sensor_relay.sanitize_record(
                b"HEADING:17.5,AIR:24.1,FLAME:3996,GAS:1315\r\n"
            ),
            b"HEADING:17.5,AIR:24.1,GAS:1315",
        )

    def test_sanitizer_requires_valid_heading(self) -> None:
        self.assertIsNone(
            mac_sensor_relay.sanitize_record(b"AIR:24.1,GAS:1315\n")
        )
        self.assertIsNone(
            mac_sensor_relay.sanitize_record(b"HEADING:ERR,AIR:24.1,GAS:1315\n")
        )

    def test_sanitizer_omits_bad_optional_values(self) -> None:
        self.assertEqual(
            mac_sensor_relay.sanitize_record(
                b"HEADING:360,AIR:nan,GAS:5000,FLAME:2000\n"
            ),
            b"HEADING:0",
        )

    def test_sanitizer_rejects_binary_or_embedded_records(self) -> None:
        self.assertIsNone(mac_sensor_relay.sanitize_record(b"\xffHEADING:0\n"))
        self.assertIsNone(
            mac_sensor_relay.sanitize_record(b"HEADING:0\nAIR:24.1")
        )

if __name__ == "__main__":
    unittest.main()
