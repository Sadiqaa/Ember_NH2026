#!/usr/bin/env python3

import struct
import unittest

from bridge_common import (
    OLED_PAYLOAD_BYTES,
    SERIAL_MAGIC,
    OledConverter,
    build_serial_packet,
    crc16_ccitt_false,
)


class ProtocolTests(unittest.TestCase):
    def test_standard_crc_vector(self) -> None:
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_packet_layout_and_crc(self) -> None:
        payload = bytes((index * 17) & 0xFF for index in range(OLED_PAYLOAD_BYTES))
        packet = build_serial_packet(0x1234, payload)
        self.assertEqual(len(packet), 4 + 4 + OLED_PAYLOAD_BYTES + 2)
        self.assertEqual(packet[:4], SERIAL_MAGIC)
        self.assertEqual(struct.unpack_from("<HH", packet, 4), (0x1234, 1024))
        self.assertEqual(packet[8:-2], payload)
        expected_crc = crc16_ccitt_false(packet[4:-2])
        self.assertEqual(struct.unpack_from("<H", packet, len(packet) - 2)[0], expected_crc)

    def test_rejects_wrong_payload_size(self) -> None:
        with self.assertRaises(ValueError):
            build_serial_packet(0, b"too short")


class ConverterTests(unittest.TestCase):
    def test_uniform_black_and_white(self) -> None:
        converter = OledConverter(256, 144)
        self.assertEqual(converter.convert(bytes(256 * 144)), bytes(1024))
        self.assertEqual(converter.convert(bytes([255]) * (256 * 144)), bytes([255]) * 1024)

    def test_crop_is_two_to_one_and_centered(self) -> None:
        converter = OledConverter(256, 144)
        self.assertEqual((converter.crop_x, converter.crop_y), (0, 8))
        self.assertEqual((converter.crop_width, converter.crop_height), (256, 128))

    def test_page_layout_for_horizontal_split(self) -> None:
        width, height = 128, 64
        source = bytearray(width * height)
        for y in range(32):
            source[y * width : (y + 1) * width] = bytes([255]) * width
        converter = OledConverter(width, height, dither=False)
        packed = converter.convert(source)
        self.assertEqual(packed[: 4 * width], bytes([255]) * (4 * width))
        self.assertEqual(packed[4 * width :], bytes(4 * width))

    def test_edge_mode_uniform_image_is_blank(self) -> None:
        converter = OledConverter(128, 64, edge_mode=True)
        self.assertEqual(converter.convert(bytes(128 * 64)), bytes(1024))
        converter = OledConverter(128, 64, edge_mode=True)
        self.assertEqual(
            converter.convert(bytes([255]) * (128 * 64)), bytes(1024)
        )

    def test_edge_mode_preserves_a_vertical_boundary(self) -> None:
        width, height = 128, 64
        source = bytearray(width * height)
        for y in range(height):
            source[y * width + 64 : (y + 1) * width] = bytes([255]) * 64
        packed = OledConverter(width, height, edge_mode=True).convert(source)
        lit = {
            (x, y)
            for y in range(height)
            for x in range(width)
            if packed[(y >> 3) * width + x] & (1 << (y & 7))
        }
        self.assertGreater(len(lit), 50)
        self.assertTrue(all(60 <= x <= 67 for x, _y in lit))


if __name__ == "__main__":
    unittest.main()
