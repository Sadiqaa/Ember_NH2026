#!/usr/bin/env python3
"""Shared OLED conversion and serial transport for the camera bridges."""

from __future__ import annotations

import logging
import os
import select
import struct
import termios
import threading
import time
from typing import Callable, Optional, Tuple


OLED_WIDTH = 128
OLED_HEIGHT = 64
OLED_PAYLOAD_BYTES = OLED_WIDTH * OLED_HEIGHT // 8
SERIAL_MAGIC = b"VSR1"
SERIAL_BAUD = 921_600

# 4x4 Bayer matrix. The values describe the order in which pixels illuminate.
_BAYER_4X4 = (
    0, 8, 2, 10,
    12, 4, 14, 6,
    3, 11, 1, 9,
    15, 7, 13, 5,
)


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    """Return CRC-16/CCITT-FALSE (poly 0x1021, init 0xffff)."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_serial_packet(sequence: int, payload: bytes) -> bytes:
    """Wrap one SSD1306 page-layout frame in the VSR1 wire protocol."""

    if len(payload) != OLED_PAYLOAD_BYTES:
        raise ValueError(
            f"OLED payload must be {OLED_PAYLOAD_BYTES} bytes, got {len(payload)}"
        )
    header = struct.pack("<HH", sequence & 0xFFFF, len(payload))
    checksum = crc16_ccitt_false(header + payload)
    return SERIAL_MAGIC + header + payload + struct.pack("<H", checksum)


def _percentile(histogram: list[int], rank: int) -> int:
    running = 0
    for value, count in enumerate(histogram):
        running += count
        if running > rank:
            return value
    return 255


class OledConverter:
    """Crop, downsample and dither an 8-bit luma plane to SSD1306 layout."""

    def __init__(
        self,
        source_width: int,
        source_height: int,
        *,
        source_stride: Optional[int] = None,
        dither: bool = True,
        temporal_weight: float = 1.0,
        contrast_margin: int = 0,
        edge_mode: bool = False,
    ) -> None:
        if source_width < OLED_WIDTH or source_height < OLED_HEIGHT:
            raise ValueError("source image must be at least 128x64")
        self.source_width = source_width
        self.source_height = source_height
        self.source_stride = source_stride or source_width
        self.dither = dither
        if not 0.0 < temporal_weight <= 1.0:
            raise ValueError("temporal_weight must be greater than 0 and at most 1")
        if not 0 <= contrast_margin <= 64:
            raise ValueError("contrast_margin must be from 0 to 64")
        self.temporal_weight = temporal_weight
        self.contrast_margin = contrast_margin
        self.edge_mode = edge_mode
        self._previous_grey: Optional[bytearray] = None
        self._previous_edges: Optional[bytearray] = None
        self._black_level: Optional[int] = None
        self._white_level: Optional[int] = None

        # Centre-crop without stretching so circles remain circular. The OLED is 2:1.
        if source_width * OLED_HEIGHT >= source_height * OLED_WIDTH:
            crop_height = source_height
            crop_width = source_height * OLED_WIDTH // OLED_HEIGHT
        else:
            crop_width = source_width
            crop_height = source_width * OLED_HEIGHT // OLED_WIDTH
        self.crop_x = (source_width - crop_width) // 2
        self.crop_y = (source_height - crop_height) // 2
        self.crop_width = crop_width
        self.crop_height = crop_height

        self._sample_x = self._make_sample_pairs(
            self.crop_x, self.crop_width, OLED_WIDTH
        )
        self._sample_y = self._make_sample_pairs(
            self.crop_y, self.crop_height, OLED_HEIGHT
        )

    @staticmethod
    def _make_sample_pairs(start: int, span: int, count: int) -> list[Tuple[int, int]]:
        pairs: list[Tuple[int, int]] = []
        for index in range(count):
            lower = start + index * span // count
            upper = start + (index + 1) * span // count
            width = max(1, upper - lower)
            first = min(upper - 1, lower + width // 4)
            second = min(upper - 1, lower + (3 * width) // 4)
            pairs.append((first, second))
        return pairs

    def convert(self, luma: bytes | bytearray | memoryview) -> bytes:
        """Convert a tightly addressed luma plane; chroma data may follow it."""

        source = memoryview(luma).cast("B")
        required = self.source_stride * self.source_height
        if len(source) < required:
            raise ValueError(f"luma buffer needs {required} bytes, got {len(source)}")

        grey = bytearray(OLED_WIDTH * OLED_HEIGHT)
        histogram = [0] * 256
        previous = self._previous_grey
        current_weight = round(self.temporal_weight * 256)
        previous_weight = 256 - current_weight
        destination = 0
        for y_first, y_second in self._sample_y:
            row_first = y_first * self.source_stride
            row_second = y_second * self.source_stride
            for x_first, x_second in self._sample_x:
                value = (
                    source[row_first + x_first]
                    + source[row_first + x_second]
                    + source[row_second + x_first]
                    + source[row_second + x_second]
                ) >> 2
                if previous is not None and previous_weight:
                    value = (
                        value * current_weight
                        + previous[destination] * previous_weight
                        + 128
                    ) >> 8
                grey[destination] = value
                histogram[value] += 1
                destination += 1
        self._previous_grey = grey

        if self.edge_mode:
            return self._convert_edges(grey)

        # Stretch the useful range automatically. This keeps indoor scenes legible even
        # when the camera's luma occupies only part of 0..255.
        pixel_count = len(grey)
        target_black = _percentile(histogram, pixel_count * 2 // 100)
        target_white = _percentile(histogram, pixel_count * 98 // 100)
        if target_white - target_black < 24:
            midpoint = (target_white + target_black) // 2
            target_black = max(0, midpoint - 12)
            target_white = min(255, midpoint + 12)

        # Expand beyond the scene percentiles instead of mapping them directly
        # to pure black and white. This preserves more facial and fabric detail.
        target_black = max(0, target_black - self.contrast_margin)
        target_white = min(255, target_white + self.contrast_margin)

        if self._black_level is None or self._white_level is None:
            black, white = target_black, target_white
        else:
            black = (
                target_black * current_weight
                + self._black_level * previous_weight
                + 128
            ) >> 8
            white = (
                target_white * current_weight
                + self._white_level * previous_weight
                + 128
            ) >> 8
        self._black_level, self._white_level = black, white
        span = max(1, white - black)

        packed = bytearray(OLED_PAYLOAD_BYTES)
        for y in range(OLED_HEIGHT):
            source_row = y * OLED_WIDTH
            page_row = (y >> 3) * OLED_WIDTH
            bit = 1 << (y & 7)
            matrix_row = (y & 3) << 2
            for x in range(OLED_WIDTH):
                value = grey[source_row + x]
                if self.dither:
                    # Compare without division: both sides have a 0..255 scale.
                    threshold = _BAYER_4X4[matrix_row + (x & 3)] * 16 + 8
                    illuminated = (value - black) * 255 > threshold * span
                else:
                    illuminated = (value - black) * 2 > span
                if illuminated:
                    packed[page_row + x] |= bit
        return bytes(packed)

    def _convert_edges(self, grey: bytearray) -> bytes:
        """Render stable white-on-black outlines for the 1-bit HUD display."""

        width, height = OLED_WIDTH, OLED_HEIGHT
        horizontal = bytearray(len(grey))
        blurred = bytearray(len(grey))

        # Small separable [1, 2, 1] blur suppresses sensor noise before taking
        # gradients, while retaining object and facial boundaries.
        for y in range(height):
            row = y * width
            horizontal[row] = grey[row]
            for x in range(1, width - 1):
                index = row + x
                horizontal[index] = (
                    grey[index - 1] + (grey[index] << 1) + grey[index + 1]
                ) >> 2
            horizontal[row + width - 1] = grey[row + width - 1]

        blurred[:width] = horizontal[:width]
        blurred[(height - 1) * width :] = horizontal[(height - 1) * width :]
        for y in range(1, height - 1):
            row = y * width
            above = row - width
            below = row + width
            for x in range(width):
                index = row + x
                blurred[index] = (
                    horizontal[above + x]
                    + (horizontal[index] << 1)
                    + horizontal[below + x]
                ) >> 2

        magnitudes = bytearray(len(grey))
        histogram = [0] * 256
        sample_count = 0
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                index = row + x
                magnitude = (
                    abs(blurred[index + 1] - blurred[index - 1])
                    + abs(blurred[index + width] - blurred[index - width])
                )
                magnitude = min(255, magnitude)
                magnitudes[index] = magnitude
                histogram[magnitude] += 1
                sample_count += 1

        # Keep approximately the strongest 16% of gradients, with bounds that
        # prevent flat scenes becoming noisy or low-contrast scenes disappearing.
        threshold = _percentile(histogram, sample_count * 84 // 100)
        threshold = max(24, min(56, threshold))
        hold_threshold = threshold * 7 // 10
        previous = self._previous_edges
        edges = bytearray(len(grey))

        packed = bytearray(OLED_PAYLOAD_BYTES)
        for y in range(1, height - 1):
            row = y * width
            page_row = (y >> 3) * width
            bit = 1 << (y & 7)
            for x in range(1, width - 1):
                index = row + x
                illuminated = magnitudes[index] >= threshold
                if (
                    not illuminated
                    and previous is not None
                    and previous[index]
                    and magnitudes[index] >= hold_threshold
                ):
                    illuminated = True
                if illuminated:
                    edges[index] = 1
                    packed[page_row + x] |= bit

        self._previous_edges = edges
        return bytes(packed)


class LatestValue:
    """Single-slot, thread-safe mailbox that always retains the newest value."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._generation = 0
        self._value: Optional[bytes] = None

    def publish(self, value: bytes) -> int:
        with self._condition:
            self._generation += 1
            self._value = value
            self._condition.notify_all()
            return self._generation

    def wait_for_new(
        self, generation: int, timeout: float
    ) -> tuple[int, Optional[bytes]]:
        with self._condition:
            if self._generation == generation:
                self._condition.wait(timeout)
            return self._generation, self._value


def _open_serial(path: str) -> int:
    baud = getattr(termios, "B921600", None)
    if baud is None:
        raise RuntimeError("this Python/OS does not expose the 921600-baud termios rate")

    descriptor = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attributes = termios.tcgetattr(descriptor)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attributes[3] = 0
        attributes[4] = baud
        attributes[5] = baud
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
        termios.tcflush(descriptor, termios.TCIOFLUSH)
        os.set_blocking(descriptor, True)
        return descriptor
    except Exception:
        os.close(descriptor)
        raise


class SerialFrameSender(threading.Thread):
    """Send only fresh OLED frames, reconnecting harmlessly when USB is absent."""

    def __init__(
        self,
        device: str,
        stop_event: threading.Event,
        *,
        logger: Optional[logging.Logger] = None,
        receive_callback: Optional[Callable[[bytes], None]] = None,
    ) -> None:
        super().__init__(name="oled-serial", daemon=True)
        self.device = device
        self.stop_event = stop_event
        self.log = logger or logging.getLogger(__name__)
        self.receive_callback = receive_callback
        self.mailbox = LatestValue()

    def submit(self, payload: bytes) -> None:
        if len(payload) != OLED_PAYLOAD_BYTES:
            raise ValueError("invalid OLED payload size")
        self.mailbox.publish(payload)

    @staticmethod
    def _write_all(descriptor: int, packet: bytes) -> None:
        view = memoryview(packet)
        written = 0
        while written < len(packet):
            count = os.write(descriptor, view[written:])
            if count <= 0:
                raise OSError("serial write returned no data")
            written += count

    def _read_available(self, descriptor: int) -> None:
        if self.receive_callback is None:
            return
        while True:
            readable, _, _ = select.select([descriptor], [], [], 0)
            if not readable:
                return
            data = os.read(descriptor, 4096)
            if not data:
                return
            self.receive_callback(data)

    def run(self) -> None:
        descriptor: Optional[int] = None
        generation = 0
        pending: Optional[bytes] = None
        sequence = 0
        next_warning = 0.0

        try:
            while not self.stop_event.is_set():
                new_generation, newest = self.mailbox.wait_for_new(generation, 1.0)
                if new_generation != generation and newest is not None:
                    generation = new_generation
                    pending = newest

                if descriptor is None:
                    try:
                        descriptor = _open_serial(self.device)
                        self.log.info(
                            "OLED serial connected: %s at %d baud",
                            self.device,
                            SERIAL_BAUD,
                        )
                    except OSError as error:
                        now = time.monotonic()
                        if now >= next_warning:
                            self.log.warning(
                                "OLED serial unavailable at %s (%s); retrying",
                                self.device,
                                error,
                            )
                            next_warning = now + 10.0
                        continue

                try:
                    self._read_available(descriptor)
                except OSError as error:
                    self.log.warning("OLED serial read failed (%s)", error)
                    os.close(descriptor)
                    descriptor = None
                    continue

                if pending is None:
                    continue

                try:
                    packet = build_serial_packet(sequence, pending)
                    self._write_all(descriptor, packet)
                    self._read_available(descriptor)
                    sequence = (sequence + 1) & 0xFFFF
                    pending = None
                except OSError as error:
                    self.log.warning("OLED serial disconnected (%s)", error)
                    os.close(descriptor)
                    descriptor = None
        finally:
            if descriptor is not None:
                os.close(descriptor)
