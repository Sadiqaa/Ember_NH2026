#!/usr/bin/env python3
"""Dependency-free camera bridge using rpicam-vid's raw YUV420 output."""

from __future__ import annotations

import argparse
import logging
import signal
import socket
import subprocess
import threading
import time
from typing import BinaryIO, Optional

from bridge_common import LatestValue, OledConverter, SerialFrameSender


LOG = logging.getLogger("raw-camera-oled")


def make_server_socket(address: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((address, port))
    server.listen(1)
    server.settimeout(1.0)
    return server


def raw_video_server(
    server: socket.socket,
    frames: LatestValue,
    stop_event: threading.Event,
) -> None:
    """Serve complete raw frames; a slow viewer skips to the freshest frame."""

    while not stop_event.is_set():
        try:
            client, peer = server.accept()
        except socket.timeout:
            continue
        except OSError:
            break

        LOG.info("Mac raw-video viewer connected from %s:%s", *peer[:2])
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.settimeout(2.0)
        generation = 0
        try:
            while not stop_event.is_set():
                new_generation, frame = frames.wait_for_new(generation, 1.0)
                if new_generation == generation or frame is None:
                    continue
                generation = new_generation
                client.sendall(frame)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError) as error:
            LOG.info("Mac raw-video viewer disconnected (%s)", error)
        finally:
            client.close()


def read_exact(stream: BinaryIO, size: int) -> Optional[bytes]:
    buffer = bytearray(size)
    view = memoryview(buffer)
    offset = 0
    while offset < size:
        count = stream.readinto(view[offset:])
        if count is None:
            continue
        if count == 0:
            return None
        offset += count
    return bytes(buffer)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=576)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--oled-fps", type=float, default=15.0)
    parser.add_argument("--serial-device", default="/dev/nathacks-oled")
    parser.add_argument("--rotation", type=int, choices=(0, 180), default=180)
    parser.add_argument(
        "--no-dither", action="store_true", help="use a hard monochrome threshold"
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.width < 128 or args.height < 64 or args.width % 2 or args.height % 2:
        raise SystemExit("width/height must be even and at least 128x64")
    if not 1 <= args.camera_fps <= 60:
        raise SystemExit("camera-fps must be from 1 to 60")
    if not 1 <= args.oled_fps <= 20:
        raise SystemExit("oled-fps must be from 1 to 20")
    if not 1 <= args.port <= 65535:
        raise SystemExit("port must be from 1 to 65535")


def main() -> int:
    args = parse_args()
    validate_args(args)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    server = make_server_socket(args.bind, args.port)
    frames = LatestValue()
    network_thread = threading.Thread(
        target=raw_video_server,
        args=(server, frames, stop_event),
        name="raw-video-server",
        daemon=True,
    )
    network_thread.start()

    serial_sender = SerialFrameSender(args.serial_device, stop_event, logger=LOG)
    serial_sender.start()
    converter = OledConverter(
        args.width, args.height, dither=not args.no_dither
    )

    command = [
        "/usr/bin/rpicam-vid",
        "--nopreview",
        "--timeout", "0",
        "--width", str(args.width),
        "--height", str(args.height),
        "--framerate", str(args.camera_fps),
        "--rotation", str(args.rotation),
        "--codec", "yuv420",
        "--flush",
        "--denoise", "cdn_fast",
        "--sharpness", "1.25",
        "--no-raw",
        "--output", "-",
    ]
    LOG.info(
        "starting camera: %dx%d at %d fps; OLED %.1f fps",
        args.width,
        args.height,
        args.camera_fps,
        args.oled_fps,
    )
    camera = subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=0)
    assert camera.stdout is not None

    frame_bytes = args.width * args.height * 3 // 2
    next_oled = 0.0
    oled_period = 1.0 / args.oled_fps
    frame_counter = 0
    stats_started = time.monotonic()

    try:
        while not stop_event.is_set():
            frame = read_exact(camera.stdout, frame_bytes)
            if frame is None:
                if stop_event.is_set():
                    break
                raise RuntimeError(
                    f"rpicam-vid ended unexpectedly with status {camera.poll()}"
                )
            frames.publish(frame)
            frame_counter += 1

            now = time.monotonic()
            if now >= next_oled:
                serial_sender.submit(converter.convert(frame))
                next_oled = max(next_oled + oled_period, now)

            if now - stats_started >= 10.0:
                LOG.info(
                    "camera throughput %.1f fps",
                    frame_counter / (now - stats_started),
                )
                frame_counter = 0
                stats_started = now
    finally:
        stop_event.set()
        server.close()
        if camera.poll() is None:
            camera.send_signal(signal.SIGINT)
            try:
                camera.wait(timeout=4)
            except subprocess.TimeoutExpired:
                camera.terminate()
                try:
                    camera.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    camera.kill()
        network_thread.join(timeout=2)
        serial_sender.join(timeout=2)
        LOG.info("camera/OLED bridge stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
