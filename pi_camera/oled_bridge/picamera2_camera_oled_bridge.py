#!/usr/bin/env python3
"""1080p H.264 + OLED bridge using one Picamera2 camera configuration."""

from __future__ import annotations

import argparse
import logging
import signal
import socket
import threading
import time
from collections import deque
from typing import Optional

from bridge_common import OledConverter, SerialFrameSender
from visor_hud import VisorHud, normalize_ir_strength


LOG = logging.getLogger("picamera2-camera-oled")


class H264Hub:
    """Bounded encoder-to-network queue that never blocks the camera thread."""

    def __init__(self, max_chunks: int = 12) -> None:
        self._condition = threading.Condition()
        self._chunks: deque[bytes] = deque()
        self._connected = False
        self._waiting_for_keyframe = True
        self._max_chunks = max_chunks

    def set_connected(self, connected: bool) -> None:
        with self._condition:
            self._connected = connected
            self._chunks.clear()
            self._waiting_for_keyframe = connected
            self._condition.notify_all()

    def publish(self, chunk: bytes, keyframe: bool) -> None:
        with self._condition:
            if not self._connected:
                return
            if self._waiting_for_keyframe:
                if not keyframe:
                    return
                self._waiting_for_keyframe = False
            if len(self._chunks) >= self._max_chunks:
                # The viewer is too slow. Discard the backlog and resume only at
                # an inline SPS/PPS + I-frame, keeping latency bounded.
                self._chunks.clear()
                if not keyframe:
                    self._waiting_for_keyframe = True
                    return
            self._chunks.append(chunk)
            self._condition.notify()

    def take(self, timeout: float) -> Optional[bytes]:
        with self._condition:
            if not self._chunks:
                self._condition.wait(timeout)
            if self._chunks:
                return self._chunks.popleft()
            return None


def make_h264_output(hub: H264Hub, output_base: type) -> object:
    """Build an Output subclass while keeping Picamera2 an optional import."""

    class H264Output(output_base):
        """Picamera2 output that preserves encoder keyframe metadata."""

        def __init__(self) -> None:
            super().__init__()

        def outputframe(
            self,
            frame: bytes | bytearray | memoryview,
            keyframe: bool = True,
            timestamp: Optional[int] = None,
            packet: object = None,
            audio: bool = False,
        ) -> None:
            if audio:
                raise RuntimeError("H264Output does not support audio")
            if self.recording:
                hub.publish(bytes(frame), keyframe)

    return H264Output()


def make_server_socket(address: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((address, port))
    server.listen(1)
    server.settimeout(1.0)
    return server


def h264_video_server(
    server: socket.socket,
    hub: H264Hub,
    encoder: object,
    stop_event: threading.Event,
) -> None:
    while not stop_event.is_set():
        try:
            client, peer = server.accept()
        except socket.timeout:
            continue
        except OSError:
            break

        LOG.info("Mac H.264 viewer connected from %s:%s", *peer[:2])
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.settimeout(2.0)
        hub.set_connected(True)
        # Start every viewer on a fresh SPS/PPS + I-frame instead of making it
        # decode arbitrary data from the middle of the current GOP.
        encoder.force_key_frame()
        try:
            while not stop_event.is_set():
                chunk = hub.take(1.0)
                if chunk is not None:
                    client.sendall(chunk)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError) as error:
            LOG.info("Mac H.264 viewer disconnected (%s)", error)
        finally:
            hub.set_connected(False)
            client.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--bitrate", type=int, default=18_000_000)
    parser.add_argument("--keyframe-interval", type=int, default=15)
    parser.add_argument("--lores-width", type=int, default=256)
    parser.add_argument("--lores-height", type=int, default=144)
    parser.add_argument("--oled-fps", type=float, default=15.0)
    parser.add_argument("--temporal-weight", type=float, default=0.65)
    parser.add_argument("--contrast-margin", type=int, default=16)
    parser.add_argument("--serial-device", default="/dev/nathacks-oled")
    parser.add_argument("--telemetry-udp-port", type=int, default=8892)
    parser.add_argument(
        "--sensor-port",
        default="auto",
        help="direct Pi sensor serial port, 'auto', or empty to disable",
    )
    parser.add_argument("--sensor-baud", type=int, default=115200)
    parser.add_argument(
        "--target-db",
        help="SQLite file used to persist IR target-lock sightings",
    )
    parser.add_argument(
        "--ir-ambient",
        type=float,
        help="GPIO10 raw value with IR emitter off/receiver covered",
    )
    parser.add_argument(
        "--ir-target",
        type=float,
        help="GPIO10 raw value with IR emitter on and aimed at the receiver",
    )
    parser.add_argument("--fake-sensors", action="store_true")
    parser.add_argument("--no-hud", action="store_true")
    parser.add_argument("--rotation", type=int, choices=(0, 180), default=180)
    parser.add_argument("--no-dither", action="store_true")
    parser.add_argument(
        "--edge-mode",
        action="store_true",
        help="render crisp white-on-black outlines instead of photographic halftones",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    dimensions = (args.width, args.height, args.lores_width, args.lores_height)
    if any(value <= 0 or value % 2 for value in dimensions):
        raise SystemExit("all stream dimensions must be positive even numbers")
    if args.lores_width < 128 or args.lores_height < 64:
        raise SystemExit("lores stream must be at least 128x64")
    if not 1 <= args.camera_fps <= 60:
        raise SystemExit("camera-fps must be from 1 to 60")
    if not 1 <= args.oled_fps <= 20:
        raise SystemExit("oled-fps must be from 1 to 20")
    if not 0.0 < args.temporal_weight <= 1.0:
        raise SystemExit("temporal-weight must be greater than 0 and at most 1")
    if not 0 <= args.contrast_margin <= 64:
        raise SystemExit("contrast-margin must be from 0 to 64")
    if not 1 <= args.telemetry_udp_port <= 65535:
        raise SystemExit("telemetry-udp-port must be from 1 to 65535")
    if args.sensor_baud <= 0:
        raise SystemExit("sensor-baud must be positive")
    if (args.ir_ambient is None) != (args.ir_target is None):
        raise SystemExit("--ir-ambient and --ir-target must be supplied together")
    if args.ir_ambient is not None and args.ir_target is not None:
        try:
            normalize_ir_strength(args.ir_ambient, args.ir_ambient, args.ir_target)
        except ValueError as error:
            raise SystemExit(str(error)) from error


def main() -> int:
    args = parse_args()
    validate_args(args)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    try:
        from libcamera import Transform
        from picamera2 import MappedArray, Picamera2
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import Output
    except ModuleNotFoundError as error:
        raise SystemExit(
            "Picamera2 is not installed. Install the Raspberry Pi OS package "
            "with: sudo apt install --no-install-recommends python3-picamera2"
        ) from error

    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    server = make_server_socket(args.bind, args.port)
    h264_hub = H264Hub()
    hud = None
    if not args.no_hud:
        hud = VisorHud(
            udp_port=args.telemetry_udp_port,
            sensor_port=args.sensor_port or None,
            sensor_baud=args.sensor_baud,
            display_serial_device=args.serial_device,
            fake=args.fake_sensors,
            ir_ambient_raw=args.ir_ambient,
            ir_target_raw=args.ir_target,
            target_db_path=args.target_db,
        )
        hud.start()
    serial_sender = SerialFrameSender(
        args.serial_device,
        stop_event,
        logger=LOG,
        receive_callback=hud.feed_bytes if hud is not None else None,
    )
    serial_sender.start()
    converter = OledConverter(
        args.lores_width,
        args.lores_height,
        dither=not args.no_dither,
        temporal_weight=args.temporal_weight,
        contrast_margin=args.contrast_margin,
        edge_mode=args.edge_mode,
    )

    camera = Picamera2()
    transform = Transform(hflip=args.rotation == 180, vflip=args.rotation == 180)
    main_format = "RGB888" if hud is not None else "YUV420"
    configuration = camera.create_video_configuration(
        main={"size": (args.width, args.height), "format": main_format},
        lores={
            "size": (args.lores_width, args.lores_height),
            "format": "YUV420",
        },
        transform=transform,
        controls={"FrameRate": float(args.camera_fps), "Sharpness": 1.25},
        buffer_count=6,
        encode="main",
        display=None,
        raw=None,
        queue=False,
    )
    camera.configure(configuration)
    if hud is not None:
        def draw_hud(request: object) -> None:
            with MappedArray(request, "main") as mapped:
                hud.draw_rgb(mapped.array)

        camera.pre_callback = draw_hud

    encoder = H264Encoder(
        bitrate=args.bitrate,
        repeat=True,
        iperiod=args.keyframe_interval,
        framerate=args.camera_fps,
        enable_sps_framerate=True,
        profile="high",
    )
    output = make_h264_output(h264_hub, Output)

    network_thread = threading.Thread(
        target=h264_video_server,
        args=(server, h264_hub, encoder, stop_event),
        name="h264-video-server",
        daemon=True,
    )
    network_thread.start()

    LOG.info(
        "starting single-capture bridge: H.264 %dx%d at %d fps; OLED %.1f fps",
        args.width,
        args.height,
        args.camera_fps,
        args.oled_fps,
    )
    camera.start_recording(encoder, output)
    period = 1.0 / args.oled_fps
    next_frame = time.monotonic()

    try:
        while not stop_event.is_set():
            now = time.monotonic()
            if now < next_frame:
                stop_event.wait(next_frame - now)
                continue
            yuv = camera.capture_array("lores")
            luma = yuv[: args.lores_height, : args.lores_width]
            payload = converter.convert(luma.tobytes(order="C"))
            if hud is not None:
                payload = hud.draw_oled(payload)
            serial_sender.submit(payload)
            now = time.monotonic()
            next_frame = max(next_frame + period, now)
    finally:
        stop_event.set()
        server.close()
        camera.stop_recording()
        camera.close()
        network_thread.join(timeout=2)
        serial_sender.join(timeout=2)
        if hud is not None:
            hud.stop()
        LOG.info("camera/OLED bridge stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
