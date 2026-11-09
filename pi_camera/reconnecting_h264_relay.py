#!/usr/bin/env python3
"""Keep one local H.264 viewer alive while the Pi stream reconnects.

ffplay exits when its TCP peer disappears.  This relay keeps ffplay's local
socket open and reconnects only the upstream Pi socket, so a camera-service
restart produces a short pause instead of closing the firefighter window.
"""

from __future__ import annotations

import argparse
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional


Log = Callable[[str], None]


def relay_reconnecting(
    downstream: socket.socket,
    upstream_host: str,
    upstream_port: int,
    *,
    stop_event: threading.Event,
    connect_timeout: float = 3.0,
    retry_delay: float = 0.5,
    log: Optional[Log] = None,
) -> None:
    """Forward bytes, reconnecting upstream until downstream closes or stopped."""

    logger = log or (lambda _message: None)
    next_warning = 0.0
    connected_once = False

    while not stop_event.is_set():
        try:
            upstream = socket.create_connection(
                (upstream_host, upstream_port), timeout=connect_timeout
            )
            upstream.settimeout(1.0)
            logger(
                "Pi H.264 stream reconnected"
                if connected_once
                else "Pi H.264 stream connected"
            )
            connected_once = True
            with upstream:
                while not stop_event.is_set():
                    try:
                        data = upstream.recv(65536)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    try:
                        downstream.sendall(data)
                    except (BrokenPipeError, ConnectionResetError):
                        # ffplay was closed deliberately (normally with q).
                        # Do not turn that into an endless reconnect loop.
                        return
            if not stop_event.is_set():
                logger("Pi stream paused; reconnecting...")
        except (ConnectionError, OSError) as error:
            now = time.monotonic()
            if now >= next_warning:
                logger(f"Pi stream unavailable ({error}); retrying...")
                next_warning = now + 5.0
        if stop_event.wait(retry_delay):
            return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream-host", required=True)
    parser.add_argument("--upstream-port", type=int, required=True)
    parser.add_argument("--port-file", required=True)
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--retry-delay", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.upstream_port <= 65535:
        raise SystemExit("upstream port must be from 1 to 65535")
    if args.connect_timeout <= 0 or args.retry_delay <= 0:
        raise SystemExit("timeouts must be positive")

    stop_event = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(0.5)
    local_port = listener.getsockname()[1]

    port_file = Path(args.port_file)
    temporary = port_file.with_name(f"{port_file.name}.{os.getpid()}.tmp")
    temporary.write_text(f"{local_port}\n", encoding="ascii")
    temporary.replace(port_file)
    print(f"Local reconnecting stream ready on 127.0.0.1:{local_port}", flush=True)

    downstream: Optional[socket.socket] = None
    try:
        while not stop_event.is_set():
            try:
                downstream, _address = listener.accept()
                break
            except socket.timeout:
                continue
        if downstream is None:
            return 0
        with downstream:
            relay_reconnecting(
                downstream,
                args.upstream_host,
                args.upstream_port,
                stop_event=stop_event,
                connect_timeout=args.connect_timeout,
                retry_delay=args.retry_delay,
                log=lambda message: print(message, flush=True),
            )
        return 0
    finally:
        listener.close()
        temporary.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
