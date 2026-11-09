#!/usr/bin/env python3

import socket
import threading
import time
import unittest

from reconnecting_h264_relay import relay_reconnecting


class ReconnectingRelayTests(unittest.TestCase):
    def test_upstream_disconnect_is_hidden_from_downstream(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen(2)
        port = listener.getsockname()[1]

        def upstream() -> None:
            try:
                for payload in (b"first", b"second"):
                    connection, _address = listener.accept()
                    with connection:
                        connection.sendall(payload)
                    time.sleep(0.03)
            finally:
                listener.close()

        upstream_thread = threading.Thread(target=upstream, daemon=True)
        upstream_thread.start()

        relay_socket, client_socket = socket.socketpair()
        stop = threading.Event()
        relay_thread = threading.Thread(
            target=relay_reconnecting,
            args=(relay_socket, "127.0.0.1", port),
            kwargs={
                "stop_event": stop,
                "connect_timeout": 0.2,
                "retry_delay": 0.01,
            },
            daemon=True,
        )
        relay_thread.start()

        client_socket.settimeout(2.0)
        received = bytearray()
        while len(received) < len(b"firstsecond"):
            received.extend(client_socket.recv(64))

        self.assertEqual(bytes(received), b"firstsecond")
        stop.set()
        relay_socket.close()
        client_socket.close()
        relay_thread.join(timeout=1.0)
        upstream_thread.join(timeout=1.0)


if __name__ == "__main__":
    unittest.main()
