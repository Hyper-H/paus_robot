from __future__ import annotations

from collections import deque
import math
import select
import socket
import threading
import time

from .frames import NormalizedFrame, normalize_packet
from . import protocol_v2


class FrameServer:
    """Single-client USFR v2 server with a one-frame handoff to the ROS thread."""

    def __init__(self, host: str, port: int, stale_timeout_s: float = 1.0) -> None:
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0:
            raise ValueError("stale_timeout_s must be finite and positive")
        self.host = host
        self.port = port
        self.stale_timeout_s = stale_timeout_s
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._server: socket.socket | None = None
        self._connection: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._latest: NormalizedFrame | None = None
        self._session: str | None = None
        self._sequence: int | None = None
        self._frame_times: deque[float] = deque(maxlen=5000)
        self._state = "STOPPED"
        self._last_error = ""
        self._received = 0
        self._acknowledged = 0
        self._skipped = 0
        self._overwritten = 0
        self._rejected = 0
        self._disconnects = 0
        self._last_received_monotonic: float | None = None
        self._connected_monotonic: float | None = None
        self._last_metadata: dict = {}

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("server already started")
        server = socket.create_server((self.host, self.port), backlog=1, reuse_port=False)
        server.settimeout(0.2)
        self._server = server
        self.port = server.getsockname()[1]
        self._state = "LISTENING"
        self._thread = threading.Thread(target=self._run, name="usfr-v2-receiver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        connection = self._connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._server is not None:
            self._server.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def pop_latest(self) -> NormalizedFrame | None:
        with self._lock:
            self._expire_locked(time.monotonic())
            frame, self._latest = self._latest, None
            return frame

    def _expire_locked(self, now: float) -> None:
        if self._state not in {"CONNECTED", "STREAMING", "STALE"}:
            self._latest = None
            return
        origin = self._last_received_monotonic or self._connected_monotonic
        if origin is not None and now - origin >= self.stale_timeout_s:
            self._state = "STALE"
            self._latest = None

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            self._expire_locked(now)
            while self._frame_times and self._frame_times[0] < now - 2.0:
                self._frame_times.popleft()
            return {
                "state": self._state,
                "timed_out": self._state == "STALE",
                "stale_timeout_s": self.stale_timeout_s,
                "listen_host": self.host,
                "listen_port": self.port,
                "received": self._received,
                "acknowledged": self._acknowledged,
                "skipped_sequences": self._skipped,
                "overwritten": self._overwritten,
                "rejected": self._rejected,
                "disconnects": self._disconnects,
                "fps_2s": len(self._frame_times) / 2.0,
                "latest_age_s": (
                    None if self._last_received_monotonic is None else max(0.0, now - self._last_received_monotonic)
                ),
                "last_error": self._last_error,
                "frame_metadata": dict(self._last_metadata),
            }

    def _run(self) -> None:
        assert self._server is not None
        while not self._stop.is_set():
            try:
                connection, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                if not self._stop.is_set():
                    with self._lock:
                        self._state = "ERROR"
                        self._last_error = "listener closed unexpectedly"
                break
            with connection:
                self._connection = connection
                connection.settimeout(0.1)
                with self._lock:
                    self._state = "CONNECTED"
                    self._last_error = ""
                    self._connected_monotonic = time.monotonic()
                    self._last_received_monotonic = None
                    self._latest = None
                    self._last_metadata = {}
                try:
                    self._receive(connection)
                except (protocol_v2.ProtocolError, TimeoutError, EOFError, OSError, ValueError, TypeError, OverflowError) as exc:
                    if not self._stop.is_set():
                        with self._lock:
                            self._rejected += 1
                            self._last_error = str(exc)
                finally:
                    self._connection = None
                    with self._lock:
                        self._disconnects += 1
                        self._latest = None
                        self._connected_monotonic = None
                        if not self._stop.is_set():
                            self._state = "LISTENING"
        with self._lock:
            self._state = "STOPPED"

    def _receive(self, connection: socket.socket) -> None:
        while not self._stop.is_set():
            readable, _, _ = select.select([connection], [], [], 0.2)
            if not readable:
                with self._lock:
                    self._expire_locked(time.monotonic())
                continue
            if not connection.recv(1, socket.MSG_PEEK):
                return
            packet = protocol_v2.read_packet(connection, timeout_s=1.0, stop_event=self._stop)
            frame = normalize_packet(packet)
            session = frame.header["session_id"]
            sequence = frame.header["sequence_id"]
            with self._lock:
                if session == self._session and self._sequence is not None:
                    if sequence <= self._sequence:
                        raise protocol_v2.ProtocolError("duplicate or out-of-order sequence")
                    self._skipped += sequence - self._sequence - 1
                if self._latest is not None:
                    self._overwritten += 1
                self._session, self._sequence = session, sequence
                self._latest = frame
                self._last_metadata = {
                    key: frame.header[key]
                    for key in (
                        "session_id", "sequence_id", "width", "height", "encoding",
                        "roi_confirmed", "capture_size", "roi_pixels", "profile_revision",
                        "imaging_context", "display_scale_mm_per_pixel", "geometry_valid",
                    )
                    if key in frame.header
                }
                self._received += 1
                self._last_received_monotonic = frame.received_monotonic_s
                self._frame_times.append(frame.received_monotonic_s)
                self._state = "STREAMING"
                self._last_error = ""
            connection.sendall(protocol_v2.acknowledgement(frame.header))
            with self._lock:
                self._acknowledged += 1
