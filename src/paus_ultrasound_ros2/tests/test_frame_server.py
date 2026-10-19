from __future__ import annotations

import socket
import sys
import time
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from paus_ultrasound_ros2 import protocol_v2
from paus_ultrasound_ros2.frame_server import FrameServer
from test_protocol_and_frames import header_and_payload


@pytest.fixture
def server():
    listener = FrameServer("127.0.0.1", 0)
    listener.start()
    yield listener
    listener.stop()
    assert listener.snapshot()["state"] == "STOPPED"


def wait_for(check, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while not check():
        if time.monotonic() > deadline:
            raise AssertionError("receiver did not reach expected state")
        time.sleep(0.01)


def send_frame(sock: socket.socket, sequence: int, session: str = "test-session") -> dict:
    header, payload = header_and_payload()
    header.update(session_id=session, sequence_id=sequence)
    sock.sendall(protocol_v2.encode_packet(header, payload))
    return protocol_v2.read_packet(sock, timeout_s=1.0).header


def test_sender_ack_handoff_and_reconnect_gap(server: FrameServer) -> None:
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        ack = send_frame(sock, 1)
        assert (ack["status"], ack["session_id"], ack["sequence_id"]) == ("accepted", "test-session", 1)
        assert send_frame(sock, 3)["sequence_id"] == 3
        wait_for(lambda: server.snapshot()["received"] == 2)
        frame = server.pop_latest()
        assert frame is not None and frame.header["sequence_id"] == 3
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        assert send_frame(sock, 4)["sequence_id"] == 4
        wait_for(lambda: server.snapshot()["received"] == 3)
    stats = server.snapshot()
    assert stats["acknowledged"] == 3
    assert stats["skipped_sequences"] == 1
    assert stats["overwritten"] == 1


def test_bad_packet_disconnects_without_ack_and_listener_recovers(server: FrameServer) -> None:
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.2)
        sock.sendall(protocol_v2.PREFIX.pack(b"USFR", 20000, 0))
        assert sock.recv(1) == b""
    wait_for(lambda: server.snapshot()["rejected"] == 1)
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        assert send_frame(sock, 1)["status"] == "accepted"


def test_duplicate_sequence_is_not_acked(server: FrameServer) -> None:
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        send_frame(sock, 4)
        header, payload = header_and_payload()
        header["sequence_id"] = 4
        sock.sendall(protocol_v2.encode_packet(header, payload))
        assert sock.recv(1) == b""
    wait_for(lambda: server.snapshot()["rejected"] == 1)
    assert server.snapshot()["acknowledged"] == 1


def test_incomplete_frame_is_not_acked(server: FrameServer) -> None:
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        header, payload = header_and_payload()
        sock.sendall(protocol_v2.encode_packet(header, payload)[:-1])
        sock.shutdown(socket.SHUT_WR)
        assert sock.recv(1) == b""
    wait_for(lambda: server.snapshot()["disconnects"] == 1)
    assert server.snapshot()["acknowledged"] == 0
    assert server.snapshot()["rejected"] == 1


def test_live_connection_becomes_stale_discards_buffer_and_recovers() -> None:
    server = FrameServer("127.0.0.1", 0, stale_timeout_s=0.1)
    server.start()
    try:
        with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
            sock.settimeout(0.1)
            assert send_frame(sock, 1)["status"] == "accepted"
            assert server.snapshot()["state"] == "STREAMING"
            wait_for(lambda: server.snapshot()["timed_out"])
            assert server.snapshot()["state"] == "STALE"
            assert server.snapshot()["disconnects"] == 0
            assert server.pop_latest() is None
            assert send_frame(sock, 2)["status"] == "accepted"
            frame = server.pop_latest()
            assert frame is not None and frame.header["sequence_id"] == 2
            assert server.snapshot()["state"] == "STREAMING"
            assert server.snapshot()["timed_out"] is False
    finally:
        server.stop()


def test_disconnect_discards_pending_frame_and_new_connection_has_no_old_metadata(server: FrameServer) -> None:
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        send_frame(sock, 1)
        assert server.snapshot()["frame_metadata"]["sequence_id"] == 1
    wait_for(lambda: server.snapshot()["state"] == "LISTENING")
    assert server.pop_latest() is None
    with socket.create_connection(("127.0.0.1", server.port), timeout=1):
        wait_for(lambda: server.snapshot()["state"] == "CONNECTED")
        assert server.snapshot()["frame_metadata"] == {}
        assert server.snapshot()["latest_age_s"] is None


def test_connected_without_first_frame_times_out() -> None:
    server = FrameServer("127.0.0.1", 0, stale_timeout_s=0.05)
    server.start()
    try:
        with socket.create_connection(("127.0.0.1", server.port), timeout=1):
            wait_for(lambda: server.snapshot()["state"] == "STALE")
            assert server.snapshot()["latest_age_s"] is None
    finally:
        server.stop()


def test_integral_json_floats_are_normalized_without_killing_receiver(server: FrameServer) -> None:
    header, payload = header_and_payload()
    for key in ("width", "height", "sequence_id", "sender_dropped"):
        header[key] = float(header[key])
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        sock.sendall(protocol_v2.encode_packet(header, payload))
        assert protocol_v2.read_packet(sock).header["status"] == "accepted"
        frame = server.pop_latest()
        assert frame is not None
        assert all(type(frame.header[key]) is int for key in ("width", "height", "sequence_id", "sender_dropped"))
        assert type(header["width"]) is float
        assert send_frame(sock, frame.header["sequence_id"] + 1)["status"] == "accepted"


def test_unknown_nonfinite_json_extension_is_rejected_without_killing_receiver(server: FrameServer) -> None:
    import json
    header, payload = header_and_payload()
    raw = json.dumps(header)[:-1].encode() + b', "extension": {"overflow": 1e400}}'
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        sock.sendall(protocol_v2.PREFIX.pack(b"USFR", len(raw), len(payload)) + raw + payload)
        try:
            assert sock.recv(1) == b""
        except ConnectionResetError:
            pass  # Header rejection may close with unread payload still queued.
    wait_for(lambda: server.snapshot()["rejected"] == 1)
    with socket.create_connection(("127.0.0.1", server.port), timeout=1) as sock:
        sock.settimeout(0.1)
        assert send_frame(sock, 1)["status"] == "accepted"
