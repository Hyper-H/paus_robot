from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from paus_ultrasound_ros2 import protocol_v2
from paus_ultrasound_ros2.frames import normalize_packet


def header_and_payload(encoding: str = "mono8", order: str = "column_major") -> tuple[dict, bytes]:
    values = (
        [[11, 12, 13], [21, 22, 23]]
        if encoding == "mono8"
        else [[0x1234, 0xABCD, 0x0102], [0x3456, 0x0022, 0xFFFF]]
    )
    pixels = np.array(values, dtype=np.uint8 if encoding == "mono8" else "<u2")
    header = protocol_v2.frame_header(
        1, time.time(), 2, 3, session_id="test-session", encoding=encoding, storage_order=order
    )
    return header, pixels.tobytes(order="F" if order == "column_major" else "C")


@pytest.mark.parametrize("encoding", ["mono8", "mono16"])
@pytest.mark.parametrize("order", ["column_major", "row_major"])
def test_normalize_preserves_asymmetric_pixels(encoding: str, order: str) -> None:
    header, payload = header_and_payload(encoding, order)
    normalized = normalize_packet(protocol_v2.Packet(header, payload))
    dtype = np.uint8 if encoding == "mono8" else np.dtype("<u2")
    expected = (
        [[11, 12, 13], [21, 22, 23]]
        if encoding == "mono8"
        else [[0x1234, 0xABCD, 0x0102], [0x3456, 0x0022, 0xFFFF]]
    )
    assert np.frombuffer(normalized.pixels, dtype=dtype).reshape(2, 3).tolist() == expected
    assert normalized.header == header
    assert normalized.header is not header


def test_upstream_codec_round_trip_and_ack() -> None:
    header, pixels = header_and_payload()
    sender, receiver = socket.socketpair()
    try:
        receiver.settimeout(0.1)
        sender.sendall(protocol_v2.encode_packet(header, pixels))
        result = protocol_v2.read_packet(receiver)
        assert result.header == header
        assert result.payload == pixels
        sender.sendall(protocol_v2.acknowledgement(header))
        ack = protocol_v2.read_packet(receiver)
        assert ack.header == {
            "version": 2,
            "type": "ack",
            "status": "accepted",
            "session_id": "test-session",
            "sequence_id": 1,
        }
    finally:
        sender.close()
        receiver.close()


@pytest.mark.parametrize(
    "bad_prefix",
    [
        protocol_v2.PREFIX.pack(b"BAD!", 2, 0),
        protocol_v2.PREFIX.pack(b"USFR", 16385, 0),
        protocol_v2.PREFIX.pack(b"USFR", 10, 16 * 1024 * 1024 + 1),
    ],
)
def test_rejects_bad_prefix_before_payload(bad_prefix: bytes) -> None:
    sender, receiver = socket.socketpair()
    try:
        receiver.settimeout(0.1)
        sender.sendall(bad_prefix)
        with pytest.raises(protocol_v2.ProtocolError):
            protocol_v2.read_packet(receiver, timeout_s=0.3)
    finally:
        sender.close()
        receiver.close()


def test_rejects_bad_size_and_duplicate_json_keys() -> None:
    header, payload = header_and_payload()
    with pytest.raises(protocol_v2.ProtocolError, match="length"):
        protocol_v2.encode_packet(header, payload[:-1])
    raw = json.dumps(header).encode()
    duplicate = raw[:-1] + b',"width":3}'
    sender, receiver = socket.socketpair()
    try:
        receiver.settimeout(0.1)
        sender.sendall(protocol_v2.PREFIX.pack(b"USFR", len(duplicate), len(payload)) + duplicate)
        with pytest.raises(protocol_v2.ProtocolError, match="duplicate JSON"):
            protocol_v2.read_packet(receiver, timeout_s=0.3)
    finally:
        sender.close()
        receiver.close()


def test_truncated_payload_fails_without_implicit_ack() -> None:
    header, payload = header_and_payload()
    sender, receiver = socket.socketpair()
    try:
        receiver.settimeout(0.1)
        sender.sendall(protocol_v2.encode_packet(header, payload)[:-1])
        sender.shutdown(socket.SHUT_WR)
        with pytest.raises(EOFError, match="peer closed"):
            protocol_v2.read_packet(receiver, timeout_s=0.3)
    finally:
        sender.close()
        receiver.close()


def test_partial_payload_deadline() -> None:
    header, payload = header_and_payload()
    sender, receiver = socket.socketpair()
    try:
        receiver.settimeout(0.02)
        sender.sendall(protocol_v2.encode_packet(header, payload)[:-1])
        with pytest.raises(TimeoutError, match="deadline"):
            protocol_v2.read_packet(receiver, timeout_s=0.08)
    finally:
        sender.close()
        receiver.close()
