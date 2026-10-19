from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from paus_ultrasound_ros2 import protocol_v2
from paus_ultrasound_ros2.frames import normalize_packet


def test_image_and_aggregate_preserve_frame_identity() -> None:
    pytest.importorskip("rclpy")
    pytest.importorskip("paus_interfaces.msg")
    from paus_ultrasound_ros2.receiver_node import make_messages

    header = protocol_v2.frame_header(
        7, time.time(), 2, 3, session_id="run-123", encoding="mono16",
        storage_order="column_major", timestamp_kind="display_callback",
    )
    # MATLAB's column-major order for [[11,12,13], [21,22,23]].
    payload = bytes([11, 0, 21, 0, 12, 0, 22, 0, 13, 0, 23, 0])
    frame = normalize_packet(protocol_v2.Packet(header, payload))
    image, combined = make_messages(frame)

    assert (image.height, image.width, image.encoding, image.step, image.is_bigendian) == (2, 3, "mono16", 6, False)
    assert bytes(image.data) == bytes([11, 0, 12, 0, 13, 0, 21, 0, 22, 0, 23, 0])
    assert image.header.frame_id == "ultrasound_image"
    assert image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec == frame.received_ns
    assert combined.image == image
    assert (combined.protocol_version, combined.stream_id, combined.session_id, combined.sequence_id) == (
        2, "us", "run-123", 7
    )
    assert json.loads(combined.metadata_json) == header
