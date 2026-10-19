from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from paus_ultrasound_ros2 import protocol_v2


def test_ros_image_and_frame_are_published_from_tcp_packet() -> None:
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("paus_interfaces.msg")
    from rclpy.node import Node
    from paus_interfaces.msg import UltrasoundFrame
    from sensor_msgs.msg import Image

    from paus_ultrasound_ros2.receiver_node import IMAGE_QOS, UltrasoundReceiverNode

    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        port = reserved.getsockname()[1]

    rclpy.init(args=["--ros-args", "-p", f"listen_port:={port}"])
    receiver = None
    observer = None
    try:
        receiver = UltrasoundReceiverNode()
        observer = Node("ultrasound_test_observer")
        images: list[Image] = []
        frames: list[UltrasoundFrame] = []
        observer.create_subscription(Image, "/ultrasound/us/image_raw", images.append, IMAGE_QOS)
        observer.create_subscription(UltrasoundFrame, "/ultrasound/us/frame", frames.append, IMAGE_QOS)
        # Allow DDS discovery to complete before transmitting the only frame.
        deadline = time.monotonic() + 5.0
        while observer.count_publishers("/ultrasound/us/frame") == 0 and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)

        header = protocol_v2.frame_header(
            2, time.time(), 2, 3, session_id="ros-test", storage_order="column_major",
            timestamp_kind="display_callback",
        )
        with socket.create_connection(("127.0.0.1", port), timeout=1) as sock:
            sock.settimeout(0.1)
            sock.sendall(protocol_v2.encode_packet(header, bytes([11, 21, 12, 22, 13, 23])))
            assert protocol_v2.read_packet(sock).header["sequence_id"] == 2
            while (not images or not frames) and time.monotonic() < deadline:
                rclpy.spin_once(receiver, timeout_sec=0.01)
                rclpy.spin_once(observer, timeout_sec=0.01)
        assert len(images) == len(frames) == 1
        assert bytes(images[0].data) == bytes([11, 12, 13, 21, 22, 23])
        assert frames[0].image == images[0]
        assert frames[0].sequence_id == 2
        assert json.loads(frames[0].metadata_json)["timestamp_kind"] == "display_callback"
    finally:
        if observer is not None:
            observer.destroy_node()
        if receiver is not None:
            receiver.destroy_node()
        rclpy.shutdown()
