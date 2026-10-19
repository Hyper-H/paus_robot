from __future__ import annotations

from datetime import datetime
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from paus_ultrasound_ros2.record_bag import (
    FRAME_TOPIC,
    IMAGE_TOPIC,
    STATUS_TOPIC,
    build_record_command,
    make_output_path,
)


def test_default_output_path_uses_timestamp() -> None:
    output = make_output_path(Path("/tmp/rosbags"), None, datetime(2026, 9, 24, 12, 34, 56))
    assert output == Path("/tmp/rosbags/us_real_20260924_123456")


def test_record_command_avoids_duplicate_image_by_default() -> None:
    command = build_record_command(Path("/tmp/rosbags/run"), include_image_raw=False)
    assert FRAME_TOPIC in command
    assert STATUS_TOPIC in command
    assert IMAGE_TOPIC not in command


def test_record_command_can_include_standard_image_topic() -> None:
    command = build_record_command(Path("/tmp/rosbags/run"), include_image_raw=True)
    assert command[-1] == IMAGE_TOPIC
