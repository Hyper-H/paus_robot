import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paus_ultrasound_ros2 import protocol_v2


def header():
    return protocol_v2.frame_header(
        1, 1.0, 2, 3, roi_confirmed=True,
        capture_size=[100, 100], roi_pixels=[10, 20, 3, 2],
        profile_revision="reviewed-1", imaging_context="B-mode depth 40mm",
        display_scale_mm_per_pixel={"horizontal": None, "vertical": 0.05},
    )


def test_single_axis_scale_is_optional_and_geometry_remains_invalid() -> None:
    metadata = header()
    assert protocol_v2.encode_packet(metadata, bytes(6))
    assert metadata["geometry_valid"] is False
    assert metadata["display_scale_mm_per_pixel"]["horizontal"] is None


@pytest.mark.parametrize("field,value", [
    ("roi_confirmed", "true"),
    ("capture_size", [100, 1.5]),
    ("roi_pixels", [99, 20, 3, 2]),
    ("roi_pixels", [10, 20, 3, 4]),
    ("display_scale_mm_per_pixel", {"horizontal": 0.0, "vertical": None}),
    ("display_scale_mm_per_pixel", {"horizontal": True, "vertical": None}),
    ("display_scale_mm_per_pixel", {"horizontal": float("nan"), "vertical": None}),
    ("display_scale_mm_per_pixel", {"vertical": 0.05}),
])
def test_invalid_display_metadata_is_rejected(field, value) -> None:
    metadata = header()
    metadata[field] = value
    with pytest.raises(protocol_v2.ProtocolError):
        protocol_v2.encode_packet(metadata, bytes(6))
