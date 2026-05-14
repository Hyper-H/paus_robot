from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
if str(PERCEPTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_PACKAGE_ROOT))

from paus_perception import (  # noqa: E402
    CameraCalibration,
    camera_calibration_to_camera_info_payload,
    camera_info_payload_summary,
)


def make_calibration() -> CameraCalibration:
    return CameraCalibration(
        image_width=2592,
        image_height=1944,
        camera_matrix=[
            [1672.5, 0.0, 1268.25],
            [0.0, 1671.75, 973.5],
            [0.0, 0.0, 1.0],
        ],
        dist_coeffs=[0.1, -0.2, 0.003, -0.004, 0.01],
        reprojection_error=0.0,
        board_rows=0,
        board_cols=0,
        square_size_m=0.0,
        source_type="sdk_factory",
        source_path="camera_count=1",
    )


class CameraInfoPayloadTests(unittest.TestCase):
    def test_payload_contains_ros_camera_info_fields(self) -> None:
        payload = camera_calibration_to_camera_info_payload(
            make_calibration(),
            frame_id="rgb_camera",
            rgb_camera_count=1,
            source="dkam_sdk",
        )

        self.assertEqual(payload["width"], 2592)
        self.assertEqual(payload["height"], 1944)
        self.assertEqual(payload["frame_id"], "rgb_camera")
        self.assertEqual(payload["rgb_camera_count"], 1)
        self.assertEqual(payload["distortion_model"], "plumb_bob")
        self.assertEqual(payload["source"], "dkam_sdk")
        self.assertEqual(payload["k"], [1672.5, 0.0, 1268.25, 0.0, 1671.75, 973.5, 0.0, 0.0, 1.0])
        self.assertEqual(payload["d"], [0.1, -0.2, 0.003, -0.004, 0.01])
        self.assertEqual(payload["r"], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(payload["p"], [1672.5, 0.0, 1268.25, 0.0, 0.0, 1671.75, 973.5, 0.0, 0.0, 0.0, 1.0, 0.0])

    def test_summary_extracts_intrinsics_from_k(self) -> None:
        payload = camera_calibration_to_camera_info_payload(make_calibration(), rgb_camera_count=1)
        payload["fx"] = 1.0
        payload["fy"] = 1.0
        payload["cx"] = 1.0
        payload["cy"] = 1.0

        summary = camera_info_payload_summary(payload)

        self.assertEqual(summary["rgb_camera_count"], 1)
        self.assertEqual(summary["fx"], 1672.5)
        self.assertEqual(summary["fy"], 1671.75)
        self.assertEqual(summary["cx"], 1268.25)
        self.assertEqual(summary["cy"], 973.5)


if __name__ == "__main__":
    unittest.main()
