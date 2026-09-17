from __future__ import annotations

from types import SimpleNamespace
import unittest
from unittest.mock import patch


import cv2
import numpy as np

from paus_perception.sdk_camera import CameraRuntime, _capture_payload_size, _decode_jpeg_payload, _has_complete_jpeg_payload, capture_rgb_frame


class SdkCameraPayloadTests(unittest.TestCase):
    def test_prefers_gvsp_payload_size_over_output_buffer_size(self) -> None:
        info = SimpleNamespace(gvsp_payload_size=6, payload_size=1024)

        self.assertEqual(_capture_payload_size(info, 1024), 6)

    def test_accepts_only_complete_jpeg_using_transport_size(self) -> None:
        info = SimpleNamespace(gvsp_payload_size=6, payload_size=1024)
        raw = b"\xff\xd8ok\xff\xd9" + bytes(1018)

        self.assertTrue(_has_complete_jpeg_payload(info, raw))

    def test_rejects_empty_or_zero_filled_capture(self) -> None:
        info = SimpleNamespace(gvsp_payload_size=6, payload_size=1024)
        raw = bytes(1024)

        self.assertFalse(_has_complete_jpeg_payload(info, raw))

    def test_decodes_only_the_transport_payload(self) -> None:
        success, encoded = cv2.imencode(".jpg", np.full((8, 8, 3), 127, dtype=np.uint8))
        self.assertTrue(success)
        payload = encoded.tobytes()
        info = SimpleNamespace(gvsp_payload_size=len(payload), payload_size=1024)
        raw = payload + bytes(1024 - len(payload))

        decoded = _decode_jpeg_payload(info, raw)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape, (8, 8, 3))

    def test_retries_sdk_capture_timeout_inside_active_runtime(self) -> None:
        runtime = CameraRuntime(camera_index=0, camera_obj=object(), rgb_width=8, rgb_height=8)
        image = np.zeros((8, 8, 3), dtype=np.uint8)

        with patch("paus_perception.sdk_camera._require_dkam_sdk", return_value=SimpleNamespace()), patch(
            "paus_perception.sdk_camera.capture_raw_frame",
            side_effect=[
                RuntimeError("TimeoutCaptureCSharp failed with code -34"),
                (SimpleNamespace(gvsp_payload_size=0), bytes(8 * 8 * 3)),
            ],
        ):
            with patch("paus_perception.sdk_camera.cv2.cvtColor", return_value=image):
                captured = capture_rgb_frame(runtime, timeout_us=1000, max_attempts=2)

        self.assertIs(captured, image)


if __name__ == "__main__":
    unittest.main()
