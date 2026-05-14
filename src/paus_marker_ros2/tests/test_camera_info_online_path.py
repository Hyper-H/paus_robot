from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
for package_root in (MARKER_PACKAGE_ROOT, PERCEPTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))


try:
    from sensor_msgs.msg import CameraInfo
except ModuleNotFoundError:  # pragma: no cover - depends on ROS env availability.
    CameraInfo = None

if CameraInfo is not None:
    from paus_marker_ros2.marker_pose_node import camera_info_signature, camera_info_to_calibration
    from paus_marker_ros2.image_receiver_node import timestamp_ns_from_header

from paus_marker_ros2.target_transform_node import TargetTransformNode


@unittest.skipIf(CameraInfo is None, "sensor_msgs is unavailable outside a sourced ROS environment")
class CameraInfoOnlinePathTests(unittest.TestCase):
    def test_camera_info_to_calibration_preserves_intrinsics(self) -> None:
        message = CameraInfo()
        message.width = 2592
        message.height = 1944
        message.distortion_model = "plumb_bob"
        message.k = [1672.5, 0.0, 1268.25, 0.0, 1671.75, 973.5, 0.0, 0.0, 1.0]
        message.d = [0.1, -0.2, 0.003, -0.004, 0.01]

        calibration = camera_info_to_calibration(message)

        self.assertEqual(calibration.image_width, 2592)
        self.assertEqual(calibration.image_height, 1944)
        self.assertEqual(calibration.camera_matrix[0], [1672.5, 0.0, 1268.25])
        self.assertEqual(calibration.camera_matrix[1], [0.0, 1671.75, 973.5])
        self.assertEqual(calibration.camera_matrix[2], [0.0, 0.0, 1.0])
        self.assertEqual(calibration.dist_coeffs, [0.1, -0.2, 0.003, -0.004, 0.01])
        self.assertEqual(calibration.source_type, "ros_camera_info")
        self.assertEqual(calibration.source_path, "/camera/camera_info")

    def test_camera_info_signature_rejects_invalid_dimensions(self) -> None:
        message = CameraInfo()
        message.width = 0
        message.height = 1944
        message.k = [1.0] * 9

        with self.assertRaises(ValueError):
            camera_info_signature(message)

    def test_camera_info_to_calibration_rejects_invalid_k_length(self) -> None:
        message = SimpleNamespace(width=2592, height=1944, k=[1.0] * 8, d=[])

        with self.assertRaises(ValueError):
            camera_info_to_calibration(message)

    def test_header_timestamp_validation_rejects_missing_zero_and_bool(self) -> None:
        self.assertEqual(timestamp_ns_from_header({"timestamp_ns": 123456789}), 123456789)
        self.assertIsNone(timestamp_ns_from_header({}))
        self.assertIsNone(timestamp_ns_from_header({"timestamp_ns": 0}))
        self.assertIsNone(timestamp_ns_from_header({"timestamp_ns": True}))


class OnlineStackStaticTests(unittest.TestCase):
    def test_online_stack_no_longer_uses_tmp_camera_yaml(self) -> None:
        online_paths = [
            PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py",
            PROJECT_ROOT / "src" / "paus_marker_ros2" / "scripts" / "camera_bridge.py",
            PROJECT_ROOT / "src" / "paus_marker_ros2" / "paus_marker_ros2" / "image_receiver_node.py",
            PROJECT_ROOT / "src" / "paus_marker_ros2" / "paus_marker_ros2" / "marker_pose_node.py",
        ]
        forbidden_terms = [
            "/tmp/paus_robot/camera.yaml",
            "camera_config_output",
            "camera_config_path",
            "save_runtime_calibration_to_yaml",
        ]

        joined = "\n".join(path.read_text(encoding="utf-8") for path in online_paths)

        for term in forbidden_terms:
            self.assertNotIn(term, joined)

    def test_online_stack_defaults_to_rgb_camera_count_one(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )
        bridge_text = (PROJECT_ROOT / "src" / "paus_marker_ros2" / "scripts" / "camera_bridge.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"rgb_camera_count"', launch_text)
        self.assertIn('default_value="1"', launch_text)
        self.assertIn('"--rgb-camera-count"', launch_text)
        self.assertIn('parser.add_argument("--rgb-camera-count", type=int, default=1', bridge_text)
        self.assertIn("camera_info_loaded", bridge_text)
        self.assertIn('"camera_info": camera_info_payload', bridge_text)


class TargetTransformValidationTests(unittest.TestCase):
    def test_large_target_jump_within_workspace_is_not_filtered(self) -> None:
        node = TargetTransformNode.__new__(TargetTransformNode)
        node.max_target_distance_mm = 1500.0

        valid, reject_reason = node._validate_target_translation(
            np.asarray([500.0, 0.0, 100.0], dtype=np.float64)
        )

        self.assertTrue(valid)
        self.assertEqual(reject_reason, "")

    def test_non_finite_target_is_filtered(self) -> None:
        node = TargetTransformNode.__new__(TargetTransformNode)
        node.max_target_distance_mm = 1500.0

        valid, reject_reason = node._validate_target_translation(
            np.asarray([np.nan, 0.0, 100.0], dtype=np.float64)
        )

        self.assertFalse(valid)
        self.assertEqual(reject_reason, "target_translation_non_finite")

    def test_target_above_max_distance_is_filtered(self) -> None:
        node = TargetTransformNode.__new__(TargetTransformNode)
        node.max_target_distance_mm = 1500.0

        valid, reject_reason = node._validate_target_translation(
            np.asarray([1600.0, 0.0, 0.0], dtype=np.float64)
        )

        self.assertFalse(valid)
        self.assertIn("target_distance_above_max", reject_reason)


if __name__ == "__main__":
    unittest.main()
