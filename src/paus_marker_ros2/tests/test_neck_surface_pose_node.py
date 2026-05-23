from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
for package_root in (PERCEPTION_PACKAGE_ROOT, MARKER_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

for mod_name in (
    "rclpy",
    "rclpy.node",
    "rclpy.executors",
    "geometry_msgs",
    "geometry_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "cv_bridge",
    "ament_index_python",
    "ament_index_python.packages",
):
    if mod_name not in sys.modules:
        sys.modules[mod_name] = mock.MagicMock()

sys.modules["rclpy.node"].Node = object
sys.modules["rclpy.executors"].ExternalShutdownException = RuntimeError

from paus_marker_ros2 import neck_surface_pose_node
from paus_perception.neck_surface import NeckSurfaceEstimate, STATUS_FAILED, LATCHED_POSE


class NeckSurfacePoseNodeModuleTests(unittest.TestCase):
    def test_module_imports_without_mediapipe(self) -> None:
        self.assertTrue(hasattr(neck_surface_pose_node, "NeckSurfacePoseNode"))

    def test_depth_normalization_removes_single_channel_axis(self) -> None:
        import numpy as np

        depth = np.zeros((2, 3, 1), dtype=np.float32)
        normalized = neck_surface_pose_node._normalize_depth_image(depth)
        self.assertEqual(normalized.shape, (2, 3))

    def test_failed_estimate_payload_keeps_markerless_source(self) -> None:
        estimate = NeckSurfaceEstimate(status=STATUS_FAILED, reason="low_keypoint_confidence", message="bad keypoints")
        payload = estimate.status_payload()
        self.assertEqual(payload["source"], "markerless_neck")
        self.assertEqual(payload["status"], STATUS_FAILED)
        self.assertEqual(payload["reason"], "low_keypoint_confidence")

    def test_latched_pose_constant_is_status_value(self) -> None:
        self.assertEqual(LATCHED_POSE, "latched_pose")


    def test_nearest_depth_pair_uses_configured_window(self) -> None:
        node = neck_surface_pose_node.NeckSurfacePoseNode.__new__(neck_surface_pose_node.NeckSurfacePoseNode)
        node.max_rgb_depth_delta_ms = 200.0
        node.pending_depths = {1_000_000_000: object(), 1_120_000_000: object(), 1_450_000_000: object()}

        key, delta_ms = node._find_nearest_depth_key(1_100_000_000)

        self.assertEqual(key, 1_120_000_000)
        self.assertEqual(delta_ms, 20.0)

    def test_nearest_depth_pair_rejects_stale_depth(self) -> None:
        node = neck_surface_pose_node.NeckSurfacePoseNode.__new__(neck_surface_pose_node.NeckSurfacePoseNode)
        node.max_rgb_depth_delta_ms = 50.0
        node.pending_depths = {1_200_000_000: object()}

        key, delta_ms = node._find_nearest_depth_key(1_000_000_000)

        self.assertIsNone(key)
        self.assertEqual(delta_ms, 200.0)

    def test_nearest_pair_uses_latest_camera_info(self) -> None:
        def header(stamp_ns: int):
            return SimpleNamespace(
                stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
                frame_id="camera",
            )

        node = neck_surface_pose_node.NeckSurfacePoseNode.__new__(neck_surface_pose_node.NeckSurfacePoseNode)
        node.max_rgb_depth_delta_ms = 200.0
        node.pair_queue_size = 5
        node.pending_images = {1_000_000_000: SimpleNamespace(header=header(1_000_000_000))}
        node.pending_depths = {1_060_000_000: SimpleNamespace(header=header(1_060_000_000))}
        camera_info = SimpleNamespace(header=header(990_000_000))
        node._latest_camera_info = camera_info
        processed = []
        node._process_triplet = lambda image, depth, info, *, rgb_depth_delta_ms=None: processed.append(
            (image, depth, info, rgb_depth_delta_ms)
        )

        node._try_process_nearest_pair()

        self.assertEqual(len(processed), 1)
        self.assertIs(processed[0][2], camera_info)
        self.assertEqual(processed[0][3], 60.0)
        self.assertEqual(node.pending_images, {})
        self.assertEqual(node.pending_depths, {})

    def test_pose_message_uses_target_point_when_available(self) -> None:
        node = neck_surface_pose_node.NeckSurfacePoseNode.__new__(neck_surface_pose_node.NeckSurfacePoseNode)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2), frame_id="camera")
        image_message = SimpleNamespace(header=header)
        estimate = NeckSurfaceEstimate(
            status="ok",
            reason=None,
            message="ok",
            surface_point_camera_m=[0.0, 0.0, 0.7],
            target_point_camera_m=[-0.03, 0.01, 0.7],
            rotation_matrix_camera=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )

        pose = node._build_pose_message(image_message, estimate)

        self.assertEqual(pose.pose.position.x, -0.03)
        self.assertEqual(pose.pose.position.y, 0.01)
        self.assertEqual(pose.pose.position.z, 0.7)

    def test_latched_base_outputs_keep_robot_base_frame(self) -> None:
        class Publisher:
            def __init__(self) -> None:
                self.messages = []

            def publish(self, message) -> None:
                self.messages.append(message)

        def header(frame_id: str, sec: int):
            return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0), frame_id=frame_id)

        node = neck_surface_pose_node.NeckSurfacePoseNode.__new__(neck_surface_pose_node.NeckSurfacePoseNode)
        node.execute_motion = False
        node.latched_pose_timeout_s = 5.0
        node._last_valid_time_monotonic = time.monotonic()
        node._last_valid_pose_message = SimpleNamespace(header=header("camera", 1))
        node._last_valid_target_pose_message = SimpleNamespace(header=header("robot_base", 1))
        node._last_valid_point_message = SimpleNamespace(header=header("robot_base", 1))
        node._last_valid_status_payload = {"source": "markerless_neck"}
        node.neck_surface_pose_publisher = Publisher()
        node.target_pose_publisher = Publisher()
        node.approach_target_publisher = Publisher()
        published_status = []
        node._publish_status = published_status.append

        node._maybe_publish_latched(header("camera_color_optical_frame", 2))

        self.assertEqual(node.neck_surface_pose_publisher.messages[0].header.frame_id, "camera_color_optical_frame")
        self.assertEqual(node.target_pose_publisher.messages[0].header.frame_id, "robot_base")
        self.assertEqual(node.approach_target_publisher.messages[0].header.frame_id, "robot_base")
        self.assertEqual(node.target_pose_publisher.messages[0].header.stamp.sec, 2)
        self.assertEqual(published_status[0]["pose_freshness"], LATCHED_POSE)


if __name__ == "__main__":
    unittest.main()
