from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from geometry_msgs.msg import PoseStamped
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
for package_root in (MARKER_PACKAGE_ROOT, PERCEPTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from paus_marker_ros2.target_transform_node import TargetTransformNode


class TargetTransformLoggingTests(unittest.TestCase):
    def test_callback_preserves_camera_input_frame_in_status(self) -> None:
        node = TargetTransformNode.__new__(TargetTransformNode)
        node.eye_to_hand_solution = SimpleNamespace(
            success=True,
            base_to_camera=SimpleNamespace(
                translation_m=[0.0, 0.0, 0.0],
                rotation_matrix=np.eye(3).tolist(),
            ),
        )
        node.marker_to_target = np.eye(4)
        node.max_target_distance_mm = 1500.0
        node.target_pose_topic = "/marker_target_pose_base"
        node.target_point_topic = "/target_point_base"
        node.target_pose_publisher = mock.MagicMock()
        node.target_point_publisher = mock.MagicMock()
        node._publish_status = mock.MagicMock()

        message = PoseStamped()
        message.header.frame_id = "camera"
        message.pose.position.z = 0.5
        message.pose.orientation.w = 1.0

        node._marker_pose_callback(message)

        self.assertEqual(message.header.frame_id, "camera")
        published_pose = node.target_pose_publisher.publish.call_args.args[0]
        self.assertEqual(published_pose.header.frame_id, "robot_base")
        status_extra = node._publish_status.call_args.args[2]
        self.assertEqual(status_extra["input_frame_id"], "camera")

    def test_transform_status_writes_trace_and_latest(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            node = TargetTransformNode.__new__(TargetTransformNode)
            node.run_dir = Path(tmpdir)
            node.trace_path = node.run_dir / "transform_trace.jsonl"
            node.status_latest_path = node.run_dir / "transform_status_latest.json"
            node.status_publisher = mock.MagicMock()
            node.get_logger = mock.MagicMock(return_value=mock.MagicMock())

            node._publish_status(
                "target_filtered",
                "Target transform rejected.",
                {
                    "event": "target_transform_rejected",
                    "reject_reason": "target_distance_above_max:1600.0mm",
                    "target_point_base_mm": [1600.0, 0.0, 0.0],
                },
            )

            trace_rows = node.trace_path.read_text(encoding="utf-8").splitlines()
            latest = node.status_latest_path.read_text(encoding="utf-8")
            self.assertEqual(len(trace_rows), 1)
            self.assertIn("target_transform_rejected", trace_rows[0])
            self.assertIn("target_distance_above_max", latest)


if __name__ == "__main__":
    unittest.main()
