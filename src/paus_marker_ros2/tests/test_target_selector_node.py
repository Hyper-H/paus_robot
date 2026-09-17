from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
for package_root in (MARKER_PACKAGE_ROOT, PERCEPTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from paus_marker_ros2.target_selector_node import MARKER_MODE, MARKERLESS_NECK_MODE, TargetSelectorNode


def _make_pose(stamp_ns: int = 1_000_000_000, frame_id: str = "robot_base"):
    msg = mock.MagicMock()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.pose.position.x = 0.1
    msg.pose.position.y = 0.2
    msg.pose.position.z = 0.3
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    return msg


def _make_node(mode: str = MARKERLESS_NECK_MODE, now_ns: int = 1_050_000_000) -> TargetSelectorNode:
    node = TargetSelectorNode.__new__(TargetSelectorNode)
    node.mode = mode
    node.source_timeout_ms = 500
    node.markerless_target_pose_topic = "/markerless_neck_target_pose_base"
    node.marker_target_pose_topic = "/marker_target_pose_base"
    node.selected_target_pose_topic = "/selected_target_pose_base"
    node.latest_markerless_pose = None
    node.latest_marker_pose = None
    node.last_selected_stamp_ns = None
    node.last_status_signature = None
    node.selected_publisher = mock.MagicMock()
    node.status_publisher = mock.MagicMock()
    node.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    clock = mock.MagicMock()
    clock.now.return_value.nanoseconds = now_ns
    node.get_clock = mock.MagicMock(return_value=clock)
    node.run_dir = None
    node.trace_path = None
    node.status_latest_path = None
    return node


def _status_payloads(node: TargetSelectorNode) -> list[dict]:
    payloads = []
    for call in node.status_publisher.publish.call_args_list:
        payloads.append(json.loads(call.args[0].data))
    return payloads


class TargetSelectorNodeTests(unittest.TestCase):
    def test_markerless_mode_forwards_markerless_only(self) -> None:
        node = _make_node(MARKERLESS_NECK_MODE)

        node._marker_pose_callback(_make_pose(1_000_000_000))
        self.assertEqual(node.selected_publisher.publish.call_count, 0)
        self.assertEqual(_status_payloads(node)[-1]["reason"], "source_not_selected")

        node._markerless_pose_callback(_make_pose(1_000_000_000))
        self.assertEqual(node.selected_publisher.publish.call_count, 1)
        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["selected_source"], MARKERLESS_NECK_MODE)
        self.assertTrue(payload["published"])

    def test_marker_mode_forwards_marker_only(self) -> None:
        node = _make_node(MARKER_MODE)

        node._markerless_pose_callback(_make_pose(1_000_000_000))
        self.assertEqual(node.selected_publisher.publish.call_count, 0)
        self.assertEqual(_status_payloads(node)[-1]["reason"], "source_not_selected")

        node._marker_pose_callback(_make_pose(1_000_000_000))
        self.assertEqual(node.selected_publisher.publish.call_count, 1)
        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["selected_source"], MARKER_MODE)
        self.assertTrue(payload["published"])

    def test_stale_selected_source_does_not_fallback(self) -> None:
        node = _make_node(MARKERLESS_NECK_MODE, now_ns=2_000_000_000)
        node.latest_marker_pose = _make_pose(2_000_000_000)

        node._markerless_pose_callback(_make_pose(1_000_000_000))

        self.assertEqual(node.selected_publisher.publish.call_count, 0)
        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["status"], "stale")
        self.assertEqual(payload["reason"], "source_timeout")
        self.assertEqual(payload["selected_source"], MARKERLESS_NECK_MODE)
        self.assertFalse(payload["published"])

    def test_missing_selected_source_publishes_waiting(self) -> None:
        node = _make_node(MARKERLESS_NECK_MODE)

        node._status_timer_callback()

        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["status"], "waiting")
        self.assertEqual(payload["reason"], "source_missing")
        self.assertFalse(payload["published"])

    def test_invalid_mode_fails_without_selected_publish(self) -> None:
        node = _make_node("bad_mode")

        node._markerless_pose_callback(_make_pose())

        self.assertEqual(node.selected_publisher.publish.call_count, 0)
        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["status"], "failed")
        self.assertEqual(payload["reason"], "invalid_mode")

    def test_selector_writes_trace_and_latest(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            node = _make_node(MARKERLESS_NECK_MODE)
            node.run_dir = Path(tmpdir)
            node.trace_path = node.run_dir / "selector_trace.jsonl"
            node.status_latest_path = node.run_dir / "selector_status_latest.json"

            node._markerless_pose_callback(_make_pose())

            trace_rows = node.trace_path.read_text(encoding="utf-8").splitlines()
            latest = json.loads(node.status_latest_path.read_text(encoding="utf-8"))
            self.assertEqual(len(trace_rows), 1)
            self.assertEqual(latest["status"], "ok")
            self.assertEqual(latest["selected_source"], MARKERLESS_NECK_MODE)


if __name__ == "__main__":
    unittest.main()
