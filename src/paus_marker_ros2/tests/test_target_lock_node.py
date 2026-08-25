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

from paus_marker_ros2.target_lock_node import TargetLockNode, average_quaternion_xyzw


def _make_pose(x: float, y: float = 0.0, z: float = 0.3, stamp_ns: int = 1_000_000_000, frame_id: str = "robot_base"):
    msg = mock.MagicMock()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    return msg


def _make_node(now_ns: int = 4_000_000_000) -> TargetLockNode:
    node = TargetLockNode.__new__(TargetLockNode)
    node.enabled = True
    node.live_target_pose_topic = "/selected_target_pose_base"
    node.locked_target_pose_topic = "/locked_target_pose_base"
    node.collect_duration_s = 0.0
    node.min_samples = 3
    node.max_sample_age_ms = 5000
    node.max_sample_jump_mm = 60.0
    node.max_position_spread_mm = 50.0
    node.locked_publish_rate_hz = 5.0
    node.drift_warning_mm = 60.0
    node.drift_action = "warn_only"
    node.policy = "lock_once_episode"
    node.stale_after_locked_action = "warn_only"
    node.allow_relock_during_motion = False
    node.samples = []
    node.rejected_sample_count = 0
    node.dropped_sample_count = 0
    node.collecting_started_ns = None
    node.locked_pose = None
    node.locked_at_ns = None
    node.last_live_pose = None
    node.last_reject_reason = None
    node.last_drop_reason = None
    node.lock_ready_reason = None
    node.last_status_signature = None
    node.last_locked_publish_monotonic = 0.0
    node.state = "waiting"
    node.locked_publisher = mock.MagicMock()
    node.status_publisher = mock.MagicMock()
    node.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    clock = mock.MagicMock()
    clock.now.return_value.nanoseconds = now_ns
    clock.now.return_value.to_msg.return_value.sec = now_ns // 1_000_000_000
    clock.now.return_value.to_msg.return_value.nanosec = now_ns % 1_000_000_000
    node.get_clock = mock.MagicMock(return_value=clock)
    node.run_dir = None
    node.trace_path = None
    node.status_latest_path = None
    return node


def _status_payloads(node: TargetLockNode) -> list[dict]:
    return [json.loads(call.args[0].data) for call in node.status_publisher.publish.call_args_list]


class TargetLockNodeTests(unittest.TestCase):
    def test_stable_samples_lock_to_median_pose(self) -> None:
        node = _make_node()

        node._live_pose_callback(_make_pose(0.10))
        node._live_pose_callback(_make_pose(0.11))
        node._live_pose_callback(_make_pose(0.12))

        self.assertEqual(node.state, "locked")
        self.assertIsNotNone(node.locked_pose)
        self.assertAlmostEqual(node.locked_pose.pose.position.x, 0.11)
        self.assertEqual(node.lock_ready_reason, "recent_samples_stable")
        self.assertEqual(node.locked_publisher.publish.call_count, 1)
        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["state"], "locked")
        self.assertTrue(payload["locked"])
        self.assertEqual(payload["lock_ready_reason"], "recent_samples_stable")
        self.assertAlmostEqual(payload["recent_sample_spread_mm"], 10.0)
        self.assertEqual(payload["policy"], "lock_once_episode")
        self.assertEqual(payload["stale_after_locked_action"], "warn_only")
        self.assertFalse(payload["allow_relock_during_motion"])

    def test_recent_samples_lock_without_waiting_collect_duration(self) -> None:
        node = _make_node()
        node.collect_duration_s = 4.0

        node._live_pose_callback(_make_pose(0.10, stamp_ns=1_000_000_000))
        node._live_pose_callback(_make_pose(0.11, stamp_ns=1_500_000_000))
        node._live_pose_callback(_make_pose(0.12, stamp_ns=2_000_000_000))

        self.assertEqual(node.state, "locked")
        self.assertIsNotNone(node.locked_pose)
        self.assertAlmostEqual(node.locked_pose.pose.position.x, 0.11)
        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["sample_retention_window_s"], 4.0)
        self.assertEqual(payload["lock_ready_reason"], "recent_samples_stable")

    def test_jump_sample_is_rejected(self) -> None:
        node = _make_node()

        node._live_pose_callback(_make_pose(0.10))
        node._live_pose_callback(_make_pose(0.30))

        self.assertEqual(node.rejected_sample_count, 1)
        self.assertEqual(node.last_reject_reason, "sample_jump_over_threshold")
        self.assertEqual(node.lock_ready_reason, "sample_jump_over_threshold")
        self.assertEqual(len(node.samples), 1)

    def test_spread_over_threshold_waits_without_locking(self) -> None:
        node = _make_node()
        node.max_sample_jump_mm = 100.0
        node.max_position_spread_mm = 20.0

        node._live_pose_callback(_make_pose(0.10))
        node._live_pose_callback(_make_pose(0.14))
        node._live_pose_callback(_make_pose(0.18))

        self.assertIsNone(node.locked_pose)
        self.assertEqual(node.last_reject_reason, "position_spread_over_threshold")
        self.assertEqual(node.last_drop_reason, "position_spread_over_threshold")
        self.assertEqual(node.lock_ready_reason, "position_spread_over_threshold")

    def test_spread_outlier_is_dropped_and_later_stable_window_locks(self) -> None:
        node = _make_node()
        node.max_sample_jump_mm = 200.0
        node.max_position_spread_mm = 20.0

        node._live_pose_callback(_make_pose(0.10))
        node._live_pose_callback(_make_pose(0.18))
        node._live_pose_callback(_make_pose(0.11))

        self.assertIsNone(node.locked_pose)
        self.assertGreaterEqual(node.dropped_sample_count, 1)

        node._live_pose_callback(_make_pose(0.12))

        self.assertEqual(node.state, "locked")
        self.assertIsNotNone(node.locked_pose)
        self.assertAlmostEqual(node.locked_pose.pose.position.x, 0.11)

    def test_prunes_samples_by_latest_target_stamp_not_node_now(self) -> None:
        node = _make_node(now_ns=100_000_000_000)
        node.collect_duration_s = 1.0
        node.samples = [
            _make_pose(0.10, stamp_ns=1_000_000_000),
            _make_pose(0.11, stamp_ns=2_000_000_000),
            _make_pose(0.12, stamp_ns=2_500_000_000),
        ]
        node.collecting_started_ns = 1_000_000_000

        node._prune_sample_window(reference_stamp_ns=2_500_000_000)

        self.assertEqual([sample.pose.position.x for sample in node.samples], [0.11, 0.12])
        self.assertEqual(node.collecting_started_ns, 2_000_000_000)
        self.assertEqual(node.last_drop_reason, "sample_outside_collect_window")

    def test_retention_window_does_not_require_full_duration_to_lock(self) -> None:
        node = _make_node()
        node.collect_duration_s = 4.0

        node._live_pose_callback(_make_pose(0.10, stamp_ns=10_000_000_000))
        node._live_pose_callback(_make_pose(0.11, stamp_ns=10_500_000_000))
        node._live_pose_callback(_make_pose(0.12, stamp_ns=11_000_000_000))

        self.assertIsNotNone(node.locked_pose)
        self.assertEqual(node.lock_ready_reason, "recent_samples_stable")

    def test_locked_target_drift_warning_does_not_change_locked_pose(self) -> None:
        node = _make_node()
        for value in (0.10, 0.11, 0.12):
            node._live_pose_callback(_make_pose(value))
        locked_x = node.locked_pose.pose.position.x

        node._live_pose_callback(_make_pose(0.30))

        self.assertEqual(node.state, "drift_warning")
        self.assertAlmostEqual(node.locked_pose.pose.position.x, locked_x)
        payload = _status_payloads(node)[-1]
        self.assertEqual(payload["event"], "target_drift_warning")

    def test_writes_trace_and_latest(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            node = _make_node()
            node.run_dir = Path(tmpdir)
            node.trace_path = node.run_dir / "target_lock_trace.jsonl"
            node.status_latest_path = node.run_dir / "target_lock_status_latest.json"

            for value in (0.10, 0.11, 0.12):
                node._live_pose_callback(_make_pose(value))

            latest = json.loads(node.status_latest_path.read_text(encoding="utf-8"))
            trace_rows = node.trace_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(latest["state"], "locked")
            self.assertGreaterEqual(len(trace_rows), 1)

    def test_average_quaternion_normalizes_signs(self) -> None:
        quat = average_quaternion_xyzw([
            __import__("numpy").asarray([0.0, 0.0, 0.0, 1.0]),
            __import__("numpy").asarray([0.0, 0.0, 0.0, -1.0]),
        ])

        self.assertAlmostEqual(float(quat[3]), 1.0)


if __name__ == "__main__":
    unittest.main()
