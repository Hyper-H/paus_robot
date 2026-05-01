from __future__ import annotations

import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path

import pytest
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
if str(MARKER_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MARKER_PACKAGE_ROOT))

pytest.importorskip("rclpy")
pytest.importorskip("std_srvs.srv")

from std_srvs.srv import Trigger

from paus_marker_ros2.eye_to_hand_calibration_node import EyeToHandCalibrationNode
from paus_marker_ros2.semi_auto_calibration import sample_log_targets, session_owner_matches


class EyeToHandSessionTests(unittest.TestCase):
    def test_session_owner_matches_for_active_semi_auto_capture(self) -> None:
        self.assertTrue(session_owner_matches("semi_auto", "semi_auto"))

    def test_session_owner_mismatch_for_manual_capture_after_semi_auto(self) -> None:
        self.assertFalse(session_owner_matches("semi_auto", "manual"))
        self.assertFalse(session_owner_matches(None, "manual"))

    def test_sample_log_targets_keep_session_archive_when_override_is_set(self) -> None:
        session_path = Path("/tmp/session/samples.jsonl")
        override_path = Path("/tmp/legacy/samples.jsonl")

        self.assertEqual(sample_log_targets(session_path, override_path), [session_path, override_path])

    def test_sample_log_targets_deduplicate_matching_override(self) -> None:
        session_path = Path("/tmp/session/samples.jsonl")

        self.assertEqual(sample_log_targets(session_path, session_path), [session_path])
        self.assertEqual(sample_log_targets(session_path, None), [session_path])

    def test_semi_auto_validation_failure_uses_fresh_session(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            session_root = root / "sessions"
            session_root.mkdir()
            trajectory_path = root / "bad_trajectory.yaml"
            trajectory_path.write_text(
                yaml.safe_dump(
                    {
                        "version": 1,
                        "defaults": {"motion": "movel"},
                        "waypoints": [
                            {
                                "name": "bad_motion",
                                "joint_deg": [1, 2, 3, 4, 5, 6],
                                "expected_tcp_pose_mmdeg": [100, 200, 300, 10, 20, 30],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            old_session = session_root / "old_session"
            old_session.mkdir()
            old_run_log = old_session / "run.log"
            old_content = '{"event":"old_session_marker"}\n'
            old_run_log.write_text(old_content, encoding="utf-8")

            node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
            node._semi_auto_lock = threading.Lock()
            node._semi_auto_active = False
            node.trajectory_path = trajectory_path
            node.session_root_path = session_root
            node.execute_motion = False
            node.session_dir = old_session
            node.sample_log_path = old_session / "samples.jsonl"
            node.report_path = old_session / "report.yaml"
            node.run_log_path = old_run_log
            node.session_owner = "semi_auto"
            node.samples = []
            node.current_solution = None
            published: list[dict[str, object]] = []
            node._publish_status = lambda status, message, payload=None: published.append(
                {"status": status, "message": message, "payload": payload or {}}
            )

            response = EyeToHandCalibrationNode._run_semi_auto_callback(node, Trigger.Request(), Trigger.Response())

            self.assertFalse(response.success)
            self.assertNotEqual(node.session_dir, old_session)
            self.assertEqual(old_run_log.read_text(encoding="utf-8"), old_content)
            self.assertIsNotNone(node.run_log_path)
            records = [json.loads(line) for line in node.run_log_path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(records[-1]["event"], "semi_auto_failed")
            self.assertEqual(published[-1]["status"], "semi_auto_failed")
            self.assertEqual(Path(published[-1]["payload"]["session_dir"]), node.session_dir)


if __name__ == "__main__":
    unittest.main()
