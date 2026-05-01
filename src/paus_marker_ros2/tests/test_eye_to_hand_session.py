from __future__ import annotations

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

    def test_record_waypoint_starts_manual_session_before_logging(self) -> None:
        class FakeTrajectory:
            def __init__(self) -> None:
                self.waypoints = []

            def to_payload(self) -> dict[str, object]:
                return {"waypoints": []}

        class FakeClient:
            def get_actual_joint_pos_degree(self) -> tuple[int, list[float]]:
                return 0, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

            def get_actual_tcp_pose(self) -> tuple[int, list[float]]:
                return 0, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]

        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node._semi_auto_lock = threading.Lock()
        node._semi_auto_active = False
        node.linux_client = FakeClient()
        node.move_vel = 10.0
        node.move_acc = 10.0
        node.dwell_s = 0.5
        node.trajectory_path = Path("/tmp/trajectory.yaml")
        node.recorded_trajectory = FakeTrajectory()
        node.samples = []
        node.current_solution = None
        started_modes: list[str] = []
        appended: list[tuple[str, dict[str, object] | None]] = []
        published: list[dict[str, object]] = []
        node._probe_current_board_quality = lambda: {
            "detected": True,
            "reprojection_error_px": 1.25,
            "board_margin_px": 42.0,
            "reprojection_filter_enabled": True,
            "reprojection_filter_passed": True,
            "margin_filter_passed": True,
            "max_reprojection_error_px": 4.0,
            "min_board_margin_px": 10.0,
        }
        node._ensure_session_started = lambda owner="manual": started_modes.append(owner)
        node._append_run_log = lambda event, payload=None: appended.append((event, payload))
        node._publish_status = lambda status, message, payload=None: published.append(
            {"status": status, "message": message, "payload": payload or {}}
        )

        response = EyeToHandCalibrationNode._record_waypoint_callback(node, Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success)
        self.assertEqual(started_modes, ["manual"])
        self.assertEqual(appended[-1][0], "waypoint_recorded")
        self.assertEqual(appended[-1][1]["waypoint_name"], "waypoint_001")
        self.assertEqual(published[-1]["status"], "waypoint_recorded")
        self.assertEqual(published[-1]["payload"]["waypoint_name"], "waypoint_001")

    def test_delete_last_waypoint_logs_normalized_payload(self) -> None:
        class FakeWaypoint:
            def __init__(self, name: str) -> None:
                self.name = name

            def to_payload(self) -> dict[str, object]:
                return {"name": self.name, "motion": "movej", "capture": True}

        class FakeTrajectory:
            def __init__(self) -> None:
                self.waypoints = [FakeWaypoint("waypoint_001")]

            def to_payload(self) -> dict[str, object]:
                return {"waypoints": [waypoint.to_payload() for waypoint in self.waypoints]}

        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node._semi_auto_lock = threading.Lock()
        node._semi_auto_active = False
        node.recorded_trajectory = FakeTrajectory()
        appended: list[tuple[str, dict[str, object] | None]] = []
        published: list[dict[str, object]] = []
        started_modes: list[str] = []
        node._ensure_session_started = lambda owner="manual": started_modes.append(owner)
        node._append_run_log = lambda event, payload=None: appended.append((event, payload))
        node._publish_status = lambda status, message, payload=None: published.append(
            {"status": status, "message": message, "payload": payload or {}}
        )
        node._reject_manual_service_if_semi_auto_active = lambda response, **kwargs: False

        response = EyeToHandCalibrationNode._delete_last_waypoint_callback(node, Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success)
        self.assertEqual(appended[-1][0], "waypoint_deleted")
        self.assertEqual(appended[-1][1]["waypoint_name"], "waypoint_001")
        self.assertEqual(appended[-1][1]["waypoint"]["name"], "waypoint_001")
        self.assertEqual(started_modes, ["manual"])
        self.assertEqual(published[-1]["status"], "waypoint_deleted")
        self.assertEqual(published[-1]["payload"]["waypoint_name"], "waypoint_001")
        self.assertEqual(published[-1]["payload"]["waypoint"]["name"], "waypoint_001")

    def test_semi_auto_validation_failure_does_not_create_session(self) -> None:
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
            self.assertEqual(node.session_dir, old_session)
            self.assertEqual(old_run_log.read_text(encoding="utf-8"), old_content)
            self.assertEqual([item.name for item in session_root.iterdir()], ["old_session"])
            self.assertEqual(published[-1]["status"], "semi_auto_failed")
            self.assertIsNone(published[-1]["payload"]["session_dir"])


if __name__ == "__main__":
    unittest.main()
