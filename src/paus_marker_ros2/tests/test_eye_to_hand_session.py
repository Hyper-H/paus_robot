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

from paus_marker_ros2.eye_to_hand_calibration_node import EyeToHandCalibrationNode, SampleRejectedError
from paus_marker_ros2.semi_auto_calibration import build_recorded_waypoint, sample_log_targets, session_owner_matches


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


    def test_delete_selected_waypoint_removes_named_waypoint(self) -> None:
        class FakeParamValue:
            string_value = "waypoint_001"

        class FakeParam:
            def get_parameter_value(self) -> FakeParamValue:
                return FakeParamValue()

        class FakeTrajectory:
            def __init__(self) -> None:
                self.waypoints = [
                    build_recorded_waypoint(
                        index=1,
                        joint_deg=[1, 1, 1, 1, 1, 1],
                        tcp_pose_mmdeg=[1, 2, 3, 4, 5, 6],
                        vel=10.0,
                        acc=10.0,
                        dwell_s=0.5,
                    ),
                    build_recorded_waypoint(
                        index=2,
                        joint_deg=[2, 2, 2, 2, 2, 2],
                        tcp_pose_mmdeg=[2, 3, 4, 5, 6, 7],
                        vel=10.0,
                        acc=10.0,
                        dwell_s=0.5,
                    ),
                ]

            def to_payload(self) -> dict[str, object]:
                return {"waypoints": [waypoint.to_payload() for waypoint in self.waypoints]}

        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node._semi_auto_lock = threading.Lock()
        node._semi_auto_active = False
        node.recorded_trajectory = FakeTrajectory()
        node.trajectory_path = Path("/tmp/trajectory.yaml")
        appended: list[tuple[str, dict[str, object] | None]] = []
        published: list[dict[str, object]] = []
        node._ensure_session_started = lambda owner="manual": None
        node._write_recorded_trajectory = lambda: None
        node._append_run_log = lambda event, payload=None: appended.append((event, payload))
        node._publish_status = lambda status, message, payload=None: published.append(
            {"status": status, "message": message, "payload": payload or {}}
        )
        node._reject_manual_service_if_semi_auto_active = lambda response, **kwargs: False
        node.get_parameter = lambda name: FakeParam()

        response = EyeToHandCalibrationNode._delete_selected_waypoint_callback(node, Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success)
        self.assertEqual([waypoint.name for waypoint in node.recorded_trajectory.waypoints], ["waypoint_002"])
        self.assertEqual(appended[-1][0], "waypoint_deleted")
        self.assertEqual(appended[-1][1]["waypoint_name"], "waypoint_001")
        self.assertEqual(published[-1]["payload"]["recorded_trajectory"]["waypoints"][0]["name"], "waypoint_002")

    def test_semi_auto_skips_rejected_capture_and_reports_insufficient_after_loop(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            session_root = root / "sessions"
            session_root.mkdir()
            trajectory_path = root / "trajectory.yaml"
            trajectory_path.write_text(
                yaml.safe_dump(
                    {
                        "version": 1,
                        "tool_id": 0,
                        "user_id": 0,
                        "defaults": {"motion": "movej", "vel": 10.0, "acc": 10.0, "dwell_s": 0.0},
                        "waypoints": [
                            {
                                "name": "waypoint_001",
                                "motion": "movej",
                                "joint_deg": [1, 2, 3, 4, 5, 6],
                                "expected_tcp_pose_mmdeg": [100, 200, 300, 10, 20, 30],
                                "vel": 10.0,
                                "acc": 10.0,
                                "dwell_s": 0.0,
                                "capture": True,
                            },
                            {
                                "name": "waypoint_002",
                                "motion": "movej",
                                "joint_deg": [2, 3, 4, 5, 6, 7],
                                "expected_tcp_pose_mmdeg": [110, 210, 310, 11, 21, 31],
                                "vel": 10.0,
                                "acc": 10.0,
                                "dwell_s": 0.0,
                                "capture": True,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            class FakeClient:
                def __init__(self) -> None:
                    self.moves: list[list[float]] = []

                def move_j(self, joint_deg: list[float], **_kwargs: object) -> int:
                    self.moves.append(joint_deg)
                    return 0

            class FakeSample:
                sample_quality = {"accepted": True, "reprojection_error_px": 0.2, "board_margin_px": 40.0}

            node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
            node._semi_auto_lock = threading.Lock()
            node._semi_auto_active = False
            node.trajectory_path = trajectory_path
            node.session_root_path = session_root
            node.execute_motion = True
            node.session_dir = None
            node.sample_log_path = None
            node.report_path = None
            node.run_log_path = None
            node.session_owner = None
            node.samples = []
            node.current_solution = None
            node.linux_client = FakeClient()
            node.min_sample_count = 10
            node.tool_id = 0
            node.user_id = 0
            node.observation_mode = "rgb_pnp"
            node.solver_method = "ax_xb_park"
            node.sample_log_override_path = None
            node._archive_current_trajectory = lambda: None
            node._wait_until_tcp_stable = lambda: [0, 0, 0, 0, 0, 0]
            node._sample_to_log_record = lambda sample: {"sample_index": 1, "row_index": 1}
            published: list[dict[str, object]] = []
            node._publish_status = lambda status, message, payload=None: published.append(
                {"status": status, "message": message, "payload": payload or {}}
            )
            captures = iter([
                SampleRejectedError("chessboard_not_found", {"accepted": False, "reject_reason": "chessboard_not_found"}),
                FakeSample(),
            ])

            def capture(owner: str) -> FakeSample:
                item = next(captures)
                if isinstance(item, Exception):
                    raise item
                node.samples.append(item)
                return item

            node._capture_one_sample = capture

            response = EyeToHandCalibrationNode._run_semi_auto_callback(node, Trigger.Request(), Trigger.Response())

            self.assertFalse(response.success)
            self.assertEqual(len(node.linux_client.moves), 2)
            self.assertIn("Need at least", response.message)
            statuses = [item["status"] for item in published]
            self.assertIn("waypoint_capture_skipped", statuses)
            self.assertIn("waypoint_sample_captured", statuses)
            self.assertEqual(published[-1]["status"], "semi_auto_insufficient_samples")
            assert node.run_log_path is not None
            run_log = node.run_log_path.read_text(encoding="utf-8")
            self.assertIn('"event": "waypoint_capture_skipped"', run_log)
            self.assertIn('"event": "waypoint_sample_captured"', run_log)

    def test_save_trajectory_archives_saved_trajectory(self) -> None:
        class FakeTrajectory:
            def __init__(self) -> None:
                self.waypoints = [object()]

            def to_payload(self) -> dict[str, object]:
                return {"waypoints": [{"name": "waypoint_001", "capture": True}]}

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            session_root = root / "sessions"
            session_root.mkdir()
            trajectory_path = root / "trajectory.yaml"

            node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
            node._semi_auto_lock = threading.Lock()
            node._semi_auto_active = False
            node.recorded_trajectory = FakeTrajectory()
            node.trajectory_path = trajectory_path
            node.session_root_path = session_root
            node.session_dir = None
            node.sample_log_path = None
            node.report_path = None
            node.run_log_path = None
            node.session_owner = None
            node.samples = []
            node.current_solution = None
            node._publish_status = lambda *_args, **_kwargs: None
            node._write_recorded_trajectory = lambda: trajectory_path.write_text("saved: true\n", encoding="utf-8")

            response = EyeToHandCalibrationNode._save_trajectory_callback(node, Trigger.Request(), Trigger.Response())

            self.assertTrue(response.success)
            self.assertIsNotNone(node.session_dir)
            assert node.session_dir is not None
            archived_path = node.session_dir / "trajectory_used.yaml"
            self.assertTrue(archived_path.exists())
            self.assertEqual(archived_path.read_text(encoding="utf-8"), trajectory_path.read_text(encoding="utf-8"))
            self.assertEqual(node.session_owner, "manual")

    def test_save_trajectory_clears_empty_trajectory_file_after_delete(self) -> None:
        class FakeTrajectory:
            def __init__(self) -> None:
                self.waypoints = []

            def to_payload(self) -> dict[str, object]:
                return {"waypoints": []}

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            session_root = root / "sessions"
            session_root.mkdir()
            trajectory_path = root / "trajectory.yaml"
            trajectory_path.write_text("old: true\n", encoding="utf-8")

            node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
            node._semi_auto_lock = threading.Lock()
            node._semi_auto_active = False
            node.recorded_trajectory = FakeTrajectory()
            node.trajectory_path = trajectory_path
            node.session_root_path = session_root
            node.session_dir = None
            node.sample_log_path = None
            node.report_path = None
            node.run_log_path = None
            node.session_owner = None
            node.samples = []
            node.current_solution = None
            node._publish_status = lambda *_args, **_kwargs: None
            node._write_recorded_trajectory = lambda: (_ for _ in ()).throw(AssertionError("_write_recorded_trajectory should not run"))
            started_modes: list[str] = []

            def ensure_session_started(owner: str = "manual") -> None:
                started_modes.append(owner)
                node.session_dir = session_root / "manual_session"
                node.session_owner = owner

            node._ensure_session_started = ensure_session_started

            response = EyeToHandCalibrationNode._save_trajectory_callback(node, Trigger.Request(), Trigger.Response())

            self.assertTrue(response.success)
            self.assertFalse(trajectory_path.exists())
            self.assertEqual(started_modes, ["manual"])
            self.assertIsNotNone(node.session_dir)
            self.assertEqual(node.session_owner, "manual")

    def test_save_trajectory_clears_empty_dirty_state_without_existing_file(self) -> None:
        class FakeTrajectory:
            def __init__(self) -> None:
                self.waypoints = []

            def to_payload(self) -> dict[str, object]:
                return {"waypoints": []}

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            session_root = root / "sessions"
            session_root.mkdir()
            trajectory_path = root / "trajectory.yaml"

            node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
            node._semi_auto_lock = threading.Lock()
            node._semi_auto_active = False
            node.recorded_trajectory = FakeTrajectory()
            node.trajectory_path = trajectory_path
            node.session_root_path = session_root
            node.session_dir = session_root / "manual_session"
            node.sample_log_path = None
            node.report_path = None
            node.run_log_path = None
            node.session_owner = "manual"
            node.samples = []
            node.current_solution = None
            node._publish_status = lambda *_args, **_kwargs: None
            node._write_recorded_trajectory = lambda: (_ for _ in ()).throw(AssertionError("_write_recorded_trajectory should not run"))
            started_modes: list[str] = []

            def ensure_session_started(owner: str = "manual") -> None:
                started_modes.append(owner)
                node.session_dir = session_root / "manual_session"
                node.session_owner = owner

            node._ensure_session_started = ensure_session_started

            response = EyeToHandCalibrationNode._save_trajectory_callback(node, Trigger.Request(), Trigger.Response())

            self.assertTrue(response.success)
            self.assertFalse(trajectory_path.exists())
            self.assertEqual(started_modes, ["manual"])
            self.assertEqual(node.session_owner, "manual")
            self.assertIn("Cleared empty trajectory state", response.message)

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
