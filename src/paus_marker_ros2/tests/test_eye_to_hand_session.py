from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from pathlib import Path

import pytest
import yaml
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
if str(MARKER_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MARKER_PACKAGE_ROOT))

pytest.importorskip("rclpy")
pytest.importorskip("std_srvs.srv")

from std_srvs.srv import Trigger

from paus_marker_ros2.board_observation import BoardPoseEstimate
from paus_marker_ros2.eye_to_hand_calibration_node import CalibrationSample, EyeToHandCalibrationNode, SampleRejectedError
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

    def test_rgb_quality_gate_rejects_high_reprojection_sample(self) -> None:
        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node.max_reprojection_error_px = 2.5
        node.min_board_margin_px = 10.0
        estimate = BoardPoseEstimate(
            camera_to_board_matrix=np.eye(4),
            quality={
                "observation_mode": "rgb_pnp",
                "accepted": True,
                "status": "accepted",
                "reprojection_rms_px": 3.2,
                "board_margin_px": 50.0,
            },
            corners_xy=None,
        )

        gated = EyeToHandCalibrationNode._apply_rgb_quality_gate(node, estimate, (100, 100, 3))

        self.assertIsNone(gated.camera_to_board_matrix)
        self.assertFalse(gated.quality["accepted"])
        self.assertEqual(gated.quality["reject_reason"], "reprojection_error_too_high")
        self.assertFalse(gated.quality["reprojection_filter_passed"])

    def test_capture_retry_selects_best_accepted_candidate(self) -> None:
        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node.capture_retry_count = 3
        node.capture_retry_delay_s = 0.0
        node._stop_lock = threading.Lock()
        node._stop_requested = False
        node.samples = []
        node.session_dir = None
        node._ensure_session_started = lambda owner="semi_auto": None
        node._append_sample_log = lambda sample: None
        node._append_run_log = lambda event, payload=None: None
        node._save_sample_image = lambda image, sample_index: None

        candidates = [
            SampleRejectedError("chessboard_not_found", {"accepted": False, "reject_reason": "chessboard_not_found"}),
            CalibrationSample(
                base_to_tool_matrix=np.eye(4),
                camera_to_board_matrix=np.eye(4),
                image_header_time_s=None,
                image_received_time_s=1.0,
                tcp_read_start_time_s=1.1,
                tcp_read_end_time_s=1.2,
                tcp_pose_mmdeg=[0.0] * 6,
                image_sequence=2,
                observation_mode="rgb_pnp",
                sample_quality={"accepted": True, "reprojection_error_px": 0.5, "board_margin_px": 80.0},
            ),
            CalibrationSample(
                base_to_tool_matrix=np.eye(4),
                camera_to_board_matrix=np.eye(4),
                image_header_time_s=None,
                image_received_time_s=2.0,
                tcp_read_start_time_s=2.1,
                tcp_read_end_time_s=2.2,
                tcp_pose_mmdeg=[0.0] * 6,
                image_sequence=3,
                observation_mode="rgb_pnp",
                sample_quality={"accepted": True, "reprojection_error_px": 0.1, "board_margin_px": 20.0},
            ),
        ]

        def capture_candidate(*, owner: str, attempt_index: int):
            item = candidates[attempt_index - 1]
            if isinstance(item, SampleRejectedError):
                raise item
            return item, {
                "attempt_index": attempt_index,
                "accepted": True,
                "reprojection_error_px": item.sample_quality["reprojection_error_px"],
                "board_margin_px": item.sample_quality["board_margin_px"],
            }

        node._capture_sample_candidate = capture_candidate

        sample, attempts, selected_attempt_index = EyeToHandCalibrationNode._capture_best_sample_with_retries(
            node,
            owner="semi_auto",
            waypoint_name="waypoint_001",
            waypoint_index=1,
            waypoint_count=1,
        )

        self.assertIs(sample, candidates[2])
        self.assertEqual(selected_attempt_index, 3)
        self.assertEqual(len(attempts), 3)
        self.assertEqual(len(node.samples), 1)
        self.assertEqual(node.samples[0].sample_quality["accepted_attempt_count"], 2)
        self.assertEqual(node.samples[0].sample_quality["selected_attempt_index"], 3)

    def test_capture_retry_tie_breaks_by_margin_then_attempt_index(self) -> None:
        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node.capture_retry_count = 3
        node.capture_retry_delay_s = 0.0
        node._stop_lock = threading.Lock()
        node._stop_requested = False
        node.samples = []
        node.session_dir = None
        node._ensure_session_started = lambda owner="semi_auto": None
        node._append_sample_log = lambda sample: None
        node._append_run_log = lambda event, payload=None: None
        node._save_sample_image = lambda image, sample_index: None

        candidates = [
            CalibrationSample(
                base_to_tool_matrix=np.eye(4),
                camera_to_board_matrix=np.eye(4),
                image_header_time_s=None,
                image_received_time_s=1.0,
                tcp_read_start_time_s=1.1,
                tcp_read_end_time_s=1.2,
                tcp_pose_mmdeg=[0.0] * 6,
                image_sequence=1,
                observation_mode="rgb_pnp",
                sample_quality={"accepted": True, "reprojection_error_px": 0.2, "board_margin_px": 30.0},
            ),
            CalibrationSample(
                base_to_tool_matrix=np.eye(4),
                camera_to_board_matrix=np.eye(4),
                image_header_time_s=None,
                image_received_time_s=2.0,
                tcp_read_start_time_s=2.1,
                tcp_read_end_time_s=2.2,
                tcp_pose_mmdeg=[0.0] * 6,
                image_sequence=2,
                observation_mode="rgb_pnp",
                sample_quality={"accepted": True, "reprojection_error_px": 0.2, "board_margin_px": 60.0},
            ),
            CalibrationSample(
                base_to_tool_matrix=np.eye(4),
                camera_to_board_matrix=np.eye(4),
                image_header_time_s=None,
                image_received_time_s=3.0,
                tcp_read_start_time_s=3.1,
                tcp_read_end_time_s=3.2,
                tcp_pose_mmdeg=[0.0] * 6,
                image_sequence=3,
                observation_mode="rgb_pnp",
                sample_quality={"accepted": True, "reprojection_error_px": 0.2, "board_margin_px": 60.0},
            ),
        ]

        def capture_candidate(*, owner: str, attempt_index: int):
            item = candidates[attempt_index - 1]
            return item, {
                "attempt_index": attempt_index,
                "accepted": True,
                "reprojection_error_px": item.sample_quality["reprojection_error_px"],
                "board_margin_px": item.sample_quality["board_margin_px"],
            }

        node._capture_sample_candidate = capture_candidate

        sample, _attempts, selected_attempt_index = EyeToHandCalibrationNode._capture_best_sample_with_retries(
            node,
            owner="semi_auto",
            waypoint_name="waypoint_001",
            waypoint_index=1,
            waypoint_count=1,
        )

        self.assertIs(sample, candidates[1])
        self.assertEqual(selected_attempt_index, 2)

    def test_capture_retry_stop_between_attempts_interrupts_without_commit(self) -> None:
        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node.capture_retry_count = 2
        node.capture_retry_delay_s = 0.1
        node._stop_lock = threading.Lock()
        node._stop_requested = False
        node.samples = []
        node.session_dir = None
        node._ensure_session_started = lambda owner="semi_auto": None
        node._append_sample_log = lambda sample: None
        node._append_run_log = lambda event, payload=None: None
        node._save_sample_image = lambda image, sample_index: None

        first = CalibrationSample(
            base_to_tool_matrix=np.eye(4),
            camera_to_board_matrix=np.eye(4),
            image_header_time_s=None,
            image_received_time_s=1.0,
            tcp_read_start_time_s=1.1,
            tcp_read_end_time_s=1.2,
            tcp_pose_mmdeg=[0.0] * 6,
            image_sequence=1,
            observation_mode="rgb_pnp",
            sample_quality={"accepted": True, "reprojection_error_px": 0.2, "board_margin_px": 30.0},
        )

        def capture_candidate(*, owner: str, attempt_index: int):
            self.assertEqual(attempt_index, 1)
            node._request_stop()
            return first, {"attempt_index": 1, "accepted": True, "reprojection_error_px": 0.2}

        node._capture_sample_candidate = capture_candidate

        with self.assertRaises(Exception) as raised:
            EyeToHandCalibrationNode._capture_best_sample_with_retries(
                node,
                owner="semi_auto",
                waypoint_name="waypoint_001",
                waypoint_index=1,
                waypoint_count=1,
            )

        self.assertEqual(str(raised.exception), "Semi-auto calibration stopped.")
        self.assertEqual(node.samples, [])

    def test_capture_retry_raises_after_all_attempts_rejected(self) -> None:
        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node.capture_retry_count = 2
        node.capture_retry_delay_s = 0.0
        node._stop_lock = threading.Lock()
        node._stop_requested = False
        node.session_dir = None
        rejected_events: list[dict[str, object]] = []
        node._append_run_log = lambda event, payload=None: rejected_events.append(payload or {}) if event == "capture_attempt_rejected" else None

        def capture_candidate(*, owner: str, attempt_index: int):
            raise SampleRejectedError(
                "chessboard_not_found",
                {"accepted": False, "reject_reason": "chessboard_not_found", "capture_attempt_index": attempt_index},
            )

        node._capture_sample_candidate = capture_candidate

        with self.assertRaises(SampleRejectedError) as raised:
            EyeToHandCalibrationNode._capture_best_sample_with_retries(
                node,
                owner="semi_auto",
                waypoint_name="waypoint_001",
                waypoint_index=1,
                waypoint_count=1,
            )

        self.assertEqual(raised.exception.reject_reason, "chessboard_not_found")
        self.assertEqual(raised.exception.sample_quality["capture_attempt_count"], 2)
        self.assertEqual(len(raised.exception.sample_quality["capture_attempts"]), 2)
        self.assertEqual(len(rejected_events), 2)

    def test_semi_auto_stop_before_solve_keeps_samples_and_does_not_save(self) -> None:
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
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            class FakeClient:
                def move_j(self, joint_deg: list[float], **_kwargs: object) -> int:
                    return 0

            class FakeSample:
                sample_quality = {"accepted": True, "reprojection_error_px": 0.2, "board_margin_px": 40.0}

            node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
            node._semi_auto_lock = threading.Lock()
            node._semi_auto_active = False
            node._stop_lock = threading.Lock()
            node._stop_requested = False
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
            node.post_motion_capture_delay_s = 0.0
            node.sample_log_override_path = None
            node._archive_current_trajectory = lambda: None
            node._wait_until_tcp_stable = lambda: [0, 0, 0, 0, 0, 0]
            node._sample_to_log_record = lambda sample: {"sample_index": 1}
            node._save_current_solution = lambda: (_ for _ in ()).throw(AssertionError("save must not run after stop"))
            node._solve_callback = lambda *_args: (_ for _ in ()).throw(AssertionError("solve must not run after stop"))
            published: list[dict[str, object]] = []
            node._publish_status = lambda status, message, payload=None: published.append(
                {"status": status, "message": message, "payload": payload or {}}
            )

            def capture_best(**_kwargs: object) -> tuple[FakeSample, list[dict[str, object]], int]:
                sample = FakeSample()
                node.samples.append(sample)
                node._request_stop()
                return sample, [{"attempt_index": 1, "accepted": True}], 1

            node._capture_best_sample_with_retries = capture_best

            response = EyeToHandCalibrationNode._run_semi_auto_callback(node, Trigger.Request(), Trigger.Response())

            self.assertFalse(response.success)
            self.assertEqual(response.message, "Semi-auto calibration stopped.")
            self.assertEqual(len(node.samples), 1)
            self.assertEqual(published[-1]["status"], "semi_auto_stopped")
            assert node.run_log_path is not None
            self.assertIn('"event": "semi_auto_stopped"', node.run_log_path.read_text(encoding="utf-8"))

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
                SampleRejectedError(
                    "chessboard_not_found",
                    {
                        "accepted": False,
                        "reject_reason": "chessboard_not_found",
                        "capture_attempts": [
                            {"attempt_index": 1, "accepted": False, "reason": "chessboard_not_found"}
                        ],
                    },
                ),
                FakeSample(),
            ])

            def capture_best(**_kwargs: object) -> tuple[FakeSample, list[dict[str, object]], int]:
                item = next(captures)
                if isinstance(item, Exception):
                    raise item
                node.samples.append(item)
                return item, [{"attempt_index": 1, "accepted": True, "reprojection_error_px": 0.2}], 1

            node.capture_retry_count = 1
            node.capture_retry_delay_s = 0.0
            node.post_motion_capture_delay_s = 0.0
            node._stop_lock = threading.Lock()
            node._stop_requested = False
            node._capture_best_sample_with_retries = capture_best

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
