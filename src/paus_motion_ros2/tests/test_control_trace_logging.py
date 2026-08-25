from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
for package_root in (PERCEPTION_PACKAGE_ROOT, MOTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from paus_motion_ros2.fairino_control_node import (
    FINAL_HOVER_STAGE,
    PRE_APPROACH_STAGE,
    TRACKING_ACTIVE,
    FairinoControlNode,
)


class ControlTraceLoggingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.node = FairinoControlNode.__new__(FairinoControlNode)
        self.node.execute_motion = True
        self.node.orientation_mode = "face_marker_normal"
        self.node.flange_face_axis = "-Z"
        self.node.prefer_positive_z_surface_normal = False
        self.node.enable_safe_lift_on_low_clearance = True
        self.node.safe_lift_step_mm = 80.0
        self.node.safe_lift_above_marker_mm = 180.0
        self.node.safe_lift_max_z_mm = 500.0
        self.node.debug_reset_after_success = False
        self.node.max_execution_stage = FINAL_HOVER_STAGE
        self.node.control_backend = "mock_pose"
        self.node.targeting_mode = "markerless_neck"
        self.node.target_hold_timeout_ms = 500
        self.node.require_locked_target_before_motion = True
        self.node.continue_with_last_locked_target_on_source_loss = True
        self.node.max_marker_displacement_before_relatch_mm = 200.0
        self.node.last_valid_target_point_base_m = [0.5, 0.0, 0.2]
        self.node.last_selector_status = {
            "selected_source": "markerless_neck",
            "source_status": "ok",
            "source_age_ms": 12.0,
        }
        self.node.target_pose_topic = "/selected_target_pose_base"
        self.node.last_target_lock_status = None
        self.node.last_trace_signature = None
        self.node.last_trace_write_monotonic = None
        self.node.latest_risk_flags = []
        self.node.last_tracking_state = None
        self.node.last_completion_reason = None
        self.node.status_publisher = mock.MagicMock()
        self.node.debug_status_publisher = mock.MagicMock()
        self.node.get_logger = mock.MagicMock(return_value=mock.MagicMock())

    def _enable_trace_dir(self, tmpdir: str) -> None:
        self.node.run_dir = Path(tmpdir)
        self.node.control_trace_path = self.node.run_dir / "control_trace.jsonl"
        self.node.control_status_latest_path = self.node.run_dir / "control_status_latest.json"
        self.node.motion_summary_path = self.node.run_dir / "motion_summary.json"
        self.node.run_report_path = self.node.run_dir / "run_report.md"

    def test_control_executed_writes_trace_latest_and_report(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            self._enable_trace_dir(tmpdir)
            (Path(tmpdir) / "selector_status_latest.json").write_text(
                json.dumps(
                    {
                        "source": "target_selector",
                        "status": "ok",
                        "selected_source": "markerless_neck",
                        "published": True,
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            self.node._publish_status(
                event="control_executed",
                error_message="",
                check_passed=True,
                executed=True,
                target_point_base_m=[0.9, 0.0, 0.2],
                target_point_base_mm=[900.0, 0.0, 200.0],
                current_tcp_pose_mmdeg=[500.0, 0.0, 300.0, 180.0, 0.0, -180.0],
                final_hover_pose_mmdeg=[850.0, 0.0, 200.0, 180.0, 0.0, -180.0],
                pre_approach_pose_mmdeg=[770.0, 0.0, 200.0, 180.0, 0.0, -180.0],
                candidate_pose_mmdeg=[770.0, 0.0, 200.0, 180.0, 0.0, -180.0],
                candidate_stage=PRE_APPROACH_STAGE,
                motion_command="MoveJ",
                tracking_state=TRACKING_ACTIVE,
                target_age_ms=600.0,
                marker_visibility_status="not_found",
                transform_validity_status="ok",
            )

            trace_rows = self.node.control_trace_path.read_text(encoding="utf-8").splitlines()
            latest = self.node.control_status_latest_path.read_text(encoding="utf-8")
            summary = self.node.motion_summary_path.read_text(encoding="utf-8")
            report = self.node.run_report_path.read_text(encoding="utf-8")

            self.assertEqual(len(trace_rows), 1)
            self.assertIn("control_executed", trace_rows[0])
            self.assertIn("target_age_over_timeout", latest)
            self.assertIn("target_far_from_tcp", latest)
            self.assertIn("large_target_jump", latest)
            self.assertIn("candidate_not_final_hover", latest)
            self.assertIn("marker_eval_unavailable", latest)
            self.assertNotIn("marker_not_found", latest)
            self.assertIn('"target_validity_source": "selected_target"', latest)
            self.assertIn('"marker_visibility_role": "eval_only"', latest)
            self.assertIn('"control_gate_decision": "tracking"', latest)
            self.assertIn("latest_selector_status", summary)
            self.assertIn("markerless_neck", summary)
            self.assertIn("target_validity_source", summary)
            self.assertIn("marker_visibility_role", report)
            self.assertIn("selector_status", report)
            self.assertIn('"executed": true', summary)
            self.assertIn("Motion Trace Report", report)
            self.assertIn("control_trace.jsonl", report)

    def test_repeated_heartbeat_trace_is_throttled(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            self._enable_trace_dir(tmpdir)
            payload = dict(
                event="tracking_active",
                error_message="",
                check_passed=True,
                target_point_base_m=[0.5, 0.0, 0.2],
                target_point_base_mm=[500.0, 0.0, 200.0],
                current_tcp_pose_mmdeg=[500.0, 0.0, 300.0, 180.0, 0.0, -180.0],
                candidate_stage=FINAL_HOVER_STAGE,
                motion_command="MoveL",
                tracking_state=TRACKING_ACTIVE,
                marker_visibility_status="ok",
                transform_validity_status="ok",
            )

            self.node._publish_status(**payload)
            self.node._publish_status(**payload)

            trace_rows = self.node.control_trace_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(trace_rows), 1)
            latest = self.node.control_status_latest_path.read_text(encoding="utf-8")
            self.assertIn("risk_flags", latest)


if __name__ == "__main__":
    unittest.main()
