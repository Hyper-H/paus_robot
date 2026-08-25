from __future__ import annotations

import sys
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
    DEFAULT_LOCKED_TARGET_POSE_TOPIC,
    MARKER_ROLE_CONTROL_GATE,
    MARKER_ROLE_EVAL_ONLY,
    TARGET_VALIDITY_LOCKED_TARGET,
    TARGET_VALIDITY_MARKER_VISIBILITY,
    TARGET_VALIDITY_SELECTED_TARGET,
    FairinoControlNode,
)


class SourceAwareControlValidityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.node = FairinoControlNode.__new__(FairinoControlNode)
        self.node.target_hold_timeout_ms = 500
        self.node.last_selector_status = {
            "selected_source": "markerless_neck",
            "source_status": "ok",
            "source_age_ms": 12.5,
        }
        self.node.targeting_mode = "markerless_neck"
        self.node.target_pose_topic = "/selected_target_pose_base"
        self.node.last_target_lock_status = None
        self.node.require_locked_target_before_motion = True
        self.node.continue_with_last_locked_target_on_source_loss = True

    def test_markerless_uses_selected_target_validity(self) -> None:
        self.assertFalse(self.node._uses_marker_visibility_gate())
        self.assertEqual(self.node._target_validity_source(), TARGET_VALIDITY_SELECTED_TARGET)
        self.assertEqual(self.node._marker_visibility_role(), MARKER_ROLE_EVAL_ONLY)
        self.assertTrue(
            self.node._source_fresh_for_tracking(
                marker_visibility_status="not_found",
                transform_validity_status="invalid",
                selected_source_status="ok",
            )
        )

    def test_markerless_locked_target_uses_lock_validity_not_selector_freshness(self) -> None:
        self.node.target_pose_topic = DEFAULT_LOCKED_TARGET_POSE_TOPIC
        self.node.last_selector_status = {
            "selected_source": "markerless_neck",
            "source_status": "stale",
            "source_age_ms": 1200.0,
        }
        self.node.last_target_lock_status = {
            "state": "locked",
            "locked": True,
        }

        self.assertEqual(self.node._target_validity_source(), TARGET_VALIDITY_LOCKED_TARGET)
        self.assertTrue(
            self.node._source_fresh_for_tracking(
                marker_visibility_status="not_found",
                transform_validity_status="invalid",
                selected_source_status="stale",
            )
        )
        self.assertIsNone(
            self.node._target_loss_reason(
                target_age_ms=600.0,
                marker_visibility_status="not_found",
                transform_validity_status="invalid",
                selected_source_status="stale",
            )
        )

    def test_markerless_locked_target_reports_not_locked_when_lock_missing(self) -> None:
        self.node.target_pose_topic = DEFAULT_LOCKED_TARGET_POSE_TOPIC
        self.node.last_target_lock_status = {
            "state": "collecting",
            "locked": False,
        }

        self.assertFalse(
            self.node._source_fresh_for_tracking(
                marker_visibility_status="ok",
                transform_validity_status="ok",
                selected_source_status="ok",
            )
        )
        self.assertEqual(
            self.node._target_loss_reason(
                target_age_ms=100.0,
                marker_visibility_status="ok",
                transform_validity_status="ok",
                selected_source_status="ok",
            ),
            "target_lock_not_locked",
        )

    def test_locked_target_message_is_not_allowed_for_motion_until_locked(self) -> None:
        self.node.target_pose_topic = DEFAULT_LOCKED_TARGET_POSE_TOPIC
        self.node.last_target_lock_status = {
            "state": "collecting",
            "locked": False,
        }
        self.assertFalse(self.node._target_message_allowed_for_motion())

        self.node.last_target_lock_status = {
            "state": "locked",
            "locked": True,
        }
        self.assertTrue(self.node._target_message_allowed_for_motion())

    def test_marker_mode_uses_marker_visibility_gate(self) -> None:
        self.node.targeting_mode = "marker"

        self.assertTrue(self.node._uses_marker_visibility_gate())
        self.assertEqual(self.node._target_validity_source(), TARGET_VALIDITY_MARKER_VISIBILITY)
        self.assertEqual(self.node._marker_visibility_role(), MARKER_ROLE_CONTROL_GATE)
        self.assertFalse(
            self.node._source_fresh_for_tracking(
                marker_visibility_status="not_found",
                transform_validity_status="ok",
                selected_source_status="ok",
            )
        )

    def test_markerless_timeout_reason_points_to_selected_target(self) -> None:
        reason = self.node._target_loss_reason(
            target_age_ms=600.0,
            marker_visibility_status="not_found",
            transform_validity_status="invalid",
            selected_source_status="stale",
        )

        self.assertEqual(reason, "selected_target_timeout")

    def test_locked_target_timeout_can_be_restored_as_fatal_when_configured(self) -> None:
        self.node.target_pose_topic = DEFAULT_LOCKED_TARGET_POSE_TOPIC
        self.node.last_target_lock_status = {
            "state": "locked",
            "locked": True,
        }
        self.node.continue_with_last_locked_target_on_source_loss = False

        reason = self.node._target_loss_reason(
            target_age_ms=600.0,
            marker_visibility_status="not_found",
            transform_validity_status="invalid",
            selected_source_status="stale",
        )

        self.assertEqual(reason, "selected_target_timeout")

    def test_publish_status_defaults_explain_gate_fields(self) -> None:
        self.node.execute_motion = False
        self.node.orientation_mode = "face_marker_normal"
        self.node.flange_face_axis = "-Z"
        self.node.prefer_positive_z_surface_normal = False
        self.node.enable_safe_lift_on_low_clearance = True
        self.node.safe_lift_step_mm = 80.0
        self.node.safe_lift_above_marker_mm = 180.0
        self.node.safe_lift_max_z_mm = 500.0
        self.node.debug_reset_after_success = False
        self.node.max_execution_stage = "final_hover"
        self.node.control_backend = "mock_pose"
        self.node.max_marker_displacement_before_relatch_mm = 200.0
        self.node.require_locked_target_before_motion = True
        self.node.continue_with_last_locked_target_on_source_loss = True
        self.node.last_valid_target_point_base_m = None
        self.node.status_publisher = mock.MagicMock()
        self.node.debug_status_publisher = mock.MagicMock()
        self.node.get_logger = mock.MagicMock(return_value=mock.MagicMock())
        self.node._write_control_log = mock.MagicMock()
        self.node.last_tracking_state = None
        self.node.last_completion_reason = None

        self.node._publish_status(
            event="control_dry_run",
            error_message="",
            check_passed=True,
            tracking_state="tracking_active",
            marker_visibility_status="not_found",
            transform_validity_status="invalid",
        )

        debug_payload = self.node.debug_status_publisher.publish.call_args.args[0].data
        self.assertIn('"target_validity_source": "selected_target"', debug_payload)
        self.assertIn('"marker_visibility_role": "eval_only"', debug_payload)
        self.assertIn('"selected_source": "markerless_neck"', debug_payload)
        self.assertIn('"selected_source_status": "ok"', debug_payload)
        self.assertIn("marker_eval_unavailable", debug_payload)
        self.assertNotIn("marker_not_found", debug_payload)


if __name__ == "__main__":
    unittest.main()
