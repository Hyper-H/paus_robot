from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from geometry_msgs.msg import PoseStamped


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
for package_root in (PERCEPTION_PACKAGE_ROOT, MOTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from paus_motion_ros2.control_logic import build_approach_decision
from paus_motion_ros2.fairino_control_node import (
    APPROACH_READY_STAGE,
    FINAL_HOVER_STAGE,
    MOTION_STRATEGY_STAGED_PATIENT_LEFT_FINAL_HOVER,
    PRE_APPROACH_STAGE,
    ROLL_POLICY_BASE_UP_PROJECTION,
    STAGE_PROGRESS_APPROACH_READY_DONE,
    STAGE_PROGRESS_FINAL_HOVER_DONE,
    STAGE_PROGRESS_NONE,
    STAGE_PROGRESS_PRE_APPROACH_DONE,
    FairinoControlNode,
)


class StagedMotionStrategyTests(unittest.TestCase):
    def _node(self) -> FairinoControlNode:
        node = FairinoControlNode.__new__(FairinoControlNode)
        node.motion_strategy = MOTION_STRATEGY_STAGED_PATIENT_LEFT_FINAL_HOVER
        node.approach_ready_joint_deg = None
        node.approach_ready_tolerance_deg = 5.0
        node.approach_ready_vel = 20.0
        node.pre_approach_vel = 10.0
        node.final_hover_vel = 5.0
        node.move_acc = 10.0
        node.move_vel = 10.0
        node.tool_id = 0
        node.user_id = 0
        node.max_direct_final_hover_distance_mm = 120.0
        node.move_to_approach_ready_on_start = True
        node.execute_motion = True
        node.control_backend = "linux_sdk"
        node.linux_client = mock.MagicMock()
        node.admittance_enabled = False
        node.force_worker = None
        node.stage_latch = None
        node.stage_progress = STAGE_PROGRESS_NONE
        node.stage_regression_blocked = False
        node.startup_approach_ready_attempted = False
        node.startup_approach_ready_executed = False
        node.startup_approach_ready_error = None
        node.previous_stage = None
        node.stage_transition_reason = None
        node.target_pose_topic = "/locked_target_pose_base"
        node.targeting_mode = "markerless_neck"
        node.last_target_lock_status = None
        node.get_logger = mock.MagicMock(return_value=mock.MagicMock())
        node.stage_timeline = []
        return node

    def _decision(self):
        decision = mock.MagicMock()
        decision.candidate_stage = PRE_APPROACH_STAGE
        decision.current_tcp_pose_mmdeg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        decision.pre_approach_pose_mmdeg = [300.0, 40.0, 200.0, 0.0, 0.0, 0.0]
        decision.final_hover_pose_mmdeg = [220.0, 40.0, 200.0, 10.0, 20.0, 30.0]
        decision.candidate_pose_mmdeg = [80.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        decision.step_distance_mm = 80.0
        return decision

    def test_missing_approach_ready_prevents_real_staged_execution(self) -> None:
        node = self._node()
        decision = mock.MagicMock()

        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[0, 0, 0, 0, 0, 0])

        self.assertEqual(reason, "approach_ready_not_configured")
        self.assertFalse(changed)

        node.execute_motion = False
        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[0, 0, 0, 0, 0, 0])
        self.assertIsNone(reason)
        self.assertFalse(changed)

    def test_approach_ready_stage_uses_move_j_with_taught_joints(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [1, 2, 3, 4, 5, 6]
        node.linux_client.move_j.return_value = 0

        node._execute_move([0, 0, 0, 0, 0, 0], APPROACH_READY_STAGE)

        node.linux_client.move_j.assert_called_once()
        kwargs = node.linux_client.move_j.call_args.kwargs
        self.assertEqual(kwargs["vel"], 20.0)
        self.assertEqual(kwargs["acc"], 10.0)
        self.assertEqual(node._motion_command_for_stage(APPROACH_READY_STAGE), "MoveJ")

    def test_startup_approach_ready_runs_once_before_target_when_enabled(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [1, 2, 3, 4, 5, 6]
        node.linux_client.move_j.return_value = 0
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node._publish_status = mock.MagicMock()

        handled = node._run_startup_approach_ready_if_needed()
        second_handled = node._run_startup_approach_ready_if_needed()

        self.assertTrue(handled)
        self.assertFalse(second_handled)
        node.linux_client.move_j.assert_called_once()
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_APPROACH_READY_DONE)
        self.assertTrue(node.startup_approach_ready_attempted)
        self.assertTrue(node.startup_approach_ready_executed)
        self.assertEqual(node.last_executed_stage, APPROACH_READY_STAGE)
        self.assertEqual(node._publish_status.call_args.kwargs["event"], "startup_approach_ready_executed")

    def test_startup_approach_ready_does_not_run_when_execute_motion_false(self) -> None:
        node = self._node()
        node.execute_motion = False
        node.approach_ready_joint_deg = [1, 2, 3, 4, 5, 6]

        handled = node._run_startup_approach_ready_if_needed()

        self.assertFalse(handled)
        node.linux_client.move_j.assert_not_called()

    def test_startup_approach_ready_failure_is_latched(self) -> None:
        node = self._node()
        node.startup_approach_ready_attempted = True
        node.startup_approach_ready_executed = False
        node.startup_approach_ready_error = "RuntimeError('MoveJ failed with code 154.')"

        self.assertTrue(node._startup_approach_ready_failed())

    def test_latched_startup_failure_blocks_target_motion(self) -> None:
        node = self._node()
        node.startup_approach_ready_attempted = True
        node.startup_approach_ready_executed = False
        node.startup_approach_ready_error = "RuntimeError('MoveJ failed with code 154.')"
        node._target_pose_to_mmdeg = mock.MagicMock(return_value=[0.0] * 6)
        node._marker_visibility_status = mock.MagicMock(return_value="ok")
        node._transform_validity_status = mock.MagicMock(return_value="ok")
        node._selected_source_status = mock.MagicMock(return_value="ok")
        node.get_clock = mock.MagicMock(return_value=mock.MagicMock(now=mock.MagicMock()))
        message = PoseStamped()

        node._target_callback(message)

        node.linux_client.move_j.assert_not_called()
        node.linux_client.move_j_pose.assert_not_called()
        node.linux_client.move_l.assert_not_called()

    def test_startup_approach_ready_rejects_missing_config_for_real_motion(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = None
        node._publish_status = mock.MagicMock()

        handled = node._run_startup_approach_ready_if_needed()

        self.assertTrue(handled)
        node.linux_client.move_j.assert_not_called()
        self.assertEqual(node.startup_approach_ready_error, "approach_ready_not_configured")
        self.assertEqual(node._publish_status.call_args.kwargs["event"], "startup_approach_ready_rejected")

    def test_pre_approach_uses_ik_seeded_move_j(self) -> None:
        node = self._node()
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node.linux_client.move_j_pose.return_value = 0

        node._execute_move([100, 0, 200, 0, 0, 0], PRE_APPROACH_STAGE)

        node.linux_client.move_j_pose.assert_called_once()
        self.assertEqual(node.linux_client.move_j_pose.call_args.kwargs["vel"], 10.0)

    def test_final_hover_uses_low_speed_move_l(self) -> None:
        node = self._node()
        node.linux_client.move_l.return_value = 0

        node._execute_move([100, 0, 200, 0, 0, 0], FINAL_HOVER_STAGE)

        node.linux_client.move_l.assert_called_once()
        self.assertEqual(node.linux_client.move_l.call_args.kwargs["vel"], 5.0)
        self.assertEqual(node._motion_command_for_stage(FINAL_HOVER_STAGE), "MoveL")

    def test_staged_progress_does_not_return_to_approach_ready_after_ready_done(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_APPROACH_READY_DONE
        decision = self._decision()

        reason, changed = node._apply_staged_motion_policy(
            decision,
            current_joint_deg=[40, 40, 40, 40, 40, 40],
        )

        self.assertIsNone(reason)
        self.assertTrue(changed)
        self.assertEqual(decision.candidate_stage, PRE_APPROACH_STAGE)
        self.assertEqual(decision.candidate_pose_mmdeg, decision.pre_approach_pose_mmdeg)
        self.assertNotEqual(decision.candidate_stage, APPROACH_READY_STAGE)

    def test_pre_approach_candidate_uses_full_pose_without_step_slicing(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_APPROACH_READY_DONE
        decision = self._decision()
        decision.current_tcp_pose_mmdeg = [0, 0, 0, 0, 0, 0]
        decision.pre_approach_pose_mmdeg = [500, 0, 0, 0, 0, 0]
        decision.candidate_pose_mmdeg = [80, 0, 0, 0, 0, 0]

        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[90, 0, 0, 0, 0, 0])

        self.assertIsNone(reason)
        self.assertTrue(changed)
        self.assertEqual(decision.candidate_stage, PRE_APPROACH_STAGE)
        self.assertEqual(decision.candidate_pose_mmdeg, [500, 0, 0, 0, 0, 0])
        self.assertEqual(decision.step_distance_mm, 500.0)

    def test_pre_approach_done_selects_final_hover_full_pose(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_PRE_APPROACH_DONE
        node.max_direct_final_hover_distance_mm = 500.0
        decision = self._decision()
        decision.candidate_stage = PRE_APPROACH_STAGE
        decision.candidate_pose_mmdeg = [80, 0, 0, 0, 0, 0]

        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[90, 0, 0, 0, 0, 0])

        self.assertIsNone(reason)
        self.assertTrue(changed)
        self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE)
        self.assertEqual(decision.candidate_pose_mmdeg, decision.final_hover_pose_mmdeg)

    def test_static_locked_target_replay_is_ready_after_pre_approach(self) -> None:
        node = self._node()
        node.static_target_after_lock = True
        node.locked_target_pose_received = True
        node.last_locked_target_message = PoseStamped()
        node.stage_progress = STAGE_PROGRESS_PRE_APPROACH_DONE
        node.tracking_episode_completed = False
        node.motion_in_progress = False

        self.assertTrue(node._should_replay_locked_target_for_stage_progress())

        node.stage_progress = STAGE_PROGRESS_APPROACH_READY_DONE
        self.assertFalse(node._should_replay_locked_target_for_stage_progress())

    def test_stage_progress_advances_after_successful_stage_execution(self) -> None:
        node = self._node()

        node._mark_stage_progress_after_execution(APPROACH_READY_STAGE)
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_APPROACH_READY_DONE)
        node._mark_stage_progress_after_execution(PRE_APPROACH_STAGE)
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_PRE_APPROACH_DONE)
        node._mark_stage_progress_after_execution(FINAL_HOVER_STAGE)
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_FINAL_HOVER_DONE)

    def test_stage_regression_blocked_is_recorded_for_underlying_earlier_candidate(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_PRE_APPROACH_DONE
        node.max_direct_final_hover_distance_mm = 500.0
        decision = self._decision()
        decision.candidate_stage = PRE_APPROACH_STAGE

        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[90, 0, 0, 0, 0, 0])

        self.assertIsNone(reason)
        self.assertTrue(changed)
        self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE)
        self.assertTrue(node.stage_regression_blocked)

    def test_far_final_hover_is_rejected_for_real_staged_execution(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_PRE_APPROACH_DONE
        decision = mock.MagicMock()
        decision.candidate_stage = FINAL_HOVER_STAGE
        decision.current_tcp_pose_mmdeg = [0, 0, 0, 0, 0, 0]
        decision.final_hover_pose_mmdeg = [500, 0, 0, 0, 0, 0]

        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[0, 0, 0, 0, 0, 0])

        self.assertEqual(reason, "current_tcp_too_far_for_final_hover")
        self.assertFalse(changed)

    def test_locked_target_drift_is_warning_only_for_real_execution(self) -> None:
        node = self._node()
        node.last_target_lock_status = {
            "state": "drift_warning",
            "locked": True,
            "latest_live_to_locked_mm": 81.0,
            "drift_warning_mm": 80.0,
            "drift_action": "warn_only",
        }

        self.assertTrue(node._target_lock_drift_warning_exceeded())
        flags = node._compute_risk_flags(
            {
                "candidate_stage": FINAL_HOVER_STAGE,
                "live_to_locked_drift_mm": 81.0,
                "drift_warning_mm": 80.0,
            }
        )
        self.assertIn("drift_exceeded_warning_threshold", flags)

    def test_locked_target_age_is_not_reported_as_timeout_risk(self) -> None:
        node = self._node()
        flags = node._compute_risk_flags(
            {
                "candidate_stage": FINAL_HOVER_STAGE,
                "target_age_ms": 10_000.0,
                "stage_progress": STAGE_PROGRESS_PRE_APPROACH_DONE,
                "approach_ready_configured": True,
            }
        )

        self.assertNotIn("target_age_over_timeout", flags)

    def test_fixed_roll_policy_is_stable_across_current_tcp_orientation(self) -> None:
        kwargs = dict(
            target_position_base_m=[0.5, 0.0, 0.2],
            raw_target_orientation_rpy_deg=[0.0, 0.0, 0.0],
            frame_id="robot_base",
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=50.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=0.0,
            workspace_min_mm=[-1000.0, -1000.0, -1000.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            prefer_positive_z_surface_normal=False,
            enable_safe_lift_on_low_clearance=False,
            roll_policy=ROLL_POLICY_BASE_UP_PROJECTION,
        )

        d1 = build_approach_decision(current_tcp_pose_mmdeg=[500, 0, 500, 0, 0, 0], **kwargs)
        d2 = build_approach_decision(current_tcp_pose_mmdeg=[500, 0, 500, 90, 45, 30], **kwargs)

        self.assertTrue(d1.check_passed)
        self.assertTrue(d2.check_passed)
        self.assertEqual(
            [round(v, 6) for v in d1.final_hover_pose_mmdeg[3:6]],
            [round(v, 6) for v in d2.final_hover_pose_mmdeg[3:6]],
        )

    def test_pre_approach_pose_is_already_face_aligned(self) -> None:
        decision = build_approach_decision(
            target_position_base_m=[0.5, 0.0, 0.2],
            raw_target_orientation_rpy_deg=[0.0, 0.0, 0.0],
            frame_id="robot_base",
            current_tcp_pose_mmdeg=[500.0, 0.0, 500.0, 0.0, 90.0, 0.0],
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=50.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=0.0,
            workspace_min_mm=[-1000.0, -1000.0, -1000.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            prefer_positive_z_surface_normal=False,
            enable_safe_lift_on_low_clearance=False,
            roll_policy=ROLL_POLICY_BASE_UP_PROJECTION,
        )

        self.assertTrue(decision.check_passed, msg=decision.error_message)
        self.assertEqual(
            [round(v, 6) for v in decision.pre_approach_pose_mmdeg[3:6]],
            [round(v, 6) for v in decision.final_hover_pose_mmdeg[3:6]],
        )
        self.assertEqual(decision.candidate_stage, PRE_APPROACH_STAGE)
        self.assertEqual(
            [round(v, 6) for v in decision.candidate_pose_mmdeg[3:6]],
            [round(v, 6) for v in decision.final_hover_pose_mmdeg[3:6]],
        )

    def test_motion_report_includes_staged_diagnostics(self) -> None:
        node = self._node()
        node.execute_motion = False
        node.orientation_mode = "face_marker_normal"
        node.flange_face_axis = "-Z"
        node.roll_policy = ROLL_POLICY_BASE_UP_PROJECTION
        node.roll_offset_deg = 0.0
        node.prefer_positive_z_surface_normal = False
        node.enable_safe_lift_on_low_clearance = True
        node.safe_lift_step_mm = 80.0
        node.safe_lift_above_marker_mm = 180.0
        node.safe_lift_max_z_mm = 500.0
        node.debug_reset_after_success = False
        node.max_execution_stage = FINAL_HOVER_STAGE
        node.hover_clearance_mm = 50.0
        node.pre_approach_distance_mm = 80.0
        node.target_hold_timeout_ms = 500
        node.last_selector_status = None
        node.last_target_lock_status = {
            "state": "drift_warning",
            "locked": True,
            "latest_live_to_locked_mm": 92.0,
            "drift_warning_mm": 80.0,
            "drift_action": "warn_only",
        }
        node.last_valid_target_point_base_m = None
        node.status_publisher = mock.MagicMock()
        node.debug_status_publisher = mock.MagicMock()
        node.last_trace_signature = None
        node.last_trace_write_monotonic = None
        node.latest_risk_flags = []
        node.last_tracking_state = None
        node.last_completion_reason = None

        with tempfile.TemporaryDirectory() as tmpdir:
            node.run_dir = Path(tmpdir)
            node.control_trace_path = node.run_dir / "control_trace.jsonl"
            node.control_status_latest_path = node.run_dir / "control_status_latest.json"
            node.motion_summary_path = node.run_dir / "motion_summary.json"
            node.run_report_path = node.run_dir / "run_report.md"

            node._publish_status(
                event="control_dry_run",
                error_message="",
                check_passed=True,
                target_point_base_mm=[500, 0, 200],
                target_pose_base_mmdeg=[500, 0, 200, 0, 0, 0],
                current_tcp_pose_mmdeg=[0, 0, 200, 0, 0, 0],
                surface_normal_base=[-1, 0, 0],
                final_hover_pose_mmdeg=[450, 0, 200, 0, 0, 0],
                pre_approach_pose_mmdeg=[370, 0, 200, 0, 0, 0],
                candidate_pose_mmdeg=[370, 0, 200, 0, 0, 0],
                candidate_stage=PRE_APPROACH_STAGE,
                motion_command="MoveJ",
                tracking_state="tracking_active",
                current_joint_deg=[0, 0, 0, 0, 0, 0],
                current_tcp_to_final_hover_mm=450.0,
                current_tcp_to_pre_approach_mm=370.0,
                pre_approach_to_final_hover_mm=80.0,
                locked_target_pose_base_mmdeg=[500, 0, 200, 0, 0, 0],
            )

            summary = json.loads(node.motion_summary_path.read_text(encoding="utf-8"))
            report = node.run_report_path.read_text(encoding="utf-8")

        self.assertEqual(summary["motion_strategy"], MOTION_STRATEGY_STAGED_PATIENT_LEFT_FINAL_HOVER)
        self.assertEqual(summary["roll_policy"], ROLL_POLICY_BASE_UP_PROJECTION)
        self.assertEqual(summary["live_to_locked_drift_mm"], 92.0)
        self.assertIn("Approach Ready", report)
        self.assertIn("Target Lock", report)
        self.assertIn("current_tcp_to_final_hover_mm", report)


if __name__ == "__main__":
    unittest.main()
