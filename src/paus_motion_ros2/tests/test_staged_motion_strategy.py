from __future__ import annotations

import json
import math
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

from paus_motion_ros2.control_logic import build_approach_decision
from paus_motion_ros2.fairino_control_node import (
    APPROACH_READY_STAGE,
    FINAL_HOVER_STAGE,
    MOTION_STRATEGY_STAGED_PATIENT_LEFT_FINAL_HOVER,
    PRE_APPROACH_STAGE,
    REORIENT_STAGE,
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
        node.final_hover_servo_enabled = True
        node.final_hover_servo_cmd_t_s = 0.0
        node.final_hover_servo_max_step_mm = 1.0
        node.final_hover_servo_max_step_deg = 1.0
        node.final_hover_servo_ready_timeout_s = 0.0
        node.final_hover_servo_orientation_enabled = False
        node.pre_approach_distance_mm = 80.0
        node.move_acc = 10.0
        node.move_vel = 10.0
        node.joint_motion_blend_time_ms = 0.0
        node.joint_motion_done_timeout_s = 1.0
        node.tool_id = 0
        node.user_id = 0
        node.max_direct_final_hover_distance_mm = 120.0
        node.move_to_approach_ready_on_start = True
        node.execute_motion = True
        node.control_backend = "linux_sdk"
        node.linux_client = mock.MagicMock()
        node.linux_client.prepare_motion.return_value = {"ResetAllError": 0, "RobotEnable": 0, "Mode": 0}
        node.linux_client.get_actual_tcp_num.return_value = (0, 0)
        node.linux_client.get_actual_tcp_pose.return_value = (0, [100.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node.linux_client.servo_move_start.return_value = 0
        node.linux_client.servo_cart_tool_delta.return_value = 0
        node.linux_client.servo_move_end.return_value = 0
        node.linux_client.get_robot_motion_done.return_value = (0, 1)
        node.linux_client.get_motion_queue_length.return_value = (0, 0)
        node.sdk_motion_prepared = False
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
        node.target_hold_timeout_ms = 500
        node.require_locked_target_before_motion = True
        node.continue_with_last_locked_target_on_source_loss = True
        node.get_logger = mock.MagicMock(return_value=mock.MagicMock())
        return node

    def _decision(self):
        decision = mock.MagicMock()
        decision.candidate_stage = PRE_APPROACH_STAGE
        decision.current_tcp_pose_mmdeg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        decision.pre_approach_pose_mmdeg = [300.0, 40.0, 200.0, 0.0, 0.0, 0.0]
        decision.final_hover_pose_mmdeg = [220.0, 40.0, 200.0, 10.0, 20.0, 30.0]
        decision.candidate_pose_mmdeg = [80.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        decision.step_distance_mm = 80.0
        decision.surface_normal_base = None
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
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [1, 2, 3, 4, 5, 6])

        node._execute_move([0, 0, 0, 0, 0, 0], APPROACH_READY_STAGE)

        node.linux_client.move_j.assert_called_once()
        kwargs = node.linux_client.move_j.call_args.kwargs
        self.assertEqual(kwargs["vel"], 20.0)
        self.assertEqual(kwargs["acc"], 10.0)
        self.assertEqual(node._motion_command_for_stage(APPROACH_READY_STAGE), "MoveJ")

    def test_execute_move_prepares_sdk_motion_once(self) -> None:
        node = self._node()
        node.linux_client.move_l.return_value = 0

        node._execute_move([100, 0, 200, 0, 0, 0], FINAL_HOVER_STAGE)
        node._execute_move([101, 0, 200, 0, 0, 0], FINAL_HOVER_STAGE)

        node.linux_client.prepare_motion.assert_called_once()

    def test_approach_ready_uses_nonblocking_move_j_and_waits_until_idle(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.linux_client.move_j.return_value = 0

        node._execute_move([0, 0, 0, 0, 0, 0], APPROACH_READY_STAGE)

        self.assertEqual(node.linux_client.move_j.call_args.kwargs["blend_time_ms"], 0.0)
        node.linux_client.get_actual_joint_pos_degree.assert_called()

    def test_pre_approach_waits_until_tcp_pose_reaches_commanded_pose(self) -> None:
        node = self._node()
        node.move_to_approach_ready_on_start = False
        node.linux_client.move_j_pose.return_value = 0
        node.linux_client.get_actual_tcp_num.return_value = (0, 0)
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node.linux_client.get_actual_tcp_pose.side_effect = [
            (0, [0, 0, 200, 0, 0, 0]),
            (0, [100, 0, 200, 0, 0, 0]),
        ]

        node._execute_move([100, 0, 200, 0, 0, 0], PRE_APPROACH_STAGE)

        self.assertGreaterEqual(node.linux_client.get_actual_tcp_pose.call_count, 2)

    def test_pose_target_wait_accepts_grace_period_arrival_after_timeout(self) -> None:
        node = self._node()
        node.pose_target_timeout_grace_s = 0.06
        node.stage_switch_buffer_mm = 10.0
        node.completion_normal_tolerance_deg = 5.0
        node.linux_client.get_actual_tcp_pose.side_effect = [
            (0, [0, 0, 200, 0, 0, 0]),
            (0, [100, 0, 200, 0, 0, 0]),
        ]

        node._wait_for_pose_target_reached([100, 0, 200, 0, 0, 0], timeout_s=0.0)

        self.assertEqual(node.linux_client.get_actual_tcp_pose.call_count, 2)

    def test_execute_move_rejects_failed_sdk_motion_preparation(self) -> None:
        node = self._node()
        node.linux_client.prepare_motion.return_value = {"ResetAllError": 0, "RobotEnable": 14}

        with self.assertRaisesRegex(RuntimeError, "motion preparation failed"):
            node._execute_move([100, 0, 200, 0, 0, 0], FINAL_HOVER_STAGE)

        node.linux_client.move_l.assert_not_called()

    def test_release_robot_control_stops_motion_and_enters_manual_mode(self) -> None:
        node = self._node()
        node.sdk_motion_prepared = True
        node.linux_client.servo_move_end.return_value = 0
        node.linux_client.stop_motion.return_value = 0
        node.linux_client.set_manual_mode.return_value = 0

        result = node._release_robot_control()

        self.assertEqual(result, {"ServoMoveEnd": 0, "StopMotion": 0, "ModeManual": 0})
        self.assertFalse(node.sdk_motion_prepared)
        node.linux_client.servo_move_end.assert_called_once()
        node.linux_client.stop_motion.assert_called_once()
        node.linux_client.set_manual_mode.assert_called_once()

    def test_current_tcp_pose_uses_configured_tool_when_active_tool_differs(self) -> None:
        node = self._node()
        node.tool_id = 1
        node.linux_client.get_actual_tcp_num.return_value = (0, 0)
        node.linux_client.get_actual_tool_flange_pose.return_value = (0, [100, 200, 300, 0, 0, 0])
        node.linux_client.get_tool_coord_with_id.return_value = (0, [10, 0, 0, 0, 0, 0])

        pose = node._get_current_tcp_pose_mmdeg()

        self.assertEqual([round(value, 6) for value in pose], [110, 200, 300, 0, 0, 0])
        node.linux_client.get_actual_tcp_pose.assert_not_called()
        node.linux_client.get_tool_coord_with_id.assert_called_once_with(1)

    def test_current_tcp_pose_uses_robot_tcp_when_active_tool_matches(self) -> None:
        node = self._node()
        node.tool_id = 1
        node.linux_client.get_actual_tcp_num.return_value = (0, 1)
        node.linux_client.get_actual_tcp_pose.return_value = (0, [1, 2, 3, 4, 5, 6])

        pose = node._get_current_tcp_pose_mmdeg()

        self.assertEqual(pose, [1, 2, 3, 4, 5, 6])
        node.linux_client.get_actual_tool_flange_pose.assert_not_called()

    def test_command_pose_converts_configured_tool_target_to_active_tool(self) -> None:
        node = self._node()
        node.tool_id = 1
        node.linux_client.get_actual_tcp_num.return_value = (0, 0)
        node.linux_client.get_tool_coord_with_id.side_effect = [
            (0, [10, 0, 0, 0, 0, 0]),
            (0, [0, 0, 0, 0, 0, 0]),
        ]

        pose, tool_id = node._command_pose_and_tool_for_active_tcp([110, 200, 300, 0, 0, 0])

        self.assertEqual(tool_id, 0)
        self.assertEqual([round(value, 6) for value in pose], [100, 200, 300, 0, 0, 0])

    def test_execute_move_uses_configured_tool_pose_for_pose_motion(self) -> None:
        node = self._node()
        node.tool_id = 1
        node.move_to_approach_ready_on_start = False
        node.linux_client.get_actual_tcp_num.return_value = (0, 0)
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node.linux_client.get_tool_coord_with_id.side_effect = [
            (0, [10, 0, 0, 0, 0, 0]),
            (0, [0, 0, 0, 0, 0, 0]),
        ]
        node.linux_client.move_j_pose.return_value = 0
        node._wait_for_pose_target_reached = mock.MagicMock()

        node._execute_move([110, 200, 300, 0, 0, 0], PRE_APPROACH_STAGE)

        node.linux_client.move_j_pose.assert_called_once()
        kwargs = node.linux_client.move_j_pose.call_args.kwargs
        self.assertEqual(kwargs["tool_id"], 0)
        self.assertEqual([round(value, 6) for value in node.linux_client.move_j_pose.call_args.args[0]], [100, 200, 300, 0, 0, 0])

    def test_startup_approach_ready_runs_once_before_target_when_enabled(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [1, 2, 3, 4, 5, 6]
        node.linux_client.move_j.return_value = 0
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [1, 2, 3, 4, 5, 6])
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

    def test_startup_approach_ready_rejects_missing_config_for_real_motion(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = None
        node._publish_status = mock.MagicMock()

        handled = node._run_startup_approach_ready_if_needed()

        self.assertTrue(handled)
        node.linux_client.move_j.assert_not_called()
        self.assertEqual(node.startup_approach_ready_error, "approach_ready_not_configured")
        self.assertEqual(node._publish_status.call_args.kwargs["event"], "startup_approach_ready_rejected")

    def test_pre_approach_uses_ik_seeded_move_j_pose(self) -> None:
        node = self._node()
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node.linux_client.move_j_pose.return_value = 0

        executed_pose = node._execute_move([100, 0, 200, 0, 0, 0], PRE_APPROACH_STAGE)

        node.linux_client.move_l.assert_not_called()
        node.linux_client.move_j_pose.assert_called_once()
        self.assertEqual(node.linux_client.move_j_pose.call_args.kwargs["vel"], 10.0)
        self.assertEqual(executed_pose, [100, 0, 200, 0, 0, 0])
        self.assertEqual(node._motion_command_for_stage(PRE_APPROACH_STAGE), "MoveJ")

    def test_pre_approach_backs_off_when_move_j_pose_is_unreachable(self) -> None:
        node = self._node()
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node.linux_client.get_actual_tcp_pose.return_value = (0, [0, 0, 200, 0, 0, 0])
        node.linux_client.move_j_pose.side_effect = [112, 0]

        executed_pose = node._execute_move([20, 0, 200, 0, 0, 0], PRE_APPROACH_STAGE)

        self.assertEqual(node.linux_client.move_j_pose.call_count, 2)
        self.assertEqual([round(value, 6) for value in executed_pose], [10, 0, 200, 0, 0, 0])
        self.assertEqual(node.last_move_j_pose_backoff[-1]["result"], 0)

    def test_pre_approach_uses_ik_seeded_move_j_pose_when_startup_ready_disabled(self) -> None:
        node = self._node()
        node.move_to_approach_ready_on_start = False
        node.linux_client.get_actual_joint_pos_degree.return_value = (0, [0, 0, 0, 0, 0, 0])
        node.linux_client.move_j_pose.return_value = 0

        node._execute_move([100, 0, 200, 0, 0, 0], PRE_APPROACH_STAGE)

        node.linux_client.move_l.assert_not_called()
        node.linux_client.move_j_pose.assert_called_once()
        self.assertEqual(node.linux_client.move_j_pose.call_args.kwargs["vel"], 10.0)
        self.assertEqual(node._motion_command_for_stage(PRE_APPROACH_STAGE), "MoveJ")

    def test_final_hover_uses_servo_cart_small_steps(self) -> None:
        node = self._node()

        node._execute_move([103, 0, 200, 0, 0, 0], FINAL_HOVER_STAGE)

        node.linux_client.move_l.assert_not_called()
        node.linux_client.servo_move_start.assert_called_once()
        self.assertEqual(node.linux_client.servo_cart_tool_delta.call_count, 3)
        node.linux_client.servo_move_end.assert_called_once()
        self.assertEqual(node._motion_command_for_stage(FINAL_HOVER_STAGE), "ServoCart")

    def test_final_hover_servo_uses_short_rotation_path_across_rpy_wrap(self) -> None:
        node = self._node()
        node.final_hover_servo_orientation_enabled = True
        node.final_hover_servo_max_step_mm = 100.0
        node.final_hover_servo_max_step_deg = 1.1
        node.linux_client.get_actual_tcp_pose.return_value = (0, [100.0, 0.0, 200.0, 0.0, 0.0, 179.0])

        node._execute_move([100.0, 0.0, 200.0, 0.0, 0.0, -179.0], FINAL_HOVER_STAGE)

        self.assertEqual(node.linux_client.servo_cart_tool_delta.call_count, 2)
        final_pose = node.linux_client.servo_cart_tool_delta.call_args.args[0]
        self.assertAlmostEqual(final_pose[5], 181.0, places=6)

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

    def test_staged_motion_starts_at_pre_approach_when_startup_ready_disabled(self) -> None:
        node = self._node()
        node.move_to_approach_ready_on_start = False
        node.max_direct_final_hover_distance_mm = 500.0
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_NONE
        decision = self._decision()

        reason, changed = node._apply_staged_motion_policy(
            decision,
            current_joint_deg=[40, 40, 40, 40, 40, 40],
        )

        self.assertIsNone(reason)
        self.assertTrue(changed)
        self.assertEqual(node._stage_sequence_planned(), [REORIENT_STAGE, PRE_APPROACH_STAGE, FINAL_HOVER_STAGE])
        self.assertEqual(decision.candidate_stage, PRE_APPROACH_STAGE)
        self.assertEqual(decision.candidate_pose_mmdeg, decision.pre_approach_pose_mmdeg)
        self.assertNotEqual(decision.candidate_stage, APPROACH_READY_STAGE)

    def test_staged_motion_skips_pre_approach_when_current_is_already_closer_to_final(self) -> None:
        node = self._node()
        node.move_to_approach_ready_on_start = False
        node.max_direct_final_hover_distance_mm = 500.0
        node.stage_progress = STAGE_PROGRESS_NONE
        decision = self._decision()
        decision.current_tcp_pose_mmdeg = [210, 40, 200, 0, 0, 0]
        decision.pre_approach_pose_mmdeg = [300, 40, 200, 0, 0, 0]
        decision.final_hover_pose_mmdeg = [220, 40, 200, 0, 0, 0]

        reason, changed = node._apply_staged_motion_policy(
            decision,
            current_joint_deg=[40, 40, 40, 40, 40, 40],
        )

        self.assertIsNone(reason)
        self.assertTrue(changed)
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_PRE_APPROACH_DONE)
        self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE)
        self.assertEqual(decision.candidate_pose_mmdeg, decision.final_hover_pose_mmdeg)

    def test_staged_motion_rejects_far_pre_approach_when_startup_ready_disabled(self) -> None:
        node = self._node()
        node.move_to_approach_ready_on_start = False
        node.max_direct_final_hover_distance_mm = 120.0
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_NONE
        decision = self._decision()
        decision.current_tcp_pose_mmdeg = [0, 0, 0, 0, 0, 0]
        decision.pre_approach_pose_mmdeg = [500, 0, 0, 0, 0, 0]

        reason, changed = node._apply_staged_motion_policy(
            decision,
            current_joint_deg=[40, 40, 40, 40, 40, 40],
        )

        self.assertEqual(reason, "current_tcp_too_far_for_pre_approach")
        self.assertFalse(changed)
        self.assertEqual(decision.candidate_stage, PRE_APPROACH_STAGE)

    def test_pre_approach_uses_full_candidate_after_approach_ready(self) -> None:
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
        self.assertEqual(decision.candidate_pose_mmdeg, decision.pre_approach_pose_mmdeg)
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

    def test_pre_approach_done_rejects_final_hover_when_normal_is_misaligned(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_PRE_APPROACH_DONE
        node.max_direct_final_hover_distance_mm = 500.0
        node.flange_face_axis = "-Z"
        node.completion_normal_tolerance_deg = 5.0
        decision = self._decision()
        decision.current_tcp_pose_mmdeg = [300.0, 40.0, 200.0, 0.0, 0.0, 0.0]
        decision.surface_normal_base = [1.0, 0.0, 0.0]

        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[90, 0, 0, 0, 0, 0])

        self.assertEqual(reason, "pre_approach_reached_with_misaligned_tcp")
        self.assertFalse(changed)

    def test_misaligned_tcp_reorients_before_pre_approach_motion(self) -> None:
        node = self._node()
        node.approach_ready_joint_deg = [0, 0, 0, 0, 0, 0]
        node.stage_progress = STAGE_PROGRESS_APPROACH_READY_DONE
        node.flange_face_axis = "-Z"
        node.completion_normal_tolerance_deg = 5.0
        decision = self._decision()
        decision.current_tcp_pose_mmdeg = [300.0, 40.0, 500.0, 0.0, 0.0, 0.0]
        decision.surface_normal_base = [1.0, 0.0, 0.0]

        reason, changed = node._apply_staged_motion_policy(decision, current_joint_deg=[90, 0, 0, 0, 0, 0])

        self.assertIsNone(reason)
        self.assertTrue(changed)
        self.assertEqual(decision.candidate_stage, REORIENT_STAGE)
        self.assertGreater(
            node._distance_between_pose_positions_mm(decision.candidate_pose_mmdeg, decision.pre_approach_pose_mmdeg),
            170.0,
        )
        self.assertEqual(decision.candidate_pose_mmdeg[3:6], decision.final_hover_pose_mmdeg[3:6])

    def test_pre_approach_pose_uses_final_hover_orientation(self) -> None:
        decision = build_approach_decision(
            target_position_base_m=[0.5, 0.0, 0.2],
            raw_target_orientation_rpy_deg=[0.0, 90.0, 0.0],
            frame_id="robot_base",
            current_tcp_pose_mmdeg=[300.0, 40.0, 500.0, 11.0, 22.0, 33.0],
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=50.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=0.0,
            min_plane_clearance_mm=0.0,
            workspace_min_mm=[-1000.0, -1000.0, -1000.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            prefer_positive_z_surface_normal=False,
            enable_safe_lift_on_low_clearance=False,
            roll_policy=ROLL_POLICY_BASE_UP_PROJECTION,
        )

        self.assertTrue(decision.check_passed)
        self.assertEqual(
            [round(value, 6) for value in decision.pre_approach_pose_mmdeg[3:6]],
            [round(value, 6) for value in decision.final_hover_pose_mmdeg[3:6]],
        )

    def test_stage_progress_advances_after_successful_stage_execution(self) -> None:
        node = self._node()

        node._mark_stage_progress_after_execution(APPROACH_READY_STAGE)
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_APPROACH_READY_DONE)
        node._mark_stage_progress_after_execution(
            PRE_APPROACH_STAGE,
            candidate_pose_mmdeg=[500, 0, 0, 0, 0, 0],
            pre_approach_pose_mmdeg=[500, 0, 0, 0, 0, 0],
        )
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_PRE_APPROACH_DONE)
        node._mark_stage_progress_after_execution(FINAL_HOVER_STAGE)
        self.assertEqual(node.stage_progress, STAGE_PROGRESS_FINAL_HOVER_DONE)

    def test_stage_progress_waits_until_pre_approach_position_is_reached(self) -> None:
        node = self._node()

        node._mark_stage_progress_after_execution(APPROACH_READY_STAGE)
        node._mark_stage_progress_after_execution(
            PRE_APPROACH_STAGE,
            candidate_pose_mmdeg=[80, 0, 0, 0, 0, 0],
            pre_approach_pose_mmdeg=[500, 0, 0, 0, 0, 0],
        )

        self.assertEqual(node.stage_progress, STAGE_PROGRESS_APPROACH_READY_DONE)

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

    def test_locked_target_age_is_reported_as_warning_not_fatal_timeout_risk(self) -> None:
        node = self._node()
        node.last_target_lock_status = {
            "state": "locked",
            "locked": True,
        }
        flags = node._compute_risk_flags(
            {
                "candidate_stage": FINAL_HOVER_STAGE,
                "target_age_ms": 10_000.0,
                "selected_source_status": "stale",
                "stage_progress": STAGE_PROGRESS_PRE_APPROACH_DONE,
                "approach_ready_configured": True,
            }
        )

        self.assertNotIn("target_age_over_timeout", flags)
        self.assertIn("locked_target_age_over_timeout_warning", flags)
        self.assertIn("source_stale_warning", flags)

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
            min_plane_clearance_mm=0.0,
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
        node.require_locked_target_before_motion = True
        node.continue_with_last_locked_target_on_source_loss = True
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
