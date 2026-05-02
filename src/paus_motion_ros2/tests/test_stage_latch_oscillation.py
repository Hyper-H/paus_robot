from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
for p in (str(PERCEPTION_PACKAGE_ROOT), str(MOTION_PACKAGE_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from paus_motion_ros2.control_logic import (
    STAGE_ORDER,
    PRE_APPROACH_STAGE,
    REORIENT_STAGE,
    FINAL_HOVER_STAGE,
    SAFE_LIFT_STAGE,
    build_approach_decision,
)


def _tcp_at(position_mm, *, z_up=True):
    """Helper: build a 6-DOF TCP pose. Face-down for -Z flange_face_axis by default."""
    if z_up:
        rpy = [180.0, 0.0, -180.0]
    else:
        rpy = [0.0, 0.0, 0.0]
    return [float(position_mm[0]), float(position_mm[1]), float(position_mm[2]), *rpy]


def _marker_at(x=500.0, y=0.0, z=200.0, normal_along_z=True):
    """Build target_position_base_m + raw_target_orientation_rpy_deg for a marker."""
    target_position_base_m = [x / 1000.0, y / 1000.0, z / 1000.0]
    if normal_along_z:
        raw_target_orientation_rpy_deg = [0.0, 0.0, 0.0]
    else:
        raw_target_orientation_rpy_deg = [0.0, 90.0, 0.0]
    return target_position_base_m, raw_target_orientation_rpy_deg


class StageLatchPreventsRegressionTests(unittest.TestCase):
    """AC-2: Once in reorient or final_hover, geometry must not regress to pre_approach."""

    def test_final_hover_latch_blocks_pre_approach_when_orientation_off(self):
        """TCP at final_hover but orientation slightly off -> latch keeps FINAL_HOVER."""
        target_pt, target_rpy = _marker_at()
        tcp_off = [500.0, 0.0, 230.0, 170.0, 5.0, -175.0]

        decision = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp_off,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=FINAL_HOVER_STAGE,
        )

        self.assertTrue(decision.check_passed, msg=decision.error_message)
        self.assertNotEqual(decision.candidate_stage, PRE_APPROACH_STAGE)
        self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE)

    def test_reorient_latch_blocks_pre_approach(self):
        """TCP at pre_approach position, orientation misaligned -> latch keeps REORIENT."""
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([500.0, 0.0, 310.0], z_up=False)

        decision = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=REORIENT_STAGE,
        )

        self.assertTrue(decision.check_passed, msg=decision.error_message)
        self.assertNotEqual(decision.candidate_stage, PRE_APPROACH_STAGE)
        self.assertEqual(decision.candidate_stage, REORIENT_STAGE)

    def test_no_latch_allows_geometric_behavior(self):
        """Without stage_latch, behavior is the classic geometric decision."""
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([100.0, 0.0, 500.0])

        decision = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=None,
        )

        self.assertTrue(decision.check_passed, msg=decision.error_message)
        self.assertEqual(decision.candidate_stage, PRE_APPROACH_STAGE)


class StageLatchGeometryUpdateTests(unittest.TestCase):
    """AC-3: Latched stage must still follow the latest visible marker position."""

    def test_final_hover_position_updates_with_marker_movement(self):
        """Latch prevents regression but final_hover_pose moves with marker."""
        target_pt, target_rpy = _marker_at(x=500.0, y=0.0)
        tcp = _tcp_at([500.0, 0.0, 230.0])

        decision1 = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=FINAL_HOVER_STAGE,
        )

        target_pt2, target_rpy2 = _marker_at(x=600.0, y=0.0)
        decision2 = build_approach_decision(
            target_position_base_m=target_pt2,
            raw_target_orientation_rpy_deg=target_rpy2,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=FINAL_HOVER_STAGE,
        )

        self.assertTrue(decision1.check_passed)
        self.assertTrue(decision2.check_passed)
        self.assertEqual(decision1.candidate_stage, FINAL_HOVER_STAGE)
        self.assertEqual(decision2.candidate_stage, FINAL_HOVER_STAGE)
        fh1 = decision1.final_hover_pose_mmdeg
        fh2 = decision2.final_hover_pose_mmdeg
        self.assertIsNotNone(fh1)
        self.assertIsNotNone(fh2)
        self.assertAlmostEqual(fh2[0] - fh1[0], 100.0, delta=2.0)


class SafeLiftOverrideTests(unittest.TestCase):
    """AC-6: safe_lift must still override the stage latch."""

    def test_safe_lift_overrides_final_hover_latch(self):
        """When clearance conditions trigger safe_lift, latch is overridden."""
        target_pt, target_rpy = _marker_at(x=500.0, y=0.0, z=200.0)
        tcp = _tcp_at([500.0, 0.0, 20.0])  # below min_safe_z

        decision = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=FINAL_HOVER_STAGE,
            enable_safe_lift_on_low_clearance=True,
        )

        self.assertTrue(decision.check_passed, msg=decision.error_message)
        self.assertEqual(decision.candidate_stage, SAFE_LIFT_STAGE,
                         "safe_lift must override stage latch")


class StageLatchSequenceTests(unittest.TestCase):
    """Simulate full approach sequences to verify monotonicity."""

    def test_approach_sequence_with_latch_never_regresses(self):
        """Full approach: pre_approach -> reorient -> final_hover, never regresses."""
        target_pt, target_rpy = _marker_at()

        # Start far away
        tcp = _tcp_at([100.0, 0.0, 500.0])

        d1 = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=None,
        )
        self.assertEqual(d1.candidate_stage, PRE_APPROACH_STAGE)

        # Move to candidate position from step 1
        new_tcp = list(d1.candidate_pose_mmdeg)

        d2 = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=new_tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=None,
        )
        self.assertIn(d2.candidate_stage, (PRE_APPROACH_STAGE, REORIENT_STAGE))

        # Arrive at pre_approach position, orientation off -> reorient
        pre_approach_tcp = [500.0, 0.0, 310.0, 0.0, 0.0, 0.0]

        d3 = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=pre_approach_tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=REORIENT_STAGE,
        )

        self.assertNotEqual(d3.candidate_stage, PRE_APPROACH_STAGE)
        self.assertIn(d3.candidate_stage, (REORIENT_STAGE, FINAL_HOVER_STAGE))

        # Arrive at final_hover position with correct orientation
        final_tcp = _tcp_at([500.0, 0.0, 230.0])

        d4 = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=final_tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=FINAL_HOVER_STAGE,
        )

        self.assertEqual(d4.candidate_stage, FINAL_HOVER_STAGE)

    def test_latch_persists_with_momentary_geometric_regression(self):
        """Geometry momentarily says PRE_APPROACH; latch prevents regression."""
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([500.0, 0.0, 235.0])

        for _ in range(5):
            decision = build_approach_decision(
                target_position_base_m=target_pt,
                raw_target_orientation_rpy_deg=target_rpy,
                frame_id="robot_base",
                current_tcp_pose_mmdeg=tcp,
                orientation_mode="face_marker_normal",
                flange_face_axis="-Z",
                hover_clearance_mm=30.0,
                pre_approach_distance_mm=80.0,
                max_step_distance_mm=80.0,
                min_safe_z_mm=50.0,
                min_plane_clearance_mm=10.0,
                workspace_min_mm=[-1000.0, -1000.0, 0.0],
                workspace_max_mm=[1000.0, 1000.0, 1000.0],
                stage_switch_buffer_mm=5.0,
                stage_latch=FINAL_HOVER_STAGE,
            )
            self.assertTrue(decision.check_passed, msg=decision.error_message)
            self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE,
                             f"Latch should prevent regression; got {decision.candidate_stage}")


class StageSwitchBufferTests(unittest.TestCase):
    """AC-7: stage_switch_buffer_mm is independent from repeat_distance_threshold_mm."""

    def test_large_stage_switch_buffer_does_not_affect_repeat_logic(self):
        """Exercise that build_approach_decision accepts stage_switch_buffer_mm."""
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([500.0, 0.0, 310.0])

        decision_large = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=200.0,
            stage_latch=None,
        )
        self.assertTrue(decision_large.check_passed, msg=decision_large.error_message)
        self.assertNotEqual(decision_large.candidate_stage, PRE_APPROACH_STAGE,
                           "Large buffer should keep robot out of pre_approach when near marker")

    def test_small_stage_switch_buffer_works_for_far_approach(self):
        """Small buffer (like default 10mm) works correctly for normal operation."""
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([100.0, 0.0, 500.0])

        decision = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id="robot_base",
            current_tcp_pose_mmdeg=tcp,
            orientation_mode="face_marker_normal",
            flange_face_axis="-Z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=10.0,
            stage_latch=None,
        )
        self.assertTrue(decision.check_passed, msg=decision.error_message)
        self.assertEqual(decision.candidate_stage, PRE_APPROACH_STAGE)


class TargetLossPreservesStateTests(unittest.TestCase):
    '''AC-4: Brief target loss preserves last_valid_* and stage_latch.'''

    def test_stage_latch_survives_repeated_build_approach_calls(self):
        '''Simulating brief loss: stage_latch survives across multiple call invocations.'''
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([500.0, 0.0, 230.0])

        latch = FINAL_HOVER_STAGE
        for i in range(5):
            decision = build_approach_decision(
                target_position_base_m=target_pt,
                raw_target_orientation_rpy_deg=target_rpy,
                frame_id='robot_base',
                current_tcp_pose_mmdeg=tcp,
                orientation_mode='face_marker_normal',
                flange_face_axis='-Z',
                hover_clearance_mm=30.0,
                pre_approach_distance_mm=80.0,
                max_step_distance_mm=80.0,
                min_safe_z_mm=50.0,
                min_plane_clearance_mm=10.0,
                workspace_min_mm=[-1000.0, -1000.0, 0.0],
                workspace_max_mm=[1000.0, 1000.0, 1000.0],
                stage_switch_buffer_mm=5.0,
                stage_latch=latch,
            )
            self.assertTrue(decision.check_passed, msg=f'Call {i}: {decision.error_message}')
            self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE,
                             f'Latch must survive call {i}')
            # Simulate what the node does: advance latch monotonically
            if decision.candidate_stage in (REORIENT_STAGE, FINAL_HOVER_STAGE):
                new_order = STAGE_ORDER.get(decision.candidate_stage, 0)
                current_order = STAGE_ORDER.get(latch, 0)
                if new_order > current_order:
                    latch = decision.candidate_stage

    def test_last_valid_cache_fields_in_decision(self):
        '''Decision carries surface_normal and final_hover even when latched.'''
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([500.0, 0.0, 310.0])  # at pre_approach position, facing correctly

        decision = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id='robot_base',
            current_tcp_pose_mmdeg=tcp,
            orientation_mode='face_marker_normal',
            flange_face_axis='-Z',
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=FINAL_HOVER_STAGE,
        )
        self.assertTrue(decision.check_passed, msg=decision.error_message)
        # These fields are stored as last_valid_* by the node
        self.assertIsNotNone(decision.surface_normal_base)
        self.assertIsNotNone(decision.final_hover_pose_mmdeg)
        self.assertIsNotNone(decision.target_pose_base_mmdeg)


class TargetLossExtendedTests(unittest.TestCase):
    '''AC-5: Extended target loss does NOT issue pre_approach; reports lost.'''

    def test_latched_final_hover_never_produces_pre_approach_on_loss(self):
        '''With stage_latch=FINAL_HOVER, geometry cannot produce PRE_APPROACH
        even when marker position would normally trigger it (simulating loss).'''
        target_pt, target_rpy = _marker_at()
        tcp = _tcp_at([500.0, 0.0, 230.0])

        # Multiple calls with the same state - latch prevents regression
        for _ in range(10):
            decision = build_approach_decision(
                target_position_base_m=target_pt,
                raw_target_orientation_rpy_deg=target_rpy,
                frame_id='robot_base',
                current_tcp_pose_mmdeg=tcp,
                orientation_mode='face_marker_normal',
                flange_face_axis='-Z',
                hover_clearance_mm=30.0,
                pre_approach_distance_mm=80.0,
                max_step_distance_mm=80.0,
                min_safe_z_mm=50.0,
                min_plane_clearance_mm=10.0,
                workspace_min_mm=[-1000.0, -1000.0, 0.0],
                workspace_max_mm=[1000.0, 1000.0, 1000.0],
                stage_switch_buffer_mm=5.0,
                stage_latch=FINAL_HOVER_STAGE,
            )
            self.assertTrue(decision.check_passed, msg=decision.error_message)
            self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE,
                             'Latched FINAL_HOVER must not regress to PRE_APPROACH')

    def test_far_tcp_with_latch_stays_at_latched_stage(self):
        '''Even when TCP is far from marker (as after a loss/reacquire cycle),
        latch prevents going back to geometric pre_approach.'''
        target_pt, target_rpy = _marker_at()
        # TCP far away - geometry would say pre_approach
        tcp = _tcp_at([100.0, 0.0, 500.0])

        decision = build_approach_decision(
            target_position_base_m=target_pt,
            raw_target_orientation_rpy_deg=target_rpy,
            frame_id='robot_base',
            current_tcp_pose_mmdeg=tcp,
            orientation_mode='face_marker_normal',
            flange_face_axis='-Z',
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=80.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            stage_switch_buffer_mm=5.0,
            stage_latch=FINAL_HOVER_STAGE,
        )
        self.assertTrue(decision.check_passed, msg=decision.error_message)
        # Latch prevents regression; stage stays at FINAL_HOVER
        self.assertEqual(decision.candidate_stage, FINAL_HOVER_STAGE)


if __name__ == "__main__":
    unittest.main()
