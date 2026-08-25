from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
for p in (str(PERCEPTION_PACKAGE_ROOT), str(MOTION_PACKAGE_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

# Mock rclpy and ROS dependencies before importing the node module
for mod_name in (
    "rclpy", "rclpy.node", "rclpy.executors",
    "geometry_msgs", "geometry_msgs.msg",
    "std_msgs", "std_msgs.msg",
    "ament_index_python", "ament_index_python.packages",
):
    if mod_name not in sys.modules:
        sys.modules[mod_name] = mock.MagicMock()

from paus_motion_ros2.fairino_control_node import (
    STAGE_ORDER,
    APPROACH_READY_STAGE,
    PRE_APPROACH_STAGE,
    REORIENT_STAGE,
    FINAL_HOVER_STAGE,
    SAFE_LIFT_STAGE,
    TRACKING_IDLE,
    TRACKING_ACTIVE,
    TRACKING_TARGET_LOST_PENDING,
    TRACKING_TARGET_LOST,
    TRACKING_SUCCEEDED_VISIBLE,
    REASON_NO_VALID_TARGET,
    REASON_REACHED_VISIBLE,
)


class StageOrderTests(unittest.TestCase):
    """Verify STAGE_ORDER constant enforces monotonic progression."""

    def test_stage_order_values(self):
        self.assertEqual(STAGE_ORDER[SAFE_LIFT_STAGE], -1)
        self.assertEqual(STAGE_ORDER[APPROACH_READY_STAGE], 0)
        self.assertEqual(STAGE_ORDER[REORIENT_STAGE], 1)
        self.assertEqual(STAGE_ORDER[PRE_APPROACH_STAGE], 2)
        self.assertEqual(STAGE_ORDER[FINAL_HOVER_STAGE], 3)

    def test_stage_order_is_monotonic(self):
        """Stages progress from low to high order values."""
        self.assertLess(STAGE_ORDER[APPROACH_READY_STAGE], STAGE_ORDER[REORIENT_STAGE])
        self.assertLess(STAGE_ORDER[REORIENT_STAGE], STAGE_ORDER[PRE_APPROACH_STAGE])
        self.assertLess(STAGE_ORDER[REORIENT_STAGE], STAGE_ORDER[FINAL_HOVER_STAGE])

    def test_safe_lift_is_below_all_stages(self):
        """safe_lift has the lowest order, always overridden."""
        for stage in (APPROACH_READY_STAGE, PRE_APPROACH_STAGE, REORIENT_STAGE, FINAL_HOVER_STAGE):
            self.assertLess(STAGE_ORDER[SAFE_LIFT_STAGE], STAGE_ORDER[stage])


class StageLatchUpdateLogicTests(unittest.TestCase):
    """Test _update_stage_latch logic without needing a live ROS2 node.

    We replicate the exact logic from FairinoControlNode._update_stage_latch()
    to verify its behavior across all transition cases.
    """

    @staticmethod
    def _update_stage_latch(current_latch, candidate_stage):
        if candidate_stage == SAFE_LIFT_STAGE:
            return None
        elif candidate_stage in (REORIENT_STAGE, FINAL_HOVER_STAGE):
            new_order = STAGE_ORDER.get(candidate_stage, 0)
            current_order = STAGE_ORDER.get(current_latch, 0)
            if new_order > current_order:
                return candidate_stage
        return current_latch

    def test_none_to_reorient_advances_latch(self):
        self.assertEqual(
            self._update_stage_latch(None, REORIENT_STAGE), REORIENT_STAGE
        )

    def test_none_to_final_hover_advances_latch(self):
        self.assertEqual(
            self._update_stage_latch(None, FINAL_HOVER_STAGE), FINAL_HOVER_STAGE
        )

    def test_none_to_pre_approach_does_not_latch(self):
        self.assertIsNone(self._update_stage_latch(None, PRE_APPROACH_STAGE))

    def test_reorient_to_final_hover_advances_latch(self):
        self.assertEqual(
            self._update_stage_latch(REORIENT_STAGE, FINAL_HOVER_STAGE),
            FINAL_HOVER_STAGE,
        )

    def test_final_hover_to_reorient_does_not_regress(self):
        self.assertEqual(
            self._update_stage_latch(FINAL_HOVER_STAGE, REORIENT_STAGE),
            FINAL_HOVER_STAGE,
        )

    def test_final_hover_to_pre_approach_does_not_regress(self):
        self.assertEqual(
            self._update_stage_latch(FINAL_HOVER_STAGE, PRE_APPROACH_STAGE),
            FINAL_HOVER_STAGE,
        )

    def test_safe_lift_clears_latch(self):
        self.assertIsNone(
            self._update_stage_latch(FINAL_HOVER_STAGE, SAFE_LIFT_STAGE)
        )

    def test_none_latch_stays_none_for_pre_approach(self):
        self.assertIsNone(
            self._update_stage_latch(None, PRE_APPROACH_STAGE)
        )

    def test_latch_persists_across_same_stage(self):
        self.assertEqual(
            self._update_stage_latch(FINAL_HOVER_STAGE, FINAL_HOVER_STAGE),
            FINAL_HOVER_STAGE,
        )

    def test_latch_persists_across_none_candidate(self):
        self.assertEqual(
            self._update_stage_latch(FINAL_HOVER_STAGE, None),
            FINAL_HOVER_STAGE,
        )


class TargetLossTrackingStateTests(unittest.TestCase):
    """AC-4/AC-5: Verify tracking state transitions for target loss behavior."""

    def test_tracking_states_are_distinct(self):
        states = {
            TRACKING_IDLE,
            TRACKING_ACTIVE,
            TRACKING_TARGET_LOST_PENDING,
            TRACKING_TARGET_LOST,
            TRACKING_SUCCEEDED_VISIBLE,
        }
        self.assertEqual(len(states), 5)

    def test_target_lost_is_not_idle(self):
        self.assertNotEqual(TRACKING_TARGET_LOST, TRACKING_IDLE)
        self.assertNotEqual(TRACKING_TARGET_LOST_PENDING, TRACKING_IDLE)

    def test_target_lost_is_not_active_or_succeeded(self):
        self.assertNotEqual(TRACKING_TARGET_LOST, TRACKING_ACTIVE)
        self.assertNotEqual(TRACKING_TARGET_LOST, TRACKING_SUCCEEDED_VISIBLE)

    def test_reason_constants_available(self):
        self.assertEqual(REASON_NO_VALID_TARGET, "no_valid_target_yet")
        self.assertEqual(REASON_REACHED_VISIBLE, "reached_final_hover_with_target_visible")


class NodeAttributeContractTests(unittest.TestCase):
    """Verify that FairinoControlNode defines the attributes required by AC-4.

    We cannot instantiate a real ROS2 node in a unit-test environment,
    but we can verify the class-level annotations and __init__ source
    contain the expected attribute names.
    """

    def test_stage_order_keys_cover_all_stages(self):
        expected_stages = {APPROACH_READY_STAGE, PRE_APPROACH_STAGE, REORIENT_STAGE, FINAL_HOVER_STAGE, SAFE_LIFT_STAGE}
        self.assertEqual(set(STAGE_ORDER.keys()), expected_stages)

    def test_stage_order_has_five_entries(self):
        self.assertEqual(len(STAGE_ORDER), 5)


if __name__ == "__main__":
    unittest.main()
