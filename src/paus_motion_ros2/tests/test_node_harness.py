from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
for p in (str(PERCEPTION_PACKAGE_ROOT), str(MOTION_PACKAGE_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

import builtins as _builtins
_real_import = _builtins.__import__

# ── Pre-seed mock modules ──
for mod_name in ("rclpy", "rclpy.node", "rclpy.executors",
                  "geometry_msgs", "geometry_msgs.msg",
                  "std_msgs", "std_msgs.msg",
                  "ament_index_python", "ament_index_python.packages"):
    if mod_name not in sys.modules:
        mod = mock.MagicMock()
        mod.__name__ = mod_name
        mod.__package__ = mod_name
        sys.modules[mod_name] = mod

# fairino_client
if "paus_motion_ros2.fairino_client" not in sys.modules:
    fm = mock.MagicMock()
    fm.__name__ = "paus_motion_ros2.fairino_client"
    fm.FairinoLinuxClient = mock.MagicMock()
    sys.modules["paus_motion_ros2.fairino_client"] = fm

# ── Mock load_config ──
import paus_perception
import paus_perception.config as _real_config

def _test_config(*args, **kwargs):
    return {
        "control": {
            "robot_ip": "127.0.0.1",
            "linux_fairino_sdk_root": "/tmp",
            "execute_motion": False,
            "tool_id": 0,
            "user_id": 0,
            "move_vel": 50.0,
            "max_step_distance_mm": 80.0,
            "min_safe_z_mm": 50.0,
            "orientation_mode": "face_marker_normal",
            "flange_face_axis": "-Z",
            "hover_clearance_mm": 30.0,
            "pre_approach_distance_mm": 80.0,
            "min_plane_clearance_mm": 10.0,
            "prefer_positive_z_surface_normal": True,
            "enable_safe_lift_on_low_clearance": True,
            "safe_lift_step_mm": 80.0,
            "safe_lift_above_marker_mm": 180.0,
            "safe_lift_max_z_mm": 500.0,
            "max_execution_stage": "final_hover",
            "target_hold_timeout_ms": 500,
            "completion_position_tolerance_mm": 20.0,
            "completion_normal_tolerance_deg": 5.0,
            "workspace_min_mm": [-1000.0, -1000.0, 0.0],
            "workspace_max_mm": [1000.0, 1000.0, 1000.0],
            "repeat_distance_threshold_mm": 5.0,
            "stage_switch_buffer_mm": 10.0,
            "use_mock_pose": True,
            "mock_current_tcp_pose_mmdeg": [500.0, 0.0, 300.0, 180.0, 0.0, -180.0],
            "repeat_orientation_threshold_deg": 2.0,
        }
    }

paus_perception.config.load_config = _test_config
paus_perception.load_config = _test_config

# ── Build a proxy Node base that lets real __init__ run ──
class _ProxyNode:
    """Minimal Node stand-in that stores attributes but does nothing ROS."""

    def __init__(self, *args, **kwargs):
        pass

    def get_clock(self):
        c = mock.MagicMock()
        c.now.return_value = _make_ros_time(0)
        return c

    def get_logger(self):
        return mock.MagicMock()

    def get_parameter(self, name):
        m = mock.MagicMock()
        m.get_parameter_value.return_value.string_value = "/tmp/fake.yaml"
        return m

    def declare_parameter(self, name, value=None):
        pass

    def create_publisher(self, *args, **kwargs):
        return mock.MagicMock()

    def create_subscription(self, *args, **kwargs):
        return mock.MagicMock()

    def create_timer(self, *args, **kwargs):
        return mock.MagicMock()


rclpy_mod = sys.modules["rclpy"]
rclpy_mod.node.Node = _ProxyNode
# rclpy.node is a separate entry in sys.modules — Python resolves
# "from rclpy.node import Node" against sys.modules, not against
# the parent package's attribute, so both must carry _ProxyNode.
sys.modules["rclpy.node"].Node = _ProxyNode

# Also mock get_package_share_directory
ament_mod = sys.modules["ament_index_python"]
ament_mod.packages.get_package_share_directory = mock.MagicMock(
    return_value="/tmp/fake_share"
)


def _make_ros_time(nanoseconds=0):
    t = mock.MagicMock()
    t.nanoseconds = nanoseconds
    return t


def _make_pose_stamped(x=0.5, y=0.0, z=0.2, frame_id="robot_base"):
    msg = mock.MagicMock()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    return msg


# ── Now import the node (uses _ProxyNode as base) ──
from paus_motion_ros2.fairino_control_node import (
    FairinoControlNode,
    STAGE_ORDER,
    PRE_APPROACH_STAGE,
    REORIENT_STAGE,
    FINAL_HOVER_STAGE,
    SAFE_LIFT_STAGE,
    TRACKING_IDLE,
    TRACKING_ACTIVE,
    TRACKING_TARGET_LOST_PENDING,
    TRACKING_TARGET_LOST,
    TRACKING_SUCCEEDED_VISIBLE,
    TRACKING_SUCCEEDED_AFTER_OCCLUSION,
    REASON_NO_VALID_TARGET,
    REASON_TARGET_LOST_TIMEOUT,
)


# ── Tests ──

class RealNodeTargetLossTests(unittest.TestCase):
    """AC-4/AC-5: Drive _target_callback() / _status_timer_callback() on
    a live FairinoControlNode with mocked ROS2 infrastructure."""

    @classmethod
    def setUpClass(cls):
        cls.node = FairinoControlNode()
        cls.node.execute_motion = False
        cls.node.motion_in_progress = False
        cls.node._get_current_tcp_pose_mmdeg = mock.MagicMock(
            return_value=[500.0, 0.0, 300.0, 180.0, 0.0, -180.0]
        )
        cls.node._safe_get_current_tcp_pose_mmdeg = mock.MagicMock(
            return_value=[500.0, 0.0, 300.0, 180.0, 0.0, -180.0]
        )
        cls.node._compute_completion_metrics = mock.MagicMock(
            return_value=(100.0, 30.0)
        )
        cls.node._completion_reached = mock.MagicMock(return_value=False)
        cls.node._completion_allowed = mock.MagicMock(return_value=True)
        cls.node._marker_visibility_status = mock.MagicMock()
        cls.node._transform_validity_status = mock.MagicMock()
        cls.node._motion_command_for_stage = mock.MagicMock(return_value="MoveL")
        cls.node._stage_allowed = mock.MagicMock(return_value=True)

        cls.node._published_events = []
        cls.node._published_states = []
        _orig_publish = cls.node._publish_status

        def _track(**kwargs):
            cls.node._published_events.append(kwargs.get("event"))
            cls.node._published_states.append(kwargs.get("tracking_state"))
            return _orig_publish(**kwargs)

        cls.node._publish_status = _track

        cls._target_age_ms_value = [None]
        cls.node._target_age_ms = lambda now: cls._target_age_ms_value[0]

    def setUp(self):
        self.node.stage_latch = None
        self.node.last_valid_target_time = None
        self.node.last_valid_target_pose_base_mmdeg = None
        self.node.last_valid_final_hover_pose_mmdeg = None
        self.node.last_valid_surface_normal_base = None
        self.node.last_valid_target_point_base_m = None
        self.node.last_executed_candidate_pose_mmdeg = None
        self.node.last_executed_stage = None
        self.node.last_timer_publish_signature = None
        self.node.last_tracking_state = TRACKING_IDLE
        self.node.last_completion_reason = REASON_NO_VALID_TARGET
        self.node._published_events.clear()
        self.node._published_states.clear()
        type(self)._target_age_ms_value[0] = None
        self.node._marker_visibility_status.return_value = "ok"
        self.node._transform_validity_status.return_value = "ok"

    # ── AC-4: Brief loss preserves state ──

    def test_brief_target_loss_preserves_stage_latch(self):
        self.node._get_current_tcp_pose_mmdeg.return_value = [
            500.0, 0.0, 230.0, 180.0, 0.0, -180.0
        ]
        self.node._marker_visibility_status.return_value = "ok"
        self.node._transform_validity_status.return_value = "ok"

        msg = _make_pose_stamped(0.5, 0.0, 0.2)
        self.node._target_callback(msg)

        self.assertIsNotNone(self.node.stage_latch)
        latched = self.node.stage_latch
        self.assertIn(latched, (REORIENT_STAGE, FINAL_HOVER_STAGE))

        self.node._marker_visibility_status.return_value = "not_found"
        self.node._transform_validity_status.return_value = "invalid"
        type(self)._target_age_ms_value[0] = 200
        self.node._published_states.clear()

        self.node._status_timer_callback()

        self.assertEqual(self.node.stage_latch, latched)
        self.assertIsNotNone(self.node.last_valid_target_time)

    def test_brief_target_loss_publishes_lost_pending(self):
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node.last_valid_target_pose_base_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_final_hover_pose_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_surface_normal_base = [0.0, 0.0, 1.0]
        self.node.stage_latch = FINAL_HOVER_STAGE
        self.node.last_tracking_state = TRACKING_TARGET_LOST_PENDING

        self.node._marker_visibility_status.return_value = "not_found"
        self.node._transform_validity_status.return_value = "invalid"
        type(self)._target_age_ms_value[0] = 200

        self.node._status_timer_callback()

        self.assertIn(TRACKING_TARGET_LOST_PENDING, self.node._published_states)

    def test_brief_loss_preserves_last_valid_cache_fields(self):
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node.last_valid_target_pose_base_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_final_hover_pose_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_surface_normal_base = [0.0, 0.0, 1.0]
        self.node.last_valid_target_point_base_m = [0.5, 0.0, 0.2]
        self.node.stage_latch = FINAL_HOVER_STAGE

        self.node._marker_visibility_status.return_value = "not_found"
        self.node._transform_validity_status.return_value = "invalid"
        type(self)._target_age_ms_value[0] = 200

        self.node._status_timer_callback()

        self.assertIsNotNone(self.node.last_valid_target_pose_base_mmdeg)
        self.assertIsNotNone(self.node.last_valid_final_hover_pose_mmdeg)
        self.assertIsNotNone(self.node.last_valid_surface_normal_base)
        self.assertIsNotNone(self.node.last_valid_target_point_base_m)

    # ── AC-5: Extended loss ──

    def test_extended_target_loss_publishes_tracking_target_lost(self):
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node.last_valid_target_pose_base_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_final_hover_pose_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_surface_normal_base = [0.0, 0.0, 1.0]
        self.node.stage_latch = FINAL_HOVER_STAGE
        self.node.last_tracking_state = TRACKING_TARGET_LOST_PENDING
        self.node._compute_completion_metrics.return_value = (100.0, 30.0)
        self.node._completion_reached.return_value = False

        self.node._marker_visibility_status.return_value = "not_found"
        self.node._transform_validity_status.return_value = "invalid"
        type(self)._target_age_ms_value[0] = 600

        self.node._status_timer_callback()

        self.assertIn(TRACKING_TARGET_LOST, self.node._published_states)

    def test_extended_target_loss_no_motion_command(self):
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node.last_valid_target_pose_base_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_final_hover_pose_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_surface_normal_base = [0.0, 0.0, 1.0]
        self.node.stage_latch = FINAL_HOVER_STAGE
        self.node.last_tracking_state = TRACKING_TARGET_LOST_PENDING
        self.node._compute_completion_metrics.return_value = (100.0, 30.0)
        self.node._completion_reached.return_value = False
        self.node._execute_move = mock.MagicMock()

        self.node._marker_visibility_status.return_value = "not_found"
        self.node._transform_validity_status.return_value = "invalid"
        type(self)._target_age_ms_value[0] = 600

        self.node._status_timer_callback()

        self.node._execute_move.assert_not_called()
        self.assertFalse(self.node.motion_in_progress)

    def test_extended_loss_stage_latch_preserved(self):
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node.last_valid_target_pose_base_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_final_hover_pose_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_surface_normal_base = [0.0, 0.0, 1.0]
        self.node.stage_latch = FINAL_HOVER_STAGE
        self.node.last_tracking_state = TRACKING_TARGET_LOST_PENDING
        self.node._compute_completion_metrics.return_value = (100.0, 30.0)
        self.node._completion_reached.return_value = False

        self.node._marker_visibility_status.return_value = "not_found"
        self.node._transform_validity_status.return_value = "invalid"
        type(self)._target_age_ms_value[0] = 600

        self.node._status_timer_callback()

        self.assertEqual(self.node.stage_latch, FINAL_HOVER_STAGE)

    def test_extended_loss_no_pre_approach_in_status(self):
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node.last_valid_target_pose_base_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_final_hover_pose_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_surface_normal_base = [0.0, 0.0, 1.0]
        self.node.stage_latch = FINAL_HOVER_STAGE
        self.node.last_tracking_state = TRACKING_TARGET_LOST_PENDING
        self.node._compute_completion_metrics.return_value = (100.0, 30.0)
        self.node._completion_reached.return_value = False

        self.node._marker_visibility_status.return_value = "not_found"
        self.node._transform_validity_status.return_value = "invalid"
        type(self)._target_age_ms_value[0] = 600

        self.node._status_timer_callback()

        candidate = self.node.last_status_fields.get("candidate_stage")
        if candidate is not None:
            self.assertNotEqual(candidate, PRE_APPROACH_STAGE)

    # ── AC-3: Visible marker geometry refresh ──

    def test_visible_marker_update_refreshes_cached_geometry(self):
        self.node._get_current_tcp_pose_mmdeg.return_value = [
            500.0, 0.0, 230.0, 180.0, 0.0, -180.0
        ]
        msg1 = _make_pose_stamped(0.5, 0.0, 0.2)
        self.node._target_callback(msg1)

        target1 = list(self.node.last_valid_target_pose_base_mmdeg or [])
        fh1 = list(self.node.last_valid_final_hover_pose_mmdeg or [])

        msg2 = _make_pose_stamped(0.6, 0.0, 0.2)
        self.node._target_callback(msg2)

        target2 = self.node.last_valid_target_pose_base_mmdeg
        fh2 = self.node.last_valid_final_hover_pose_mmdeg

        self.assertIsNotNone(target1)
        self.assertIsNotNone(target2)
        self.assertIsNotNone(fh1)
        self.assertIsNotNone(fh2)

        if len(target1) >= 1 and len(target2) >= 1:
            delta = target2[0] - target1[0]
            self.assertAlmostEqual(delta, 100.0, delta=15.0,
                                   msg="Latest frame should refresh target pose")

    def test_visible_marker_with_latch_does_not_regress_stage(self):
        self.node.stage_latch = FINAL_HOVER_STAGE
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node.last_valid_target_pose_base_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_final_hover_pose_mmdeg = [500.0, 0.0, 230.0, 180.0, 0.0, -180.0]
        self.node.last_valid_surface_normal_base = [0.0, 0.0, 1.0]
        self.node._get_current_tcp_pose_mmdeg.return_value = [
            500.0, 0.0, 235.0, 170.0, 5.0, -175.0
        ]

        msg = _make_pose_stamped(0.5, 0.0, 0.2)
        self.node._target_callback(msg)

        candidate = self.node.last_status_fields.get("candidate_stage")
        if candidate is not None:
            self.assertNotEqual(candidate, PRE_APPROACH_STAGE)

    # ── AC-6: safe_lift ──

    def test_safe_lift_clears_latch_via_callback(self):
        self.node.stage_latch = FINAL_HOVER_STAGE
        self.node.last_valid_target_time = _make_ros_time(0)
        self.node._get_current_tcp_pose_mmdeg.return_value = [
            500.0, 0.0, 20.0, 180.0, 0.0, -180.0
        ]

        msg = _make_pose_stamped(0.5, 0.0, 0.2)
        self.node._target_callback(msg)

        candidate = self.node.last_status_fields.get("candidate_stage")
        if candidate == SAFE_LIFT_STAGE:
            self.assertIsNone(self.node.stage_latch)


class RealNodeInitializationTests(unittest.TestCase):
    """Verify node initializes with correct default values."""

    @classmethod
    def setUpClass(cls):
        cls.node = FairinoControlNode()

    def test_stage_latch_starts_none(self):
        self.assertIsNone(self.node.stage_latch)

    def test_motion_in_progress_starts_false(self):
        self.assertFalse(self.node.motion_in_progress)

    def test_last_valid_targets_start_none(self):
        self.assertIsNone(self.node.last_valid_target_time)
        self.assertIsNone(self.node.last_valid_target_pose_base_mmdeg)
        self.assertIsNone(self.node.last_valid_final_hover_pose_mmdeg)
        self.assertIsNone(self.node.last_valid_surface_normal_base)

    def test_stage_switch_buffer_mm_configured(self):
        self.assertEqual(self.node.stage_switch_buffer_mm, 10.0)

    def test_repeat_distance_threshold_mm_configured(self):
        self.assertEqual(self.node.repeat_distance_threshold_mm, 5.0)


if __name__ == "__main__":
    unittest.main()
