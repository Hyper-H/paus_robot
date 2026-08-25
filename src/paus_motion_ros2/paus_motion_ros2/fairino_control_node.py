from __future__ import annotations

import json
from pathlib import Path
import time
import traceback
from typing import Any

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from paus_motion_ros2 import FairinoLinuxClient, build_approach_decision, compute_normal_alignment_error_deg
from paus_motion_ros2.control_logic import _quaternion_slerp, _rotation_delta_deg
from paus_perception import (
    load_config,
    quaternion_xyzw_to_rotation_matrix,
    rotation_matrix_to_quaternion_xyzw,
    rotation_matrix_to_rpy_deg,
    rpy_deg_to_rotation_matrix,
)


APPROACH_READY_STAGE = "approach_ready"
PRE_APPROACH_STAGE = "pre_approach"
REORIENT_STAGE = "reorient"
FINAL_HOVER_STAGE = "final_hover"
SAFE_LIFT_STAGE = "safe_lift"
MOTION_STRATEGY_LEGACY = "legacy_direct_final_hover"
MOTION_STRATEGY_STAGED_PATIENT_LEFT_FINAL_HOVER = "staged_patient_left_final_hover"
ROLL_POLICY_CURRENT_TCP_PROJECTION = "current_tcp_projection"
ROLL_POLICY_BASE_UP_PROJECTION = "base_up_projection"
STAGE_PROGRESS_NONE = "none"
STAGE_PROGRESS_APPROACH_READY_DONE = "approach_ready_done"
STAGE_PROGRESS_PRE_APPROACH_DONE = "pre_approach_done"
STAGE_PROGRESS_FINAL_HOVER_DONE = "final_hover_done"

STAGE_ORDER = {
    SAFE_LIFT_STAGE: -1,
    APPROACH_READY_STAGE: 0,
    REORIENT_STAGE: 1,
    PRE_APPROACH_STAGE: 2,
    FINAL_HOVER_STAGE: 3,
}

TRACKING_IDLE = "idle_no_target"
TRACKING_ACTIVE = "tracking_active"
TRACKING_WAITING_NEXT_FRAME = "waiting_next_frame"
TRACKING_TARGET_LOST_PENDING = "target_lost_pending"
TRACKING_SUCCEEDED_VISIBLE = "tracking_succeeded_visible"
TRACKING_SUCCEEDED_AFTER_OCCLUSION = "tracking_succeeded_after_occlusion"
TRACKING_TARGET_LOST = "tracking_target_lost"
TRACKING_STAGE_GATED = "stage_gated"

REASON_REACHED_VISIBLE = "reached_final_hover_with_target_visible"
REASON_REACHED_AFTER_OCCLUSION = "reached_last_valid_final_hover_after_occlusion"
REASON_TARGET_LOST_TIMEOUT = "target_lost_timeout_before_reach"
REASON_MAX_STAGE_GATED = "max_execution_stage_gated"
REASON_REPEAT_THRESHOLD_HOLD = "repeat_threshold_hold"
REASON_NO_VALID_TARGET = "no_valid_target_yet"
TARGETING_MARKERLESS_NECK = "markerless_neck"
TARGETING_MARKER = "marker"
TARGET_VALIDITY_SELECTED_TARGET = "selected_target"
TARGET_VALIDITY_LOCKED_TARGET = "locked_target"
TARGET_VALIDITY_MARKER_VISIBILITY = "marker_visibility"
MARKER_ROLE_EVAL_ONLY = "eval_only"
MARKER_ROLE_CONTROL_GATE = "control_gate"
DEFAULT_LOCKED_TARGET_POSE_TOPIC = "/locked_target_pose_base"

STATUS_TIMER_PERIOD_S = 0.1


class FairinoControlNode(Node):
    def __init__(self) -> None:
        super().__init__("fairino_control_node")

        bringup_share = get_package_share_directory("paus_bringup")
        self.repeat_orientation_threshold_deg = 3.0

        self.declare_parameter("config_path", f"{bringup_share}/configs/default.yaml")
        self.declare_parameter("target_pose_topic", "/target_pose_base")
        self.declare_parameter("control_status_topic", "/control_status")
        self.declare_parameter("control_debug_status_topic", "/control_debug")
        self.declare_parameter("detection_status_topic", "/detection_status")
        self.declare_parameter("transform_status_topic", "/transform_status")
        self.declare_parameter("selector_status_topic", "/target_selector_status")
        self.declare_parameter("target_lock_status_topic", "/target_lock_status")
        self.declare_parameter("targeting_mode", "")
        self.declare_parameter("run_dir", "")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("control_status_topic").get_parameter_value().string_value
        debug_status_topic = self.get_parameter("control_debug_status_topic").get_parameter_value().string_value
        detection_status_topic = self.get_parameter("detection_status_topic").get_parameter_value().string_value
        transform_status_topic = self.get_parameter("transform_status_topic").get_parameter_value().string_value
        selector_status_topic = self.get_parameter("selector_status_topic").get_parameter_value().string_value
        target_lock_status_topic = self.get_parameter("target_lock_status_topic").get_parameter_value().string_value
        targeting_mode_param = self.get_parameter("targeting_mode").get_parameter_value().string_value.strip()
        run_dir_param = self.get_parameter("run_dir").get_parameter_value().string_value.strip()
        self.run_dir = Path(run_dir_param) if run_dir_param else None
        self.control_trace_path = self.run_dir / "control_trace.jsonl" if self.run_dir is not None else None
        self.control_status_latest_path = self.run_dir / "control_status_latest.json" if self.run_dir is not None else None
        self.motion_summary_path = self.run_dir / "motion_summary.json" if self.run_dir is not None else None
        self.run_report_path = self.run_dir / "run_report.md" if self.run_dir is not None else None

        self.config = load_config(config_path)
        control_cfg = self.config["control"]
        targeting_cfg = self.config.get("targeting", {})
        configured_targeting_mode = str(targeting_cfg.get("mode", TARGETING_MARKERLESS_NECK))
        self.targeting_mode = targeting_mode_param or configured_targeting_mode
        self.target_pose_topic = target_pose_topic
        self.declare_parameter("execute_motion", bool(control_cfg["execute_motion"]))
        self.declare_parameter("max_execution_stage", str(control_cfg.get("max_execution_stage", FINAL_HOVER_STAGE)))
        self.declare_parameter("approach_ready_vel", float(control_cfg.get("approach_ready_vel", control_cfg["move_vel"])))
        self.declare_parameter("pre_approach_vel", float(control_cfg.get("pre_approach_vel", control_cfg["move_vel"])))
        self.declare_parameter("final_hover_vel", float(control_cfg.get("final_hover_vel", control_cfg["move_vel"])))
        self.declare_parameter("final_hover_servo_cmd_t_s", float(control_cfg.get("final_hover_servo_cmd_t_s", 0.008)))
        self.declare_parameter("final_hover_servo_max_step_mm", float(control_cfg.get("final_hover_servo_max_step_mm", 1.0)))
        self.declare_parameter("final_hover_servo_max_step_deg", float(control_cfg.get("final_hover_servo_max_step_deg", 1.0)))

        self.robot_ip = str(control_cfg["robot_ip"])
        self.linux_fairino_sdk_root = str(control_cfg["linux_fairino_sdk_root"])
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)
        self.tool_id = int(control_cfg["tool_id"])
        self.user_id = int(control_cfg["user_id"])
        self.move_vel = float(control_cfg["move_vel"])
        self.move_acc = float(control_cfg.get("move_acc", 10.0))
        self.move_ovl = float(control_cfg.get("move_ovl", 20.0))
        self.joint_motion_blend_time_ms = float(control_cfg.get("joint_motion_blend_time_ms", 0.0))
        self.joint_motion_done_timeout_s = float(control_cfg.get("joint_motion_done_timeout_s", 60.0))
        self.motion_strategy = str(control_cfg.get("motion_strategy", MOTION_STRATEGY_LEGACY))
        self.move_to_approach_ready_on_start = bool(control_cfg.get("move_to_approach_ready_on_start", False))
        self.approach_ready_joint_deg = self._parse_optional_joint_deg(control_cfg.get("approach_ready_joint_deg"))
        self.approach_ready_tolerance_deg = float(control_cfg.get("approach_ready_tolerance_deg", 5.0))
        self.approach_ready_vel = float(self.get_parameter("approach_ready_vel").get_parameter_value().double_value)
        self.pre_approach_vel = float(self.get_parameter("pre_approach_vel").get_parameter_value().double_value)
        self.final_hover_vel = float(self.get_parameter("final_hover_vel").get_parameter_value().double_value)
        self.final_hover_servo_enabled = bool(control_cfg.get("final_hover_servo_enabled", True))
        self.final_hover_servo_cmd_t_s = float(self.get_parameter("final_hover_servo_cmd_t_s").get_parameter_value().double_value)
        self.final_hover_servo_max_step_mm = float(
            self.get_parameter("final_hover_servo_max_step_mm").get_parameter_value().double_value
        )
        self.final_hover_servo_max_step_deg = float(
            self.get_parameter("final_hover_servo_max_step_deg").get_parameter_value().double_value
        )
        self.final_hover_servo_ready_timeout_s = float(control_cfg.get("final_hover_servo_ready_timeout_s", 3.0))
        self.final_hover_servo_orientation_enabled = bool(control_cfg.get("final_hover_servo_orientation_enabled", False))
        self.max_direct_final_hover_distance_mm = float(control_cfg.get("max_direct_final_hover_distance_mm", 120.0))
        self.roll_policy = str(control_cfg.get("roll_policy", ROLL_POLICY_CURRENT_TCP_PROJECTION))
        self.roll_offset_deg = float(control_cfg.get("roll_offset_deg", 0.0))
        self.max_step_distance_mm = float(control_cfg["max_step_distance_mm"])
        self.min_safe_z_mm = float(control_cfg["min_safe_z_mm"])
        self.orientation_mode = str(control_cfg.get("orientation_mode", "face_marker_normal"))
        self.flange_face_axis = str(control_cfg.get("flange_face_axis", "-Z"))
        self.hover_clearance_mm = float(control_cfg["hover_clearance_mm"])
        self.pre_approach_distance_mm = float(control_cfg["pre_approach_distance_mm"])
        self.min_plane_clearance_mm = float(control_cfg["min_plane_clearance_mm"])
        self.prefer_positive_z_surface_normal = bool(control_cfg.get("prefer_positive_z_surface_normal", True))
        self.enable_safe_lift_on_low_clearance = bool(control_cfg.get("enable_safe_lift_on_low_clearance", True))
        self.safe_lift_step_mm = float(control_cfg.get("safe_lift_step_mm", 80.0))
        self.safe_lift_above_marker_mm = float(control_cfg.get("safe_lift_above_marker_mm", 180.0))
        self.safe_lift_max_z_mm = float(control_cfg.get("safe_lift_max_z_mm", 500.0))
        self.max_execution_stage = str(self.get_parameter("max_execution_stage").get_parameter_value().string_value)
        if self.max_execution_stage not in STAGE_ORDER:
            self.get_logger().warning(
                json.dumps(
                    {
                        "event": "invalid_max_execution_stage",
                        "configured_value": self.max_execution_stage,
                        "fallback": FINAL_HOVER_STAGE,
                    },
                    ensure_ascii=False,
                )
            )
            self.max_execution_stage = FINAL_HOVER_STAGE

        self.target_hold_timeout_ms = int(control_cfg.get("target_hold_timeout_ms", 500))
        self.require_locked_target_before_motion = bool(control_cfg.get("require_locked_target_before_motion", True))
        self.continue_with_last_locked_target_on_source_loss = bool(
            control_cfg.get("continue_with_last_locked_target_on_source_loss", True)
        )
        self.completion_position_tolerance_mm = float(control_cfg.get("completion_position_tolerance_mm", 20.0))
        self.completion_normal_tolerance_deg = float(control_cfg.get("completion_normal_tolerance_deg", 5.0))
        self.workspace_min_mm = [float(value) for value in control_cfg["workspace_min_mm"]]
        self.workspace_max_mm = [float(value) for value in control_cfg["workspace_max_mm"]]
        self.repeat_distance_threshold_mm = float(control_cfg["repeat_distance_threshold_mm"])
        self.stage_switch_buffer_mm = float(control_cfg.get("stage_switch_buffer_mm", self.repeat_distance_threshold_mm))
        self.max_marker_displacement_before_relatch_mm = float(control_cfg.get("max_marker_displacement_before_relatch_mm", 200.0))
        self.debug_reset_after_success = bool(control_cfg.get("debug_reset_after_success", False))
        self.min_consecutive_detections = int(control_cfg.get("min_consecutive_detections", 2))
        self.use_mock_pose = bool(control_cfg["use_mock_pose"])
        self.mock_current_tcp_pose_mmdeg = [float(value) for value in control_cfg["mock_current_tcp_pose_mmdeg"]]

        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.debug_status_publisher = self.create_publisher(String, debug_status_topic, 10)
        self.target_subscription = self.create_subscription(PoseStamped, target_pose_topic, self._target_callback, 10)
        self.detection_status_subscription = self.create_subscription(String, detection_status_topic, self._detection_status_callback, 10)
        self.transform_status_subscription = self.create_subscription(String, transform_status_topic, self._transform_status_callback, 10)
        self.selector_status_subscription = self.create_subscription(String, selector_status_topic, self._selector_status_callback, 10)
        self.target_lock_status_subscription = self.create_subscription(String, target_lock_status_topic, self._target_lock_status_callback, 10)
        self.status_timer = self.create_timer(STATUS_TIMER_PERIOD_S, self._guarded_status_timer_callback)

        self.last_executed_candidate_pose_mmdeg: list[float] | None = None
        self.last_executed_stage: str | None = None
        self.stage_latch: str | None = None
        self.stage_progress: str = STAGE_PROGRESS_NONE
        self.stage_regression_blocked = False
        self.startup_approach_ready_attempted = False
        self.startup_approach_ready_executed = False
        self.startup_approach_ready_error: str | None = None
        self.previous_stage: str | None = None
        self.stage_transition_reason: str | None = None
        self.motion_in_progress = False
        self.motion_execution_faulted = False
        self.control_backend = "mock_pose"
        self.linux_client: FairinoLinuxClient | None = None

        self.last_valid_target_pose_base_mmdeg: list[float] | None = None
        self.last_valid_final_hover_pose_mmdeg: list[float] | None = None
        self.last_valid_surface_normal_base: list[float] | None = None
        self.last_valid_target_point_base_m: list[float] | None = None
        self.last_valid_target_time = None
        self._consecutive_valid_count: int = 0
        self._last_cold_start_target_m: list[float] | None = None
        self.last_detection_status: dict[str, Any] | None = None
        self.last_transform_status: dict[str, Any] | None = None
        self.last_selector_status: dict[str, Any] | None = None
        self.last_target_lock_status: dict[str, Any] | None = None
        self.last_tracking_state = TRACKING_IDLE
        self.last_completion_reason = REASON_NO_VALID_TARGET
        self.last_timer_publish_signature: tuple[Any, ...] | None = None
        self.last_trace_signature: tuple[Any, ...] | None = None
        self.last_trace_write_monotonic: float | None = None
        self.latest_risk_flags: list[str] = []
        self.sdk_motion_prepared = False
        self.tracking_episode_completed = False
        self.completed_tracking_state: str | None = None
        self.completed_target_point_base_m: list[float] | None = None
        self.completed_final_hover_pose_mmdeg: list[float] | None = None
        self.last_status_fields: dict[str, Any] = {
            "target_point_base_m": None,
            "target_point_base_mm": None,
            "target_pose_base_mmdeg": None,
            "raw_target_pose_base_mmdeg": None,
            "current_tcp_pose_mmdeg": None,
            "surface_normal_base": None,
            "final_hover_pose_mmdeg": None,
            "pre_approach_pose_mmdeg": None,
            "candidate_pose_mmdeg": None,
            "candidate_stage": None,
            "motion_command": None,
            "step_distance_mm": None,
            "clearance_to_plane_mm": None,
        }

        if not self.use_mock_pose:
            try:
                self.linux_client = FairinoLinuxClient(self.linux_fairino_sdk_root, self.robot_ip)
                self.linux_client.connect()
                self.control_backend = "linux_sdk"
            except Exception as exc:
                self.control_backend = "mock_pose"
                self.get_logger().warning(
                    json.dumps(
                        {
                            "event": "linux_sdk_connect_failed",
                            "robot_ip": self.robot_ip,
                            "sdk_root": self.linux_fairino_sdk_root,
                            "error": repr(exc),
                        },
                        ensure_ascii=False,
                    )
                )

        self._publish_status(
            event="fairino_control_node_started",
            error_message="",
            check_passed=False,
            tracking_state=TRACKING_IDLE,
            completion_reason=REASON_NO_VALID_TARGET,
        )

    @staticmethod
    def _parse_optional_joint_deg(value: Any) -> list[float] | None:
        if value is None:
            return None
        if not isinstance(value, (list, tuple)):
            return None
        if len(value) != 6:
            return None
        try:
            joints = [float(item) for item in value]
        except (TypeError, ValueError):
            return None
        if not all(np.isfinite(joints)):
            return None
        return joints

    def _uses_staged_motion(self) -> bool:
        return getattr(self, "motion_strategy", MOTION_STRATEGY_LEGACY) == MOTION_STRATEGY_STAGED_PATIENT_LEFT_FINAL_HOVER

    def _uses_approach_ready_stage(self) -> bool:
        return self._uses_staged_motion() and bool(getattr(self, "move_to_approach_ready_on_start", False))

    def _stage_sequence_planned(self) -> list[str]:
        if self._uses_approach_ready_stage():
            return [APPROACH_READY_STAGE, REORIENT_STAGE, PRE_APPROACH_STAGE, FINAL_HOVER_STAGE]
        if self._uses_staged_motion():
            return [REORIENT_STAGE, PRE_APPROACH_STAGE, FINAL_HOVER_STAGE]
        return [REORIENT_STAGE, PRE_APPROACH_STAGE, FINAL_HOVER_STAGE]

    def _approach_ready_configured(self) -> bool:
        return getattr(self, "approach_ready_joint_deg", None) is not None

    def _safe_get_current_joint_deg(self) -> list[float] | None:
        if self.control_backend != "linux_sdk" or self.linux_client is None:
            return getattr(self, "approach_ready_joint_deg", None)
        try:
            error, joints = self.linux_client.get_actual_joint_pos_degree()
        except Exception as exc:
            self.get_logger().warning(
                json.dumps({"event": "current_joint_read_failed", "error": repr(exc)}, ensure_ascii=False)
            )
            return None
        if error != 0:
            self.get_logger().warning(
                json.dumps({"event": "current_joint_read_failed", "error_code": error}, ensure_ascii=False)
            )
            return None
        return joints

    def _approach_ready_joint_delta_deg(self, current_joint_deg: list[float] | None) -> list[float] | None:
        approach_ready_joint_deg = getattr(self, "approach_ready_joint_deg", None)
        if current_joint_deg is None or approach_ready_joint_deg is None:
            return None
        if len(current_joint_deg) != 6:
            return None
        return [float(target) - float(current) for target, current in zip(approach_ready_joint_deg, current_joint_deg)]

    def _approach_ready_reached(self, current_joint_deg: list[float] | None) -> bool:
        delta = self._approach_ready_joint_delta_deg(current_joint_deg)
        if delta is None:
            return False
        return max(abs(value) for value in delta) <= getattr(self, "approach_ready_tolerance_deg", 5.0)

    def _live_to_locked_drift_mm(self) -> float | None:
        if self.last_target_lock_status is None:
            return None
        value = self.last_target_lock_status.get("latest_live_to_locked_mm")
        if isinstance(value, (int, float)):
            return float(value)
        return None

    def _target_lock_drift_warning_mm(self) -> float | None:
        if self.last_target_lock_status is None:
            return None
        value = self.last_target_lock_status.get("drift_warning_mm")
        if isinstance(value, (int, float)):
            return float(value)
        return None

    def _target_lock_drift_action(self) -> str | None:
        if self.last_target_lock_status is None:
            return None
        value = self.last_target_lock_status.get("drift_action")
        return str(value) if value is not None else None

    def _publish_status(
        self,
        *,
        event: str,
        error_message: str,
        check_passed: bool,
        target_point_base_m: list[float] | None = None,
        target_point_base_mm: list[float] | None = None,
        target_pose_base_mmdeg: list[float] | None = None,
        raw_target_pose_base_mmdeg: list[float] | None = None,
        current_tcp_pose_mmdeg: list[float] | None = None,
        surface_normal_base: list[float] | None = None,
        final_hover_pose_mmdeg: list[float] | None = None,
        pre_approach_pose_mmdeg: list[float] | None = None,
        candidate_pose_mmdeg: list[float] | None = None,
        candidate_stage: str | None = None,
        motion_command: str | None = None,
        step_distance_mm: float | None = None,
        clearance_to_plane_mm: float | None = None,
        executed: bool = False,
        tracking_state: str | None = None,
        completion_reason: str | None = None,
        target_age_ms: float | None = None,
        target_hold_timeout_ms: int | None = None,
        last_valid_target_pose_base_mmdeg: list[float] | None = None,
        last_valid_final_hover_pose_mmdeg: list[float] | None = None,
        position_error_to_last_valid_final_hover_mm: float | None = None,
        normal_alignment_error_deg: float | None = None,
        marker_visibility_status: str | None = None,
        transform_validity_status: str | None = None,
        selected_source: str | None = None,
        selected_source_status: str | None = None,
        selected_source_age_ms: float | None = None,
        target_lock_state: str | None = None,
        target_lock_locked: bool | None = None,
        target_validity_source: str | None = None,
        marker_visibility_role: str | None = None,
        target_loss_reason: str | None = None,
        control_gate_inputs: dict[str, Any] | None = None,
        control_gate_decision: str | None = None,
        target_displacement_from_completed_mm: float | None = None,
        completed_tracking_state: str | None = None,
        reject_reason: str | None = None,
        previous_stage: str | None = None,
        stage_transition_reason: str | None = None,
        stage_progress: str | None = None,
        stage_regression_blocked: bool | None = None,
        startup_approach_ready_attempted: bool | None = None,
        startup_approach_ready_executed: bool | None = None,
        startup_approach_ready_error: str | None = None,
        approach_ready_joint_deg: list[float] | None = None,
        current_joint_deg: list[float] | None = None,
        approach_ready_joint_delta_deg: list[float] | None = None,
        approach_ready_max_joint_delta_deg: float | None = None,
        approach_ready_reached: bool | None = None,
        current_tcp_to_final_hover_mm: float | None = None,
        current_tcp_to_pre_approach_mm: float | None = None,
        pre_approach_to_final_hover_mm: float | None = None,
        locked_target_pose_base_mmdeg: list[float] | None = None,
        live_to_locked_drift_mm: float | None = None,
        **extra_fields: Any,
    ) -> None:
        if selected_source is None:
            selected_source = self._selected_source()
        if selected_source_status is None:
            selected_source_status = self._selected_source_status()
        if selected_source_age_ms is None:
            selected_source_age_ms = self._selected_source_age_ms()
        if target_lock_state is None:
            target_lock_state = self._target_lock_state()
        if target_lock_locked is None:
            target_lock_locked = self._target_lock_locked()
        if target_validity_source is None:
            target_validity_source = self._target_validity_source()
        if marker_visibility_role is None:
            marker_visibility_role = self._marker_visibility_role()
        if control_gate_decision is None:
            control_gate_decision = "tracking" if check_passed else "not_tracking"
        if control_gate_inputs is None:
            control_gate_inputs = {
                "marker_visibility_status": marker_visibility_status,
                "transform_validity_status": transform_validity_status,
                "selected_source_status": selected_source_status,
                "selected_source_age_ms": selected_source_age_ms,
                "target_lock_state": target_lock_state,
                "target_lock_locked": target_lock_locked,
                "target_hold_timeout_ms": self.target_hold_timeout_ms,
                "require_locked_target_before_motion": self.require_locked_target_before_motion,
                "continue_with_last_locked_target_on_source_loss": self.continue_with_last_locked_target_on_source_loss,
            }
        if approach_ready_joint_deg is None:
            approach_ready_joint_deg = getattr(self, "approach_ready_joint_deg", None)
        if approach_ready_joint_delta_deg is None:
            approach_ready_joint_delta_deg = self._approach_ready_joint_delta_deg(current_joint_deg)
        if approach_ready_max_joint_delta_deg is None and approach_ready_joint_delta_deg is not None:
            approach_ready_max_joint_delta_deg = max(abs(value) for value in approach_ready_joint_delta_deg)
        if approach_ready_reached is None:
            approach_ready_reached = self._approach_ready_reached(current_joint_deg)
        if previous_stage is None:
            previous_stage = getattr(self, "previous_stage", None)
        if stage_transition_reason is None:
            stage_transition_reason = getattr(self, "stage_transition_reason", None)
        if stage_progress is None:
            stage_progress = getattr(self, "stage_progress", STAGE_PROGRESS_NONE)
        if stage_regression_blocked is None:
            stage_regression_blocked = bool(getattr(self, "stage_regression_blocked", False))
        if startup_approach_ready_attempted is None:
            startup_approach_ready_attempted = bool(getattr(self, "startup_approach_ready_attempted", False))
        if startup_approach_ready_executed is None:
            startup_approach_ready_executed = bool(getattr(self, "startup_approach_ready_executed", False))
        if startup_approach_ready_error is None:
            startup_approach_ready_error = getattr(self, "startup_approach_ready_error", None)
        if live_to_locked_drift_mm is None:
            live_to_locked_drift_mm = self._live_to_locked_drift_mm()
        if locked_target_pose_base_mmdeg is None:
            locked_target_pose_base_mmdeg = target_pose_base_mmdeg if self._uses_target_lock_gate() else None
        full_payload = {
            "event": event,
            "error_message": error_message,
            "reject_reason": reject_reason,
            "check_passed": check_passed,
            "execute_motion": self.execute_motion,
            "executed": executed,
            "motion_strategy": getattr(self, "motion_strategy", MOTION_STRATEGY_LEGACY),
            "stage_sequence_planned": self._stage_sequence_planned(),
            "previous_stage": previous_stage,
            "stage_progress": stage_progress,
            "stage_regression_blocked": stage_regression_blocked,
            "full_pre_approach_move_enabled": self._uses_staged_motion(),
            "pre_approach_step_slicing_enabled": not self._uses_staged_motion(),
            "move_to_approach_ready_on_start": getattr(self, "move_to_approach_ready_on_start", False),
            "startup_approach_ready_attempted": startup_approach_ready_attempted,
            "startup_approach_ready_executed": startup_approach_ready_executed,
            "startup_approach_ready_error": startup_approach_ready_error,
            "orientation_mode": self.orientation_mode,
            "flange_face_axis": self.flange_face_axis,
            "roll_policy": getattr(self, "roll_policy", ROLL_POLICY_CURRENT_TCP_PROJECTION),
            "roll_offset_deg": getattr(self, "roll_offset_deg", 0.0),
            "prefer_positive_z_surface_normal": self.prefer_positive_z_surface_normal,
            "enable_safe_lift_on_low_clearance": self.enable_safe_lift_on_low_clearance,
            "safe_lift_step_mm": self.safe_lift_step_mm,
            "safe_lift_above_marker_mm": self.safe_lift_above_marker_mm,
            "safe_lift_max_z_mm": self.safe_lift_max_z_mm,
            "debug_reset_after_success": self.debug_reset_after_success,
            "max_execution_stage": self.max_execution_stage,
            "target_point_base_m": target_point_base_m,
            "target_point_base_mm": target_point_base_mm,
            "target_pose_base_mmdeg": target_pose_base_mmdeg,
            "raw_target_pose_base_mmdeg": raw_target_pose_base_mmdeg,
            "current_tcp_pose_mmdeg": current_tcp_pose_mmdeg,
            "surface_normal_base": surface_normal_base,
            "final_hover_pose_mmdeg": final_hover_pose_mmdeg,
            "pre_approach_pose_mmdeg": pre_approach_pose_mmdeg,
            "candidate_pose_mmdeg": candidate_pose_mmdeg,
            "candidate_stage": candidate_stage,
            "motion_command": motion_command,
            "stage_latch": getattr(self, "stage_latch", None),
            "stage_transition_reason": stage_transition_reason,
            "step_distance_mm": step_distance_mm,
            "clearance_to_plane_mm": clearance_to_plane_mm,
            "approach_ready_configured": self._approach_ready_configured(),
            "approach_ready_joint_deg": approach_ready_joint_deg,
            "current_joint_deg": current_joint_deg,
            "approach_ready_joint_delta_deg": approach_ready_joint_delta_deg,
            "approach_ready_max_joint_delta_deg": approach_ready_max_joint_delta_deg,
            "approach_ready_reached": approach_ready_reached,
            "current_tcp_to_final_hover_mm": current_tcp_to_final_hover_mm,
            "current_tcp_to_pre_approach_mm": current_tcp_to_pre_approach_mm,
            "pre_approach_to_final_hover_mm": pre_approach_to_final_hover_mm,
            "hover_clearance_mm": getattr(self, "hover_clearance_mm", None),
            "pre_approach_distance_mm": getattr(self, "pre_approach_distance_mm", None),
            "max_direct_final_hover_distance_mm": getattr(self, "max_direct_final_hover_distance_mm", None),
            "tracking_state": tracking_state,
            "completion_reason": completion_reason,
            "target_age_ms": target_age_ms,
            "target_hold_timeout_ms": target_hold_timeout_ms,
            "require_locked_target_before_motion": self.require_locked_target_before_motion,
            "continue_with_last_locked_target_on_source_loss": self.continue_with_last_locked_target_on_source_loss,
            "last_valid_target_pose_base_mmdeg": last_valid_target_pose_base_mmdeg,
            "last_valid_final_hover_pose_mmdeg": last_valid_final_hover_pose_mmdeg,
            "position_error_to_last_valid_final_hover_mm": position_error_to_last_valid_final_hover_mm,
            "normal_alignment_error_deg": normal_alignment_error_deg,
            "locked_target_pose_base_mmdeg": locked_target_pose_base_mmdeg,
            "live_to_locked_drift_mm": live_to_locked_drift_mm,
            "drift_warning_mm": self._target_lock_drift_warning_mm(),
            "drift_action": self._target_lock_drift_action(),
            "marker_visibility_status": marker_visibility_status,
            "transform_validity_status": transform_validity_status,
            "targeting_mode": self.targeting_mode,
            "selected_source": selected_source,
            "selected_source_status": selected_source_status,
            "selected_source_age_ms": selected_source_age_ms,
            "target_lock_state": target_lock_state,
            "target_lock_locked": target_lock_locked,
            "target_validity_source": target_validity_source,
            "marker_visibility_role": marker_visibility_role,
            "target_loss_reason": target_loss_reason,
            "control_gate_inputs": control_gate_inputs,
            "control_gate_decision": control_gate_decision,
            "target_displacement_from_completed_mm": target_displacement_from_completed_mm,
            "completed_tracking_state": completed_tracking_state,
            "control_backend": self.control_backend,
        }
        full_payload.update(extra_fields)
        full_payload["stamp_unix_s"] = time.time()
        full_payload["risk_flags"] = self._compute_risk_flags(full_payload)
        summary_payload = {
            "event": event,
            "tracking_state": tracking_state,
            "completion_reason": completion_reason,
            "motion_strategy": getattr(self, "motion_strategy", MOTION_STRATEGY_LEGACY),
            "stage_sequence_planned": self._stage_sequence_planned(),
            "previous_stage": previous_stage,
            "stage_progress": stage_progress,
            "stage_regression_blocked": stage_regression_blocked,
            "full_pre_approach_move_enabled": self._uses_staged_motion(),
            "pre_approach_step_slicing_enabled": not self._uses_staged_motion(),
            "move_to_approach_ready_on_start": getattr(self, "move_to_approach_ready_on_start", False),
            "startup_approach_ready_attempted": startup_approach_ready_attempted,
            "startup_approach_ready_executed": startup_approach_ready_executed,
            "startup_approach_ready_error": startup_approach_ready_error,
            "candidate_stage": candidate_stage,
            "motion_command": motion_command,
            "stage_latch": getattr(self, "stage_latch", None),
            "stage_transition_reason": stage_transition_reason,
            "reject_reason": reject_reason,
            "check_passed": check_passed,
            "executed": executed,
            "marker_visibility_status": marker_visibility_status,
            "transform_validity_status": transform_validity_status,
            "targeting_mode": self.targeting_mode,
            "selected_source": selected_source,
            "selected_source_status": selected_source_status,
            "selected_source_age_ms": selected_source_age_ms,
            "target_lock_state": target_lock_state,
            "target_lock_locked": target_lock_locked,
            "target_validity_source": target_validity_source,
            "marker_visibility_role": marker_visibility_role,
            "target_loss_reason": target_loss_reason,
            "control_gate_decision": control_gate_decision,
            "target_displacement_from_completed_mm": target_displacement_from_completed_mm,
            "completed_tracking_state": completed_tracking_state,
            "target_age_ms": target_age_ms,
            "position_error_to_last_valid_final_hover_mm": position_error_to_last_valid_final_hover_mm,
            "normal_alignment_error_deg": normal_alignment_error_deg,
            "live_to_locked_drift_mm": live_to_locked_drift_mm,
            "control_backend": self.control_backend,
            "risk_flags": full_payload["risk_flags"],
        }

        status_message = String()
        status_message.data = json.dumps(summary_payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)

        debug_message = String()
        debug_message.data = json.dumps(full_payload, ensure_ascii=False)
        self.debug_status_publisher.publish(debug_message)
        self.get_logger().info(debug_message.data)
        self._write_control_log(full_payload)
        if tracking_state is not None:
            self.last_tracking_state = tracking_state
        if completion_reason is not None or tracking_state not in {TRACKING_ACTIVE, TRACKING_WAITING_NEXT_FRAME}:
            self.last_completion_reason = completion_reason

    def _compute_risk_flags(self, payload: dict[str, Any]) -> list[str]:
        flags: list[str] = []
        target_age_ms = payload.get("target_age_ms")
        if (
            not self._uses_target_lock_gate()
            and isinstance(target_age_ms, (int, float))
            and target_age_ms > float(self.target_hold_timeout_ms)
        ):
            flags.append("target_age_over_timeout")
        elif (
            self._uses_target_lock_gate()
            and isinstance(target_age_ms, (int, float))
            and target_age_ms > float(self.target_hold_timeout_ms)
        ):
            flags.append("locked_target_age_over_timeout_warning")

        if self._uses_target_lock_gate() and self._target_lock_locked():
            selected_source_status = payload.get("selected_source_status")
            if selected_source_status not in (None, "ok", "unknown"):
                flags.append("source_stale_warning")

        current_tcp = payload.get("current_tcp_pose_mmdeg")
        target_point_mm = payload.get("target_point_base_mm")
        if isinstance(current_tcp, list) and isinstance(target_point_mm, list) and len(current_tcp) >= 3 and len(target_point_mm) >= 3:
            try:
                distance_mm = float(
                    np.linalg.norm(
                        np.asarray(target_point_mm[:3], dtype=np.float64)
                        - np.asarray(current_tcp[:3], dtype=np.float64)
                    )
                )
                if distance_mm > 250.0:
                    flags.append("target_far_from_tcp")
            except (TypeError, ValueError):
                pass

        target_point_m = payload.get("target_point_base_m")
        if (
            isinstance(target_point_m, list)
            and len(target_point_m) >= 3
            and self.last_valid_target_point_base_m is not None
        ):
            try:
                jump_mm = float(
                    np.linalg.norm(
                        np.asarray(target_point_m[:3], dtype=np.float64)
                        - np.asarray(self.last_valid_target_point_base_m[:3], dtype=np.float64)
                    )
                ) * 1000.0
                if jump_mm > self.max_marker_displacement_before_relatch_mm:
                    flags.append("large_target_jump")
            except (TypeError, ValueError):
                pass

        candidate_stage = payload.get("candidate_stage")
        if candidate_stage is not None and candidate_stage != FINAL_HOVER_STAGE:
            flags.append("candidate_not_final_hover")
        if payload.get("reject_reason") is not None:
            flags.append(str(payload.get("reject_reason")))
        if self._uses_approach_ready_stage() and not payload.get("approach_ready_configured", False):
            flags.append("approach_ready_not_configured")
        stage_progress = str(payload.get("stage_progress") or getattr(self, "stage_progress", STAGE_PROGRESS_NONE))
        if (
            self._uses_approach_ready_stage()
            and candidate_stage == FINAL_HOVER_STAGE
            and stage_progress not in {STAGE_PROGRESS_PRE_APPROACH_DONE, STAGE_PROGRESS_FINAL_HOVER_DONE}
            and not payload.get("approach_ready_reached", False)
        ):
            flags.append("final_hover_before_approach_ready")
        current_to_final = payload.get("current_tcp_to_final_hover_mm")
        if (
            self._uses_staged_motion()
            and candidate_stage == FINAL_HOVER_STAGE
            and isinstance(current_to_final, (int, float))
            and float(current_to_final) > getattr(self, "max_direct_final_hover_distance_mm", 120.0)
        ):
            flags.append("current_tcp_too_far_for_final_hover")

        marker_status = payload.get("marker_visibility_status")
        marker_role = payload.get("marker_visibility_role")
        if marker_status not in (None, "ok", "unknown") and marker_role == MARKER_ROLE_CONTROL_GATE:
            flags.append("marker_not_found")
        elif marker_status not in (None, "ok", "unknown") and marker_role == MARKER_ROLE_EVAL_ONLY:
            flags.append("marker_eval_unavailable")

        transform_status = payload.get("transform_validity_status")
        if transform_status not in (None, "ok", "ready", "unknown"):
            flags.append("transform_not_ok")

        live_drift = payload.get("live_to_locked_drift_mm")
        drift_warning = payload.get("drift_warning_mm")
        if (
            isinstance(live_drift, (int, float))
            and isinstance(drift_warning, (int, float))
            and float(live_drift) > float(drift_warning)
        ):
            flags.append("drift_exceeded_warning_threshold")

        return flags

    def _should_write_control_trace(self, payload: dict[str, Any]) -> bool:
        event = str(payload.get("event") or "")
        if event in {
            "control_executed",
            "control_execute_failed",
            "control_error",
            "tracking_target_lost",
            "control_stage_gated",
        }:
            return True
        if event.startswith("control_") or event == "motion_busy":
            return True
        signature = (
            payload.get("event"),
            payload.get("tracking_state"),
            payload.get("completion_reason"),
            payload.get("candidate_stage"),
            payload.get("motion_command"),
            tuple(payload.get("risk_flags") or []),
        )
        now = time.monotonic()
        if signature != self.last_trace_signature:
            self.last_trace_signature = signature
            self.last_trace_write_monotonic = now
            return True
        if self.last_trace_write_monotonic is None or now - self.last_trace_write_monotonic >= 1.0:
            self.last_trace_write_monotonic = now
            return True
        return False

    def _write_control_log(self, payload: dict[str, Any]) -> None:
        if self.control_trace_path is None or self.control_status_latest_path is None:
            return
        try:
            self.control_trace_path.parent.mkdir(parents=True, exist_ok=True)
            self.control_status_latest_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            if self._should_write_control_trace(payload):
                with self.control_trace_path.open("a", encoding="utf-8") as trace_file:
                    trace_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self.latest_risk_flags = list(payload.get("risk_flags") or [])
            self._write_motion_summary_and_report(payload)
        except Exception as exc:
            self.get_logger().warning(json.dumps({"event": "control_trace_write_failed", "error": repr(exc)}, ensure_ascii=False))

    def _write_motion_summary_and_report(self, payload: dict[str, Any]) -> None:
        if self.motion_summary_path is None or self.run_report_path is None:
            return
        transform_latest = self._read_json_file(self.run_dir / "transform_status_latest.json") if self.run_dir is not None else None
        selector_latest = self._read_json_file(self.run_dir / "selector_status_latest.json") if self.run_dir is not None else None
        target_lock_latest = self._read_json_file(self.run_dir / "target_lock_status_latest.json") if self.run_dir is not None else None
        neck_latest = self._read_json_file(self.run_dir / "status_latest.json") if self.run_dir is not None else None
        summary = {
            "source": "motion_trace_summary",
            "stamp_unix_s": payload.get("stamp_unix_s"),
            "run_dir": str(self.run_dir) if self.run_dir is not None else None,
            "execute_motion": payload.get("execute_motion"),
            "control_backend": payload.get("control_backend"),
            "latest_event": payload.get("event"),
            "tracking_state": payload.get("tracking_state"),
            "completion_reason": payload.get("completion_reason"),
            "motion_strategy": payload.get("motion_strategy"),
            "stage_sequence_planned": payload.get("stage_sequence_planned"),
            "previous_stage": payload.get("previous_stage"),
            "stage_progress": payload.get("stage_progress"),
            "stage_regression_blocked": payload.get("stage_regression_blocked"),
            "full_pre_approach_move_enabled": payload.get("full_pre_approach_move_enabled"),
            "pre_approach_step_slicing_enabled": payload.get("pre_approach_step_slicing_enabled"),
            "move_to_approach_ready_on_start": payload.get("move_to_approach_ready_on_start"),
            "startup_approach_ready_attempted": payload.get("startup_approach_ready_attempted"),
            "startup_approach_ready_executed": payload.get("startup_approach_ready_executed"),
            "startup_approach_ready_error": payload.get("startup_approach_ready_error"),
            "candidate_stage": payload.get("candidate_stage"),
            "motion_command": payload.get("motion_command"),
            "stage_latch": payload.get("stage_latch"),
            "stage_transition_reason": payload.get("stage_transition_reason"),
            "reject_reason": payload.get("reject_reason"),
            "executed": payload.get("executed"),
            "target_point_base_mm": payload.get("target_point_base_mm"),
            "current_tcp_pose_mmdeg": payload.get("current_tcp_pose_mmdeg"),
            "final_hover_pose_mmdeg": payload.get("final_hover_pose_mmdeg"),
            "pre_approach_pose_mmdeg": payload.get("pre_approach_pose_mmdeg"),
            "approach_ready_configured": payload.get("approach_ready_configured"),
            "approach_ready_joint_deg": payload.get("approach_ready_joint_deg"),
            "current_joint_deg": payload.get("current_joint_deg"),
            "approach_ready_joint_delta_deg": payload.get("approach_ready_joint_delta_deg"),
            "approach_ready_max_joint_delta_deg": payload.get("approach_ready_max_joint_delta_deg"),
            "approach_ready_reached": payload.get("approach_ready_reached"),
            "current_tcp_to_final_hover_mm": payload.get("current_tcp_to_final_hover_mm"),
            "current_tcp_to_pre_approach_mm": payload.get("current_tcp_to_pre_approach_mm"),
            "pre_approach_to_final_hover_mm": payload.get("pre_approach_to_final_hover_mm"),
            "hover_clearance_mm": payload.get("hover_clearance_mm"),
            "pre_approach_distance_mm": payload.get("pre_approach_distance_mm"),
            "surface_normal_base": payload.get("surface_normal_base"),
            "roll_policy": payload.get("roll_policy"),
            "roll_offset_deg": payload.get("roll_offset_deg"),
            "target_age_ms": payload.get("target_age_ms"),
            "position_error_to_last_valid_final_hover_mm": payload.get("position_error_to_last_valid_final_hover_mm"),
            "normal_alignment_error_deg": payload.get("normal_alignment_error_deg"),
            "locked_target_pose_base_mmdeg": payload.get("locked_target_pose_base_mmdeg"),
            "live_to_locked_drift_mm": payload.get("live_to_locked_drift_mm"),
            "drift_warning_mm": payload.get("drift_warning_mm"),
            "drift_action": payload.get("drift_action"),
            "risk_flags": payload.get("risk_flags") or [],
            "marker_visibility_status": payload.get("marker_visibility_status"),
            "transform_validity_status": payload.get("transform_validity_status"),
            "targeting_mode": payload.get("targeting_mode"),
            "selected_source": payload.get("selected_source"),
            "selected_source_status": payload.get("selected_source_status"),
            "selected_source_age_ms": payload.get("selected_source_age_ms"),
            "target_lock_state": payload.get("target_lock_state"),
            "target_lock_locked": payload.get("target_lock_locked"),
            "target_validity_source": payload.get("target_validity_source"),
            "marker_visibility_role": payload.get("marker_visibility_role"),
            "target_loss_reason": payload.get("target_loss_reason"),
            "control_gate_inputs": payload.get("control_gate_inputs"),
            "control_gate_decision": payload.get("control_gate_decision"),
            "latest_selector_status": selector_latest,
            "latest_target_lock_status": target_lock_latest,
            "latest_transform_status": transform_latest,
            "latest_neck_surface_status": self._compact_neck_status(neck_latest),
            "files": {
                "control_trace": str(self.control_trace_path),
                "control_status_latest": str(self.control_status_latest_path),
                "selector_trace": str(self.run_dir / "selector_trace.jsonl") if self.run_dir is not None else None,
                "selector_status_latest": str(self.run_dir / "selector_status_latest.json") if self.run_dir is not None else None,
                "target_lock_trace": str(self.run_dir / "target_lock_trace.jsonl") if self.run_dir is not None else None,
                "target_lock_status_latest": str(self.run_dir / "target_lock_status_latest.json") if self.run_dir is not None else None,
                "transform_trace": str(self.run_dir / "transform_trace.jsonl") if self.run_dir is not None else None,
                "transform_status_latest": str(self.run_dir / "transform_status_latest.json") if self.run_dir is not None else None,
                "neck_status_latest": str(self.run_dir / "status_latest.json") if self.run_dir is not None else None,
                "frames_dir": str(self.run_dir / "frames") if self.run_dir is not None else None,
            },
        }
        self.motion_summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.run_report_path.write_text(self._format_run_report(summary), encoding="utf-8")

    @staticmethod
    def _read_json_file(path: Path) -> dict[str, Any] | None:
        try:
            if not path.exists():
                return None
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None
        return data if isinstance(data, dict) else None

    @staticmethod
    def _compact_neck_status(payload: dict[str, Any] | None) -> dict[str, Any] | None:
        if payload is None:
            return None
        keys = (
            "status",
            "reason",
            "message",
            "target_point_camera_m",
            "target_pose_base_m",
            "motion_normal_camera",
            "target_region",
            "lateral_offset_mm",
            "frame_label",
            "run_dir",
            "execute_motion",
            "target_pose_frame",
            "transform_status",
        )
        return {key: payload.get(key) for key in keys if key in payload}

    @staticmethod
    def _format_value(value: Any) -> str:
        if value is None:
            return "null"
        return json.dumps(value, ensure_ascii=False)

    def _format_run_report(self, summary: dict[str, Any]) -> str:
        files = summary.get("files") or {}
        latest_selector = summary.get("latest_selector_status") or {}
        latest_target_lock = summary.get("latest_target_lock_status") or {}
        latest_transform = summary.get("latest_transform_status") or {}
        latest_neck = summary.get("latest_neck_surface_status") or {}
        lines = [
            "# Motion Trace Report",
            "",
            f"- run_dir: `{summary.get('run_dir')}`",
            f"- execute_motion: `{summary.get('execute_motion')}`",
            f"- control_backend: `{summary.get('control_backend')}`",
            f"- latest_event: `{summary.get('latest_event')}`",
            f"- tracking_state: `{summary.get('tracking_state')}`",
            f"- completion_reason: `{summary.get('completion_reason')}`",
            f"- executed: `{summary.get('executed')}`",
            f"- motion_strategy: `{summary.get('motion_strategy')}`",
            f"- stage_sequence_planned: `{self._format_value(summary.get('stage_sequence_planned'))}`",
            f"- previous_stage: `{summary.get('previous_stage')}`",
            f"- stage_progress: `{summary.get('stage_progress')}`",
            f"- stage_regression_blocked: `{summary.get('stage_regression_blocked')}`",
            f"- full_pre_approach_move_enabled: `{summary.get('full_pre_approach_move_enabled')}`",
            f"- pre_approach_step_slicing_enabled: `{summary.get('pre_approach_step_slicing_enabled')}`",
            f"- move_to_approach_ready_on_start: `{summary.get('move_to_approach_ready_on_start')}`",
            f"- startup_approach_ready_attempted: `{summary.get('startup_approach_ready_attempted')}`",
            f"- startup_approach_ready_executed: `{summary.get('startup_approach_ready_executed')}`",
            f"- startup_approach_ready_error: `{summary.get('startup_approach_ready_error')}`",
            f"- candidate_stage: `{summary.get('candidate_stage')}`",
            f"- motion_command: `{summary.get('motion_command')}`",
            f"- stage_latch: `{summary.get('stage_latch')}`",
            f"- stage_transition_reason: `{summary.get('stage_transition_reason')}`",
            f"- reject_reason: `{summary.get('reject_reason')}`",
            f"- targeting_mode: `{summary.get('targeting_mode')}`",
            f"- selected_source: `{summary.get('selected_source')}`",
            f"- selected_source_status: `{summary.get('selected_source_status')}`",
            f"- target_lock_state: `{summary.get('target_lock_state')}`",
            f"- target_lock_locked: `{summary.get('target_lock_locked')}`",
            f"- target_validity_source: `{summary.get('target_validity_source')}`",
            f"- control_gate_decision: `{summary.get('control_gate_decision')}`",
            "",
            "## Latest Geometry",
            "",
            f"- target_point_base_mm: `{self._format_value(summary.get('target_point_base_mm'))}`",
            f"- current_tcp_pose_mmdeg: `{self._format_value(summary.get('current_tcp_pose_mmdeg'))}`",
            f"- final_hover_pose_mmdeg: `{self._format_value(summary.get('final_hover_pose_mmdeg'))}`",
            f"- pre_approach_pose_mmdeg: `{self._format_value(summary.get('pre_approach_pose_mmdeg'))}`",
            f"- current_tcp_to_final_hover_mm: `{summary.get('current_tcp_to_final_hover_mm')}`",
            f"- current_tcp_to_pre_approach_mm: `{summary.get('current_tcp_to_pre_approach_mm')}`",
            f"- pre_approach_to_final_hover_mm: `{summary.get('pre_approach_to_final_hover_mm')}`",
            f"- hover_clearance_mm: `{summary.get('hover_clearance_mm')}`",
            f"- pre_approach_distance_mm: `{summary.get('pre_approach_distance_mm')}`",
            f"- surface_normal_base: `{self._format_value(summary.get('surface_normal_base'))}`",
            f"- roll_policy: `{summary.get('roll_policy')}`",
            f"- roll_offset_deg: `{summary.get('roll_offset_deg')}`",
            f"- target_age_ms: `{summary.get('target_age_ms')}`",
            f"- position_error_to_last_valid_final_hover_mm: `{summary.get('position_error_to_last_valid_final_hover_mm')}`",
            f"- normal_alignment_error_deg: `{summary.get('normal_alignment_error_deg')}`",
            "",
            "## Approach Ready",
            "",
            f"- approach_ready_configured: `{summary.get('approach_ready_configured')}`",
            f"- approach_ready_joint_deg: `{self._format_value(summary.get('approach_ready_joint_deg'))}`",
            f"- current_joint_deg: `{self._format_value(summary.get('current_joint_deg'))}`",
            f"- approach_ready_joint_delta_deg: `{self._format_value(summary.get('approach_ready_joint_delta_deg'))}`",
            f"- approach_ready_max_joint_delta_deg: `{summary.get('approach_ready_max_joint_delta_deg')}`",
            f"- approach_ready_reached: `{summary.get('approach_ready_reached')}`",
            "",
            "## Target Lock",
            "",
            f"- locked_target_pose_base_mmdeg: `{self._format_value(summary.get('locked_target_pose_base_mmdeg'))}`",
            f"- live_to_locked_drift_mm: `{summary.get('live_to_locked_drift_mm')}`",
            f"- drift_warning_mm: `{summary.get('drift_warning_mm')}`",
            f"- drift_action: `{summary.get('drift_action')}`",
            "",
            "## Risk Flags",
            "",
            f"- risk_flags: `{self._format_value(summary.get('risk_flags'))}`",
            f"- marker_visibility_status: `{summary.get('marker_visibility_status')}`",
            f"- marker_visibility_role: `{summary.get('marker_visibility_role')}`",
            f"- transform_validity_status: `{summary.get('transform_validity_status')}`",
            f"- selected_source_age_ms: `{summary.get('selected_source_age_ms')}`",
            f"- target_loss_reason: `{summary.get('target_loss_reason')}`",
            f"- control_gate_inputs: `{self._format_value(summary.get('control_gate_inputs'))}`",
            "",
            "## Latest Upstream Status",
            "",
            f"- selector_status: `{self._format_value(latest_selector)}`",
            f"- target_lock_status: `{self._format_value(latest_target_lock)}`",
            f"- transform_status: `{self._format_value(latest_transform)}`",
            f"- neck_surface_status: `{self._format_value(latest_neck)}`",
            "",
            "## Files",
            "",
        ]
        for label, path in files.items():
            lines.append(f"- {label}: `{path}`")
        lines.append("")
        return "\n".join(lines)

    def _parse_status_message(self, message: String, *, source_name: str) -> dict[str, Any] | None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                json.dumps(
                    {
                        "event": "status_json_parse_failed",
                        "source": source_name,
                        "error": repr(exc),
                        "payload_preview": message.data[:200],
                    },
                    ensure_ascii=False,
                )
            )
            return None
        if not isinstance(payload, dict):
            return None
        return payload

    def _detection_status_callback(self, message: String) -> None:
        payload = self._parse_status_message(message, source_name="detection_status")
        if payload is not None:
            self.last_detection_status = payload

    def _transform_status_callback(self, message: String) -> None:
        payload = self._parse_status_message(message, source_name="transform_status")
        if payload is not None:
            self.last_transform_status = payload

    def _selector_status_callback(self, message: String) -> None:
        payload = self._parse_status_message(message, source_name="target_selector_status")
        if payload is not None:
            self.last_selector_status = payload

    def _target_lock_status_callback(self, message: String) -> None:
        payload = self._parse_status_message(message, source_name="target_lock_status")
        if payload is not None:
            self.last_target_lock_status = payload

    def _get_current_tcp_pose_mmdeg(self) -> list[float]:
        if self.control_backend == "mock_pose" or self.linux_client is None:
            return list(self.mock_current_tcp_pose_mmdeg)
        active_tool_error, active_tool_id = self.linux_client.get_actual_tcp_num()
        if active_tool_error == 0 and int(active_tool_id) != int(self.tool_id):
            return self._get_current_configured_tool_pose_mmdeg()
        error, pose = self.linux_client.get_actual_tcp_pose()
        if error != 0:
            raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
        return pose

    def _get_current_configured_tool_pose_mmdeg(self) -> list[float]:
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        flange_error, flange_pose = self.linux_client.get_actual_tool_flange_pose()
        if flange_error != 0 or len(flange_pose) < 6:
            raise RuntimeError(f"GetActualToolFlangePose failed with code {flange_error}.")
        tool_error, tool_coord = self.linux_client.get_tool_coord_with_id(self.tool_id)
        if tool_error != 0 or len(tool_coord) < 6:
            raise RuntimeError(f"GetToolCoordWithID({self.tool_id}) failed with code {tool_error}.")
        flange_position = np.asarray(flange_pose[:3], dtype=np.float64)
        flange_rotation = rpy_deg_to_rotation_matrix(flange_pose[3:6])
        tool_translation = np.asarray(tool_coord[:3], dtype=np.float64)
        tool_rotation = rpy_deg_to_rotation_matrix(tool_coord[3:6])
        tcp_position = flange_position + flange_rotation @ tool_translation
        tcp_rotation = flange_rotation @ tool_rotation
        return [float(value) for value in tcp_position] + rotation_matrix_to_rpy_deg(tcp_rotation)

    def _pose_mmdeg_to_transform(self, pose_mmdeg: list[float]) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rpy_deg_to_rotation_matrix(pose_mmdeg[3:6])
        transform[:3, 3] = np.asarray(pose_mmdeg[:3], dtype=np.float64)
        return transform

    def _transform_to_pose_mmdeg(self, transform: np.ndarray) -> list[float]:
        return [float(value) for value in transform[:3, 3]] + rotation_matrix_to_rpy_deg(transform[:3, :3])

    def _command_pose_and_tool_for_active_tcp(self, configured_tool_pose_mmdeg: list[float]) -> tuple[list[float], int]:
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        active_tool_error, active_tool_id = self.linux_client.get_actual_tcp_num()
        if active_tool_error != 0 or active_tool_id is None or int(active_tool_id) == int(self.tool_id):
            return list(configured_tool_pose_mmdeg), int(self.tool_id)
        configured_tool_error, configured_tool_coord = self.linux_client.get_tool_coord_with_id(self.tool_id)
        if configured_tool_error != 0 or len(configured_tool_coord) < 6:
            raise RuntimeError(f"GetToolCoordWithID({self.tool_id}) failed with code {configured_tool_error}.")
        active_tool_error, active_tool_coord = self.linux_client.get_tool_coord_with_id(int(active_tool_id))
        if active_tool_error != 0 or len(active_tool_coord) < 6:
            raise RuntimeError(f"GetToolCoordWithID({active_tool_id}) failed with code {active_tool_error}.")
        configured_tool_transform = self._pose_mmdeg_to_transform(configured_tool_coord)
        active_tool_transform = self._pose_mmdeg_to_transform(active_tool_coord)
        configured_target_transform = self._pose_mmdeg_to_transform(configured_tool_pose_mmdeg)
        flange_target_transform = configured_target_transform @ np.linalg.inv(configured_tool_transform)
        active_target_transform = flange_target_transform @ active_tool_transform
        return self._transform_to_pose_mmdeg(active_target_transform), int(active_tool_id)

    def _safe_get_current_tcp_pose_mmdeg(self) -> list[float] | None:
        try:
            return self._get_current_tcp_pose_mmdeg()
        except Exception as exc:
            self.get_logger().warning(
                json.dumps(
                    {
                        "event": "current_tcp_read_failed",
                        "error": repr(exc),
                    },
                    ensure_ascii=False,
                )
            )
            return None

    def _motion_command_for_stage(self, candidate_stage: str | None) -> str | None:
        if candidate_stage == APPROACH_READY_STAGE:
            return "MoveJ"
        if candidate_stage == SAFE_LIFT_STAGE:
            return "MoveL"
        if candidate_stage == PRE_APPROACH_STAGE:
            return "MoveJ"
        if candidate_stage == REORIENT_STAGE:
            return "MoveJ"
        if candidate_stage == FINAL_HOVER_STAGE:
            return "ServoCart" if getattr(self, "final_hover_servo_enabled", True) else "MoveL"
        return None

    def _stage_allowed(self, candidate_stage: str | None) -> bool:
        if candidate_stage is None:
            return False
        return STAGE_ORDER[candidate_stage] <= STAGE_ORDER[self.max_execution_stage]

    def _stage_velocity(self, candidate_stage: str | None) -> float:
        if candidate_stage == APPROACH_READY_STAGE:
            return self.approach_ready_vel
        if candidate_stage in (PRE_APPROACH_STAGE, REORIENT_STAGE):
            return self.pre_approach_vel
        if candidate_stage == FINAL_HOVER_STAGE:
            return self.final_hover_vel
        return self.move_vel

    def _prepare_sdk_motion_if_needed(self) -> None:
        if self.sdk_motion_prepared:
            return
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        result = self.linux_client.prepare_motion()
        failed = {name: code for name, code in result.items() if int(code) != 0}
        if failed:
            raise RuntimeError(f"FAIRINO motion preparation failed: {failed}.")
        self.sdk_motion_prepared = True

    def _release_robot_control(self) -> dict[str, int | str]:
        result: dict[str, int | str] = {}
        if self.control_backend != "linux_sdk" or self.linux_client is None:
            return result
        for name, call in (
            ("ServoMoveEnd", self.linux_client.servo_move_end),
            ("StopMotion", self.linux_client.stop_motion),
            ("ModeManual", self.linux_client.set_manual_mode),
        ):
            try:
                result[name] = int(call())
            except Exception as exc:
                result[name] = repr(exc)
        self.sdk_motion_prepared = False
        return result

    def _wait_for_motion_idle(self, timeout_s: float) -> None:
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        last_state: dict[str, Any] = {}
        while True:
            done_code, done = self.linux_client.get_robot_motion_done()
            queue_code, queue_len = self.linux_client.get_motion_queue_length()
            last_state = {
                "motion_done_code": done_code,
                "motion_done": done,
                "queue_code": queue_code,
                "queue_len": queue_len,
            }
            if done_code == 0 and queue_code == 0 and int(done or 0) == 1 and int(queue_len or 0) == 0:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Timed out waiting for motion idle before ServoCart: {last_state}.")
            time.sleep(0.02)

    def _wait_for_joint_target_reached(self, target_joint_deg: list[float], timeout_s: float) -> None:
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        tolerance_deg = max(float(getattr(self, "approach_ready_tolerance_deg", 5.0)), 0.5)
        last_joint: list[float] | None = None
        last_error: int | None = None
        while True:
            error, joint_deg = self.linux_client.get_actual_joint_pos_degree()
            last_error = int(error)
            if error == 0 and len(joint_deg) == 6:
                last_joint = [float(value) for value in joint_deg]
                max_delta = max(abs(float(target) - float(current)) for target, current in zip(target_joint_deg, last_joint))
                if max_delta <= tolerance_deg:
                    return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"Timed out waiting for joint target: error={last_error}, "
                    f"last_joint={last_joint}, target_joint={target_joint_deg}."
                )
            time.sleep(0.05)

    def _pose_target_error(
        self,
        current_pose_mmdeg: list[float],
        target_pose_mmdeg: list[float],
    ) -> tuple[float | None, float]:
        position_error_mm = self._distance_between_pose_positions_mm(current_pose_mmdeg, target_pose_mmdeg)
        orientation_error_deg = _rotation_delta_deg(
            rpy_deg_to_rotation_matrix(current_pose_mmdeg[3:6]),
            rpy_deg_to_rotation_matrix(target_pose_mmdeg[3:6]),
        )
        return position_error_mm, orientation_error_deg

    def _pose_target_reached(
        self,
        current_pose_mmdeg: list[float],
        target_pose_mmdeg: list[float],
        position_tolerance_mm: float,
        orientation_tolerance_deg: float,
    ) -> tuple[bool, float | None, float]:
        position_error_mm, orientation_error_deg = self._pose_target_error(current_pose_mmdeg, target_pose_mmdeg)
        reached = (
            position_error_mm is not None
            and position_error_mm <= position_tolerance_mm
            and orientation_error_deg <= orientation_tolerance_deg
        )
        return reached, position_error_mm, orientation_error_deg

    def _wait_for_pose_target_reached(self, target_pose_mmdeg: list[float], timeout_s: float) -> None:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        position_tolerance_mm = max(float(getattr(self, "stage_switch_buffer_mm", 10.0)), 3.0)
        orientation_tolerance_deg = max(float(getattr(self, "completion_normal_tolerance_deg", 5.0)), 3.0)
        timeout_grace_s = max(float(getattr(self, "pose_target_timeout_grace_s", 0.5)), 0.0)
        last_pose: list[float] | None = None
        last_position_error_mm: float | None = None
        last_orientation_error_deg: float | None = None
        while True:
            try:
                current_pose = self._get_current_tcp_pose_mmdeg()
            except Exception:
                current_pose = None
            if current_pose is not None and len(current_pose) >= 6:
                last_pose = [float(value) for value in current_pose]
                reached, last_position_error_mm, last_orientation_error_deg = self._pose_target_reached(
                    last_pose,
                    target_pose_mmdeg,
                    position_tolerance_mm,
                    orientation_tolerance_deg,
                )
                if reached:
                    return
            if time.monotonic() >= deadline:
                grace_deadline = time.monotonic() + timeout_grace_s
                while time.monotonic() < grace_deadline:
                    time.sleep(0.05)
                    try:
                        current_pose = self._get_current_tcp_pose_mmdeg()
                    except Exception:
                        current_pose = None
                    if current_pose is None or len(current_pose) < 6:
                        continue
                    last_pose = [float(value) for value in current_pose]
                    reached, last_position_error_mm, last_orientation_error_deg = self._pose_target_reached(
                        last_pose,
                        target_pose_mmdeg,
                        position_tolerance_mm,
                        orientation_tolerance_deg,
                    )
                    if reached:
                        return
                raise RuntimeError(
                    "Timed out waiting for pose target: "
                    f"last_pose={last_pose}, target_pose={target_pose_mmdeg}, "
                    f"position_error_mm={last_position_error_mm}, "
                    f"orientation_error_deg={last_orientation_error_deg}."
                )
            time.sleep(0.05)

    def _execute_move(self, candidate_pose_mmdeg: list[float], candidate_stage: str | None) -> list[float]:
        if self.control_backend != "linux_sdk" or self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        self._prepare_sdk_motion_if_needed()
        vel = self._stage_velocity(candidate_stage)
        self.last_move_j_pose_backoff = None
        executed_pose_mmdeg = list(candidate_pose_mmdeg)
        if candidate_stage == APPROACH_READY_STAGE:
            if self.approach_ready_joint_deg is None:
                raise RuntimeError("approach_ready_not_configured")
            active_tool_error, active_tool_id = self.linux_client.get_actual_tcp_num()
            joint_motion_tool_id = int(active_tool_id) if active_tool_error == 0 and active_tool_id is not None else int(self.tool_id)
            result = self.linux_client.move_j(
                self.approach_ready_joint_deg,
                tool_id=joint_motion_tool_id,
                user_id=self.user_id,
                vel=vel,
                acc=self.move_acc,
                blend_time_ms=self.joint_motion_blend_time_ms,
            )
        elif candidate_stage == PRE_APPROACH_STAGE:
            result, executed_pose_mmdeg = self._move_j_pose_with_reachability_backoff(
                candidate_pose_mmdeg,
                vel=vel,
            )
        elif candidate_stage == REORIENT_STAGE:
            result, executed_pose_mmdeg = self._move_j_pose_with_reachability_backoff(
                candidate_pose_mmdeg,
                vel=vel,
            )
        elif candidate_stage == FINAL_HOVER_STAGE and self.final_hover_servo_enabled:
            self._servo_cart_to_pose(candidate_pose_mmdeg)
            result = 0
        else:
            result = self.linux_client.move_l(candidate_pose_mmdeg, tool_id=self.tool_id, user_id=self.user_id, vel=vel)
        if result != 0:
            raise RuntimeError(f"{self._motion_command_for_stage(candidate_stage)} failed with code {result}.")
        if candidate_stage == APPROACH_READY_STAGE:
            self._wait_for_joint_target_reached(self.approach_ready_joint_deg or [], self.joint_motion_done_timeout_s)
        elif candidate_stage in {PRE_APPROACH_STAGE, REORIENT_STAGE}:
            self._wait_for_pose_target_reached(executed_pose_mmdeg, self.joint_motion_done_timeout_s)
        return executed_pose_mmdeg

    def _move_j_pose_once(self, configured_tool_pose_mmdeg: list[float], joint_pos_ref_deg: list[float], vel: float) -> int:
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        command_pose_mmdeg, command_tool_id = self._command_pose_and_tool_for_active_tcp(configured_tool_pose_mmdeg)
        return self.linux_client.move_j_pose(
            command_pose_mmdeg,
            joint_pos_ref_deg=joint_pos_ref_deg,
            tool_id=command_tool_id,
            user_id=self.user_id,
            vel=vel,
            acc=self.move_acc,
            blend_time_ms=self.joint_motion_blend_time_ms,
        )

    def _move_j_pose_with_reachability_backoff(self, target_pose_mmdeg: list[float], vel: float) -> tuple[int, list[float]]:
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        joint_error, current_joint_pos_deg = self.linux_client.get_actual_joint_pos_degree()
        if joint_error != 0:
            raise RuntimeError(f"GetActualJointPosDegree failed with code {joint_error}.")

        target_pose = [float(value) for value in target_pose_mmdeg]
        result = self._move_j_pose_once(target_pose, current_joint_pos_deg, vel)
        if int(result) != 112:
            return int(result), target_pose

        current_pose = self._get_current_tcp_pose_mmdeg()
        current = np.asarray(current_pose, dtype=np.float64)
        target = np.asarray(target_pose, dtype=np.float64)
        translation_distance_mm = float(np.linalg.norm(target[:3] - current[:3]))
        min_backoff_step_mm = 1.0
        if translation_distance_mm <= min_backoff_step_mm:
            return int(result), target_pose

        attempts: list[dict[str, float | int]] = [
            {
                "factor": 1.0,
                "translation_distance_mm": translation_distance_mm,
                "result": int(result),
            }
        ]
        factor = 0.5
        while translation_distance_mm * factor >= min_backoff_step_mm:
            candidate = (current + factor * (target - current)).tolist()
            result = self._move_j_pose_once(candidate, current_joint_pos_deg, vel)
            attempts.append(
                {
                    "factor": float(factor),
                    "translation_distance_mm": float(translation_distance_mm * factor),
                    "result": int(result),
                }
            )
            if int(result) == 0:
                self.last_move_j_pose_backoff = attempts
                return 0, [float(value) for value in candidate]
            if int(result) != 112:
                self.last_move_j_pose_backoff = attempts
                return int(result), [float(value) for value in candidate]
            factor *= 0.5

        self.last_move_j_pose_backoff = attempts
        return int(result), target_pose

    def _servo_cart_to_pose(self, target_pose_mmdeg: list[float]) -> None:
        if self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        self._wait_for_motion_idle(self.final_hover_servo_ready_timeout_s)
        current_pose_mmdeg = self._get_current_tcp_pose_mmdeg()
        target = np.asarray(target_pose_mmdeg, dtype=np.float64)
        current = np.asarray(current_pose_mmdeg, dtype=np.float64)
        if not self.final_hover_servo_orientation_enabled:
            target[3:6] = current[3:6]
        current_rotation = rpy_deg_to_rotation_matrix(current[3:6])
        target_rotation = rpy_deg_to_rotation_matrix(target[3:6])
        current_quaternion = np.asarray(rotation_matrix_to_quaternion_xyzw(current_rotation), dtype=np.float64)
        target_quaternion = np.asarray(rotation_matrix_to_quaternion_xyzw(target_rotation), dtype=np.float64)
        translation_delta_mm = float(np.linalg.norm(target[:3] - current[:3]))
        orientation_delta_deg = _rotation_delta_deg(current_rotation, target_rotation)
        translation_steps = int(np.ceil(translation_delta_mm / max(self.final_hover_servo_max_step_mm, 0.001)))
        orientation_steps = int(np.ceil(orientation_delta_deg / max(self.final_hover_servo_max_step_deg, 0.001)))
        steps = max(1, translation_steps, orientation_steps)

        start_code = self.linux_client.servo_move_start()
        if start_code != 0:
            raise RuntimeError(f"ServoMoveStart failed with code {start_code}.")
        last_code = start_code
        try:
            previous_rpy_deg = current[3:6].copy()
            for index in range(1, steps + 1):
                alpha = float(index) / float(steps)
                intermediate = (current + alpha * (target - current)).tolist()
                if self.final_hover_servo_orientation_enabled:
                    interpolated_quaternion = _quaternion_slerp(current_quaternion, target_quaternion, alpha)
                    interpolated_rotation = quaternion_xyzw_to_rotation_matrix(interpolated_quaternion.tolist())
                    interpolated_rpy = np.asarray(rotation_matrix_to_rpy_deg(interpolated_rotation), dtype=np.float64)
                    for axis_index in range(3):
                        while interpolated_rpy[axis_index] - previous_rpy_deg[axis_index] > 180.0:
                            interpolated_rpy[axis_index] -= 360.0
                        while interpolated_rpy[axis_index] - previous_rpy_deg[axis_index] < -180.0:
                            interpolated_rpy[axis_index] += 360.0
                    intermediate[3:6] = [float(value) for value in interpolated_rpy]
                    previous_rpy_deg = interpolated_rpy
                command_pose_mmdeg, _command_tool_id = self._command_pose_and_tool_for_active_tcp(intermediate)
                last_code = self.linux_client.servo_cart_tool_delta(
                    command_pose_mmdeg,
                    cmd_t=self.final_hover_servo_cmd_t_s,
                    mode=0,
                )
                if last_code != 0:
                    raise RuntimeError(f"ServoCart final_hover failed with code {last_code} at step {index}/{steps}.")
                time.sleep(max(self.final_hover_servo_cmd_t_s, 0.001))
        finally:
            end_code = self.linux_client.servo_move_end()
            if last_code == 0 and end_code != 0:
                raise RuntimeError(f"ServoMoveEnd failed with code {end_code}.")

    def _should_move_to_approach_ready_on_start(self) -> bool:
        return bool(
            self.execute_motion
            and self._uses_staged_motion()
            and getattr(self, "move_to_approach_ready_on_start", False)
            and not getattr(self, "startup_approach_ready_attempted", False)
            and getattr(self, "stage_progress", STAGE_PROGRESS_NONE) == STAGE_PROGRESS_NONE
        )

    def _run_startup_approach_ready_if_needed(self) -> bool:
        if not self._should_move_to_approach_ready_on_start():
            return False
        self.startup_approach_ready_attempted = True
        self.startup_approach_ready_error = None

        if not self._approach_ready_configured():
            self.startup_approach_ready_error = "approach_ready_not_configured"
            self._publish_status(
                event="startup_approach_ready_rejected",
                error_message=self.startup_approach_ready_error,
                check_passed=False,
                reject_reason=self.startup_approach_ready_error,
                candidate_stage=APPROACH_READY_STAGE,
                motion_command=self._motion_command_for_stage(APPROACH_READY_STAGE),
                tracking_state=TRACKING_IDLE,
                completion_reason=REASON_NO_VALID_TARGET,
                control_gate_decision="startup_rejected",
            )
            return True

        try:
            self.motion_in_progress = True
            current_joint_deg = self._safe_get_current_joint_deg()
            self._execute_move([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], APPROACH_READY_STAGE)
            self._mark_stage_progress_after_execution(APPROACH_READY_STAGE)
            self._update_stage_latch(APPROACH_READY_STAGE)
            self.previous_stage = getattr(self, "last_executed_stage", None)
            self.last_executed_stage = APPROACH_READY_STAGE
            self.startup_approach_ready_executed = True
            self._publish_status(
                event="startup_approach_ready_executed",
                error_message="",
                check_passed=True,
                executed=True,
                candidate_stage=APPROACH_READY_STAGE,
                motion_command=self._motion_command_for_stage(APPROACH_READY_STAGE),
                current_joint_deg=current_joint_deg,
                approach_ready_reached=True,
                tracking_state=TRACKING_IDLE,
                completion_reason=REASON_NO_VALID_TARGET,
                control_gate_decision="waiting_for_target",
            )
        except Exception as exc:
            self.startup_approach_ready_error = repr(exc)
            self._publish_status(
                event="startup_approach_ready_failed",
                error_message=repr(exc),
                check_passed=False,
                reject_reason="startup_approach_ready_failed",
                candidate_stage=APPROACH_READY_STAGE,
                motion_command=self._motion_command_for_stage(APPROACH_READY_STAGE),
                tracking_state=TRACKING_IDLE,
                completion_reason=REASON_NO_VALID_TARGET,
                control_gate_decision="startup_failed",
            )
        finally:
            self.motion_in_progress = False
        return True

    def _target_pose_to_mmdeg(self, message: PoseStamped) -> list[float]:
        rotation_matrix = quaternion_xyzw_to_rotation_matrix(
            [
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ]
        )
        rotation_rpy_deg = rotation_matrix_to_rpy_deg(rotation_matrix)
        return [
            float(message.pose.position.x) * 1000.0,
            float(message.pose.position.y) * 1000.0,
            float(message.pose.position.z) * 1000.0,
            *[float(value) for value in rotation_rpy_deg],
        ]

    def _target_age_ms(self, now) -> float | None:
        if self.last_valid_target_time is None:
            return None
        return float((now - self.last_valid_target_time).nanoseconds) / 1_000_000.0

    def _completion_allowed(self) -> bool:
        return self.max_execution_stage == FINAL_HOVER_STAGE

    def _compute_completion_metrics(self, current_tcp_pose_mmdeg: list[float] | None) -> tuple[float | None, float | None]:
        if (
            current_tcp_pose_mmdeg is None
            or self.last_valid_final_hover_pose_mmdeg is None
            or self.last_valid_surface_normal_base is None
        ):
            return None, None
        position_error_mm = float(
            np.linalg.norm(
                np.asarray(current_tcp_pose_mmdeg[:3], dtype=np.float64)
                - np.asarray(self.last_valid_final_hover_pose_mmdeg[:3], dtype=np.float64)
            )
        )
        normal_alignment_error_deg = compute_normal_alignment_error_deg(
            current_tcp_pose_mmdeg,
            self.last_valid_surface_normal_base,
            self.flange_face_axis,
        )
        return position_error_mm, normal_alignment_error_deg

    def _completion_reached(
        self,
        position_error_mm: float | None,
        normal_alignment_error_deg: float | None,
    ) -> bool:
        if not self._completion_allowed():
            return False
        if position_error_mm is None or normal_alignment_error_deg is None:
            return False
        return (
            position_error_mm <= self.completion_position_tolerance_mm
            and normal_alignment_error_deg <= self.completion_normal_tolerance_deg
        )

    def _final_hover_completion_reached(
        self,
        position_error_mm: float | None,
        normal_alignment_error_deg: float | None,
    ) -> bool:
        if not self._completion_allowed() or position_error_mm is None:
            return False
        if self.final_hover_servo_enabled and not self.final_hover_servo_orientation_enabled:
            return position_error_mm <= self.completion_position_tolerance_mm
        return self._completion_reached(position_error_mm, normal_alignment_error_deg)

    def _marker_visibility_status(self, *, assume_visible: bool = False) -> str:
        if assume_visible:
            return "ok"
        if self.last_detection_status is None:
            return "unknown"
        return str(self.last_detection_status.get("status", "unknown"))

    def _transform_validity_status(self, *, assume_valid: bool = False) -> str:
        if assume_valid:
            return "ok"
        if self.last_transform_status is None:
            return "unknown"
        return str(self.last_transform_status.get("status", "unknown"))

    def _selected_source(self) -> str:
        if self.last_selector_status is not None:
            selected_source = self.last_selector_status.get("selected_source")
            if selected_source is not None:
                return str(selected_source)
        return self.targeting_mode

    def _selected_source_status(self, *, assume_valid: bool = False) -> str:
        if assume_valid:
            return "ok"
        if self.last_selector_status is None:
            return "unknown"
        status = self.last_selector_status.get("source_status", self.last_selector_status.get("status", "unknown"))
        return str(status)

    def _selected_source_age_ms(self) -> float | None:
        if self.last_selector_status is None:
            return None
        value = self.last_selector_status.get("source_age_ms")
        if isinstance(value, (int, float)):
            return float(value)
        return None

    def _uses_marker_visibility_gate(self) -> bool:
        return self.targeting_mode == TARGETING_MARKER

    def _uses_target_lock_gate(self) -> bool:
        return self.targeting_mode == TARGETING_MARKERLESS_NECK and self.target_pose_topic == DEFAULT_LOCKED_TARGET_POSE_TOPIC

    def _target_lock_state(self) -> str:
        if self.last_target_lock_status is None:
            return "unknown"
        return str(self.last_target_lock_status.get("state", "unknown"))

    def _target_lock_locked(self) -> bool:
        if self.last_target_lock_status is None:
            return False
        return bool(self.last_target_lock_status.get("locked", False))

    def _locked_target_timeout_is_warning_only(self) -> bool:
        return (
            self._uses_target_lock_gate()
            and self._target_lock_locked()
            and self.continue_with_last_locked_target_on_source_loss
        )

    def _target_validity_source(self) -> str:
        if self._uses_marker_visibility_gate():
            return TARGET_VALIDITY_MARKER_VISIBILITY
        if self._uses_target_lock_gate():
            return TARGET_VALIDITY_LOCKED_TARGET
        return TARGET_VALIDITY_SELECTED_TARGET

    def _marker_visibility_role(self) -> str:
        return MARKER_ROLE_CONTROL_GATE if self._uses_marker_visibility_gate() else MARKER_ROLE_EVAL_ONLY

    def _source_fresh_for_tracking(self, marker_visibility_status: str, transform_validity_status: str, selected_source_status: str) -> bool:
        if self._uses_marker_visibility_gate():
            return marker_visibility_status == "ok" and transform_validity_status == "ok"
        if self._uses_target_lock_gate():
            return self._target_lock_locked()
        return selected_source_status == "ok"

    def _target_message_allowed_for_motion(self) -> bool:
        if self._uses_target_lock_gate() and self.require_locked_target_before_motion:
            return self._target_lock_locked()
        return True

    def _target_loss_reason(
        self,
        *,
        target_age_ms: float | None,
        marker_visibility_status: str,
        transform_validity_status: str,
        selected_source_status: str,
    ) -> str | None:
        if self._uses_marker_visibility_gate():
            if target_age_ms is not None and target_age_ms >= float(self.target_hold_timeout_ms):
                return "selected_target_timeout"
            if marker_visibility_status != "ok":
                return "marker_not_visible"
            if transform_validity_status != "ok":
                return "transform_not_ok"
        elif self._uses_target_lock_gate():
            if not self._target_lock_locked():
                return "target_lock_not_locked"
            if (
                target_age_ms is not None
                and target_age_ms >= float(self.target_hold_timeout_ms)
                and not self._locked_target_timeout_is_warning_only()
            ):
                return "selected_target_timeout"
        elif selected_source_status not in ("ok", "unknown"):
            if target_age_ms is not None and target_age_ms >= float(self.target_hold_timeout_ms):
                return "selected_target_timeout"
            return "selected_source_not_ok"
        elif target_age_ms is not None and target_age_ms >= float(self.target_hold_timeout_ms):
            return "selected_target_timeout"
        return None

    def _control_gate_fields(
        self,
        *,
        marker_visibility_status: str,
        transform_validity_status: str,
        selected_source_status: str,
        selected_source_age_ms: float | None,
        target_loss_reason: str | None,
        control_gate_decision: str,
    ) -> dict[str, Any]:
        target_lock_state = self._target_lock_state()
        target_lock_locked = self._target_lock_locked()
        return {
            "selected_source": self._selected_source(),
            "selected_source_status": selected_source_status,
            "selected_source_age_ms": selected_source_age_ms,
            "target_lock_state": target_lock_state,
            "target_lock_locked": target_lock_locked,
            "target_validity_source": self._target_validity_source(),
            "marker_visibility_role": self._marker_visibility_role(),
            "target_loss_reason": target_loss_reason,
            "control_gate_inputs": {
                "marker_visibility_status": marker_visibility_status,
                "transform_validity_status": transform_validity_status,
                "selected_source_status": selected_source_status,
                "selected_source_age_ms": selected_source_age_ms,
                "target_lock_state": target_lock_state,
                "target_lock_locked": target_lock_locked,
                "target_hold_timeout_ms": self.target_hold_timeout_ms,
                "require_locked_target_before_motion": self.require_locked_target_before_motion,
                "continue_with_last_locked_target_on_source_loss": self.continue_with_last_locked_target_on_source_loss,
            },
            "control_gate_decision": control_gate_decision,
        }

    def _tracking_fields(
        self,
        *,
        tracking_state: str,
        completion_reason: str | None,
        current_tcp_pose_mmdeg: list[float] | None,
        marker_visibility_status: str,
        transform_validity_status: str,
        selected_source_status: str | None = None,
        target_loss_reason: str | None = None,
        control_gate_decision: str | None = None,
        now=None,
    ) -> dict[str, Any]:
        if now is None:
            now = self.get_clock().now()
        target_age_ms = self._target_age_ms(now)
        position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(current_tcp_pose_mmdeg)
        selected_source_status = selected_source_status or self._selected_source_status()
        selected_source_age_ms = self._selected_source_age_ms()
        if control_gate_decision is None:
            control_gate_decision = "tracking" if tracking_state in {
                TRACKING_ACTIVE,
                TRACKING_WAITING_NEXT_FRAME,
                TRACKING_SUCCEEDED_VISIBLE,
                TRACKING_SUCCEEDED_AFTER_OCCLUSION,
                TRACKING_STAGE_GATED,
            } else "not_tracking"
        return {
            "tracking_state": tracking_state,
            "completion_reason": completion_reason,
            "target_age_ms": target_age_ms,
            "target_hold_timeout_ms": self.target_hold_timeout_ms,
            "last_valid_target_pose_base_mmdeg": self.last_valid_target_pose_base_mmdeg,
            "last_valid_final_hover_pose_mmdeg": self.last_valid_final_hover_pose_mmdeg,
            "position_error_to_last_valid_final_hover_mm": position_error_mm,
            "normal_alignment_error_deg": normal_alignment_error_deg,
            "marker_visibility_status": marker_visibility_status,
            "transform_validity_status": transform_validity_status,
            **self._control_gate_fields(
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                selected_source_age_ms=selected_source_age_ms,
                target_loss_reason=target_loss_reason,
                control_gate_decision=control_gate_decision,
            ),
        }

    def _update_last_valid_target_cache(self, target_point_base_m: list[float], decision, now) -> None:
        self.last_valid_target_point_base_m = [float(value) for value in target_point_base_m]
        self.last_valid_target_pose_base_mmdeg = list(decision.target_pose_base_mmdeg)
        self.last_valid_final_hover_pose_mmdeg = list(decision.final_hover_pose_mmdeg) if decision.final_hover_pose_mmdeg is not None else None
        self.last_valid_surface_normal_base = list(decision.surface_normal_base) if decision.surface_normal_base is not None else None
        self.last_valid_target_time = now
        self.last_status_fields.update(
            {
                "target_point_base_m": [float(value) for value in target_point_base_m],
                "target_point_base_mm": list(decision.target_point_base_mm),
                "target_pose_base_mmdeg": list(decision.target_pose_base_mmdeg),
                "raw_target_pose_base_mmdeg": list(decision.raw_target_pose_base_mmdeg),
                "surface_normal_base": list(decision.surface_normal_base) if decision.surface_normal_base is not None else None,
                "final_hover_pose_mmdeg": list(decision.final_hover_pose_mmdeg) if decision.final_hover_pose_mmdeg is not None else None,
                "pre_approach_pose_mmdeg": list(decision.pre_approach_pose_mmdeg) if decision.pre_approach_pose_mmdeg is not None else None,
            }
        )

    def _mark_tracking_episode_completed(self, tracking_state: str) -> None:
        self.tracking_episode_completed = True
        self.completed_tracking_state = tracking_state
        self.completed_target_point_base_m = (
            list(self.last_valid_target_point_base_m)
            if self.last_valid_target_point_base_m is not None
            else None
        )
        self.completed_final_hover_pose_mmdeg = (
            list(self.last_valid_final_hover_pose_mmdeg)
            if self.last_valid_final_hover_pose_mmdeg is not None
            else None
        )

    def _reset_tracking_episode_for_new_target(self) -> None:
        self.tracking_episode_completed = False
        self.completed_tracking_state = None
        self.completed_target_point_base_m = None
        self.completed_final_hover_pose_mmdeg = None
        self.stage_latch = None
        self.stage_progress = STAGE_PROGRESS_NONE
        self.stage_regression_blocked = False
        self.previous_stage = None
        self.stage_transition_reason = None
        self.last_executed_candidate_pose_mmdeg = None
        self.last_executed_stage = None
        self.last_valid_target_pose_base_mmdeg = None
        self.last_valid_final_hover_pose_mmdeg = None
        self.last_valid_surface_normal_base = None
        self.last_valid_target_point_base_m = None
        self.last_valid_target_time = None
        self._consecutive_valid_count = 0
        self._last_cold_start_target_m = None
        self.last_timer_publish_signature = None

    def _target_displacement_from_completed_mm(self, target_point_base_m: list[float]) -> float | None:
        if self.completed_target_point_base_m is None:
            return None
        return float(
            np.linalg.norm(
                np.asarray(target_point_base_m, dtype=np.float64)
                - np.asarray(self.completed_target_point_base_m, dtype=np.float64)
            )
        ) * 1000.0

    def _update_stage_latch(self, candidate_stage: str | None) -> None:
        if candidate_stage == SAFE_LIFT_STAGE:
            self.stage_latch = None
        elif candidate_stage in (APPROACH_READY_STAGE, PRE_APPROACH_STAGE, REORIENT_STAGE, FINAL_HOVER_STAGE):
            new_order = STAGE_ORDER.get(candidate_stage, 0)
            current_order = STAGE_ORDER.get(self.stage_latch, -1)
            if new_order > current_order:
                self.stage_latch = candidate_stage

    def _mark_stage_progress_after_execution(
        self,
        candidate_stage: str | None,
        *,
        candidate_pose_mmdeg: list[float] | None = None,
        pre_approach_pose_mmdeg: list[float] | None = None,
    ) -> None:
        if not self._uses_staged_motion():
            return
        if candidate_stage == APPROACH_READY_STAGE:
            self.stage_progress = STAGE_PROGRESS_APPROACH_READY_DONE
        elif candidate_stage == PRE_APPROACH_STAGE:
            remaining_mm = self._distance_between_pose_positions_mm(candidate_pose_mmdeg, pre_approach_pose_mmdeg)
            if remaining_mm is not None and remaining_mm <= float(getattr(self, "stage_switch_buffer_mm", 5.0)):
                self.stage_progress = STAGE_PROGRESS_PRE_APPROACH_DONE
        elif candidate_stage == FINAL_HOVER_STAGE:
            self.stage_progress = STAGE_PROGRESS_FINAL_HOVER_DONE

    def _force_staged_candidate(self, decision, *, candidate_stage: str, candidate_pose_mmdeg: list[float]) -> None:
        decision.candidate_stage = candidate_stage
        decision.candidate_pose_mmdeg = list(candidate_pose_mmdeg)
        decision.step_distance_mm = self._distance_between_pose_positions_mm(
            decision.current_tcp_pose_mmdeg,
            candidate_pose_mmdeg,
        )

    def _distance_between_pose_positions_mm(
        self,
        lhs_pose_mmdeg: list[float] | None,
        rhs_pose_mmdeg: list[float] | None,
    ) -> float | None:
        if lhs_pose_mmdeg is None or rhs_pose_mmdeg is None or len(lhs_pose_mmdeg) < 3 or len(rhs_pose_mmdeg) < 3:
            return None
        return float(np.linalg.norm(np.asarray(lhs_pose_mmdeg[:3], dtype=np.float64) - np.asarray(rhs_pose_mmdeg[:3], dtype=np.float64)))

    def _normal_alignment_error_for_decision(self, decision) -> float | None:
        surface_normal_base = getattr(decision, "surface_normal_base", None)
        if surface_normal_base is None:
            return None
        try:
            return compute_normal_alignment_error_deg(
                decision.current_tcp_pose_mmdeg,
                surface_normal_base,
                getattr(self, "flange_face_axis", "-Z"),
            )
        except Exception:
            return None

    def _force_reorient_at_current_pose_candidate(self, decision) -> str | None:
        if decision.final_hover_pose_mmdeg is None:
            return "final_hover_pose_unavailable"
        reorient_pose_mmdeg = list(decision.current_tcp_pose_mmdeg)
        if decision.pre_approach_pose_mmdeg is not None:
            current = np.asarray(decision.current_tcp_pose_mmdeg[:3], dtype=np.float64)
            pre_approach = np.asarray(decision.pre_approach_pose_mmdeg[:3], dtype=np.float64)
            current_to_pre_approach_mm = float(np.linalg.norm(pre_approach - current))
            if current_to_pre_approach_mm > 250.0:
                step_mm = min(150.0, current_to_pre_approach_mm - 180.0)
                if step_mm > 1.0:
                    reorient_position = current + (pre_approach - current) * (step_mm / current_to_pre_approach_mm)
                    reorient_pose_mmdeg[:3] = [float(value) for value in reorient_position]
        reorient_pose_mmdeg[3:6] = list(decision.final_hover_pose_mmdeg[3:6])
        self._force_staged_candidate(
            decision,
            candidate_stage=REORIENT_STAGE,
            candidate_pose_mmdeg=reorient_pose_mmdeg,
        )
        return None

    def _force_reorient_or_final_hover_candidate(self, decision) -> str | None:
        if decision.final_hover_pose_mmdeg is None:
            return "final_hover_pose_unavailable"
        normal_alignment_error_deg = self._normal_alignment_error_for_decision(decision)
        if (
            normal_alignment_error_deg is not None
            and normal_alignment_error_deg > float(getattr(self, "completion_normal_tolerance_deg", 5.0))
        ):
            return "pre_approach_reached_with_misaligned_tcp"
        else:
            self._force_staged_candidate(
                decision,
                candidate_stage=FINAL_HOVER_STAGE,
                candidate_pose_mmdeg=decision.final_hover_pose_mmdeg,
            )
        return None

    def _target_lock_drift_warning_exceeded(self) -> bool:
        drift_warning_mm = self._target_lock_drift_warning_mm()
        live_to_locked_mm = self._live_to_locked_drift_mm()
        return bool(
            isinstance(drift_warning_mm, (int, float))
            and isinstance(live_to_locked_mm, (int, float))
            and live_to_locked_mm > drift_warning_mm
        )

    def _apply_staged_motion_policy(
        self,
        decision,
        *,
        current_joint_deg: list[float] | None,
    ) -> tuple[str | None, bool]:
        if not self._uses_staged_motion():
            return None, False

        self.stage_regression_blocked = False

        if self._uses_approach_ready_stage() and not self._approach_ready_configured():
            if self.execute_motion:
                return "approach_ready_not_configured", False
            return None, False

        changed = False
        original_candidate_stage = decision.candidate_stage
        progress = getattr(self, "stage_progress", STAGE_PROGRESS_NONE)
        current_to_final = self._distance_between_pose_positions_mm(
            decision.current_tcp_pose_mmdeg,
            decision.final_hover_pose_mmdeg,
        )
        current_to_pre_approach = self._distance_between_pose_positions_mm(
            decision.current_tcp_pose_mmdeg,
            decision.pre_approach_pose_mmdeg,
        )
        pre_approach_to_final = self._distance_between_pose_positions_mm(
            decision.pre_approach_pose_mmdeg,
            decision.final_hover_pose_mmdeg,
        )
        normal_alignment_error_deg = self._normal_alignment_error_for_decision(decision)
        normal_misaligned = (
            normal_alignment_error_deg is not None
            and normal_alignment_error_deg > float(getattr(self, "completion_normal_tolerance_deg", 5.0))
        )

        if progress == STAGE_PROGRESS_NONE:
            if self._uses_approach_ready_stage():
                decision.candidate_stage = APPROACH_READY_STAGE
                decision.candidate_pose_mmdeg = list(decision.current_tcp_pose_mmdeg)
                decision.step_distance_mm = None
            else:
                if decision.pre_approach_pose_mmdeg is None:
                    return "pre_approach_pose_unavailable", False
                if normal_misaligned:
                    error_message = self._force_reorient_at_current_pose_candidate(decision)
                    if error_message is not None:
                        return error_message, False
                elif (
                    decision.final_hover_pose_mmdeg is not None
                    and current_to_final is not None
                    and current_to_pre_approach is not None
                    and pre_approach_to_final is not None
                    and current_to_final < current_to_pre_approach
                    and current_to_final <= pre_approach_to_final
                    and current_to_final <= float(getattr(self, "pre_approach_distance_mm", pre_approach_to_final))
                ):
                    self.stage_progress = STAGE_PROGRESS_PRE_APPROACH_DONE
                    error_message = self._force_reorient_or_final_hover_candidate(decision)
                    if error_message is not None:
                        return error_message, False
                else:
                    self._force_staged_candidate(
                        decision,
                        candidate_stage=PRE_APPROACH_STAGE,
                        candidate_pose_mmdeg=decision.pre_approach_pose_mmdeg,
                    )
            changed = True
        elif progress == STAGE_PROGRESS_APPROACH_READY_DONE:
            if decision.pre_approach_pose_mmdeg is None:
                return "pre_approach_pose_unavailable", False
            if normal_misaligned:
                error_message = self._force_reorient_at_current_pose_candidate(decision)
                if error_message is not None:
                    return error_message, False
                changed = True
            if (
                not normal_misaligned
                and
                decision.candidate_stage == PRE_APPROACH_STAGE
                and decision.candidate_pose_mmdeg is not None
            ):
                self._force_staged_candidate(
                    decision,
                    candidate_stage=PRE_APPROACH_STAGE,
                    candidate_pose_mmdeg=decision.pre_approach_pose_mmdeg,
                )
            elif not normal_misaligned:
                self._force_staged_candidate(
                    decision,
                    candidate_stage=PRE_APPROACH_STAGE,
                    candidate_pose_mmdeg=decision.pre_approach_pose_mmdeg,
                )
            changed = True
        elif progress == STAGE_PROGRESS_PRE_APPROACH_DONE:
            error_message = self._force_reorient_or_final_hover_candidate(decision)
            if error_message is not None:
                return error_message, False
            changed = True
        elif progress == STAGE_PROGRESS_FINAL_HOVER_DONE:
            if decision.final_hover_pose_mmdeg is not None:
                self._force_staged_candidate(
                    decision,
                    candidate_stage=FINAL_HOVER_STAGE,
                    candidate_pose_mmdeg=decision.final_hover_pose_mmdeg,
                )
            return None, True

        if original_candidate_stage is not None and original_candidate_stage != decision.candidate_stage:
            original_order = STAGE_ORDER.get(original_candidate_stage, -1)
            selected_order = STAGE_ORDER.get(decision.candidate_stage, -1)
            if original_order < selected_order:
                self.stage_regression_blocked = True

        if (
            self.execute_motion
            and self._uses_staged_motion()
            and not self._uses_approach_ready_stage()
            and decision.candidate_stage == PRE_APPROACH_STAGE
            and current_to_pre_approach is not None
            and current_to_pre_approach > self.max_direct_final_hover_distance_mm
        ):
            return "current_tcp_too_far_for_pre_approach", False
        if (
            self.execute_motion
            and decision.candidate_stage == FINAL_HOVER_STAGE
            and current_to_final is not None
            and current_to_final > self.max_direct_final_hover_distance_mm
        ):
            return "current_tcp_too_far_for_final_hover", False
        return None, changed

    def _status_timer_event_for_state(self, tracking_state: str) -> str:
        return {
            TRACKING_IDLE: "tracking_idle",
            TRACKING_TARGET_LOST_PENDING: "tracking_target_lost_pending",
            TRACKING_SUCCEEDED_VISIBLE: "tracking_succeeded_visible",
            TRACKING_SUCCEEDED_AFTER_OCCLUSION: "tracking_succeeded_after_occlusion",
            TRACKING_TARGET_LOST: "tracking_target_lost",
            TRACKING_STAGE_GATED: "tracking_stage_gated",
            TRACKING_WAITING_NEXT_FRAME: "tracking_waiting_next_frame",
            TRACKING_ACTIVE: "tracking_active",
        }.get(tracking_state, "tracking_status_update")

    def _record_internal_error(self, *, event: str, exc: BaseException) -> None:
        payload = {
            "event": event,
            "error_message": repr(exc),
            "traceback": traceback.format_exc(),
            "execute_motion": getattr(self, "execute_motion", None),
            "motion_strategy": getattr(self, "motion_strategy", None),
            "stage_progress": getattr(self, "stage_progress", None),
            "stage_latch": getattr(self, "stage_latch", None),
            "control_backend": getattr(self, "control_backend", None),
            "stamp_unix_s": time.time(),
        }
        try:
            self.get_logger().error(json.dumps(payload, ensure_ascii=False))
        except Exception:
            pass
        try:
            if self.control_trace_path is not None:
                self.control_trace_path.parent.mkdir(parents=True, exist_ok=True)
                with self.control_trace_path.open("a", encoding="utf-8") as trace_file:
                    trace_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def _guarded_status_timer_callback(self) -> None:
        try:
            self._status_timer_callback()
        except Exception as exc:
            self._record_internal_error(event="control_status_timer_failed", exc=exc)

    def _status_timer_callback(self) -> None:
        if self.motion_in_progress:
            return
        if self._run_startup_approach_ready_if_needed():
            return

        now = self.get_clock().now()
        marker_visibility_status = self._marker_visibility_status()
        transform_validity_status = self._transform_validity_status()
        selected_source_status = self._selected_source_status()
        source_fresh = self._source_fresh_for_tracking(marker_visibility_status, transform_validity_status, selected_source_status)
        target_loss_reason = None
        control_gate_decision = "tracking" if source_fresh else "not_tracking"

        if self.tracking_episode_completed:
            tracking_state = self.completed_tracking_state or self.last_tracking_state
            completion_reason = self.last_completion_reason
            signature = (
                tracking_state,
                completion_reason,
                marker_visibility_status,
                transform_validity_status,
                "completed_hold",
            )
            if signature == self.last_timer_publish_signature:
                return
            self.last_timer_publish_signature = signature
            tracking_fields = self._tracking_fields(
                tracking_state=tracking_state,
                completion_reason=completion_reason,
                current_tcp_pose_mmdeg=self.last_status_fields.get("current_tcp_pose_mmdeg"),
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                control_gate_decision="completed_hold",
                now=now,
            )
            self._publish_status(
                event="tracking_completed_hold",
                error_message="Tracking episode already completed; holding final target.",
                check_passed=True,
                **dict(self.last_status_fields),
                **tracking_fields,
                completed_tracking_state=self.completed_tracking_state,
            )
            return

        if self.last_valid_target_time is None:
            tracking_state = TRACKING_IDLE
            completion_reason = REASON_NO_VALID_TARGET
            current_tcp_pose_mmdeg = None
            control_gate_decision = "no_valid_target"
        elif not self._completion_allowed() and self.last_tracking_state == TRACKING_STAGE_GATED:
            tracking_state = TRACKING_STAGE_GATED
            completion_reason = REASON_MAX_STAGE_GATED
            current_tcp_pose_mmdeg = self._safe_get_current_tcp_pose_mmdeg()
            control_gate_decision = "stage_gated"
        elif source_fresh:
            current_tcp_pose_mmdeg = self._safe_get_current_tcp_pose_mmdeg()
            position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(current_tcp_pose_mmdeg)
            if self._completion_reached(position_error_mm, normal_alignment_error_deg):
                tracking_state = TRACKING_SUCCEEDED_VISIBLE
                completion_reason = REASON_REACHED_VISIBLE
            else:
                return
        else:
            current_tcp_pose_mmdeg = self._safe_get_current_tcp_pose_mmdeg()
            target_age_ms = self._target_age_ms(now)
            target_loss_reason = self._target_loss_reason(
                target_age_ms=target_age_ms,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
            )
            if target_age_ms is None:
                tracking_state = TRACKING_IDLE
                completion_reason = REASON_NO_VALID_TARGET
                control_gate_decision = "no_valid_target"
            elif target_age_ms < float(self.target_hold_timeout_ms):
                tracking_state = TRACKING_TARGET_LOST_PENDING
                completion_reason = None
                control_gate_decision = "hold_last_valid_target"
            else:
                position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(current_tcp_pose_mmdeg)
                if self._completion_allowed() and self._completion_reached(position_error_mm, normal_alignment_error_deg):
                    tracking_state = TRACKING_SUCCEEDED_AFTER_OCCLUSION
                    completion_reason = REASON_REACHED_AFTER_OCCLUSION
                    control_gate_decision = "completed_after_target_loss"
                elif not self._completion_allowed() and self.last_tracking_state == TRACKING_STAGE_GATED:
                    tracking_state = TRACKING_STAGE_GATED
                    completion_reason = REASON_MAX_STAGE_GATED
                    control_gate_decision = "stage_gated"
                else:
                    tracking_state = TRACKING_TARGET_LOST
                    completion_reason = REASON_TARGET_LOST_TIMEOUT
                    control_gate_decision = "target_lost"
        if source_fresh:
            target_loss_reason = None
            control_gate_decision = "tracking"

        signature = (tracking_state, completion_reason, marker_visibility_status, transform_validity_status, selected_source_status)
        if signature == self.last_timer_publish_signature:
            return
        self.last_timer_publish_signature = signature
        if tracking_state in {TRACKING_SUCCEEDED_VISIBLE, TRACKING_SUCCEEDED_AFTER_OCCLUSION}:
            self._mark_tracking_episode_completed(tracking_state)

        tracking_fields = self._tracking_fields(
            tracking_state=tracking_state,
            completion_reason=completion_reason,
            current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
            marker_visibility_status=marker_visibility_status,
            transform_validity_status=transform_validity_status,
            selected_source_status=selected_source_status,
            target_loss_reason=target_loss_reason,
            control_gate_decision=control_gate_decision,
            now=now,
        )
        error_message = ""
        if tracking_state == TRACKING_TARGET_LOST_PENDING:
            error_message = "Waiting for selected target recovery before hold timeout expires."
        elif tracking_state == TRACKING_TARGET_LOST:
            error_message = "Selected target remained unavailable past hold timeout before final hover was reached."
        self._publish_status(
            event=self._status_timer_event_for_state(tracking_state),
            error_message=error_message,
            check_passed=tracking_state in {TRACKING_ACTIVE, TRACKING_WAITING_NEXT_FRAME, TRACKING_SUCCEEDED_VISIBLE, TRACKING_SUCCEEDED_AFTER_OCCLUSION, TRACKING_STAGE_GATED},
            **(dict(self.last_status_fields) | {"current_tcp_pose_mmdeg": current_tcp_pose_mmdeg}),
            **tracking_fields,
        )

    def _target_callback(self, message: PoseStamped) -> None:
        target_point_base_m = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
        raw_target_pose_base_mmdeg = self._target_pose_to_mmdeg(message)
        marker_visibility_status = self._marker_visibility_status(assume_visible=True)
        transform_validity_status = self._transform_validity_status(assume_valid=True)
        selected_source_status = self._selected_source_status(assume_valid=True)
        now = self.get_clock().now()

        if not self._target_message_allowed_for_motion():
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_IDLE,
                completion_reason=REASON_NO_VALID_TARGET,
                current_tcp_pose_mmdeg=self.last_status_fields.get("current_tcp_pose_mmdeg"),
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                target_loss_reason="target_lock_not_locked",
                control_gate_decision="waiting_for_target_lock",
                now=now,
            )
            self._publish_status(
                event="control_waiting_for_target_lock",
                error_message="Locked target pose received before target_lock_locked=true.",
                check_passed=False,
                target_point_base_m=target_point_base_m,
                raw_target_pose_base_mmdeg=raw_target_pose_base_mmdeg,
                **tracking_fields,
            )
            return

        if self.motion_in_progress:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE,
                completion_reason=None,
                current_tcp_pose_mmdeg=self.last_status_fields.get("current_tcp_pose_mmdeg"),
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                now=now,
            )
            self._publish_status(
                event="motion_busy",
                error_message="Motion is already in progress.",
                check_passed=False,
                target_point_base_m=target_point_base_m,
                raw_target_pose_base_mmdeg=raw_target_pose_base_mmdeg,
                **tracking_fields,
            )
            return

        if self.tracking_episode_completed:
            target_displacement_mm = self._target_displacement_from_completed_mm(target_point_base_m)
            should_reset_for_debug = (
                self.debug_reset_after_success
                and target_displacement_mm is not None
                and target_displacement_mm > self.max_marker_displacement_before_relatch_mm
            )
            if should_reset_for_debug:
                self.get_logger().info(
                    json.dumps(
                        {
                            "event": "tracking_episode_reset_after_success",
                            "target_displacement_from_completed_mm": target_displacement_mm,
                            "reset_threshold_mm": self.max_marker_displacement_before_relatch_mm,
                        },
                        ensure_ascii=False,
                    )
                )
                self._reset_tracking_episode_for_new_target()
            else:
                tracking_state = self.completed_tracking_state or self.last_tracking_state
                completion_reason = self.last_completion_reason
                tracking_fields = self._tracking_fields(
                    tracking_state=tracking_state,
                    completion_reason=completion_reason,
                    current_tcp_pose_mmdeg=self.last_status_fields.get("current_tcp_pose_mmdeg"),
                    marker_visibility_status=marker_visibility_status,
                    transform_validity_status=transform_validity_status,
                    selected_source_status=selected_source_status,
                    now=now,
                )
                self._publish_status(
                    event="control_hold_after_success",
                    error_message="Tracking episode already completed; holding final target.",
                    check_passed=True,
                    **dict(self.last_status_fields),
                    **tracking_fields,
                    target_displacement_from_completed_mm=target_displacement_mm,
                    completed_tracking_state=self.completed_tracking_state,
                )
                return

        try:
            current_tcp_pose_mmdeg = self._get_current_tcp_pose_mmdeg()
            current_joint_deg = self._safe_get_current_joint_deg() if self._uses_staged_motion() else None

            if (
                self.stage_latch is not None
                and self.last_valid_target_point_base_m is not None
            ):
                displacement_m = float(
                    np.linalg.norm(
                        np.asarray(target_point_base_m, dtype=np.float64)
                        - np.asarray(self.last_valid_target_point_base_m, dtype=np.float64)
                    )
                )
                if displacement_m * 1000.0 > self.max_marker_displacement_before_relatch_mm:
                    self.stage_latch = None
                    self.stage_progress = STAGE_PROGRESS_NONE
                    self.stage_regression_blocked = False

            decision = build_approach_decision(
                target_position_base_m=target_point_base_m,
                raw_target_orientation_rpy_deg=raw_target_pose_base_mmdeg[3:6],
                frame_id=message.header.frame_id,
                current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
                orientation_mode=self.orientation_mode,
                flange_face_axis=self.flange_face_axis,
                hover_clearance_mm=self.hover_clearance_mm,
                pre_approach_distance_mm=self.pre_approach_distance_mm,
                max_step_distance_mm=self.max_step_distance_mm,
                min_safe_z_mm=self.min_safe_z_mm,
                min_plane_clearance_mm=self.min_plane_clearance_mm,
                workspace_min_mm=self.workspace_min_mm,
                workspace_max_mm=self.workspace_max_mm,
                stage_switch_buffer_mm=self.stage_switch_buffer_mm,
                stage_latch=self.stage_latch,
                prefer_positive_z_surface_normal=self.prefer_positive_z_surface_normal,
                enable_safe_lift_on_low_clearance=self.enable_safe_lift_on_low_clearance,
                safe_lift_step_mm=self.safe_lift_step_mm,
                safe_lift_above_marker_mm=self.safe_lift_above_marker_mm,
                safe_lift_max_z_mm=self.safe_lift_max_z_mm,
                roll_policy=self.roll_policy,
                roll_offset_deg=self.roll_offset_deg,
            )
            staged_reject_reason, staged_changed_candidate = self._apply_staged_motion_policy(
                decision,
                current_joint_deg=current_joint_deg,
            )
            reject_reason = staged_reject_reason
        except Exception as exc:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE if self.last_valid_target_time is not None else TRACKING_IDLE,
                completion_reason=None if self.last_valid_target_time is not None else REASON_NO_VALID_TARGET,
                current_tcp_pose_mmdeg=None,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                now=now,
            )
            self._publish_status(
                event="control_error",
                error_message=repr(exc),
                check_passed=False,
                target_point_base_m=target_point_base_m,
                raw_target_pose_base_mmdeg=raw_target_pose_base_mmdeg,
                **tracking_fields,
            )
            return

        if decision.check_passed:
            self._update_last_valid_target_cache(target_point_base_m, decision, now)

        current_tcp_for_status = list(decision.current_tcp_pose_mmdeg)
        approach_ready_joint_delta_deg = self._approach_ready_joint_delta_deg(current_joint_deg)
        approach_ready_max_joint_delta_deg = (
            max(abs(value) for value in approach_ready_joint_delta_deg)
            if approach_ready_joint_delta_deg is not None
            else None
        )
        current_tcp_to_final_hover_mm = self._distance_between_pose_positions_mm(
            current_tcp_for_status,
            decision.final_hover_pose_mmdeg,
        )
        current_tcp_to_pre_approach_mm = self._distance_between_pose_positions_mm(
            current_tcp_for_status,
            decision.pre_approach_pose_mmdeg,
        )
        pre_approach_to_final_hover_mm = self._distance_between_pose_positions_mm(
            decision.pre_approach_pose_mmdeg,
            decision.final_hover_pose_mmdeg,
        )
        stage_transition_reason = None
        if decision.candidate_stage != self.last_executed_stage:
            stage_transition_reason = f"{self.last_executed_stage or 'none'}->{decision.candidate_stage or 'none'}"
        elif staged_changed_candidate:
            stage_transition_reason = "staged_policy_override"
        self.stage_transition_reason = stage_transition_reason
        common_status = {
            "target_point_base_m": target_point_base_m,
            "target_point_base_mm": decision.target_point_base_mm,
            "target_pose_base_mmdeg": decision.target_pose_base_mmdeg,
            "raw_target_pose_base_mmdeg": decision.raw_target_pose_base_mmdeg,
            "current_tcp_pose_mmdeg": current_tcp_for_status,
            "surface_normal_base": decision.surface_normal_base,
            "final_hover_pose_mmdeg": decision.final_hover_pose_mmdeg,
            "pre_approach_pose_mmdeg": decision.pre_approach_pose_mmdeg,
            "candidate_pose_mmdeg": decision.candidate_pose_mmdeg,
            "candidate_stage": decision.candidate_stage,
            "motion_command": self._motion_command_for_stage(decision.candidate_stage),
            "step_distance_mm": decision.step_distance_mm,
            "clearance_to_plane_mm": decision.clearance_to_plane_mm,
            "current_joint_deg": current_joint_deg,
            "approach_ready_joint_delta_deg": approach_ready_joint_delta_deg,
            "approach_ready_max_joint_delta_deg": approach_ready_max_joint_delta_deg,
            "approach_ready_reached": self._approach_ready_reached(current_joint_deg),
            "current_tcp_to_final_hover_mm": current_tcp_to_final_hover_mm,
            "current_tcp_to_pre_approach_mm": current_tcp_to_pre_approach_mm,
            "pre_approach_to_final_hover_mm": pre_approach_to_final_hover_mm,
            "locked_target_pose_base_mmdeg": decision.target_pose_base_mmdeg if self._uses_target_lock_gate() else None,
            "live_to_locked_drift_mm": self._live_to_locked_drift_mm(),
        }
        self.last_status_fields.update(common_status)

        if reject_reason is not None:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE if self.last_valid_target_time is not None else TRACKING_IDLE,
                completion_reason=None if self.last_valid_target_time is not None else REASON_NO_VALID_TARGET,
                current_tcp_pose_mmdeg=current_tcp_for_status,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                control_gate_decision="rejected",
                target_loss_reason=reject_reason,
                now=now,
            )
            self._publish_status(
                event="control_rejected",
                error_message=reject_reason,
                check_passed=False,
                reject_reason=reject_reason,
                **common_status,
                **tracking_fields,
            )
            return

        if not decision.check_passed:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE if self.last_valid_target_time is not None else TRACKING_IDLE,
                completion_reason=None if self.last_valid_target_time is not None else REASON_NO_VALID_TARGET,
                current_tcp_pose_mmdeg=current_tcp_for_status,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                now=now,
            )
            self._publish_status(
                event="control_rejected",
                error_message=decision.error_message,
                check_passed=False,
                reject_reason=decision.error_message,
                **common_status,
                **tracking_fields,
            )
            return

        if self._consecutive_valid_count < self.min_consecutive_detections:
            was_cold_start = self.last_valid_target_time is None
            target_point_mm = decision.target_point_base_mm[:3]
            if self._last_cold_start_target_m is not None:
                jump_mm = float(
                    np.linalg.norm(
                        np.asarray(target_point_mm, dtype=np.float64)
                        - np.asarray(self._last_cold_start_target_m, dtype=np.float64)
                    )
                )
                if jump_mm <= self.repeat_distance_threshold_mm * 2.0:
                    self._consecutive_valid_count += 1
                else:
                    self._consecutive_valid_count = 1
                    self._last_cold_start_target_m = list(target_point_mm)
            else:
                self._consecutive_valid_count = 1
                self._last_cold_start_target_m = list(target_point_mm)

            if was_cold_start and self._consecutive_valid_count < self.min_consecutive_detections:
                tracking_fields = self._tracking_fields(
                    tracking_state=TRACKING_ACTIVE,
                    completion_reason=None,
                    current_tcp_pose_mmdeg=current_tcp_for_status,
                    marker_visibility_status=marker_visibility_status,
                    transform_validity_status=transform_validity_status,
                    selected_source_status=selected_source_status,
                    now=now,
                )
                self._publish_status(
                    event="control_cold_start_gated",
                    error_message=(
                        f"Cold start gate ({self._consecutive_valid_count}/"
                        f"{self.min_consecutive_detections}) - waiting for "
                        f"consecutive consistent detections."
                    ),
                    check_passed=True,
                    **common_status,
                    **tracking_fields,
                )
                return

        if self.last_executed_candidate_pose_mmdeg is not None and self.last_executed_stage == decision.candidate_stage:
            translation_delta = np.linalg.norm(
                np.asarray(decision.candidate_pose_mmdeg[:3], dtype=np.float64)
                - np.asarray(self.last_executed_candidate_pose_mmdeg[:3], dtype=np.float64)
            )
            orientation_delta = max(
                abs(float(lhs) - float(rhs))
                for lhs, rhs in zip(decision.candidate_pose_mmdeg[3:6], self.last_executed_candidate_pose_mmdeg[3:6])
            )
            if translation_delta < self.repeat_distance_threshold_mm and orientation_delta < self.repeat_orientation_threshold_deg:
                position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(current_tcp_for_status)
                tracking_state = (
                    TRACKING_SUCCEEDED_VISIBLE
                    if self._completion_reached(position_error_mm, normal_alignment_error_deg)
                    else TRACKING_WAITING_NEXT_FRAME
                )
                completion_reason = (
                    REASON_REACHED_VISIBLE
                    if tracking_state == TRACKING_SUCCEEDED_VISIBLE
                    else REASON_REPEAT_THRESHOLD_HOLD
                )
                tracking_fields = self._tracking_fields(
                    tracking_state=tracking_state,
                    completion_reason=completion_reason,
                    current_tcp_pose_mmdeg=current_tcp_for_status,
                    marker_visibility_status=marker_visibility_status,
                    transform_validity_status=transform_validity_status,
                    selected_source_status=selected_source_status,
                    now=now,
                )
                if tracking_state == TRACKING_SUCCEEDED_VISIBLE:
                    self._mark_tracking_episode_completed(tracking_state)
                self._publish_status(
                    event="control_skipped_repeat",
                    error_message="Target change is smaller than repeat threshold.",
                    check_passed=True,
                    **common_status,
                    **tracking_fields,
                )
                return

        if not self.execute_motion:
            position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(current_tcp_for_status)
            tracking_state = (
                TRACKING_SUCCEEDED_VISIBLE
                if self._completion_reached(position_error_mm, normal_alignment_error_deg)
                else TRACKING_ACTIVE
            )
            completion_reason = REASON_REACHED_VISIBLE if tracking_state == TRACKING_SUCCEEDED_VISIBLE else None
            tracking_fields = self._tracking_fields(
                tracking_state=tracking_state,
                completion_reason=completion_reason,
                current_tcp_pose_mmdeg=current_tcp_for_status,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                now=now,
            )
            if tracking_state == TRACKING_SUCCEEDED_VISIBLE:
                self._mark_tracking_episode_completed(tracking_state)
            self._publish_status(
                event="control_dry_run",
                error_message="",
                check_passed=True,
                **common_status,
                **tracking_fields,
            )
            return

        if not self._stage_allowed(decision.candidate_stage):
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_STAGE_GATED,
                completion_reason=REASON_MAX_STAGE_GATED,
                current_tcp_pose_mmdeg=current_tcp_for_status,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                now=now,
            )
            self._publish_status(
                event="control_stage_gated",
                error_message=f"Stage {decision.candidate_stage} is gated by max_execution_stage={self.max_execution_stage}.",
                check_passed=True,
                **common_status,
                **tracking_fields,
            )
            return

        try:
            self.motion_in_progress = True
            assert decision.candidate_pose_mmdeg is not None
            executed_candidate_pose_mmdeg = self._execute_move(decision.candidate_pose_mmdeg, decision.candidate_stage)
            self.last_executed_candidate_pose_mmdeg = list(executed_candidate_pose_mmdeg)
            common_status["executed_candidate_pose_mmdeg"] = list(executed_candidate_pose_mmdeg)
            common_status["move_j_pose_backoff"] = getattr(self, "last_move_j_pose_backoff", None)
            self.previous_stage = self.last_executed_stage
            self.last_executed_stage = decision.candidate_stage
            self._update_stage_latch(decision.candidate_stage)
            self._mark_stage_progress_after_execution(
                decision.candidate_stage,
                candidate_pose_mmdeg=executed_candidate_pose_mmdeg,
                pre_approach_pose_mmdeg=decision.pre_approach_pose_mmdeg,
            )

            post_move_tcp_pose_mmdeg = self._safe_get_current_tcp_pose_mmdeg() or current_tcp_for_status
            common_status["current_tcp_pose_mmdeg"] = post_move_tcp_pose_mmdeg
            common_status["current_tcp_to_final_hover_mm"] = self._distance_between_pose_positions_mm(
                post_move_tcp_pose_mmdeg,
                decision.final_hover_pose_mmdeg,
            )
            common_status["current_tcp_to_pre_approach_mm"] = self._distance_between_pose_positions_mm(
                post_move_tcp_pose_mmdeg,
                decision.pre_approach_pose_mmdeg,
            )
            self.last_status_fields["current_tcp_pose_mmdeg"] = post_move_tcp_pose_mmdeg
            self.last_status_fields["current_tcp_to_final_hover_mm"] = common_status["current_tcp_to_final_hover_mm"]
            self.last_status_fields["current_tcp_to_pre_approach_mm"] = common_status["current_tcp_to_pre_approach_mm"]

            position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(post_move_tcp_pose_mmdeg)
            tracking_state = TRACKING_WAITING_NEXT_FRAME
            if decision.candidate_stage == FINAL_HOVER_STAGE and self._final_hover_completion_reached(
                position_error_mm,
                normal_alignment_error_deg,
            ):
                tracking_state = TRACKING_SUCCEEDED_VISIBLE
            elif self._completion_reached(position_error_mm, normal_alignment_error_deg):
                tracking_state = TRACKING_SUCCEEDED_VISIBLE
            completion_reason = REASON_REACHED_VISIBLE if tracking_state == TRACKING_SUCCEEDED_VISIBLE else None
            tracking_fields = self._tracking_fields(
                tracking_state=tracking_state,
                completion_reason=completion_reason,
                current_tcp_pose_mmdeg=post_move_tcp_pose_mmdeg,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                now=self.get_clock().now(),
            )
            if tracking_state == TRACKING_SUCCEEDED_VISIBLE:
                self._mark_tracking_episode_completed(tracking_state)
                if self.execute_motion:
                    common_status["robot_release_result"] = self._release_robot_control()
                    common_status["execute_motion_disabled_after_success"] = True
                    self.execute_motion = False
            self._publish_status(
                event="control_executed",
                error_message="",
                check_passed=True,
                executed=True,
                **common_status,
                **tracking_fields,
            )
        except Exception as exc:
            release_result = self._release_robot_control()
            self.motion_execution_faulted = True
            self.execute_motion = False
            common_status["move_j_pose_backoff"] = getattr(self, "last_move_j_pose_backoff", None)
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE,
                completion_reason=None,
                current_tcp_pose_mmdeg=current_tcp_for_status,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                selected_source_status=selected_source_status,
                now=self.get_clock().now(),
            )
            self._publish_status(
                event="control_execute_failed",
                error_message=repr(exc),
                check_passed=True,
                robot_release_result=release_result,
                execute_motion_disabled_after_fault=True,
                **common_status,
                **tracking_fields,
            )
        finally:
            self.motion_in_progress = False

    def destroy_node(self) -> bool:
        try:
            self._release_robot_control()
        except BaseException as exc:
            self._record_internal_error(event="control_destroy_release_failed", exc=exc)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FairinoControlNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception as exc:
        node._record_internal_error(event="control_spin_failed", exc=exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
