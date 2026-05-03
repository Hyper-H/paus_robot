from __future__ import annotations

import json
from typing import Any

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from paus_motion_ros2 import FairinoLinuxClient, build_approach_decision, compute_normal_alignment_error_deg
from paus_perception import load_config, quaternion_xyzw_to_rotation_matrix, rotation_matrix_to_rpy_deg


PRE_APPROACH_STAGE = "pre_approach"
REORIENT_STAGE = "reorient"
FINAL_HOVER_STAGE = "final_hover"
SAFE_LIFT_STAGE = "safe_lift"

STAGE_ORDER = {
    SAFE_LIFT_STAGE: -1,
    PRE_APPROACH_STAGE: 0,
    REORIENT_STAGE: 1,
    FINAL_HOVER_STAGE: 2,
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

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("control_status_topic").get_parameter_value().string_value
        debug_status_topic = self.get_parameter("control_debug_status_topic").get_parameter_value().string_value
        detection_status_topic = self.get_parameter("detection_status_topic").get_parameter_value().string_value
        transform_status_topic = self.get_parameter("transform_status_topic").get_parameter_value().string_value

        self.config = load_config(config_path)
        control_cfg = self.config["control"]

        self.robot_ip = str(control_cfg["robot_ip"])
        self.linux_fairino_sdk_root = str(control_cfg["linux_fairino_sdk_root"])
        self.execute_motion = bool(control_cfg["execute_motion"])
        self.tool_id = int(control_cfg["tool_id"])
        self.user_id = int(control_cfg["user_id"])
        self.move_vel = float(control_cfg["move_vel"])
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
        self.max_execution_stage = str(control_cfg.get("max_execution_stage", FINAL_HOVER_STAGE))
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
        self.completion_position_tolerance_mm = float(control_cfg.get("completion_position_tolerance_mm", 20.0))
        self.completion_normal_tolerance_deg = float(control_cfg.get("completion_normal_tolerance_deg", 5.0))
        self.workspace_min_mm = [float(value) for value in control_cfg["workspace_min_mm"]]
        self.workspace_max_mm = [float(value) for value in control_cfg["workspace_max_mm"]]
        self.repeat_distance_threshold_mm = float(control_cfg["repeat_distance_threshold_mm"])
        self.stage_switch_buffer_mm = float(control_cfg.get("stage_switch_buffer_mm", self.repeat_distance_threshold_mm))
        self.use_mock_pose = bool(control_cfg["use_mock_pose"])
        self.mock_current_tcp_pose_mmdeg = [float(value) for value in control_cfg["mock_current_tcp_pose_mmdeg"]]

        self.declare_parameter("execute_motion", bool(control_cfg["execute_motion"]))
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)

        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.debug_status_publisher = self.create_publisher(String, debug_status_topic, 10)
        self.target_subscription = self.create_subscription(PoseStamped, target_pose_topic, self._target_callback, 10)
        self.detection_status_subscription = self.create_subscription(String, detection_status_topic, self._detection_status_callback, 10)
        self.transform_status_subscription = self.create_subscription(String, transform_status_topic, self._transform_status_callback, 10)
        self.status_timer = self.create_timer(STATUS_TIMER_PERIOD_S, self._status_timer_callback)

        self.last_executed_candidate_pose_mmdeg: list[float] | None = None
        self.last_executed_stage: str | None = None
        self.stage_latch: str | None = None
        self.motion_in_progress = False
        self.control_backend = "mock_pose"
        self.linux_client: FairinoLinuxClient | None = None

        self.last_valid_target_pose_base_mmdeg: list[float] | None = None
        self.last_valid_final_hover_pose_mmdeg: list[float] | None = None
        self.last_valid_surface_normal_base: list[float] | None = None
        self.last_valid_target_point_base_m: list[float] | None = None
        self.last_valid_target_time = None
        self.last_detection_status: dict[str, Any] | None = None
        self.last_transform_status: dict[str, Any] | None = None
        self.last_tracking_state = TRACKING_IDLE
        self.last_completion_reason = REASON_NO_VALID_TARGET
        self.last_timer_publish_signature: tuple[Any, ...] | None = None
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
    ) -> None:
        full_payload = {
            "event": event,
            "error_message": error_message,
            "check_passed": check_passed,
            "execute_motion": self.execute_motion,
            "executed": executed,
            "orientation_mode": self.orientation_mode,
            "flange_face_axis": self.flange_face_axis,
            "prefer_positive_z_surface_normal": self.prefer_positive_z_surface_normal,
            "enable_safe_lift_on_low_clearance": self.enable_safe_lift_on_low_clearance,
            "safe_lift_step_mm": self.safe_lift_step_mm,
            "safe_lift_above_marker_mm": self.safe_lift_above_marker_mm,
            "safe_lift_max_z_mm": self.safe_lift_max_z_mm,
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
            "step_distance_mm": step_distance_mm,
            "clearance_to_plane_mm": clearance_to_plane_mm,
            "tracking_state": tracking_state,
            "completion_reason": completion_reason,
            "target_age_ms": target_age_ms,
            "target_hold_timeout_ms": target_hold_timeout_ms,
            "last_valid_target_pose_base_mmdeg": last_valid_target_pose_base_mmdeg,
            "last_valid_final_hover_pose_mmdeg": last_valid_final_hover_pose_mmdeg,
            "position_error_to_last_valid_final_hover_mm": position_error_to_last_valid_final_hover_mm,
            "normal_alignment_error_deg": normal_alignment_error_deg,
            "marker_visibility_status": marker_visibility_status,
            "transform_validity_status": transform_validity_status,
            "control_backend": self.control_backend,
        }
        summary_payload = {
            "event": event,
            "tracking_state": tracking_state,
            "completion_reason": completion_reason,
            "candidate_stage": candidate_stage,
            "motion_command": motion_command,
            "check_passed": check_passed,
            "executed": executed,
            "marker_visibility_status": marker_visibility_status,
            "transform_validity_status": transform_validity_status,
            "target_age_ms": target_age_ms,
            "position_error_to_last_valid_final_hover_mm": position_error_to_last_valid_final_hover_mm,
            "normal_alignment_error_deg": normal_alignment_error_deg,
            "control_backend": self.control_backend,
        }

        status_message = String()
        status_message.data = json.dumps(summary_payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)

        debug_message = String()
        debug_message.data = json.dumps(full_payload, ensure_ascii=False)
        self.debug_status_publisher.publish(debug_message)
        self.get_logger().info(debug_message.data)
        if tracking_state is not None:
            self.last_tracking_state = tracking_state
        self.last_completion_reason = completion_reason

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

    def _get_current_tcp_pose_mmdeg(self) -> list[float]:
        if self.control_backend == "mock_pose" or self.linux_client is None:
            return list(self.mock_current_tcp_pose_mmdeg)
        error, pose = self.linux_client.get_actual_tcp_pose()
        if error != 0:
            raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
        return pose

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
        if candidate_stage == SAFE_LIFT_STAGE:
            return "MoveL"
        if candidate_stage in (PRE_APPROACH_STAGE, REORIENT_STAGE):
            return "MoveJ"
        if candidate_stage == FINAL_HOVER_STAGE:
            return "MoveL"
        return None

    def _stage_allowed(self, candidate_stage: str | None) -> bool:
        if candidate_stage is None:
            return False
        return STAGE_ORDER[candidate_stage] <= STAGE_ORDER[self.max_execution_stage]

    def _execute_move(self, candidate_pose_mmdeg: list[float], candidate_stage: str | None) -> None:
        if self.control_backend != "linux_sdk" or self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        if candidate_stage in (PRE_APPROACH_STAGE, REORIENT_STAGE):
            joint_error, current_joint_pos_deg = self.linux_client.get_actual_joint_pos_degree()
            if joint_error != 0:
                raise RuntimeError(f"GetActualJointPosDegree failed with code {joint_error}.")
            result = self.linux_client.move_j_pose(
                candidate_pose_mmdeg,
                joint_pos_ref_deg=current_joint_pos_deg,
                tool_id=self.tool_id,
                user_id=self.user_id,
                vel=self.move_vel,
            )
        else:
            result = self.linux_client.move_l(candidate_pose_mmdeg, tool_id=self.tool_id, user_id=self.user_id, vel=self.move_vel)
        if result != 0:
            raise RuntimeError(f"{self._motion_command_for_stage(candidate_stage)} failed with code {result}.")

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

    def _tracking_fields(
        self,
        *,
        tracking_state: str,
        completion_reason: str | None,
        current_tcp_pose_mmdeg: list[float] | None,
        marker_visibility_status: str,
        transform_validity_status: str,
        now=None,
    ) -> dict[str, Any]:
        if now is None:
            now = self.get_clock().now()
        target_age_ms = self._target_age_ms(now)
        position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(current_tcp_pose_mmdeg)
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

    def _update_stage_latch(self, candidate_stage: str | None) -> None:
        if candidate_stage == SAFE_LIFT_STAGE:
            self.stage_latch = None
        elif candidate_stage in (REORIENT_STAGE, FINAL_HOVER_STAGE):
            new_order = STAGE_ORDER.get(candidate_stage, 0)
            current_order = STAGE_ORDER.get(self.stage_latch, 0)
            if new_order > current_order:
                self.stage_latch = candidate_stage

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

    def _status_timer_callback(self) -> None:
        if self.motion_in_progress:
            return

        now = self.get_clock().now()
        marker_visibility_status = self._marker_visibility_status()
        transform_validity_status = self._transform_validity_status()

        if self.last_valid_target_time is None:
            tracking_state = TRACKING_IDLE
            completion_reason = REASON_NO_VALID_TARGET
            current_tcp_pose_mmdeg = None
        elif not self._completion_allowed() and self.last_tracking_state == TRACKING_STAGE_GATED:
            tracking_state = TRACKING_STAGE_GATED
            completion_reason = REASON_MAX_STAGE_GATED
            current_tcp_pose_mmdeg = self._safe_get_current_tcp_pose_mmdeg()
        elif marker_visibility_status == "ok" and transform_validity_status == "ok":
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
            if target_age_ms is None:
                tracking_state = TRACKING_IDLE
                completion_reason = REASON_NO_VALID_TARGET
            elif target_age_ms < float(self.target_hold_timeout_ms):
                tracking_state = TRACKING_TARGET_LOST_PENDING
                completion_reason = None
            else:
                position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(current_tcp_pose_mmdeg)
                if self._completion_allowed() and self._completion_reached(position_error_mm, normal_alignment_error_deg):
                    tracking_state = TRACKING_SUCCEEDED_AFTER_OCCLUSION
                    completion_reason = REASON_REACHED_AFTER_OCCLUSION
                elif not self._completion_allowed() and self.last_tracking_state == TRACKING_STAGE_GATED:
                    tracking_state = TRACKING_STAGE_GATED
                    completion_reason = REASON_MAX_STAGE_GATED
                else:
                    tracking_state = TRACKING_TARGET_LOST
                    completion_reason = REASON_TARGET_LOST_TIMEOUT

        signature = (tracking_state, completion_reason, marker_visibility_status, transform_validity_status)
        if signature == self.last_timer_publish_signature:
            return
        self.last_timer_publish_signature = signature

        tracking_fields = self._tracking_fields(
            tracking_state=tracking_state,
            completion_reason=completion_reason,
            current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
            marker_visibility_status=marker_visibility_status,
            transform_validity_status=transform_validity_status,
            now=now,
        )
        error_message = ""
        if tracking_state == TRACKING_TARGET_LOST_PENDING:
            error_message = "Waiting for marker recovery before hold timeout expires."
        elif tracking_state == TRACKING_TARGET_LOST:
            error_message = "Marker remained unavailable past hold timeout before final hover was reached."
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
        now = self.get_clock().now()

        if self.motion_in_progress:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE,
                completion_reason=None,
                current_tcp_pose_mmdeg=self.last_status_fields.get("current_tcp_pose_mmdeg"),
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
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

        try:
            current_tcp_pose_mmdeg = self._get_current_tcp_pose_mmdeg()
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
            )
        except Exception as exc:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE if self.last_valid_target_time is not None else TRACKING_IDLE,
                completion_reason=None if self.last_valid_target_time is not None else REASON_NO_VALID_TARGET,
                current_tcp_pose_mmdeg=None,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
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
        }
        self.last_status_fields.update(common_status)

        if not decision.check_passed:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE if self.last_valid_target_time is not None else TRACKING_IDLE,
                completion_reason=None if self.last_valid_target_time is not None else REASON_NO_VALID_TARGET,
                current_tcp_pose_mmdeg=current_tcp_for_status,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                now=now,
            )
            self._publish_status(
                event="control_rejected",
                error_message=decision.error_message,
                check_passed=False,
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
                    now=now,
                )
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
                now=now,
            )
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
            self._execute_move(decision.candidate_pose_mmdeg, decision.candidate_stage)
            self.last_executed_candidate_pose_mmdeg = list(decision.candidate_pose_mmdeg)
            self.last_executed_stage = decision.candidate_stage
            self._update_stage_latch(decision.candidate_stage)

            post_move_tcp_pose_mmdeg = self._safe_get_current_tcp_pose_mmdeg() or current_tcp_for_status
            common_status["current_tcp_pose_mmdeg"] = post_move_tcp_pose_mmdeg
            self.last_status_fields["current_tcp_pose_mmdeg"] = post_move_tcp_pose_mmdeg

            position_error_mm, normal_alignment_error_deg = self._compute_completion_metrics(post_move_tcp_pose_mmdeg)
            tracking_state = (
                TRACKING_SUCCEEDED_VISIBLE
                if self._completion_reached(position_error_mm, normal_alignment_error_deg)
                else TRACKING_WAITING_NEXT_FRAME
            )
            completion_reason = REASON_REACHED_VISIBLE if tracking_state == TRACKING_SUCCEEDED_VISIBLE else None
            tracking_fields = self._tracking_fields(
                tracking_state=tracking_state,
                completion_reason=completion_reason,
                current_tcp_pose_mmdeg=post_move_tcp_pose_mmdeg,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                now=self.get_clock().now(),
            )
            self._publish_status(
                event="control_executed",
                error_message="",
                check_passed=True,
                executed=True,
                **common_status,
                **tracking_fields,
            )
        except Exception as exc:
            tracking_fields = self._tracking_fields(
                tracking_state=TRACKING_ACTIVE,
                completion_reason=None,
                current_tcp_pose_mmdeg=current_tcp_for_status,
                marker_visibility_status=marker_visibility_status,
                transform_validity_status=transform_validity_status,
                now=self.get_clock().now(),
            )
            self._publish_status(
                event="control_execute_failed",
                error_message=repr(exc),
                check_passed=True,
                **common_status,
                **tracking_fields,
            )
        finally:
            self.motion_in_progress = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FairinoControlNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
