from __future__ import annotations

import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from paus_perception import (
    load_config,
    load_eye_to_hand_solution,
    make_transform_matrix,
    make_transform_struct,
    quaternion_xyzw_to_rotation_matrix,
    rpy_deg_to_rotation_matrix,
    split_transform_matrix,
)


class TargetTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("target_transform_node")

        bringup_share = get_package_share_directory("paus_bringup")
        self.declare_parameter("config_path", f"{bringup_share}/configs/default.yaml")
        self.declare_parameter("extrinsics_path", f"{bringup_share}/configs/extrinsics.yaml")
        self.declare_parameter("marker_pose_topic", "/marker_pose")
        self.declare_parameter("target_pose_topic", "/target_pose_base")
        self.declare_parameter("target_point_topic", "/target_point_base")
        self.declare_parameter("status_topic", "/transform_status")
        self.declare_parameter("run_dir", "")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        extrinsics_path = self.get_parameter("extrinsics_path").get_parameter_value().string_value
        marker_pose_topic = self.get_parameter("marker_pose_topic").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        target_point_topic = self.get_parameter("target_point_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        run_dir_param = self.get_parameter("run_dir").get_parameter_value().string_value.strip()
        self.run_dir = Path(run_dir_param) if run_dir_param else None
        self.trace_path = self.run_dir / "transform_trace.jsonl" if self.run_dir is not None else None
        self.status_latest_path = self.run_dir / "transform_status_latest.json" if self.run_dir is not None else None
        self.target_pose_topic = target_pose_topic
        self.target_point_topic = target_point_topic

        self.config = load_config(config_path)
        self.eye_to_hand_solution = load_eye_to_hand_solution(extrinsics_path)

        transform_cfg = self.config["transform"]
        self.max_target_distance_mm = float(transform_cfg.get("max_target_distance_mm", 1500.0))

        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.target_pose_publisher = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.target_point_publisher = self.create_publisher(PointStamped, target_point_topic, 10)
        self.subscription = self.create_subscription(PoseStamped, marker_pose_topic, self._marker_pose_callback, 10)

        marker_to_target_cfg = transform_cfg["marker_to_target"]
        self.marker_to_target = make_transform_matrix(
            marker_to_target_cfg["translation_m"],
            rpy_deg_to_rotation_matrix(marker_to_target_cfg["rotation_rpy_deg"]),
        )

        self._publish_status(
            "ready" if self.eye_to_hand_solution.success else "missing_extrinsic",
            self.eye_to_hand_solution.message,
            {
                "event": "target_transform_node_started",
                "target_pose_topic": self.target_pose_topic,
                "target_point_topic": self.target_point_topic,
                "run_dir": str(self.run_dir) if self.run_dir is not None else None,
                "extrinsics_path": self.eye_to_hand_solution.source_path,
                "extrinsics_artifact_kind": self.eye_to_hand_solution.artifact_kind,
                "extrinsics_dummy": self.eye_to_hand_solution.is_dummy,
            },
        )

    def _publish_status(self, status: str, message: str, extra: dict[str, object] | None = None) -> None:
        payload = {
            "source": "target_transform",
            "status": status,
            "message": message,
            "stamp_unix_s": time.time(),
        }
        if extra:
            payload.update(extra)
        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)
        self._write_transform_log(payload)

    def _write_transform_log(self, payload: dict[str, object]) -> None:
        if self.trace_path is None or self.status_latest_path is None:
            return
        try:
            self.trace_path.parent.mkdir(parents=True, exist_ok=True)
            self.status_latest_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            with self.trace_path.open("a", encoding="utf-8") as trace_file:
                trace_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
        except Exception as exc:
            self.get_logger().warning(json.dumps({"event": "transform_trace_write_failed", "error": repr(exc)}, ensure_ascii=False))

    def _validate_target_translation(self, translation_mm: np.ndarray) -> tuple[bool, str]:
        if not np.all(np.isfinite(translation_mm)):
            return False, "target_translation_non_finite"
        target_distance_mm = float(np.linalg.norm(translation_mm))
        if target_distance_mm > self.max_target_distance_mm:
            return False, f"target_distance_above_max:{target_distance_mm:.1f}mm"
        return True, ""

    def _marker_pose_callback(self, message: PoseStamped) -> None:
        input_frame_id = str(message.header.frame_id)
        input_stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if not self.eye_to_hand_solution.success or self.eye_to_hand_solution.base_to_camera is None:
            self._publish_status(
                "missing_extrinsic",
                "base_to_camera is unavailable.",
                {
                    "event": "target_transform_missing_extrinsic",
                    "input_frame_id": input_frame_id,
                    "input_stamp_ns": input_stamp_ns,
                    "target_pose_topic": self.target_pose_topic,
                    "target_point_topic": self.target_point_topic,
                    "extrinsics_path": self.eye_to_hand_solution.source_path,
                    "extrinsics_artifact_kind": self.eye_to_hand_solution.artifact_kind,
                    "extrinsics_dummy": self.eye_to_hand_solution.is_dummy,
                },
            )
            return

        marker_position_camera_m = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
        camera_to_marker = make_transform_matrix(
            marker_position_camera_m,
            quaternion_xyzw_to_rotation_matrix(
                [
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ]
            ),
        )
        base_to_camera = make_transform_matrix(
            self.eye_to_hand_solution.base_to_camera.translation_m,
            self.eye_to_hand_solution.base_to_camera.rotation_matrix,
        )
        base_to_target = base_to_camera @ camera_to_marker @ self.marker_to_target
        translation_m, rotation = split_transform_matrix(base_to_target)
        translation_mm = np.asarray(translation_m, dtype=np.float64).reshape(3) * 1000.0

        valid, reject_reason = self._validate_target_translation(translation_mm)
        if not valid:
            self._publish_status(
                "target_filtered",
                f"Target transform rejected: {reject_reason}",
                {
                    "event": "target_transform_rejected",
                    "input_frame_id": input_frame_id,
                    "input_stamp_ns": input_stamp_ns,
                    "marker_position_camera_m": [float(value) for value in marker_position_camera_m],
                    "target_point_base_mm": [float(value) for value in translation_mm.tolist()],
                    "reject_reason": reject_reason,
                    "max_target_distance_mm": self.max_target_distance_mm,
                    "target_pose_topic": self.target_pose_topic,
                    "target_point_topic": self.target_point_topic,
                },
            )
            return

        transform = make_transform_struct(translation_m, rotation, "robot_base", "target_region")

        pose_message = PoseStamped()
        pose_message.header.stamp = message.header.stamp
        pose_message.header.frame_id = "robot_base"
        pose_message.pose.position.x = transform.translation_m[0]
        pose_message.pose.position.y = transform.translation_m[1]
        pose_message.pose.position.z = transform.translation_m[2]
        pose_message.pose.orientation.x = transform.rotation_quaternion_xyzw[0]
        pose_message.pose.orientation.y = transform.rotation_quaternion_xyzw[1]
        pose_message.pose.orientation.z = transform.rotation_quaternion_xyzw[2]
        pose_message.pose.orientation.w = transform.rotation_quaternion_xyzw[3]
        self.target_pose_publisher.publish(pose_message)

        point_message = PointStamped()
        point_message.header.stamp = pose_message.header.stamp
        point_message.header.frame_id = pose_message.header.frame_id
        point_message.point.x = transform.translation_m[0]
        point_message.point.y = transform.translation_m[1]
        point_message.point.z = transform.translation_m[2]
        self.target_point_publisher.publish(point_message)

        self._publish_status(
            "ok",
            "Target transform published successfully.",
            {
                "event": "target_transform_published",
                "input_frame_id": input_frame_id,
                "input_stamp_ns": input_stamp_ns,
                "marker_position_camera_m": [float(value) for value in marker_position_camera_m],
                "target_point_base": transform.translation_m,
                "target_point_base_mm": [float(value) for value in translation_mm.tolist()],
                "target_pose_topic": self.target_pose_topic,
                "target_point_topic": self.target_point_topic,
            },
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetTransformNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
