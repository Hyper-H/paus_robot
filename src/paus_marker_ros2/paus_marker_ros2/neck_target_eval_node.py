from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .neck_target_eval import (
    DEFAULT_EVAL_STATUS_TOPIC,
    DEFAULT_LOGGING_OUTPUT_DIR,
    DEFAULT_MARKER_POSE_TOPIC,
    DEFAULT_MAX_PAIR_DELTA_MS,
    DEFAULT_NECK_STATUS_TOPIC,
    build_eval_record,
    stamp_to_ns,
)


class NeckTargetEvalNode(Node):
    def __init__(self) -> None:
        super().__init__("neck_target_eval_node")
        self.declare_parameter("marker_pose_topic", DEFAULT_MARKER_POSE_TOPIC)
        self.declare_parameter("neck_status_topic", DEFAULT_NECK_STATUS_TOPIC)
        self.declare_parameter("status_topic", DEFAULT_EVAL_STATUS_TOPIC)
        self.declare_parameter("logging_output_dir", DEFAULT_LOGGING_OUTPUT_DIR)
        self.declare_parameter("max_pair_delta_ms", DEFAULT_MAX_PAIR_DELTA_MS)
        self.declare_parameter("logging_enabled", True)

        marker_pose_topic = self.get_parameter("marker_pose_topic").get_parameter_value().string_value
        neck_status_topic = self.get_parameter("neck_status_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.max_pair_delta_ms = float(self.get_parameter("max_pair_delta_ms").get_parameter_value().double_value)
        self.logging_enabled = bool(self.get_parameter("logging_enabled").get_parameter_value().bool_value)
        self.log_dir = self._resolve_log_dir(self.get_parameter("logging_output_dir").get_parameter_value().string_value)
        self.log_path: Path | None = None
        self.status_latest_path: Path | None = None

        self.latest_neck_payload: dict[str, Any] | None = None
        self.latest_neck_stamp_ns: int | None = None
        self.latest_marker_pose_camera_m: list[float] | None = None
        self.latest_marker_stamp_ns: int | None = None
        self.last_published_pair: tuple[int, int] | None = None

        self.neck_subscription = self.create_subscription(String, neck_status_topic, self._neck_status_callback, 10)
        self.marker_subscription = self.create_subscription(PoseStamped, marker_pose_topic, self._marker_pose_callback, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)

        if self.logging_enabled:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.log_path = self.log_dir / "neck_target_eval.jsonl"
            self.status_latest_path = self.log_dir / "status_latest.json"

        self._publish_status(
            {
                "source": "neck_target_eval",
                "status": "ready",
                "marker_pose_topic": marker_pose_topic,
                "neck_status_topic": neck_status_topic,
                "status_topic": status_topic,
                "logging_enabled": self.logging_enabled,
                "logging_output_dir": str(self.log_dir),
                "max_pair_delta_ms": self.max_pair_delta_ms,
            }
        )

    def _resolve_log_dir(self, logging_output_dir: str) -> Path:
        path = Path(logging_output_dir)
        if path.is_absolute():
            return path
        package_share = Path(get_package_share_directory("paus_marker_ros2"))
        workspace_root = package_share.parents[3]
        return workspace_root / path

    def _neck_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self._publish_status(
                {
                    "source": "neck_target_eval",
                    "status": "waiting",
                    "reason": "invalid_neck_status_json",
                    "message": str(exc),
                }
            )
            return
        if not isinstance(payload, dict):
            self._publish_status(
                {
                    "source": "neck_target_eval",
                    "status": "waiting",
                    "reason": "invalid_neck_status_payload",
                    "message": "Neck status JSON must be an object.",
                }
            )
            return
        self.latest_neck_payload = payload
        self.latest_neck_stamp_ns = self._neck_payload_stamp_ns(payload)
        self._try_publish_eval()

    def _neck_payload_stamp_ns(self, payload: dict[str, Any]) -> int:
        stamp = payload.get("image_stamp_ns") or payload.get("stamp_ns")
        if stamp is not None:
            try:
                return int(stamp)
            except (TypeError, ValueError):
                pass
        return int(self.get_clock().now().nanoseconds)

    def _marker_pose_callback(self, message: PoseStamped) -> None:
        self.latest_marker_pose_camera_m = [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ]
        self.latest_marker_stamp_ns = stamp_to_ns(message.header.stamp)
        self._try_publish_eval()

    def _try_publish_eval(self) -> None:
        if self.latest_neck_payload is None or self.latest_neck_stamp_ns is None:
            self._publish_status(
                {
                    "source": "neck_target_eval",
                    "status": "waiting",
                    "reason": "no_markerless_status_yet",
                }
            )
            return
        if self.latest_marker_pose_camera_m is None or self.latest_marker_stamp_ns is None:
            self._publish_status(
                {
                    "source": "neck_target_eval",
                    "status": "waiting",
                    "reason": "no_marker_pose_yet",
                }
            )
            return

        pair = (self.latest_neck_stamp_ns, self.latest_marker_stamp_ns)
        if pair == self.last_published_pair:
            return
        record = build_eval_record(
            self.latest_neck_payload,
            self.latest_marker_pose_camera_m,
            neck_stamp_ns=self.latest_neck_stamp_ns,
            marker_stamp_ns=self.latest_marker_stamp_ns,
            max_pair_delta_ms=self.max_pair_delta_ms,
        )
        self._publish_status(record)
        if record.get("status") == "ok":
            self.last_published_pair = pair
            self._append_record(record)

    def _publish_status(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)
        self._write_latest_status(payload)

    def _write_latest_status(self, payload: dict[str, Any]) -> None:
        if not self.logging_enabled or self.status_latest_path is None:
            return
        try:
            self.status_latest_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            self.get_logger().warning(json.dumps({"event": "neck_target_eval_status_latest_failed", "error": repr(exc)}, ensure_ascii=False))

    def _append_record(self, record: dict[str, Any]) -> None:
        if not self.logging_enabled or self.log_path is None:
            return
        try:
            with self.log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as exc:
            self.get_logger().warning(json.dumps({"event": "neck_target_eval_logging_failed", "error": repr(exc)}, ensure_ascii=False))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NeckTargetEvalNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
