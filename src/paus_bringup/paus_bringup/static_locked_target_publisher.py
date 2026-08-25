from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from paus_perception import rotation_matrix_to_quaternion_xyzw, rpy_deg_to_rotation_matrix


DEFAULT_LOCKED_TARGET_POSE_TOPIC = "/locked_target_pose_base"
DEFAULT_TARGET_LOCK_STATUS_TOPIC = "/target_lock_status"
DEFAULT_SELECTOR_STATUS_TOPIC = "/target_selector_status"


def _parse_pose_mmdeg(value: str) -> list[float]:
    parts = [part.strip() for part in value.replace(";", ",").split(",") if part.strip()]
    if len(parts) != 6:
        raise ValueError("pose_mmdeg must contain exactly 6 comma-separated values: x,y,z,rx,ry,rz")
    return [float(part) for part in parts]


def _pose_from_trace(trace_path: Path, field: str) -> list[float]:
    if not trace_path.exists():
        raise FileNotFoundError(f"Static target trace does not exist: {trace_path}")
    fallback_field = "raw_target_pose_base_mmdeg" if field != "raw_target_pose_base_mmdeg" else "target_pose_base_mmdeg"
    selected: list[float] | None = None
    fallback: list[float] | None = None
    with trace_path.open("r", encoding="utf-8") as trace_file:
        for line in trace_file:
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            value = payload.get(field)
            if isinstance(value, list) and len(value) >= 6:
                selected = [float(item) for item in value[:6]]
            fallback_value = payload.get(fallback_field)
            if isinstance(fallback_value, list) and len(fallback_value) >= 6:
                fallback = [float(item) for item in fallback_value[:6]]
    if selected is not None:
        return selected
    if fallback is not None:
        return fallback
    raise ValueError(f"No usable {field!r} pose found in {trace_path}")


def _pose_message_from_mmdeg(pose_mmdeg: list[float], frame_id: str, node: Node) -> PoseStamped:
    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = frame_id
    message.pose.position.x = float(pose_mmdeg[0]) / 1000.0
    message.pose.position.y = float(pose_mmdeg[1]) / 1000.0
    message.pose.position.z = float(pose_mmdeg[2]) / 1000.0
    rotation = rpy_deg_to_rotation_matrix(pose_mmdeg[3:6])
    quat = rotation_matrix_to_quaternion_xyzw(rotation)
    message.pose.orientation.x = float(quat[0])
    message.pose.orientation.y = float(quat[1])
    message.pose.orientation.z = float(quat[2])
    message.pose.orientation.w = float(quat[3])
    return message


class StaticLockedTargetPublisher(Node):
    def __init__(
        self,
        *,
        pose_mmdeg: list[float],
        frame_id: str,
        locked_target_pose_topic: str,
        target_lock_status_topic: str,
        selector_status_topic: str,
        publish_rate_hz: float,
        trace_path: str | None,
    ) -> None:
        super().__init__("static_locked_target_publisher")
        self.pose_mmdeg = [float(value) for value in pose_mmdeg]
        self.frame_id = frame_id
        self.locked_target_pose_topic = locked_target_pose_topic
        self.target_lock_status_topic = target_lock_status_topic
        self.selector_status_topic = selector_status_topic
        self.trace_path = trace_path
        self.pose_publisher = self.create_publisher(PoseStamped, locked_target_pose_topic, 10)
        self.lock_status_publisher = self.create_publisher(String, target_lock_status_topic, 10)
        self.selector_status_publisher = self.create_publisher(String, selector_status_topic, 10)
        self.timer = self.create_timer(1.0 / max(0.1, float(publish_rate_hz)), self._publish)
        self.get_logger().info(
            json.dumps(
                {
                    "event": "static_locked_target_started",
                    "pose_mmdeg": self.pose_mmdeg,
                    "frame_id": self.frame_id,
                    "locked_target_pose_topic": self.locked_target_pose_topic,
                    "target_lock_status_topic": self.target_lock_status_topic,
                    "selector_status_topic": self.selector_status_topic,
                    "trace_path": self.trace_path,
                },
                ensure_ascii=False,
            )
        )

    def _publish_json(self, publisher, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(message)

    def _publish(self) -> None:
        pose_message = _pose_message_from_mmdeg(self.pose_mmdeg, self.frame_id, self)
        self.pose_publisher.publish(pose_message)
        now = time.time()
        pose_payload = {
            "position_m": [pose_message.pose.position.x, pose_message.pose.position.y, pose_message.pose.position.z],
            "orientation_xyzw": [
                pose_message.pose.orientation.x,
                pose_message.pose.orientation.y,
                pose_message.pose.orientation.z,
                pose_message.pose.orientation.w,
            ],
            "pose_mmdeg": self.pose_mmdeg,
            "frame_id": self.frame_id,
        }
        self._publish_json(
            self.lock_status_publisher,
            {
                "source": "static_locked_target",
                "event": "static_locked_target_published",
                "state": "locked",
                "enabled": True,
                "locked": True,
                "locked_pose_base": pose_payload,
                "latest_live_to_locked_mm": 0.0,
                "drift_warning_mm": None,
                "drift_action": "static_target",
                "locked_target_pose_topic": self.locked_target_pose_topic,
                "stamp_unix_s": now,
            },
        )
        self._publish_json(
            self.selector_status_publisher,
            {
                "source": "static_locked_target",
                "event": "static_target_selected",
                "status": "ok",
                "source_status": "ok",
                "selected_source": "markerless_neck",
                "source_age_ms": 0.0,
                "stamp_unix_s": now,
            },
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish a fixed locked markerless target pose for motion debugging.")
    parser.add_argument("--pose-mmdeg", default="", help="Static raw target pose as x,y,z,rx,ry,rz in mm/deg.")
    parser.add_argument("--trace-path", default="", help="Read the static target pose from a control_trace.jsonl file.")
    parser.add_argument("--trace-field", default="raw_target_pose_base_mmdeg", help="Trace field to use for the target pose.")
    parser.add_argument("--frame-id", default="robot_base", help="Pose frame id.")
    parser.add_argument("--locked-target-pose-topic", default=DEFAULT_LOCKED_TARGET_POSE_TOPIC)
    parser.add_argument("--target-lock-status-topic", default=DEFAULT_TARGET_LOCK_STATUS_TOPIC)
    parser.add_argument("--selector-status-topic", default=DEFAULT_SELECTOR_STATUS_TOPIC)
    parser.add_argument("--publish-rate-hz", type=float, default=5.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args, ros_args = parser.parse_known_args(argv)
    try:
        if args.pose_mmdeg.strip():
            pose_mmdeg = _parse_pose_mmdeg(args.pose_mmdeg)
            trace_path = None
        elif args.trace_path.strip():
            trace_path_obj = Path(args.trace_path).expanduser()
            pose_mmdeg = _pose_from_trace(trace_path_obj, args.trace_field)
            trace_path = str(trace_path_obj)
        else:
            parser.error("Either --pose-mmdeg or --trace-path is required.")
    except Exception as exc:
        print(f"static_locked_target_publisher: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = StaticLockedTargetPublisher(
        pose_mmdeg=pose_mmdeg,
        frame_id=args.frame_id,
        locked_target_pose_topic=args.locked_target_pose_topic,
        target_lock_status_topic=args.target_lock_status_topic,
        selector_status_topic=args.selector_status_topic,
        publish_rate_hz=args.publish_rate_hz,
        trace_path=trace_path,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
