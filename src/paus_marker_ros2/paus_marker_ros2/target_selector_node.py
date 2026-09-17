from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from paus_perception import load_config


MARKERLESS_NECK_MODE = "markerless_neck"
MARKER_MODE = "marker"
SUPPORTED_TARGETING_MODES = {MARKERLESS_NECK_MODE, MARKER_MODE}

DEFAULT_MARKERLESS_TARGET_POSE_TOPIC = "/markerless_neck_target_pose_base"
DEFAULT_MARKER_TARGET_POSE_TOPIC = "/marker_target_pose_base"
DEFAULT_SELECTED_TARGET_POSE_TOPIC = "/selected_target_pose_base"
DEFAULT_STATUS_TOPIC = "/target_selector_status"
DEFAULT_SOURCE_TIMEOUT_MS = 500
STATUS_TIMER_PERIOD_S = 0.1


class TargetSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("target_selector_node")
        bringup_share = Path(get_package_share_directory("paus_bringup"))

        self.declare_parameter("config_path", str(bringup_share / "configs" / "default.yaml"))
        self.declare_parameter("mode", "")
        self.declare_parameter("source_timeout_ms", 0)
        self.declare_parameter("markerless_target_pose_topic", DEFAULT_MARKERLESS_TARGET_POSE_TOPIC)
        self.declare_parameter("marker_target_pose_topic", DEFAULT_MARKER_TARGET_POSE_TOPIC)
        self.declare_parameter("selected_target_pose_topic", DEFAULT_SELECTED_TARGET_POSE_TOPIC)
        self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
        self.declare_parameter("run_dir", "")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        config = load_config(config_path)
        targeting_cfg = config.get("targeting", {})
        configured_mode = str(targeting_cfg.get("mode", MARKERLESS_NECK_MODE))
        mode_param = self.get_parameter("mode").get_parameter_value().string_value.strip()
        self.mode = mode_param or configured_mode

        timeout_param = int(self.get_parameter("source_timeout_ms").get_parameter_value().integer_value)
        self.source_timeout_ms = timeout_param if timeout_param > 0 else int(targeting_cfg.get("source_timeout_ms", DEFAULT_SOURCE_TIMEOUT_MS))
        self.markerless_target_pose_topic = self.get_parameter("markerless_target_pose_topic").get_parameter_value().string_value
        self.marker_target_pose_topic = self.get_parameter("marker_target_pose_topic").get_parameter_value().string_value
        self.selected_target_pose_topic = self.get_parameter("selected_target_pose_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        run_dir_param = self.get_parameter("run_dir").get_parameter_value().string_value.strip()
        self.run_dir = Path(run_dir_param) if run_dir_param else None
        self.trace_path = self.run_dir / "selector_trace.jsonl" if self.run_dir is not None else None
        self.status_latest_path = self.run_dir / "selector_status_latest.json" if self.run_dir is not None else None

        self.latest_markerless_pose: PoseStamped | None = None
        self.latest_marker_pose: PoseStamped | None = None
        self.last_selected_stamp_ns: int | None = None
        self.last_status_signature: tuple[Any, ...] | None = None

        self.markerless_subscription = self.create_subscription(
            PoseStamped,
            self.markerless_target_pose_topic,
            self._markerless_pose_callback,
            10,
        )
        self.marker_subscription = self.create_subscription(
            PoseStamped,
            self.marker_target_pose_topic,
            self._marker_pose_callback,
            10,
        )
        self.selected_publisher = self.create_publisher(PoseStamped, self.selected_target_pose_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.status_timer = self.create_timer(STATUS_TIMER_PERIOD_S, self._status_timer_callback)

        if self.mode not in SUPPORTED_TARGETING_MODES:
            self._publish_status(
                self._build_status(
                    "failed",
                    "invalid_mode",
                    published=False,
                    message=f"Unsupported targeting mode: {self.mode!r}",
                ),
                force_trace=True,
            )
        else:
            self._publish_status(
                self._build_status(
                    "ready",
                    None,
                    published=False,
                    message="Target selector node started.",
                ),
                force_trace=True,
            )

    def _markerless_pose_callback(self, message: PoseStamped) -> None:
        self.latest_markerless_pose = message
        self._handle_source_pose(MARKERLESS_NECK_MODE, message)

    def _marker_pose_callback(self, message: PoseStamped) -> None:
        self.latest_marker_pose = message
        self._handle_source_pose(MARKER_MODE, message)

    def _handle_source_pose(self, source: str, message: PoseStamped) -> None:
        if self.mode not in SUPPORTED_TARGETING_MODES:
            self._publish_status(
                self._build_status(
                    "failed",
                    "invalid_mode",
                    source=source,
                    source_pose=message,
                    published=False,
                    message=f"Unsupported targeting mode: {self.mode!r}",
                ),
                force_trace=True,
            )
            return
        if source != self.mode:
            self._publish_status(
                self._build_status(
                    "ignored",
                    "source_not_selected",
                    source=source,
                    source_pose=message,
                    published=False,
                    message=f"Ignoring {source} target because mode is {self.mode}.",
                ),
                force_trace=True,
            )
            return

        source_age_ms = self._source_age_ms(message)
        if source_age_ms is not None and source_age_ms > float(self.source_timeout_ms):
            self._publish_status(
                self._build_status(
                    "stale",
                    "source_timeout",
                    source=source,
                    source_pose=message,
                    published=False,
                    source_age_ms=source_age_ms,
                    message="Selected source target is stale.",
                ),
                force_trace=True,
            )
            return

        selected_message = deepcopy(message)
        selected_message.header.frame_id = message.header.frame_id
        self.selected_publisher.publish(selected_message)
        self.last_selected_stamp_ns = self._stamp_ns(message)
        self._publish_status(
            self._build_status(
                "ok",
                None,
                source=source,
                source_pose=message,
                published=True,
                source_age_ms=source_age_ms,
                message="Selected target pose published.",
            ),
            force_trace=True,
        )

    def _status_timer_callback(self) -> None:
        if self.mode not in SUPPORTED_TARGETING_MODES:
            self._publish_status(
                self._build_status("failed", "invalid_mode", published=False, message=f"Unsupported targeting mode: {self.mode!r}"),
            )
            return
        source_pose = self._selected_source_pose()
        if source_pose is None:
            self._publish_status(
                self._build_status(
                    "waiting",
                    "source_missing",
                    source=self.mode,
                    published=False,
                    message="Waiting for selected source target.",
                )
            )
            return
        source_age_ms = self._source_age_ms(source_pose)
        if source_age_ms is not None and source_age_ms > float(self.source_timeout_ms):
            self._publish_status(
                self._build_status(
                    "stale",
                    "source_timeout",
                    source=self.mode,
                    source_pose=source_pose,
                    published=False,
                    source_age_ms=source_age_ms,
                    message="Selected source target is stale.",
                )
            )

    def _selected_source_pose(self) -> PoseStamped | None:
        if self.mode == MARKERLESS_NECK_MODE:
            return self.latest_markerless_pose
        if self.mode == MARKER_MODE:
            return self.latest_marker_pose
        return None

    def _build_status(
        self,
        status: str,
        reason: str | None,
        *,
        source: str | None = None,
        source_pose: PoseStamped | None = None,
        published: bool,
        source_age_ms: float | None = None,
        message: str | None = None,
    ) -> dict[str, Any]:
        selected_source = self.mode if self.mode in SUPPORTED_TARGETING_MODES else None
        source_stamp_ns = self._stamp_ns(source_pose) if source_pose is not None else None
        return {
            "source": "target_selector",
            "status": status,
            "reason": reason,
            "message": message,
            "mode": self.mode,
            "selected_source": selected_source,
            "event_source": source,
            "source_status": status,
            "source_age_ms": source_age_ms,
            "source_timeout_ms": self.source_timeout_ms,
            "published": published,
            "source_stamp_ns": source_stamp_ns,
            "last_selected_stamp_ns": self.last_selected_stamp_ns,
            "markerless_target_pose_topic": self.markerless_target_pose_topic,
            "marker_target_pose_topic": self.marker_target_pose_topic,
            "selected_target_pose_topic": self.selected_target_pose_topic,
            "stamp_unix_s": time.time(),
        }

    def _publish_status(self, payload: dict[str, Any], *, force_trace: bool = False) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)
        self._write_selector_log(payload, force_trace=force_trace)

    def _write_selector_log(self, payload: dict[str, Any], *, force_trace: bool = False) -> None:
        if self.trace_path is None or self.status_latest_path is None:
            return
        try:
            self.trace_path.parent.mkdir(parents=True, exist_ok=True)
            self.status_latest_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            signature = (
                payload.get("status"),
                payload.get("reason"),
                payload.get("selected_source"),
                payload.get("event_source"),
                payload.get("published"),
                payload.get("source_stamp_ns"),
            )
            if force_trace or signature != self.last_status_signature:
                self.last_status_signature = signature
                with self.trace_path.open("a", encoding="utf-8") as trace_file:
                    trace_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
        except Exception as exc:
            self.get_logger().warning(json.dumps({"event": "target_selector_trace_write_failed", "error": repr(exc)}, ensure_ascii=False))

    def _source_age_ms(self, message: PoseStamped) -> float | None:
        stamp_ns = self._stamp_ns(message)
        if stamp_ns is None or stamp_ns <= 0:
            return None
        now_ns = int(self.get_clock().now().nanoseconds)
        return max(0.0, float(now_ns - stamp_ns) / 1_000_000.0)

    @staticmethod
    def _stamp_ns(message: PoseStamped | None) -> int | None:
        if message is None:
            return None
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
