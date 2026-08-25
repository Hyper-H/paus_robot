from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from paus_perception import load_config


DEFAULT_LIVE_TARGET_POSE_TOPIC = "/selected_target_pose_base"
DEFAULT_LOCKED_TARGET_POSE_TOPIC = "/locked_target_pose_base"
DEFAULT_STATUS_TOPIC = "/target_lock_status"
STATUS_TIMER_PERIOD_S = 0.1


class TargetLockNode(Node):
    def __init__(self) -> None:
        super().__init__("target_lock_node")
        bringup_share = Path(get_package_share_directory("paus_bringup"))

        self.declare_parameter("config_path", str(bringup_share / "configs" / "default.yaml"))
        self.declare_parameter("enabled", True)
        self.declare_parameter("live_target_pose_topic", DEFAULT_LIVE_TARGET_POSE_TOPIC)
        self.declare_parameter("locked_target_pose_topic", DEFAULT_LOCKED_TARGET_POSE_TOPIC)
        self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
        self.declare_parameter("collect_duration_s", 0.0)
        self.declare_parameter("min_samples", 0)
        self.declare_parameter("max_sample_age_ms", 0)
        self.declare_parameter("max_sample_jump_mm", 0.0)
        self.declare_parameter("max_position_spread_mm", 0.0)
        self.declare_parameter("locked_publish_rate_hz", 0.0)
        self.declare_parameter("drift_warning_mm", 0.0)
        self.declare_parameter("drift_action", "")
        self.declare_parameter("policy", "")
        self.declare_parameter("stale_after_locked_action", "")
        self.declare_parameter("allow_relock_during_motion", False)
        self.declare_parameter("run_dir", "")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        config = load_config(config_path)
        lock_cfg = config.get("target_lock", {})

        self.enabled = bool(self.get_parameter("enabled").get_parameter_value().bool_value)
        self.live_target_pose_topic = self.get_parameter("live_target_pose_topic").get_parameter_value().string_value
        self.locked_target_pose_topic = self.get_parameter("locked_target_pose_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.collect_duration_s = _param_float(self, "collect_duration_s", lock_cfg, 3.0)
        self.min_samples = int(_param_int(self, "min_samples", lock_cfg, 3))
        self.max_sample_age_ms = int(_param_int(self, "max_sample_age_ms", lock_cfg, 2000))
        self.max_sample_jump_mm = float(_param_float(self, "max_sample_jump_mm", lock_cfg, 60.0))
        self.max_position_spread_mm = float(_param_float(self, "max_position_spread_mm", lock_cfg, 50.0))
        self.locked_publish_rate_hz = float(_param_float(self, "locked_publish_rate_hz", lock_cfg, 5.0))
        self.drift_warning_mm = float(_param_float(self, "drift_warning_mm", lock_cfg, 60.0))
        drift_action_param = self.get_parameter("drift_action").get_parameter_value().string_value.strip()
        self.drift_action = drift_action_param or str(lock_cfg.get("drift_action", "warn_only"))
        policy_param = self.get_parameter("policy").get_parameter_value().string_value.strip()
        self.policy = policy_param or str(lock_cfg.get("policy", "lock_once_episode"))
        stale_action_param = self.get_parameter("stale_after_locked_action").get_parameter_value().string_value.strip()
        self.stale_after_locked_action = stale_action_param or str(lock_cfg.get("stale_after_locked_action", "warn_only"))
        allow_relock_param = self.get_parameter("allow_relock_during_motion").get_parameter_value().bool_value
        self.allow_relock_during_motion = bool(lock_cfg.get("allow_relock_during_motion", allow_relock_param))

        run_dir_param = self.get_parameter("run_dir").get_parameter_value().string_value.strip()
        self.run_dir = Path(run_dir_param) if run_dir_param else None
        self.trace_path = self.run_dir / "target_lock_trace.jsonl" if self.run_dir is not None else None
        self.status_latest_path = self.run_dir / "target_lock_status_latest.json" if self.run_dir is not None else None

        self.samples: list[PoseStamped] = []
        self.rejected_sample_count = 0
        self.dropped_sample_count = 0
        self.collecting_started_ns: int | None = None
        self.locked_pose: PoseStamped | None = None
        self.locked_at_ns: int | None = None
        self.last_live_pose: PoseStamped | None = None
        self.last_reject_reason: str | None = None
        self.last_drop_reason: str | None = None
        self.lock_ready_reason: str | None = None
        self.last_status_signature: tuple[Any, ...] | None = None
        self.last_locked_publish_monotonic = 0.0
        self.state = "disabled" if not self.enabled else "waiting"

        self.live_subscription = self.create_subscription(PoseStamped, self.live_target_pose_topic, self._live_pose_callback, 10)
        self.locked_publisher = self.create_publisher(PoseStamped, self.locked_target_pose_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.status_timer = self.create_timer(STATUS_TIMER_PERIOD_S, self._status_timer_callback)

        self._publish_status("started", force_trace=True)

    def _live_pose_callback(self, message: PoseStamped) -> None:
        self.last_live_pose = deepcopy(message)
        if not self.enabled:
            self._publish_status("disabled_live_ignored", force_trace=True)
            return
        if self.locked_pose is not None:
            drift_mm = self._latest_live_to_locked_mm()
            if drift_mm is not None and drift_mm > self.drift_warning_mm:
                self.state = "drift_warning"
                self._publish_status("target_drift_warning", force_trace=True)
            return
        if self.collecting_started_ns is None:
            self.collecting_started_ns = self._stamp_ns(message) or self._now_ns()
            self.state = "collecting"
        self._prune_sample_window(reference_stamp_ns=self._stamp_ns(message))
        reject_reason = self._sample_reject_reason(message)
        if reject_reason is not None:
            self.rejected_sample_count += 1
            self.last_reject_reason = reject_reason
            self.lock_ready_reason = reject_reason
            self._publish_status("sample_rejected", force_trace=True)
            return
        self.samples.append(deepcopy(message))
        self.last_reject_reason = None
        self._prune_sample_window(reference_stamp_ns=self._stamp_ns(message))
        self._publish_status("sample_accepted", force_trace=True)
        self._try_lock()

    def _status_timer_callback(self) -> None:
        if self.locked_pose is not None:
            self._publish_locked_pose_if_due()
            drift_mm = self._latest_live_to_locked_mm()
            if drift_mm is not None and drift_mm > self.drift_warning_mm:
                self.state = "drift_warning"
            elif self.state == "drift_warning":
                self.state = "locked"
            self._publish_status("heartbeat")
            return
        if not self.enabled:
            self._publish_status("heartbeat")
            return
        self._prune_sample_window()
        if self.samples:
            self._try_lock()
        elif self.collecting_started_ns is not None:
            self.state = "collecting"
            self._publish_status("heartbeat")
        else:
            self.state = "waiting"
            self._publish_status("heartbeat")

    def _sample_reject_reason(self, message: PoseStamped) -> str | None:
        age_ms = self._pose_age_ms(message)
        if age_ms is not None and age_ms > float(self.max_sample_age_ms):
            return "sample_stale"
        if self.samples:
            previous = _position_array(self.samples[-1])
            current = _position_array(message)
            jump_mm = float(np.linalg.norm(current - previous) * 1000.0)
            if jump_mm > self.max_sample_jump_mm:
                return "sample_jump_over_threshold"
        return None

    def _collection_ready(self) -> bool:
        return len(self._recent_samples()) >= self.min_samples

    def _try_lock(self) -> None:
        recent_samples = self._recent_samples()
        if len(recent_samples) < self.min_samples:
            self.lock_ready_reason = "not_enough_recent_samples"
            return
        stats = self._sample_stats(recent_samples)
        if stats.get("position_spread_mm") is not None and float(stats["position_spread_mm"]) > self.max_position_spread_mm:
            self._drop_farthest_sample_from_median("position_spread_over_threshold")
            self.state = "collecting"
            self.last_reject_reason = "position_spread_over_threshold"
            self.lock_ready_reason = "position_spread_over_threshold"
            self._publish_status("lock_waiting_spread", force_trace=True)
            return
        recent_samples = self._recent_samples()
        if len(recent_samples) < self.min_samples:
            self.lock_ready_reason = "not_enough_recent_samples"
            return
        self.lock_ready_reason = "recent_samples_stable"
        self.locked_pose = self._build_locked_pose(recent_samples)
        self.locked_at_ns = self._now_ns()
        self.state = "locked"
        self._publish_locked_pose(force=True)
        self._publish_status("locked", force_trace=True)

    def _prune_sample_window(self, *, reference_stamp_ns: int | None = None) -> None:
        if not self.samples:
            return
        if reference_stamp_ns is None:
            reference_stamp_ns = self._latest_sample_stamp_ns()
        if reference_stamp_ns is None:
            return
        min_stamp_ns = reference_stamp_ns - int(max(self.collect_duration_s, 0.1) * 1_000_000_000)
        kept: list[PoseStamped] = []
        dropped = 0
        for sample in self.samples:
            stamp_ns = self._stamp_ns(sample)
            if stamp_ns is None or stamp_ns <= 0 or stamp_ns >= min_stamp_ns:
                kept.append(sample)
            else:
                dropped += 1
        if dropped:
            self.samples = kept
            self.dropped_sample_count += dropped
            self.last_drop_reason = "sample_outside_collect_window"
            if self.samples:
                first_stamp_ns = self._stamp_ns(self.samples[0])
                self.collecting_started_ns = first_stamp_ns if first_stamp_ns is not None else self._now_ns()
            else:
                self.collecting_started_ns = None

    def _drop_farthest_sample_from_median(self, reason: str) -> None:
        if len(self.samples) <= 1:
            return
        positions = np.asarray([_position_array(sample) for sample in self.samples], dtype=np.float64)
        median = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median, axis=1)
        drop_index = int(np.argmax(distances))
        del self.samples[drop_index]
        self.dropped_sample_count += 1
        self.last_drop_reason = reason
        if self.samples:
            first_stamp_ns = self._stamp_ns(self.samples[0])
            self.collecting_started_ns = first_stamp_ns if first_stamp_ns is not None else self._now_ns()
        else:
            self.collecting_started_ns = None

    def _latest_sample_stamp_ns(self) -> int | None:
        stamps = [self._stamp_ns(sample) for sample in self.samples]
        valid_stamps = [stamp for stamp in stamps if stamp is not None and stamp > 0]
        if not valid_stamps:
            return None
        return max(valid_stamps)

    def _recent_samples(self) -> list[PoseStamped]:
        if self.min_samples <= 0:
            return list(self.samples)
        return list(self.samples[-self.min_samples :])

    def _build_locked_pose(self, samples: list[PoseStamped]) -> PoseStamped:
        output = deepcopy(samples[-1])
        positions = np.asarray([_position_array(sample) for sample in samples], dtype=np.float64)
        median_position = np.median(positions, axis=0)
        output.pose.position.x = float(median_position[0])
        output.pose.position.y = float(median_position[1])
        output.pose.position.z = float(median_position[2])
        quat = average_quaternion_xyzw([_quaternion_array(sample) for sample in samples])
        output.pose.orientation.x = float(quat[0])
        output.pose.orientation.y = float(quat[1])
        output.pose.orientation.z = float(quat[2])
        output.pose.orientation.w = float(quat[3])
        self._stamp_now(output)
        return output

    def _publish_locked_pose_if_due(self) -> None:
        if self.locked_pose is None:
            return
        period_s = 1.0 / max(0.1, self.locked_publish_rate_hz)
        if time.monotonic() - self.last_locked_publish_monotonic >= period_s:
            self._publish_locked_pose(force=False)

    def _publish_locked_pose(self, *, force: bool) -> None:
        if self.locked_pose is None:
            return
        output = deepcopy(self.locked_pose)
        self._stamp_now(output)
        self.locked_publisher.publish(output)
        self.last_locked_publish_monotonic = time.monotonic()
        if force:
            self._write_trace(self._build_status("locked_pose_published"))

    def _build_status(self, event: str) -> dict[str, Any]:
        stats = self._sample_stats()
        recent_stats = self._sample_stats(self._recent_samples())
        live_to_locked_mm = self._latest_live_to_locked_mm()
        return {
            "source": "target_lock",
            "event": event,
            "state": self.state,
            "enabled": self.enabled,
            "sample_count": len(self.samples),
            "accepted_sample_count": len(self.samples),
            "rejected_sample_count": self.rejected_sample_count,
            "dropped_sample_count": self.dropped_sample_count,
            "last_reject_reason": self.last_reject_reason,
            "last_drop_reason": self.last_drop_reason,
            "collect_duration_s": self.collect_duration_s,
            "sample_retention_window_s": self.collect_duration_s,
            "min_samples": self.min_samples,
            "max_sample_age_ms": self.max_sample_age_ms,
            "max_sample_jump_mm": self.max_sample_jump_mm,
            "max_position_spread_mm": self.max_position_spread_mm,
            "drift_warning_mm": self.drift_warning_mm,
            "drift_action": self.drift_action,
            "policy": self.policy,
            "stale_after_locked_action": self.stale_after_locked_action,
            "allow_relock_during_motion": self.allow_relock_during_motion,
            "lock_ready_reason": self.lock_ready_reason,
            "position_spread_mm": stats.get("position_spread_mm"),
            "recent_sample_spread_mm": recent_stats.get("position_spread_mm"),
            "sample_stamp_ns": stats.get("sample_stamp_ns"),
            "recent_sample_stamp_ns": recent_stats.get("sample_stamp_ns"),
            "locked": self.locked_pose is not None,
            "locked_at_ns": self.locked_at_ns,
            "locked_pose_base": _pose_payload(self.locked_pose),
            "latest_live_to_locked_mm": live_to_locked_mm,
            "live_target_pose_topic": self.live_target_pose_topic,
            "locked_target_pose_topic": self.locked_target_pose_topic,
            "stamp_unix_s": time.time(),
        }

    def _publish_status(self, event: str, *, force_trace: bool = False) -> None:
        payload = self._build_status(event)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)
        self._write_status(payload, force_trace=force_trace)

    def _write_status(self, payload: dict[str, Any], *, force_trace: bool = False) -> None:
        if self.status_latest_path is None:
            return
        try:
            self.status_latest_path.parent.mkdir(parents=True, exist_ok=True)
            self.status_latest_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            signature = (
                payload.get("state"),
                payload.get("event"),
                payload.get("sample_count"),
                payload.get("rejected_sample_count"),
                payload.get("dropped_sample_count"),
                payload.get("locked"),
                payload.get("last_reject_reason"),
                payload.get("last_drop_reason"),
                payload.get("lock_ready_reason"),
            )
            if force_trace or signature != self.last_status_signature:
                self.last_status_signature = signature
                self._write_trace(payload)
        except Exception as exc:
            self.get_logger().warning(json.dumps({"event": "target_lock_trace_write_failed", "error": repr(exc)}, ensure_ascii=False))

    def _write_trace(self, payload: dict[str, Any]) -> None:
        if self.trace_path is None:
            return
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        with self.trace_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def _sample_stats(self, samples: list[PoseStamped] | None = None) -> dict[str, Any]:
        target_samples = self.samples if samples is None else samples
        if not target_samples:
            return {"position_spread_mm": None, "sample_stamp_ns": []}
        positions = np.asarray([_position_array(sample) for sample in target_samples], dtype=np.float64)
        median = np.median(positions, axis=0)
        distances_mm = np.linalg.norm(positions - median, axis=1) * 1000.0
        return {
            "position_spread_mm": float(np.max(distances_mm)) if distances_mm.size else None,
            "sample_stamp_ns": [self._stamp_ns(sample) for sample in target_samples],
        }

    def _latest_live_to_locked_mm(self) -> float | None:
        if self.last_live_pose is None or self.locked_pose is None:
            return None
        return float(np.linalg.norm(_position_array(self.last_live_pose) - _position_array(self.locked_pose)) * 1000.0)

    def _pose_age_ms(self, message: PoseStamped) -> float | None:
        stamp_ns = self._stamp_ns(message)
        if stamp_ns is None or stamp_ns <= 0:
            return None
        return max(0.0, float(self._now_ns() - stamp_ns) / 1_000_000.0)

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _stamp_now(self, message: PoseStamped) -> None:
        now = self.get_clock().now().to_msg()
        message.header.stamp.sec = int(now.sec)
        message.header.stamp.nanosec = int(now.nanosec)

    @staticmethod
    def _stamp_ns(message: PoseStamped | None) -> int | None:
        if message is None:
            return None
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def average_quaternion_xyzw(quaternions: list[np.ndarray]) -> np.ndarray:
    if not quaternions:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    base = np.asarray(quaternions[0], dtype=np.float64)
    output = np.zeros(4, dtype=np.float64)
    for quat in quaternions:
        item = np.asarray(quat, dtype=np.float64)
        if np.dot(item, base) < 0:
            item = -item
        output += item
    norm = float(np.linalg.norm(output))
    if norm <= 1e-9:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return output / norm


def _position_array(message: PoseStamped) -> np.ndarray:
    return np.asarray([message.pose.position.x, message.pose.position.y, message.pose.position.z], dtype=np.float64)


def _quaternion_array(message: PoseStamped) -> np.ndarray:
    return np.asarray(
        [
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ],
        dtype=np.float64,
    )


def _pose_payload(message: PoseStamped | None) -> dict[str, Any] | None:
    if message is None:
        return None
    return {
        "frame_id": message.header.frame_id,
        "stamp_ns": TargetLockNode._stamp_ns(message),
        "position_m": [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ],
        "orientation_xyzw": [
            float(message.pose.orientation.x),
            float(message.pose.orientation.y),
            float(message.pose.orientation.z),
            float(message.pose.orientation.w),
        ],
    }


def _param_float(node: Node, name: str, cfg: dict[str, Any], default: float) -> float:
    value = float(node.get_parameter(name).get_parameter_value().double_value)
    return value if value > 0.0 else float(cfg.get(name, default))


def _param_int(node: Node, name: str, cfg: dict[str, Any], default: int) -> int:
    value = int(node.get_parameter(name).get_parameter_value().integer_value)
    return value if value > 0 else int(cfg.get(name, default))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetLockNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
