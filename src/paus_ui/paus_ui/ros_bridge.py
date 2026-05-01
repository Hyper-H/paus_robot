from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
from typing import Any

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from paus_perception import load_camera_calibration, load_config

from .operator_messages import classify_operator_message
from .overlay import BoardOverlayDetector, encode_jpeg, make_placeholder_image
from .path_resolvers import resolve_ui_calibration_paths
from .session_store import SessionStore


STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CAMERA_FRESHNESS_S = 3.0


@dataclass
class CachedImage:
    image_bgr: np.ndarray
    sequence: int
    header_time_s: float | None
    received_time_s: float


class EventBuffer:
    def __init__(self) -> None:
        self._events: deque[dict[str, Any]] = deque(maxlen=500)
        self._lock = threading.Lock()
        self._next_id = 0

    def push(self, event: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._next_id += 1
            payload = {"id": self._next_id, "time_s": time.time(), **event}
            self._events.append(payload)
            return payload

    def since(self, last_id: int) -> tuple[list[dict[str, Any]], int]:
        with self._lock:
            events = [event for event in self._events if int(event["id"]) > int(last_id)]
            next_id = int(events[-1]["id"]) if events else int(last_id)
        return events, next_id


class UiRosBridge(Node):
    def __init__(self) -> None:
        super().__init__("paus_ui_server")
        bringup_share = Path(get_package_share_directory("paus_bringup"))
        default_config_path = self._resolve_default_config_path(bringup_share)

        self.declare_parameter("ui_host", "0.0.0.0")
        self.declare_parameter("ui_port", 8080)
        self.declare_parameter("config_path", str(default_config_path))
        self.declare_parameter("camera_config_path", "/tmp/paus_robot/camera.yaml")
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("status_topic", "/eye_to_hand/status")

        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.config = load_config(self.config_path)
        calibration_cfg = self.config["calibration"]
        control_cfg = self.config["control"]

        self.declare_parameter("board_rows", int(calibration_cfg.get("board_rows", 6)))
        self.declare_parameter("board_cols", int(calibration_cfg.get("board_cols", 9)))
        self.declare_parameter("square_size_m", float(calibration_cfg.get("square_size_m", 0.01)))
        self.declare_parameter("trajectory_path", str(calibration_cfg.get("trajectory_path", bringup_share / "configs" / "eye_to_hand_trajectory.yaml")))
        self.declare_parameter("session_root_path", str(calibration_cfg.get("session_root_path", "calibration_sessions")))
        self.declare_parameter("max_reprojection_error_px", float(calibration_cfg.get("max_reprojection_error_px", 0.0)))
        self.declare_parameter("min_board_margin_px", float(calibration_cfg.get("min_board_margin_px", 10.0)))
        self.declare_parameter("execute_motion", bool(control_cfg.get("execute_motion", False)))

        self.ui_host = self.get_parameter("ui_host").get_parameter_value().string_value
        self.ui_port = int(self.get_parameter("ui_port").get_parameter_value().integer_value)
        self.camera_config_path = Path(self.get_parameter("camera_config_path").get_parameter_value().string_value)
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.board_rows = int(self.get_parameter("board_rows").get_parameter_value().integer_value)
        self.board_cols = int(self.get_parameter("board_cols").get_parameter_value().integer_value)
        self.square_size_m = float(self.get_parameter("square_size_m").get_parameter_value().double_value)
        self.trajectory_path, self.session_root_path = resolve_ui_calibration_paths(
            self.config_path,
            self.get_parameter("trajectory_path").get_parameter_value().string_value,
            self.get_parameter("session_root_path").get_parameter_value().string_value,
        )
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").get_parameter_value().double_value)
        self.min_board_margin_px = float(self.get_parameter("min_board_margin_px").get_parameter_value().double_value)
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)
        self.effective_trajectory_path = self.trajectory_path
        self.effective_session_root_path = self.session_root_path
        self.effective_max_reprojection_error_px = self.max_reprojection_error_px
        self.effective_min_board_margin_px = self.min_board_margin_px
        self.effective_execute_motion = self.execute_motion

        self.bridge = CvBridge()
        self.events = EventBuffer()
        self.session_store = SessionStore(
            session_root_path=self.effective_session_root_path,
            trajectory_path=self.effective_trajectory_path,
            max_reprojection_error_px=self.effective_max_reprojection_error_px,
            min_board_margin_px=self.effective_min_board_margin_px,
        )

        self._image_lock = threading.Lock()
        self._latest_image: CachedImage | None = None
        self._image_sequence = 0
        self._last_status_lock = threading.Lock()
        self._last_status: dict[str, Any] | None = None
        self._last_status_time_s: float | None = None
        self._detector: BoardOverlayDetector | None = None
        self._detector_mtime_ns: int | None = None
        self._detector_signature: tuple[str, int, int, int, float] | None = None
        self._run_lock = threading.Lock()
        self._run_thread: threading.Thread | None = None
        self._last_command_result: dict[str, Any] | None = None

        self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.create_subscription(String, self.status_topic, self._status_callback, STATUS_QOS)
        self._service_clients = {
            "record_waypoint": self.create_client(Trigger, "/eye_to_hand/record_waypoint"),
            "delete_last_waypoint": self.create_client(Trigger, "/eye_to_hand/delete_last_waypoint"),
            "save_trajectory": self.create_client(Trigger, "/eye_to_hand/save_trajectory"),
            "run_semi_auto": self.create_client(Trigger, "/eye_to_hand/run_semi_auto_calibration"),
        }
        self.events.push({"type": "ui_started", "message": "PAUS UI server started.", "operator_message": "UI 服务已启动。"})

    def _resolve_default_config_path(self, bringup_share: Path) -> Path:
        workspace_root = bringup_share.parents[3]
        source_config = workspace_root / "src" / "paus_bringup" / "configs" / "default.yaml"
        return source_config if source_config.exists() else bringup_share / "configs" / "default.yaml"

    def _image_callback(self, message: Image) -> None:
        image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        header_time_s = None
        if message.header.stamp.sec != 0 or message.header.stamp.nanosec != 0:
            header_time_s = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        with self._image_lock:
            self._image_sequence += 1
            self._latest_image = CachedImage(
                image_bgr=image_bgr,
                sequence=self._image_sequence,
                header_time_s=header_time_s,
                received_time_s=time.monotonic(),
            )

    def _status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"status": "invalid_json", "message": message.data}
        if not isinstance(payload, dict):
            payload = {"status": "invalid_status", "message": str(payload)}
        shaped = self._shape_status_payload(payload)
        with self._last_status_lock:
            self._last_status = payload
            self._last_status_time_s = time.monotonic()
        self.events.push(
            {
                "type": "eye_to_hand_status",
                "payload": payload,
                "status": shaped["workflow"]["status"],
                "operator_message": shaped["operator_message"],
                "dedupe_key": self._event_dedupe_key("eye_to_hand_status", payload),
            }
        )

    def _sync_backend_state(self, last_status: dict[str, Any] | None = None, last_status_age_s: float | None = None) -> dict[str, Any]:
        if last_status is None and last_status_age_s is None:
            with self._last_status_lock:
                last_status = dict(self._last_status) if self._last_status else None
                last_status_age_s = (time.monotonic() - self._last_status_time_s) if self._last_status_time_s else None
        services_ready = any(client.service_is_ready() for client in self._service_clients.values())
        status_recent = last_status is not None and (last_status_age_s is None or last_status_age_s < 10.0)
        backend_connected = services_ready or status_recent
        payload = last_status if backend_connected and last_status is not None else {}
        trajectory_path = Path(str(payload.get("trajectory_path") or self.trajectory_path))
        session_root_path = Path(str(payload.get("session_root_path") or self.session_root_path))
        max_reprojection_error_px = _to_float(payload.get("max_reprojection_error_px"))
        min_board_margin_px = _to_float(payload.get("min_board_margin_px"))
        board_rows = _to_int(payload.get("board_rows"))
        board_cols = _to_int(payload.get("board_cols"))
        square_size_m = _to_float(payload.get("square_size_m"))
        camera_config_path = payload.get("camera_config_path")
        if not hasattr(self, "board_rows"):
            self.board_rows = 6
        if not hasattr(self, "board_cols"):
            self.board_cols = 9
        if not hasattr(self, "square_size_m"):
            self.square_size_m = 0.01
        if not hasattr(self, "camera_config_path"):
            self.camera_config_path = Path("")
        detector_settings_changed = False
        if board_rows is not None and board_rows != self.board_rows:
            self.board_rows = board_rows
            detector_settings_changed = True
        if board_cols is not None and board_cols != self.board_cols:
            self.board_cols = board_cols
            detector_settings_changed = True
        if square_size_m is not None and square_size_m != self.square_size_m:
            self.square_size_m = square_size_m
            detector_settings_changed = True
        if camera_config_path:
            backend_camera_config_path = Path(str(camera_config_path))
            if backend_camera_config_path != self.camera_config_path:
                self.camera_config_path = backend_camera_config_path
                detector_settings_changed = True
        if detector_settings_changed:
            self._detector = None
            self._detector_mtime_ns = None
            self._detector_signature = None
        self.effective_trajectory_path = trajectory_path
        self.effective_session_root_path = session_root_path
        self.effective_max_reprojection_error_px = self.max_reprojection_error_px if max_reprojection_error_px is None else max_reprojection_error_px
        self.effective_min_board_margin_px = self.min_board_margin_px if min_board_margin_px is None else min_board_margin_px
        motion_state_known = bool(backend_connected and last_status is not None)
        if motion_state_known:
            self.effective_execute_motion = _to_bool(payload.get("execute_motion"), self.execute_motion)
        elif services_ready:
            self.effective_execute_motion = True
        else:
            self.effective_execute_motion = self.execute_motion
        if (
            self.session_store.trajectory_path != self.effective_trajectory_path
            or self.session_store.session_root_path != self.effective_session_root_path
            or self.session_store.max_reprojection_error_px != self.effective_max_reprojection_error_px
            or self.session_store.min_board_margin_px != self.effective_min_board_margin_px
        ):
            self.session_store = SessionStore(
                session_root_path=self.effective_session_root_path,
                trajectory_path=self.effective_trajectory_path,
                max_reprojection_error_px=self.effective_max_reprojection_error_px,
                min_board_margin_px=self.effective_min_board_margin_px,
            )
        return {
            "backend_connected": backend_connected,
            "execute_motion": self.effective_execute_motion,
            "trajectory_path": self.effective_trajectory_path,
            "session_root_path": self.effective_session_root_path,
            "max_reprojection_error_px": self.effective_max_reprojection_error_px,
            "min_board_margin_px": self.effective_min_board_margin_px,
            "board_rows": self.board_rows,
            "board_cols": self.board_cols,
            "square_size_m": self.square_size_m,
            "camera_config_path": self.camera_config_path,
            "config_source": "backend_status" if backend_connected and last_status is not None else ("backend_service_ready" if services_ready else "ui_local_fallback"),
            "services_ready": services_ready,
            "status_recent": status_recent,
            "motion_state_known": motion_state_known,
        }

    def get_status(self) -> dict[str, Any]:
        with self._image_lock:
            latest_image = self._latest_image
        with self._last_status_lock:
            last_status = dict(self._last_status) if self._last_status else None
            last_status_age_s = (time.monotonic() - self._last_status_time_s) if self._last_status_time_s else None
        camera_age_s = self._image_age_s(latest_image)
        backend_state = self._sync_backend_state(last_status, last_status_age_s)
        live_status = last_status if backend_state["backend_connected"] and last_status is not None else None
        shaped_status = self._shape_status_payload(live_status)
        current_waypoint = dict(shaped_status["current_waypoint"])
        status_payload = live_status or {}
        current_waypoint.update(
            {
                "camera_to_board_translation_m": status_payload.get("camera_to_board_translation_m"),
                "camera_to_board_rotation_rpy_deg": status_payload.get("camera_to_board_rotation_rpy_deg"),
                "board_angle_deg": status_payload.get("board_angle_deg"),
                "quality_detected": status_payload.get("quality_detected", status_payload.get("detected")),
                "quality_reason_code": status_payload.get("quality_reason_code", status_payload.get("reason_code")),
                "empty_reason": status_payload.get("empty_reason"),
                "image_sequence": status_payload.get("image_sequence"),
            }
        )
        latest_session = self.session_store.latest_valid_session_id()
        return {
            "ui": {
                "host": self.ui_host,
                "port": self.ui_port,
                "url": f"http://{self.ui_host}:{self.ui_port}",
            },
            "camera": {
                "connected": camera_age_s is not None and camera_age_s < CAMERA_FRESHNESS_S,
                "image_sequence": latest_image.sequence if latest_image else 0,
                "age_s": camera_age_s,
                "topic": self.image_topic,
                "camera_config_path": str(self.camera_config_path),
                "camera_config_exists": self.camera_config_path.exists(),
            },
            "handeye": {
                "status_topic": self.status_topic,
                "last_status": live_status,
                "last_status_age_s": last_status_age_s if live_status is not None else None,
                "calibration_node_connected": backend_state["backend_connected"],
                "execute_motion": backend_state["execute_motion"],
                "requires_motion_confirmation": backend_state["execute_motion"] or not backend_state["backend_connected"] or not backend_state["motion_state_known"],
                "motion_state_known": backend_state["motion_state_known"],
                "backend_config_source": backend_state["config_source"],
                "trajectory_path": str(backend_state["trajectory_path"]),
                "trajectory_dirty": status_payload.get("trajectory_dirty"),
                "session_root_path": str(backend_state["session_root_path"]),
                "max_reprojection_error_px": backend_state["max_reprojection_error_px"],
                "min_board_margin_px": backend_state["min_board_margin_px"],
                "last_command_result": self._last_command_result,
                "run_active": bool(self._run_thread and self._run_thread.is_alive()),
                "workflow": shaped_status["workflow"],
                "current_waypoint": current_waypoint,
                "session": shaped_status["session"] | {"latest_valid_session_id": latest_session},
                "motion": self._motion_summary(),
                "stop": self._stop_status(),
                "operator_message": shaped_status["operator_message"],
            },
            "config": {
                "config_path": self.config_path,
                "board_rows": self.board_rows,
                "board_cols": self.board_cols,
                "square_size_m": self.square_size_m,
            },
        }

    def get_latest_quality(self) -> dict[str, Any]:
        self._sync_backend_state()
        latest_image = self._latest_image_copy()
        if latest_image is None:
            return {
                "detected": False,
                "reason": "No image received.",
                "reason_code": "no_image",
                "operator_message": "尚未收到相机图像。",
                "image_sequence": 0,
                "thresholds": self._quality_flags(None, None),
            }
        image_age_s = self._image_age_s(latest_image)
        if image_age_s is None or image_age_s >= CAMERA_FRESHNESS_S:
            return {
                "detected": False,
                "reason": "Latest image is stale.",
                "reason_code": "stale_image",
                "operator_message": "Camera image is stale; check the camera stream.",
                "image_sequence": latest_image.sequence,
                "image_age_s": image_age_s,
                "thresholds": self._quality_flags(None, None),
            }
        detector = self._get_detector()
        if detector is None:
            return {
                "detected": False,
                "reason": f"Camera calibration is unavailable: {self.camera_config_path}",
                "reason_code": "camera_calibration_missing",
                "operator_message": "相机内参不可用，请检查 camera.yaml 是否存在。",
                "image_sequence": latest_image.sequence,
                "thresholds": self._quality_flags(None, None),
            }
        result = detector.estimate(latest_image.image_bgr)
        payload = result.to_payload(image_sequence=latest_image.sequence)
        payload["thresholds"] = self._quality_flags(result.reprojection_error_px, result.board_margin_px)
        return payload

    def get_latest_jpeg(self, *, mode: str = "overlay", show_axes: bool = True) -> bytes:
        self._sync_backend_state()
        latest_image = self._latest_image_copy()
        if latest_image is None:
            return encode_jpeg(make_placeholder_image("Waiting for /camera/image_bridge"))
        image_age_s = self._image_age_s(latest_image)
        if image_age_s is None or image_age_s >= CAMERA_FRESHNESS_S:
            return encode_jpeg(make_placeholder_image("Camera stream is stale"))
        if mode == "raw":
            return encode_jpeg(latest_image.image_bgr)
        detector = self._get_detector()
        if detector is None:
            image = latest_image.image_bgr.copy()
            cv2.putText(
                image,
                f"Camera calibration unavailable: {self.camera_config_path}",
                (36, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 96, 255),
                2,
                cv2.LINE_AA,
            )
            return encode_jpeg(image)
        rendered, _ = detector.render(latest_image.image_bgr, mode=mode, image_sequence=latest_image.sequence, show_axes=show_axes)
        return encode_jpeg(rendered)

    def get_sample_jpeg(self, *, session_id: str, row_index: int, mode: str = "overlay", show_axes: bool = True) -> bytes:
        self._sync_backend_state()
        sample = self.session_store.sample_for_row(session_id, row_index)
        if sample is None:
            raise FileNotFoundError(f"Sample row {row_index} was not found in session {session_id}.")
        image_path_value = sample.get("image_path")
        image_path = Path(str(image_path_value)) if image_path_value else None
        if image_path is None or not image_path.exists():
            raise FileNotFoundError(f"Sample image is not available for row {row_index} in session {session_id}.")
        image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise ValueError(f"Failed to read sample image: {image_path}")
        if mode == "raw":
            return encode_jpeg(image_bgr)
        rendered = self._render_archived_sample_overlay(image_bgr, sample, mode=mode, row_index=row_index, show_axes=show_axes)
        return encode_jpeg(rendered)

    def record_waypoint(self) -> dict[str, Any]:
        return self._call_trigger("record_waypoint", timeout_s=15.0)

    def delete_last_waypoint(self) -> dict[str, Any]:
        return self._call_trigger("delete_last_waypoint", timeout_s=5.0)

    def save_trajectory(self) -> dict[str, Any]:
        return self._call_trigger("save_trajectory", timeout_s=5.0)

    def get_waypoints(self) -> dict[str, Any]:
        backend_state = self._sync_backend_state()
        with self._last_status_lock:
            last_status = dict(self._last_status) if self._last_status else None
        recorded_trajectory = last_status.get("recorded_trajectory") if backend_state["backend_connected"] and isinstance(last_status, dict) else None
        if isinstance(recorded_trajectory, dict):
            return {
                **recorded_trajectory,
                "trajectory_path": str(backend_state["trajectory_path"]),
                "error": None,
                "source": "recorded_trajectory",
                "dirty": bool(last_status.get("trajectory_dirty")),
            }
        return self.session_store.read_waypoints()

    def list_sessions(self) -> list[dict[str, Any]]:
        self._sync_backend_state()
        return self.session_store.list_sessions()

    def latest_session(self) -> dict[str, Any]:
        self._sync_backend_state()
        return {"session_id": self.session_store.latest_valid_session_id()}

    def read_session_report(self, session_id: str) -> dict[str, Any]:
        self._sync_backend_state()
        return self.session_store.read_report(session_id)

    def read_session_samples(self, session_id: str) -> list[dict[str, Any]]:
        self._sync_backend_state()
        return self.session_store.read_samples(session_id)

    def read_session_waypoints(self, session_id: str) -> list[dict[str, Any]]:
        self._sync_backend_state()
        return self.session_store.read_session_waypoints(session_id)

    def start_semi_auto_run(self, *, confirmed: bool = False) -> dict[str, Any]:
        backend_state = self._sync_backend_state()
        confirmation = self._motion_summary()
        requires_confirmation = (
            bool(backend_state["execute_motion"])
            or not bool(backend_state["backend_connected"])
            or not bool(backend_state["motion_state_known"])
        )
        if requires_confirmation and not confirmed:
            result = self._shape_command_result(
                "run_semi_auto",
                False,
                "Calibration node motion state requires UI confirmation before starting.",
                extra={"requires_confirmation": True, "confirmation": confirmation},
            )
            self._last_command_result = result
            return result
        with self._run_lock:
            if self._run_thread is not None and self._run_thread.is_alive():
                result = self._shape_command_result("run_semi_auto", False, "Semi-auto calibration is already running.")
                self._last_command_result = result
                return result
            client = self._service_clients["run_semi_auto"]
            if not client.wait_for_service(timeout_sec=2.0):
                result = self._shape_command_result("run_semi_auto", False, f"Service is unavailable: {client.srv_name}")
                self._last_command_result = result
                return result
            self._run_thread = threading.Thread(target=self._run_semi_auto_worker, daemon=True)
            self._run_thread.start()
        result = self._shape_command_result(
            "run_semi_auto",
            False,
            "Semi-auto calibration request queued.",
            extra={"accepted": True, "queued": True, "confirmation": confirmation},
        )
        result["operator_message"] = "半自动标定请求已排队，等待标定节点确认。"
        self._last_command_result = result
        return result

    def stop_run(self) -> dict[str, Any]:
        result = self._shape_command_result(
            "stop",
            False,
            "Stop is not supported by the current calibration node. Use terminal interrupt or the physical emergency stop if motion must stop immediately.",
            extra={"stop_supported": False},
        )
        self._last_command_result = result
        return result

    def get_events_since(self, last_id: int) -> tuple[list[dict[str, Any]], int]:
        return self.events.since(last_id)

    def _run_semi_auto_worker(self) -> None:
        self.events.push({"type": "ui_command", "command": "run_semi_auto", "message": "Semi-auto service call started.", "operator_message": "半自动标定请求已发送。"})
        self._call_trigger("run_semi_auto", timeout_s=3600.0)

    def _call_trigger(self, key: str, *, timeout_s: float) -> dict[str, Any]:
        client = self._service_clients[key]
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            result = self._shape_command_result(key, False, f"Service is unavailable: {client.srv_name}")
            self._last_command_result = result
            self.events.push({"type": "ui_command_result", "command": key, "result": result, "operator_message": result["operator_message"], "dedupe_key": self._event_dedupe_key("ui_command_result", result)})
            return result
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_s):
            result = self._shape_command_result(key, False, f"Service timed out: {client.srv_name}")
            self._last_command_result = result
            self.events.push({"type": "ui_command_result", "command": key, "result": result, "operator_message": result["operator_message"], "dedupe_key": self._event_dedupe_key("ui_command_result", result)})
            return result
        try:
            response = future.result()
        except Exception as exc:
            result = self._shape_command_result(key, False, repr(exc))
        else:
            result = self._shape_command_result(key, bool(response.success), str(response.message))
        self._last_command_result = result
        self.events.push({"type": "ui_command_result", "command": key, "result": result, "operator_message": result["operator_message"], "dedupe_key": self._event_dedupe_key("ui_command_result", result)})
        return result

    def _latest_image_copy(self) -> CachedImage | None:
        with self._image_lock:
            if self._latest_image is None:
                return None
            latest = self._latest_image
            return CachedImage(
                image_bgr=latest.image_bgr.copy(),
                sequence=latest.sequence,
                header_time_s=latest.header_time_s,
                received_time_s=latest.received_time_s,
            )

    def _image_age_s(self, image: CachedImage | None) -> float | None:
        return (time.monotonic() - image.received_time_s) if image is not None else None

    def _get_detector(self) -> BoardOverlayDetector | None:
        if not self.camera_config_path.exists():
            self._detector = None
            self._detector_mtime_ns = None
            self._detector_signature = None
            return None
        try:
            mtime_ns = self.camera_config_path.stat().st_mtime_ns
        except OSError:
            self._detector = None
            self._detector_mtime_ns = None
            self._detector_signature = None
            return None
        board_rows = int(getattr(self, "board_rows", 6))
        board_cols = int(getattr(self, "board_cols", 9))
        square_size_m = float(getattr(self, "square_size_m", 0.01))
        signature = (str(self.camera_config_path), mtime_ns, board_rows, board_cols, square_size_m)
        if self._detector is not None and self._detector_mtime_ns == mtime_ns:
            if getattr(self, "_detector_signature", None) == signature:
                return self._detector
        try:
            calibration = load_camera_calibration(self.camera_config_path)
        except Exception:
            self._detector = None
            self._detector_mtime_ns = None
            self._detector_signature = None
            return None
        self._detector = BoardOverlayDetector(
            board_rows=board_rows,
            board_cols=board_cols,
            square_size_m=square_size_m,
            camera_calibration=calibration,
        )
        self._detector_mtime_ns = mtime_ns
        self._detector_signature = signature
        return self._detector

    def _shape_status_payload(self, payload: dict[str, Any] | None) -> dict[str, Any]:
        payload = payload or {}
        status = str(payload.get("status", "idle"))
        info = classify_operator_message(payload.get("message", ""))
        waypoint = payload.get("waypoint") if isinstance(payload.get("waypoint"), dict) else {}
        waypoint_name = payload.get("waypoint_name") or waypoint.get("name")
        workflow = self._workflow_for_status(status)
        session_dir = payload.get("session_dir")
        return {
            "workflow": {
                "status": status,
                "stage": workflow["stage"],
                "label": workflow["label"],
                "progress_index": payload.get("waypoint_index"),
                "progress_count": payload.get("waypoint_count"),
                "sample_count": payload.get("sample_count"),
                "min_sample_count": payload.get("min_sample_count"),
            },
            "current_waypoint": {
                "name": waypoint_name,
                "index": payload.get("waypoint_index"),
                "count": payload.get("waypoint_count"),
                "motion": waypoint.get("motion"),
                "vel": waypoint.get("vel"),
                "acc": waypoint.get("acc"),
                "dwell_s": waypoint.get("dwell_s"),
                "stable_tcp_pose_mmdeg": payload.get("stable_tcp_pose_mmdeg"),
                "tcp_pose_mmdeg": payload.get("tcp_pose_mmdeg"),
            },
            "session": {
                "session_dir": session_dir,
                "session_id": Path(str(session_dir)).name if session_dir else None,
                "report_path": payload.get("report_path"),
            },
            "operator_message": info["message"] if payload.get("message") else workflow["label"],
            "message_code": info["code"],
        }

    def _workflow_for_status(self, status: str) -> dict[str, str]:
        mapping = {
            "idle": ("idle", "等待状态"),
            "ready": ("idle", "节点就绪"),
            "waypoint_recorded": ("recorded", "已记录当前点"),
            "record_waypoint_failed": ("error", "记录点失败"),
            "waypoint_deleted": ("recorded", "已删除上一个点"),
            "semi_auto_dry_run_waypoint": ("dry_run", "Dry-run 检查中"),
            "waypoint_dry_run_complete": ("dry_run", "Dry-run"),
            "semi_auto_dry_run_complete": ("finished", "Dry-run 完成"),
            "waypoint_motion_started": ("movej", "MoveJ 运动中"),
            "waypoint_waiting_stable": ("wait_stable", "等待机械臂稳定"),
            "waypoint_reached": ("reached", "已到达采样点"),
            "waypoint_capture_started": ("capture", "正在采集图像"),
            "waypoint_detection_started": ("detect", "正在检测棋盘"),
            "waypoint_chessboard_detected": ("detect", "棋盘已检测到"),
            "waypoint_pose_estimated": ("detect", "棋盘位姿已估计"),
            "waypoint_capture_skipped": ("skipped", "该点已跳过"),
            "waypoint_sample_captured": ("accepted", "样本已接受"),
            "semi_auto_insufficient_samples": ("error", "有效样本不足"),
            "solving": ("solve", "正在求解手眼标定"),
            "solved": ("solve", "标定已求解"),
            "semi_auto_finished": ("finished", "半自动标定完成"),
            "semi_auto_finished_with_skips": ("finished", "半自动标定完成，存在跳过点"),
            "semi_auto_failed": ("error", "半自动标定失败"),
        }
        stage, label = mapping.get(status, ("idle", status or "等待状态"))
        return {"stage": stage, "label": label}

    def _motion_summary(self) -> dict[str, Any]:
        trajectory = self.session_store.read_waypoints()
        waypoints = trajectory.get("waypoints", []) if isinstance(trajectory.get("waypoints"), list) else []
        defaults = trajectory.get("defaults", {}) if isinstance(trajectory.get("defaults"), dict) else {}
        motions = {str(item.get("motion", defaults.get("motion", "movej"))).lower() for item in waypoints if isinstance(item, dict)}
        return {
            "execute_motion": self.effective_execute_motion,
            "waypoint_count": len(waypoints),
            "motion": ",".join(sorted(motions)) if motions else str(defaults.get("motion", "movej")),
            "vel": defaults.get("vel"),
            "acc": defaults.get("acc"),
            "dwell_s": defaults.get("dwell_s"),
            "trajectory_path": str(self.effective_trajectory_path),
            "trajectory_error": trajectory.get("error"),
        }

    def _stop_status(self) -> dict[str, Any]:
        return {
            "supported": False,
            "operator_message": "当前停止按钮只能给出提示；如需立即停止真实运动，请使用实体急停或终端中断。",
        }

    def _render_archived_sample_overlay(
        self,
        image_bgr: np.ndarray,
        sample: dict[str, Any],
        *,
        mode: str,
        row_index: int,
        show_axes: bool,
    ) -> np.ndarray:
        output = image_bgr.copy()
        translation = sample.get("camera_to_board_translation_m") or [None, None, None]
        rotation = sample.get("camera_to_board_rotation_rpy_deg") or [None, None, None]
        lines = [
            f"archived_sample: {row_index}",
            f"mode: {mode if mode in {'overlay', 'pose'} else 'overlay'}",
            f"image_sequence: {sample.get('image_sequence', '-')}",
            f"reprojection_error_px: {_format_optional_float(sample.get('reprojection_error_px'), 3)}",
            f"board_margin_px: {_format_optional_float(sample.get('board_margin_px'), 1)}",
            "T_camera_board: "
            f"x={_format_optional_float(translation[0], 3)} "
            f"y={_format_optional_float(translation[1], 3)} "
            f"z={_format_optional_float(translation[2], 3)} m",
        ]
        if mode == "pose":
            lines.append(
                "rpy_deg: "
                f"rx={_format_optional_float(rotation[0], 3)} "
                f"ry={_format_optional_float(rotation[1], 3)} "
                f"rz={_format_optional_float(rotation[2], 3)}"
            )
            lines.append(f"axes: {'metadata only' if show_axes else 'off'}")

        line_height = 24
        panel_width = min(max(620, int(output.shape[1] * 0.45)), output.shape[1] - 20)
        panel_height = 18 + line_height * len(lines)
        panel = output.copy()
        cv2.rectangle(panel, (12, 12), (12 + panel_width, 12 + panel_height), (15, 22, 33), -1)
        cv2.addWeighted(panel, 0.72, output, 0.28, 0.0, output)
        for index, line in enumerate(lines):
            cv2.putText(
                output,
                line[:88],
                (26, 42 + line_height * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (238, 247, 246),
                2,
                cv2.LINE_AA,
            )
        return output

    def _shape_command_result(self, command: str, success: bool, message: str, *, extra: dict[str, Any] | None = None) -> dict[str, Any]:
        info = classify_operator_message(message)
        success_messages = {
            "record_waypoint": "当前 waypoint 已记录。",
            "delete_last_waypoint": "已删除上一个 waypoint。",
        }
        operator_message = info["message"] if not success else success_messages.get(command, message or "操作已完成。")
        result = {
            "command": command,
            "success": bool(success),
            "message": message,
            "operator_message": operator_message,
            "message_code": info["code"],
        }
        if extra:
            result.update(extra)
        return result

    def _quality_flags(self, reprojection_error_px: Any, board_margin_px: Any) -> dict[str, Any]:
        reprojection = _to_float(reprojection_error_px)
        margin = _to_float(board_margin_px)
        reprojection_limit_enabled = self.effective_max_reprojection_error_px > 0.0
        reprojection_ok = None if reprojection is None or not reprojection_limit_enabled else reprojection <= self.effective_max_reprojection_error_px
        margin_ok = None if margin is None else margin >= self.effective_min_board_margin_px
        return {
            "max_reprojection_error_px": self.effective_max_reprojection_error_px,
            "min_board_margin_px": self.effective_min_board_margin_px,
            "reprojection_limit_enabled": reprojection_limit_enabled,
            "reprojection_ok": reprojection_ok,
            "margin_ok": margin_ok,
            "quality_ok": (reprojection_ok is not False) and (margin_ok is not False),
        }

    def _event_dedupe_key(self, event_type: str, payload: dict[str, Any]) -> str:
        status = payload.get("status") or payload.get("command") or payload.get("message") or ""
        waypoint = payload.get("waypoint_name") or (payload.get("waypoint", {}) if isinstance(payload.get("waypoint"), dict) else {}).get("name", "")
        return f"{event_type}:{status}:{waypoint}:{payload.get('sample_count', '')}:{payload.get('waypoint_index', '')}"


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _format_optional_float(value: Any, digits: int) -> str:
    number = _to_float(value)
    return "--" if number is None else f"{number:.{digits}f}"


def _to_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def _to_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    return bool(default)
