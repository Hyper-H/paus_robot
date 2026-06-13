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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from paus_perception import load_camera_calibration, load_config

from .operator_messages import classify_operator_message
from paus_marker_ros2.board_observation import DEPTH_ALIGNED_MODE, RGB_PNP_MODE, SUPPORTED_OBSERVATION_MODES, normalize_observation_mode

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

        self.declare_parameter("observation_mode", str(calibration_cfg.get("observation_mode", RGB_PNP_MODE)))
        self.declare_parameter("depth_topic", str(calibration_cfg.get("depth_topic", "/camera/depth_aligned")))

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
        self.observation_mode = normalize_observation_mode(self.get_parameter("observation_mode").get_parameter_value().string_value)
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.supported_observation_modes = [RGB_PNP_MODE, DEPTH_ALIGNED_MODE]
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
        self.local_camera_config_path = self.camera_config_path
        self.local_board_rows = self.board_rows
        self.local_board_cols = self.board_cols
        self.local_square_size_m = self.square_size_m
        self.local_execute_motion = self.execute_motion
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
        self._raw_jpeg_cache_lock = threading.Lock()
        self._raw_jpeg_cache_sequence: int | None = None
        self._raw_jpeg_cache_bytes: bytes | None = None
        self._overlay_cache: dict[tuple[int, str, bool], bytes] = {}
        self._overlay_cache_lock = threading.Lock()

        self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.create_subscription(String, self.status_topic, self._status_callback, STATUS_QOS)
        self._parameter_clients = {
            "eye_to_hand": self.create_client(SetParameters, "/eye_to_hand_calibration_node/set_parameters"),
        }
        self._service_clients = {
            "record_waypoint": self.create_client(Trigger, "/eye_to_hand/record_waypoint"),
            "delete_last_waypoint": self.create_client(Trigger, "/eye_to_hand/delete_last_waypoint"),
            "delete_selected_waypoint": self.create_client(Trigger, "/eye_to_hand/delete_selected_waypoint"),
            "save_trajectory": self.create_client(Trigger, "/eye_to_hand/save_trajectory"),
            "run_semi_auto": self.create_client(Trigger, "/eye_to_hand/run_semi_auto_calibration"),
            "stop": self.create_client(Trigger, "/eye_to_hand/stop"),
        }
        self._set_parameters_client = self.create_client(SetParameters, "/eye_to_hand_calibration_node/set_parameters")
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
        run_thread = getattr(self, "_run_thread", None)
        run_active = bool(run_thread and run_thread.is_alive())
        status_recent = last_status is not None and (last_status_age_s is None or last_status_age_s < 10.0)
        backend_connected = services_ready or status_recent or run_active
        payload = last_status if (status_recent or run_active) and last_status is not None else {}
        if not hasattr(self, "effective_trajectory_path"):
            self.effective_trajectory_path = self.trajectory_path
        if not hasattr(self, "effective_session_root_path"):
            self.effective_session_root_path = self.session_root_path
        if not hasattr(self, "effective_max_reprojection_error_px"):
            self.effective_max_reprojection_error_px = self.max_reprojection_error_px
        if not hasattr(self, "effective_min_board_margin_px"):
            self.effective_min_board_margin_px = self.min_board_margin_px
        if not hasattr(self, "effective_execute_motion"):
            self.effective_execute_motion = self.execute_motion
        if not hasattr(self, "observation_mode"):
            self.observation_mode = RGB_PNP_MODE
        if not hasattr(self, "depth_topic"):
            self.depth_topic = "/camera/depth_aligned"
        local_camera_config_path = Path(getattr(self, "local_camera_config_path", getattr(self, "camera_config_path", Path(""))))
        local_board_rows = int(getattr(self, "local_board_rows", getattr(self, "board_rows", 6)))
        local_board_cols = int(getattr(self, "local_board_cols", getattr(self, "board_cols", 9)))
        local_square_size_m = float(getattr(self, "local_square_size_m", getattr(self, "square_size_m", 0.01)))
        local_execute_motion = bool(getattr(self, "local_execute_motion", getattr(self, "execute_motion", False)))
        if not hasattr(self, "camera_config_path"):
            self.camera_config_path = local_camera_config_path
        if not hasattr(self, "board_rows"):
            self.board_rows = local_board_rows
        if not hasattr(self, "board_cols"):
            self.board_cols = local_board_cols
        if not hasattr(self, "square_size_m"):
            self.square_size_m = local_square_size_m
        board_rows = _to_int(payload.get("board_rows"))
        board_cols = _to_int(payload.get("board_cols"))
        square_size_m = _to_float(payload.get("square_size_m"))
        camera_config_path = payload.get("camera_config_path")
        detector_settings_changed = False
        if backend_connected:
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
        else:
            if self.board_rows != local_board_rows:
                self.board_rows = local_board_rows
                detector_settings_changed = True
            if self.board_cols != local_board_cols:
                self.board_cols = local_board_cols
                detector_settings_changed = True
            if self.square_size_m != local_square_size_m:
                self.square_size_m = local_square_size_m
                detector_settings_changed = True
            if self.camera_config_path != local_camera_config_path:
                self.camera_config_path = local_camera_config_path
                detector_settings_changed = True
            self.effective_execute_motion = local_execute_motion
        if detector_settings_changed:
            self._detector = None
            self._detector_mtime_ns = None
            self._detector_signature = None
        if backend_connected:
            trajectory_path_value = payload.get("trajectory_path")
            if trajectory_path_value:
                self.effective_trajectory_path = Path(str(trajectory_path_value))
            session_root_path_value = payload.get("session_root_path")
            if session_root_path_value:
                self.effective_session_root_path = Path(str(session_root_path_value))
            max_reprojection_error_px = _to_float(payload.get("max_reprojection_error_px"))
            if max_reprojection_error_px is not None:
                self.effective_max_reprojection_error_px = max_reprojection_error_px
            min_board_margin_px = _to_float(payload.get("min_board_margin_px"))
            if min_board_margin_px is not None:
                self.effective_min_board_margin_px = min_board_margin_px
        else:
            self.effective_trajectory_path = self.trajectory_path
            self.effective_session_root_path = self.session_root_path
            self.effective_max_reprojection_error_px = self.max_reprojection_error_px
            self.effective_min_board_margin_px = self.min_board_margin_px
        motion_state_known = bool((status_recent or run_active) and last_status is not None)
        if status_recent or run_active:
            self.effective_execute_motion = _to_bool(payload.get("execute_motion"), self.effective_execute_motion)
            if payload.get("observation_mode"):
                try:
                    self.observation_mode = normalize_observation_mode(payload.get("observation_mode"))
                except ValueError:
                    pass
            if payload.get("depth_topic"):
                self.depth_topic = str(payload.get("depth_topic"))
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
            "config_source": "backend_status" if (status_recent or run_active) and last_status is not None else ("backend_service_ready" if services_ready else "ui_local_fallback"),
            "services_ready": services_ready,
            "run_active": run_active,
            "status_recent": status_recent,
            "motion_state_known": motion_state_known,
        }

    def get_status(self) -> dict[str, Any]:
        latest_meta = self._latest_image_meta()
        with self._last_status_lock:
            last_status = dict(self._last_status) if self._last_status else None
            last_status_age_s = (time.monotonic() - self._last_status_time_s) if self._last_status_time_s else None
        camera_age_s = self._image_age_s_from_meta(latest_meta)
        backend_state = self._sync_backend_state(last_status, last_status_age_s)
        live_status = last_status if (backend_state.get("status_recent") or backend_state.get("run_active")) and last_status is not None else None
        shaped_status = self._shape_status_payload(live_status)
        current_waypoint = dict(shaped_status["current_waypoint"])
        status_payload = live_status or {}
        reprojection = _to_float(status_payload.get("reprojection_error_px"))
        board_margin = _to_float(status_payload.get("board_margin_px"))
        quality_payload = status_payload.get("sample_quality") or status_payload.get("record_quality") or status_payload.get("quality") or status_payload.get("quality_payload")
        current_waypoint.update(
            {
                "camera_to_board_translation_m": status_payload.get("camera_to_board_translation_m"),
                "camera_to_board_rotation_rpy_deg": status_payload.get("camera_to_board_rotation_rpy_deg"),
                "board_angle_deg": status_payload.get("board_angle_deg"),
                "reprojection_error_px": reprojection,
                "board_margin_px": board_margin,
                "thresholds": self._quality_flags(reprojection, board_margin),
                "quality_detected": status_payload.get("quality_detected", status_payload.get("detected")),
                "quality_reason_code": status_payload.get("quality_reason_code", status_payload.get("reason_code")),
                "empty_reason": status_payload.get("empty_reason"),
                "image_sequence": status_payload.get("image_sequence"),
                "observation_mode": status_payload.get("observation_mode", self.observation_mode),
                "quality_payload": quality_payload,
            }
        )
        latest_session = self.session_store.latest_valid_session_id()
        ui_host = self.ui_host if self.ui_host not in {"0.0.0.0", "::", ""} else "localhost"
        return {
            "ui": {
                "host": self.ui_host,
                "port": self.ui_port,
                "url": f"http://{ui_host}:{self.ui_port}",
            },
            "camera": {
                "connected": camera_age_s is not None and camera_age_s < CAMERA_FRESHNESS_S,
                "image_sequence": latest_meta[0] if latest_meta else 0,
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
                "observation_mode": self.observation_mode,
                "depth_topic": self.depth_topic,
                "supported_observation_modes": self.supported_observation_modes,
                "quality_schema": self._quality_schema(self.observation_mode),
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

    def get_latest_jpeg(self, *, mode: str = "raw", show_axes: bool = True) -> bytes:
        self._sync_backend_state()
        latest_meta = self._latest_image_meta()
        if latest_meta is None:
            return encode_jpeg(make_placeholder_image("Waiting for /camera/image_bridge"))
        sequence, _, received_time_s = latest_meta
        image_age_s = time.monotonic() - received_time_s
        if image_age_s is None or image_age_s >= CAMERA_FRESHNESS_S:
            return encode_jpeg(make_placeholder_image("Camera stream is stale"))
        if mode == "raw":
            with self._raw_jpeg_cache_lock:
                if self._raw_jpeg_cache_sequence == sequence and self._raw_jpeg_cache_bytes is not None:
                    return self._raw_jpeg_cache_bytes
            latest_image = self._latest_image_copy()
            if latest_image is None:
                return encode_jpeg(make_placeholder_image("Waiting for /camera/image_bridge"))
            jpeg_bytes = encode_jpeg(latest_image.image_bgr)
            with self._raw_jpeg_cache_lock:
                self._raw_jpeg_cache_sequence = sequence
                self._raw_jpeg_cache_bytes = jpeg_bytes
            return jpeg_bytes
        latest_image = self._latest_image_copy()
        if latest_image is None:
            return encode_jpeg(make_placeholder_image("Waiting for /camera/image_bridge"))
        cache_key = (latest_image.sequence, mode, show_axes)
        with self._overlay_cache_lock:
            cached = self._overlay_cache.get(cache_key)
            if cached is not None:
                return cached
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
        jpeg_bytes = encode_jpeg(rendered)
        with self._overlay_cache_lock:
            self._overlay_cache = {k: v for k, v in self._overlay_cache.items() if k[0] == latest_image.sequence}
            self._overlay_cache[cache_key] = jpeg_bytes
        return jpeg_bytes

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

    def delete_selected_waypoint(self, waypoint_name: str) -> dict[str, Any]:
        name = str(waypoint_name or "").strip()
        if not name:
            result = self._shape_command_result("delete_selected_waypoint", False, "No waypoint selected.")
            self._last_command_result = result
            return result
        parameter_result = self._set_eye_to_hand_string_parameter("selected_waypoint_name", name, timeout_s=5.0)
        if not parameter_result["success"]:
            result = self._shape_command_result("delete_selected_waypoint", False, str(parameter_result["message"]))
            self._last_command_result = result
            return result
        return self._call_trigger("delete_selected_waypoint", timeout_s=5.0)

    def save_trajectory(self) -> dict[str, Any]:
        return self._call_trigger("save_trajectory", timeout_s=5.0)

    def get_waypoints(self) -> dict[str, Any]:
        backend_state = self._sync_backend_state()
        with self._last_status_lock:
            last_status = dict(self._last_status) if self._last_status else None
        recorded_trajectory = last_status.get("recorded_trajectory") if isinstance(last_status, dict) else None
        if isinstance(recorded_trajectory, dict):
            trajectory_dirty = bool(last_status.get("trajectory_dirty")) if isinstance(last_status, dict) else False
            if backend_state.get("status_recent") or backend_state.get("run_active") or trajectory_dirty:
                waypoints = recorded_trajectory.get("waypoints", [])
                shaped_waypoints = [self._shape_recorded_waypoint(w) for w in (waypoints if isinstance(waypoints, list) else [])]
                return {
                    "waypoints": shaped_waypoints,
                    "defaults": recorded_trajectory.get("defaults", {}),
                    "trajectory_path": str(backend_state["trajectory_path"]),
                    "error": None,
                    "source": "recorded_trajectory",
                    "dirty": trajectory_dirty,
                }
        trajectory = self.session_store.read_waypoints()
        waypoints = trajectory.get("waypoints", [])
        if isinstance(waypoints, list):
            trajectory = dict(trajectory)
            trajectory["waypoints"] = [self._shape_recorded_waypoint(w) for w in waypoints]
            trajectory["source"] = trajectory.get("source", "trajectory_file")
        return trajectory

    def _shape_recorded_waypoint(self, waypoint: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(waypoint, dict):
            return {"name": str(waypoint), "status": "pending", "result": "-"}
        record_quality = waypoint.get("record_quality") if isinstance(waypoint.get("record_quality"), dict) else {}
        reprojection = _to_float(record_quality.get("reprojection_error_px", record_quality.get("reprojection_rms_px")))
        board_margin = _to_float(record_quality.get("board_margin_px"))
        flags = self._quality_flags(reprojection, board_margin)
        accepted = record_quality.get("accepted")
        quality_status = str(record_quality.get("status", "")).strip().lower()
        reject_reason = str(record_quality.get("reject_reason", record_quality.get("reason", "")) or "")
        if accepted is True or quality_status == "accepted":
            status = "accepted"
            result = "OK"
            reason = ""
        elif accepted is False or quality_status == "rejected" or reject_reason:
            status = "skipped"
            result = "FAIL"
            reason = reject_reason or quality_status or "sample_quality_rejected"
        else:
            status = "pending"
            result = "-"
            reason = str(record_quality.get("reason", "") or "")
        reason_info = classify_operator_message(reason) if reason else {"message": ""}
        return {
            "waypoint": waypoint,
            "index": None,
            "name": str(waypoint.get("name", "")),
            "status": status,
            "result": result,
            "capture": waypoint.get("capture", True),
            "reason": reason,
            "reason_display": reason_info.get("message") or reason,
            "reprojection_error_px": reprojection,
            "board_margin_px": board_margin,
            "thresholds": flags,
            "camera_to_board_translation_m": record_quality.get("camera_to_board_translation_m"),
            "camera_to_board_rotation_rpy_deg": record_quality.get("camera_to_board_rotation_rpy_deg"),
            "board_angle_deg": _to_float(record_quality.get("board_angle_deg")),
            "observation_mode": record_quality.get("observation_mode", self.observation_mode),
            "quality_payload": record_quality,
            "has_image": False,
            "sample_row_index": None,
            "thumbnail_url": None,
        }

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

    def load_session_trajectory(self, session_id: str) -> dict[str, Any]:
        self._sync_backend_state()
        result = self.session_store.load_session_trajectory(session_id)
        self.effective_trajectory_path = self.session_store.trajectory_path
        self.trajectory_path = self.session_store.trajectory_path
        result["success"] = True
        result["loaded"] = True
        self._last_command_result = {
            "command": "load_session_trajectory",
            "success": True,
            "message": str(result.get("operator_message", "Session trajectory loaded.")),
            "operator_message": str(result.get("operator_message", "\u5df2\u8f7d\u5165\u5386\u53f2 session \u8f68\u8ff9\u3002")),
        }
        self.events.push(
            {
                "type": "ui_command_result",
                "command": "load_session_trajectory",
                "result": self._last_command_result,
                "operator_message": self._last_command_result["operator_message"],
                "dedupe_key": self._event_dedupe_key("ui_command_result", self._last_command_result),
            }
        )
        return result

    def per_sample_residuals(self, session_id: str) -> list[dict[str, Any]]:
        self._sync_backend_state()
        return self.session_store.per_sample_residuals(session_id)

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

    def set_observation_mode(self, mode: str) -> dict[str, Any]:
        normalized_mode = normalize_observation_mode(mode)
        if normalized_mode == self.observation_mode:
            result = self._shape_command_result(
                "set_observation_mode",
                True,
                f"Observation mode already set to {normalized_mode}.",
                extra={"mode": normalized_mode, "observation_mode": normalized_mode, "depth_topic": self.depth_topic},
            )
            self._last_command_result = result
            return result
        client = self._set_parameters_client
        if not client.wait_for_service(timeout_sec=2.0):
            result = self._shape_command_result("set_observation_mode", False, f"Service is unavailable: {client.srv_name}")
            self._last_command_result = result
            return result
        request = SetParameters.Request()
        request.parameters = [Parameter("observation_mode", Parameter.Type.STRING, normalized_mode).to_parameter_msg()]
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=5.0):
            result = self._shape_command_result("set_observation_mode", False, "Timed out while switching observation mode.")
            self._last_command_result = result
            return result
        try:
            response = future.result()
        except Exception as exc:
            result = self._shape_command_result("set_observation_mode", False, repr(exc))
        else:
            ok = bool(response.results) and all(getattr(item, "successful", False) for item in response.results)
            if ok:
                self.observation_mode = normalized_mode
                result = self._shape_command_result(
                    "set_observation_mode",
                    True,
                    f"Observation mode switched to {normalized_mode}.",
                    extra={"mode": normalized_mode, "observation_mode": normalized_mode, "depth_topic": self.depth_topic},
                )
            else:
                reasons = [getattr(item, 'reason', '') for item in getattr(response, 'results', []) if not getattr(item, 'successful', False)]
                result = self._shape_command_result("set_observation_mode", False, "; ".join(reason for reason in reasons if reason) or "Failed to switch observation mode.")
        self._last_command_result = result
        self.events.push({"type": "ui_command_result", "command": "set_observation_mode", "result": result, "operator_message": result["operator_message"], "dedupe_key": self._event_dedupe_key("ui_command_result", result)})
        return result

    def stop_run(self) -> dict[str, Any]:
        result = self._call_trigger("stop", timeout_s=2.0)
        if result.get("success"):
            result["accepted"] = True
            result["stopping"] = True
            result["operator_message"] = "\u6b63\u5728\u505c\u6b62\u534a\u81ea\u52a8\u6807\u5b9a\uff0c\u5f53\u524d\u52a8\u4f5c\u5b8c\u6210\u540e\u4f1a\u5728\u4e0b\u4e00\u4e2a\u5b89\u5168\u68c0\u67e5\u70b9\u505c\u4e0b\u3002"
        return result

    def get_events_since(self, last_id: int) -> tuple[list[dict[str, Any]], int]:
        return self.events.since(last_id)

    def _run_semi_auto_worker(self) -> None:
        self.events.push({"type": "ui_command", "command": "run_semi_auto", "message": "Semi-auto service call started.", "operator_message": "半自动标定请求已发送。"})
        self._call_trigger("run_semi_auto", timeout_s=3600.0)

    def _set_eye_to_hand_string_parameter(self, name: str, value: str, *, timeout_s: float) -> dict[str, Any]:
        client = self._parameter_clients["eye_to_hand"]
        if not client.wait_for_service(timeout_sec=timeout_s):
            return {"success": False, "message": f"Service is unavailable: {client.srv_name}"}
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value),
            )
        ]
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_s):
            return {"success": False, "message": f"Service timed out: {client.srv_name}"}
        try:
            response = future.result()
        except Exception as exc:
            return {"success": False, "message": repr(exc)}
        for result in response.results:
            if not result.successful:
                return {"success": False, "message": result.reason or f"Failed to set parameter {name}."}
        return {"success": True, "message": "ok"}

    def _call_trigger(self, key: str, *, timeout_s: float) -> dict[str, Any]:
        client = self._service_clients[key]
        if not client.wait_for_service(timeout_sec=timeout_s):
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

    def _latest_image_meta(self) -> tuple[int, float | None, float] | None:
        with self._image_lock:
            if self._latest_image is None:
                return None
            latest = self._latest_image
            return latest.sequence, latest.header_time_s, latest.received_time_s

    def _image_age_s_from_meta(self, meta: tuple[int, float | None, float] | None) -> float | None:
        return (time.monotonic() - meta[2]) if meta is not None else None

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
        quality_payload = payload.get("sample_quality") or payload.get("record_quality") or payload.get("quality") or payload.get("quality_payload")
        return {
            "workflow": {
                "status": status,
                "stage": workflow["stage"],
                "label": workflow["label"],
                "progress_index": payload.get("waypoint_index"),
                "progress_count": payload.get("waypoint_count"),
                "sample_count": payload.get("sample_count"),
                "min_sample_count": payload.get("min_sample_count"),
                "observation_mode": payload.get("observation_mode", self.observation_mode),
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
                "observation_mode": payload.get("observation_mode", self.observation_mode),
                "quality_payload": quality_payload,
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
            "semi_auto_started": ("movej", "半自动标定已启动"),
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
            "waypoint_capture_disabled": ("skipped", "无需采样"),
            "waypoint_capture_skipped": ("skipped", "该点已跳过"),
            "waypoint_sample_captured": ("accepted", "样本已接受"),
            "sample_captured": ("accepted", "样本已接受"),
            "capture_failed": ("error", "采样失败"),
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
        backend_state = self._sync_backend_state()
        with self._last_status_lock:
            last_status = dict(self._last_status) if self._last_status else None
        trajectory = (
            last_status.get("recorded_trajectory")
            if (backend_state.get("status_recent") or backend_state.get("run_active")) and isinstance(last_status, dict)
            else None
        )
        if not isinstance(trajectory, dict):
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

    def _quality_schema(self, observation_mode: str) -> dict[str, Any]:
        mode = normalize_observation_mode(observation_mode)
        if mode == DEPTH_ALIGNED_MODE:
            live_metrics = [
                {"key": "valid_depth_ratio", "label": "\u6df1\u5ea6\u6709\u6548\u7387", "digits": 2},
                {"key": "board_model_fit_rmse_mm", "label": "\u5e73\u9762\u5bf9\u9f50\u8bef\u5dee", "digits": 2, "suffix": " mm"},
                {"key": "plane_residual_std_mm_median", "label": "\u6df1\u5ea6\u6b8b\u5dee", "digits": 2, "suffix": " mm"},
                {"key": "global_plane_point_count", "label": "\u6709\u6548\u70b9\u6570", "digits": 0},
                {"key": "board_angle_deg", "label": "\u68cb\u76d8\u89d2\u5ea6", "digits": 2, "suffix": " deg"},
            ]
        else:
            live_metrics = [
                {"key": "detected", "label": "\u68c0\u6d4b"},
                {"key": "reprojection_error_px", "label": "\u91cd\u6295\u5f71", "digits": 3},
                {"key": "board_margin_px", "label": "\u8fb9\u8ddd", "digits": 1, "suffix": " px"},
                {"path": "camera_to_board_translation_m.2", "label": "Tz", "digits": 3, "suffix": " m"},
                {"key": "board_angle_deg", "label": "\u68cb\u76d8\u89d2\u5ea6", "digits": 2, "suffix": " deg"},
            ]
        return {"mode": mode, "live_metrics": live_metrics}

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
        max_reprojection_error_px = float(getattr(self, "effective_max_reprojection_error_px", getattr(self, "max_reprojection_error_px", 0.0)))
        min_board_margin_px = float(getattr(self, "effective_min_board_margin_px", getattr(self, "min_board_margin_px", 10.0)))
        reprojection_limit_enabled = max_reprojection_error_px > 0.0
        reprojection_ok = None if reprojection is None or not reprojection_limit_enabled else reprojection <= max_reprojection_error_px
        margin_ok = None if margin is None else margin >= min_board_margin_px
        return {
            "max_reprojection_error_px": max_reprojection_error_px,
            "min_board_margin_px": min_board_margin_px,
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
