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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from paus_perception import load_camera_calibration, load_config, resolve_config_path

from .overlay import BoardOverlayDetector, encode_jpeg, make_placeholder_image
from .session_store import SessionStore


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
        self.trajectory_path = Path(resolve_config_path(self.get_parameter("trajectory_path").get_parameter_value().string_value, self.config_path))
        self.session_root_path = Path(resolve_config_path(self.get_parameter("session_root_path").get_parameter_value().string_value, self.config_path))
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").get_parameter_value().double_value)
        self.min_board_margin_px = float(self.get_parameter("min_board_margin_px").get_parameter_value().double_value)
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)

        self.bridge = CvBridge()
        self.events = EventBuffer()
        self.session_store = SessionStore(session_root_path=self.session_root_path, trajectory_path=self.trajectory_path)

        self._image_lock = threading.Lock()
        self._latest_image: CachedImage | None = None
        self._image_sequence = 0
        self._last_status_lock = threading.Lock()
        self._last_status: dict[str, Any] | None = None
        self._last_status_time_s: float | None = None
        self._detector: BoardOverlayDetector | None = None
        self._detector_mtime_ns: int | None = None
        self._run_lock = threading.Lock()
        self._run_thread: threading.Thread | None = None
        self._last_command_result: dict[str, Any] | None = None

        self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.create_subscription(String, self.status_topic, self._status_callback, 10)
        self._service_clients = {
            "record_waypoint": self.create_client(Trigger, "/eye_to_hand/record_waypoint"),
            "delete_last_waypoint": self.create_client(Trigger, "/eye_to_hand/delete_last_waypoint"),
            "run_semi_auto": self.create_client(Trigger, "/eye_to_hand/run_semi_auto_calibration"),
        }
        self.events.push({"type": "ui_started", "message": "PAUS UI server started."})

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
        with self._last_status_lock:
            self._last_status = payload
            self._last_status_time_s = time.monotonic()
        self.events.push({"type": "eye_to_hand_status", "payload": payload})

    def get_status(self) -> dict[str, Any]:
        with self._image_lock:
            latest_image = self._latest_image
        with self._last_status_lock:
            last_status = dict(self._last_status) if self._last_status else None
            last_status_age_s = (time.monotonic() - self._last_status_time_s) if self._last_status_time_s else None
        camera_age_s = (time.monotonic() - latest_image.received_time_s) if latest_image else None
        return {
            "ui": {
                "host": self.ui_host,
                "port": self.ui_port,
                "url": f"http://{self.ui_host}:{self.ui_port}",
            },
            "camera": {
                "connected": camera_age_s is not None and camera_age_s < 3.0,
                "image_sequence": latest_image.sequence if latest_image else 0,
                "age_s": camera_age_s,
                "topic": self.image_topic,
                "camera_config_path": str(self.camera_config_path),
                "camera_config_exists": self.camera_config_path.exists(),
            },
            "handeye": {
                "status_topic": self.status_topic,
                "last_status": last_status,
                "last_status_age_s": last_status_age_s,
                "calibration_node_connected": last_status_age_s is not None and last_status_age_s < 10.0,
                "execute_motion": self.execute_motion,
                "trajectory_path": str(self.trajectory_path),
                "session_root_path": str(self.session_root_path),
                "max_reprojection_error_px": self.max_reprojection_error_px,
                "min_board_margin_px": self.min_board_margin_px,
                "last_command_result": self._last_command_result,
                "run_active": bool(self._run_thread and self._run_thread.is_alive()),
            },
            "config": {
                "config_path": self.config_path,
                "board_rows": self.board_rows,
                "board_cols": self.board_cols,
                "square_size_m": self.square_size_m,
            },
        }

    def get_latest_quality(self) -> dict[str, Any]:
        latest_image = self._latest_image_copy()
        if latest_image is None:
            return {"detected": False, "reason": "No image received.", "image_sequence": 0}
        detector = self._get_detector()
        if detector is None:
            return {
                "detected": False,
                "reason": f"Camera calibration is unavailable: {self.camera_config_path}",
                "image_sequence": latest_image.sequence,
            }
        result = detector.estimate(latest_image.image_bgr)
        return result.to_payload(image_sequence=latest_image.sequence)

    def get_latest_jpeg(self, *, mode: str = "overlay") -> bytes:
        latest_image = self._latest_image_copy()
        if latest_image is None:
            return encode_jpeg(make_placeholder_image("Waiting for /camera/image_bridge"))
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
        rendered, _ = detector.render(latest_image.image_bgr, mode=mode, image_sequence=latest_image.sequence)
        return encode_jpeg(rendered)

    def get_sample_jpeg(self, *, session_id: str, row_index: int, mode: str = "overlay") -> bytes:
        image_path = self.session_store.sample_image_path(session_id, row_index)
        if image_path is None:
            return encode_jpeg(make_placeholder_image("Sample image not found"))
        image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image_bgr is None:
            return encode_jpeg(make_placeholder_image("Failed to read sample image"))
        if mode == "raw":
            return encode_jpeg(image_bgr)
        detector = self._get_detector()
        if detector is None:
            return encode_jpeg(image_bgr)
        rendered, _ = detector.render(image_bgr, mode=mode, image_sequence=row_index)
        return encode_jpeg(rendered)

    def record_waypoint(self) -> dict[str, Any]:
        return self._call_trigger("record_waypoint", timeout_s=15.0)

    def delete_last_waypoint(self) -> dict[str, Any]:
        return self._call_trigger("delete_last_waypoint", timeout_s=5.0)

    def start_semi_auto_run(self, *, confirmed: bool = False) -> dict[str, Any]:
        if self.execute_motion and not confirmed:
            return {
                "success": False,
                "requires_confirmation": True,
                "message": "execute_motion=true requires UI confirmation before starting.",
            }
        with self._run_lock:
            if self._run_thread is not None and self._run_thread.is_alive():
                return {"success": False, "message": "Semi-auto calibration is already running."}
            self._run_thread = threading.Thread(target=self._run_semi_auto_worker, daemon=True)
            self._run_thread.start()
        return {"success": True, "message": "Semi-auto calibration request started."}

    def stop_run(self) -> dict[str, Any]:
        return {
            "success": False,
            "message": "Stop is not supported by the current calibration node. Use terminal interrupt or the physical emergency stop if motion must stop immediately.",
        }

    def get_events_since(self, last_id: int) -> tuple[list[dict[str, Any]], int]:
        return self.events.since(last_id)

    def _run_semi_auto_worker(self) -> None:
        self.events.push({"type": "ui_command", "command": "run_semi_auto", "message": "Semi-auto service call started."})
        result = self._call_trigger("run_semi_auto", timeout_s=3600.0)
        self._last_command_result = {"command": "run_semi_auto", **result}
        self.events.push({"type": "ui_command_result", "command": "run_semi_auto", "result": result})

    def _call_trigger(self, key: str, *, timeout_s: float) -> dict[str, Any]:
        client = self._service_clients[key]
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            result = {"success": False, "message": f"Service is unavailable: {client.srv_name}"}
            self._last_command_result = {"command": key, **result}
            return result
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_s):
            result = {"success": False, "message": f"Service timed out: {client.srv_name}"}
            self._last_command_result = {"command": key, **result}
            return result
        try:
            response = future.result()
        except Exception as exc:
            result = {"success": False, "message": repr(exc)}
        else:
            result = {"success": bool(response.success), "message": str(response.message)}
        self._last_command_result = {"command": key, **result}
        self.events.push({"type": "ui_command_result", "command": key, "result": result})
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

    def _get_detector(self) -> BoardOverlayDetector | None:
        if not self.camera_config_path.exists():
            return None
        mtime_ns = self.camera_config_path.stat().st_mtime_ns
        if self._detector is not None and self._detector_mtime_ns == mtime_ns:
            return self._detector
        calibration = load_camera_calibration(self.camera_config_path)
        self._detector = BoardOverlayDetector(
            board_rows=self.board_rows,
            board_cols=self.board_cols,
            square_size_m=self.square_size_m,
            camera_calibration=calibration,
        )
        self._detector_mtime_ns = mtime_ns
        return self._detector
