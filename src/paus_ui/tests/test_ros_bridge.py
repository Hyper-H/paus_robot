from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
from pathlib import Path

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

PROJECT_ROOT = Path(__file__).resolve().parents[3]
UI_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_ui"
if str(UI_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(UI_PACKAGE_ROOT))

pytest.importorskip("ament_index_python")
pytest.importorskip("cv_bridge")
pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")
pytest.importorskip("std_msgs")
pytest.importorskip("std_srvs")

from paus_ui.ros_bridge import CachedImage, UiRosBridge
from paus_ui.session_store import SessionStore


class _ServiceClient:
    def __init__(self, ready: bool) -> None:
        self._ready = ready
        self.srv_name = "/eye_to_hand/test"

    def service_is_ready(self) -> bool:
        return self._ready

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self._ready


def test_get_detector_returns_none_for_malformed_camera_yaml() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        camera_yaml = Path(temp_dir) / "camera.yaml"
        camera_yaml.write_text("not: [valid", encoding="utf-8")
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.camera_config_path = camera_yaml
        bridge._detector = None
        bridge._detector_mtime_ns = None

        assert UiRosBridge._get_detector(bridge) is None
        assert bridge._detector is None
        assert bridge._detector_mtime_ns is None


def test_backend_status_overrides_ui_local_motion_and_paths() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = root / "local_trajectory.yaml"
        bridge.session_root_path = root / "local_sessions"
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(False)}
        bridge.session_store = SessionStore(
            session_root_path=bridge.session_root_path,
            trajectory_path=bridge.trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        state = UiRosBridge._sync_backend_state(
            bridge,
            {
                "execute_motion": True,
                "trajectory_path": str(root / "backend_trajectory.yaml"),
                "session_root_path": str(root / "backend_sessions"),
                "max_reprojection_error_px": 4.0,
                "min_board_margin_px": 20.0,
            },
            0.1,
        )

        assert state["config_source"] == "backend_status"
        assert state["execute_motion"] is True
        assert bridge.session_store.trajectory_path == root / "backend_trajectory.yaml"
        assert bridge.session_store.session_root_path == root / "backend_sessions"


def test_stale_status_uses_service_ready_fallback() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = root / "local_trajectory.yaml"
        bridge.session_root_path = root / "local_sessions"
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(True)}
        bridge.session_store = SessionStore(
            session_root_path=bridge.session_root_path,
            trajectory_path=bridge.trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        state = UiRosBridge._sync_backend_state(bridge, {"execute_motion": False}, 120.0)

        assert state["backend_connected"] is True
        assert state["status_recent"] is False
        assert state["motion_state_known"] is False
        assert state["config_source"] == "backend_service_ready"
        assert state["execute_motion"] is False


def test_stale_status_disconnects_when_services_disappear() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = root / "local_trajectory.yaml"
        bridge.session_root_path = root / "local_sessions"
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(False)}
        bridge.session_store = SessionStore(
            session_root_path=bridge.session_root_path,
            trajectory_path=bridge.trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        state = UiRosBridge._sync_backend_state(
            bridge,
            {"execute_motion": True, "trajectory_path": str(root / "stale_backend.yaml")},
            120.0,
        )

        assert state["backend_connected"] is False
        assert state["execute_motion"] is False
        assert state["config_source"] == "ui_local_fallback"
        assert bridge.session_store.trajectory_path == root / "local_trajectory.yaml"


def test_service_ready_without_status_requires_motion_confirmation() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = root / "local_trajectory.yaml"
        bridge.session_root_path = root / "local_sessions"
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(True)}
        bridge.session_store = SessionStore(
            session_root_path=bridge.session_root_path,
            trajectory_path=bridge.trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        state = UiRosBridge._sync_backend_state(bridge, None, 0.0)

        assert state["backend_connected"] is True
        assert state["motion_state_known"] is False
        assert state["execute_motion"] is False
        assert state["config_source"] == "backend_service_ready"


def test_backend_status_updates_overlay_detector_settings() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        backend_camera = root / "backend_camera.yaml"
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.camera_config_path = root / "local_camera.yaml"
        bridge.board_rows = 6
        bridge.board_cols = 9
        bridge.square_size_m = 0.01
        bridge.trajectory_path = root / "local_trajectory.yaml"
        bridge.session_root_path = root / "local_sessions"
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(False)}
        bridge._detector = object()
        bridge._detector_mtime_ns = 123
        bridge._detector_signature = ("old", 123, 6, 9, 0.01)
        bridge.session_store = SessionStore(
            session_root_path=bridge.session_root_path,
            trajectory_path=bridge.trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        state = UiRosBridge._sync_backend_state(
            bridge,
            {
                "execute_motion": False,
                "camera_config_path": str(backend_camera),
                "board_rows": 8,
                "board_cols": 11,
                "square_size_m": 0.02,
            },
            0.1,
        )

        assert state["board_rows"] == 8
        assert state["board_cols"] == 11
        assert state["square_size_m"] == 0.02
        assert state["camera_config_path"] == backend_camera
        assert bridge.board_rows == 8
        assert bridge.board_cols == 11
        assert bridge.square_size_m == 0.02
        assert bridge.camera_config_path == backend_camera
        assert bridge._detector is None
        assert bridge._detector_signature is None


def test_get_status_does_not_recompute_latest_quality() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        trajectory_path = root / "trajectory.yaml"
        trajectory_path.write_text("version: 1\nwaypoints: []\n", encoding="utf-8")
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.ui_host = "0.0.0.0"
        bridge.ui_port = 8080
        bridge.image_topic = "/camera/image_bridge"
        bridge.status_topic = "/eye_to_hand/status"
        bridge.camera_config_path = root / "camera.yaml"
        bridge.config_path = str(root / "default.yaml")
        bridge.board_rows = 6
        bridge.board_cols = 9
        bridge.square_size_m = 0.01
        bridge._image_lock = threading.Lock()
        bridge._latest_image = None
        bridge._last_status_lock = threading.Lock()
        bridge._last_status = {
            "status": "waypoint_pose_estimated",
            "camera_to_board_translation_m": [0.1, 0.2, 0.3],
            "camera_to_board_rotation_rpy_deg": [1.0, 2.0, 3.0],
            "board_angle_deg": 12.0,
            "quality_detected": True,
            "image_sequence": 42,
        }
        bridge._last_status_time_s = time.monotonic()
        bridge._last_command_result = None
        bridge._run_thread = None
        bridge.session_store = SessionStore(
            session_root_path=root / "sessions",
            trajectory_path=trajectory_path,
            max_reprojection_error_px=0.0,
            min_board_margin_px=10.0,
        )
        bridge._sync_backend_state = lambda *_args: {
            "backend_connected": True,
            "execute_motion": False,
            "motion_state_known": True,
            "status_recent": True,
            "config_source": "backend_status",
            "trajectory_path": trajectory_path,
            "session_root_path": root / "sessions",
            "max_reprojection_error_px": 0.0,
            "min_board_margin_px": 10.0,
        }
        bridge._motion_summary = lambda: {}
        bridge._stop_status = lambda: {}
        bridge.get_latest_quality = lambda: (_ for _ in ()).throw(AssertionError("get_status must stay cheap"))

        status = UiRosBridge.get_status(bridge)

        assert status["handeye"]["current_waypoint"]["camera_to_board_translation_m"] == [0.1, 0.2, 0.3]
        assert status["handeye"]["current_waypoint"]["image_sequence"] == 42


def test_get_status_drops_stale_backend_payload_when_disconnected() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        trajectory_path = root / "trajectory.yaml"
        trajectory_path.write_text("version: 1\nwaypoints: []\n", encoding="utf-8")
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.ui_host = "0.0.0.0"
        bridge.ui_port = 8080
        bridge.image_topic = "/camera/image_bridge"
        bridge.status_topic = "/eye_to_hand/status"
        bridge.camera_config_path = root / "camera.yaml"
        bridge.config_path = str(root / "default.yaml")
        bridge.board_rows = 6
        bridge.board_cols = 9
        bridge.square_size_m = 0.01
        bridge._image_lock = threading.Lock()
        bridge._latest_image = None
        bridge._last_status_lock = threading.Lock()
        bridge._last_status = {"status": "waypoint_pose_estimated", "waypoint_name": "stale_001", "session_dir": str(root / "old_session")}
        bridge._last_status_time_s = time.monotonic() - 120.0
        bridge._last_command_result = None
        bridge._run_thread = None
        bridge.session_store = SessionStore(
            session_root_path=root / "sessions",
            trajectory_path=trajectory_path,
            max_reprojection_error_px=0.0,
            min_board_margin_px=10.0,
        )
        bridge._sync_backend_state = lambda *_args: {
            "backend_connected": False,
            "execute_motion": False,
            "motion_state_known": False,
            "status_recent": False,
            "config_source": "ui_local_fallback",
            "trajectory_path": trajectory_path,
            "session_root_path": root / "sessions",
            "max_reprojection_error_px": 0.0,
            "min_board_margin_px": 10.0,
        }
        bridge._motion_summary = lambda: {}
        bridge._stop_status = lambda: {}

        status = UiRosBridge.get_status(bridge)

        assert status["handeye"]["last_status"] is None
        assert status["handeye"]["current_waypoint"]["name"] is None


def test_get_waypoints_prefers_live_recorded_trajectory() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        trajectory_path = root / "trajectory.yaml"
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge._last_status_lock = threading.Lock()
        bridge._last_status = {
            "recorded_trajectory": {
                "version": 1,
                "tool_id": 0,
                "user_id": 0,
                "defaults": {"motion": "movej", "vel": 10.0, "acc": 10.0, "dwell_s": 0.5},
                "waypoints": [{"name": "waypoint_001", "capture": True}],
            },
            "trajectory_dirty": True,
        }
        bridge.session_store = SessionStore(
            session_root_path=root / "sessions",
            trajectory_path=trajectory_path,
            max_reprojection_error_px=0.0,
            min_board_margin_px=10.0,
        )
        bridge._sync_backend_state = lambda *_args: {
            "backend_connected": True,
            "trajectory_path": trajectory_path,
            "session_root_path": root / "sessions",
            "execute_motion": False,
            "motion_state_known": True,
            "config_source": "backend_status",
            "max_reprojection_error_px": 0.0,
            "min_board_margin_px": 10.0,
        }

        waypoints = UiRosBridge.get_waypoints(bridge)

        assert waypoints["source"] == "recorded_trajectory"
        assert waypoints["dirty"] is True
        assert waypoints["waypoints"][0]["name"] == "waypoint_001"


def test_successful_run_command_preserves_backend_message() -> None:
    bridge = UiRosBridge.__new__(UiRosBridge)

    result = UiRosBridge._shape_command_result(
        bridge,
        "run_semi_auto",
        True,
        "Dry-run complete for 2 waypoints. No motion, capture, solve, or save was executed.",
    )

    assert result["operator_message"] == result["message"]


def test_start_run_reports_queued_request_as_pending_not_success() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = root / "trajectory.yaml"
        bridge.session_root_path = root / "sessions"
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._last_status_lock = threading.Lock()
        bridge._last_status = {"execute_motion": False}
        bridge._last_status_time_s = None
        bridge._service_clients = {"run_semi_auto": _ServiceClient(True)}
        bridge._run_lock = threading.Lock()
        bridge._run_thread = None
        bridge._last_command_result = None
        bridge._run_semi_auto_worker = lambda: None
        bridge.session_store = SessionStore(
            session_root_path=bridge.session_root_path,
            trajectory_path=bridge.trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        result = UiRosBridge.start_semi_auto_run(bridge, confirmed=True)

        assert result["accepted"] is True
        assert result["queued"] is True
        assert result["success"] is False
        assert bridge._last_command_result == result


def test_latest_quality_treats_stale_cached_image_as_unavailable() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = root / "trajectory.yaml"
        bridge.session_root_path = root / "sessions"
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(False)}
        bridge._last_status_lock = threading.Lock()
        bridge._last_status = None
        bridge._last_status_time_s = None
        bridge._image_lock = threading.Lock()
        bridge._latest_image = CachedImage(
            image_bgr=np.zeros((32, 32, 3), dtype=np.uint8),
            sequence=7,
            header_time_s=None,
            received_time_s=time.monotonic() - 10.0,
        )
        bridge.session_store = SessionStore(
            session_root_path=bridge.session_root_path,
            trajectory_path=bridge.trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        quality = UiRosBridge.get_latest_quality(bridge)

        assert quality["detected"] is False
        assert quality["reason_code"] == "stale_image"
        assert quality["image_sequence"] == 7


def test_archived_sample_overlay_uses_saved_metadata_without_live_detector() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        trajectory_path = root / "trajectory.yaml"
        trajectory_path.write_text("version: 1\nwaypoints: []\n", encoding="utf-8")
        session_root = root / "sessions"
        session_path = session_root / "2026-04-29_120000"
        images_path = session_path / "images"
        images_path.mkdir(parents=True)
        image_path = images_path / "sample_001.png"
        cv2.imwrite(str(image_path), np.full((96, 128, 3), 210, dtype=np.uint8))
        with (session_path / "samples.jsonl").open("w", encoding="utf-8") as handle:
            handle.write(
                json.dumps(
                    {
                        "sample_index": 1,
                        "image_path": str(image_path),
                        "image_sequence": 42,
                        "reprojection_error_px": 2.5,
                        "board_margin_px": 120.0,
                        "camera_to_board_translation_m": [0.1, 0.2, 0.3],
                        "camera_to_board_rotation_rpy_deg": [1.0, 2.0, 3.0],
                    }
                )
                + "\n"
            )
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = trajectory_path
        bridge.session_root_path = session_root
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(False)}
        bridge._last_status_lock = threading.Lock()
        bridge._last_status = None
        bridge._last_status_time_s = None
        bridge.session_store = SessionStore(
            session_root_path=session_root,
            trajectory_path=trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        raw = UiRosBridge.get_sample_jpeg(bridge, session_id=session_path.name, row_index=1, mode="raw")
        overlay = UiRosBridge.get_sample_jpeg(bridge, session_id=session_path.name, row_index=1, mode="overlay")

        assert raw.startswith(b"\xff\xd8")
        assert overlay.startswith(b"\xff\xd8")
        assert overlay != raw


def test_missing_archived_sample_image_raises_not_found() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        trajectory_path = root / "trajectory.yaml"
        trajectory_path.write_text("version: 1\nwaypoints: []\n", encoding="utf-8")
        session_root = root / "sessions"
        session_path = session_root / "2026-04-29_120000"
        session_path.mkdir(parents=True)
        with (session_path / "samples.jsonl").open("w", encoding="utf-8") as handle:
            handle.write(json.dumps({"sample_index": 1}) + "\n")
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.trajectory_path = trajectory_path
        bridge.session_root_path = session_root
        bridge.max_reprojection_error_px = 0.0
        bridge.min_board_margin_px = 10.0
        bridge.execute_motion = False
        bridge.effective_trajectory_path = bridge.trajectory_path
        bridge.effective_session_root_path = bridge.session_root_path
        bridge.effective_max_reprojection_error_px = bridge.max_reprojection_error_px
        bridge.effective_min_board_margin_px = bridge.min_board_margin_px
        bridge.effective_execute_motion = bridge.execute_motion
        bridge._service_clients = {"run": _ServiceClient(False)}
        bridge._last_status_lock = threading.Lock()
        bridge._last_status = None
        bridge._last_status_time_s = None
        bridge.session_store = SessionStore(
            session_root_path=session_root,
            trajectory_path=trajectory_path,
            max_reprojection_error_px=bridge.max_reprojection_error_px,
            min_board_margin_px=bridge.min_board_margin_px,
        )

        with pytest.raises(FileNotFoundError):
            UiRosBridge.get_sample_jpeg(bridge, session_id=session_path.name, row_index=1, mode="overlay")


def test_workflow_maps_backend_dry_run_waypoint_status() -> None:
    bridge = UiRosBridge.__new__(UiRosBridge)

    workflow = UiRosBridge._workflow_for_status(bridge, "waypoint_dry_run_complete")

    assert workflow["stage"] == "dry_run"
