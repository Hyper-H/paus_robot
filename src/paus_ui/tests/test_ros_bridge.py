from __future__ import annotations

import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
UI_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_ui"
if str(UI_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(UI_PACKAGE_ROOT))

from paus_ui.ros_bridge import UiRosBridge
from paus_ui.session_store import SessionStore


class _ServiceClient:
    def __init__(self, ready: bool) -> None:
        self._ready = ready

    def service_is_ready(self) -> bool:
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


def test_backend_status_does_not_expire_when_node_is_idle() -> None:
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
        assert state["config_source"] == "backend_status"


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
