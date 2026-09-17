from __future__ import annotations

import html
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[3]
UI_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_ui"
if str(UI_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(UI_PACKAGE_ROOT))

from paus_ui.web_server import _parse_confirmed_flag


def test_parse_confirmed_flag_rejects_string_values() -> None:
    assert _parse_confirmed_flag(None) is False
    assert _parse_confirmed_flag({"confirmed": False}) is False
    assert _parse_confirmed_flag({"confirmed": True}) is True

    with pytest.raises(ValueError, match="JSON boolean"):
        _parse_confirmed_flag({"confirmed": "false"})


def test_frontend_bundle_contains_live_ws_and_cache_bust() -> None:
    import paus_ui.web_server as web_server

    web_server_path = Path(web_server.__file__).resolve()
    static_root = web_server_path.parent / "static"
    app_js = (static_root / "app.js").read_text(encoding="utf-8")
    index_html = (static_root / "index.html").read_text(encoding="utf-8")

    assert "ws/image" in app_js
    assert "startLiveImageWatchdog" in app_js
    assert "visibilitychange" in app_js
    assert "load-session-trajectory-btn" in index_html
    assert "/static/app.js?v=20260911-1" in index_html
    assert "/static/styles.css?v=20260911-1" in index_html
    decoded_html = html.unescape(index_html)
    assert "当前标定" in decoded_html
    assert "采集 RGB PnP / Depth Align 观测" in decoded_html
    assert "????" not in decoded_html
    assert 'id="failed-count"' in index_html
    assert 'id="rename-session-btn"' in index_html
    assert 'id="delete-session-btn"' in index_html
    assert 'id="batch-session-btn"' in index_html
    assert 'id="batch-session-modal"' in index_html
    assert 'id="batch-session-confirm-modal"' in index_html
    assert "WAYPOINT_STATUS_LABELS" in app_js
    assert "motion-config" in index_html
    assert "/api/handeye/motion-config" in app_js
    assert "/api/sessions/${encodeURIComponent(sessionId)}" in app_js
    assert "永久删除" in app_js


def test_frontend_stages_complete_frames_before_canvas_commit() -> None:
    import paus_ui.web_server as web_server

    web_server_path = Path(web_server.__file__).resolve()
    app_js = (web_server_path.parent / "static" / "app.js").read_text(encoding="utf-8")

    assert 'const stagingImage = new Image();' in app_js
    assert "context.drawImage(stagingImage, 0, 0, width, height);" in app_js
    assert "liveImagePendingUrl" in app_js
    assert "new Blob([payload], { type: \"image/jpeg\" })" in app_js


def test_frontend_prevents_duplicate_real_run_requests() -> None:
    import paus_ui.web_server as web_server

    web_server_path = Path(web_server.__file__).resolve()
    app_js = (web_server_path.parent / "static" / "app.js").read_text(encoding="utf-8")

    assert "let runRequestInFlight = false;" in app_js
    assert "if (runRequestInFlight) return;" in app_js
    assert "runRequestInFlight = false;" in app_js


def test_frontend_avoids_live_http_resync_and_lazy_loads_thumbnails() -> None:
    import paus_ui.web_server as web_server

    web_server_path = Path(web_server.__file__).resolve()
    app_js = (web_server_path.parent / "static" / "app.js").read_text(encoding="utf-8")

    assert "图像 HTTP resync" not in app_js
    assert "图像 WS stale" in app_js
    assert "IntersectionObserver" in app_js
    assert "\\u8f7d\\u5165\\u4e3a\\u5f53\\u524d\\u8f68\\u8ff9" in app_js
    assert "scheduleFullRefresh" in app_js
    assert 'class="thumb-image" data-src=' in app_js


def test_frontend_deduplicates_waypoint_refreshes_and_preserves_history_selection() -> None:
    import paus_ui.web_server as web_server

    web_server_path = Path(web_server.__file__).resolve()
    app_js = (web_server_path.parent / "static" / "app.js").read_text(encoding="utf-8")

    assert "waypointsRefreshInFlight" in app_js
    assert "const requestKey = `${selectionVersion}:${sessionId}`;" in app_js
    assert "await Promise.all([refreshReport(), refreshWaypoints()]);" in app_js
    assert "if (!state.userSelectedSession)" in app_js


def test_websocket_routes_accept_connections() -> None:
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from paus_ui.web_server import create_app

    class FakeBridge:
        def get_events_since(self, last_id: int):
            return ([{"id": 1, "type": "test"}], 1)

        def _latest_image_meta(self):
            return None

    client = TestClient(create_app(FakeBridge()))

    response = client.get("/")
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("text/html")
    assert "charset=utf-8" in response.headers["content-type"].lower()
    assert response.headers["cache-control"] == "no-store"

    with client.websocket_connect("/ws/events") as websocket:
        assert websocket.receive_json()["type"] == "test"

    with client.websocket_connect("/ws/image"):
        pass


def test_session_management_routes_require_confirmation_for_delete() -> None:
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from paus_ui.web_server import create_app

    class FakeBridge:
        def rename_session(self, session_id: str, display_name: str):
            return {
                "success": True,
                "session_id": session_id,
                "display_name": display_name,
            }

        def delete_session(self, session_id: str):
            return {
                "success": True,
                "session_id": session_id,
                "deleted": True,
            }

    client = TestClient(create_app(FakeBridge()))

    rename_response = client.patch(
        "/api/sessions/2026-09-04_202230",
        json={"display_name": "法兰盘标定测试"},
    )
    assert rename_response.status_code == 200
    assert rename_response.json()["display_name"] == "法兰盘标定测试"

    delete_without_confirmation = client.delete("/api/sessions/2026-09-04_202230")
    assert delete_without_confirmation.status_code == 409

    delete_response = client.request(
        "DELETE",
        "/api/sessions/2026-09-04_202230",
        json={"confirmed": True},
    )
    assert delete_response.status_code == 200
    assert delete_response.json()["deleted"] is True


def test_batch_session_deletion_route_requires_confirmation_and_returns_summary() -> None:
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from paus_ui.web_server import create_app

    class FakeBridge:
        def delete_sessions(self, session_ids: list[str]):
            return {
                "success": True,
                "command": "delete_sessions",
                "deleted_session_ids": session_ids,
                "deleted_count": len(session_ids),
                "failed_count": 0,
                "deleted": [{"session_id": session_id, "deleted": True} for session_id in session_ids],
                "failed": [],
                "operator_message": f"已永久删除 {len(session_ids)} 个 session。",
            }

    client = TestClient(create_app(FakeBridge()))

    delete_without_confirmation = client.request(
        "DELETE",
        "/api/sessions",
        json={"session_ids": ["2026-09-04_202230"]},
    )
    assert delete_without_confirmation.status_code == 409

    invalid_ids = client.request(
        "DELETE",
        "/api/sessions",
        json={"session_ids": ["2026-09-04_202230"], "confirmed": True},
    )
    assert invalid_ids.status_code == 200
    assert invalid_ids.json()["deleted_count"] == 1
    assert invalid_ids.json()["deleted_session_ids"] == ["2026-09-04_202230"]
