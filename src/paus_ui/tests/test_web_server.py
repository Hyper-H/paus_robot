from __future__ import annotations

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
    assert "/static/app.js?v=20260521" in index_html


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

    with client.websocket_connect("/ws/events") as websocket:
        assert websocket.receive_json()["type"] == "test"

    with client.websocket_connect("/ws/image"):
        pass
