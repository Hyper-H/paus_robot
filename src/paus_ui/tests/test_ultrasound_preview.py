from __future__ import annotations

import json
import time
from types import SimpleNamespace

import cv2
import pytest

from paus_ui.ultrasound_preview import UltrasoundPreview


def frame(**metadata):
    seconds, nanoseconds = divmod(time.time_ns(), 1_000_000_000)
    return SimpleNamespace(
        image=SimpleNamespace(
            width=2, height=2, encoding="mono8", step=2,
            data=bytes([0, 80, 160, 255]), is_bigendian=False,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)),
        ),
        session_id=metadata.get("session_id", "preview-session"),
        metadata_json=json.dumps({
            "session_id": "preview-session",
            "roi_confirmed": True, "geometry_valid": False,
            "display_scale_mm_per_pixel": {"horizontal": None, "vertical": 0.05},
            **metadata,
        }),
    )


def status(state="STREAMING", session_id="preview-session", latest_age_s=0.0):
    return json.dumps({
        "state": state, "received": 12, "stale_timeout_s": 1.0,
        "latest_age_s": latest_age_s, "frame_metadata": {"session_id": session_id},
    })


def test_one_axis_scale_is_not_promoted_to_physical_calibration() -> None:
    preview = UltrasoundPreview()
    preview.receive_status(status())
    preview.receive_frame(frame(profile_revision="v2"))
    report = preview.snapshot()
    assert report["stream_available"] is True
    assert report["roi_confirmed"] is True
    assert report["display_scale_mm_per_pixel"] == {"horizontal": None, "vertical": 0.05}
    assert report["geometry_valid"] is False
    assert report["scale_source"] == "display_ruler"
    assert report["profile_revision"] == "v2"
    assert preview.jpeg().startswith(b"\xff\xd8")


def test_stale_preview_is_not_returned_even_when_connection_remains_open(monkeypatch) -> None:
    clock = [10.0]
    monkeypatch.setattr("paus_ui.ultrasound_preview.time.monotonic", lambda: clock[0])
    preview = UltrasoundPreview()
    preview.receive_status(status())
    preview.receive_frame(frame())
    assert preview.jpeg() is not None
    clock[0] += 1.1
    preview.receive_status(status())
    assert preview.snapshot()["state"] == "STALE"
    assert preview.jpeg() is None
    preview.receive_frame(frame())
    assert preview.jpeg() is not None
    preview.receive_status(status("LISTENING"))
    assert preview.jpeg() is None


def test_receiver_death_expires_preview_and_unknown_axes_stay_unknown(monkeypatch) -> None:
    clock = [10.0]
    monkeypatch.setattr("paus_ui.ultrasound_preview.time.monotonic", lambda: clock[0])
    preview = UltrasoundPreview()
    preview.receive_status(status())
    preview.receive_frame(frame(display_scale_mm_per_pixel={"horizontal": -1, "vertical": False}))
    assert preview.snapshot()["display_scale_mm_per_pixel"] == {"horizontal": None, "vertical": None}
    clock[0] += 1.1
    assert preview.snapshot()["state"] == "WAITING"
    assert preview.jpeg() is None


def test_mono16_preview_uses_fixed_range_and_handles_stride(monkeypatch) -> None:
    captured = []
    original = cv2.imencode
    def encode(ext, pixels, params):
        captured.append(pixels.tolist())
        return original(ext, pixels, params)
    monkeypatch.setattr(cv2, "imencode", encode)
    preview = UltrasoundPreview()
    preview.receive_status(status())
    message = frame()
    message.image.encoding = "mono16"
    message.image.step = 6
    message.image.data = bytes([0, 0, 0, 80, 12, 34, 0, 160, 255, 255, 12, 34])
    preview.receive_frame(message)
    preview.jpeg()
    assert captured == [[[0, 80], [160, 255]]]


def test_invalid_image_is_rejected() -> None:
    preview = UltrasoundPreview()
    message = frame()
    message.image.data = b"\0"
    with pytest.raises(ValueError, match="data length"):
        preview.receive_frame(message)


def test_old_receive_stamp_cannot_be_made_fresh_by_late_callback() -> None:
    preview = UltrasoundPreview()
    preview.receive_status(status())
    message = frame()
    message.image.header.stamp.sec -= 100
    preview.receive_frame(message)
    assert preview.snapshot()["state"] == "STALE"
    assert preview.snapshot()["latest_age_s"] >= 100
    assert preview.jpeg() is None


def test_old_session_cannot_be_displayed_against_new_receiver_session() -> None:
    preview = UltrasoundPreview()
    preview.receive_status(status(session_id="new-session"))
    preview.receive_frame(frame())
    assert preview.snapshot()["session_matches"] is False
    assert preview.jpeg() is None
    preview.receive_frame(frame(session_id="new-session"))
    assert preview.snapshot()["session_matches"] is True
    assert preview.jpeg() is not None


def test_stale_receiver_age_overrides_fresh_ui_callback() -> None:
    preview = UltrasoundPreview()
    preview.receive_status(status(latest_age_s=10.0))
    preview.receive_frame(frame())
    assert preview.snapshot()["state"] == "STALE"
    assert preview.jpeg() is None


def test_ui_uses_independent_best_effort_frame_subscription(monkeypatch, tmp_path) -> None:
    from paus_ui.ros_bridge import UiRosBridge, ULTRASOUND_QOS
    from rclpy.qos import ReliabilityPolicy
    import threading

    bridge = UiRosBridge.__new__(UiRosBridge)
    bridge._ultrasound = UltrasoundPreview()
    bridge._image_lock = threading.Lock()
    sentinel = object()
    bridge._latest_image = sentinel
    bridge._ultrasound_status_callback(SimpleNamespace(data=status()))
    bridge._ultrasound_frame_callback(frame())
    assert bridge._latest_image is sentinel
    assert bridge.get_ultrasound_status()["stream_available"] is True
    assert ULTRASOUND_QOS.reliability == ReliabilityPolicy.BEST_EFFORT


def test_ultrasound_http_routes_never_use_handeye_image() -> None:
    from fastapi.testclient import TestClient
    from paus_ui.web_server import create_app

    class Bridge:
        jpeg = b"us-only-jpeg"
        def get_latest_jpeg(self, **kwargs):
            raise AssertionError("handeye preview must not be used")
        def get_ultrasound_jpeg(self):
            return self.jpeg
        def get_ultrasound_status(self):
            return {"state": "STREAMING", "stream_available": True}

    bridge = Bridge()
    with TestClient(create_app(bridge)) as client:
        response = client.get("/api/ultrasound/image.jpg")
        assert response.content == b"us-only-jpeg"
        assert response.headers["cache-control"] == "no-store"
        assert client.get("/api/ultrasound/status").json()["stream_available"]
        bridge.jpeg = None
        assert client.get("/api/ultrasound/image.jpg").status_code == 204


@pytest.mark.parametrize("path", ["/ws/image", "/ws/events"])
def test_idle_websocket_returns_after_browser_disconnect(path) -> None:
    import asyncio
    from paus_ui.web_server import create_app

    class Bridge:
        def _latest_image_meta(self):
            return None
        def get_events_since(self, last_id):
            return [], last_id

    class ClosedSocket:
        async def accept(self):
            pass
        async def receive(self):
            return {"type": "websocket.disconnect"}

    endpoint = next(route.endpoint for route in create_app(Bridge()).routes if route.path == path)
    asyncio.run(asyncio.wait_for(endpoint(ClosedSocket()), timeout=1.0))
