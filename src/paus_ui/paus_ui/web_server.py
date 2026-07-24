from __future__ import annotations

import asyncio
import time
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .ros_bridge import UiRosBridge


def _parse_confirmed_flag(body: dict[str, Any] | None) -> bool:
    if body is None:
        return False
    if not isinstance(body, dict):
        raise ValueError("confirmed must be provided in a JSON object.")
    confirmed = body.get("confirmed", False)
    if isinstance(confirmed, bool):
        return confirmed
    raise ValueError("confirmed must be a JSON boolean.")


def create_app(bridge: "UiRosBridge"):
    try:
        from fastapi import Body, FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
        from fastapi.responses import FileResponse
        from fastapi.staticfiles import StaticFiles
    except ImportError as exc:
        raise RuntimeError("paus_ui requires FastAPI and uvicorn. Install them in the paus_robot conda environment.") from exc

    globals()["WebSocket"] = WebSocket
    globals()["WebSocketDisconnect"] = WebSocketDisconnect

    app = FastAPI(title="PAUS Robot UI")
    static_root = Path(__file__).resolve().parent / "static"
    app.mount("/static", StaticFiles(directory=str(static_root)), name="static")

    @app.get("/")
    async def index():
        return FileResponse(
            str(static_root / "index.html"),
            media_type="text/html",
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/status")
    async def status() -> dict[str, Any]:
        return bridge.get_status()

    @app.get("/api/events")
    async def events(since: int = 0) -> dict[str, Any]:
        rows, last_id = bridge.get_events_since(since)
        return {"events": rows, "last_id": last_id}

    @app.get("/api/debug/frontend")
    async def frontend_debug() -> dict[str, Any]:
        app_js_path = static_root / "app.js"
        index_path = static_root / "index.html"
        try:
            app_js = app_js_path.read_text(encoding="utf-8")
        except OSError:
            app_js = ""
        try:
            index_html = index_path.read_text(encoding="utf-8")
        except OSError:
            index_html = ""
        return {
            "web_server_path": str(Path(__file__).resolve()),
            "static_root": str(static_root),
            "app_js_path": str(app_js_path),
            "index_path": str(index_path),
            "app_js_exists": app_js_path.exists(),
            "index_exists": index_path.exists(),
            "app_js_has_ws_image": "ws/image" in app_js,
            "app_js_has_old_live_axes_true": "/api/image/latest.jpg?mode=raw&axes=true" in app_js,
            "index_cache_busts_app_js": "/static/app.js?v=" in index_html,
        }

    @app.get("/api/image/latest.jpg")
    async def latest_image(mode: str = "raw", axes: bool = True) -> Response:
        return Response(content=await asyncio.to_thread(bridge.get_latest_jpeg, mode=mode, show_axes=axes), media_type="image/jpeg")

    @app.websocket("/ws/image")
    async def websocket_image(websocket: WebSocket) -> None:
        await websocket.accept()
        last_preview_sequence = -1
        last_raw_sequence = -1
        try:
            while True:
                preview_getter = getattr(bridge, "get_latest_preview_jpeg", None)
                preview = await asyncio.to_thread(preview_getter) if preview_getter is not None else None
                if preview is not None:
                    sequence, jpeg_bytes, _ = preview
                    if sequence != last_preview_sequence:
                        await websocket.send_bytes(jpeg_bytes)
                        last_preview_sequence = sequence
                    await asyncio.sleep(0.04)
                    continue
                latest_meta = await asyncio.to_thread(bridge._latest_image_meta)
                if latest_meta is None:
                    last_raw_sequence = -1
                    await asyncio.sleep(0.2)
                    continue
                sequence, _, received_time_s = latest_meta
                image_age_s = time.monotonic() - received_time_s
                if image_age_s >= 3.0:
                    await asyncio.sleep(0.2)
                    continue
                if sequence != last_raw_sequence:
                    await websocket.send_bytes(await asyncio.to_thread(bridge.get_latest_jpeg, mode="raw", show_axes=False))
                    last_raw_sequence = sequence
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            return

    @app.get("/api/handeye/quality")
    async def quality() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.get_latest_quality)

    @app.post("/api/handeye/observation-mode")
    async def observation_mode(body: dict[str, Any] | None = Body(default=None)) -> dict[str, Any]:
        payload = body or {}
        mode = payload.get("mode", payload.get("observation_mode", ""))
        if not isinstance(mode, str) or not mode.strip():
            raise HTTPException(status_code=422, detail="mode must be provided.")
        return await asyncio.to_thread(bridge.set_observation_mode, mode.strip())

    @app.get("/api/handeye/waypoints")
    async def waypoints() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.get_waypoints)

    @app.post("/api/handeye/record_waypoint")
    async def record_waypoint() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.record_waypoint)

    @app.post("/api/handeye/delete_last_waypoint")
    async def delete_last_waypoint() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.delete_last_waypoint)

    @app.post("/api/handeye/delete_waypoint")
    async def delete_waypoint(body: dict[str, Any] | None = Body(default=None)) -> dict[str, Any]:
        if not isinstance(body, dict):
            raise HTTPException(status_code=422, detail="waypoint_name must be provided in a JSON object.")
        waypoint_name = str(body.get("waypoint_name", "")).strip()
        if not waypoint_name:
            raise HTTPException(status_code=422, detail="waypoint_name is required.")
        return await asyncio.to_thread(bridge.delete_selected_waypoint, waypoint_name)

    @app.post("/api/handeye/save_trajectory")
    async def save_trajectory() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.save_trajectory)

    @app.post("/api/handeye/new_trajectory")
    async def new_trajectory() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.new_trajectory)

    @app.post("/api/handeye/run")
    async def run_handeye(body: dict[str, Any] | None = Body(default=None)) -> dict[str, Any]:
        try:
            confirmed = _parse_confirmed_flag(body)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        return await asyncio.to_thread(bridge.start_semi_auto_run, confirmed=confirmed)

    @app.post("/api/handeye/stop")
    async def stop_handeye() -> dict[str, Any]:
        return bridge.stop_run()

    @app.get("/api/sessions")
    async def sessions() -> list[dict[str, Any]]:
        return await asyncio.to_thread(bridge.list_sessions)

    @app.get("/api/sessions/latest")
    async def latest_session() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.latest_session)

    @app.get("/api/sessions/{session_id}/report")
    async def session_report(session_id: str) -> dict[str, Any]:
        try:
            return await asyncio.to_thread(bridge.read_session_report, session_id)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.get("/api/sessions/{session_id}/samples")
    async def session_samples(session_id: str) -> list[dict[str, Any]]:
        try:
            return await asyncio.to_thread(bridge.read_session_samples, session_id)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.get("/api/sessions/{session_id}/waypoints")
    async def session_waypoints(session_id: str) -> list[dict[str, Any]]:
        try:
            return await asyncio.to_thread(bridge.read_session_waypoints, session_id)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.post("/api/sessions/{session_id}/load_trajectory")
    async def load_session_trajectory(session_id: str) -> dict[str, Any]:
        try:
            return await asyncio.to_thread(bridge.load_session_trajectory, session_id)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.get("/api/sessions/{session_id}/per-sample-residuals")
    async def per_sample_residuals(session_id: str) -> list[dict[str, Any]]:
        try:
            return await asyncio.to_thread(bridge.per_sample_residuals, session_id)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.get("/api/sessions/{session_id}/sample-image/{row_index}.jpg")
    async def session_sample_image(session_id: str, row_index: int, mode: str = "overlay", axes: bool = True) -> Response:
        try:
            image = await asyncio.to_thread(bridge.get_sample_jpeg, session_id=session_id, row_index=row_index, mode=mode, show_axes=axes)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return Response(content=image, media_type="image/jpeg")

    @app.get("/api/sessions/{session_id}/capture-image/{image_name}.jpg")
    async def session_capture_image(session_id: str, image_name: str) -> Response:
        try:
            image = await asyncio.to_thread(bridge.get_capture_jpeg, session_id=session_id, image_name=image_name)
        except (FileNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return Response(content=image, media_type="image/jpeg")

    @app.websocket("/ws/events")
    async def websocket_events(websocket: WebSocket) -> None:
        await websocket.accept()
        last_id = 0
        try:
            while True:
                events, last_id = bridge.get_events_since(last_id)
                for event in events:
                    await websocket.send_json(event)
                await asyncio.sleep(0.5)
        except WebSocketDisconnect:
            return

    return app


def main(args: list[str] | None = None) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    from .ros_bridge import UiRosBridge

    rclpy.init(args=args)
    bridge = UiRosBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        import uvicorn

        uvicorn.run(create_app(bridge), host=bridge.ui_host, port=bridge.ui_port, log_level="info")
    finally:
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
