from __future__ import annotations

import asyncio
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

    app = FastAPI(title="PAUS Robot UI")
    static_root = Path(__file__).resolve().parent / "static"
    app.mount("/static", StaticFiles(directory=str(static_root)), name="static")

    @app.get("/")
    async def index():
        return FileResponse(str(static_root / "index.html"))

    @app.get("/api/status")
    async def status() -> dict[str, Any]:
        return bridge.get_status()

    @app.get("/api/events")
    async def events(since: int = 0) -> dict[str, Any]:
        rows, last_id = bridge.get_events_since(since)
        return {"events": rows, "last_id": last_id}

    @app.get("/api/image/latest.jpg")
    async def latest_image(mode: str = "overlay", axes: bool = True) -> Response:
        return Response(content=await asyncio.to_thread(bridge.get_latest_jpeg, mode=mode, show_axes=axes), media_type="image/jpeg")

    @app.get("/api/handeye/quality")
    async def quality() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.get_latest_quality)

    @app.get("/api/handeye/waypoints")
    async def waypoints() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.get_waypoints)

    @app.post("/api/handeye/record_waypoint")
    async def record_waypoint() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.record_waypoint)

    @app.post("/api/handeye/delete_last_waypoint")
    async def delete_last_waypoint() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.delete_last_waypoint)

    @app.post("/api/handeye/save_trajectory")
    async def save_trajectory() -> dict[str, Any]:
        return await asyncio.to_thread(bridge.save_trajectory)

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

    @app.get("/api/sessions/{session_id}/sample-image/{row_index}.jpg")
    async def session_sample_image(session_id: str, row_index: int, mode: str = "overlay", axes: bool = True) -> Response:
        try:
            image = await asyncio.to_thread(bridge.get_sample_jpeg, session_id=session_id, row_index=row_index, mode=mode, show_axes=axes)
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
