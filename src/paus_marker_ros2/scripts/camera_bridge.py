from __future__ import annotations

import argparse
import inspect
import json
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

LOCAL_PERCEPTION_ROOT = Path(__file__).resolve().parents[2] / "paus_perception"
if (LOCAL_PERCEPTION_ROOT / "setup.py").exists() and str(LOCAL_PERCEPTION_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_PERCEPTION_ROOT))

from paus_perception import (  # noqa: E402
    camera_calibration_to_camera_info_payload,
    camera_info_payload_summary,
    capture_rgb_frame,
    compress_payload,
    encode_bgr_frame_to_jpeg,
    open_camera_runtime,
    pack_frame_packet,
    read_stream_transport_diagnostics,
    read_runtime_calibration,
    save_camera_calibration,
)

_SHUTDOWN_REQUESTED = False


class _DepthReconfigureRequested(Exception):
    """Exit the current SDK session so it can be reopened with new streams."""


@dataclass
class _DepthControl:
    requested_enabled: bool

    def __post_init__(self) -> None:
        self.condition = threading.Condition()
        self.active_enabled = False
        self.depth_sequence = 0
        self.last_error: str | None = None

    def requested(self) -> bool:
        with self.condition:
            return bool(self.requested_enabled)

    def mark_runtime(self, enabled: bool) -> None:
        with self.condition:
            self.active_enabled = bool(enabled)
            self.condition.notify_all()

    def mark_depth_frame(self) -> None:
        with self.condition:
            self.depth_sequence += 1
            self.condition.notify_all()

    def fail(self, message: str) -> None:
        with self.condition:
            self.last_error = str(message)
            self.requested_enabled = False
            self.active_enabled = False
            self.condition.notify_all()

    def status(self) -> dict[str, object]:
        with self.condition:
            return {
                "requested": bool(self.requested_enabled),
                "active": bool(self.active_enabled),
                "depth_sequence": int(self.depth_sequence),
                "last_error": self.last_error,
            }

    def request(self, enabled: bool, timeout_s: float) -> dict[str, object]:
        enabled = bool(enabled)
        timeout_s = max(0.1, float(timeout_s))
        deadline = time.monotonic() + timeout_s
        with self.condition:
            previous_depth_sequence = self.depth_sequence
            self.requested_enabled = enabled
            self.last_error = None
            self.condition.notify_all()
            while True:
                if self.last_error is not None:
                    return {
                        "success": False,
                        "enabled": enabled,
                        "active": bool(self.active_enabled),
                        "message": self.last_error,
                        **self.status(),
                    }
                active = bool(self.active_enabled)
                fresh_depth = self.depth_sequence > previous_depth_sequence
                if active == enabled and (not enabled or fresh_depth):
                    return {
                        "success": True,
                        "enabled": enabled,
                        "active": active,
                        "message": "Depth capture enabled." if enabled else "Depth capture disabled.",
                        **self.status(),
                    }
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return {
                        "success": False,
                        "enabled": enabled,
                        "active": active,
                        "message": (
                            "Timed out waiting for a fresh depth frame."
                            if enabled
                            else "Timed out waiting for depth capture to stop."
                        ),
                        **self.status(),
                    }
                self.condition.wait(timeout=remaining)


def _serve_depth_control(control: _DepthControl, host: str, port: int) -> None:
    try:
        with socket.create_server((host, int(port)), reuse_port=False) as server:
            server.settimeout(0.5)
            _log_event(
                {
                    "event": "camera_depth_control_ready",
                    "host": host,
                    "port": int(port),
                }
            )
            while not _SHUTDOWN_REQUESTED:
                try:
                    connection, _address = server.accept()
                except socket.timeout:
                    continue
                with connection:
                    connection.settimeout(30.0)
                    try:
                        request_line = connection.makefile("rb").readline()
                        request = json.loads(request_line.decode("utf-8"))
                        command = str(request.get("command", "")).strip()
                        if command == "set_depth_enabled":
                            enabled = request.get("enabled")
                            if not isinstance(enabled, bool):
                                raise ValueError("enabled must be a JSON boolean.")
                            response = control.request(
                                enabled,
                                float(request.get("timeout_s", 20.0)),
                            )
                        elif command == "get_status":
                            response = {"success": True, **control.status()}
                        else:
                            response = {"success": False, "message": f"Unsupported camera command: {command!r}"}
                    except Exception as exc:
                        response = {"success": False, "message": repr(exc), **control.status()}
                    connection.sendall((json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8"))
    except Exception as exc:
        control.fail(f"Camera depth control server failed to bind {host}:{int(port)}: {exc!r}")
        _log_event({"event": "camera_depth_control_error", "error": repr(exc), "host": host, "port": int(port)})


def _request_shutdown(signum=None, _frame=None) -> None:
    global _SHUTDOWN_REQUESTED
    if not _SHUTDOWN_REQUESTED:
        print(json.dumps({"event": "camera_bridge_shutdown_requested", "signal": signum}, ensure_ascii=False), flush=True)
    _SHUTDOWN_REQUESTED = True


def _install_signal_handlers() -> None:
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)


def _log_event(payload: dict[str, object]) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def _load_depth_alignment_helpers():
    from paus_perception.dkam_depth_alignment import DepthAlignmentConfig, capture_aligned_depth_frame

    return DepthAlignmentConfig, capture_aligned_depth_frame


def _open_camera_runtime_for_args(args: argparse.Namespace, *, depth_enabled: bool):
    parameters = inspect.signature(open_camera_runtime).parameters
    kwargs: dict[str, object] = {
        "camera_ip": args.camera_ip,
        "camera_index": args.camera_index,
    }
    if "stream_channels" in parameters:
        kwargs["stream_channels"] = (args.rgb_channel, args.point_channel) if depth_enabled else (args.rgb_channel,)
    elif depth_enabled:
        raise RuntimeError("The installed sdk_camera.open_camera_runtime does not support depth stream channels.")
    if "rgb_channel" in parameters:
        kwargs["rgb_channel"] = args.rgb_channel
    elif int(args.rgb_channel) != 2:
        raise RuntimeError("The installed sdk_camera.open_camera_runtime only supports RGB channel 2.")
    if "point_channel" in parameters:
        kwargs["point_channel"] = args.point_channel
    return open_camera_runtime(**kwargs)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge DkamSDK RGB frames to ROS2 over local TCP.")
    parser.add_argument("--host", default="127.0.0.1", help="TCP destination host.")
    parser.add_argument("--port", type=int, default=5001, help="TCP destination port.")
    parser.add_argument("--camera-ip", default=None, help="Optional camera IP to connect.")
    parser.add_argument("--camera-index", type=int, default=None, help="Optional camera index to connect.")
    parser.add_argument("--jpeg-quality", type=int, default=90, help="JPEG encoding quality.")
    parser.add_argument("--frame-id", default="camera", help="Frame id attached to bridge messages.")
    parser.add_argument("--camera-config-output", default=None, help="Optional path to write runtime camera calibration YAML.")
    parser.add_argument("--preview-max-fps", type=float, default=8.0, help="Max preview JPEG FPS sent for the UI stream. 0 disables preview packets.")
    parser.add_argument("--preview-width", type=int, default=1280, help="Resize UI preview JPEG to this max width. 0 keeps source width.")
    parser.add_argument("--preview-jpeg-quality", type=int, default=80, help="JPEG quality for UI preview packets.")
    parser.add_argument("--max-fps", type=float, default=8.0, help="Maximum RGB capture/send FPS. 0 disables throttling.")
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Seconds to wait before reconnect.")
    parser.add_argument("--timeout-us", type=int, default=3_000_000, help="SDK capture timeout in microseconds.")
    parser.add_argument("--enable-depth", action="store_true", help="Also publish RGB-aligned depth frames from DkamSDK channel 1.")
    parser.add_argument("--control-host", default="127.0.0.1", help="Local host for camera control commands.")
    parser.add_argument("--control-port", type=int, default=5002, help="Local TCP port for camera control commands.")
    parser.add_argument("--point-channel", type=int, default=1, help="DkamSDK point cloud channel.")
    parser.add_argument("--rgb-channel", type=int, default=2, help="DkamSDK RGB channel.")
    parser.add_argument("--rgb-camera-count", type=int, default=1, help="Factory calibration camera_count for RGB intrinsics/extrinsics.")
    parser.add_argument(
        "--extrinsic-direction",
        choices=("factory_rt", "factory_rt_inverse"),
        default="factory_rt_inverse",
        help="Factory extrinsic direction used for point cloud projection.",
    )
    parser.add_argument("--depth-compression-level", type=int, default=1, help="zlib compression level for aligned depth payloads.")
    return parser.parse_args()


def _make_preview_image(image_bgr: np.ndarray, *, max_width: int) -> np.ndarray:
    if max_width <= 0 or image_bgr.shape[1] <= max_width:
        return image_bgr
    scale = float(max_width) / float(image_bgr.shape[1])
    height = max(1, int(round(float(image_bgr.shape[0]) * scale)))
    return cv2.resize(image_bgr, (int(max_width), height), interpolation=cv2.INTER_AREA)


def main() -> int:
    args = parse_args()
    _install_signal_handlers()
    depth_control = _DepthControl(bool(args.enable_depth))
    control_thread = threading.Thread(
        target=_serve_depth_control,
        args=(depth_control, args.control_host, int(args.control_port)),
        daemon=True,
        name="camera-depth-control",
    )
    control_thread.start()
    preview_interval_s = 0.0 if float(args.preview_max_fps) <= 0.0 else 1.0 / float(args.preview_max_fps)
    capture_interval_s = 0.0 if float(args.max_fps) <= 0.0 else 1.0 / float(args.max_fps)
    _log_event(
        {
            "event": "camera_bridge_started",
            "host": args.host,
            "port": args.port,
            "enable_depth": bool(args.enable_depth),
            "camera_ip": args.camera_ip,
            "camera_index": args.camera_index,
            "preview_max_fps": args.preview_max_fps,
            "preview_width": args.preview_width,
            "max_fps": args.max_fps,
        }
    )
    try:
        while not _SHUTDOWN_REQUESTED:
            depth_enabled = depth_control.requested()
            runtime_mode_marked = False
            try:
                depth_config = None
                capture_aligned_depth_frame = None
                if depth_enabled:
                    DepthAlignmentConfig, capture_aligned_depth_frame = _load_depth_alignment_helpers()
                    depth_config = DepthAlignmentConfig(
                        point_channel=args.point_channel,
                        rgb_channel=args.rgb_channel,
                        rgb_camera_count=args.rgb_camera_count,
                        extrinsic_direction=args.extrinsic_direction,
                    )
                with _open_camera_runtime_for_args(args, depth_enabled=depth_enabled) as runtime:
                    calibration = read_runtime_calibration(runtime, camera_count=args.rgb_camera_count)
                    camera_info_payload = camera_calibration_to_camera_info_payload(
                        calibration,
                        frame_id=args.frame_id,
                        rgb_camera_count=args.rgb_camera_count,
                        source="dkam_sdk",
                    )
                    _log_event(
                        {
                            "event": "camera_transport_ready",
                            **read_stream_transport_diagnostics(runtime, args.rgb_channel),
                        }
                    )
                    if args.camera_config_output:
                        save_camera_calibration(calibration, args.camera_config_output)
                        _log_event({"event": "camera_config_written", "path": args.camera_config_output})
                    _log_event({"event": "camera_info_loaded", **camera_info_payload_summary(camera_info_payload)})
                    with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
                        _log_event(
                            {
                                "event": "camera_bridge_connected",
                                "camera_index": runtime.camera_index,
                                "host": args.host,
                                "port": args.port,
                                "image_width": calibration.image_width,
                                "image_height": calibration.image_height,
                                "enable_depth": bool(args.enable_depth),
                                "point_channel": args.point_channel,
                                "rgb_channel": args.rgb_channel,
                                "rgb_camera_count": args.rgb_camera_count,
                                "extrinsic_direction": args.extrinsic_direction,
                                "depth_enabled_runtime": bool(depth_enabled),
                            }
                        )
                        depth_control.mark_runtime(depth_enabled)
                        runtime_mode_marked = True
                        depth_error_count = 0
                        max_depth_capture_errors = 5
                        last_preview_sent_s = 0.0
                        next_capture_time_s = 0.0
                        while not _SHUTDOWN_REQUESTED:
                            if depth_control.requested() != depth_enabled:
                                raise _DepthReconfigureRequested()
                            if capture_interval_s > 0.0:
                                now_s = time.monotonic()
                                if next_capture_time_s > now_s:
                                    time.sleep(next_capture_time_s - now_s)
                                next_capture_time_s = time.monotonic() + capture_interval_s
                            try:
                                image_bgr = capture_rgb_frame(runtime, timeout_us=args.timeout_us)
                            except Exception as exc:
                                _log_event(
                                    {
                                        "event": "rgb_capture_error",
                                        "error": repr(exc),
                                        "transport": read_stream_transport_diagnostics(runtime, args.rgb_channel),
                                    }
                                )
                                raise
                            if depth_control.requested() != depth_enabled:
                                raise _DepthReconfigureRequested()
                            depth_frame = None
                            if depth_enabled:
                                try:
                                    assert depth_config is not None
                                    assert capture_aligned_depth_frame is not None
                                    depth_frame = capture_aligned_depth_frame(
                                        runtime,
                                        calibration=calibration,
                                        timeout_us=args.timeout_us,
                                        config=depth_config,
                                    )
                                    depth_error_count = 0
                                except Exception as exc:
                                    depth_error_count += 1
                                    _log_event(
                                        {
                                            "event": "depth_capture_error",
                                            "error": repr(exc),
                                            "consecutive_errors": depth_error_count,
                                        }
                                    )
                                    if depth_error_count >= max_depth_capture_errors:
                                        _log_event(
                                            {
                                                "event": "depth_capture_disabled",
                                                "error": repr(exc),
                                                "consecutive_errors": depth_error_count,
                                            }
                                        )
                                        depth_control.fail(
                                            f"Depth capture failed {depth_error_count} consecutive times: {exc!r}"
                                        )
                                        raise _DepthReconfigureRequested()
                            frame_timestamp_ns = time.time_ns()
                            payload = encode_bgr_frame_to_jpeg(image_bgr, jpeg_quality=args.jpeg_quality)
                            header = {
                                "frame_type": "rgb",
                                "encoding": "jpeg",
                                "width": int(image_bgr.shape[1]),
                                "height": int(image_bgr.shape[0]),
                                "timestamp_ns": frame_timestamp_ns,
                                "frame_id": args.frame_id,
                                "camera_info_version": "inline_camera_info",
                                "camera_info": camera_info_payload,
                            }
                            connection.sendall(pack_frame_packet(header, payload))
                            now_s = time.monotonic()
                            if preview_interval_s > 0.0 and now_s - last_preview_sent_s >= preview_interval_s:
                                preview_bgr = _make_preview_image(image_bgr, max_width=int(args.preview_width))
                                preview_payload = encode_bgr_frame_to_jpeg(
                                    preview_bgr,
                                    jpeg_quality=int(args.preview_jpeg_quality),
                                )
                                preview_header = {
                                    "frame_type": "preview_jpeg",
                                    "encoding": "jpeg",
                                    "width": int(preview_bgr.shape[1]),
                                    "height": int(preview_bgr.shape[0]),
                                    "timestamp_ns": frame_timestamp_ns,
                                    "frame_id": args.frame_id,
                                    "source_frame_type": "rgb",
                                }
                                connection.sendall(pack_frame_packet(preview_header, preview_payload))
                                last_preview_sent_s = now_s
                            if depth_frame is not None:
                                depth_control.mark_depth_frame()
                                depth_map = np.ascontiguousarray(depth_frame.aligned_depth_m, dtype=np.float32)
                                depth_payload = compress_payload(depth_map.tobytes(), level=args.depth_compression_level)
                                depth_header = {
                                    "frame_type": "aligned_depth",
                                    "encoding": "32FC1",
                                    "compression": "zlib",
                                    "width": int(depth_frame.rgb_width),
                                    "height": int(depth_frame.rgb_height),
                                    "timestamp_ns": frame_timestamp_ns,
                                    "frame_id": args.frame_id,
                                    "camera_info_version": "inline_camera_info",
                                    "camera_info": camera_info_payload,
                                    "unit": "m",
                                    "point_channel": args.point_channel,
                                    "rgb_camera_count": args.rgb_camera_count,
                                    "extrinsic_direction": args.extrinsic_direction,
                                    "aligned_pixel_count": int(depth_frame.aligned_pixel_count),
                                    "coverage_ratio": float(depth_frame.coverage_ratio),
                                }
                                connection.sendall(pack_frame_packet(depth_header, depth_payload))
            except _DepthReconfigureRequested:
                continue
            except KeyboardInterrupt:
                _request_shutdown("KeyboardInterrupt")
                break
            except Exception as exc:
                if _SHUTDOWN_REQUESTED:
                    break
                if depth_enabled:
                    depth_control.fail(f"Depth-enabled camera runtime failed: {exc!r}")
                    _log_event({"event": "camera_bridge_depth_fallback", "error": repr(exc)})
                _log_event({"event": "camera_bridge_error", "error": repr(exc)})
                time.sleep(float(args.reconnect_delay))
            finally:
                if runtime_mode_marked:
                    depth_control.mark_runtime(False)
    finally:
        _log_event({"event": "camera_bridge_stopped"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
