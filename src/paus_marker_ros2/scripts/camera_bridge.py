from __future__ import annotations

import argparse
import inspect
import json
import socket
import sys
import time
from pathlib import Path

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
    read_runtime_calibration,
)


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
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Seconds to wait before reconnect.")
    parser.add_argument("--timeout-us", type=int, default=3_000_000, help="SDK capture timeout in microseconds.")
    parser.add_argument("--enable-depth", action="store_true", help="Also publish RGB-aligned depth frames from DkamSDK channel 1.")
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


def main() -> int:
    args = parse_args()
    depth_enabled = bool(args.enable_depth)
    while True:
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
                print(
                    json.dumps(
                        {"event": "camera_info_loaded", **camera_info_payload_summary(camera_info_payload)},
                        ensure_ascii=False,
                    )
                )
                with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
                    print(
                        json.dumps(
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
                            },
                            ensure_ascii=False,
                        )
                    )
                    depth_error_count = 0
                    while True:
                        image_bgr = capture_rgb_frame(runtime, timeout_us=args.timeout_us)
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
                                print(
                                    json.dumps(
                                        {
                                            "event": "depth_capture_error",
                                            "error": repr(exc),
                                            "consecutive_errors": depth_error_count,
                                        },
                                        ensure_ascii=False,
                                    )
                                )
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
                        if depth_frame is not None:
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
        except KeyboardInterrupt:
            return 0
        except Exception as exc:
            if bool(args.enable_depth) and depth_enabled:
                print(json.dumps({"event": "camera_bridge_depth_fallback", "error": repr(exc)}, ensure_ascii=False))
                depth_enabled = False
                time.sleep(float(args.reconnect_delay))
                continue
            print(json.dumps({"event": "camera_bridge_error", "error": repr(exc)}, ensure_ascii=False))
            time.sleep(float(args.reconnect_delay))


if __name__ == "__main__":
    raise SystemExit(main())
