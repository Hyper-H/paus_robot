from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

LOCAL_PERCEPTION_ROOT = Path(__file__).resolve().parents[2] / "paus_perception"
if (LOCAL_PERCEPTION_ROOT / "setup.py").exists() and str(LOCAL_PERCEPTION_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_PERCEPTION_ROOT))

from paus_perception import (
    capture_rgb_frame,
    encode_bgr_frame_to_jpeg,
    open_camera_runtime,
    pack_frame_packet,
    save_runtime_calibration_to_yaml,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge DkamSDK RGB frames to ROS2 over local TCP.")
    parser.add_argument("--host", default="127.0.0.1", help="TCP destination host.")
    parser.add_argument("--port", type=int, default=5001, help="TCP destination port.")
    parser.add_argument("--camera-ip", default=None, help="Optional camera IP to connect.")
    parser.add_argument("--camera-index", type=int, default=None, help="Optional camera index to connect.")
    parser.add_argument("--camera-config-output", default="/tmp/paus_robot/camera.yaml", help="Output path for converted camera calibration YAML.")
    parser.add_argument("--jpeg-quality", type=int, default=90, help="JPEG encoding quality.")
    parser.add_argument("--frame-id", default="camera", help="Frame id attached to bridge messages.")
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Seconds to wait before reconnect.")
    parser.add_argument("--timeout-us", type=int, default=3_000_000, help="SDK capture timeout in microseconds.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    while True:
        try:
            with open_camera_runtime(camera_ip=args.camera_ip, camera_index=args.camera_index) as runtime:
                calibration = save_runtime_calibration_to_yaml(runtime, output_path=args.camera_config_output, camera_count=0)
                with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
                    print(
                        json.dumps(
                            {
                                "event": "camera_bridge_connected",
                                "camera_index": runtime.camera_index,
                                "host": args.host,
                                "port": args.port,
                                "camera_yaml": args.camera_config_output,
                                "image_width": calibration.image_width,
                                "image_height": calibration.image_height,
                            },
                            ensure_ascii=False,
                        )
                    )
                    while True:
                        image_bgr = capture_rgb_frame(runtime, timeout_us=args.timeout_us)
                        payload = encode_bgr_frame_to_jpeg(image_bgr, jpeg_quality=args.jpeg_quality)
                        header = {
                            "frame_type": "rgb",
                            "encoding": "jpeg",
                            "width": int(image_bgr.shape[1]),
                            "height": int(image_bgr.shape[0]),
                            "timestamp_ns": time.time_ns(),
                            "frame_id": args.frame_id,
                            "camera_info_version": "camera_yaml",
                        }
                        packet = pack_frame_packet(header, payload)
                        connection.sendall(packet)
        except KeyboardInterrupt:
            return 0
        except Exception as exc:
            print(json.dumps({"event": "camera_bridge_error", "error": repr(exc)}, ensure_ascii=False))
            time.sleep(float(args.reconnect_delay))


if __name__ == "__main__":
    raise SystemExit(main())
