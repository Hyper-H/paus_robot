from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 json，用于输出结构化日志。
import json
# 导入 socket，用于向本地图像接收节点发送帧数据。
import socket
# 导入 sys，用于在源码直跑场景下补充包路径。
import sys
# 导入 time，用于生成时间戳和重连等待。
import time
# 导入 Path，便于处理输出路径和源码路径。
from pathlib import Path

import numpy as np

# 计算当前 marker 包旁边的 perception 包根目录。
LOCAL_PERCEPTION_ROOT = Path(__file__).resolve().parents[2] / "paus_perception"
# 如果当前脚本是从源码目录直接运行，就把 perception 包根目录加入搜索路径。
if (LOCAL_PERCEPTION_ROOT / "setup.py").exists() and str(LOCAL_PERCEPTION_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_PERCEPTION_ROOT))

# 导入相机采帧、JPEG 编码、相机运行时管理和桥接协议打包函数。
from paus_perception import (
    DepthAlignmentConfig,
    capture_aligned_depth_frame,
    capture_rgb_frame,
    compress_payload,
    encode_bgr_frame_to_jpeg,
    open_camera_runtime,
    pack_frame_packet,
    save_runtime_calibration_to_yaml,
)


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Bridge DkamSDK RGB frames to ROS2 over local TCP.")
    # 指定 TCP 目标主机。
    parser.add_argument("--host", default="127.0.0.1", help="TCP destination host.")
    # 指定 TCP 目标端口。
    parser.add_argument("--port", type=int, default=5001, help="TCP destination port.")
    # 可选指定相机 IP。
    parser.add_argument("--camera-ip", default=None, help="Optional camera IP to connect.")
    # 可选指定相机索引。
    parser.add_argument("--camera-index", type=int, default=None, help="Optional camera index to connect.")
    # 指定导出的 `camera.yaml` 路径。
    parser.add_argument("--camera-config-output", default="/tmp/paus_robot/camera.yaml", help="Output path for converted camera calibration YAML.")
    # 指定 JPEG 压缩质量。
    parser.add_argument("--jpeg-quality", type=int, default=90, help="JPEG encoding quality.")
    # 指定桥接消息里的 frame_id。
    parser.add_argument("--frame-id", default="camera", help="Frame id attached to bridge messages.")
    # 指定重连等待时长。
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Seconds to wait before reconnect.")
    # 指定相机抓帧超时。
    parser.add_argument("--timeout-us", type=int, default=3_000_000, help="SDK capture timeout in microseconds.")
    # 可选启用 RGB 对齐深度流。
    parser.add_argument("--enable-depth", action="store_true", help="Also publish RGB-aligned depth frames from DkamSDK channel 1.")
    # 指定点云 channel。
    parser.add_argument("--point-channel", type=int, default=1, help="DkamSDK point cloud channel.")
    # 指定 RGB channel。
    parser.add_argument("--rgb-channel", type=int, default=2, help="DkamSDK RGB channel.")
    # 指定厂家 RGB 标定索引。
    parser.add_argument("--rgb-camera-count", type=int, default=1, help="Factory calibration camera_count for RGB intrinsics/extrinsics.")
    # 指定点云到 RGB 的厂家外参方向。
    parser.add_argument(
        "--extrinsic-direction",
        choices=("factory_rt", "factory_rt_inverse"),
        default="factory_rt_inverse",
        help="Factory extrinsic direction used for point cloud projection.",
    )
    # 指定深度负载压缩等级。
    parser.add_argument("--depth-compression-level", type=int, default=1, help="zlib compression level for aligned depth payloads.")
    # 返回解析结果。
    return parser.parse_args()


# 相机桥接主函数。
def main() -> int:
    # 解析命令行参数。
    args = parse_args()
    depth_enabled = bool(args.enable_depth)
    # 外层循环用于断线重连。
    while True:
        try:
            stream_channels = (args.rgb_channel, args.point_channel) if depth_enabled else (args.rgb_channel,)
            depth_config = DepthAlignmentConfig(
                point_channel=args.point_channel,
                rgb_channel=args.rgb_channel,
                rgb_camera_count=args.rgb_camera_count,
                extrinsic_direction=args.extrinsic_direction,
            )
            # 打开相机运行时。
            with open_camera_runtime(
                camera_ip=args.camera_ip,
                camera_index=args.camera_index,
                stream_channels=stream_channels,
                rgb_channel=args.rgb_channel,
                point_channel=args.point_channel,
            ) as runtime:
                # 将厂家运行时标定导出成统一格式的 `camera.yaml`。
                calibration = save_runtime_calibration_to_yaml(runtime, output_path=args.camera_config_output, camera_count=args.rgb_camera_count)
                # 连接到本地图像接收节点。
                with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
                    # 打印连接成功日志。
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
                    # 持续抓帧并发送。
                    depth_error_count = 0
                    while True:
                        # 从工业相机抓取一帧 BGR 图像。
                        image_bgr = capture_rgb_frame(runtime, timeout_us=args.timeout_us)
                        depth_frame = None
                        if depth_enabled:
                            try:
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
                        # 编码成 JPEG，减小 TCP 传输体积。
                        payload = encode_bgr_frame_to_jpeg(image_bgr, jpeg_quality=args.jpeg_quality)
                        # 组装桥接协议头。
                        header = {
                            "frame_type": "rgb",
                            "encoding": "jpeg",
                            "width": int(image_bgr.shape[1]),
                            "height": int(image_bgr.shape[0]),
                            "timestamp_ns": frame_timestamp_ns,
                            "frame_id": args.frame_id,
                            "camera_info_version": "camera_yaml",
                        }
                        # 把头和负载封装成一帧完整协议数据。
                        packet = pack_frame_packet(header, payload)
                        # 发给 `image_receiver_node`。
                        connection.sendall(packet)
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
                                "camera_info_version": "camera_yaml",
                                "unit": "m",
                                "point_channel": args.point_channel,
                                "rgb_camera_count": args.rgb_camera_count,
                                "extrinsic_direction": args.extrinsic_direction,
                                "aligned_pixel_count": int(depth_frame.aligned_pixel_count),
                                "coverage_ratio": float(depth_frame.coverage_ratio),
                            }
                            connection.sendall(pack_frame_packet(depth_header, depth_payload))
        except KeyboardInterrupt:
            # 用户主动中断时正常退出。
            return 0
        except Exception as exc:
            # 深度链路初始化失败时先降级成 RGB-only，避免把 RGB 一起带崩。
            if bool(args.enable_depth) and depth_enabled:
                print(json.dumps({"event": "camera_bridge_depth_fallback", "error": repr(exc)}, ensure_ascii=False))
                depth_enabled = False
                time.sleep(float(args.reconnect_delay))
                continue
            # 出现异常时打印错误并稍后重连。
            print(json.dumps({"event": "camera_bridge_error", "error": repr(exc)}, ensure_ascii=False))
            time.sleep(float(args.reconnect_delay))


# 当脚本被直接执行时，进入主函数。
if __name__ == "__main__":
    raise SystemExit(main())
