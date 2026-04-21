from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 json，用于日志输出结构化状态。
import json
# 导入 socket，用于通过 TCP 发送图像。
import socket
# 导入 sys，用于补充源码路径。
import sys
# 导入 time，用于重连等待和时间戳。
import time
# 导入 Path，便于处理配置输出路径。
from pathlib import Path

# 计算项目根目录。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
# 计算源码目录。
SRC_ROOT = PROJECT_ROOT / "src"
# 若源码目录未加入搜索路径，则插入。
if SRC_ROOT.exists() and str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

# 导入桥接协议和 SDK 适配层函数。
from ag_repro import (
    capture_rgb_frame,
    encode_bgr_frame_to_jpeg,
    open_camera_runtime,
    pack_frame_packet,
    save_runtime_calibration_to_yaml,
)


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Bridge DkamSDK RGB frames to ROS2 over local TCP.")
    # 指定桥接目标主机。
    parser.add_argument("--host", default="127.0.0.1", help="TCP destination host.")
    # 指定桥接目标端口。
    parser.add_argument("--port", type=int, default=5001, help="TCP destination port.")
    # 指定相机 IP；不填则默认选第一台。
    parser.add_argument("--camera-ip", default=None, help="Optional camera IP to connect.")
    # 指定相机索引；与 camera-ip 二选一。
    parser.add_argument("--camera-index", type=int, default=None, help="Optional camera index to connect.")
    # 指定导出的 camera.yaml 路径。
    parser.add_argument("--camera-config-output", default=str(PROJECT_ROOT / "configs" / "camera.yaml"), help="Output path for converted camera calibration YAML.")
    # 指定 JPEG 质量。
    parser.add_argument("--jpeg-quality", type=int, default=90, help="JPEG encoding quality.")
    # 指定桥接消息中的 frame_id。
    parser.add_argument("--frame-id", default="camera", help="Frame id attached to bridge messages.")
    # 指定重连等待秒数。
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Seconds to wait before reconnect.")
    # 指定采帧超时时间，单位微秒。
    parser.add_argument("--timeout-us", type=int, default=3_000_000, help="SDK capture timeout in microseconds.")
    # 返回解析结果。
    return parser.parse_args()


# 相机桥接主函数。
def main() -> int:
    # 解析命令行参数。
    args = parse_args()
    # 启动桥接主循环。
    while True:
        try:
            # 启动相机会话。
            with open_camera_runtime(camera_ip=args.camera_ip, camera_index=args.camera_index) as runtime:
                # 导出厂家标定参数到统一的 camera.yaml。
                calibration = save_runtime_calibration_to_yaml(runtime, output_path=args.camera_config_output, camera_count=0)
                # 打开到 ROS2 接收节点的 TCP 连接。
                with socket.create_connection((args.host, args.port), timeout=5.0) as connection:
                    # 打印启动摘要。
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
                    # 持续抓帧并发送。
                    while True:
                        # 采集一帧 BGR 图像。
                        image_bgr = capture_rgb_frame(runtime, timeout_us=args.timeout_us)
                        # 编码为 JPEG。
                        payload = encode_bgr_frame_to_jpeg(image_bgr, jpeg_quality=args.jpeg_quality)
                        # 构造协议头。
                        header = {
                            "frame_type": "rgb",
                            "encoding": "jpeg",
                            "width": int(image_bgr.shape[1]),
                            "height": int(image_bgr.shape[0]),
                            "timestamp_ns": time.time_ns(),
                            "frame_id": args.frame_id,
                            "camera_info_version": "camera_yaml",
                        }
                        # 打包并发送。
                        packet = pack_frame_packet(header, payload)
                        connection.sendall(packet)
        except KeyboardInterrupt:
            # 用户主动中断时正常退出。
            return 0
        except Exception as exc:
            # 出现异常时打印错误并等待后重连。
            print(json.dumps({"event": "camera_bridge_error", "error": repr(exc)}, ensure_ascii=False))
            time.sleep(float(args.reconnect_delay))


# 当脚本被直接运行时执行主函数。
if __name__ == "__main__":
    raise SystemExit(main())
