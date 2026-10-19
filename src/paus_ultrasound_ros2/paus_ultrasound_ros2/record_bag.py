from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from paus_interfaces.msg import UltrasoundFrame


FRAME_TOPIC = "/ultrasound/us/frame"
STATUS_TOPIC = "/ultrasound/us/status"
IMAGE_TOPIC = "/ultrasound/us/image_raw"
FRAME_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class FirstFrameProbe(Node):
    def __init__(self) -> None:
        super().__init__("ultrasound_recording_probe")
        self.frame: UltrasoundFrame | None = None
        self.create_subscription(UltrasoundFrame, FRAME_TOPIC, self._on_frame, FRAME_QOS)

    def _on_frame(self, message: UltrasoundFrame) -> None:
        self.frame = message


def make_output_path(output_root: Path, name: str | None, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    bag_name = name or f"us_real_{timestamp}"
    return output_root.expanduser().resolve() / bag_name


def build_record_command(output_path: Path, include_image_raw: bool) -> list[str]:
    topics = [FRAME_TOPIC, STATUS_TOPIC]
    if include_image_raw:
        topics.append(IMAGE_TOPIC)
    return [
        "ros2",
        "bag",
        "record",
        "--storage",
        "sqlite3",
        "--output",
        str(output_path),
        *topics,
    ]


def frame_summary(frame: UltrasoundFrame) -> str:
    image = frame.image
    return (
        f"{image.width}x{image.height}, encoding={image.encoding}, "
        f"step={image.step} bytes/row, payload={len(image.data)} bytes, "
        f"session={frame.session_id}, sequence={frame.sequence_id}"
    )


def wait_for_first_frame(timeout_s: float) -> UltrasoundFrame | None:
    rclpy.init()
    node = FirstFrameProbe()
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and node.frame is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        return node.frame
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_recorder(command: list[str]) -> int:
    process = subprocess.Popen(command, start_new_session=True)
    try:
        return process.wait()
    except KeyboardInterrupt:
        print("\n收到停止请求，正在安全结束 rosbag 录制...")
        process.send_signal(signal.SIGINT)
        try:
            return process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            return process.wait(timeout=5.0)


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="等待真实 US 首帧并录制 UltrasoundFrame rosbag。")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("/mnt/data/projects/paus_robot/rosbags"),
        help="rosbag 根目录，默认：/mnt/data/projects/paus_robot/rosbags",
    )
    parser.add_argument("--name", help="录制目录名；默认使用 us_real_YYYYmmdd_HHMMSS。")
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=20.0,
        help="等待首帧的最长秒数，默认 20。",
    )
    parser.add_argument(
        "--include-image-raw",
        action="store_true",
        help="额外录制 image_raw；默认不录，因为 frame 已包含完整图像。",
    )
    return parser.parse_args(args)


def main(args: list[str] | None = None) -> None:
    options = parse_args(args)
    if options.wait_timeout <= 0.0:
        raise SystemExit("--wait-timeout 必须大于 0")
    if shutil.which("ros2") is None:
        raise SystemExit("找不到 ros2 命令；请先 source ROS 2 和当前工作树的 install/local_setup.bash")

    output_path = make_output_path(options.output_root, options.name)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        raise SystemExit(f"录制目录已存在：{output_path}")

    free_gib = shutil.disk_usage(output_path.parent).free / (1024**3)
    print(f"等待 {FRAME_TOPIC} 首帧，超时 {options.wait_timeout:g} 秒...")
    try:
        frame = wait_for_first_frame(options.wait_timeout)
    except KeyboardInterrupt:
        print("\n已取消录制；没有创建 rosbag。")
        return
    if frame is None:
        raise SystemExit(
            "未收到超声帧。请确认 receiver 正在运行、Windows 正在发送，并检查 ROS_DOMAIN_ID。"
        )

    print(f"首帧参数：{frame_summary(frame)}")
    print(f"剩余磁盘空间：{free_gib:.1f} GiB")
    print(f"录制目录：{output_path}")
    print("录制中；完成后按 Ctrl+C。")

    command = build_record_command(output_path, options.include_image_raw)
    return_code = run_recorder(command)
    if return_code not in (0, -signal.SIGINT):
        raise SystemExit(f"rosbag 录制异常退出，返回码：{return_code}")

    print("\n录制完成，bag 摘要：")
    subprocess.run(["ros2", "bag", "info", str(output_path)], check=False)
    print(f"数据位置：{output_path}")


if __name__ == "__main__":
    main(sys.argv[1:])
