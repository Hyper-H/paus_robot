from __future__ import annotations

import ipaddress
import json
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from paus_interfaces.msg import UltrasoundFrame

from .frame_server import FrameServer
from .frames import NormalizedFrame


IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def make_messages(frame: NormalizedFrame) -> tuple[Image, UltrasoundFrame]:
    header = frame.header
    image = Image()
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(frame.received_ns, 1_000_000_000)
    image.header.frame_id = "ultrasound_image"
    image.height = header["height"]
    image.width = header["width"]
    image.encoding = header["encoding"]
    image.is_bigendian = False
    image.step = image.width * (1 if image.encoding == "mono8" else 2)
    image.data = frame.pixels

    combined = UltrasoundFrame()
    combined.image = image
    combined.protocol_version = 2
    combined.stream_id = "us"
    combined.session_id = header["session_id"]
    combined.sequence_id = header["sequence_id"]
    combined.metadata_json = json.dumps(header, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    return image, combined


class UltrasoundReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("ultrasound_receiver_node")
        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 55001)
        self.declare_parameter("stale_timeout_s", 1.0)
        host = self.get_parameter("listen_host").value
        port = self.get_parameter("listen_port").value
        ipaddress.IPv4Address(host)
        if type(port) is not int or not 0 < port < 65536:
            raise ValueError("listen_port must be an integer TCP port")
        self.image_publisher = self.create_publisher(Image, "/ultrasound/us/image_raw", IMAGE_QOS)
        self.frame_publisher = self.create_publisher(UltrasoundFrame, "/ultrasound/us/frame", IMAGE_QOS)
        self.status_publisher = self.create_publisher(String, "/ultrasound/us/status", STATUS_QOS)
        self.server = FrameServer(host, port, float(self.get_parameter("stale_timeout_s").value))
        self.server.start()
        self.create_timer(0.01, self._publish_latest)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(f"USFR v2 listening on {host}:{port} (US only; no robot control)")
        self._publish_status()

    def _publish_latest(self) -> None:
        frame = self.server.pop_latest()
        if frame is None:
            return
        image, combined = make_messages(frame)
        self.frame_publisher.publish(combined)
        self.image_publisher.publish(image)

    def _publish_status(self) -> None:
        status = self.server.snapshot()
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, allow_nan=False)
        self.status_publisher.publish(message)

    def destroy_node(self) -> bool:
        self.server.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = UltrasoundReceiverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch can forward a second SIGINT while the receiver thread is joining.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
