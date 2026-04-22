from __future__ import annotations

import json
import socket
import threading

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from paus_perception import recv_frame_packet


class ImageReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("image_receiver_node")

        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 5001)
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("frame_id", "camera")

        self.listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)

        self._latest_frame: tuple[dict[str, object], np.ndarray] | None = None
        self._frame_lock = threading.Lock()

        self._server_thread = threading.Thread(target=self._serve_loop, daemon=True)
        self._server_thread.start()
        self.create_timer(0.01, self._publish_latest_frame)

        self.get_logger().info(
            json.dumps(
                {
                    "event": "image_receiver_started",
                    "listen_host": self.listen_host,
                    "listen_port": self.listen_port,
                    "image_topic": self.image_topic,
                },
                ensure_ascii=False,
            )
        )

    def _serve_loop(self) -> None:
        with socket.create_server((self.listen_host, int(self.listen_port)), reuse_port=False) as server:
            while rclpy.ok():
                connection, address = server.accept()
                self.get_logger().info(json.dumps({"event": "bridge_connected", "peer": address[0], "port": address[1]}, ensure_ascii=False))
                with connection:
                    while rclpy.ok():
                        try:
                            header, payload = recv_frame_packet(connection)
                        except Exception as exc:
                            self.get_logger().warning(json.dumps({"event": "bridge_disconnected", "reason": repr(exc)}, ensure_ascii=False))
                            break
                        if header.get("frame_type") != "rgb" or header.get("encoding") != "jpeg":
                            continue
                        image_array = np.frombuffer(payload, dtype=np.uint8)
                        image_bgr = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                        if image_bgr is None:
                            continue
                        with self._frame_lock:
                            self._latest_frame = (header, image_bgr)

    def _publish_latest_frame(self) -> None:
        with self._frame_lock:
            if self._latest_frame is None:
                return
            header, image_bgr = self._latest_frame
            self._latest_frame = None

        image_message = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        image_message.header.frame_id = str(header.get("frame_id", self.default_frame_id))
        timestamp_ns = header.get("timestamp_ns")
        if isinstance(timestamp_ns, int):
            image_message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            image_message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        self.image_publisher.publish(image_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImageReceiverNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
