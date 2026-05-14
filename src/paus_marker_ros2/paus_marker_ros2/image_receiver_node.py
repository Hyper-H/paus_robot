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

from paus_perception import decompress_payload, recv_frame_packet


class ImageReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("image_receiver_node")

        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 5001)
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("depth_topic", "/camera/depth_aligned")
        self.declare_parameter("frame_id", "camera")

        self.listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)
        self.depth_publisher = self.create_publisher(Image, self.depth_topic, 10)

        self._latest_rgb_frame: tuple[dict[str, object], np.ndarray] | None = None
        self._latest_depth_frame: tuple[dict[str, object], np.ndarray] | None = None
        self._frame_lock = threading.Lock()

        self._server_thread = threading.Thread(target=self._serve_loop, daemon=True)
        self._server_thread.start()
        self.create_timer(0.01, self._publish_latest_rgb_frame)
        self.create_timer(0.01, self._publish_latest_depth_frame)

        self.get_logger().info(
            json.dumps(
                {
                    "event": "image_receiver_started",
                    "listen_host": self.listen_host,
                    "listen_port": self.listen_port,
                    "image_topic": self.image_topic,
                    "depth_topic": self.depth_topic,
                },
                ensure_ascii=False,
            )
        )

    def _serve_loop(self) -> None:
        with socket.create_server((self.listen_host, int(self.listen_port)), reuse_port=False) as server:
            while rclpy.ok():
                connection, address = server.accept()
                self.get_logger().info(
                    json.dumps(
                        {
                            "event": "bridge_connected",
                            "peer": address[0],
                            "port": address[1],
                        },
                        ensure_ascii=False,
                    )
                )
                with connection:
                    while rclpy.ok():
                        try:
                            header, payload = recv_frame_packet(connection)
                        except Exception as exc:
                            self.get_logger().warning(
                                json.dumps({"event": "bridge_disconnected", "reason": repr(exc)}, ensure_ascii=False)
                            )
                            break

                        frame_type = str(header.get("frame_type", ""))
                        encoding = str(header.get("encoding", ""))

                        if frame_type == "rgb" and encoding == "jpeg":
                            image_array = np.frombuffer(payload, dtype=np.uint8)
                            image_bgr = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                            if image_bgr is None:
                                continue
                            with self._frame_lock:
                                self._latest_rgb_frame = (header, image_bgr)
                            continue

                        if frame_type == "aligned_depth" and encoding == "32FC1":
                            compression = str(header.get("compression", ""))
                            depth_payload = decompress_payload(payload) if compression == "zlib" else payload
                            width = int(header.get("width", 0))
                            height = int(header.get("height", 0))
                            if width <= 0 or height <= 0:
                                continue
                            depth_array = np.frombuffer(depth_payload, dtype=np.float32)
                            if depth_array.size != width * height:
                                continue
                            depth_map = depth_array.reshape((height, width))
                            with self._frame_lock:
                                self._latest_depth_frame = (header, depth_map)

    def _publish_common_header(self, message: Image, header: dict[str, object]) -> None:
        message.header.frame_id = str(header.get("frame_id", self.default_frame_id))
        timestamp_ns = header.get("timestamp_ns")
        if isinstance(timestamp_ns, int):
            message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)

    def _publish_latest_rgb_frame(self) -> None:
        with self._frame_lock:
            if self._latest_rgb_frame is None:
                return
            header, image_bgr = self._latest_rgb_frame
            self._latest_rgb_frame = None

        image_message = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        self._publish_common_header(image_message, header)
        self.image_publisher.publish(image_message)

    def _publish_latest_depth_frame(self) -> None:
        with self._frame_lock:
            if self._latest_depth_frame is None:
                return
            header, depth_map = self._latest_depth_frame
            self._latest_depth_frame = None

        depth_message = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
        self._publish_common_header(depth_message, header)
        self.depth_publisher.publish(depth_message)


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


if __name__ == "__main__":
    main()
