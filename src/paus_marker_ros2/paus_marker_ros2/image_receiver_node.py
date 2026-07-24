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
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from paus_perception import camera_info_payload_summary, decompress_payload, recv_frame_packet


def timestamp_ns_from_header(header: dict[str, object]) -> int | None:
    timestamp_ns = header.get("timestamp_ns")
    if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns <= 0:
        return None
    return int(timestamp_ns)


class ImageReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("image_receiver_node")

        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 5001)
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("depth_topic", "/camera/depth_aligned")
        self.declare_parameter("preview_topic", "/camera/preview_jpeg")
        self.declare_parameter("frame_id", "camera")

        self.listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.preview_topic = self.get_parameter("preview_topic").get_parameter_value().string_value
        self.default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.depth_publisher = self.create_publisher(Image, self.depth_topic, 10)
        self.preview_publisher = self.create_publisher(CompressedImage, self.preview_topic, 10)

        self._latest_rgb_frame: tuple[dict[str, object], np.ndarray] | None = None
        self._latest_depth_frame: tuple[dict[str, object], np.ndarray] | None = None
        self._frame_lock = threading.Lock()
        self._last_camera_info_signature: tuple[object, ...] | None = None

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
                    "camera_info_topic": self.camera_info_topic,
                    "depth_topic": self.depth_topic,
                    "preview_topic": self.preview_topic,
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

                        if frame_type == "preview_jpeg" and encoding == "jpeg":
                            self._publish_preview_frame(header, payload)
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

    def _publish_common_header(self, message: Image | CameraInfo | CompressedImage, header: dict[str, object]) -> bool:
        message.header.frame_id = str(header.get("frame_id", self.default_frame_id))
        timestamp_ns = timestamp_ns_from_header(header)
        if timestamp_ns is None:
            return False
        message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        return True

    def _publish_preview_frame(self, header: dict[str, object], payload: bytes) -> None:
        message = CompressedImage()
        if not self._publish_common_header(message, header):
            self.get_logger().warning(json.dumps({"event": "frame_timestamp_invalid", "frame_type": "preview_jpeg"}, ensure_ascii=False))
            return
        message.format = "jpeg"
        message.data = payload
        self.preview_publisher.publish(message)

    def _camera_info_from_header(self, header: dict[str, object], image_bgr: np.ndarray) -> CameraInfo | None:
        payload = header.get("camera_info")
        if not isinstance(payload, dict):
            self.get_logger().warning(
                json.dumps({"event": "camera_info_missing", "frame_type": header.get("frame_type")}, ensure_ascii=False)
            )
            return None

        k_values = payload.get("k")
        d_values = payload.get("d")
        if not isinstance(k_values, list) or len(k_values) != 9:
            self.get_logger().warning(json.dumps({"event": "camera_info_invalid", "reason": "invalid_k"}, ensure_ascii=False))
            return None
        if not isinstance(d_values, list):
            self.get_logger().warning(json.dumps({"event": "camera_info_invalid", "reason": "invalid_d"}, ensure_ascii=False))
            return None

        width = int(payload.get("width", 0) or 0)
        height = int(payload.get("height", 0) or 0)
        image_height, image_width = int(image_bgr.shape[0]), int(image_bgr.shape[1])
        if width <= 0 or height <= 0:
            self.get_logger().warning(
                json.dumps(
                    {
                        "event": "camera_info_invalid",
                        "reason": "invalid_dimensions",
                        "camera_info_width": width,
                        "camera_info_height": height,
                    },
                    ensure_ascii=False,
                )
            )
            return None
        if width != image_width or height != image_height:
            self.get_logger().warning(
                json.dumps(
                    {
                        "event": "camera_info_dimension_mismatch",
                        "camera_info_width": width,
                        "camera_info_height": height,
                        "image_width": image_width,
                        "image_height": image_height,
                    },
                    ensure_ascii=False,
                )
            )
            return None

        message = CameraInfo()
        if not self._publish_common_header(message, header):
            self.get_logger().warning(json.dumps({"event": "frame_timestamp_invalid", "frame_type": "rgb"}, ensure_ascii=False))
            return None
        message.width = width
        message.height = height
        message.distortion_model = str(payload.get("distortion_model", "plumb_bob"))
        message.d = [float(value) for value in d_values]
        message.k = [float(value) for value in k_values]

        r_values = payload.get("r")
        p_values = payload.get("p")
        message.r = [float(value) for value in r_values] if isinstance(r_values, list) and len(r_values) == 9 else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [float(value) for value in p_values] if isinstance(p_values, list) and len(p_values) == 12 else [
            message.k[0],
            0.0,
            message.k[2],
            0.0,
            0.0,
            message.k[4],
            message.k[5],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return message

    def _log_camera_info_published_once_or_changed(self, payload: dict[str, object]) -> None:
        summary = camera_info_payload_summary(payload)
        signature = (
            summary["width"],
            summary["height"],
            summary["rgb_camera_count"],
            summary["fx"],
            summary["fy"],
            summary["cx"],
            summary["cy"],
            summary["distortion_model"],
        )
        if signature == self._last_camera_info_signature:
            return
        self._last_camera_info_signature = signature
        self.get_logger().info(json.dumps({"event": "camera_info_published", **summary}, ensure_ascii=False))

    def _publish_latest_rgb_frame(self) -> None:
        with self._frame_lock:
            if self._latest_rgb_frame is None:
                return
            header, image_bgr = self._latest_rgb_frame
            self._latest_rgb_frame = None

        image_message = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        if not self._publish_common_header(image_message, header):
            self.get_logger().warning(json.dumps({"event": "frame_timestamp_invalid", "frame_type": "rgb"}, ensure_ascii=False))
            return
        camera_info_message = self._camera_info_from_header(header, image_bgr)
        if camera_info_message is None:
            return
        self.camera_info_publisher.publish(camera_info_message)
        self.image_publisher.publish(image_message)
        payload = header.get("camera_info")
        if isinstance(payload, dict):
            self._log_camera_info_published_once_or_changed(payload)

    def _publish_latest_depth_frame(self) -> None:
        with self._frame_lock:
            if self._latest_depth_frame is None:
                return
            header, depth_map = self._latest_depth_frame
            self._latest_depth_frame = None

        depth_message = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
        if not self._publish_common_header(depth_message, header):
            self.get_logger().warning(
                json.dumps({"event": "frame_timestamp_invalid", "frame_type": "aligned_depth"}, ensure_ascii=False)
            )
            return
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
