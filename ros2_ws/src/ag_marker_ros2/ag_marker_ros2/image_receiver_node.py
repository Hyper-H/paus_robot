from __future__ import annotations

# 导入 json，用于记录结构化日志。
import json
# 导入 socket，用于监听 TCP 图像桥接。
import socket
# 导入 threading，用于后台接收线程。
import threading

# 导入 OpenCV 和 NumPy，用于解码 JPEG 图像。
import cv2
import numpy as np
# 导入 cv_bridge，用于 ROS 图像消息转换。
from cv_bridge import CvBridge
# 导入 ROS2。
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# 导入图像消息。
from sensor_msgs.msg import Image

# 导入桥接协议读取函数。
from ag_repro import recv_frame_packet


# ROS2 节点：接收 TCP 图像桥接并发布 ROS 图像 topic。
class ImageReceiverNode(Node):
    # 初始化节点。
    def __init__(self) -> None:
        super().__init__("image_receiver_node")

        # 声明节点参数。
        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 5001)
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("frame_id", "camera")

        # 读取参数值。
        self.listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # 创建图像桥接器和发布者。
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)

        # 最近一帧缓存及锁。
        self._latest_frame: tuple[dict[str, object], np.ndarray] | None = None
        self._frame_lock = threading.Lock()

        # 启动后台 TCP 接收线程。
        self._server_thread = threading.Thread(target=self._serve_loop, daemon=True)
        self._server_thread.start()
        # 用定时器把后台接收的最新帧发布成 ROS2 图像。
        self.create_timer(0.01, self._publish_latest_frame)

        # 打印启动日志。
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

    # 后台 TCP 服务主循环。
    def _serve_loop(self) -> None:
        # 创建 TCP 监听 socket。
        with socket.create_server((self.listen_host, int(self.listen_port)), reuse_port=False) as server:
            while rclpy.ok():
                # 接受连接。
                connection, address = server.accept()
                self.get_logger().info(json.dumps({"event": "bridge_connected", "peer": address[0], "port": address[1]}, ensure_ascii=False))
                # 处理当前连接，断开后再回到 accept。
                with connection:
                    while rclpy.ok():
                        try:
                            header, payload = recv_frame_packet(connection)
                        except Exception as exc:
                            self.get_logger().warning(json.dumps({"event": "bridge_disconnected", "reason": repr(exc)}, ensure_ascii=False))
                            break
                        # 当前阶段只接收 RGB JPEG 图像。
                        if header.get("frame_type") != "rgb" or header.get("encoding") != "jpeg":
                            continue
                        # 解码 JPEG。
                        image_array = np.frombuffer(payload, dtype=np.uint8)
                        image_bgr = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                        if image_bgr is None:
                            continue
                        # 更新最新帧缓存。
                        with self._frame_lock:
                            self._latest_frame = (header, image_bgr)

    # 发布最新一帧图像。
    def _publish_latest_frame(self) -> None:
        # 取出当前最新帧。
        with self._frame_lock:
            if self._latest_frame is None:
                return
            header, image_bgr = self._latest_frame
            self._latest_frame = None

        # 转成 ROS 图像消息。
        image_message = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        # 设置坐标系名字。
        image_message.header.frame_id = str(header.get("frame_id", self.default_frame_id))
        # 若携带时间戳，则按纳秒写入消息头。
        timestamp_ns = header.get("timestamp_ns")
        if isinstance(timestamp_ns, int):
            image_message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            image_message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        # 发布图像。
        self.image_publisher.publish(image_message)


# ROS2 节点入口。
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
