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
# 导入 cv_bridge，用于 ROS 图像与 OpenCV 图像互转。
from cv_bridge import CvBridge
# 导入 ROS2 Python API。
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# 导入图像消息类型。
from sensor_msgs.msg import Image

# 导入项目里的桥接协议读取函数。
from paus_perception import recv_frame_packet


# 这个节点负责接收 `camera_bridge.py` 通过 TCP 发来的 JPEG 图像，
# 再把它们转成 ROS2 的 `/camera/image_bridge`。
class ImageReceiverNode(Node):
    # 节点初始化。
    def __init__(self) -> None:
        # 注册节点名字。
        super().__init__("image_receiver_node")

        # 声明监听地址、端口和输出 topic 等参数。
        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 5001)
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("frame_id", "camera")

        # 读取参数值。
        self.listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # 创建图像桥接器和 ROS 图像发布器。
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)

        # 使用一份“最新帧缓存”在后台线程和 ROS 主线程之间交接数据。
        self._latest_frame: tuple[dict[str, object], np.ndarray] | None = None
        self._frame_lock = threading.Lock()

        # 启动后台 TCP 服务线程。
        self._server_thread = threading.Thread(target=self._serve_loop, daemon=True)
        self._server_thread.start()
        # 用一个定时器周期性地把缓存中的最新帧发布成 ROS 消息。
        self.create_timer(0.01, self._publish_latest_frame)

        # 输出节点启动日志。
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
        # 创建监听 socket，等待 `camera_bridge.py` 连接。
        with socket.create_server((self.listen_host, int(self.listen_port)), reuse_port=False) as server:
            while rclpy.ok():
                # 接受一条新的 TCP 连接。
                connection, address = server.accept()
                self.get_logger().info(json.dumps({"event": "bridge_connected", "peer": address[0], "port": address[1]}, ensure_ascii=False))
                with connection:
                    while rclpy.ok():
                        try:
                            # 从桥接协议里读取一帧头部和负载。
                            header, payload = recv_frame_packet(connection)
                        except Exception as exc:
                            # 一旦连接断开，就回到 accept 等待下一次重连。
                            self.get_logger().warning(json.dumps({"event": "bridge_disconnected", "reason": repr(exc)}, ensure_ascii=False))
                            break
                        # 当前主线只处理 RGB JPEG 图像。
                        if header.get("frame_type") != "rgb" or header.get("encoding") != "jpeg":
                            continue
                        # 将 JPEG 二进制解码成 OpenCV BGR 图像。
                        image_array = np.frombuffer(payload, dtype=np.uint8)
                        image_bgr = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                        if image_bgr is None:
                            continue
                        # 更新“最新帧缓存”，让主线程稍后发布它。
                        with self._frame_lock:
                            self._latest_frame = (header, image_bgr)

    # 将最新缓存帧转换成 ROS 图像消息并发布。
    def _publish_latest_frame(self) -> None:
        # 先安全地取出缓存中的最新帧。
        with self._frame_lock:
            if self._latest_frame is None:
                return
            header, image_bgr = self._latest_frame
            self._latest_frame = None

        # 把 OpenCV 图像转换成 ROS Image。
        image_message = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        # 设置 frame_id。
        image_message.header.frame_id = str(header.get("frame_id", self.default_frame_id))
        # 如果头部里带有时间戳，则同步写入 ROS 消息头。
        timestamp_ns = header.get("timestamp_ns")
        if isinstance(timestamp_ns, int):
            image_message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
            image_message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        # 发布图像。
        self.image_publisher.publish(image_message)


# ROS2 节点入口。
def main(args: list[str] | None = None) -> None:
    # 初始化 ROS2。
    rclpy.init(args=args)
    # 创建节点实例。
    node = ImageReceiverNode()
    try:
        # 进入事件循环。
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        # 退出时销毁节点并关闭 ROS2。
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
