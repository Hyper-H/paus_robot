"""Optional adapter for existing bags; no timestamps or pixels are changed."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from .node import IMAGE_QOS


class FrameBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("thyroid_frame_bridge", **kwargs)
        try:
            from paus_interfaces.msg import UltrasoundFrame
        except ImportError as exc:
            raise RuntimeError(
                "UltrasoundFrame is absent on baseline dev. Source the built ultrasound "
                "worktree overlay before using this optional bag bridge.") from exc
        self.declare_parameter("frame_topic", "/ultrasound/us/frame")
        self.declare_parameter("image_topic", "/ultrasound/us/image_raw")
        publisher = self.create_publisher(
            Image, self.get_parameter("image_topic").value, IMAGE_QOS)
        self.create_subscription(UltrasoundFrame, self.get_parameter("frame_topic").value,
                                 lambda message: publisher.publish(message.image), IMAGE_QOS)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FrameBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
