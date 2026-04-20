from __future__ import annotations

# 导入 json，用于发布状态文本时保留结构。
import json
from pathlib import Path

# 导入 ament 索引工具，用于定位包内资源。
from ament_index_python.packages import get_package_share_directory
# 导入 cv_bridge，用于 ROS 图像和 OpenCV 图像互转。
from cv_bridge import CvBridge
# 导入标准几何消息。
from geometry_msgs.msg import PointStamped, PoseStamped
# 导入 rclpy 和节点基类。
import rclpy
from rclpy.node import Node
# 导入图像消息和字符串消息。
from sensor_msgs.msg import Image
from std_msgs.msg import String

# 导入项目现有的核心函数。
from ag_repro import load_camera_calibration, load_config, process_image_array, render_visualization
# 导入 ROS2 相关的转换辅助函数。
from .conversions import build_status_payload, rvec_to_quaternion


# ROS2 节点：订阅图像并发布 marker 位姿与接近目标点。
class MarkerPoseNode(Node):
    # 节点初始化。
    def __init__(self) -> None:
        # 调用父类构造函数，注册节点名。
        super().__init__("marker_pose_node")
        # 定位当前 ROS2 包的 share 目录。
        package_share = Path(get_package_share_directory("ag_marker_ros2"))

        # 声明节点参数。
        self.declare_parameter("config_path", str(package_share / "configs" / "default.yaml"))
        self.declare_parameter("camera_config_path", "")
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("marker_pose_topic", "/marker_pose")
        self.declare_parameter("approach_target_topic", "/approach_target")
        self.declare_parameter("status_topic", "/detection_status")
        self.declare_parameter("debug_image_topic", "/marker_debug_image")
        self.declare_parameter("publish_debug_image", False)

        # 读取参数值。
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        camera_config_path = self.get_parameter("camera_config_path").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        marker_pose_topic = self.get_parameter("marker_pose_topic").get_parameter_value().string_value
        approach_target_topic = self.get_parameter("approach_target_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        debug_image_topic = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        self.publish_debug_image = self.get_parameter("publish_debug_image").get_parameter_value().bool_value

        # 加载项目配置与相机标定。
        self.config = load_config(config_path)
        self.camera_calibration = load_camera_calibration(camera_config_path) if camera_config_path else None
        # 创建图像桥接器。
        self.bridge = CvBridge()

        # 创建订阅者与发布者。
        self.image_subscription = self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.marker_pose_publisher = self.create_publisher(PoseStamped, marker_pose_topic, 10)
        self.approach_target_publisher = self.create_publisher(PointStamped, approach_target_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.debug_image_publisher = self.create_publisher(Image, debug_image_topic, 10) if self.publish_debug_image else None

        # 打印启动信息。
        self.get_logger().info(
            json.dumps(
                {
                    "event": "node_started",
                    "image_topic": image_topic,
                    "marker_pose_topic": marker_pose_topic,
                    "approach_target_topic": approach_target_topic,
                    "status_topic": status_topic,
                    "camera_config_path": camera_config_path,
                },
                ensure_ascii=False,
            )
        )

    # 图像回调函数：每收到一帧图像就执行检测和发布。
    def _image_callback(self, message: Image) -> None:
        # 将 ROS 图像消息转换为 OpenCV BGR 图像。
        image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        # 调用现有内存版处理函数。
        result = process_image_array(image_bgr, self.config, camera_calibration=self.camera_calibration)

        # 始终发布状态文本。
        status_message = String()
        status_message.data = build_status_payload(result)
        self.status_publisher.publish(status_message)

        # 若有可用 pose，则发布 PoseStamped。
        if result.pose is not None and result.pose.status == "ok":
            pose_message = PoseStamped()
            pose_message.header = message.header
            pose_message.header.frame_id = result.pose.frame
            pose_message.pose.position.x = result.pose.tvec[0]
            pose_message.pose.position.y = result.pose.tvec[1]
            pose_message.pose.position.z = result.pose.tvec[2]
            quaternion = rvec_to_quaternion(result.pose.rvec)
            pose_message.pose.orientation.x = quaternion[0]
            pose_message.pose.orientation.y = quaternion[1]
            pose_message.pose.orientation.z = quaternion[2]
            pose_message.pose.orientation.w = quaternion[3]
            self.marker_pose_publisher.publish(pose_message)

        # 若有可用的接近目标点，则发布 PointStamped。
        if result.approach_plan is not None and result.approach_plan.target_status == "ok":
            point_message = PointStamped()
            point_message.header = message.header
            point_message.header.frame_id = result.approach_plan.target_frame
            point_message.point.x = result.approach_plan.target_point[0]
            point_message.point.y = result.approach_plan.target_point[1]
            point_message.point.z = result.approach_plan.target_point[2]
            self.approach_target_publisher.publish(point_message)

        # 若开启调试图发布，则发布叠加后的可视化图。
        if self.debug_image_publisher is not None:
            debug_image = render_visualization(image_bgr, result)
            debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_message.header = message.header
            self.debug_image_publisher.publish(debug_message)


# ROS2 脚本入口。
def main(args: list[str] | None = None) -> None:
    # 初始化 ROS2。
    rclpy.init(args=args)
    # 创建节点实例。
    node = MarkerPoseNode()
    try:
        # 进入循环，持续处理回调。
        rclpy.spin(node)
    finally:
        # 关闭节点并退出 ROS2。
        node.destroy_node()
        rclpy.shutdown()
