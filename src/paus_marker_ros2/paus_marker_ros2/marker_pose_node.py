from __future__ import annotations

# 导入 json，用于输出结构化状态日志。
import json
# 导入 Path，便于处理配置文件路径。
from pathlib import Path

# 导入 ament 索引，用于定位 bringup 包内默认配置。
from ament_index_python.packages import get_package_share_directory
# 导入 cv_bridge，用于 ROS 图像与 OpenCV 图像互转。
from cv_bridge import CvBridge
# 导入 ROS 常用几何消息类型。
from geometry_msgs.msg import PointStamped, PoseStamped
# 导入 ROS2 Python API。
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# 导入图像消息和字符串消息。
from sensor_msgs.msg import Image
from std_msgs.msg import String

# 导入感知核心库中的相机标定、配置、图像处理与可视化函数。
from paus_perception import load_camera_calibration, load_config, process_image_array, render_visualization
# 导入 ROS2 内部的消息转换辅助函数。
from .conversions import build_status_payload, rvec_to_quaternion


# 这个节点负责把 `/camera/image_bridge` 图像转换成：
# 1. marker 位姿 `/marker_pose`
# 2. 相机系接近目标点 `/approach_target`
# 3. 检测状态 `/detection_status`
class MarkerPoseNode(Node):
    # 初始化节点。
    def __init__(self) -> None:
        # 注册节点名字。
        super().__init__("marker_pose_node")
        # 找到 bringup 包安装目录，用于拿默认配置。
        bringup_share = Path(get_package_share_directory("paus_bringup"))

        # 声明节点参数。
        self.declare_parameter("config_path", str(bringup_share / "configs" / "default.yaml"))
        self.declare_parameter("camera_config_path", "/tmp/paus_robot/camera.yaml")
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

        # 加载主配置。
        self.config = load_config(config_path)
        # 记录相机标定文件路径；如果文件暂时不存在，后面会在回调里继续等它。
        self.camera_config_path = Path(camera_config_path) if camera_config_path else None
        self._camera_config_warning_emitted = False
        # 尝试立即加载一次相机标定。
        self.camera_calibration = self._try_load_camera_calibration()
        # 创建图像桥接器。
        self.bridge = CvBridge()

        # 创建订阅器和发布器。
        self.image_subscription = self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.marker_pose_publisher = self.create_publisher(PoseStamped, marker_pose_topic, 10)
        self.approach_target_publisher = self.create_publisher(PointStamped, approach_target_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.debug_image_publisher = self.create_publisher(Image, debug_image_topic, 10) if self.publish_debug_image else None

        # 输出启动摘要。
        self.get_logger().info(
            json.dumps(
                {
                    "event": "node_started",
                    "image_topic": image_topic,
                    "marker_pose_topic": marker_pose_topic,
                    "approach_target_topic": approach_target_topic,
                    "status_topic": status_topic,
                    "camera_config_path": str(self.camera_config_path) if self.camera_config_path else "",
                },
                ensure_ascii=False,
            )
        )

    # 尝试加载相机内参文件。
    # 如果 `camera.yaml` 还没被相机桥接脚本写出来，这里会返回 None。
    def _try_load_camera_calibration(self):
        if self.camera_config_path is None:
            return None
        if not self.camera_config_path.exists():
            # 只在第一次缺失时打印警告，避免刷屏。
            if not self._camera_config_warning_emitted:
                self.get_logger().warning(
                    json.dumps(
                        {
                            "event": "camera_config_waiting",
                            "camera_config_path": str(self.camera_config_path),
                        },
                        ensure_ascii=False,
                    )
                )
                self._camera_config_warning_emitted = True
            return None
        # 文件一旦存在，就真正加载相机内参。
        calibration = load_camera_calibration(self.camera_config_path)
        self._camera_config_warning_emitted = False
        return calibration

    # 图像回调：每收到一帧图像，就跑一次完整检测与发布流程。
    def _image_callback(self, message: Image) -> None:
        # 将 ROS Image 转为 OpenCV BGR 图像。
        image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        # 如果之前还没拿到 camera.yaml，这里继续尝试加载。
        if self.camera_calibration is None:
            self.camera_calibration = self._try_load_camera_calibration()
        # 调用感知核心库执行完整流水线。
        result = process_image_array(image_bgr, self.config, camera_calibration=self.camera_calibration)

        # 无论检测是否成功，都发布一份状态 JSON。
        status_message = String()
        status_message.data = build_status_payload(result)
        self.status_publisher.publish(status_message)

        # 如果成功估计出 marker 位姿，则发布 `/marker_pose`。
        if result.pose is not None and result.pose.status == "ok":
            pose_message = PoseStamped()
            pose_message.header = message.header
            pose_message.header.frame_id = result.pose.frame
            # 写入平移部分。
            pose_message.pose.position.x = result.pose.tvec[0]
            pose_message.pose.position.y = result.pose.tvec[1]
            pose_message.pose.position.z = result.pose.tvec[2]
            # 将 rvec 变成四元数并写入姿态。
            quaternion = rvec_to_quaternion(result.pose.rvec)
            pose_message.pose.orientation.x = quaternion[0]
            pose_message.pose.orientation.y = quaternion[1]
            pose_message.pose.orientation.z = quaternion[2]
            pose_message.pose.orientation.w = quaternion[3]
            self.marker_pose_publisher.publish(pose_message)

        # 如果流水线已经给出了接近目标点，则同步发布 `/approach_target`。
        if result.approach_plan is not None and result.approach_plan.target_status == "ok":
            point_message = PointStamped()
            point_message.header = message.header
            point_message.header.frame_id = result.approach_plan.target_frame
            point_message.point.x = result.approach_plan.target_point[0]
            point_message.point.y = result.approach_plan.target_point[1]
            point_message.point.z = result.approach_plan.target_point[2]
            self.approach_target_publisher.publish(point_message)

        # 如果开启调试图发布，则把叠加可视化结果发出去。
        if self.debug_image_publisher is not None:
            debug_image = render_visualization(image_bgr, result)
            debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_message.header = message.header
            self.debug_image_publisher.publish(debug_message)


# ROS2 节点入口。
def main(args: list[str] | None = None) -> None:
    # 初始化 ROS2。
    rclpy.init(args=args)
    # 创建节点实例。
    node = MarkerPoseNode()
    try:
        # 进入事件循环。
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        # 退出前销毁节点并关闭 ROS2。
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
