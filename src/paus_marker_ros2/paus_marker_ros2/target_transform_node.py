from __future__ import annotations

# 导入 json，用于发布结构化状态。
import json

# 导入 ament 索引，用于定位 bringup 包内配置文件。
from ament_index_python.packages import get_package_share_directory
# 导入 ROS2 Python API。
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# 导入几何消息和状态消息。
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

# 导入感知核心库中的配置、外参和变换工具。
from paus_perception import (
    load_config,
    load_eye_to_hand_solution,
    make_transform_matrix,
    make_transform_struct,
    quaternion_xyzw_to_rotation_matrix,
    rpy_deg_to_rotation_matrix,
    split_transform_matrix,
)


# 这个节点负责把相机系下的 marker 位姿变换到机器人基座系，
# 输出 `/target_pose_base` 和 `/target_point_base`。
class TargetTransformNode(Node):
    # 初始化节点。
    def __init__(self) -> None:
        # 注册节点名字。
        super().__init__("target_transform_node")

        # 找到 bringup 包安装目录，用来加载默认配置和外参。
        bringup_share = get_package_share_directory("paus_bringup")
        # 声明参数。
        self.declare_parameter("config_path", f"{bringup_share}/configs/default.yaml")
        self.declare_parameter("extrinsics_path", f"{bringup_share}/configs/extrinsics.yaml")
        self.declare_parameter("marker_pose_topic", "/marker_pose")
        self.declare_parameter("target_pose_topic", "/target_pose_base")
        self.declare_parameter("target_point_topic", "/target_point_base")
        self.declare_parameter("status_topic", "/transform_status")

        # 读取参数值。
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        extrinsics_path = self.get_parameter("extrinsics_path").get_parameter_value().string_value
        marker_pose_topic = self.get_parameter("marker_pose_topic").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        target_point_topic = self.get_parameter("target_point_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value

        # 加载主配置与外参。
        self.config = load_config(config_path)
        self.eye_to_hand_solution = load_eye_to_hand_solution(extrinsics_path)
        # 创建状态、目标位姿和目标点发布器。
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.target_pose_publisher = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.target_point_publisher = self.create_publisher(PointStamped, target_point_topic, 10)
        # 订阅 marker 位姿。
        self.subscription = self.create_subscription(PoseStamped, marker_pose_topic, self._marker_pose_callback, 10)

        # 从配置中读取 `marker -> target` 的固定偏移变换。
        marker_to_target_cfg = self.config["transform"]["marker_to_target"]
        self.marker_to_target = make_transform_matrix(
            marker_to_target_cfg["translation_m"],
            rpy_deg_to_rotation_matrix(marker_to_target_cfg["rotation_rpy_deg"]),
        )

        # 节点启动后发布一次初始状态。
        self._publish_status(
            "ready" if self.eye_to_hand_solution.success else "missing_extrinsic",
            self.eye_to_hand_solution.message,
        )

    # 发布变换状态。
    def _publish_status(self, status: str, message: str, extra: dict[str, object] | None = None) -> None:
        payload = {"status": status, "message": message}
        if extra:
            payload.update(extra)
        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)

    # marker 位姿回调：把 `camera -> marker` 变成 `robot_base -> target_region`。
    def _marker_pose_callback(self, message: PoseStamped) -> None:
        # 如果还没有成功加载外参，则无法继续变换。
        if not self.eye_to_hand_solution.success or self.eye_to_hand_solution.base_to_camera is None:
            self._publish_status("missing_extrinsic", "base_to_camera is unavailable.")
            return

        # 将消息中的 marker 位姿转成齐次矩阵。
        camera_to_marker = make_transform_matrix(
            [message.pose.position.x, message.pose.position.y, message.pose.position.z],
            quaternion_xyzw_to_rotation_matrix(
                [
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ]
            ),
        )
        # 组装 `base -> camera` 变换矩阵。
        base_to_camera = make_transform_matrix(
            self.eye_to_hand_solution.base_to_camera.translation_m,
            self.eye_to_hand_solution.base_to_camera.rotation_matrix,
        )
        # 完整链路：base -> camera -> marker -> target。
        base_to_target = base_to_camera @ camera_to_marker @ self.marker_to_target
        # 拆回平移和旋转。
        translation, rotation = split_transform_matrix(base_to_target)
        transform = make_transform_struct(translation, rotation, "robot_base", "target_region")

        # 发布目标位姿消息。
        pose_message = PoseStamped()
        pose_message.header = message.header
        pose_message.header.frame_id = "robot_base"
        pose_message.pose.position.x = transform.translation_m[0]
        pose_message.pose.position.y = transform.translation_m[1]
        pose_message.pose.position.z = transform.translation_m[2]
        pose_message.pose.orientation.x = transform.rotation_quaternion_xyzw[0]
        pose_message.pose.orientation.y = transform.rotation_quaternion_xyzw[1]
        pose_message.pose.orientation.z = transform.rotation_quaternion_xyzw[2]
        pose_message.pose.orientation.w = transform.rotation_quaternion_xyzw[3]
        self.target_pose_publisher.publish(pose_message)

        # 发布目标点消息。
        point_message = PointStamped()
        point_message.header = pose_message.header
        point_message.point.x = transform.translation_m[0]
        point_message.point.y = transform.translation_m[1]
        point_message.point.z = transform.translation_m[2]
        self.target_point_publisher.publish(point_message)

        # 发布一条“变换成功”的状态。
        self._publish_status(
            "ok",
            "Target transform published successfully.",
            {"target_point_base": transform.translation_m},
        )


# ROS2 节点入口。
def main(args: list[str] | None = None) -> None:
    # 初始化 ROS2。
    rclpy.init(args=args)
    # 创建节点实例。
    node = TargetTransformNode()
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
