from __future__ import annotations

# 导入 json，用于发布结构化状态。
import json
# 导入 dataclass，便于保存单次标定样本。
from dataclasses import dataclass
# 导入 Path，便于处理输出路径。
from pathlib import Path

# 导入 OpenCV 与 NumPy，用于棋盘格检测和矩阵运算。
import cv2
import numpy as np
# 导入 ament 索引，用于定位默认配置目录。
from ament_index_python.packages import get_package_share_directory
# 导入 cv_bridge，用于 ROS Image 转 OpenCV 图像。
from cv_bridge import CvBridge
# 导入 ROS2 Python API。
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# 导入消息与服务类型。
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

# 导入项目中的相机标定读取与坐标变换工具。
from paus_motion_ros2 import FairinoLinuxClient
from paus_perception import (
    EyeToHandCalibrationSolution,
    average_transform_matrices,
    invert_transform_matrix,
    load_camera_calibration,
    load_config,
    make_transform_matrix,
    make_transform_struct,
    rpy_deg_to_rotation_matrix,
    save_eye_to_hand_solution,
)


# 保存一次标定采样的结果。
@dataclass
class CalibrationSample:
    # 当前样本估计出的 `base -> camera` 齐次矩阵。
    base_to_camera_matrix: np.ndarray


# 这个节点负责采集 eye-to-hand 标定样本，并求解 `base -> camera` 外参。
class EyeToHandCalibrationNode(Node):
    # 初始化节点。
    def __init__(self) -> None:
        # 注册节点名字。
        super().__init__("eye_to_hand_calibration_node")
        # 找到 bringup 包安装目录，方便给输出路径默认值。
        bringup_share = Path(get_package_share_directory("paus_bringup"))
        default_config_path = bringup_share / "configs" / "default.yaml"

        # 声明节点参数。
        self.declare_parameter("config_path", str(default_config_path))
        self.declare_parameter("camera_config_path", "/tmp/paus_robot/camera.yaml")
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("status_topic", "/eye_to_hand/status")
        self.declare_parameter("board_rows", 6)
        self.declare_parameter("board_cols", 9)
        self.declare_parameter("square_size_m", 0.01)
        self.declare_parameter("tool_to_board.translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("tool_to_board.rotation_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("min_sample_count", 10)
        self.declare_parameter("output_path", str(bringup_share / "configs" / "extrinsics.yaml"))

        # 读取参数值。
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.config = load_config(self.config_path)
        control_cfg = self.config["control"]
        # 在拿到主配置后，再声明机器人相关参数，允许后续显式覆盖。
        self.declare_parameter("robot_ip", str(control_cfg["robot_ip"]))
        self.declare_parameter("linux_fairino_sdk_root", str(control_cfg["linux_fairino_sdk_root"]))

        camera_config_path = self.get_parameter("camera_config_path").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.board_rows = int(self.get_parameter("board_rows").get_parameter_value().integer_value)
        self.board_cols = int(self.get_parameter("board_cols").get_parameter_value().integer_value)
        self.square_size_m = float(self.get_parameter("square_size_m").get_parameter_value().double_value)
        self.tool_to_board_translation = [float(value) for value in self.get_parameter("tool_to_board.translation_m").get_parameter_value().double_array_value]
        self.tool_to_board_rotation_rpy = [float(value) for value in self.get_parameter("tool_to_board.rotation_rpy_deg").get_parameter_value().double_array_value]
        self.min_sample_count = int(self.get_parameter("min_sample_count").get_parameter_value().integer_value)
        self.output_path = Path(self.get_parameter("output_path").get_parameter_value().string_value)
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.linux_fairino_sdk_root = self.get_parameter("linux_fairino_sdk_root").get_parameter_value().string_value

        # 标定节点必须有相机内参文件，否则没法 solvePnP。
        if not camera_config_path:
            raise RuntimeError("camera_config_path is required for eye-to-hand calibration.")
        # 加载相机内参。
        self.camera_calibration = load_camera_calibration(camera_config_path)

        # 创建图像桥接器。
        self.bridge = CvBridge()
        # 缓存最新图像。
        self.latest_image_bgr: np.ndarray | None = None
        # 保存已采集样本和当前求解结果。
        self.samples: list[CalibrationSample] = []
        self.current_solution: EyeToHandCalibrationSolution | None = None
        # 建立 Linux SDK 客户端，用于直接读取当前 TCP。
        self.linux_client = FairinoLinuxClient(self.linux_fairino_sdk_root, self.robot_ip)
        self.linux_client.connect()

        # 创建图像订阅器与状态发布器。
        self.image_subscription = self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)

        # 创建“采样 / 求解 / 保存”三个服务接口。
        self.capture_service = self.create_service(Trigger, "/eye_to_hand/capture_sample", self._capture_sample_callback)
        self.solve_service = self.create_service(Trigger, "/eye_to_hand/solve", self._solve_callback)
        self.save_service = self.create_service(Trigger, "/eye_to_hand/save", self._save_callback)

        # 节点启动后发布初始状态。
        self._publish_status("ready", "Eye-to-hand calibration node started.")

    # 接收最新图像。
    def _image_callback(self, message: Image) -> None:
        self.latest_image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")

    # 发布标定状态。
    def _publish_status(self, status: str, message: str, extra: dict[str, object] | None = None) -> None:
        payload = {
            "status": status,
            "message": message,
            "sample_count": len(self.samples),
            "min_sample_count": self.min_sample_count,
        }
        if extra:
            payload.update(extra)
        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)
        self.get_logger().info(status_message.data)

    # 构造棋盘格的三维角点模板。
    def _build_board_object_points(self) -> np.ndarray:
        object_points = np.zeros((self.board_rows * self.board_cols, 3), np.float32)
        object_points[:, :2] = np.mgrid[0:self.board_cols, 0:self.board_rows].T.reshape(-1, 2)
        object_points *= self.square_size_m
        return object_points

    # 从最新图像中估计 `camera -> board` 变换。
    def _estimate_camera_to_board(self) -> np.ndarray:
        if self.latest_image_bgr is None:
            raise RuntimeError("No image has been received yet.")

        # 先转成灰度图，再做棋盘格检测。
        gray = cv2.cvtColor(self.latest_image_bgr, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (self.board_cols, self.board_rows))
        if not found:
            raise RuntimeError("Chessboard was not detected in the latest image.")

        # 对角点做亚像素优化。
        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        # 准备 solvePnP 所需的三维点、相机内参和畸变参数。
        object_points = self._build_board_object_points()
        camera_matrix = np.asarray(self.camera_calibration.camera_matrix, dtype=np.float64)
        dist_coeffs = np.asarray(self.camera_calibration.dist_coeffs, dtype=np.float64)
        # 使用 solvePnP 求解相机到棋盘格的位姿。
        success, rvec, tvec = cv2.solvePnP(object_points, refined, camera_matrix, dist_coeffs)
        if not success:
            raise RuntimeError("solvePnP failed for the calibration board.")
        # 将 Rodrigues 旋转向量转成旋转矩阵，再组装成齐次矩阵。
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return make_transform_matrix(tvec.reshape(3), rotation_matrix)

    # 通过 Linux SDK 直接读取 `base -> tool(TCP)` 位姿。
    def _current_base_to_tool(self) -> np.ndarray:
        error, tcp_pose_mmdeg = self.linux_client.get_actual_tcp_pose()
        if error != 0:
            raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
        # SDK 返回单位是 mm / deg，而手眼矩阵这里统一按 m 计算。
        translation_m = [float(value) / 1000.0 for value in tcp_pose_mmdeg[:3]]
        rotation_matrix = rpy_deg_to_rotation_matrix(tcp_pose_mmdeg[3:6])
        return make_transform_matrix(translation_m, rotation_matrix)

    # 根据固定的工具安装关系，构造 `tool -> board` 变换矩阵。
    def _tool_to_board_matrix(self) -> np.ndarray:
        rotation_matrix = rpy_deg_to_rotation_matrix(self.tool_to_board_rotation_rpy)
        return make_transform_matrix(self.tool_to_board_translation, rotation_matrix)

    # 采集一次样本。
    def _capture_sample_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            # 读取当前 `base -> tool`、估计 `camera -> board`，并准备固定的 `tool -> board`。
            base_to_tool = self._current_base_to_tool()
            camera_to_board = self._estimate_camera_to_board()
            tool_to_board = self._tool_to_board_matrix()
            # 按链路公式反推 `base -> camera`。
            base_to_camera = base_to_tool @ tool_to_board @ invert_transform_matrix(camera_to_board)
            # 保存当前样本。
            self.samples.append(CalibrationSample(base_to_camera_matrix=base_to_camera))
            response.success = True
            response.message = f"Captured sample #{len(self.samples)}."
            self._publish_status(
                "sample_captured",
                response.message,
                {
                    "robot_ip": self.robot_ip,
                    "sdk_root": self.linux_fairino_sdk_root,
                },
            )
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("capture_failed", response.message)
        return response

    # 用已有样本求解外参。
    def _solve_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        # 样本不足时直接拒绝求解。
        if len(self.samples) < self.min_sample_count:
            response.success = False
            response.message = f"Need at least {self.min_sample_count} samples, currently {len(self.samples)}."
            self._publish_status("solve_failed", response.message)
            return response
        try:
            # 对所有样本矩阵做平均，得到稳定的 `base -> camera`。
            matrices = [sample.base_to_camera_matrix for sample in self.samples]
            averaged = average_transform_matrices(matrices)
            translation = averaged[:3, 3]
            rotation = averaged[:3, :3]
            transform = make_transform_struct(translation, rotation, "robot_base", "camera")
            # 保存当前求解结果。
            self.current_solution = EyeToHandCalibrationSolution(
                status="ok",
                success=True,
                sample_count=len(self.samples),
                message="Eye-to-hand calibration solved successfully.",
                base_to_camera=transform,
                tool_to_board_translation_m=list(self.tool_to_board_translation),
                tool_to_board_rotation_rpy_deg=list(self.tool_to_board_rotation_rpy),
            )
            response.success = True
            response.message = self.current_solution.message
            self._publish_status("solved", response.message)
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("solve_failed", response.message)
        return response

    # 将当前求解结果保存成外参文件。
    def _save_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        # 如果还没有求解成功，就不允许保存。
        if self.current_solution is None or not self.current_solution.success:
            response.success = False
            response.message = "No solved extrinsic is available."
            self._publish_status("save_failed", response.message)
            return response
        try:
            # 写出到配置文件。
            save_eye_to_hand_solution(self.current_solution, self.output_path)
            response.success = True
            response.message = f"Saved extrinsic to {self.output_path}."
            self._publish_status("saved", response.message, {"output_path": str(self.output_path)})
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("save_failed", response.message)
        return response


# ROS2 节点入口。
def main(args: list[str] | None = None) -> None:
    # 初始化 ROS2。
    rclpy.init(args=args)
    # 创建节点实例。
    node = EyeToHandCalibrationNode()
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
