from __future__ import annotations

# 导入 json，用于发布结构化状态。
import json
# 导入 dataclass，便于保存单次标定样本。
from dataclasses import asdict, dataclass
# 导入 Path，便于处理输出路径。
from pathlib import Path
import threading
import time

# 导入 OpenCV 与 NumPy，用于棋盘格检测和矩阵运算。
import cv2
import numpy as np
# 导入 ament 索引，用于定位默认配置目录。(ROS2 的“包注册表 + 查找器”)
from ament_index_python.packages import get_package_share_directory
# 导入 cv_bridge，用于 ROS Image 转 OpenCV 图像。
from cv_bridge import CvBridge
# 导入 ROS2 Python API。
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
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
    evaluate_eye_to_hand_residuals,
    invert_transform_matrix,
    load_camera_calibration,
    load_config,
    make_transform_matrix,
    make_transform_struct,
    rpy_deg_to_rotation_matrix,
    save_eye_to_hand_solution,
    solve_eye_to_hand_opencv_handeye,
    solve_ax_xb_hand_eye_park,
)


# 保存一次标定采样的结果。
@dataclass
class CalibrationSample:
    # 当前样本对应的 `base -> tool` 齐次矩阵。
    base_to_tool_matrix: np.ndarray
    # 当前样本对应的 `camera -> board` 齐次矩阵。
    camera_to_board_matrix: np.ndarray
    # 图像消息 header 时间，单位秒；如果上游未填 header，则为 None。
    image_header_time_s: float | None
    # 本节点收到图像的单调时钟时间，单位秒。
    image_received_time_s: float
    # TCP 读取开始和结束的单调时钟时间，单位秒。
    tcp_read_start_time_s: float
    tcp_read_end_time_s: float
    # SDK 返回的原始 TCP 位姿，单位 mm / deg。
    tcp_pose_mmdeg: list[float]
    # 本节点内部图像序号，用来确认服务调用后确实等到了新帧。
    image_sequence: int


@dataclass
class CapturedImage:
    image_bgr: np.ndarray
    header_time_s: float | None
    received_time_s: float
    sequence: int


# 这个节点负责采集 eye-to-hand 标定样本，并求解 `base -> camera` 外参。
class EyeToHandCalibrationNode(Node):
    # 初始化节点。
    def __init__(self) -> None:
        # 注册节点名字。
        super().__init__("eye_to_hand_calibration_node")
        # 找到 bringup 包安装目录，方便给输出路径默认值。
        bringup_share = Path(get_package_share_directory("paus_bringup"))
        default_config_path = bringup_share / "configs" / "default.yaml"

        # 声明节点参数。如果用户用 launch 文件覆盖参数，这些默认值就会被覆盖掉。
        self.declare_parameter("config_path", str(default_config_path))
        self.declare_parameter("camera_config_path", "/tmp/paus_robot/camera.yaml")
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("status_topic", "/eye_to_hand/status")
        self.declare_parameter("board_rows", 6)
        self.declare_parameter("board_cols", 9)
        self.declare_parameter("square_size_m", 0.01)
        self.declare_parameter("solver_method", "opencv_handeye_park")
        self.declare_parameter("fresh_image_timeout_s", 2.0)
        self.declare_parameter("sample_log_path", "/tmp/paus_robot/eye_to_hand_samples.jsonl")
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
        self.declare_parameter("linux_fairino_sdk_root", str(control_cfg["linux_fairino_sdk_root"]))#在你这台 Linux/Ubuntu 机器上，FAIRINO 提供的 Python SDK 文件放在这个目录里。

        camera_config_path = self.get_parameter("camera_config_path").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.board_rows = int(self.get_parameter("board_rows").get_parameter_value().integer_value)
        self.board_cols = int(self.get_parameter("board_cols").get_parameter_value().integer_value)
        self.square_size_m = float(self.get_parameter("square_size_m").get_parameter_value().double_value)
        self.solver_method = self.get_parameter("solver_method").get_parameter_value().string_value.strip().lower()
        self.fresh_image_timeout_s = float(self.get_parameter("fresh_image_timeout_s").get_parameter_value().double_value)
        self.sample_log_path = Path(self.get_parameter("sample_log_path").get_parameter_value().string_value)
        self.tool_to_board_translation = [float(value) for value in self.get_parameter("tool_to_board.translation_m").get_parameter_value().double_array_value]
        self.tool_to_board_rotation_rpy = [float(value) for value in self.get_parameter("tool_to_board.rotation_rpy_deg").get_parameter_value().double_array_value]
        self.min_sample_count = int(self.get_parameter("min_sample_count").get_parameter_value().integer_value)
        self.output_path = Path(self.get_parameter("output_path").get_parameter_value().string_value)
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.linux_fairino_sdk_root = self.get_parameter("linux_fairino_sdk_root").get_parameter_value().string_value

        # 标定节点必须有相机内参文件，否则没法 solvePnP。
        ##runtimeerror表示运行时报错，即运行到这里时，状态不满足要求，所以不能继续
        if not camera_config_path:
            raise RuntimeError("camera_config_path is required for eye-to-hand calibration.")
        # 加载相机内参。
        self.camera_calibration = load_camera_calibration(camera_config_path)

        # 创建图像桥接器。
        self.bridge = CvBridge()
        # 缓存最新图像及其时间信息，供采样服务等待“调用后的新帧”。
        self._image_condition = threading.Condition()
        self.latest_image: CapturedImage | None = None
        self.image_sequence = 0
        # 保存已采集样本和当前求解结果。
        self.samples: list[CalibrationSample] = []
        self.current_solution: EyeToHandCalibrationSolution | None = None
        self.callback_group = ReentrantCallbackGroup()
        # 建立 Linux SDK 客户端，用于直接读取当前 TCP。
        self.linux_client = FairinoLinuxClient(self.linux_fairino_sdk_root, self.robot_ip)
        self.linux_client.connect()

        # 创建图像订阅器与状态发布器。
        self.image_subscription = self.create_subscription(Image, self.image_topic, self._image_callback, 10, callback_group=self.callback_group)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)

        # 创建“采样 / 求解 / 保存”三个服务接口。 service是按一下就执行一次的“请求-响应接口”是 node 提供的一次性请求-响应接口。
        self.capture_service = self.create_service(Trigger, "/eye_to_hand/capture_sample", self._capture_sample_callback, callback_group=self.callback_group)
        self.solve_service = self.create_service(Trigger, "/eye_to_hand/solve", self._solve_callback, callback_group=self.callback_group)
        self.save_service = self.create_service(Trigger, "/eye_to_hand/save", self._save_callback, callback_group=self.callback_group)

        # 节点启动后发布初始状态。
        self._publish_status("ready", "Eye-to-hand calibration node started.")

    # 接收最新图像。
    def _image_callback(self, message: Image) -> None:
        image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        header_time_s = None
        if message.header.stamp.sec != 0 or message.header.stamp.nanosec != 0:
            header_time_s = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        with self._image_condition:
            self.image_sequence += 1
            self.latest_image = CapturedImage(
                image_bgr=image_bgr,
                header_time_s=header_time_s,
                received_time_s=time.monotonic_ns() * 1e-9,
                sequence=self.image_sequence,
            )
            self._image_condition.notify_all()

    # 发布标定状态。
    def _publish_status(self, status: str, message: str, extra: dict[str, object] | None = None) -> None:
        payload = {
            "status": status,
            "message": message,
            "sample_count": len(self.samples),
            "min_sample_count": self.min_sample_count,
            "solver_method": self.solver_method,
        }
        if extra:
            payload.update(extra)
        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)
        self.get_logger().info(status_message.data)

    # 构造棋盘格的三维角点模板。
    def _build_board_object_points(self) -> np.ndarray:
        object_points = np.zeros((self.board_rows * self.board_cols, 3), np.float32) #为每个角点创建一个三维坐标，初始值为 (0, 0, 0)，后续会根据行列数和方格大小更新 x 和 y 坐标。这里的 z 坐标保持为 0，因为我们假设棋盘格是平放在一个平面上的。
        object_points[:, :2] = np.mgrid[0:self.board_cols, 0:self.board_rows].T.reshape(-1, 2)#这一句是在给每个点填上 (x, y)。
        object_points *= self.square_size_m #转化为真实世界角点位置坐标，单位是米。比如如果 square_size_m 是 0.01，那么相邻角点之间的距离就是 1 厘米。
        return object_points

    # 等待服务调用之后到达的一帧新图像，避免把旧缓存图像和当前 TCP 拼成样本。
    def _wait_for_fresh_image(self, previous_sequence: int) -> CapturedImage:
        deadline = time.monotonic() + self.fresh_image_timeout_s
        with self._image_condition:
            while self.latest_image is None or self.latest_image.sequence <= previous_sequence:
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0.0:
                    raise RuntimeError(f"No fresh image arrived within {self.fresh_image_timeout_s:.3f}s.")
                self._image_condition.wait(timeout=remaining_s)
            latest = self.latest_image
        return CapturedImage(
            image_bgr=latest.image_bgr.copy(),
            header_time_s=latest.header_time_s,
            received_time_s=latest.received_time_s,
            sequence=latest.sequence,
        )

    # 从指定图像中估计 `camera -> board` 变换。
    def _estimate_camera_to_board(self, image_bgr: np.ndarray) -> np.ndarray:
        # 先转成灰度图，再做棋盘格检测。
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (self.board_cols, self.board_rows))
        if not found:
            raise RuntimeError("Chessboard was not detected in the latest image.")

        # 对角点做亚像素优化。可以让位姿估计更精确，但需要更多计算时间。对于标定这种对精度要求较高的场景，通常是值得的。
        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),#这里表示大约在一个 11 x 11 的局部区域内看灰度变化。这里表示大约在一个 11 x 11 的局部区域内看灰度变化。
            (-1, -1),#不额外挖掉窗口中心的某一块，正常用整个窗口做优化
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),#迭代终止条件：最多迭代 30 次，或者当角点位置的变化小于 0.001 像素时停止。这个条件可以防止优化过程过长，同时确保优化结果足够精确。
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
    def _read_current_base_to_tool(self) -> tuple[np.ndarray, list[float], float, float]:
        read_start_time_s = time.monotonic_ns() * 1e-9
        error, tcp_pose_mmdeg = self.linux_client.get_actual_tcp_pose()
        read_end_time_s = time.monotonic_ns() * 1e-9
        if error != 0:
            raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
        # SDK 返回单位是 mm / deg，而手眼矩阵这里统一按 m 计算。
        translation_m = [float(value) / 1000.0 for value in tcp_pose_mmdeg[:3]]
        rotation_matrix = rpy_deg_to_rotation_matrix(tcp_pose_mmdeg[3:6])
        return make_transform_matrix(translation_m, rotation_matrix), [float(value) for value in tcp_pose_mmdeg], read_start_time_s, read_end_time_s

    def _current_base_to_tool(self) -> np.ndarray:
        base_to_tool, _, _, _ = self._read_current_base_to_tool()
        return base_to_tool

    # 根据固定的工具安装关系，构造 `tool -> board` 变换矩阵。
    def _tool_to_board_matrix(self) -> np.ndarray:
        rotation_matrix = rpy_deg_to_rotation_matrix(self.tool_to_board_rotation_rpy)
        return make_transform_matrix(self.tool_to_board_translation, rotation_matrix)

    def _sample_to_log_record(self, sample: CalibrationSample) -> dict[str, object]:
        tcp_mid_time_s = (sample.tcp_read_start_time_s + sample.tcp_read_end_time_s) * 0.5
        return {
            "sample_index": len(self.samples),
            "image_sequence": sample.image_sequence,
            "image_header_time_s": sample.image_header_time_s,
            "image_received_time_s": sample.image_received_time_s,
            "tcp_read_start_time_s": sample.tcp_read_start_time_s,
            "tcp_read_end_time_s": sample.tcp_read_end_time_s,
            "image_to_tcp_midpoint_age_ms": (tcp_mid_time_s - sample.image_received_time_s) * 1000.0,
            "tcp_pose_mmdeg": sample.tcp_pose_mmdeg,
            "base_to_tool_matrix": sample.base_to_tool_matrix.tolist(),
            "camera_to_board_matrix": sample.camera_to_board_matrix.tolist(),
        }

    def _append_sample_log(self, sample: CalibrationSample) -> None:
        self.sample_log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.sample_log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(self._sample_to_log_record(sample), ensure_ascii=False) + "\n")

    # 使用旧版“直接平均 base->camera”的方式求解。
    def _solve_board_average(self) -> np.ndarray:
        tool_to_board = self._tool_to_board_matrix()
        matrices = [
            sample.base_to_tool_matrix @ tool_to_board @ invert_transform_matrix(sample.camera_to_board_matrix)
            for sample in self.samples
        ]
        return average_transform_matrices(matrices)

    # 采集一次样本。
    def _capture_sample_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            with self._image_condition:
                previous_sequence = self.image_sequence
            # 先等待服务调用后的新图像，再尽快读取当前 `base -> tool`。
            captured_image = self._wait_for_fresh_image(previous_sequence)
            base_to_tool, tcp_pose_mmdeg, tcp_read_start_time_s, tcp_read_end_time_s = self._read_current_base_to_tool()
            camera_to_board = self._estimate_camera_to_board(captured_image.image_bgr)
            # 保存当前样本的绝对位姿，供后续 AX=XB 或旧方法统一求解。
            sample = CalibrationSample(
                base_to_tool_matrix=base_to_tool,
                camera_to_board_matrix=camera_to_board,
                image_header_time_s=captured_image.header_time_s,
                image_received_time_s=captured_image.received_time_s,
                tcp_read_start_time_s=tcp_read_start_time_s,
                tcp_read_end_time_s=tcp_read_end_time_s,
                tcp_pose_mmdeg=tcp_pose_mmdeg,
                image_sequence=captured_image.sequence,
            )
            self.samples.append(sample)
            self._append_sample_log(sample)
            response.success = True
            response.message = f"Captured sample #{len(self.samples)}."
            self._publish_status(
                "sample_captured",
                response.message,
                {
                    "robot_ip": self.robot_ip,
                    "sdk_root": self.linux_fairino_sdk_root,
                    "sample_log_path": str(self.sample_log_path),
                    "capture_timing": self._sample_to_log_record(sample),
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
            tool_to_board = self._tool_to_board_matrix()
            solver_matrices: dict[str, np.ndarray | None] = {}
            # 根据配置选择具体的求解方法。
            if self.solver_method == "board_average":
                solved_matrix = self._solve_board_average()
                solve_method = "eye_to_hand_board_average"
            elif self.solver_method == "opencv_handeye_park":
                solved_matrix = solve_eye_to_hand_opencv_handeye(
                    [sample.base_to_tool_matrix for sample in self.samples],
                    [sample.camera_to_board_matrix for sample in self.samples],
                    method=cv2.CALIB_HAND_EYE_PARK,
                )
                solve_method = "opencv_handeye_park"
            elif self.solver_method == "ax_xb_park":
                solved_matrix = solve_ax_xb_hand_eye_park(
                    [sample.base_to_tool_matrix for sample in self.samples],
                    [sample.camera_to_board_matrix for sample in self.samples],
                )
                solve_method = "ax_xb_park"
            else:
                raise RuntimeError(f"Unsupported solver_method: {self.solver_method}")
            solver_matrices[solve_method] = solved_matrix

            # 额外对同一批样本跑其它求解器，只做诊断输出，不影响当前选择。
            for method_name, method_builder in (
                ("eye_to_hand_board_average", self._solve_board_average),
                (
                    "opencv_handeye_park",
                    lambda: solve_eye_to_hand_opencv_handeye(
                        [sample.base_to_tool_matrix for sample in self.samples],
                        [sample.camera_to_board_matrix for sample in self.samples],
                        method=cv2.CALIB_HAND_EYE_PARK,
                    ),
                ),
                (
                    "ax_xb_park",
                    lambda: solve_ax_xb_hand_eye_park(
                        [sample.base_to_tool_matrix for sample in self.samples],
                        [sample.camera_to_board_matrix for sample in self.samples],
                    ),
                ),
            ):
                if method_name in solver_matrices:
                    continue
                try:
                    solver_matrices[method_name] = method_builder()
                except Exception:
                    solver_matrices[method_name] = None

            residuals = evaluate_eye_to_hand_residuals(
                solved_matrix,
                [sample.base_to_tool_matrix for sample in self.samples],
                [sample.camera_to_board_matrix for sample in self.samples],
                tool_to_board,
            )
            solver_residuals = {}
            for method_name, matrix in solver_matrices.items():
                if matrix is None:
                    solver_residuals[method_name] = {"status": "failed"}
                    continue
                summary = evaluate_eye_to_hand_residuals(
                    matrix,
                    [sample.base_to_tool_matrix for sample in self.samples],
                    [sample.camera_to_board_matrix for sample in self.samples],
                    tool_to_board,
                )
                solver_residuals[method_name] = asdict(summary)

            translation = solved_matrix[:3, 3]
            rotation = solved_matrix[:3, :3]
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
                method=solve_method,
            )
            response.success = True
            response.message = self.current_solution.message
            self._publish_status(
                "solved",
                response.message,
                {
                    "method": solve_method,
                    "base_to_camera_translation_m": transform.translation_m,
                    "residuals": asdict(residuals),
                    "solver_residuals": solver_residuals,
                },
            )
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        # 进入事件循环。
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        # 退出前销毁节点并关闭 ROS2。
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
