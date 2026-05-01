from __future__ import annotations

# 导入 json，用于发布结构化状态。
import json
# 导入 dataclass，便于保存单次标定样本。
from dataclasses import asdict, dataclass
# 导入 Path，便于处理输出路径。
from pathlib import Path
import shutil
import threading
import time

# 导入 OpenCV 与 NumPy，用于棋盘格检测和矩阵运算。
import cv2
import numpy as np
import yaml
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
    rotation_matrix_to_rpy_deg,
    rpy_deg_to_rotation_matrix,
    save_eye_to_hand_solution,
    resolve_config_artifact_path,
    resolve_runtime_data_path,
    solve_eye_to_hand_joint_absolute,
    solve_eye_to_hand_opencv_handeye,
    solve_ax_xb_hand_eye_park,
)
from paus_marker_ros2.semi_auto_calibration import (
    CalibrationTrajectory,
    TrajectoryValidationError,
    build_recorded_waypoint,
    create_session_dir,
    empty_trajectory,
    load_trajectory,
    save_trajectory,
    sample_log_targets,
    session_owner_matches,
    wrapped_rotation_delta_norm_deg,
)


STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


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
    # 棋盘角点重投影误差，单位像素。
    reprojection_error_px: float
    # 棋盘角点到图像边界的最小距离，单位像素。
    board_margin_px: float
    # 若保存了本样本图像，这里记录图像路径。
    image_path: str | None = None


@dataclass
class CapturedImage:
    image_bgr: np.ndarray
    header_time_s: float | None
    received_time_s: float
    sequence: int


@dataclass
class BoardPoseEstimate:
    camera_to_board_matrix: np.ndarray
    reprojection_error_px: float
    board_margin_px: float


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
        self.declare_parameter("camera_config_wait_timeout_s", 15.0)
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("status_topic", "/eye_to_hand/status")
        self.declare_parameter("board_rows", 6)
        self.declare_parameter("board_cols", 9)
        self.declare_parameter("square_size_m", 0.01)
        self.declare_parameter("solver_method", "joint_absolute")
        self.declare_parameter("fresh_image_timeout_s", 2.0)
        self.declare_parameter("sample_log_path", "")
        self.declare_parameter("tool_to_board.translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("tool_to_board.rotation_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("min_sample_count", 10)

        # 读取参数值。
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.config = load_config(self.config_path)
        calibration_cfg = self.config["calibration"]
        control_cfg = self.config["control"]
        # 在拿到主配置后，再声明机器人相关参数，允许后续显式覆盖。
        self.declare_parameter("robot_ip", str(control_cfg["robot_ip"]))
        self.declare_parameter("linux_fairino_sdk_root", str(control_cfg["linux_fairino_sdk_root"]))
        self.declare_parameter("tool_id", int(control_cfg["tool_id"]))
        self.declare_parameter("user_id", int(control_cfg["user_id"]))
        self.declare_parameter("move_vel", float(control_cfg["move_vel"]))
        self.declare_parameter("move_acc", float(control_cfg["move_acc"]))
        self.declare_parameter("execute_motion", bool(control_cfg["execute_motion"]))
        self.declare_parameter("output_path", str(calibration_cfg.get("output_path", "extrinsics.yaml")))
        self.declare_parameter("trajectory_path", str(calibration_cfg.get("trajectory_path", bringup_share / "configs" / "eye_to_hand_trajectory.yaml")))
        self.declare_parameter("session_root_path", str(calibration_cfg.get("session_root_path", "/home/chen_lab/paus_robot/calibration_sessions")))
        self.declare_parameter("save_sample_images", bool(calibration_cfg.get("save_sample_images", True)))
        self.declare_parameter("max_reprojection_error_px", float(calibration_cfg.get("max_reprojection_error_px", 0.0)))
        self.declare_parameter("min_board_margin_px", float(calibration_cfg.get("min_board_margin_px", 10.0)))
        self.declare_parameter("stable_position_tolerance_mm", float(calibration_cfg.get("stable_position_tolerance_mm", 0.2)))
        self.declare_parameter("stable_rotation_tolerance_deg", float(calibration_cfg.get("stable_rotation_tolerance_deg", 0.1)))
        self.declare_parameter("stable_window_s", float(calibration_cfg.get("stable_window_s", 0.5)))
        self.declare_parameter("stable_timeout_s", float(calibration_cfg.get("stable_timeout_s", 10.0)))
        self.declare_parameter("dwell_s", float(calibration_cfg.get("dwell_s", 0.5)))

        camera_config_path = self.get_parameter("camera_config_path").get_parameter_value().string_value
        self.camera_config_wait_timeout_s = float(self.get_parameter("camera_config_wait_timeout_s").get_parameter_value().double_value)
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.board_rows = int(self.get_parameter("board_rows").get_parameter_value().integer_value)
        self.board_cols = int(self.get_parameter("board_cols").get_parameter_value().integer_value)
        self.square_size_m = float(self.get_parameter("square_size_m").get_parameter_value().double_value)
        self.solver_method = self.get_parameter("solver_method").get_parameter_value().string_value.strip().lower()
        self.fresh_image_timeout_s = float(self.get_parameter("fresh_image_timeout_s").get_parameter_value().double_value)
        sample_log_path_value = self.get_parameter("sample_log_path").get_parameter_value().string_value.strip()
        self.trajectory_path = Path(resolve_config_artifact_path(self.get_parameter("trajectory_path").get_parameter_value().string_value, self.config_path))
        self.session_root_path = Path(resolve_runtime_data_path(self.get_parameter("session_root_path").get_parameter_value().string_value, self.config_path))
        self.save_sample_images = bool(self.get_parameter("save_sample_images").get_parameter_value().bool_value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").get_parameter_value().double_value)
        self.min_board_margin_px = float(self.get_parameter("min_board_margin_px").get_parameter_value().double_value)
        self.stable_position_tolerance_mm = float(self.get_parameter("stable_position_tolerance_mm").get_parameter_value().double_value)
        self.stable_rotation_tolerance_deg = float(self.get_parameter("stable_rotation_tolerance_deg").get_parameter_value().double_value)
        self.stable_window_s = float(self.get_parameter("stable_window_s").get_parameter_value().double_value)
        self.stable_timeout_s = float(self.get_parameter("stable_timeout_s").get_parameter_value().double_value)
        self.dwell_s = float(self.get_parameter("dwell_s").get_parameter_value().double_value)
        self.tool_to_board_translation = [float(value) for value in self.get_parameter("tool_to_board.translation_m").get_parameter_value().double_array_value]
        self.tool_to_board_rotation_rpy = [float(value) for value in self.get_parameter("tool_to_board.rotation_rpy_deg").get_parameter_value().double_array_value]
        self.min_sample_count = int(self.get_parameter("min_sample_count").get_parameter_value().integer_value)
        self.output_path = Path(resolve_config_artifact_path(self.get_parameter("output_path").get_parameter_value().string_value, self.config_path))
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.linux_fairino_sdk_root = self.get_parameter("linux_fairino_sdk_root").get_parameter_value().string_value
        self.tool_id = int(self.get_parameter("tool_id").get_parameter_value().integer_value)
        self.user_id = int(self.get_parameter("user_id").get_parameter_value().integer_value)
        self.move_vel = float(self.get_parameter("move_vel").get_parameter_value().double_value)
        self.move_acc = float(self.get_parameter("move_acc").get_parameter_value().double_value)
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)
        self.session_dir: Path | None = None
        self.sample_log_path_override = Path(resolve_runtime_data_path(sample_log_path_value, self.config_path)) if sample_log_path_value else None
        self.sample_log_path: Path | None = None
        self.report_path: Path | None = None
        self.run_log_path: Path | None = None
        self.session_owner: str | None = None

        # 标定节点必须有相机内参文件，否则没法 solvePnP。
        ##runtimeerror表示运行时报错，即运行到这里时，状态不满足要求，所以不能继续
        if not camera_config_path:
            raise RuntimeError("camera_config_path is required for eye-to-hand calibration.")
        camera_config_path = str(self._wait_for_camera_config(camera_config_path))
        self.camera_config_path = camera_config_path
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
        self._semi_auto_lock = threading.Lock()
        self._semi_auto_active = False
        self.recorded_trajectory = self._load_or_create_trajectory_for_recording()
        self.callback_group = ReentrantCallbackGroup()
        # 建立 Linux SDK 客户端，用于直接读取当前 TCP。
        self.linux_client = FairinoLinuxClient(self.linux_fairino_sdk_root, self.robot_ip)
        self.linux_client.connect()

        # 创建图像订阅器与状态发布器。
        self.image_subscription = self.create_subscription(Image, self.image_topic, self._image_callback, 10, callback_group=self.callback_group)
        self.status_publisher = self.create_publisher(String, self.status_topic, STATUS_QOS)

        # 创建“采样 / 求解 / 保存”三个服务接口。 service是按一下就执行一次的“请求-响应接口”是 node 提供的一次性请求-响应接口。
        self.capture_service = self.create_service(Trigger, "/eye_to_hand/capture_sample", self._capture_sample_callback, callback_group=self.callback_group)
        self.solve_service = self.create_service(Trigger, "/eye_to_hand/solve", self._solve_callback, callback_group=self.callback_group)
        self.save_service = self.create_service(Trigger, "/eye_to_hand/save", self._save_callback, callback_group=self.callback_group)
        self.record_waypoint_service = self.create_service(Trigger, "/eye_to_hand/record_waypoint", self._record_waypoint_callback, callback_group=self.callback_group)
        self.delete_waypoint_service = self.create_service(Trigger, "/eye_to_hand/delete_last_waypoint", self._delete_last_waypoint_callback, callback_group=self.callback_group)
        self.save_trajectory_service = self.create_service(Trigger, "/eye_to_hand/save_trajectory", self._save_trajectory_callback, callback_group=self.callback_group)
        self.run_semi_auto_service = self.create_service(Trigger, "/eye_to_hand/run_semi_auto_calibration", self._run_semi_auto_callback, callback_group=self.callback_group)

        # 节点启动后发布初始状态。
        self._publish_status("ready", "Eye-to-hand calibration node started.", {"trajectory_path": str(self.trajectory_path)})

    def _wait_for_camera_config(self, camera_config_path: str) -> Path:
        path = Path(camera_config_path)
        deadline = time.monotonic() + max(0.0, self.camera_config_wait_timeout_s)
        last_error: str | None = None
        while True:
            try:
                if path.exists() and path.stat().st_size > 0:
                    load_camera_calibration(path)
                    return path
            except Exception as exc:
                last_error = repr(exc)
            if time.monotonic() >= deadline:
                message = f"camera_config_path is not readable after waiting {self.camera_config_wait_timeout_s:.1f}s: {path}"
                if last_error:
                    message += f" Last parse error: {last_error}"
                raise RuntimeError(message)
            time.sleep(0.1)

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
            "session_dir": str(self.session_dir) if self.session_dir is not None else None,
            "trajectory_path": str(self.trajectory_path),
            "session_root_path": str(self.session_root_path),
            "output_path": str(self.output_path),
            "execute_motion": self.execute_motion,
            "max_reprojection_error_px": self.max_reprojection_error_px,
            "min_board_margin_px": self.min_board_margin_px,
            "camera_config_path": self.camera_config_path,
            "board_rows": self.board_rows,
            "board_cols": self.board_cols,
            "square_size_m": self.square_size_m,
        }
        if extra:
            payload.update(extra)
        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)
        self.get_logger().info(status_message.data)

    def _append_run_log(self, event: str, payload: dict[str, object] | None = None) -> None:
        if self.run_log_path is None:
            return
        record = {
            "event": event,
            "monotonic_time_s": time.monotonic(),
            "sample_count": len(self.samples),
        }
        if payload:
            record.update(payload)
        self.run_log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.run_log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _load_or_create_trajectory_for_recording(self) -> CalibrationTrajectory:
        if self.trajectory_path.exists():
            try:
                return load_trajectory(self.trajectory_path)
            except TrajectoryValidationError as exc:
                self.get_logger().warning(f"Existing trajectory is invalid and will not be used for recording cache: {exc}")
        return empty_trajectory(
            tool_id=self.tool_id,
            user_id=self.user_id,
            default_vel=self.move_vel,
            default_acc=self.move_acc,
            default_dwell_s=self.dwell_s,
        )

    def _begin_new_semi_auto_session(self) -> None:
        self.session_dir = create_session_dir(self.session_root_path)
        self.sample_log_path = self.session_dir / "samples.jsonl"
        self.report_path = self.session_dir / "report.yaml"
        self.run_log_path = self.session_dir / "run.log"
        self.session_owner = "semi_auto"
        self.samples.clear()
        self.current_solution = None

    def _ensure_session_started(self, owner: str = "manual") -> None:
        if self.session_dir is None or not session_owner_matches(self.session_owner, owner):
            self.session_dir = create_session_dir(self.session_root_path)
            self.sample_log_path = self.session_dir / "samples.jsonl"
            self.report_path = self.session_dir / "report.yaml"
            self.run_log_path = self.session_dir / "run.log"
            self.session_owner = owner
            self.samples.clear()
            self.current_solution = None

    def _clear_manual_session_after_save(self) -> None:
        self.session_dir = None
        self.sample_log_path = None
        self.report_path = None
        self.run_log_path = None
        self.session_owner = None
        self.samples.clear()
        self.current_solution = None

    def _reject_manual_service_if_semi_auto_active(
        self,
        response: Trigger.Response,
        *,
        status: str,
        operation: str,
    ) -> bool:
        with self._semi_auto_lock:
            semi_auto_active = self._semi_auto_active
        if not semi_auto_active:
            return False
        response.success = False
        response.message = f"Semi-auto calibration is running; manual {operation} is disabled."
        self._publish_status(status, response.message)
        return True

    def _write_recorded_trajectory(self) -> None:
        save_trajectory(self.recorded_trajectory, self.trajectory_path)

    def _recorded_trajectory_status(self, *, dirty: bool) -> dict[str, object]:
        return {
            "recorded_trajectory": self.recorded_trajectory.to_payload(),
            "trajectory_dirty": dirty,
        }

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
    def _estimate_camera_to_board(self, image_bgr: np.ndarray, *, apply_quality_filters: bool = True) -> BoardPoseEstimate:
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
        projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        reprojection_error_px = float(np.sqrt(np.mean(np.square(projected_points.reshape(-1, 2) - refined.reshape(-1, 2)))))
        corner_points = refined.reshape(-1, 2)
        height, width = gray.shape[:2]
        board_margin_px = float(
            min(
                np.min(corner_points[:, 0]),
                np.min(corner_points[:, 1]),
                width - 1.0 - np.max(corner_points[:, 0]),
                height - 1.0 - np.max(corner_points[:, 1]),
            )
        )
        if apply_quality_filters and self.max_reprojection_error_px > 0.0 and reprojection_error_px > self.max_reprojection_error_px:
            raise RuntimeError(
                f"Chessboard reprojection error {reprojection_error_px:.3f}px exceeds {self.max_reprojection_error_px:.3f}px."
            )
        if apply_quality_filters and board_margin_px < self.min_board_margin_px:
            raise RuntimeError(f"Chessboard margin {board_margin_px:.1f}px is below {self.min_board_margin_px:.1f}px.")
        # 将 Rodrigues 旋转向量转成旋转矩阵，再组装成齐次矩阵。
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return BoardPoseEstimate(
            camera_to_board_matrix=make_transform_matrix(tvec.reshape(3), rotation_matrix),
            reprojection_error_px=reprojection_error_px,
            board_margin_px=board_margin_px,
        )

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

    def _sample_to_log_record(self, sample: CalibrationSample, *, sample_index: int | None = None) -> dict[str, object]:
        tcp_mid_time_s = (sample.tcp_read_start_time_s + sample.tcp_read_end_time_s) * 0.5
        return {
            "sample_index": len(self.samples) if sample_index is None else sample_index,
            "image_sequence": sample.image_sequence,
            "image_header_time_s": sample.image_header_time_s,
            "image_received_time_s": sample.image_received_time_s,
            "tcp_read_start_time_s": sample.tcp_read_start_time_s,
            "tcp_read_end_time_s": sample.tcp_read_end_time_s,
            "image_to_tcp_midpoint_age_ms": (tcp_mid_time_s - sample.image_received_time_s) * 1000.0,
            "tcp_pose_mmdeg": sample.tcp_pose_mmdeg,
            "base_to_tool_matrix": sample.base_to_tool_matrix.tolist(),
            "camera_to_board_matrix": sample.camera_to_board_matrix.tolist(),
            "reprojection_error_px": sample.reprojection_error_px,
            "board_margin_px": sample.board_margin_px,
            "image_path": sample.image_path,
        }

    def _append_sample_log(self, sample: CalibrationSample) -> None:
        if self.sample_log_path is None:
            raise RuntimeError("Calibration session is not initialized.")
        record_line = json.dumps(self._sample_to_log_record(sample), ensure_ascii=False) + "\n"
        for sample_log_path in sample_log_targets(self.sample_log_path, self.sample_log_path_override):
            sample_log_path.parent.mkdir(parents=True, exist_ok=True)
            with sample_log_path.open("a", encoding="utf-8") as handle:
                handle.write(record_line)

    def _save_sample_image(self, image_bgr: np.ndarray, sample_index: int) -> str | None:
        if not self.save_sample_images:
            return None
        if self.session_dir is None:
            raise RuntimeError("Calibration session is not initialized.")
        image_path = self.session_dir / "images" / f"sample_{sample_index:03d}.png"
        image_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(image_path), image_bgr):
            raise RuntimeError(f"Failed to write sample image: {image_path}")
        return str(image_path)

    def _probe_current_board_quality(self) -> dict[str, object]:
        with self._image_condition:
            previous_sequence = self.image_sequence
        try:
            captured_image = self._wait_for_fresh_image(previous_sequence)
            estimate = self._estimate_camera_to_board(captured_image.image_bgr, apply_quality_filters=False)
        except Exception as exc:
            return {
                "detected": False,
                "reason": repr(exc),
            }

        quality: dict[str, object] = {
            "detected": True,
            "image_sequence": captured_image.sequence,
            "reprojection_error_px": estimate.reprojection_error_px,
            "board_margin_px": estimate.board_margin_px,
            "reprojection_filter_enabled": self.max_reprojection_error_px > 0.0,
            "min_board_margin_px": self.min_board_margin_px,
            "margin_filter_passed": estimate.board_margin_px >= self.min_board_margin_px,
        }
        if self.max_reprojection_error_px > 0.0:
            quality["max_reprojection_error_px"] = self.max_reprojection_error_px
            quality["reprojection_filter_passed"] = estimate.reprojection_error_px <= self.max_reprojection_error_px
        return quality

    def _format_record_quality(self, quality: dict[str, object]) -> str:
        if not quality.get("detected"):
            return f"chessboard not detected: {quality.get('reason', 'unknown error')}"

        reprojection_error_px = float(quality["reprojection_error_px"])
        board_margin_px = float(quality["board_margin_px"])
        message = f"chessboard reprojection_error_px={reprojection_error_px:.3f}, board_margin_px={board_margin_px:.1f}"
        if not bool(quality.get("reprojection_filter_enabled", True)):
            message += ", reprojection filter disabled"
        elif not bool(quality.get("reprojection_filter_passed", True)):
            message += f", reprojection would exceed {float(quality['max_reprojection_error_px']):.3f}px"
        if not bool(quality.get("margin_filter_passed", True)):
            message += f", margin below {float(quality['min_board_margin_px']):.1f}px"
        return message

    def _capture_one_sample(self, *, session_owner: str = "manual") -> CalibrationSample:
        with self._image_condition:
            previous_sequence = self.image_sequence
        captured_image = self._wait_for_fresh_image(previous_sequence)
        base_to_tool, tcp_pose_mmdeg, tcp_read_start_time_s, tcp_read_end_time_s = self._read_current_base_to_tool()
        board_estimate = self._estimate_camera_to_board(captured_image.image_bgr)
        self._ensure_session_started(session_owner)
        sample_index = len(self.samples) + 1
        image_path = self._save_sample_image(captured_image.image_bgr, sample_index)
        sample = CalibrationSample(
            base_to_tool_matrix=base_to_tool,
            camera_to_board_matrix=board_estimate.camera_to_board_matrix,
            image_header_time_s=captured_image.header_time_s,
            image_received_time_s=captured_image.received_time_s,
            tcp_read_start_time_s=tcp_read_start_time_s,
            tcp_read_end_time_s=tcp_read_end_time_s,
            tcp_pose_mmdeg=tcp_pose_mmdeg,
            image_sequence=captured_image.sequence,
            reprojection_error_px=board_estimate.reprojection_error_px,
            board_margin_px=board_estimate.board_margin_px,
            image_path=image_path,
        )
        self.samples.append(sample)
        self._append_sample_log(sample)
        self._append_run_log("sample_captured", self._sample_to_log_record(sample))
        return sample

    # 使用旧版“直接平均 base->camera”的方式求解。
    def _solve_board_average(self) -> np.ndarray:
        tool_to_board = self._tool_to_board_matrix()
        matrices = [
            sample.base_to_tool_matrix @ tool_to_board @ invert_transform_matrix(sample.camera_to_board_matrix)
            for sample in self.samples
        ]
        return average_transform_matrices(matrices)

    def _tool_to_board_payload(self, tool_to_board_matrix: np.ndarray) -> dict[str, list[float]]:
        tool_to_board_matrix = np.asarray(tool_to_board_matrix, dtype=np.float64).reshape(4, 4)
        return {
            "translation_m": [float(value) for value in tool_to_board_matrix[:3, 3].tolist()],
            "rotation_rpy_deg": [float(value) for value in rotation_matrix_to_rpy_deg(tool_to_board_matrix[:3, :3])],
        }

    def _write_report(self, payload: dict[str, object]) -> None:
        if self.report_path is None:
            raise RuntimeError("Calibration session is not initialized.")
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        with self.report_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)

    # 采集一次样本。
    def _capture_sample_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response, status="capture_rejected", operation="capture"):
            return response
        try:
            sample = self._capture_one_sample()
            response.success = True
            response.message = f"Captured sample #{len(self.samples)}."
            self._publish_status(
                "sample_captured",
                response.message,
                {
                    "robot_ip": self.robot_ip,
                    "sdk_root": self.linux_fairino_sdk_root,
                    "sample_log_path": str(self.sample_log_path) if self.sample_log_path is not None else None,
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
        if self._reject_manual_service_if_semi_auto_active(response, status="solve_rejected", operation="solve"):
            return response
        return self._solve_samples(response)

    def _solve_samples(self, response: Trigger.Response) -> Trigger.Response:
        # 样本不足时直接拒绝求解。
        if len(self.samples) < self.min_sample_count:
            response.success = False
            response.message = f"Need at least {self.min_sample_count} samples, currently {len(self.samples)}."
            self._publish_status("solve_failed", response.message)
            return response
        try:
            configured_tool_to_board = self._tool_to_board_matrix()
            solver_results: dict[str, tuple[np.ndarray, np.ndarray] | None] = {}
            # 根据配置选择具体的求解方法。
            if self.solver_method == "board_average":
                solved_matrix = self._solve_board_average()
                solved_tool_to_board = configured_tool_to_board
                solve_method = "eye_to_hand_board_average"
            elif self.solver_method == "opencv_handeye_park":
                solved_matrix = solve_eye_to_hand_opencv_handeye(
                    [sample.base_to_tool_matrix for sample in self.samples],
                    [sample.camera_to_board_matrix for sample in self.samples],
                    method=cv2.CALIB_HAND_EYE_PARK,
                )
                solved_tool_to_board = configured_tool_to_board
                solve_method = "opencv_handeye_park"
            elif self.solver_method == "ax_xb_park":
                solved_matrix = solve_ax_xb_hand_eye_park(
                    [sample.base_to_tool_matrix for sample in self.samples],
                    [sample.camera_to_board_matrix for sample in self.samples],
                )
                solved_tool_to_board = configured_tool_to_board
                solve_method = "ax_xb_park"
            elif self.solver_method == "joint_absolute":
                solved_matrix, solved_tool_to_board = solve_eye_to_hand_joint_absolute(
                    [sample.base_to_tool_matrix for sample in self.samples],
                    [sample.camera_to_board_matrix for sample in self.samples],
                    initial_tool_to_board_matrix=configured_tool_to_board,
                )
                solve_method = "joint_absolute"
            else:
                raise RuntimeError(f"Unsupported solver_method: {self.solver_method}")
            solver_results[solve_method] = (solved_matrix, solved_tool_to_board)

            # 额外对同一批样本跑其它求解器，只做诊断输出，不影响当前选择。
            for method_name, method_builder in (
                ("eye_to_hand_board_average", lambda: (self._solve_board_average(), configured_tool_to_board)),
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
                (
                    "joint_absolute",
                    lambda: solve_eye_to_hand_joint_absolute(
                        [sample.base_to_tool_matrix for sample in self.samples],
                        [sample.camera_to_board_matrix for sample in self.samples],
                        initial_tool_to_board_matrix=configured_tool_to_board,
                    ),
                ),
            ):
                if method_name in solver_results:
                    continue
                try:
                    result = method_builder()
                    if isinstance(result, tuple) and len(result) == 2:
                        matrix_result, tool_matrix_result = result
                    else:
                        matrix_result, tool_matrix_result = result, configured_tool_to_board
                    solver_results[method_name] = (
                        np.asarray(matrix_result, dtype=np.float64).reshape(4, 4),
                        np.asarray(tool_matrix_result, dtype=np.float64).reshape(4, 4),
                    )
                except Exception:
                    solver_results[method_name] = None

            residuals = evaluate_eye_to_hand_residuals(
                solved_matrix,
                [sample.base_to_tool_matrix for sample in self.samples],
                [sample.camera_to_board_matrix for sample in self.samples],
                solved_tool_to_board,
            )
            solver_residuals = {}
            for method_name, result in solver_results.items():
                if result is None:
                    solver_residuals[method_name] = {"status": "failed"}
                    continue
                matrix, tool_to_board = result
                summary = evaluate_eye_to_hand_residuals(
                    matrix,
                    [sample.base_to_tool_matrix for sample in self.samples],
                    [sample.camera_to_board_matrix for sample in self.samples],
                    tool_to_board,
                )
                solver_residuals[method_name] = {
                    **asdict(summary),
                    "tool_to_board": self._tool_to_board_payload(tool_to_board),
                }

            translation = solved_matrix[:3, 3]
            rotation = solved_matrix[:3, :3]
            transform = make_transform_struct(translation, rotation, "robot_base", "camera")
            solved_tool_payload = self._tool_to_board_payload(solved_tool_to_board)
            # 保存当前求解结果。
            self.current_solution = EyeToHandCalibrationSolution(
                status="ok",
                success=True,
                sample_count=len(self.samples),
                message="Eye-to-hand calibration solved successfully.",
                base_to_camera=transform,
                tool_to_board_translation_m=solved_tool_payload["translation_m"],
                tool_to_board_rotation_rpy_deg=solved_tool_payload["rotation_rpy_deg"],
                method=solve_method,
            )
            report_payload = {
                "session_dir": str(self.session_dir) if self.session_dir is not None else None,
                "trajectory_path": str(self.trajectory_path),
                "sample_log_path": str(self.sample_log_path) if self.sample_log_path is not None else None,
                "sample_count": len(self.samples),
                "method": solve_method,
                "base_to_camera": asdict(transform),
                "tool_to_board": solved_tool_payload,
                "residuals": asdict(residuals),
                "solver_residuals": solver_residuals,
                "samples": [
                    self._sample_to_log_record(sample, sample_index=sample_index)
                    for sample_index, sample in enumerate(self.samples, start=1)
                ],
            }
            self._write_report(report_payload)
            self._append_run_log(
                "solved",
                {
                    "report_path": str(self.report_path) if self.report_path is not None else None,
                    "method": solve_method,
                    "sample_count": len(self.samples),
                },
            )
            response.success = True
            response.message = self.current_solution.message
            self._publish_status(
                "solved",
                response.message,
                {
                    "method": solve_method,
                    "base_to_camera_translation_m": transform.translation_m,
                    "tool_to_board_translation_m": self.current_solution.tool_to_board_translation_m,
                    "tool_to_board_rotation_rpy_deg": self.current_solution.tool_to_board_rotation_rpy_deg,
                    "residuals": asdict(residuals),
                    "solver_residuals": solver_residuals,
                    "report_path": str(self.report_path) if self.report_path is not None else None,
                },
            )
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("solve_failed", response.message)
        return response

    # 将当前求解结果保存成外参文件。
    def _save_current_solution(self) -> None:
        if self.current_solution is None or not self.current_solution.success:
            raise RuntimeError("No solved extrinsic is available.")
        if self.session_dir is None:
            raise RuntimeError("Calibration session is not initialized.")
        before_path = self.session_dir / "extrinsics_before.yaml"
        after_path = self.session_dir / "extrinsics_after.yaml"
        if self.output_path.exists():
            shutil.copy2(self.output_path, before_path)
        save_eye_to_hand_solution(self.current_solution, self.output_path)
        shutil.copy2(self.output_path, after_path)
        self._append_run_log(
            "extrinsics_saved",
            {
                "output_path": str(self.output_path),
                "extrinsics_before": str(before_path) if before_path.exists() else None,
                "extrinsics_after": str(after_path),
            },
        )

    def _save_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response, status="save_rejected", operation="save"):
            return response
        # 如果还没有求解成功，就不允许保存。
        if self.current_solution is None or not self.current_solution.success:
            response.success = False
            response.message = "No solved extrinsic is available."
            self._publish_status("save_failed", response.message)
            return response
        try:
            self._save_current_solution()
            response.success = True
            response.message = f"Saved extrinsic to {self.output_path}."
            saved_session_dir = str(self.session_dir) if self.session_dir is not None else None
            self._publish_status(
                "saved",
                response.message,
                {"output_path": str(self.output_path), "session_dir": saved_session_dir},
            )
            self._clear_manual_session_after_save()
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("save_failed", response.message)
        return response

    def _record_waypoint_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response, status="record_waypoint_rejected", operation="waypoint recording"):
            return response
        try:
            joint_error, joint_deg = self.linux_client.get_actual_joint_pos_degree()
            if joint_error != 0:
                raise RuntimeError(f"GetActualJointPosDegree failed with code {joint_error}.")
            tcp_error, tcp_pose_mmdeg = self.linux_client.get_actual_tcp_pose()
            if tcp_error != 0:
                raise RuntimeError(f"GetActualTCPPose failed with code {tcp_error}.")
            record_quality = self._probe_current_board_quality()
            waypoint = build_recorded_waypoint(
                index=len(self.recorded_trajectory.waypoints) + 1,
                joint_deg=joint_deg,
                tcp_pose_mmdeg=tcp_pose_mmdeg,
                vel=self.move_vel,
                acc=self.move_acc,
                dwell_s=self.dwell_s,
                capture=True,
                record_quality=record_quality,
            )
            self.recorded_trajectory.waypoints.append(waypoint)
            self._ensure_session_started("manual")
            response.success = True
            response.message = f"Recorded {waypoint.name} in memory. Press finish/save to write {self.trajectory_path}. {self._format_record_quality(record_quality)}."
            waypoint_payload = waypoint.to_payload()
            self._append_run_log(
                "waypoint_recorded",
                {
                    "waypoint": waypoint_payload,
                    "waypoint_name": waypoint.name,
                    "record_quality": record_quality,
                },
            )
            self._publish_status(
                "waypoint_recorded",
                response.message,
                {
                    "waypoint_count": len(self.recorded_trajectory.waypoints),
                    "record_quality": record_quality,
                    "waypoint": waypoint_payload,
                    "waypoint_name": waypoint.name,
                    **self._recorded_trajectory_status(dirty=True),
                },
            )
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("record_waypoint_failed", response.message)
        return response

    def _delete_last_waypoint_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response, status="delete_waypoint_rejected", operation="waypoint deletion"):
            return response
        if not self.recorded_trajectory.waypoints:
            response.success = False
            response.message = "No recorded waypoint to delete."
            self._publish_status("delete_waypoint_failed", response.message)
            return response
        removed = self.recorded_trajectory.waypoints.pop()
        response.success = True
        response.message = f"Deleted {removed.name} from the in-memory recording."
        removed_payload = removed.to_payload()
        deleted_progress = {
            "waypoint_name": removed.name,
            "waypoint": removed_payload,
            "waypoint_count": len(self.recorded_trajectory.waypoints),
        }
        self._ensure_session_started("manual")
        self._append_run_log("waypoint_deleted", deleted_progress)
        self._publish_status(
            "waypoint_deleted",
            response.message,
            {**deleted_progress, **self._recorded_trajectory_status(dirty=True)},
        )
        return response

    def _save_trajectory_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response, status="save_trajectory_rejected", operation="trajectory save"):
            return response
        if not self.recorded_trajectory.waypoints:
            if not self.trajectory_path.exists():
                response.success = False
                response.message = "No recorded waypoints to save."
                self._publish_status("save_trajectory_failed", response.message)
                return response
            self._ensure_session_started("manual")
            self.trajectory_path.unlink()
            response.success = True
            response.message = f"Cleared stale trajectory file at {self.trajectory_path}."
            self._publish_status(
                "trajectory_saved",
                response.message,
                {"trajectory_path": str(self.trajectory_path), **self._recorded_trajectory_status(dirty=False)},
            )
            return response
        self._ensure_session_started("manual")
        self._write_recorded_trajectory()
        if self.session_dir is not None:
            shutil.copy2(self.trajectory_path, self.session_dir / "trajectory_used.yaml")
        response.success = True
        response.message = f"Saved {len(self.recorded_trajectory.waypoints)} waypoints to {self.trajectory_path}."
        self._publish_status(
            "trajectory_saved",
            response.message,
            {"trajectory_path": str(self.trajectory_path), **self._recorded_trajectory_status(dirty=False)},
        )
        return response

    def _wait_until_tcp_stable(self) -> list[float]:
        deadline = time.monotonic() + self.stable_timeout_s
        stable_since: float | None = None
        stable_reference_pose: list[float] | None = None
        last_pose: list[float] | None = None
        while time.monotonic() < deadline:
            error, pose = self.linux_client.get_actual_tcp_pose()
            if error != 0:
                raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
            last_pose = pose
            now = time.monotonic()
            if self.stable_window_s <= 0.0:
                return pose
            if stable_reference_pose is None:
                stable_reference_pose = pose
                stable_since = now
            else:
                position_delta_mm = float(np.linalg.norm(np.asarray(pose[:3], dtype=np.float64) - np.asarray(stable_reference_pose[:3], dtype=np.float64)))
                rotation_delta_deg = wrapped_rotation_delta_norm_deg(pose[3:6], stable_reference_pose[3:6])
                if position_delta_mm <= self.stable_position_tolerance_mm and rotation_delta_deg <= self.stable_rotation_tolerance_deg:
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= self.stable_window_s:
                        return pose
                else:
                    stable_reference_pose = pose
                    stable_since = now
            time.sleep(0.05)
        raise RuntimeError(f"TCP did not become stable within {self.stable_timeout_s:.3f}s. Last pose: {last_pose!r}")

    def _run_semi_auto_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._semi_auto_lock:
            if self._semi_auto_active:
                response.success = False
                response.message = "Semi-auto calibration is already running."
                self._publish_status("semi_auto_rejected", response.message)
                return response
            self._semi_auto_active = True
        started_session_dir: str | None = None
        try:
            if not self.trajectory_path.exists():
                response.success = False
                response.message = f"Trajectory YAML does not exist: {self.trajectory_path}. Record waypoints first."
                self._publish_status("semi_auto_failed", response.message, {"session_dir": None})
                return response
            trajectory = load_trajectory(self.trajectory_path)
            self._begin_new_semi_auto_session()
            started_session_dir = str(self.session_dir) if self.session_dir is not None else None
            shutil.copy2(self.trajectory_path, self.session_dir / "trajectory_used.yaml")
            self._append_run_log("semi_auto_started", {"trajectory_path": str(self.trajectory_path), "execute_motion": self.execute_motion})
            self._publish_status(
                "semi_auto_started",
                "Semi-auto calibration run started.",
                {"trajectory_path": str(self.trajectory_path), "execute_motion": self.execute_motion},
            )

            if not self.execute_motion:
                waypoint_count = len(trajectory.waypoints)
                for waypoint_index, waypoint in enumerate(trajectory.waypoints, start=1):
                    waypoint_progress = {
                        "waypoint_name": waypoint.name,
                        "waypoint_index": waypoint_index,
                        "waypoint_count": waypoint_count,
                        "captured_count": 0,
                        "skipped_count": waypoint_index,
                        "waypoint": waypoint.to_payload(),
                        "reason": "dry-run",
                    }
                    self._append_run_log("waypoint_dry_run_complete", waypoint_progress)
                    self._publish_status(
                        "waypoint_dry_run_complete",
                        f"Dry-run waypoint {waypoint.name}.",
                        waypoint_progress,
                    )
                response.success = True
                response.message = f"Dry-run complete for {len(trajectory.waypoints)} waypoints. No motion, capture, solve, or save was executed."
                self._append_run_log("semi_auto_dry_run_complete", {"waypoint_count": len(trajectory.waypoints)})
                self._publish_status("semi_auto_dry_run_complete", response.message, {"session_dir": str(self.session_dir)})
                return response

            captured_count = 0
            skipped_count = 0
            waypoint_count = len(trajectory.waypoints)
            for waypoint_index, waypoint in enumerate(trajectory.waypoints, start=1):
                if waypoint.motion != "movej":
                    raise RuntimeError(f"Unsupported waypoint motion: {waypoint.motion}")
                waypoint_progress = {
                    "waypoint_name": waypoint.name,
                    "waypoint_index": waypoint_index,
                    "waypoint_count": waypoint_count,
                    "captured_count": captured_count,
                    "skipped_count": skipped_count,
                    "waypoint": waypoint.to_payload(),
                }
                self._append_run_log("waypoint_motion_started", waypoint_progress)
                self._publish_status(
                    "waypoint_motion_started",
                    f"Moving to {waypoint.name} ({waypoint_index}/{waypoint_count}).",
                    waypoint_progress,
                )
                move_error = self.linux_client.move_j(
                    waypoint.joint_deg,
                    tool_id=trajectory.tool_id,
                    user_id=trajectory.user_id,
                    vel=waypoint.vel,
                    acc=waypoint.acc,
                )
                if move_error != 0:
                    raise RuntimeError(f"MoveJ failed at {waypoint.name} with code {move_error}.")
                self._publish_status(
                    "waypoint_waiting_stable",
                    f"Waiting for TCP stability at {waypoint.name} ({waypoint_index}/{waypoint_count}).",
                    waypoint_progress,
                )
                stable_pose = self._wait_until_tcp_stable()
                time.sleep(max(0.0, waypoint.dwell_s))
                self._append_run_log("waypoint_reached", {"waypoint": waypoint.to_payload(), "stable_tcp_pose_mmdeg": stable_pose})
                self._publish_status(
                    "waypoint_reached",
                    f"Reached {waypoint.name} ({waypoint_index}/{waypoint_count}).",
                    {**waypoint_progress, "stable_tcp_pose_mmdeg": stable_pose},
                )
                if waypoint.capture:
                    self._publish_status(
                        "waypoint_capture_started",
                        f"Capturing chessboard at {waypoint.name} ({waypoint_index}/{waypoint_count}).",
                        waypoint_progress,
                    )
                    try:
                        sample = self._capture_one_sample(session_owner="semi_auto")
                    except Exception as exc:
                        skipped_count += 1
                        self._append_run_log(
                            "waypoint_capture_skipped",
                            {
                                "waypoint": waypoint.to_payload(),
                                "reason": repr(exc),
                            },
                        )
                        self._publish_status(
                            "waypoint_capture_skipped",
                            f"Skipped {waypoint.name}: {exc}",
                            {
                                "waypoint": waypoint.to_payload(),
                                "reason": repr(exc),
                                "captured_count": captured_count,
                                "skipped_count": skipped_count,
                            },
                        )
                        continue
                    captured_count += 1
                    sample_progress = {
                        **waypoint_progress,
                        "captured_count": captured_count,
                        "skipped_count": skipped_count,
                        "sample_index": len(self.samples),
                        "reprojection_error_px": sample.reprojection_error_px,
                        "board_margin_px": sample.board_margin_px,
                        "image_path": sample.image_path,
                    }
                    self._append_run_log("waypoint_sample_captured", sample_progress)
                    self._publish_status(
                        "waypoint_sample_captured",
                        (
                            f"Captured sample #{len(self.samples)} at {waypoint.name}: "
                            f"reprojection_error_px={sample.reprojection_error_px:.3f}, "
                            f"board_margin_px={sample.board_margin_px:.1f}."
                        ),
                        sample_progress,
                    )
                else:
                    skipped_count += 1
                    disabled_progress = {
                        **waypoint_progress,
                        "captured_count": captured_count,
                        "skipped_count": skipped_count,
                        "reason": "capture=false",
                    }
                    self._append_run_log(
                        "waypoint_capture_disabled",
                        {
                            "waypoint": waypoint.to_payload(),
                            "waypoint_name": waypoint.name,
                            "reason": "capture=false",
                            "captured_count": captured_count,
                            "skipped_count": skipped_count,
                        },
                    )
                    self._publish_status(
                        "waypoint_capture_disabled",
                        f"Completed {waypoint.name} without capture (capture=false).",
                        disabled_progress,
                    )

            if len(self.samples) < self.min_sample_count:
                response.success = False
                response.message = (
                    f"Semi-auto run completed with {captured_count} accepted samples and {skipped_count} skipped waypoints, "
                    f"but need at least {self.min_sample_count} valid samples."
                )
                self._append_run_log(
                    "semi_auto_insufficient_samples",
                    {
                        "accepted_samples": captured_count,
                        "skipped_waypoints": skipped_count,
                        "min_sample_count": self.min_sample_count,
                    },
                )
                self._publish_status(
                    "semi_auto_insufficient_samples",
                    response.message,
                    {
                        "session_dir": str(self.session_dir) if self.session_dir is not None else None,
                        "accepted_samples": captured_count,
                        "skipped_waypoints": skipped_count,
                        "min_sample_count": self.min_sample_count,
                    },
                )
                return response

            solve_response = self._solve_samples(Trigger.Response())
            if not solve_response.success:
                raise RuntimeError(solve_response.message)
            self._save_current_solution()
            response.success = True
            response.message = (
                f"Semi-auto calibration finished with {captured_count} accepted samples and {skipped_count} skipped waypoints."
            )
            self._publish_status(
                "semi_auto_finished" if skipped_count == 0 else "semi_auto_finished_with_skips",
                response.message,
                {
                    "session_dir": str(self.session_dir) if self.session_dir is not None else None,
                    "report_path": str(self.report_path) if self.report_path is not None else None,
                    "accepted_samples": captured_count,
                    "skipped_waypoints": skipped_count,
                },
            )
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            if started_session_dir is not None:
                self._append_run_log("semi_auto_failed", {"error": response.message})
            self._publish_status("semi_auto_failed", response.message, {"session_dir": started_session_dir})
        finally:
            with self._semi_auto_lock:
                self._semi_auto_active = False
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
