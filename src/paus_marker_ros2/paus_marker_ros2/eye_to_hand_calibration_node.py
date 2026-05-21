from __future__ import annotations

# 导入 json，用于发布结构化状态。
import json
# 导入 dataclass，便于保存单次标定样本。
from dataclasses import asdict, dataclass
# 导入 Path，便于处理输出路径。
from pathlib import Path
import math
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
from rcl_interfaces.msg import SetParametersResult
# 导入消息与服务类型。
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .board_observation import (
    DEPTH_ALIGNED_MODE,
    HYBRID_COMPARE_MODE,
    RGB_PNP_MODE,
    BoardPoseEstimate,
    DepthObservationQualityConfig,
    compare_board_pose_estimates,
    estimate_depth_aligned_board_pose,
    estimate_rgb_pnp_board_pose,
    normalize_observation_mode,
)
from .semi_auto_calibration import (
    CalibrationTrajectory,
    TrajectoryValidationError,
    build_recorded_waypoint,
    create_session_dir,
    empty_trajectory,
    load_trajectory,
    sample_log_targets,
    save_trajectory,
    session_owner_matches,
)
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


STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _normalize_solver_method(value: str) -> str:
    method = value.strip().lower()
    if method in {"joint_absolute", "joint-absolute", "jointabsolute"}:
        return "ax_xb_park"
    return method


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
    # 本样本使用的观测模式。
    observation_mode: str
    # 单样本质量字段，供 UI / 日志判断样本是否可靠。
    sample_quality: dict[str, object]
    # 可选保存的样本图像路径。
    image_path: str | None = None
    # hybrid_compare 诊断时保留两条链路的结果。
    camera_to_board_rgb_pnp_matrix: np.ndarray | None = None
    camera_to_board_depth_aligned_matrix: np.ndarray | None = None
    hybrid_comparison: dict[str, object] | None = None


@dataclass
class CapturedImage:
    image_bgr: np.ndarray
    header_time_s: float | None
    received_time_s: float
    sequence: int


@dataclass
class CapturedDepth:
    depth_m: np.ndarray
    header_time_s: float | None
    received_time_s: float
    sequence: int


class SampleRejectedError(RuntimeError):
    def __init__(self, reject_reason: str, sample_quality: dict[str, object]) -> None:
        super().__init__(reject_reason)
        self.reject_reason = reject_reason
        self.sample_quality = sample_quality


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
        calibration_cfg = self.config.get("calibration", {})
        control_cfg = self.config["control"]
        # 在拿到主配置后，再声明机器人相关参数，允许后续显式覆盖。
        self.declare_parameter("robot_ip", str(control_cfg["robot_ip"]))
        self.declare_parameter("linux_fairino_sdk_root", str(control_cfg["linux_fairino_sdk_root"]))#在你这台 Linux/Ubuntu 机器上，FAIRINO 提供的 Python SDK 文件放在这个目录里。
        self.declare_parameter("observation_mode", str(calibration_cfg.get("observation_mode", RGB_PNP_MODE)))
        self.declare_parameter("depth_topic", str(calibration_cfg.get("depth_topic", "/camera/depth_aligned")))
        self.declare_parameter("max_rgb_depth_delta_ms", float(calibration_cfg.get("max_rgb_depth_delta_ms", 100.0)))
        self.declare_parameter("corner_patch_size_px", int(calibration_cfg.get("corner_patch_size_px", 5)))
        self.declare_parameter("corner_min_points", int(calibration_cfg.get("corner_min_points", 8)))
        self.declare_parameter("min_valid_corner_patch_ratio", float(calibration_cfg.get("min_valid_corner_patch_ratio", 0.80)))
        self.declare_parameter("max_local_plane_residual_median_mm", float(calibration_cfg.get("max_local_plane_residual_median_mm", 1.0)))
        self.declare_parameter("max_local_plane_residual_p95_mm", float(calibration_cfg.get("max_local_plane_residual_p95_mm", 2.0)))
        self.declare_parameter("min_global_plane_selected_ratio", float(calibration_cfg.get("min_global_plane_selected_ratio", 0.70)))
        self.declare_parameter("max_board_model_fit_rmse_mm", float(calibration_cfg.get("max_board_model_fit_rmse_mm", 2.0)))
        self.declare_parameter("max_board_model_fit_max_mm", float(calibration_cfg.get("max_board_model_fit_max_mm", 5.0)))
        self.declare_parameter("min_board_center_z_m", float(calibration_cfg.get("min_board_center_z_m", 0.20)))
        self.declare_parameter("max_board_center_z_m", float(calibration_cfg.get("max_board_center_z_m", 0.80)))
        self.declare_parameter("tool_id", int(control_cfg.get("tool_id", 0)))
        self.declare_parameter("user_id", int(control_cfg.get("user_id", 0)))
        self.declare_parameter("move_vel", float(control_cfg.get("move_vel", 10.0)))
        self.declare_parameter("move_acc", float(control_cfg.get("move_acc", 10.0)))
        self.declare_parameter("execute_motion", bool(control_cfg.get("execute_motion", False)))
        self.declare_parameter(
            "trajectory_path",
            str(calibration_cfg.get("trajectory_path", bringup_share / "configs" / "eye_to_hand_trajectory.yaml")),
        )
        self.declare_parameter(
            "session_root_path",
            str(calibration_cfg.get("session_root_path", "/home/chen_lab/paus_robot/calibration_sessions")),
        )
        self.declare_parameter("save_sample_images", bool(calibration_cfg.get("save_sample_images", True)))
        self.declare_parameter("max_reprojection_error_px", float(calibration_cfg.get("max_reprojection_error_px", 2.5)))
        self.declare_parameter("min_board_margin_px", float(calibration_cfg.get("min_board_margin_px", 10.0)))
        self.declare_parameter("stable_position_tolerance_mm", float(calibration_cfg.get("stable_position_tolerance_mm", 0.2)))
        self.declare_parameter("stable_rotation_tolerance_deg", float(calibration_cfg.get("stable_rotation_tolerance_deg", 0.1)))
        self.declare_parameter("stable_window_s", float(calibration_cfg.get("stable_window_s", 0.5)))
        self.declare_parameter("stable_timeout_s", float(calibration_cfg.get("stable_timeout_s", 10.0)))
        self.declare_parameter("dwell_s", float(calibration_cfg.get("dwell_s", 0.5)))

        camera_config_path = self.get_parameter("camera_config_path").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.board_rows = int(self.get_parameter("board_rows").get_parameter_value().integer_value)
        self.board_cols = int(self.get_parameter("board_cols").get_parameter_value().integer_value)
        self.square_size_m = float(self.get_parameter("square_size_m").get_parameter_value().double_value)
        self.solver_method = _normalize_solver_method(
            self.get_parameter("solver_method").get_parameter_value().string_value
        )
        if self.solver_method == "ax_xb_park" and self.get_parameter("solver_method").get_parameter_value().string_value.strip().lower() != "ax_xb_park":
            self.get_logger().warning(
                "solver_method='joint_absolute' is deprecated; normalized to 'ax_xb_park'."
            )
        self.observation_mode = normalize_observation_mode(self.get_parameter("observation_mode").get_parameter_value().string_value)
        self.fresh_image_timeout_s = float(self.get_parameter("fresh_image_timeout_s").get_parameter_value().double_value)
        self.max_rgb_depth_delta_ms = float(self.get_parameter("max_rgb_depth_delta_ms").get_parameter_value().double_value)
        sample_log_path_value = self.get_parameter("sample_log_path").get_parameter_value().string_value.strip()
        self.sample_log_override_path = Path(sample_log_path_value) if sample_log_path_value else None
        self.tool_to_board_translation = [float(value) for value in self.get_parameter("tool_to_board.translation_m").get_parameter_value().double_array_value]
        self.tool_to_board_rotation_rpy = [float(value) for value in self.get_parameter("tool_to_board.rotation_rpy_deg").get_parameter_value().double_array_value]
        self.min_sample_count = int(self.get_parameter("min_sample_count").get_parameter_value().integer_value)
        self.output_path = Path(self.get_parameter("output_path").get_parameter_value().string_value)
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.linux_fairino_sdk_root = self.get_parameter("linux_fairino_sdk_root").get_parameter_value().string_value
        self.tool_id = int(self.get_parameter("tool_id").get_parameter_value().integer_value)
        self.user_id = int(self.get_parameter("user_id").get_parameter_value().integer_value)
        self.move_vel = float(self.get_parameter("move_vel").get_parameter_value().double_value)
        self.move_acc = float(self.get_parameter("move_acc").get_parameter_value().double_value)
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)
        self.trajectory_path = Path(self.get_parameter("trajectory_path").get_parameter_value().string_value)
        self.session_root_path = Path(self.get_parameter("session_root_path").get_parameter_value().string_value)
        self.save_sample_images = bool(self.get_parameter("save_sample_images").get_parameter_value().bool_value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").get_parameter_value().double_value)
        self.min_board_margin_px = float(self.get_parameter("min_board_margin_px").get_parameter_value().double_value)
        self.stable_position_tolerance_mm = float(self.get_parameter("stable_position_tolerance_mm").get_parameter_value().double_value)
        self.stable_rotation_tolerance_deg = float(self.get_parameter("stable_rotation_tolerance_deg").get_parameter_value().double_value)
        self.stable_window_s = float(self.get_parameter("stable_window_s").get_parameter_value().double_value)
        self.stable_timeout_s = float(self.get_parameter("stable_timeout_s").get_parameter_value().double_value)
        self.dwell_s = float(self.get_parameter("dwell_s").get_parameter_value().double_value)
        self.session_dir: Path | None = None
        self.session_owner: str | None = None
        self.sample_log_path: Path | None = None
        self.report_path: Path | None = None
        self.run_log_path: Path | None = None
        self.recorded_trajectory = self._load_or_create_trajectory_for_recording()
        self._semi_auto_lock = threading.Lock()
        self._parameter_update_lock = threading.Lock()
        self._semi_auto_active = False

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
        self._depth_condition = threading.Condition()
        self.latest_depth: CapturedDepth | None = None
        self.depth_sequence = 0
        # 保存已采集样本和当前求解结果。
        self.samples: list[CalibrationSample] = []
        self.current_solution: EyeToHandCalibrationSolution | None = None
        self.callback_group = ReentrantCallbackGroup()
        # 建立 Linux SDK 客户端，用于直接读取当前 TCP。
        self.linux_client = FairinoLinuxClient(self.linux_fairino_sdk_root, self.robot_ip)
        self.linux_client.connect()

        # 创建图像订阅器与状态发布器。
        self.image_subscription = self.create_subscription(Image, self.image_topic, self._image_callback, 10, callback_group=self.callback_group)
        self.depth_subscription = self.create_subscription(Image, self.depth_topic, self._depth_callback, 10, callback_group=self.callback_group)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self.status_publisher = self.create_publisher(String, self.status_topic, STATUS_QOS)

        # 创建“采样 / 求解 / 保存”以及半自动轨迹服务接口。
        self.capture_service = self.create_service(Trigger, "/eye_to_hand/capture_sample", self._capture_sample_callback, callback_group=self.callback_group)
        self.solve_service = self.create_service(Trigger, "/eye_to_hand/solve", self._solve_callback, callback_group=self.callback_group)
        self.save_service = self.create_service(Trigger, "/eye_to_hand/save", self._save_callback, callback_group=self.callback_group)
        self.record_waypoint_service = self.create_service(Trigger, "/eye_to_hand/record_waypoint", self._record_waypoint_callback, callback_group=self.callback_group)
        self.delete_waypoint_service = self.create_service(Trigger, "/eye_to_hand/delete_last_waypoint", self._delete_last_waypoint_callback, callback_group=self.callback_group)
        self.save_trajectory_service = self.create_service(Trigger, "/eye_to_hand/save_trajectory", self._save_trajectory_callback, callback_group=self.callback_group)
        self.run_semi_auto_service = self.create_service(Trigger, "/eye_to_hand/run_semi_auto_calibration", self._run_semi_auto_callback, callback_group=self.callback_group)

        # 节点启动后发布初始状态。
        self._append_run_log(
            "node_started",
            {
                "trajectory_path": str(self.trajectory_path),
                "sample_log_override_path": str(self.sample_log_override_path) if self.sample_log_override_path else None,
                "session_root_path": str(self.session_root_path),
            },
        )
        self._publish_status(
            "ready",
            "Eye-to-hand calibration node started.",
            {
                "trajectory_path": str(self.trajectory_path),
                "session_dir": str(self.session_dir) if self.session_dir else None,
                "sample_log_path": str(self.sample_log_path) if self.sample_log_path else None,
                "sample_log_override_path": str(self.sample_log_override_path) if self.sample_log_override_path else None,
            },
        )

    def _uses_depth_observation(self) -> bool:
        return self.observation_mode in {DEPTH_ALIGNED_MODE, HYBRID_COMPARE_MODE}

    def _build_depth_quality_config(self) -> DepthObservationQualityConfig:
        return DepthObservationQualityConfig(
            corner_patch_size_px=int(self.get_parameter("corner_patch_size_px").get_parameter_value().integer_value),
            corner_min_points=int(self.get_parameter("corner_min_points").get_parameter_value().integer_value),
            max_rgb_depth_delta_ms=float(self.get_parameter("max_rgb_depth_delta_ms").get_parameter_value().double_value),
            min_valid_corner_patch_ratio=float(self.get_parameter("min_valid_corner_patch_ratio").get_parameter_value().double_value),
            max_local_plane_residual_median_mm=float(self.get_parameter("max_local_plane_residual_median_mm").get_parameter_value().double_value),
            max_local_plane_residual_p95_mm=float(self.get_parameter("max_local_plane_residual_p95_mm").get_parameter_value().double_value),
            min_global_plane_selected_ratio=float(self.get_parameter("min_global_plane_selected_ratio").get_parameter_value().double_value),
            max_board_model_fit_rmse_mm=float(self.get_parameter("max_board_model_fit_rmse_mm").get_parameter_value().double_value),
            max_board_model_fit_max_mm=float(self.get_parameter("max_board_model_fit_max_mm").get_parameter_value().double_value),
            min_board_center_z_m=float(self.get_parameter("min_board_center_z_m").get_parameter_value().double_value),
            max_board_center_z_m=float(self.get_parameter("max_board_center_z_m").get_parameter_value().double_value),
        )

    # 接收最新图像。
    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        changed: dict[str, object] = {}
        try:
            for parameter in parameters:
                if parameter.name == "observation_mode":
                    changed["observation_mode"] = normalize_observation_mode(parameter.value)
                elif parameter.name == "depth_topic":
                    depth_topic = str(parameter.value).strip()
                    if not depth_topic:
                        raise ValueError("depth_topic cannot be empty.")
                    changed["depth_topic"] = depth_topic
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        observation_mode_changed = False
        depth_topic_changed = False
        with self._parameter_update_lock:
            if "observation_mode" in changed and changed["observation_mode"] != self.observation_mode:
                self.observation_mode = str(changed["observation_mode"])
                observation_mode_changed = True
            if "depth_topic" in changed and changed["depth_topic"] != self.depth_topic:
                self.depth_topic = str(changed["depth_topic"])
                depth_topic_changed = True
                try:
                    if self.depth_subscription is not None:
                        self.destroy_subscription(self.depth_subscription)
                except Exception:
                    pass
                self.depth_subscription = self.create_subscription(Image, self.depth_topic, self._depth_callback, 10, callback_group=self.callback_group)

        if observation_mode_changed or depth_topic_changed:
            self._append_run_log(
                "observation_mode_changed",
                {
                    "observation_mode": self.observation_mode,
                    "depth_topic": self.depth_topic,
                },
            )
            self._publish_status(
                "observation_mode_changed",
                f"Observation mode set to {self.observation_mode}.",
                {
                    "observation_mode": self.observation_mode,
                    "depth_topic": self.depth_topic,
                    "session_dir": str(self.session_dir) if self.session_dir else None,
                },
            )
        return SetParametersResult(successful=True)

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

    def _depth_callback(self, message: Image) -> None:
        depth_m = self.bridge.imgmsg_to_cv2(message, desired_encoding="32FC1")
        header_time_s = None
        if message.header.stamp.sec != 0 or message.header.stamp.nanosec != 0:
            header_time_s = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        with self._depth_condition:
            self.depth_sequence += 1
            self.latest_depth = CapturedDepth(
                depth_m=np.asarray(depth_m, dtype=np.float32).copy(),
                header_time_s=header_time_s,
                received_time_s=time.monotonic_ns() * 1e-9,
                sequence=self.depth_sequence,
            )
            self._depth_condition.notify_all()

    # 发布标定状态。
    def _publish_status(self, status: str, message: str, extra: dict[str, object] | None = None) -> None:
        payload = {
            "status": status,
            "message": message,
            "sample_count": len(self.samples),
            "min_sample_count": self.min_sample_count,
            "solver_method": self.solver_method,
            "observation_mode": self.observation_mode,
        }
        if extra:
            payload.update(extra)
        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(status_message)
        self.get_logger().info(status_message.data)

    def _append_run_log(self, event: str, payload: dict[str, object] | None = None) -> None:
        run_log_path = getattr(self, "run_log_path", None)
        if run_log_path is None:
            return
        record: dict[str, object] = {
            "event": event,
            "monotonic_time_s": time.monotonic(),
            "sample_count": len(getattr(self, "samples", [])),
            "session_owner": getattr(self, "session_owner", None),
        }
        if payload:
            record.update(payload)
        run_log_path.parent.mkdir(parents=True, exist_ok=True)
        with run_log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _ensure_session_started(self, owner: str = "manual") -> None:
        session_dir = getattr(self, "session_dir", None)
        if session_dir is not None and session_owner_matches(getattr(self, "session_owner", None), owner):
            return
        self.session_dir = create_session_dir(self.session_root_path)
        self.session_owner = owner
        self.sample_log_path = self.session_dir / "samples.jsonl"
        self.report_path = self.session_dir / "report.yaml"
        self.run_log_path = self.session_dir / "run.log"
        self.samples = []
        self.current_solution = None
        self._append_run_log(
            "session_started",
            {
                "session_dir": str(self.session_dir),
                "trajectory_path": str(getattr(self, "trajectory_path", "")),
                "sample_log_path": str(self.sample_log_path),
                "sample_log_override_path": str(self.sample_log_override_path) if getattr(self, "sample_log_override_path", None) else None,
                "observation_mode": getattr(self, "observation_mode", RGB_PNP_MODE),
            },
        )

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

    def _write_recorded_trajectory(self) -> None:
        save_trajectory(self.recorded_trajectory, self.trajectory_path)

    def _archive_current_trajectory(self) -> None:
        if getattr(self, "session_dir", None) is None or not self.trajectory_path.exists():
            return
        shutil.copy2(self.trajectory_path, self.session_dir / "trajectory_used.yaml")

    def _reject_manual_service_if_semi_auto_active(
        self,
        response: Trigger.Response,
        *,
        status: str = "manual_service_rejected",
        message: str = "Semi-auto calibration is currently running.",
    ) -> bool:
        if not bool(getattr(self, "_semi_auto_active", False)):
            return False
        session_dir = getattr(self, "session_dir", None)
        response.success = False
        response.message = message
        self._publish_status(status, message, {"session_dir": str(session_dir) if session_dir else None})
        return True

    def _write_report(self, payload: dict[str, object]) -> None:
        report_path = getattr(self, "report_path", None)
        if report_path is None:
            return
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with report_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)

    def _save_sample_image(self, image_bgr: np.ndarray, sample_index: int) -> str | None:
        if not bool(getattr(self, "save_sample_images", False)) or getattr(self, "session_dir", None) is None:
            return None
        image_path = self.session_dir / "images" / f"sample_{sample_index:03d}.png"
        image_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(image_path), image_bgr)
        return str(image_path)

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

    def _wait_for_fresh_depth(self, previous_sequence: int, rgb_header_time_s: float | None) -> CapturedDepth:
        deadline = time.monotonic() + self.fresh_image_timeout_s
        with self._depth_condition:
            while True:
                latest = self.latest_depth
                if latest is not None and latest.sequence > previous_sequence:
                    delta_ms = self._timestamp_delta_ms(rgb_header_time_s, latest.header_time_s)
                    if delta_ms is None or delta_ms <= self.max_rgb_depth_delta_ms:
                        return CapturedDepth(
                            depth_m=latest.depth_m.copy(),
                            header_time_s=latest.header_time_s,
                            received_time_s=latest.received_time_s,
                            sequence=latest.sequence,
                        )
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0.0:
                    raise RuntimeError(f"No matching depth frame arrived within {self.fresh_image_timeout_s:.3f}s.")
                self._depth_condition.wait(timeout=remaining_s)

    def _timestamp_delta_ms(self, lhs_time_s: float | None, rhs_time_s: float | None) -> float | None:
        if lhs_time_s is None or rhs_time_s is None:
            return None
        return abs(float(lhs_time_s) - float(rhs_time_s)) * 1000.0

    # 从当前观测模式估计 `camera -> board` 变换。
    def _estimate_camera_to_board_observation(self, captured_image: CapturedImage, captured_depth: CapturedDepth | None) -> BoardPoseEstimate:
        camera_matrix = np.asarray(self.camera_calibration.camera_matrix, dtype=np.float64)
        dist_coeffs = np.asarray(self.camera_calibration.dist_coeffs, dtype=np.float64)
        if self.observation_mode == RGB_PNP_MODE:
            return estimate_rgb_pnp_board_pose(
                captured_image.image_bgr,
                board_rows=self.board_rows,
                board_cols=self.board_cols,
                square_size_m=self.square_size_m,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
            )
        if captured_depth is None:
            return BoardPoseEstimate(
                camera_to_board_matrix=None,
                quality={"observation_mode": self.observation_mode, "accepted": False, "status": "rejected", "reject_reason": "depth_frame_missing"},
            )

        rgb_depth_delta_ms = self._timestamp_delta_ms(captured_image.header_time_s, captured_depth.header_time_s)
        depth_estimate = estimate_depth_aligned_board_pose(
            captured_image.image_bgr,
            captured_depth.depth_m,
            board_rows=self.board_rows,
            board_cols=self.board_cols,
            square_size_m=self.square_size_m,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            quality_config=self._build_depth_quality_config(),
            rgb_depth_delta_ms=rgb_depth_delta_ms,
        )
        if self.observation_mode == DEPTH_ALIGNED_MODE:
            return depth_estimate
        if self.observation_mode == HYBRID_COMPARE_MODE:
            rgb_estimate = estimate_rgb_pnp_board_pose(
                captured_image.image_bgr,
                board_rows=self.board_rows,
                board_cols=self.board_cols,
                square_size_m=self.square_size_m,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
            )
            quality = dict(depth_estimate.quality)
            quality["observation_mode"] = HYBRID_COMPARE_MODE
            quality["rgb_pnp_quality"] = rgb_estimate.quality
            quality["depth_aligned_quality"] = depth_estimate.quality
            if rgb_estimate.camera_to_board_matrix is not None and depth_estimate.camera_to_board_matrix is not None:
                quality["hybrid_comparison"] = compare_board_pose_estimates(rgb_estimate.camera_to_board_matrix, depth_estimate.camera_to_board_matrix)
            return BoardPoseEstimate(camera_to_board_matrix=depth_estimate.camera_to_board_matrix, quality=quality, corners_xy=depth_estimate.corners_xy)
        raise RuntimeError(f"Unsupported observation_mode: {self.observation_mode}")

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

    def _compute_board_margin_px(self, corners_xy: np.ndarray, image_shape: tuple[int, int, int] | tuple[int, int]) -> float:
        corners = np.asarray(corners_xy, dtype=np.float64).reshape((-1, 2))
        height, width = image_shape[:2]
        return float(
            min(
                np.min(corners[:, 0]),
                np.min(corners[:, 1]),
                width - 1.0 - np.max(corners[:, 0]),
                height - 1.0 - np.max(corners[:, 1]),
            )
        )

    def _probe_current_board_quality(self) -> dict[str, object]:
        with self._image_condition:
            latest_image = self.latest_image
        if latest_image is None:
            return {
                "detected": False,
                "accepted": False,
                "status": "rejected",
                "reject_reason": "image_frame_missing",
                "observation_mode": self.observation_mode,
            }
        captured_image = CapturedImage(
            image_bgr=latest_image.image_bgr.copy(),
            header_time_s=latest_image.header_time_s,
            received_time_s=latest_image.received_time_s,
            sequence=latest_image.sequence,
        )
        captured_depth = None
        if self._uses_depth_observation():
            with self._depth_condition:
                latest_depth = self.latest_depth
            if latest_depth is not None:
                captured_depth = CapturedDepth(
                    depth_m=latest_depth.depth_m.copy(),
                    header_time_s=latest_depth.header_time_s,
                    received_time_s=latest_depth.received_time_s,
                    sequence=latest_depth.sequence,
                )
        observation = self._estimate_camera_to_board_observation(captured_image, captured_depth)
        quality = dict(observation.quality)
        if "reprojection_rms_px" in quality and "reprojection_error_px" not in quality:
            quality["reprojection_error_px"] = quality["reprojection_rms_px"]
        if "reprojection_error_px" in quality and "reprojection_rms_px" not in quality:
            quality["reprojection_rms_px"] = quality["reprojection_error_px"]
        if observation.corners_xy is not None and "board_margin_px" not in quality:
            quality["board_margin_px"] = self._compute_board_margin_px(observation.corners_xy, captured_image.image_bgr.shape)
        return quality

    def _sample_to_log_record(self, sample: CalibrationSample) -> dict[str, object]:
        tcp_mid_time_s = (sample.tcp_read_start_time_s + sample.tcp_read_end_time_s) * 0.5
        record: dict[str, object] = {
            "sample_index": len(self.samples),
            "image_sequence": sample.image_sequence,
            "image_header_time_s": sample.image_header_time_s,
            "image_received_time_s": sample.image_received_time_s,
            "tcp_read_start_time_s": sample.tcp_read_start_time_s,
            "tcp_read_end_time_s": sample.tcp_read_end_time_s,
            "image_to_tcp_midpoint_age_ms": (tcp_mid_time_s - sample.image_received_time_s) * 1000.0,
            "tcp_pose_mmdeg": sample.tcp_pose_mmdeg,
            "observation_mode": sample.observation_mode,
            "sample_quality": sample.sample_quality,
            "image_path": sample.image_path,
            "base_to_tool_matrix": sample.base_to_tool_matrix.tolist(),
            "camera_to_board_matrix": sample.camera_to_board_matrix.tolist(),
        }
        if sample.camera_to_board_rgb_pnp_matrix is not None:
            record["camera_to_board_rgb_pnp_matrix"] = sample.camera_to_board_rgb_pnp_matrix.tolist()
        if sample.camera_to_board_depth_aligned_matrix is not None:
            record["camera_to_board_depth_aligned_matrix"] = sample.camera_to_board_depth_aligned_matrix.tolist()
        if sample.hybrid_comparison is not None:
            record["hybrid_comparison"] = sample.hybrid_comparison
        return record

    def _append_sample_log(self, sample: CalibrationSample) -> None:
        record = json.dumps(self._sample_to_log_record(sample), ensure_ascii=False) + "\n"
        for path in sample_log_targets(self.sample_log_path, self.sample_log_override_path):
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as handle:
                handle.write(record)

    # 使用旧版“直接平均 base->camera”的方式求解。
    def _solve_board_average(self) -> np.ndarray:
        tool_to_board = self._tool_to_board_matrix()
        matrices = [
            sample.base_to_tool_matrix @ tool_to_board @ invert_transform_matrix(sample.camera_to_board_matrix)
            for sample in self.samples
        ]
        return average_transform_matrices(matrices)

    def _axis_angle_deg(self, lhs_axis: np.ndarray, rhs_axis: np.ndarray) -> float:
        lhs = np.asarray(lhs_axis, dtype=np.float64).reshape(3)
        rhs = np.asarray(rhs_axis, dtype=np.float64).reshape(3)
        lhs /= max(np.linalg.norm(lhs), 1e-12)
        rhs /= max(np.linalg.norm(rhs), 1e-12)
        cosine = float(np.clip(abs(np.dot(lhs, rhs)), -1.0, 1.0))
        return float(np.degrees(np.arccos(cosine)))

    def _summarize_sample_quality(self) -> dict[str, object]:
        summary: dict[str, object] = {
            "observation_mode": self.observation_mode,
            "accepted_sample_count": len(self.samples),
        }
        keys = (
            "valid_corner_patch_ratio",
            "plane_residual_std_mm_median",
            "plane_residual_std_mm_p95",
            "board_model_fit_rmse_mm",
            "board_model_fit_max_mm",
            "reprojection_rms_px",
        )
        collected: dict[str, list[float]] = {key: [] for key in keys}
        for sample in self.samples:
            quality = sample.sample_quality
            if not isinstance(quality, dict):
                continue
            if "valid_corner_patch_ratio" in quality:
                collected["valid_corner_patch_ratio"].append(float(quality["valid_corner_patch_ratio"]))
            if "plane_residual_std_mm_median" in quality:
                collected["plane_residual_std_mm_median"].append(float(quality["plane_residual_std_mm_median"]))
            if "plane_residual_std_mm_p95" in quality:
                collected["plane_residual_std_mm_p95"].append(float(quality["plane_residual_std_mm_p95"]))
            if "board_model_fit_rmse_mm" in quality:
                collected["board_model_fit_rmse_mm"].append(float(quality["board_model_fit_rmse_mm"]))
            if "board_model_fit_max_mm" in quality:
                collected["board_model_fit_max_mm"].append(float(quality["board_model_fit_max_mm"]))
            if "reprojection_rms_px" in quality:
                collected["reprojection_rms_px"].append(float(quality["reprojection_rms_px"]))
        for key, values in collected.items():
            if not values:
                continue
            array = np.asarray(values, dtype=np.float64)
            summary[key] = {
                "mean": float(np.mean(array)),
                "min": float(np.min(array)),
                "max": float(np.max(array)),
            }
        return summary

    def _evaluate_session_diagnostics(self, solved_matrix: np.ndarray, tool_to_board: np.ndarray) -> dict[str, object]:
        base_to_camera = np.asarray(solved_matrix, dtype=np.float64).reshape(4, 4)
        tool_to_board = np.asarray(tool_to_board, dtype=np.float64).reshape(4, 4)
        translation_z_errors_mm: list[float] = []
        normal_errors_deg: list[float] = []
        for sample in self.samples:
            observed_base_to_board = np.asarray(sample.base_to_tool_matrix, dtype=np.float64).reshape(4, 4) @ tool_to_board
            predicted_base_to_board = base_to_camera @ np.asarray(sample.camera_to_board_matrix, dtype=np.float64).reshape(4, 4)
            translation_z_errors_mm.append(float(abs(predicted_base_to_board[2, 3] - observed_base_to_board[2, 3]) * 1000.0))
            normal_errors_deg.append(self._axis_angle_deg(observed_base_to_board[:3, 2], predicted_base_to_board[:3, 2]))
        z_array = np.asarray(translation_z_errors_mm, dtype=np.float64)
        n_array = np.asarray(normal_errors_deg, dtype=np.float64)
        return {
            "translation_z_mean_mm": float(np.mean(z_array)) if z_array.size else math.nan,
            "translation_z_max_mm": float(np.max(z_array)) if z_array.size else math.nan,
            "board_normal_error_mean_deg": float(np.mean(n_array)) if n_array.size else math.nan,
            "board_normal_error_max_deg": float(np.max(n_array)) if n_array.size else math.nan,
            "sample_quality_summary": self._summarize_sample_quality(),
        }

    def _capture_one_sample(self, *, owner: str) -> CalibrationSample:
        self._ensure_session_started(owner=owner)
        with self._image_condition:
            previous_sequence = self.image_sequence
        with self._depth_condition:
            previous_depth_sequence = self.depth_sequence
        captured_image = self._wait_for_fresh_image(previous_sequence)
        captured_depth = None
        if self._uses_depth_observation():
            captured_depth = self._wait_for_fresh_depth(previous_depth_sequence, captured_image.header_time_s)
        base_to_tool, tcp_pose_mmdeg, tcp_read_start_time_s, tcp_read_end_time_s = self._read_current_base_to_tool()
        observation = self._estimate_camera_to_board_observation(captured_image, captured_depth)
        if observation.camera_to_board_matrix is None or not bool(observation.quality.get("accepted", False)):
            reject_reason = str(observation.quality.get("reject_reason", "sample_quality_rejected"))
            raise SampleRejectedError(reject_reason, observation.quality)
        rgb_matrix = None
        depth_matrix = None
        hybrid_comparison = None
        if self.observation_mode == RGB_PNP_MODE:
            rgb_matrix = observation.camera_to_board_matrix
        elif self.observation_mode == DEPTH_ALIGNED_MODE:
            depth_matrix = observation.camera_to_board_matrix
        elif self.observation_mode == HYBRID_COMPARE_MODE:
            depth_matrix = observation.camera_to_board_matrix
            hybrid_comparison = observation.quality.get("hybrid_comparison") if isinstance(observation.quality.get("hybrid_comparison"), dict) else None
        sample_quality = dict(observation.quality)
        if "reprojection_rms_px" in sample_quality and "reprojection_error_px" not in sample_quality:
            sample_quality["reprojection_error_px"] = sample_quality["reprojection_rms_px"]
        if "reprojection_error_px" in sample_quality and "reprojection_rms_px" not in sample_quality:
            sample_quality["reprojection_rms_px"] = sample_quality["reprojection_error_px"]
        if observation.corners_xy is not None and "board_margin_px" not in sample_quality:
            sample_quality["board_margin_px"] = self._compute_board_margin_px(observation.corners_xy, captured_image.image_bgr.shape)
        image_path = self._save_sample_image(captured_image.image_bgr, len(self.samples) + 1)
        sample = CalibrationSample(
            base_to_tool_matrix=base_to_tool,
            camera_to_board_matrix=observation.camera_to_board_matrix,
            image_header_time_s=captured_image.header_time_s,
            image_received_time_s=captured_image.received_time_s,
            tcp_read_start_time_s=tcp_read_start_time_s,
            tcp_read_end_time_s=tcp_read_end_time_s,
            tcp_pose_mmdeg=tcp_pose_mmdeg,
            image_sequence=captured_image.sequence,
            observation_mode=self.observation_mode,
            sample_quality=sample_quality,
            image_path=image_path,
            camera_to_board_rgb_pnp_matrix=rgb_matrix,
            camera_to_board_depth_aligned_matrix=depth_matrix,
            hybrid_comparison=hybrid_comparison,
        )
        self.samples.append(sample)
        self._append_sample_log(sample)
        self._append_run_log("sample_captured", self._sample_to_log_record(sample))
        return sample

    # 采集一次样本。
    def _capture_sample_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response, status="capture_rejected"):
            return response
        try:
            sample = self._capture_one_sample(owner="manual")
            response.success = True
            response.message = f"Captured sample #{len(self.samples)}."
            self._publish_status(
                "sample_captured",
                response.message,
                {
                    "robot_ip": self.robot_ip,
                    "sdk_root": self.linux_fairino_sdk_root,
                    "sample_log_path": str(self.sample_log_path) if self.sample_log_path else None,
                    "capture_timing": self._sample_to_log_record(sample),
                    "sample_quality": sample.sample_quality,
                    "session_dir": str(self.session_dir) if self.session_dir else None,
                },
            )
        except SampleRejectedError as exc:
            response.success = False
            response.message = f"Sample rejected: {exc.reject_reason}."
            self._publish_status("sample_rejected", response.message, {"sample_quality": exc.sample_quality, "session_dir": str(self.session_dir) if self.session_dir else None})
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("capture_failed", response.message, {"session_dir": str(self.session_dir) if self.session_dir else None})
        return response

    def _solve_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._ensure_session_started(owner=getattr(self, "session_owner", None) or "manual")
        if len(self.samples) < self.min_sample_count:
            response.success = False
            response.message = f"Need at least {self.min_sample_count} samples, currently {len(self.samples)}."
            self._publish_status("solve_failed", response.message, {"session_dir": str(self.session_dir) if self.session_dir else None})
            return response
        try:
            tool_to_board = self._tool_to_board_matrix()
            solver_matrices: dict[str, np.ndarray | None] = {}
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
            diagnostics = self._evaluate_session_diagnostics(solved_matrix, tool_to_board)
            self.current_solution = EyeToHandCalibrationSolution(
                status="ok",
                success=True,
                sample_count=len(self.samples),
                message="Eye-to-hand calibration solved successfully.",
                base_to_camera=transform,
                tool_to_board_translation_m=list(self.tool_to_board_translation),
                tool_to_board_rotation_rpy_deg=list(self.tool_to_board_rotation_rpy),
                method=solve_method,
                observation_mode=self.observation_mode,
                residuals=residuals,
                session_diagnostics=diagnostics,
                sample_quality_summary=diagnostics.get("sample_quality_summary") if isinstance(diagnostics.get("sample_quality_summary"), dict) else None,
            )
            report_payload = {
                "session_dir": str(self.session_dir) if self.session_dir else None,
                "trajectory_path": str(self.trajectory_path),
                "sample_log_path": str(self.sample_log_path) if self.sample_log_path else None,
                "sample_log_override_path": str(self.sample_log_override_path) if self.sample_log_override_path else None,
                "sample_count": len(self.samples),
                "method": solve_method,
                "observation_mode": self.observation_mode,
                "base_to_camera": asdict(transform),
                "tool_to_board": {
                    "translation_m": list(self.tool_to_board_translation),
                    "rotation_rpy_deg": list(self.tool_to_board_rotation_rpy),
                },
                "residuals": asdict(residuals),
                "solver_residuals": solver_residuals,
                "session_diagnostics": diagnostics,
                "samples": [self._sample_to_log_record(sample) for sample in self.samples],
            }
            self._write_report(report_payload)
            self._append_run_log("solved", {"method": solve_method, "report_path": str(self.report_path) if self.report_path else None, "sample_count": len(self.samples)})
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
                    **diagnostics,
                    "session_dir": str(self.session_dir) if self.session_dir else None,
                    "report_path": str(self.report_path) if self.report_path else None,
                },
            )
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("solve_failed", response.message, {"session_dir": str(self.session_dir) if self.session_dir else None})
        return response

    def _save_current_solution(self) -> None:
        if self.current_solution is None or not self.current_solution.success:
            raise RuntimeError("No solved extrinsic is available.")
        before_path = self.session_dir / "extrinsics_before.yaml" if self.session_dir else None
        after_path = self.session_dir / "extrinsics_after.yaml" if self.session_dir else None
        if before_path is not None and self.output_path.exists():
            shutil.copy2(self.output_path, before_path)
        save_eye_to_hand_solution(self.current_solution, self.output_path)
        if after_path is not None:
            shutil.copy2(self.output_path, after_path)
        self._append_run_log(
            "extrinsics_saved",
            {
                "output_path": str(self.output_path),
                "extrinsics_before": str(before_path) if before_path is not None and before_path.exists() else None,
                "extrinsics_after": str(after_path) if after_path is not None else None,
            },
        )

    def _save_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response, status="save_rejected"):
            return response
        if self.current_solution is None or not self.current_solution.success:
            response.success = False
            response.message = "No solved extrinsic is available."
            self._publish_status("save_failed", response.message, {"session_dir": str(self.session_dir) if self.session_dir else None})
            return response
        try:
            self._save_current_solution()
            response.success = True
            response.message = f"Saved extrinsic to {self.output_path}."
            self._publish_status("saved", response.message, {"output_path": str(self.output_path), "session_dir": str(self.session_dir) if self.session_dir else None})
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("save_failed", response.message, {"session_dir": str(self.session_dir) if self.session_dir else None})
        return response

    def _record_waypoint_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response):
            return response
        try:
            self._ensure_session_started(owner="manual")
            session_dir = getattr(self, "session_dir", None)
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
            self._write_recorded_trajectory()
            payload = {
                "waypoint_name": waypoint.name,
                "waypoint": waypoint.to_payload(),
                "waypoint_count": len(self.recorded_trajectory.waypoints),
                "record_quality": record_quality,
                "trajectory_path": str(self.trajectory_path),
                "session_dir": str(session_dir) if session_dir else None,
            }
            self._append_run_log("waypoint_recorded", payload)
            response.success = True
            response.message = f"Recorded {waypoint.name} to {self.trajectory_path}."
            self._publish_status("waypoint_recorded", response.message, payload)
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("record_waypoint_failed", response.message, {"session_dir": str(getattr(self, "session_dir", None)) if getattr(self, "session_dir", None) else None})
        return response

    def _delete_last_waypoint_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response):
            return response
        if not self.recorded_trajectory.waypoints:
            response.success = False
            response.message = "No recorded waypoint to delete."
            session_dir = getattr(self, "session_dir", None)
            self._publish_status("delete_waypoint_failed", response.message, {"session_dir": str(session_dir) if session_dir else None})
            return response
        try:
            self._ensure_session_started(owner="manual")
            session_dir = getattr(self, "session_dir", None)
            removed = self.recorded_trajectory.waypoints.pop()
            if self.recorded_trajectory.waypoints:
                self._write_recorded_trajectory()
            elif getattr(self, "trajectory_path", None) is not None and self.trajectory_path.exists():
                self.trajectory_path.unlink()
            trajectory_path = getattr(self, "trajectory_path", None)
            payload = {
                "waypoint_name": removed.name,
                "waypoint": removed.to_payload(),
                "waypoint_count": len(self.recorded_trajectory.waypoints),
                "trajectory_path": str(trajectory_path) if trajectory_path is not None else None,
                "session_dir": str(session_dir) if session_dir else None,
            }
            self._append_run_log("waypoint_deleted", payload)
            response.success = True
            response.message = f"Deleted {removed.name} from {trajectory_path}."
            self._publish_status("waypoint_deleted", response.message, payload)
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("delete_waypoint_failed", response.message, {"session_dir": str(getattr(self, "session_dir", None)) if getattr(self, "session_dir", None) else None})
        return response

    def _save_trajectory_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._reject_manual_service_if_semi_auto_active(response):
            return response
        try:
            self._ensure_session_started(owner="manual")
            if not self.recorded_trajectory.waypoints:
                if self.trajectory_path.exists():
                    self.trajectory_path.unlink()
                response.success = True
                response.message = "Cleared empty trajectory state."
                self._publish_status(
                    "trajectory_cleared",
                    response.message,
                    {"trajectory_path": str(self.trajectory_path), "session_dir": str(self.session_dir) if self.session_dir else None},
                )
                self._append_run_log(
                    "trajectory_cleared",
                    {"trajectory_path": str(self.trajectory_path), "session_dir": str(self.session_dir) if self.session_dir else None},
                )
                return response
            self._write_recorded_trajectory()
            self._archive_current_trajectory()
            payload = {
                "trajectory_path": str(self.trajectory_path),
                "waypoint_count": len(self.recorded_trajectory.waypoints),
                "session_dir": str(self.session_dir) if self.session_dir else None,
                "session_owner": self.session_owner,
            }
            self._append_run_log("trajectory_saved", payload)
            response.success = True
            response.message = f"Saved {len(self.recorded_trajectory.waypoints)} waypoints to {self.trajectory_path}."
            self._publish_status("trajectory_saved", response.message, payload)
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("save_trajectory_failed", response.message, {"session_dir": str(self.session_dir) if self.session_dir else None})
        return response

    def _wait_until_tcp_stable(self) -> list[float]:
        deadline = time.monotonic() + self.stable_timeout_s
        stable_since: float | None = None
        previous_pose: list[float] | None = None
        last_pose: list[float] | None = None
        while time.monotonic() < deadline:
            error, pose = self.linux_client.get_actual_tcp_pose()
            if error != 0:
                raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
            last_pose = pose
            if previous_pose is not None:
                position_delta_mm = float(np.linalg.norm(np.asarray(pose[:3], dtype=np.float64) - np.asarray(previous_pose[:3], dtype=np.float64)))
                rotation_delta_deg = float(np.linalg.norm(np.asarray(pose[3:6], dtype=np.float64) - np.asarray(previous_pose[3:6], dtype=np.float64)))
                if position_delta_mm <= self.stable_position_tolerance_mm and rotation_delta_deg <= self.stable_rotation_tolerance_deg:
                    stable_since = time.monotonic() if stable_since is None else stable_since
                    if time.monotonic() - stable_since >= self.stable_window_s:
                        return pose
                else:
                    stable_since = None
            previous_pose = pose
            time.sleep(0.05)
        raise RuntimeError(f"TCP did not become stable within {self.stable_timeout_s:.3f}s. Last pose: {last_pose!r}")

    def _run_semi_auto_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._semi_auto_lock:
            if self._semi_auto_active:
                response.success = False
                response.message = "Semi-auto calibration is already running."
                self._publish_status("semi_auto_failed", response.message, {"session_dir": str(self.session_dir) if self.session_dir else None})
                return response
            self._semi_auto_active = True
        created_session = False
        try:
            if not self.trajectory_path.exists():
                response.success = False
                response.message = f"Trajectory YAML does not exist: {self.trajectory_path}. Record waypoints first."
                self._publish_status("semi_auto_failed", response.message, {"session_dir": None})
                return response
            trajectory = load_trajectory(self.trajectory_path)
            self._ensure_session_started(owner="semi_auto")
            created_session = True
            self._archive_current_trajectory()
            self._append_run_log("semi_auto_started", {"trajectory_path": str(self.trajectory_path), "execute_motion": self.execute_motion, "waypoint_count": len(trajectory.waypoints)})
            if not self.execute_motion:
                for waypoint in trajectory.waypoints:
                    self._publish_status("semi_auto_dry_run_waypoint", f"Dry-run waypoint {waypoint.name}.", {"waypoint": waypoint.to_payload(), "session_dir": str(self.session_dir)})
                response.success = True
                response.message = f"Dry-run complete for {len(trajectory.waypoints)} waypoints. No motion, capture, solve, or save was executed."
                self._append_run_log("semi_auto_dry_run_complete", {"waypoint_count": len(trajectory.waypoints), "session_dir": str(self.session_dir)})
                self._publish_status("semi_auto_dry_run_complete", response.message, {"session_dir": str(self.session_dir), "trajectory_path": str(self.trajectory_path)})
                return response

            captured_count = 0
            for index, waypoint in enumerate(trajectory.waypoints, start=1):
                if waypoint.motion != "movej":
                    raise RuntimeError(f"Unsupported waypoint motion: {waypoint.motion}")
                move_error = self.linux_client.move_j(
                    waypoint.joint_deg,
                    tool_id=trajectory.tool_id,
                    user_id=trajectory.user_id,
                    vel=waypoint.vel,
                )
                if move_error != 0:
                    raise RuntimeError(f"MoveJ failed at {waypoint.name} with code {move_error}.")
                stable_pose = self._wait_until_tcp_stable()
                time.sleep(max(0.0, waypoint.dwell_s))
                self._append_run_log("waypoint_reached", {"waypoint": waypoint.to_payload(), "stable_tcp_pose_mmdeg": stable_pose, "waypoint_index": index, "session_dir": str(self.session_dir)})
                self._publish_status("semi_auto_waypoint_reached", f"Reached {waypoint.name}.", {"waypoint": waypoint.to_payload(), "session_dir": str(self.session_dir)})
                if waypoint.capture:
                    sample = self._capture_one_sample(owner="semi_auto")
                    captured_count += 1
                    self._publish_status(
                        "sample_captured",
                        f"Captured sample #{len(self.samples)}.",
                        {
                            "sample_quality": sample.sample_quality,
                            "capture_timing": self._sample_to_log_record(sample),
                            "sample_log_path": str(self.sample_log_path) if self.sample_log_path else None,
                            "session_dir": str(self.session_dir),
                        },
                    )

            solve_response = self._solve_callback(Trigger.Request(), Trigger.Response())
            if not solve_response.success:
                raise RuntimeError(solve_response.message)
            self._save_current_solution()
            response.success = True
            response.message = f"Semi-auto calibration finished with {captured_count} captured samples."
            self._append_run_log("semi_auto_finished", {"captured_count": captured_count, "session_dir": str(self.session_dir), "report_path": str(self.report_path) if self.report_path else None})
            self._publish_status("semi_auto_finished", response.message, {"session_dir": str(self.session_dir), "report_path": str(self.report_path) if self.report_path else None})
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            if created_session:
                self._append_run_log("semi_auto_failed", {"error": response.message, "session_dir": str(self.session_dir)})
            self._publish_status("semi_auto_failed", response.message, {"session_dir": str(self.session_dir) if created_session else None})
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
