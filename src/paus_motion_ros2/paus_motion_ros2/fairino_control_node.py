from __future__ import annotations

# 导入 json，用于把控制状态整理成结构化字符串。
import json

# 导入 NumPy，用于计算目标去抖距离。
import numpy as np
# 导入 ROS2 Python API。
import rclpy
# 导入 ament 索引，用于定位 bringup 包中的配置文件。
from ament_index_python.packages import get_package_share_directory
# 导入目标点消息类型。
from geometry_msgs.msg import PointStamped
# 导入外部关闭异常，便于优雅退出。
from rclpy.executors import ExternalShutdownException
# 导入节点基类。
from rclpy.node import Node
# 导入字符串消息，用于发布 JSON 状态。
from std_msgs.msg import String

# 导入运动控制包提供的执行适配层和接近决策函数。
from paus_motion_ros2 import FairinoLinuxClient, build_approach_decision
# 导入项目配置读取函数。
from paus_perception import load_config


# 这个节点负责把 `/target_point_base` 转换成一次安全的接近执行动作。
# 它既承担 dry-run 状态发布，也承担真实 `MoveL` 的执行入口。
class FairinoControlNode(Node):
    # 初始化节点。
    def __init__(self) -> None:
        # 注册节点名字。
        super().__init__("fairino_control_node")

        # 找到 bringup 包的 share 目录，后续默认配置都从这里取。
        bringup_share = get_package_share_directory("paus_bringup")
        # 声明核心参数。
        self.declare_parameter("config_path", f"{bringup_share}/configs/default.yaml")
        self.declare_parameter("approach_target_topic", "/target_point_base")
        self.declare_parameter("control_status_topic", "/control_status")

        # 读取参数值。
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        target_topic = self.get_parameter("approach_target_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("control_status_topic").get_parameter_value().string_value

        # 加载主配置，并解析控制段。
        self.config = load_config(config_path)
        control_cfg = self.config["control"]

        # 读取控制和执行相关配置。
        self.robot_ip = str(control_cfg["robot_ip"])
        self.linux_fairino_sdk_root = str(control_cfg["linux_fairino_sdk_root"])
        self.execute_motion = bool(control_cfg["execute_motion"])
        self.tool_id = int(control_cfg["tool_id"])
        self.user_id = int(control_cfg["user_id"])
        self.move_vel = float(control_cfg["move_vel"])
        self.max_step_distance_mm = float(control_cfg["max_step_distance_mm"])
        self.final_standoff_mm = float(control_cfg["final_standoff_mm"])
        self.min_safe_z_mm = float(control_cfg["min_safe_z_mm"])
        self.workspace_min_mm = [float(value) for value in control_cfg["workspace_min_mm"]]
        self.workspace_max_mm = [float(value) for value in control_cfg["workspace_max_mm"]]
        self.repeat_distance_threshold_mm = float(control_cfg["repeat_distance_threshold_mm"])
        self.use_mock_pose = bool(control_cfg["use_mock_pose"])
        self.mock_current_tcp_pose_mmdeg = [float(value) for value in control_cfg["mock_current_tcp_pose_mmdeg"]]
        # 允许 launch 在运行时覆盖 `execute_motion`，从而实现默认 dry-run、显式 real-run。
        self.declare_parameter("execute_motion", bool(control_cfg["execute_motion"]))
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)

        # 创建状态发布器和目标点订阅器。
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.subscription = self.create_subscription(PointStamped, target_topic, self._target_callback, 10)

        # 初始化执行状态。
        self.last_executed_target_mm: list[float] | None = None
        self.motion_in_progress = False
        self.control_backend = "mock_pose"
        self.linux_client: FairinoLinuxClient | None = None

        # 如果当前不是 mock 模式，就尝试连接真实 Linux SDK。
        if not self.use_mock_pose:
            try:
                self.linux_client = FairinoLinuxClient(self.linux_fairino_sdk_root, self.robot_ip)
                self.linux_client.connect()
                self.control_backend = "linux_sdk"
            except Exception as exc:
                # 如果连接失败，则退回 mock 模式，但继续允许节点启动。
                self.control_backend = "mock_pose"
                self.get_logger().warning(
                    json.dumps(
                        {
                            "event": "linux_sdk_connect_failed",
                            "robot_ip": self.robot_ip,
                            "sdk_root": self.linux_fairino_sdk_root,
                            "error": repr(exc),
                        },
                        ensure_ascii=False,
                    )
                )

        # 节点启动后立即发布一次初始状态。
        self._publish_status(
            event="fairino_control_node_started",
            error_message="",
            check_passed=False,
            target_point_base_m=None,
            target_point_base_mm=None,
            current_tcp_pose_mmdeg=None,
            candidate_pose_mmdeg=None,
            executed=False,
        )

    # 统一发布 `/control_status`。
    def _publish_status(
        self,
        event: str,
        error_message: str,
        check_passed: bool,
        target_point_base_m: list[float] | None,
        target_point_base_mm: list[float] | None,
        current_tcp_pose_mmdeg: list[float] | None,
        candidate_pose_mmdeg: list[float] | None,
        executed: bool,
    ) -> None:
        # 将当前事件整理成结构化 JSON。
        payload = {
            "event": event,
            "error_message": error_message,
            "check_passed": check_passed,
            "execute_motion": self.execute_motion,
            "executed": executed,
            "target_point_base_m": target_point_base_m,
            "target_point_base_mm": target_point_base_mm,
            "current_tcp_pose_mmdeg": current_tcp_pose_mmdeg,
            "candidate_pose_mmdeg": candidate_pose_mmdeg,
            "control_backend": self.control_backend,
        }
        # 打包为字符串消息并发布。
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)
        # 同时打印到节点日志，方便终端联调。
        self.get_logger().info(message.data)

    # 读取当前 TCP 位姿。
    # 如果当前是 mock 模式，则直接返回配置里给的假位姿。
    def _get_current_tcp_pose_mmdeg(self) -> list[float]:
        if self.control_backend == "mock_pose" or self.linux_client is None:
            return list(self.mock_current_tcp_pose_mmdeg)
        # 否则通过 Linux SDK 读取真实 TCP。
        error, pose = self.linux_client.get_actual_tcp_pose()
        if error != 0:
            raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
        return pose

    # 执行一次笛卡尔直线运动。
    def _execute_move(self, candidate_pose_mmdeg: list[float]) -> None:
        if self.control_backend != "linux_sdk" or self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        result = self.linux_client.move_l(candidate_pose_mmdeg, tool_id=self.tool_id, user_id=self.user_id, vel=self.move_vel)
        if result != 0:
            raise RuntimeError(f"MoveL failed with code {result}.")

    # 目标点回调：每收到一个新的粗定位目标，就尝试生成并执行接近动作。
    def _target_callback(self, message: PointStamped) -> None:
        # 如果上一次动作还没结束，先拒绝这次请求。
        if self.motion_in_progress:
            self._publish_status(
                event="motion_busy",
                error_message="Motion is already in progress.",
                check_passed=False,
                target_point_base_m=[message.point.x, message.point.y, message.point.z],
                target_point_base_mm=None,
                current_tcp_pose_mmdeg=None,
                candidate_pose_mmdeg=None,
                executed=False,
            )
            return

        try:
            # 先读取当前 TCP 位姿。
            current_tcp_pose_mmdeg = self._get_current_tcp_pose_mmdeg()
            # 基于当前位姿和目标点，生成一次接近决策。
            decision = build_approach_decision(
                target_point_base_m=[message.point.x, message.point.y, message.point.z],
                frame_id=message.header.frame_id,
                current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
                final_standoff_mm=self.final_standoff_mm,
                max_step_distance_mm=self.max_step_distance_mm,
                min_safe_z_mm=self.min_safe_z_mm,
                workspace_min_mm=self.workspace_min_mm,
                workspace_max_mm=self.workspace_max_mm,
            )
        except Exception as exc:
            # 如果读状态或生成决策失败，则直接发布控制错误。
            self._publish_status(
                event="control_error",
                error_message=repr(exc),
                check_passed=False,
                target_point_base_m=[message.point.x, message.point.y, message.point.z],
                target_point_base_mm=None,
                current_tcp_pose_mmdeg=None,
                candidate_pose_mmdeg=None,
                executed=False,
            )
            return

        # 如果安全检查未通过，则发布拒绝原因。
        if not decision.check_passed:
            self._publish_status(
                event="control_rejected",
                error_message=decision.error_message,
                check_passed=False,
                target_point_base_m=[message.point.x, message.point.y, message.point.z],
                target_point_base_mm=decision.target_point_base_mm,
                current_tcp_pose_mmdeg=decision.current_tcp_pose_mmdeg,
                candidate_pose_mmdeg=decision.candidate_pose_mmdeg,
                executed=False,
            )
            return

        # 对已经执行过的目标做去抖，避免目标点轻微抖动导致重复执行。
        if self.last_executed_target_mm is not None:
            delta = np.linalg.norm(np.asarray(decision.target_point_base_mm) - np.asarray(self.last_executed_target_mm))
            if delta < self.repeat_distance_threshold_mm:
                self._publish_status(
                    event="control_skipped_repeat",
                    error_message="Target change is smaller than repeat threshold.",
                    check_passed=True,
                    target_point_base_m=[message.point.x, message.point.y, message.point.z],
                    target_point_base_mm=decision.target_point_base_mm,
                    current_tcp_pose_mmdeg=decision.current_tcp_pose_mmdeg,
                    candidate_pose_mmdeg=decision.candidate_pose_mmdeg,
                    executed=False,
                )
                return

        # 如果当前处于 dry-run，就只发布候选位姿，不真正执行。
        if not self.execute_motion:
            self._publish_status(
                event="control_dry_run",
                error_message="",
                check_passed=True,
                target_point_base_m=[message.point.x, message.point.y, message.point.z],
                target_point_base_mm=decision.target_point_base_mm,
                current_tcp_pose_mmdeg=decision.current_tcp_pose_mmdeg,
                candidate_pose_mmdeg=decision.candidate_pose_mmdeg,
                executed=False,
            )
            return

        try:
            # 标记当前动作开始执行。
            self.motion_in_progress = True
            assert decision.candidate_pose_mmdeg is not None
            # 调用 Linux SDK 执行运动。
            self._execute_move(decision.candidate_pose_mmdeg)
            # 记录本次已执行目标，用于后续去抖。
            self.last_executed_target_mm = list(decision.target_point_base_mm)
            # 发布执行成功状态。
            self._publish_status(
                event="control_executed",
                error_message="",
                check_passed=True,
                target_point_base_m=[message.point.x, message.point.y, message.point.z],
                target_point_base_mm=decision.target_point_base_mm,
                current_tcp_pose_mmdeg=decision.current_tcp_pose_mmdeg,
                candidate_pose_mmdeg=decision.candidate_pose_mmdeg,
                executed=True,
            )
        except Exception as exc:
            # 如果执行阶段出错，则发布失败状态。
            self._publish_status(
                event="control_execute_failed",
                error_message=repr(exc),
                check_passed=True,
                target_point_base_m=[message.point.x, message.point.y, message.point.z],
                target_point_base_mm=decision.target_point_base_mm,
                current_tcp_pose_mmdeg=decision.current_tcp_pose_mmdeg,
                candidate_pose_mmdeg=decision.candidate_pose_mmdeg,
                executed=False,
            )
        finally:
            # 无论成功还是失败，都要把“动作执行中”标志清掉。
            self.motion_in_progress = False


# ROS2 节点入口。
def main(args: list[str] | None = None) -> None:
    # 初始化 ROS2。
    rclpy.init(args=args)
    # 创建节点实例。
    node = FairinoControlNode()
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
