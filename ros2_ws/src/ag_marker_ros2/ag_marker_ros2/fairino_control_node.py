from __future__ import annotations

import json

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from ag_repro import FairinoLinuxClient, build_approach_decision, load_config


class FairinoControlNode(Node):
    def __init__(self) -> None:
        super().__init__("fairino_control_node")

        package_share = get_package_share_directory("ag_marker_ros2")
        self.declare_parameter("config_path", f"{package_share}/configs/default.yaml")
        self.declare_parameter("approach_target_topic", "/target_point_base")
        self.declare_parameter("control_status_topic", "/control_status")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        target_topic = self.get_parameter("approach_target_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("control_status_topic").get_parameter_value().string_value

        self.config = load_config(config_path)
        control_cfg = self.config["control"]

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
        self.declare_parameter("execute_motion", bool(control_cfg["execute_motion"]))
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)

        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.subscription = self.create_subscription(PointStamped, target_topic, self._target_callback, 10)

        self.last_executed_target_mm: list[float] | None = None
        self.motion_in_progress = False
        self.control_backend = "mock_pose"
        self.linux_client: FairinoLinuxClient | None = None

        if not self.use_mock_pose:
            try:
                self.linux_client = FairinoLinuxClient(self.linux_fairino_sdk_root, self.robot_ip)
                self.linux_client.connect()
                self.control_backend = "linux_sdk"
            except Exception as exc:
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
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)
        self.get_logger().info(message.data)

    def _get_current_tcp_pose_mmdeg(self) -> list[float]:
        if self.control_backend == "mock_pose" or self.linux_client is None:
            return list(self.mock_current_tcp_pose_mmdeg)
        error, pose = self.linux_client.get_actual_tcp_pose()
        if error != 0:
            raise RuntimeError(f"GetActualTCPPose failed with code {error}.")
        return pose

    def _execute_move(self, candidate_pose_mmdeg: list[float]) -> None:
        if self.control_backend != "linux_sdk" or self.linux_client is None:
            raise RuntimeError("Linux FAIRINO SDK backend is unavailable.")
        result = self.linux_client.move_l(candidate_pose_mmdeg, tool_id=self.tool_id, user_id=self.user_id, vel=self.move_vel)
        if result != 0:
            raise RuntimeError(f"MoveL failed with code {result}.")

    def _target_callback(self, message: PointStamped) -> None:
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
            current_tcp_pose_mmdeg = self._get_current_tcp_pose_mmdeg()
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
            self.motion_in_progress = True
            assert decision.candidate_pose_mmdeg is not None
            self._execute_move(decision.candidate_pose_mmdeg)
            self.last_executed_target_mm = list(decision.target_point_base_mm)
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
            self.motion_in_progress = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FairinoControlNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
