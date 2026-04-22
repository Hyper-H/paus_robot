from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from paus_perception import (
    EyeToHandCalibrationSolution,
    average_transform_matrices,
    invert_transform_matrix,
    load_camera_calibration,
    make_transform_matrix,
    make_transform_struct,
    quaternion_xyzw_to_rotation_matrix,
    rpy_deg_to_rotation_matrix,
    save_eye_to_hand_solution,
)


@dataclass
class CalibrationSample:
    base_to_camera_matrix: np.ndarray


class EyeToHandCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("eye_to_hand_calibration_node")
        bringup_share = Path(get_package_share_directory("paus_bringup"))

        self.declare_parameter("camera_config_path", "/tmp/paus_robot/camera.yaml")
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("tool_pose_topic", "/nonrt_state_data")
        self.declare_parameter("status_topic", "/eye_to_hand/status")
        self.declare_parameter("board_rows", 6)
        self.declare_parameter("board_cols", 9)
        self.declare_parameter("square_size_m", 0.01)
        self.declare_parameter("tool_to_board.translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("tool_to_board.rotation_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("min_sample_count", 10)
        self.declare_parameter("output_path", str(bringup_share / "configs" / "extrinsics.yaml"))

        camera_config_path = self.get_parameter("camera_config_path").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.tool_pose_topic = self.get_parameter("tool_pose_topic").get_parameter_value().string_value
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.board_rows = int(self.get_parameter("board_rows").get_parameter_value().integer_value)
        self.board_cols = int(self.get_parameter("board_cols").get_parameter_value().integer_value)
        self.square_size_m = float(self.get_parameter("square_size_m").get_parameter_value().double_value)
        self.tool_to_board_translation = [float(value) for value in self.get_parameter("tool_to_board.translation_m").get_parameter_value().double_array_value]
        self.tool_to_board_rotation_rpy = [float(value) for value in self.get_parameter("tool_to_board.rotation_rpy_deg").get_parameter_value().double_array_value]
        self.min_sample_count = int(self.get_parameter("min_sample_count").get_parameter_value().integer_value)
        self.output_path = Path(self.get_parameter("output_path").get_parameter_value().string_value)

        if not camera_config_path:
            raise RuntimeError("camera_config_path is required for eye-to-hand calibration.")
        self.camera_calibration = load_camera_calibration(camera_config_path)

        self.bridge = CvBridge()
        self.latest_image_bgr: np.ndarray | None = None
        self.latest_tool_pose: PoseStamped | None = None
        self.samples: list[CalibrationSample] = []
        self.current_solution: EyeToHandCalibrationSolution | None = None

        self.image_subscription = self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.tool_pose_subscription = self.create_subscription(PoseStamped, self.tool_pose_topic, self._tool_pose_callback, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)

        self.capture_service = self.create_service(Trigger, "/eye_to_hand/capture_sample", self._capture_sample_callback)
        self.solve_service = self.create_service(Trigger, "/eye_to_hand/solve", self._solve_callback)
        self.save_service = self.create_service(Trigger, "/eye_to_hand/save", self._save_callback)

        self._publish_status("ready", "Eye-to-hand calibration node started.")

    def _image_callback(self, message: Image) -> None:
        self.latest_image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")

    def _tool_pose_callback(self, message: PoseStamped) -> None:
        self.latest_tool_pose = message

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

    def _build_board_object_points(self) -> np.ndarray:
        object_points = np.zeros((self.board_rows * self.board_cols, 3), np.float32)
        object_points[:, :2] = np.mgrid[0:self.board_cols, 0:self.board_rows].T.reshape(-1, 2)
        object_points *= self.square_size_m
        return object_points

    def _estimate_camera_to_board(self) -> np.ndarray:
        if self.latest_image_bgr is None:
            raise RuntimeError("No image has been received yet.")

        gray = cv2.cvtColor(self.latest_image_bgr, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (self.board_cols, self.board_rows))
        if not found:
            raise RuntimeError("Chessboard was not detected in the latest image.")

        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        object_points = self._build_board_object_points()
        camera_matrix = np.asarray(self.camera_calibration.camera_matrix, dtype=np.float64)
        dist_coeffs = np.asarray(self.camera_calibration.dist_coeffs, dtype=np.float64)
        success, rvec, tvec = cv2.solvePnP(object_points, refined, camera_matrix, dist_coeffs)
        if not success:
            raise RuntimeError("solvePnP failed for the calibration board.")
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return make_transform_matrix(tvec.reshape(3), rotation_matrix)

    def _latest_base_to_tool(self) -> np.ndarray:
        if self.latest_tool_pose is None:
            raise RuntimeError("No tool pose has been received yet.")
        pose = self.latest_tool_pose.pose
        translation = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrix = quaternion_xyzw_to_rotation_matrix(quaternion)
        return make_transform_matrix(translation, rotation_matrix)

    def _tool_to_board_matrix(self) -> np.ndarray:
        rotation_matrix = rpy_deg_to_rotation_matrix(self.tool_to_board_rotation_rpy)
        return make_transform_matrix(self.tool_to_board_translation, rotation_matrix)

    def _capture_sample_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            base_to_tool = self._latest_base_to_tool()
            camera_to_board = self._estimate_camera_to_board()
            tool_to_board = self._tool_to_board_matrix()
            base_to_camera = base_to_tool @ tool_to_board @ invert_transform_matrix(camera_to_board)
            self.samples.append(CalibrationSample(base_to_camera_matrix=base_to_camera))
            response.success = True
            response.message = f"Captured sample #{len(self.samples)}."
            self._publish_status("sample_captured", response.message)
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("capture_failed", response.message)
        return response

    def _solve_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if len(self.samples) < self.min_sample_count:
            response.success = False
            response.message = f"Need at least {self.min_sample_count} samples, currently {len(self.samples)}."
            self._publish_status("solve_failed", response.message)
            return response
        try:
            matrices = [sample.base_to_camera_matrix for sample in self.samples]
            averaged = average_transform_matrices(matrices)
            translation = averaged[:3, 3]
            rotation = averaged[:3, :3]
            transform = make_transform_struct(translation, rotation, "robot_base", "camera")
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

    def _save_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self.current_solution is None or not self.current_solution.success:
            response.success = False
            response.message = "No solved extrinsic is available."
            self._publish_status("save_failed", response.message)
            return response
        try:
            save_eye_to_hand_solution(self.current_solution, self.output_path)
            response.success = True
            response.message = f"Saved extrinsic to {self.output_path}."
            self._publish_status("saved", response.message, {"output_path": str(self.output_path)})
        except Exception as exc:
            response.success = False
            response.message = repr(exc)
            self._publish_status("save_failed", response.message)
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EyeToHandCalibrationNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
