from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from paus_perception import load_camera_calibration, load_config, process_image_array, render_visualization

from .conversions import build_status_payload, rvec_to_quaternion


class MarkerPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("marker_pose_node")

        bringup_share = Path(get_package_share_directory("paus_bringup"))

        self.declare_parameter("config_path", str(bringup_share / "configs" / "default.yaml"))
        self.declare_parameter("camera_config_path", "/tmp/paus_robot/camera.yaml")
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("marker_pose_topic", "/marker_pose")
        self.declare_parameter("approach_target_topic", "/approach_target")
        self.declare_parameter("status_topic", "/detection_status")
        self.declare_parameter("debug_image_topic", "/marker_debug_image")
        self.declare_parameter("publish_debug_image", False)

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        camera_config_path = self.get_parameter("camera_config_path").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        marker_pose_topic = self.get_parameter("marker_pose_topic").get_parameter_value().string_value
        approach_target_topic = self.get_parameter("approach_target_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        debug_image_topic = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        self.publish_debug_image = self.get_parameter("publish_debug_image").get_parameter_value().bool_value

        self.config = load_config(config_path)
        pose_cfg = self.config["pose"]
        self.min_forward_distance_m = float(pose_cfg.get("min_forward_distance_m", 0.02))
        self.max_pose_distance_m = float(pose_cfg.get("max_pose_distance_m", 1.5))
        self.max_pose_jump_m = float(pose_cfg.get("max_pose_jump_m", 0.25))
        self.last_valid_pose_tvec: np.ndarray | None = None

        self.camera_config_path = Path(camera_config_path) if camera_config_path else None
        self._camera_config_warning_emitted = False
        self.camera_calibration = self._try_load_camera_calibration()
        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.marker_pose_publisher = self.create_publisher(PoseStamped, marker_pose_topic, 10)
        self.approach_target_publisher = self.create_publisher(PointStamped, approach_target_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.debug_image_publisher = self.create_publisher(Image, debug_image_topic, 10) if self.publish_debug_image else None

        self.get_logger().info(
            json.dumps(
                {
                    "event": "node_started",
                    "image_topic": image_topic,
                    "marker_pose_topic": marker_pose_topic,
                    "approach_target_topic": approach_target_topic,
                    "status_topic": status_topic,
                    "camera_config_path": str(self.camera_config_path) if self.camera_config_path else "",
                    "pose_filter": {
                        "min_forward_distance_m": self.min_forward_distance_m,
                        "max_pose_distance_m": self.max_pose_distance_m,
                        "max_pose_jump_m": self.max_pose_jump_m,
                    },
                },
                ensure_ascii=False,
            )
        )

    def _try_load_camera_calibration(self):
        if self.camera_config_path is None:
            return None
        if not self.camera_config_path.exists():
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

        calibration = load_camera_calibration(self.camera_config_path)
        self._camera_config_warning_emitted = False
        return calibration

    def _validate_pose_result(self, result) -> tuple[bool, str]:
        pose = result.pose
        if pose is None or pose.status != "ok":
            return False, "pose_unavailable"

        translation = np.asarray(pose.tvec, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(translation)):
            return False, "pose_non_finite"
        if float(translation[2]) < self.min_forward_distance_m:
            return False, "pose_behind_or_too_close_to_camera"
        if float(pose.distance_m) > self.max_pose_distance_m:
            return False, "pose_distance_above_max"
        if self.last_valid_pose_tvec is not None:
            pose_jump_m = float(np.linalg.norm(translation - self.last_valid_pose_tvec))
            if pose_jump_m > self.max_pose_jump_m:
                return False, f"pose_jump_above_max:{pose_jump_m:.3f}m"
        return True, ""

    def _reject_pose_result(self, result, reason: str) -> None:
        self.get_logger().warning(
            json.dumps(
                {
                    "event": "marker_pose_rejected",
                    "reason": reason,
                    "marker_id": result.marker.marker_id if result.marker is not None else None,
                    "selection_reason": result.marker.selection_reason if result.marker is not None else None,
                    "tvec": result.pose.tvec if result.pose is not None else None,
                    "distance_m": result.pose.distance_m if result.pose is not None else None,
                },
                ensure_ascii=False,
            )
        )
        result.status = "pose_filtered"
        result.message = f"Marker pose rejected: {reason}"
        result.pose = None
        result.approach_plan = None

    def _image_callback(self, message: Image) -> None:
        image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if self.camera_calibration is None:
            self.camera_calibration = self._try_load_camera_calibration()

        result = process_image_array(image_bgr, self.config, camera_calibration=self.camera_calibration)
        pose_valid, pose_reject_reason = self._validate_pose_result(result)
        if pose_valid and result.pose is not None:
            self.last_valid_pose_tvec = np.asarray(result.pose.tvec, dtype=np.float64).reshape(3)
        elif result.pose is not None and result.pose.status == "ok":
            self._reject_pose_result(result, pose_reject_reason)

        status_message = String()
        status_message.data = build_status_payload(result)
        self.status_publisher.publish(status_message)

        if pose_valid and result.pose is not None and result.pose.status == "ok":
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

        if pose_valid and result.approach_plan is not None and result.approach_plan.target_status == "ok":
            point_message = PointStamped()
            point_message.header = message.header
            point_message.header.frame_id = result.approach_plan.target_frame
            point_message.point.x = result.approach_plan.target_point[0]
            point_message.point.y = result.approach_plan.target_point[1]
            point_message.point.z = result.approach_plan.target_point[2]
            self.approach_target_publisher.publish(point_message)

        if self.debug_image_publisher is not None:
            debug_image = render_visualization(image_bgr, result)
            debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_message.header = message.header
            self.debug_image_publisher.publish(debug_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MarkerPoseNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
