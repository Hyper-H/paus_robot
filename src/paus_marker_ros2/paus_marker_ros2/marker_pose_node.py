from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from paus_perception import CameraCalibration, load_config, process_image_array, render_visualization

from .conversions import build_status_payload, rvec_to_quaternion


def camera_info_to_calibration(message: CameraInfo) -> CameraCalibration:
    k_values = [float(value) for value in message.k]
    if len(k_values) != 9:
        raise ValueError(f"CameraInfo.k must contain 9 values, got {len(k_values)}.")
    if int(message.width) <= 0 or int(message.height) <= 0:
        raise ValueError(f"CameraInfo dimensions must be positive, got {message.width}x{message.height}.")
    camera_matrix = [
        k_values[0:3],
        k_values[3:6],
        k_values[6:9],
    ]
    return CameraCalibration(
        image_width=int(message.width),
        image_height=int(message.height),
        camera_matrix=camera_matrix,
        dist_coeffs=[float(value) for value in message.d],
        reprojection_error=0.0,
        board_rows=0,
        board_cols=0,
        square_size_m=0.0,
        source_type="ros_camera_info",
        source_path="/camera/camera_info",
    )


def stamp_key(message: Image | CameraInfo) -> tuple[int, int]:
    return (int(message.header.stamp.sec), int(message.header.stamp.nanosec))


def camera_info_signature(message: CameraInfo) -> tuple[object, ...]:
    k_values = [float(value) for value in message.k]
    if len(k_values) != 9:
        raise ValueError(f"CameraInfo.k must contain 9 values, got {len(k_values)}.")
    if int(message.width) <= 0 or int(message.height) <= 0:
        raise ValueError(f"CameraInfo dimensions must be positive, got {message.width}x{message.height}.")
    return (
        int(message.width),
        int(message.height),
        k_values[0],
        k_values[4],
        k_values[2],
        k_values[5],
        str(message.distortion_model),
    )


class MarkerPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("marker_pose_node")
        bringup_share = Path(get_package_share_directory("paus_bringup"))

        self.declare_parameter("config_path", str(bringup_share / "configs" / "default.yaml"))
        self.declare_parameter("image_topic", "/camera/image_bridge")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("marker_pose_topic", "/marker_pose")
        self.declare_parameter("approach_target_topic", "/approach_target")
        self.declare_parameter("status_topic", "/detection_status")
        self.declare_parameter("debug_image_topic", "/marker_debug_image")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("pair_queue_size", 5)

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        marker_pose_topic = self.get_parameter("marker_pose_topic").get_parameter_value().string_value
        approach_target_topic = self.get_parameter("approach_target_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        debug_image_topic = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        self.publish_debug_image = self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        self.pair_queue_size = max(1, int(self.get_parameter("pair_queue_size").get_parameter_value().integer_value))

        self.config = load_config(config_path)
        self.bridge = CvBridge()
        self.pending_images: dict[tuple[int, int], Image] = {}
        self.pending_camera_infos: dict[tuple[int, int], CameraInfo] = {}
        self._camera_info_signature: tuple[object, ...] | None = None
        self._waiting_for_camera_info_logged = False
        self._has_accepted_camera_info = False

        self.image_subscription = self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.camera_info_subscription = self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, 10)
        self.marker_pose_publisher = self.create_publisher(PoseStamped, marker_pose_topic, 10)
        self.approach_target_publisher = self.create_publisher(PointStamped, approach_target_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.debug_image_publisher = self.create_publisher(Image, debug_image_topic, 10) if self.publish_debug_image else None

        self.get_logger().info(
            json.dumps(
                {
                    "event": "node_started",
                    "image_topic": image_topic,
                    "camera_info_topic": camera_info_topic,
                    "marker_pose_topic": marker_pose_topic,
                    "approach_target_topic": approach_target_topic,
                    "status_topic": status_topic,
                    "pair_queue_size": self.pair_queue_size,
                },
                ensure_ascii=False,
            )
        )

    def _image_callback(self, message: Image) -> None:
        key = stamp_key(message)
        camera_info = self.pending_camera_infos.pop(key, None)
        if camera_info is None:
            self.pending_images[key] = message
            self._prune_pending(self.pending_images, "image")
            if not self._has_accepted_camera_info and not self._waiting_for_camera_info_logged:
                self.get_logger().warning(
                    json.dumps(
                        {
                            "event": "waiting_for_camera_info",
                            "image_stamp_sec": key[0],
                            "image_stamp_nanosec": key[1],
                        },
                        ensure_ascii=False,
                    )
                )
                self._waiting_for_camera_info_logged = True
            return
        self._process_pair(message, camera_info)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        try:
            self._log_camera_info_accepted_once_or_changed(message)
        except ValueError as exc:
            self.get_logger().warning(json.dumps({"event": "camera_info_invalid", "reason": str(exc)}, ensure_ascii=False))
            return
        key = stamp_key(message)
        image = self.pending_images.pop(key, None)
        if image is None:
            self.pending_camera_infos[key] = message
            self._prune_pending(self.pending_camera_infos, "camera_info")
            return
        self._process_pair(image, message)

    def _prune_pending(self, pending: dict[tuple[int, int], Image | CameraInfo], frame_type: str) -> None:
        while len(pending) > self.pair_queue_size:
            oldest_key = sorted(pending.keys())[0]
            pending.pop(oldest_key, None)
            self.get_logger().warning(
                json.dumps(
                    {
                        "event": "unpaired_frame_dropped",
                        "frame_type": frame_type,
                        "stamp_sec": oldest_key[0],
                        "stamp_nanosec": oldest_key[1],
                    },
                    ensure_ascii=False,
                )
            )

    def _log_camera_info_accepted_once_or_changed(self, message: CameraInfo) -> None:
        signature = camera_info_signature(message)
        self._has_accepted_camera_info = True
        if signature == self._camera_info_signature:
            return
        self._camera_info_signature = signature
        self.get_logger().info(
            json.dumps(
                {
                    "event": "camera_info_accepted",
                    "width": int(message.width),
                    "height": int(message.height),
                    "fx": float(message.k[0]),
                    "fy": float(message.k[4]),
                    "cx": float(message.k[2]),
                    "cy": float(message.k[5]),
                    "distortion_model": str(message.distortion_model),
                },
                ensure_ascii=False,
            )
        )

    def _process_pair(self, image_message: Image, camera_info_message: CameraInfo) -> None:
        self._waiting_for_camera_info_logged = False
        image_bgr = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        calibration = camera_info_to_calibration(camera_info_message)
        result = process_image_array(image_bgr, self.config, camera_calibration=calibration)

        status_message = String()
        status_message.data = build_status_payload(result)
        self.status_publisher.publish(status_message)

        if result.pose is not None and result.pose.status == "ok":
            pose_message = PoseStamped()
            pose_message.header = image_message.header
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

        if result.approach_plan is not None and result.approach_plan.target_status == "ok":
            point_message = PointStamped()
            point_message.header = image_message.header
            point_message.header.frame_id = result.approach_plan.target_frame
            point_message.point.x = result.approach_plan.target_point[0]
            point_message.point.y = result.approach_plan.target_point[1]
            point_message.point.z = result.approach_plan.target_point[2]
            self.approach_target_publisher.publish(point_message)

        if self.debug_image_publisher is not None:
            debug_image = render_visualization(image_bgr, result)
            debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_message.header = image_message.header
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
