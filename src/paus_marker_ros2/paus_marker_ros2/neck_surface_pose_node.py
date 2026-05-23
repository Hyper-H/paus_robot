from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from paus_perception import (
    LATCHED_POSE,
    MediaPipePoseBackend,
    NeckSurfaceConfig,
    NeckSurfaceEstimate,
    draw_debug_overlay,
    estimate_neck_surface_pose,
    load_config,
    load_eye_to_hand_solution,
    make_transform_matrix,
    make_transform_struct,
    rotation_matrix_to_quaternion_xyzw,
    split_transform_matrix,
    write_logging_artifacts,
)

from .marker_pose_node import camera_info_to_calibration, camera_info_signature


def _normalize_depth_image(depth_image: np.ndarray) -> np.ndarray:
    if depth_image.ndim == 3 and depth_image.shape[2] == 1:
        return depth_image[:, :, 0]
    return depth_image


class NeckSurfacePoseNode(Node):
    def __init__(self) -> None:
        super().__init__("neck_surface_pose_node")
        bringup_share = Path(get_package_share_directory("paus_bringup"))

        self.declare_parameter("config_path", str(bringup_share / "configs" / "default.yaml"))
        self.declare_parameter("extrinsics_path", str(bringup_share / "configs" / "extrinsics.yaml"))
        self.declare_parameter("image_topic", "")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("neck_surface_pose_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("approach_target_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("debug_image_topic", "")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("pair_queue_size", 5)
        self.declare_parameter("max_rgb_depth_delta_ms", 200.0)

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        extrinsics_path = self.get_parameter("extrinsics_path").get_parameter_value().string_value
        self.config = load_config(config_path)
        neck_cfg = self.config.get("neck_surface", {})
        self.estimator_config = NeckSurfaceConfig.from_mapping(neck_cfg)
        self.latched_pose_timeout_s = float(neck_cfg.get("latched_pose_timeout_s", 0.5))
        self.logging_enabled = bool(neck_cfg.get("logging_enabled", False))
        self.logging_output_dir = str(neck_cfg.get("logging_output_dir", "runtime_logs/neck_surface"))

        image_topic = self._string_param_or_config("image_topic", neck_cfg, "/camera/color/image_raw")
        depth_topic = self._string_param_or_config("depth_topic", neck_cfg, "/camera/aligned_depth_to_color/image_raw")
        camera_info_topic = self._string_param_or_config("camera_info_topic", neck_cfg, "/camera/color/camera_info")
        neck_surface_pose_topic = self._string_param_or_config("neck_surface_pose_topic", neck_cfg, "/neck_surface_pose", "pose_topic")
        target_pose_topic = self._string_param_or_config("target_pose_topic", neck_cfg, "/target_pose_base")
        approach_target_topic = self._string_param_or_config("approach_target_topic", neck_cfg, "/approach_target")
        status_topic = self._string_param_or_config("status_topic", neck_cfg, "/neck_surface_status")
        debug_image_topic = self._string_param_or_config("debug_image_topic", neck_cfg, "/neck_roi_debug_image")
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").get_parameter_value().bool_value)
        self.publish_debug_image = bool(neck_cfg.get("publish_debug_image", self.publish_debug_image))
        self.execute_motion = bool(self.get_parameter("execute_motion").get_parameter_value().bool_value)
        self.pair_queue_size = max(1, int(self.get_parameter("pair_queue_size").get_parameter_value().integer_value))
        self.max_rgb_depth_delta_ms = float(neck_cfg.get("max_rgb_depth_delta_ms", 200.0))

        self.eye_to_hand_solution = load_eye_to_hand_solution(extrinsics_path)
        self.bridge = CvBridge()
        self.pending_images: dict[int, Image] = {}
        self.pending_depths: dict[int, Image] = {}
        self._latest_camera_info: CameraInfo | None = None
        self._camera_info_signature: tuple[object, ...] | None = None
        self._last_valid_pose_message: PoseStamped | None = None
        self._last_valid_target_pose_message: PoseStamped | None = None
        self._last_valid_point_message: PointStamped | None = None
        self._last_valid_status_payload: dict[str, Any] | None = None
        self._last_valid_time_monotonic: float | None = None

        try:
            self.keypoint_backend = MediaPipePoseBackend(
                min_detection_confidence=float(neck_cfg.get("keypoint_confidence_threshold", 0.5)),
                min_tracking_confidence=float(neck_cfg.get("keypoint_confidence_threshold", 0.5)),
            )
            self._backend_error = None
        except RuntimeError as exc:
            self.keypoint_backend = None
            self._backend_error = str(exc)
            self.get_logger().warning(json.dumps({"event": "mediapipe_backend_unavailable", "error": self._backend_error}, ensure_ascii=False))

        self.image_subscription = self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.depth_subscription = self.create_subscription(Image, depth_topic, self._depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, 10)
        self.neck_surface_pose_publisher = self.create_publisher(PoseStamped, neck_surface_pose_topic, 10)
        self.target_pose_publisher = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.approach_target_publisher = self.create_publisher(PointStamped, approach_target_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.debug_image_publisher = self.create_publisher(Image, debug_image_topic, 10) if self.publish_debug_image else None

        self._publish_status(
            {
                "source": "markerless_neck",
                "status": "ready" if self.keypoint_backend is not None else "failed",
                "reason": None if self.keypoint_backend is not None else "mediapipe_unavailable",
                "message": "Neck surface pose node started." if self.keypoint_backend is not None else self._backend_error,
                "execute_motion": self.execute_motion,
                "image_topic": image_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "max_rgb_depth_delta_ms": self.max_rgb_depth_delta_ms,
            }
        )

    def _string_param_or_config(self, parameter_name: str, config: dict[str, Any], fallback: str, config_key: str | None = None) -> str:
        parameter_value = self.get_parameter(parameter_name).get_parameter_value().string_value
        if parameter_value:
            return parameter_value
        return str(config.get(config_key or parameter_name, fallback))

    def _publish_status(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)

    @staticmethod
    def _stamp_ns(message: Image | CameraInfo) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _image_callback(self, message: Image) -> None:
        self.pending_images[self._stamp_ns(message)] = message
        self._prune_pending(self.pending_images, "image")
        self._try_process_nearest_pair()

    def _depth_callback(self, message: Image) -> None:
        self.pending_depths[self._stamp_ns(message)] = message
        self._prune_pending(self.pending_depths, "depth")
        self._try_process_nearest_pair()

    def _camera_info_callback(self, message: CameraInfo) -> None:
        try:
            signature = camera_info_signature(message)
        except ValueError as exc:
            self._publish_status({"source": "markerless_neck", "status": "failed", "reason": "camera_info_invalid", "message": str(exc)})
            return
        self._camera_info_signature = signature
        self._latest_camera_info = message
        self._try_process_nearest_pair()

    def _prune_pending(self, pending: dict[int, Image], frame_type: str) -> None:
        while len(pending) > self.pair_queue_size:
            oldest_key = sorted(pending.keys())[0]
            pending.pop(oldest_key, None)
            self.get_logger().warning(
                json.dumps(
                    {
                        "event": "unpaired_frame_dropped",
                        "frame_type": frame_type,
                        "stamp_ns": oldest_key,
                    },
                    ensure_ascii=False,
                )
            )

    def _find_nearest_depth_key(self, image_key: int) -> tuple[int | None, float | None]:
        if not self.pending_depths:
            return None, None
        depth_key = min(self.pending_depths.keys(), key=lambda candidate: abs(candidate - image_key))
        delta_ms = abs(float(depth_key - image_key)) / 1_000_000.0
        if delta_ms > self.max_rgb_depth_delta_ms:
            return None, delta_ms
        return depth_key, delta_ms

    def _try_process_nearest_pair(self) -> None:
        if self._latest_camera_info is None or not self.pending_images or not self.pending_depths:
            return
        for image_key in sorted(self.pending_images.keys()):
            depth_key, delta_ms = self._find_nearest_depth_key(image_key)
            if depth_key is None:
                continue
            image_message = self.pending_images.pop(image_key)
            depth_message = self.pending_depths.pop(depth_key)
            self._process_triplet(image_message, depth_message, self._latest_camera_info, rgb_depth_delta_ms=delta_ms)
            return

    def _process_triplet(self, image_message: Image, depth_message: Image, camera_info_message: CameraInfo, *, rgb_depth_delta_ms: float | None = None) -> None:
        if self.keypoint_backend is None:
            self._publish_status({"source": "markerless_neck", "status": "failed", "reason": "mediapipe_unavailable", "message": self._backend_error})
            self._maybe_publish_latched(image_message.header)
            return

        image_bgr = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        depth_cv = _normalize_depth_image(self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough"))
        calibration = camera_info_to_calibration(camera_info_message)
        keypoints = self.keypoint_backend.detect(image_bgr)
        estimate = estimate_neck_surface_pose(keypoints, depth_cv, calibration.camera_matrix, self.estimator_config)
        debug_image = draw_debug_overlay(image_bgr, estimate, calibration.camera_matrix)

        if estimate.ok:
            self._publish_fresh_estimate(image_message, estimate, rgb_depth_delta_ms=rgb_depth_delta_ms)
        else:
            payload = estimate.status_payload()
            payload.update(
                {
                    "image_stamp_ns": self._stamp_ns(image_message),
                    "image_frame_id": image_message.header.frame_id,
                }
            )
            payload["execute_motion"] = self.execute_motion
            payload["rgb_depth_delta_ms"] = rgb_depth_delta_ms
            self._publish_status(payload)
            self._maybe_publish_latched(image_message.header)

        if self.debug_image_publisher is not None:
            debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_message.header = image_message.header
            self.debug_image_publisher.publish(debug_message)
        if self.logging_enabled:
            timestamp = f"frame_{image_message.header.stamp.sec}_{image_message.header.stamp.nanosec}"
            try:
                write_logging_artifacts(self.logging_output_dir, image_bgr, depth_cv, debug_image, estimate, timestamp)
            except Exception as exc:
                self.get_logger().warning(json.dumps({"event": "neck_surface_logging_failed", "error": repr(exc)}, ensure_ascii=False))

    def _publish_fresh_estimate(self, image_message: Image, estimate: NeckSurfaceEstimate, *, rgb_depth_delta_ms: float | None = None) -> None:
        assert estimate.surface_point_camera_m is not None
        assert estimate.rotation_matrix_camera is not None
        pose_message = self._build_pose_message(image_message, estimate)
        self.neck_surface_pose_publisher.publish(pose_message)
        self._last_valid_pose_message = pose_message

        target_pose_message = self._build_base_pose_message(pose_message)
        point_message = None
        if target_pose_message is not None:
            self.target_pose_publisher.publish(target_pose_message)
            point_message = PointStamped()
            point_message.header = target_pose_message.header
            point_message.point.x = target_pose_message.pose.position.x
            point_message.point.y = target_pose_message.pose.position.y
            point_message.point.z = target_pose_message.pose.position.z
            self.approach_target_publisher.publish(point_message)
            self._last_valid_target_pose_message = target_pose_message
            self._last_valid_point_message = point_message

        payload = estimate.status_payload()
        payload.update(
            {
                "image_stamp_ns": self._stamp_ns(image_message),
                "image_frame_id": image_message.header.frame_id,
                "execute_motion": self.execute_motion,
                "target_pose_frame": target_pose_message.header.frame_id if target_pose_message is not None else None,
                "rgb_depth_delta_ms": rgb_depth_delta_ms,
                "transform_status": "ok" if target_pose_message is not None else "missing_extrinsic",
            }
        )
        self._publish_status(payload)
        self._last_valid_status_payload = payload
        self._last_valid_time_monotonic = time.monotonic()

    def _build_pose_message(self, image_message: Image, estimate: NeckSurfaceEstimate) -> PoseStamped:
        assert estimate.surface_point_camera_m is not None
        assert estimate.rotation_matrix_camera is not None
        target_point = estimate.target_point_camera_m or estimate.surface_point_camera_m
        quaternion = rotation_matrix_to_quaternion_xyzw(np.asarray(estimate.rotation_matrix_camera, dtype=np.float64))
        pose_message = PoseStamped()
        pose_message.header = image_message.header
        pose_message.header.frame_id = image_message.header.frame_id or "camera"
        pose_message.pose.position.x = float(target_point[0])
        pose_message.pose.position.y = float(target_point[1])
        pose_message.pose.position.z = float(target_point[2])
        pose_message.pose.orientation.x = float(quaternion[0])
        pose_message.pose.orientation.y = float(quaternion[1])
        pose_message.pose.orientation.z = float(quaternion[2])
        pose_message.pose.orientation.w = float(quaternion[3])
        return pose_message

    def _build_base_pose_message(self, camera_pose: PoseStamped) -> PoseStamped | None:
        if not self.eye_to_hand_solution.success or self.eye_to_hand_solution.base_to_camera is None:
            return None
        camera_to_surface = make_transform_matrix(
            [camera_pose.pose.position.x, camera_pose.pose.position.y, camera_pose.pose.position.z],
            self._quaternion_to_rotation(camera_pose),
        )
        base_to_camera = make_transform_matrix(
            self.eye_to_hand_solution.base_to_camera.translation_m,
            self.eye_to_hand_solution.base_to_camera.rotation_matrix,
        )
        base_to_surface = base_to_camera @ camera_to_surface
        translation_m, rotation = split_transform_matrix(base_to_surface)
        transform = make_transform_struct(translation_m, rotation, "robot_base", "neck_surface")
        pose_message = PoseStamped()
        pose_message.header = camera_pose.header
        pose_message.header.frame_id = "robot_base"
        pose_message.pose.position.x = transform.translation_m[0]
        pose_message.pose.position.y = transform.translation_m[1]
        pose_message.pose.position.z = transform.translation_m[2]
        pose_message.pose.orientation.x = transform.rotation_quaternion_xyzw[0]
        pose_message.pose.orientation.y = transform.rotation_quaternion_xyzw[1]
        pose_message.pose.orientation.z = transform.rotation_quaternion_xyzw[2]
        pose_message.pose.orientation.w = transform.rotation_quaternion_xyzw[3]
        return pose_message

    def _quaternion_to_rotation(self, pose_message: PoseStamped) -> np.ndarray:
        from paus_perception import quaternion_xyzw_to_rotation_matrix

        return quaternion_xyzw_to_rotation_matrix(
            [
                pose_message.pose.orientation.x,
                pose_message.pose.orientation.y,
                pose_message.pose.orientation.z,
                pose_message.pose.orientation.w,
            ]
        )

    def _maybe_publish_latched(self, header) -> None:
        if self.execute_motion:
            return
        if self._last_valid_time_monotonic is None:
            return
        if time.monotonic() - self._last_valid_time_monotonic > self.latched_pose_timeout_s:
            return
        if self._last_valid_pose_message is not None:
            latched_pose = deepcopy(self._last_valid_pose_message)
            latched_pose.header.stamp = header.stamp
            latched_pose.header.frame_id = header.frame_id or latched_pose.header.frame_id
            self.neck_surface_pose_publisher.publish(latched_pose)
        if self._last_valid_target_pose_message is not None:
            latched_target_pose = deepcopy(self._last_valid_target_pose_message)
            latched_target_pose.header.stamp = header.stamp
            self.target_pose_publisher.publish(latched_target_pose)
        if self._last_valid_point_message is not None:
            latched_point = deepcopy(self._last_valid_point_message)
            latched_point.header.stamp = header.stamp
            self.approach_target_publisher.publish(latched_point)
        payload = dict(self._last_valid_status_payload or {})
        payload.update({"status": "ok", "pose_freshness": LATCHED_POSE, "message": "Publishing recent latched neck surface pose in dry-run mode."})
        payload["image_stamp_ns"] = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        payload["image_frame_id"] = header.frame_id or payload.get("image_frame_id")
        self._publish_status(payload)

    def destroy_node(self) -> bool:
        if self.keypoint_backend is not None:
            self.keypoint_backend.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NeckSurfacePoseNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
