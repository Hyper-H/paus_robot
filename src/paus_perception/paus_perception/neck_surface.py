from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Mapping

import cv2
import numpy as np


STATUS_OK = "ok"
STATUS_FAILED = "failed"
FRESH_POSE = "fresh_pose"
LATCHED_POSE = "latched_pose"

REASON_LOW_KEYPOINT_CONFIDENCE = "low_keypoint_confidence"
REASON_NECK_WINDOW_OUT_OF_BOUNDS = "neck_window_out_of_bounds"
REASON_INSUFFICIENT_CENTER_DEPTH_POINTS = "insufficient_center_depth_points"
REASON_INSUFFICIENT_PATCH_POINTS = "insufficient_patch_points"
REASON_DEPTH_OUTLIER_RATIO_TOO_HIGH = "depth_outlier_ratio_too_high"
REASON_PLANE_FIT_RMSE_TOO_HIGH = "plane_fit_rmse_too_high"
REASON_NORMAL_DIRECTION_INVALID = "normal_direction_invalid"
REASON_TANGENT_DIRECTION_DEGENERATE = "tangent_direction_degenerate"
REASON_MEDIAPIPE_UNAVAILABLE = "mediapipe_unavailable"
REASON_INSUFFICIENT_REFINEMENT_POINTS = "insufficient_refinement_points"
REASON_INSUFFICIENT_FINAL_PATCH_POINTS = "insufficient_final_patch_points"

TARGET_MODE_REFINED_SURFACE = "refined_surface"
TARGET_MODE_SHOULDER_CENTER = "shoulder_center"
SUPPORTED_TARGET_MODES = {TARGET_MODE_REFINED_SURFACE, TARGET_MODE_SHOULDER_CENTER}


@dataclass(frozen=True)
class PoseKeypoint:
    name: str
    x_px: float
    y_px: float
    confidence: float = 1.0


@dataclass(frozen=True)
class ImageWindow:
    x: int
    y: int
    width: int
    height: int

    @property
    def x2(self) -> int:
        return self.x + self.width

    @property
    def y2(self) -> int:
        return self.y + self.height

    @property
    def center(self) -> tuple[float, float]:
        return (float(self.x) + float(self.width) / 2.0, float(self.y) + float(self.height) / 2.0)

    def as_list(self) -> list[int]:
        return [int(self.x), int(self.y), int(self.width), int(self.height)]


@dataclass(frozen=True)
class NeckSurfaceConfig:
    keypoint_confidence_threshold: float = 0.5
    alpha_with_ears: float = 0.55
    alpha_with_nose: float = 0.65
    roi_width_shoulder_scale: float = 0.35
    roi_height_head_shoulder_scale: float = 0.50
    center_window_scale: float = 0.45
    depth_outlier_mm: float = 50.0
    patch_radius_mm: float = 30.0
    min_center_points: int = 80
    min_patch_points: int = 50
    max_plane_rmse_mm: float = 8.0
    max_depth_outlier_ratio: float = 0.80
    target_region: str = "center"
    target_mode: str = TARGET_MODE_REFINED_SURFACE
    lateral_offset_mm: float = 0.0
    inferior_offset_mm: float = 0.0

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "NeckSurfaceConfig":
        if payload is None:
            return cls()
        defaults = cls()
        return cls(
            keypoint_confidence_threshold=float(payload.get("keypoint_confidence_threshold", defaults.keypoint_confidence_threshold)),
            alpha_with_ears=float(payload.get("alpha_with_ears", defaults.alpha_with_ears)),
            alpha_with_nose=float(payload.get("alpha_with_nose", defaults.alpha_with_nose)),
            roi_width_shoulder_scale=float(payload.get("roi_width_shoulder_scale", defaults.roi_width_shoulder_scale)),
            roi_height_head_shoulder_scale=float(payload.get("roi_height_head_shoulder_scale", defaults.roi_height_head_shoulder_scale)),
            center_window_scale=float(payload.get("center_window_scale", defaults.center_window_scale)),
            depth_outlier_mm=float(payload.get("depth_outlier_mm", defaults.depth_outlier_mm)),
            patch_radius_mm=float(payload.get("patch_radius_mm", defaults.patch_radius_mm)),
            min_center_points=int(payload.get("min_center_points", defaults.min_center_points)),
            min_patch_points=int(payload.get("min_patch_points", defaults.min_patch_points)),
            max_plane_rmse_mm=float(payload.get("max_plane_rmse_mm", defaults.max_plane_rmse_mm)),
            max_depth_outlier_ratio=float(payload.get("max_depth_outlier_ratio", defaults.max_depth_outlier_ratio)),
            target_region=str(payload.get("target_region", defaults.target_region)).strip().lower(),
            target_mode=str(payload.get("target_mode", defaults.target_mode)).strip().lower(),
            lateral_offset_mm=float(payload.get("lateral_offset_mm", defaults.lateral_offset_mm)),
            inferior_offset_mm=float(payload.get("inferior_offset_mm", defaults.inferior_offset_mm)),
        )


@dataclass(frozen=True)
class NeckWindowResult:
    status: str
    reason: str | None
    message: str
    search_window: ImageWindow | None = None
    center_window: ImageWindow | None = None
    using_ears: bool = False
    shoulder_width_px: float = 0.0
    head_to_shoulder_px: float = 0.0


@dataclass(frozen=True)
class NeckSurfaceEstimate:
    status: str
    reason: str | None
    message: str
    pose_freshness: str = FRESH_POSE
    search_window: ImageWindow | None = None
    center_window: ImageWindow | None = None
    using_ears: bool = False
    surface_point_camera_m: list[float] | None = None
    target_point_camera_m: list[float] | None = None
    surface_normal_camera: list[float] | None = None
    tangent_x_camera: list[float] | None = None
    tangent_y_camera: list[float] | None = None
    rotation_matrix_camera: list[list[float]] | None = None
    motion_normal_camera: list[float] | None = None
    patient_left_outward_normal_camera: list[float] | None = None
    pca_surface_normal_camera: list[float] | None = None
    pca_rotation_matrix_camera: list[list[float]] | None = None
    coarse_surface_point_camera_m: list[float] | None = None
    coarse_target_point_camera_m: list[float] | None = None
    coarse_surface_normal_camera: list[float] | None = None
    coarse_tangent_x_camera: list[float] | None = None
    coarse_tangent_y_camera: list[float] | None = None
    coarse_rotation_matrix_camera: list[list[float]] | None = None
    coarse_patch_points: int = 0
    coarse_plane_rmse_mm: float | None = None
    refinement_status: str | None = None
    refinement_reason: str | None = None
    refinement_points: int = 0
    target_mode: str = TARGET_MODE_REFINED_SURFACE
    target_region: str = "center"
    lateral_offset_mm: float = 0.0
    inferior_offset_mm: float = 0.0
    left_shoulder_camera_m: list[float] | None = None
    right_shoulder_camera_m: list[float] | None = None
    shoulder_center_camera_m: list[float] | None = None
    valid_depth_points: int = 0
    filtered_center_points: int = 0
    patch_points: int = 0
    plane_rmse_mm: float | None = None
    depth_outlier_ratio: float | None = None
    keypoints: dict[str, list[float]] = field(default_factory=dict)
    center_pixels: list[list[float]] = field(default_factory=list, repr=False, compare=False)
    patch_pixels: list[list[float]] = field(default_factory=list, repr=False, compare=False)
    coarse_patch_pixels: list[list[float]] = field(default_factory=list, repr=False, compare=False)
    refinement_pixels: list[list[float]] = field(default_factory=list, repr=False, compare=False)

    @property
    def ok(self) -> bool:
        return self.status == STATUS_OK

    def status_payload(self) -> dict[str, Any]:
        return {
            "source": "markerless_neck",
            "status": self.status,
            "reason": self.reason,
            "message": self.message,
            "pose_freshness": self.pose_freshness,
            "keypoint_backend": "mediapipe_pose",
            "using_ears": self.using_ears,
            "neck_search_window": self.search_window.as_list() if self.search_window else None,
            "center_window": self.center_window.as_list() if self.center_window else None,
            "surface_point_camera_m": self.surface_point_camera_m,
            "neck_anchor_camera_m": self.surface_point_camera_m,
            "target_point_camera_m": self.target_point_camera_m,
            "surface_normal_camera": self.surface_normal_camera,
            "tangent_x_camera": self.tangent_x_camera,
            "tangent_y_camera": self.tangent_y_camera,
            "rotation_matrix_camera": self.rotation_matrix_camera,
            "motion_normal_camera": self.motion_normal_camera,
            "patient_left_outward_normal_camera": self.patient_left_outward_normal_camera,
            "pca_surface_normal_camera": self.pca_surface_normal_camera,
            "pca_rotation_matrix_camera": self.pca_rotation_matrix_camera,
            "coarse_surface_point_camera_m": self.coarse_surface_point_camera_m,
            "coarse_target_point_camera_m": self.coarse_target_point_camera_m,
            "coarse_surface_normal_camera": self.coarse_surface_normal_camera,
            "coarse_tangent_x_camera": self.coarse_tangent_x_camera,
            "coarse_tangent_y_camera": self.coarse_tangent_y_camera,
            "coarse_rotation_matrix_camera": self.coarse_rotation_matrix_camera,
            "coarse_patch_points": self.coarse_patch_points,
            "coarse_plane_rmse_mm": self.coarse_plane_rmse_mm,
            "refinement_status": self.refinement_status,
            "refinement_reason": self.refinement_reason,
            "refinement_points": self.refinement_points,
            "target_mode": self.target_mode,
            "target_region": self.target_region,
            "lateral_offset_mm": self.lateral_offset_mm,
            "inferior_offset_mm": self.inferior_offset_mm,
            "left_shoulder_camera_m": self.left_shoulder_camera_m,
            "right_shoulder_camera_m": self.right_shoulder_camera_m,
            "shoulder_center_camera_m": self.shoulder_center_camera_m,
            "valid_depth_points": self.valid_depth_points,
            "filtered_center_points": self.filtered_center_points,
            "patch_points": self.patch_points,
            "plane_rmse_mm": self.plane_rmse_mm,
            "depth_outlier_ratio": self.depth_outlier_ratio,
        }


def _failure(reason: str, message: str, *, window_result: NeckWindowResult | None = None, **kwargs: Any) -> NeckSurfaceEstimate:
    return NeckSurfaceEstimate(
        status=STATUS_FAILED,
        reason=reason,
        message=message,
        search_window=window_result.search_window if window_result else kwargs.pop("search_window", None),
        center_window=window_result.center_window if window_result else kwargs.pop("center_window", None),
        using_ears=window_result.using_ears if window_result else bool(kwargs.pop("using_ears", False)),
        **kwargs,
    )


def _config_failure(reason: str, message: str, cfg: NeckSurfaceConfig, *, window_result: NeckWindowResult | None = None, **kwargs: Any) -> NeckSurfaceEstimate:
    return _failure(
        reason,
        message,
        window_result=window_result,
        target_mode=cfg.target_mode,
        target_region=cfg.target_region,
        lateral_offset_mm=float(cfg.lateral_offset_mm),
        inferior_offset_mm=float(cfg.inferior_offset_mm),
        **kwargs,
    )


def _as_keypoint_map(keypoints: Mapping[str, PoseKeypoint]) -> dict[str, list[float]]:
    return {
        name: [float(point.x_px), float(point.y_px), float(point.confidence)]
        for name, point in keypoints.items()
    }


def _valid_keypoint(point: PoseKeypoint | None, threshold: float) -> bool:
    if point is None:
        return False
    return (
        math.isfinite(float(point.x_px))
        and math.isfinite(float(point.y_px))
        and math.isfinite(float(point.confidence))
        and float(point.confidence) >= float(threshold)
    )


def _clip_window(raw_x: float, raw_y: float, raw_width: float, raw_height: float, image_width: int, image_height: int) -> tuple[ImageWindow | None, bool]:
    if raw_width <= 1.0 or raw_height <= 1.0:
        return None, True
    x1 = int(math.floor(raw_x))
    y1 = int(math.floor(raw_y))
    x2 = int(math.ceil(raw_x + raw_width))
    y2 = int(math.ceil(raw_y + raw_height))
    clipped = x1 < 0 or y1 < 0 or x2 > image_width or y2 > image_height
    x1 = max(0, min(int(image_width), x1))
    y1 = max(0, min(int(image_height), y1))
    x2 = max(0, min(int(image_width), x2))
    y2 = max(0, min(int(image_height), y2))
    width = x2 - x1
    height = y2 - y1
    if width <= 1 or height <= 1:
        return None, True
    return ImageWindow(x=x1, y=y1, width=width, height=height), clipped


def build_neck_windows(
    keypoints: Mapping[str, PoseKeypoint],
    image_width: int,
    image_height: int,
    config: NeckSurfaceConfig,
) -> NeckWindowResult:
    threshold = config.keypoint_confidence_threshold
    left_shoulder = keypoints.get("left_shoulder")
    right_shoulder = keypoints.get("right_shoulder")
    nose = keypoints.get("nose")
    left_ear = keypoints.get("left_ear")
    right_ear = keypoints.get("right_ear")

    if not _valid_keypoint(left_shoulder, threshold) or not _valid_keypoint(right_shoulder, threshold):
        return NeckWindowResult(STATUS_FAILED, REASON_LOW_KEYPOINT_CONFIDENCE, "Shoulder keypoints are missing or low confidence.")
    assert left_shoulder is not None and right_shoulder is not None

    use_ears = _valid_keypoint(left_ear, threshold) and _valid_keypoint(right_ear, threshold)
    if use_ears:
        assert left_ear is not None and right_ear is not None
        head_x = (float(left_ear.x_px) + float(right_ear.x_px)) / 2.0
        head_y = (float(left_ear.y_px) + float(right_ear.y_px)) / 2.0
        alpha = config.alpha_with_ears
    else:
        if not _valid_keypoint(nose, threshold):
            return NeckWindowResult(STATUS_FAILED, REASON_LOW_KEYPOINT_CONFIDENCE, "Nose is missing or low confidence and ears are unavailable.")
        assert nose is not None
        head_x = float(nose.x_px)
        head_y = float(nose.y_px)
        alpha = config.alpha_with_nose

    shoulder_x = (float(left_shoulder.x_px) + float(right_shoulder.x_px)) / 2.0
    shoulder_y = (float(left_shoulder.y_px) + float(right_shoulder.y_px)) / 2.0
    shoulder_width = float(math.hypot(float(right_shoulder.x_px) - float(left_shoulder.x_px), float(right_shoulder.y_px) - float(left_shoulder.y_px)))
    head_to_shoulder = float(math.hypot(shoulder_x - head_x, shoulder_y - head_y))
    if shoulder_width <= 1.0 or head_to_shoulder <= 1.0:
        return NeckWindowResult(STATUS_FAILED, REASON_LOW_KEYPOINT_CONFIDENCE, "Keypoint geometry is degenerate.")

    center_x = head_x + float(alpha) * (shoulder_x - head_x)
    center_y = head_y + float(alpha) * (shoulder_y - head_y)
    roi_width = config.roi_width_shoulder_scale * shoulder_width
    roi_height = config.roi_height_head_shoulder_scale * head_to_shoulder

    search_window, search_clipped = _clip_window(
        center_x - roi_width / 2.0,
        center_y - roi_height / 2.0,
        roi_width,
        roi_height,
        image_width,
        image_height,
    )
    if search_window is None:
        return NeckWindowResult(STATUS_FAILED, REASON_NECK_WINDOW_OUT_OF_BOUNDS, "Neck search window is outside the image.")

    center_width = max(2.0, float(config.center_window_scale) * float(search_window.width))
    center_height = max(2.0, float(config.center_window_scale) * float(search_window.height))
    center_window, center_clipped = _clip_window(
        center_x - center_width / 2.0,
        center_y - center_height / 2.0,
        center_width,
        center_height,
        image_width,
        image_height,
    )
    if center_window is None or center_clipped:
        return NeckWindowResult(STATUS_FAILED, REASON_NECK_WINDOW_OUT_OF_BOUNDS, "Neck center window is outside the image.")
    if search_clipped:
        return NeckWindowResult(STATUS_FAILED, REASON_NECK_WINDOW_OUT_OF_BOUNDS, "Neck search window was clipped by image bounds.", search_window, center_window, use_ears, shoulder_width, head_to_shoulder)

    return NeckWindowResult(
        status=STATUS_OK,
        reason=None,
        message="Neck windows computed successfully.",
        search_window=search_window,
        center_window=center_window,
        using_ears=use_ears,
        shoulder_width_px=shoulder_width,
        head_to_shoulder_px=head_to_shoulder,
    )


def depth_to_meters(depth_image: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth_image)
    if depth.dtype == np.uint16:
        return depth.astype(np.float64) / 1000.0
    return depth.astype(np.float64)


def _camera_intrinsics(camera_matrix: list[list[float]] | np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    return float(matrix[0, 0]), float(matrix[1, 1]), float(matrix[0, 2]), float(matrix[1, 2])


def project_depth_window_to_points(
    depth_image_m: np.ndarray,
    window: ImageWindow,
    camera_matrix: list[list[float]] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    fx, fy, cx, cy = _camera_intrinsics(camera_matrix)
    patch = np.asarray(depth_image_m[window.y:window.y2, window.x:window.x2], dtype=np.float64)
    valid_mask = np.isfinite(patch) & (patch > 0.0)
    rows, cols = np.nonzero(valid_mask)
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 2), dtype=np.float64)
    z = patch[rows, cols]
    u = cols.astype(np.float64) + float(window.x)
    v = rows.astype(np.float64) + float(window.y)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    points = np.column_stack((x, y, z)).astype(np.float64)
    pixels = np.column_stack((u, v)).astype(np.float64)
    return points, pixels


def filter_points_by_depth_band(points: np.ndarray, depth_outlier_mm: float) -> tuple[np.ndarray, float]:
    if points.size == 0:
        return points.reshape(0, 3), 0.0
    z = points[:, 2]
    median_z = float(np.median(z))
    threshold_m = float(depth_outlier_mm) / 1000.0
    keep = np.abs(z - median_z) <= threshold_m
    outlier_ratio = 1.0 - (float(np.count_nonzero(keep)) / float(points.shape[0]))
    return points[keep], outlier_ratio


def filter_points_and_pixels_by_depth_band(points: np.ndarray, pixels: np.ndarray, depth_outlier_mm: float) -> tuple[np.ndarray, np.ndarray, float]:
    if points.size == 0:
        return points.reshape(0, 3), np.empty((0, 2), dtype=np.float64), 0.0
    z = points[:, 2]
    median_z = float(np.median(z))
    threshold_m = float(depth_outlier_mm) / 1000.0
    keep = np.abs(z - median_z) <= threshold_m
    outlier_ratio = 1.0 - (float(np.count_nonzero(keep)) / float(points.shape[0]))
    return points[keep], np.asarray(pixels, dtype=np.float64)[keep], outlier_ratio


def median_surface_point(points: np.ndarray) -> np.ndarray:
    if points.size == 0:
        raise ValueError("Cannot estimate a surface point from zero points.")
    return np.median(points, axis=0).astype(np.float64)


def extract_local_patch(points: np.ndarray, center: np.ndarray, radius_mm: float) -> np.ndarray:
    if points.size == 0:
        return points.reshape(0, 3)
    radius_m = float(radius_mm) / 1000.0
    distances = np.linalg.norm(points - np.asarray(center, dtype=np.float64).reshape(3), axis=1)
    return points[distances <= radius_m]


def estimate_pca_normal(points: np.ndarray) -> tuple[np.ndarray, float]:
    if points.shape[0] < 3:
        raise ValueError("At least three points are required for PCA normal estimation.")
    centered = points - np.mean(points, axis=0)
    covariance = centered.T @ centered / float(points.shape[0])
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normal = eigenvectors[:, int(np.argmin(eigenvalues))]
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        raise ValueError("PCA normal is degenerate.")
    normal = normal / norm
    residuals = centered @ normal
    rmse_m = float(np.sqrt(np.mean(residuals ** 2)))
    return normal.astype(np.float64), rmse_m * 1000.0


def orient_normal_toward_camera(normal: np.ndarray, surface_point: np.ndarray) -> np.ndarray:
    normal_vec = np.asarray(normal, dtype=np.float64).reshape(3)
    point_vec = np.asarray(surface_point, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(normal_vec))
    if norm <= 1e-12:
        raise ValueError("Surface normal is degenerate.")
    normal_vec = normal_vec / norm
    camera_direction = -point_vec
    if float(np.linalg.norm(camera_direction)) > 1e-12 and float(np.dot(normal_vec, camera_direction)) < 0.0:
        normal_vec = -normal_vec
    return normal_vec


def estimate_keypoint_3d(
    keypoint: PoseKeypoint,
    depth_image_m: np.ndarray,
    camera_matrix: list[list[float]] | np.ndarray,
    window_radius_px: int = 3,
) -> np.ndarray | None:
    image_height, image_width = depth_image_m.shape[:2]
    u = int(round(float(keypoint.x_px)))
    v = int(round(float(keypoint.y_px)))
    window, _ = _clip_window(
        u - window_radius_px,
        v - window_radius_px,
        window_radius_px * 2 + 1,
        window_radius_px * 2 + 1,
        image_width,
        image_height,
    )
    if window is None:
        return None
    points, _ = project_depth_window_to_points(depth_image_m, window, camera_matrix)
    if points.shape[0] == 0:
        return None
    return median_surface_point(points)


def project_tangent_to_plane(tangent: np.ndarray, normal: np.ndarray) -> np.ndarray:
    tangent_vec = np.asarray(tangent, dtype=np.float64).reshape(3)
    normal_vec = np.asarray(normal, dtype=np.float64).reshape(3)
    normal_norm = float(np.linalg.norm(normal_vec))
    if normal_norm <= 1e-12:
        raise ValueError("Normal is degenerate.")
    normal_vec = normal_vec / normal_norm
    projected = tangent_vec - float(np.dot(tangent_vec, normal_vec)) * normal_vec
    projected_norm = float(np.linalg.norm(projected))
    if projected_norm <= 1e-12:
        raise ValueError("Tangent direction is degenerate after projection.")
    return projected / projected_norm


def build_surface_rotation(tangent_x: np.ndarray, normal_z: np.ndarray) -> np.ndarray:
    z_axis = np.asarray(normal_z, dtype=np.float64).reshape(3)
    z_norm = float(np.linalg.norm(z_axis))
    if z_norm <= 1e-12:
        raise ValueError("Surface normal is degenerate.")
    z_axis = z_axis / z_norm
    x_axis = project_tangent_to_plane(tangent_x, z_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1e-12:
        raise ValueError("Surface side axis is degenerate.")
    y_axis = y_axis / y_norm
    x_axis = np.cross(y_axis, z_axis)
    x_axis = x_axis / float(np.linalg.norm(x_axis))
    rotation = np.column_stack((x_axis, y_axis, z_axis)).astype(np.float64)
    if float(np.linalg.det(rotation)) < 0.0:
        y_axis = -y_axis
        rotation = np.column_stack((x_axis, y_axis, z_axis)).astype(np.float64)
    return rotation


def build_side_approach_rotation(patient_left_outward: np.ndarray, shoulder_vec: np.ndarray, fallback_tangent: np.ndarray) -> np.ndarray:
    z_axis = np.asarray(patient_left_outward, dtype=np.float64).reshape(3)
    z_norm = float(np.linalg.norm(z_axis))
    if z_norm <= 1e-12:
        raise ValueError("Patient-left outward normal is degenerate.")
    z_axis = z_axis / z_norm

    tangent_seed = np.asarray(shoulder_vec, dtype=np.float64).reshape(3)
    tangent_projection = tangent_seed - float(np.dot(tangent_seed, z_axis)) * z_axis
    if float(np.linalg.norm(tangent_projection)) <= 1e-12:
        tangent_seed = np.asarray(fallback_tangent, dtype=np.float64).reshape(3)
        tangent_projection = tangent_seed - float(np.dot(tangent_seed, z_axis)) * z_axis
    if float(np.linalg.norm(tangent_projection)) <= 1e-12:
        for candidate in (
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            np.array([1.0, 0.0, 0.0], dtype=np.float64),
        ):
            tangent_projection = candidate - float(np.dot(candidate, z_axis)) * z_axis
            if float(np.linalg.norm(tangent_projection)) > 1e-12:
                break
    x_axis = project_tangent_to_plane(tangent_projection, z_axis)
    return build_surface_rotation(x_axis, z_axis)


def compute_target_point_from_anchor(
    surface_point: np.ndarray,
    rotation_matrix: np.ndarray,
    target_region: str,
    lateral_offset_mm: float,
    inferior_offset_mm: float,
) -> np.ndarray:
    region = str(target_region or "center").strip().lower()
    if region not in {"center", "patient_left", "patient_right"}:
        raise ValueError(f"Unsupported target_region: {target_region}")

    rotation = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    x_axis = rotation[:, 0]
    y_axis = rotation[:, 1]
    lateral_sign = 0.0
    if region == "patient_left":
        # x_axis is built from patient-left shoulder toward patient-right shoulder.
        lateral_sign = -1.0
    elif region == "patient_right":
        lateral_sign = 1.0

    return (
        np.asarray(surface_point, dtype=np.float64).reshape(3)
        + lateral_sign * (float(lateral_offset_mm) / 1000.0) * x_axis
        + (float(inferior_offset_mm) / 1000.0) * y_axis
    )


def _as_float_list(vector: np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(vector, dtype=np.float64).reshape(3).tolist()]


def _as_float_matrix(matrix: np.ndarray) -> list[list[float]]:
    return [[float(value) for value in row] for row in np.asarray(matrix, dtype=np.float64).reshape(3, 3).tolist()]


def _as_unit_float_list(vector: np.ndarray) -> list[float]:
    values = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        return [0.0, 0.0, 0.0]
    return _as_float_list(values / norm)


def estimate_neck_surface_pose(
    keypoints: Mapping[str, PoseKeypoint],
    depth_image: np.ndarray,
    camera_matrix: list[list[float]] | np.ndarray,
    config: NeckSurfaceConfig | None = None,
) -> NeckSurfaceEstimate:
    cfg = config or NeckSurfaceConfig()
    if cfg.target_mode not in SUPPORTED_TARGET_MODES:
        return _config_failure(
            "unsupported_target_mode",
            f"Unsupported neck_surface.target_mode: {cfg.target_mode!r}",
            cfg,
            keypoints=_as_keypoint_map(keypoints),
        )
    depth_m = depth_to_meters(depth_image)
    image_height, image_width = depth_m.shape[:2]
    window_result = build_neck_windows(keypoints, image_width, image_height, cfg)
    if window_result.status != STATUS_OK or window_result.center_window is None:
        return _config_failure(
            window_result.reason or REASON_NECK_WINDOW_OUT_OF_BOUNDS,
            window_result.message,
            cfg,
            window_result=window_result,
            keypoints=_as_keypoint_map(keypoints),
        )

    center_points, center_pixels = project_depth_window_to_points(depth_m, window_result.center_window, camera_matrix)
    valid_depth_points = int(center_points.shape[0])
    if valid_depth_points < cfg.min_center_points:
        return _config_failure(
            REASON_INSUFFICIENT_CENTER_DEPTH_POINTS,
            "Center window has too few valid depth points.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            keypoints=_as_keypoint_map(keypoints),
        )

    filtered_points, filtered_pixels, depth_outlier_ratio = filter_points_and_pixels_by_depth_band(center_points, center_pixels, cfg.depth_outlier_mm)
    if depth_outlier_ratio > cfg.max_depth_outlier_ratio:
        return _config_failure(
            REASON_DEPTH_OUTLIER_RATIO_TOO_HIGH,
            "Too many center-window depth points are outliers.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            depth_outlier_ratio=depth_outlier_ratio,
            keypoints=_as_keypoint_map(keypoints),
        )
    if filtered_points.shape[0] < cfg.min_center_points:
        return _config_failure(
            REASON_INSUFFICIENT_CENTER_DEPTH_POINTS,
            "Too few center-window points remain after depth filtering.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            depth_outlier_ratio=depth_outlier_ratio,
            keypoints=_as_keypoint_map(keypoints),
        )

    coarse_surface_point = median_surface_point(filtered_points)
    coarse_patch_points, coarse_patch_pixels = extract_local_patch_with_pixels(filtered_points, filtered_pixels, coarse_surface_point, cfg.patch_radius_mm)
    if coarse_patch_points.shape[0] < cfg.min_patch_points:
        return _config_failure(
            REASON_INSUFFICIENT_PATCH_POINTS,
            "Coarse local surface patch has too few points.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            coarse_patch_points=int(coarse_patch_points.shape[0]),
            patch_points=int(coarse_patch_points.shape[0]),
            depth_outlier_ratio=depth_outlier_ratio,
            coarse_surface_point_camera_m=[float(value) for value in coarse_surface_point.tolist()],
            keypoints=_as_keypoint_map(keypoints),
        )

    try:
        raw_coarse_normal, coarse_plane_rmse_mm = estimate_pca_normal(coarse_patch_points)
        coarse_normal = orient_normal_toward_camera(raw_coarse_normal, coarse_surface_point)
    except ValueError as exc:
        return _config_failure(
            REASON_NORMAL_DIRECTION_INVALID,
            str(exc),
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            coarse_patch_points=int(coarse_patch_points.shape[0]),
            patch_points=int(coarse_patch_points.shape[0]),
            depth_outlier_ratio=depth_outlier_ratio,
            coarse_surface_point_camera_m=[float(value) for value in coarse_surface_point.tolist()],
            keypoints=_as_keypoint_map(keypoints),
        )
    if coarse_plane_rmse_mm > cfg.max_plane_rmse_mm:
        return _config_failure(
            REASON_PLANE_FIT_RMSE_TOO_HIGH,
            "Coarse local surface patch plane residual is too high.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            coarse_patch_points=int(coarse_patch_points.shape[0]),
            coarse_plane_rmse_mm=coarse_plane_rmse_mm,
            patch_points=int(coarse_patch_points.shape[0]),
            plane_rmse_mm=coarse_plane_rmse_mm,
            depth_outlier_ratio=depth_outlier_ratio,
            coarse_surface_point_camera_m=[float(value) for value in coarse_surface_point.tolist()],
            coarse_surface_normal_camera=[float(value) for value in coarse_normal.tolist()],
            keypoints=_as_keypoint_map(keypoints),
        )

    left_shoulder = keypoints["left_shoulder"]
    right_shoulder = keypoints["right_shoulder"]
    left_shoulder_3d = estimate_keypoint_3d(left_shoulder, depth_m, camera_matrix)
    right_shoulder_3d = estimate_keypoint_3d(right_shoulder, depth_m, camera_matrix)
    if left_shoulder_3d is None or right_shoulder_3d is None:
        fx, fy, cx, cy = _camera_intrinsics(camera_matrix)
        z = float(coarse_surface_point[2])
        left_shoulder_3d = np.array([(left_shoulder.x_px - cx) * z / fx, (left_shoulder.y_px - cy) * z / fy, z], dtype=np.float64)
        right_shoulder_3d = np.array([(right_shoulder.x_px - cx) * z / fx, (right_shoulder.y_px - cy) * z / fy, z], dtype=np.float64)
    shoulder_vec = np.asarray(right_shoulder_3d, dtype=np.float64) - np.asarray(left_shoulder_3d, dtype=np.float64)
    shoulder_center_3d = (np.asarray(left_shoulder_3d, dtype=np.float64) + np.asarray(right_shoulder_3d, dtype=np.float64)) / 2.0
    shoulder_kwargs = {
        "left_shoulder_camera_m": _as_float_list(left_shoulder_3d),
        "right_shoulder_camera_m": _as_float_list(right_shoulder_3d),
        "shoulder_center_camera_m": _as_float_list(shoulder_center_3d),
    }
    try:
        coarse_tangent_x = project_tangent_to_plane(shoulder_vec, coarse_normal)
        coarse_rotation = build_surface_rotation(coarse_tangent_x, coarse_normal)
        coarse_target_point = compute_target_point_from_anchor(
            coarse_surface_point,
            coarse_rotation,
            cfg.target_region,
            cfg.lateral_offset_mm,
            cfg.inferior_offset_mm,
        )
    except ValueError as exc:
        return _config_failure(
            REASON_TANGENT_DIRECTION_DEGENERATE,
            str(exc),
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            coarse_patch_points=int(coarse_patch_points.shape[0]),
            coarse_plane_rmse_mm=coarse_plane_rmse_mm,
            patch_points=int(coarse_patch_points.shape[0]),
            plane_rmse_mm=coarse_plane_rmse_mm,
            depth_outlier_ratio=depth_outlier_ratio,
            coarse_surface_point_camera_m=[float(value) for value in coarse_surface_point.tolist()],
            coarse_surface_normal_camera=[float(value) for value in coarse_normal.tolist()],
            keypoints=_as_keypoint_map(keypoints),
            **shoulder_kwargs,
        )

    coarse_kwargs = {
        "coarse_surface_point_camera_m": _as_float_list(coarse_surface_point),
        "coarse_target_point_camera_m": _as_float_list(coarse_target_point),
        "coarse_surface_normal_camera": _as_float_list(coarse_normal),
        "coarse_tangent_x_camera": _as_float_list(coarse_tangent_x),
        "coarse_tangent_y_camera": _as_float_list(np.asarray(coarse_rotation[:, 1], dtype=np.float64)),
        "coarse_rotation_matrix_camera": _as_float_matrix(coarse_rotation),
        "coarse_patch_points": int(coarse_patch_points.shape[0]),
        "coarse_plane_rmse_mm": float(coarse_plane_rmse_mm),
        **shoulder_kwargs,
    }

    if cfg.target_mode == TARGET_MODE_SHOULDER_CENTER:
        return NeckSurfaceEstimate(
            status=STATUS_OK,
            reason=None,
            message="Shoulder center target estimated from pose keypoints.",
            search_window=window_result.search_window,
            center_window=window_result.center_window,
            using_ears=window_result.using_ears,
            surface_point_camera_m=_as_float_list(shoulder_center_3d),
            target_point_camera_m=_as_float_list(shoulder_center_3d),
            surface_normal_camera=_as_float_list(coarse_normal),
            tangent_x_camera=_as_float_list(np.asarray(coarse_rotation[:, 0], dtype=np.float64)),
            tangent_y_camera=_as_float_list(np.asarray(coarse_rotation[:, 1], dtype=np.float64)),
            rotation_matrix_camera=_as_float_matrix(coarse_rotation),
            motion_normal_camera=_as_float_list(coarse_normal),
            patient_left_outward_normal_camera=_as_unit_float_list(-shoulder_vec),
            pca_surface_normal_camera=_as_float_list(coarse_normal),
            pca_rotation_matrix_camera=_as_float_matrix(coarse_rotation),
            refinement_status="skipped",
            refinement_reason=TARGET_MODE_SHOULDER_CENTER,
            refinement_points=0,
            target_mode=cfg.target_mode,
            target_region=cfg.target_region,
            lateral_offset_mm=float(cfg.lateral_offset_mm),
            inferior_offset_mm=float(cfg.inferior_offset_mm),
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            patch_points=int(coarse_patch_points.shape[0]),
            plane_rmse_mm=float(coarse_plane_rmse_mm),
            depth_outlier_ratio=float(depth_outlier_ratio),
            keypoints=_as_keypoint_map(keypoints),
            center_pixels=_sample_pixels(filtered_pixels),
            coarse_patch_pixels=_sample_pixels(coarse_patch_pixels),
            **coarse_kwargs,
        )

    assert window_result.search_window is not None
    search_points, search_pixels = project_depth_window_to_points(depth_m, window_result.search_window, camera_matrix)
    search_filtered_points, search_filtered_pixels, _search_depth_outlier_ratio = filter_points_and_pixels_by_depth_band(
        search_points,
        search_pixels,
        cfg.depth_outlier_mm,
    )
    refinement_points, refinement_pixels = extract_local_patch_with_pixels(
        search_filtered_points,
        search_filtered_pixels,
        coarse_target_point,
        cfg.patch_radius_mm,
    )
    if refinement_points.shape[0] < cfg.min_patch_points:
        return _config_failure(
            REASON_INSUFFICIENT_PATCH_POINTS,
            "Refinement target-side neighborhood has too few points.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            patch_points=0,
            depth_outlier_ratio=depth_outlier_ratio,
            refinement_status=STATUS_FAILED,
            refinement_reason=REASON_INSUFFICIENT_REFINEMENT_POINTS,
            refinement_points=int(refinement_points.shape[0]),
            keypoints=_as_keypoint_map(keypoints),
            center_pixels=_sample_pixels(filtered_pixels),
            coarse_patch_pixels=_sample_pixels(coarse_patch_pixels),
            refinement_pixels=_sample_pixels(refinement_pixels),
            **coarse_kwargs,
        )

    refined_surface_point = median_surface_point(refinement_points)
    final_patch_points, final_patch_pixels = extract_local_patch_with_pixels(
        search_filtered_points,
        search_filtered_pixels,
        refined_surface_point,
        cfg.patch_radius_mm,
    )
    if final_patch_points.shape[0] < cfg.min_patch_points:
        return _config_failure(
            REASON_INSUFFICIENT_PATCH_POINTS,
            "Refined final surface patch has too few points.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            patch_points=int(final_patch_points.shape[0]),
            depth_outlier_ratio=depth_outlier_ratio,
            refinement_status=STATUS_FAILED,
            refinement_reason=REASON_INSUFFICIENT_FINAL_PATCH_POINTS,
            refinement_points=int(refinement_points.shape[0]),
            keypoints=_as_keypoint_map(keypoints),
            center_pixels=_sample_pixels(filtered_pixels),
            coarse_patch_pixels=_sample_pixels(coarse_patch_pixels),
            refinement_pixels=_sample_pixels(refinement_pixels),
            patch_pixels=_sample_pixels(final_patch_pixels),
            **coarse_kwargs,
        )

    try:
        raw_final_normal, plane_rmse_mm = estimate_pca_normal(final_patch_points)
        normal = orient_normal_toward_camera(raw_final_normal, refined_surface_point)
    except ValueError as exc:
        return _config_failure(
            REASON_NORMAL_DIRECTION_INVALID,
            str(exc),
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            patch_points=int(final_patch_points.shape[0]),
            depth_outlier_ratio=depth_outlier_ratio,
            refinement_status=STATUS_FAILED,
            refinement_reason=REASON_NORMAL_DIRECTION_INVALID,
            refinement_points=int(refinement_points.shape[0]),
            keypoints=_as_keypoint_map(keypoints),
            center_pixels=_sample_pixels(filtered_pixels),
            coarse_patch_pixels=_sample_pixels(coarse_patch_pixels),
            refinement_pixels=_sample_pixels(refinement_pixels),
            patch_pixels=_sample_pixels(final_patch_pixels),
            **coarse_kwargs,
        )
    if plane_rmse_mm > cfg.max_plane_rmse_mm:
        return _config_failure(
            REASON_PLANE_FIT_RMSE_TOO_HIGH,
            "Refined final surface patch plane residual is too high.",
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            patch_points=int(final_patch_points.shape[0]),
            plane_rmse_mm=plane_rmse_mm,
            depth_outlier_ratio=depth_outlier_ratio,
            surface_point_camera_m=[float(value) for value in refined_surface_point.tolist()],
            surface_normal_camera=[float(value) for value in normal.tolist()],
            refinement_status=STATUS_FAILED,
            refinement_reason=REASON_PLANE_FIT_RMSE_TOO_HIGH,
            refinement_points=int(refinement_points.shape[0]),
            keypoints=_as_keypoint_map(keypoints),
            center_pixels=_sample_pixels(filtered_pixels),
            coarse_patch_pixels=_sample_pixels(coarse_patch_pixels),
            refinement_pixels=_sample_pixels(refinement_pixels),
            patch_pixels=_sample_pixels(final_patch_pixels),
            **coarse_kwargs,
        )

    try:
        tangent_x = project_tangent_to_plane(shoulder_vec, normal)
        pca_rotation = build_surface_rotation(tangent_x, normal)
        patient_left_outward = -shoulder_vec
    except ValueError as exc:
        return _config_failure(
            REASON_TANGENT_DIRECTION_DEGENERATE,
            str(exc),
            cfg,
            window_result=window_result,
            valid_depth_points=valid_depth_points,
            filtered_center_points=int(filtered_points.shape[0]),
            patch_points=int(final_patch_points.shape[0]),
            plane_rmse_mm=plane_rmse_mm,
            depth_outlier_ratio=depth_outlier_ratio,
            surface_point_camera_m=[float(value) for value in refined_surface_point.tolist()],
            surface_normal_camera=[float(value) for value in normal.tolist()],
            refinement_status=STATUS_FAILED,
            refinement_reason=REASON_TANGENT_DIRECTION_DEGENERATE,
            refinement_points=int(refinement_points.shape[0]),
            keypoints=_as_keypoint_map(keypoints),
            center_pixels=_sample_pixels(filtered_pixels),
            coarse_patch_pixels=_sample_pixels(coarse_patch_pixels),
            refinement_pixels=_sample_pixels(refinement_pixels),
            patch_pixels=_sample_pixels(final_patch_pixels),
            **coarse_kwargs,
        )

    return NeckSurfaceEstimate(
        status=STATUS_OK,
        reason=None,
        message="Neck surface pose estimated from refined target-side patch.",
        search_window=window_result.search_window,
        center_window=window_result.center_window,
        using_ears=window_result.using_ears,
        surface_point_camera_m=[float(value) for value in refined_surface_point.tolist()],
        target_point_camera_m=[float(value) for value in refined_surface_point.tolist()],
        surface_normal_camera=[float(value) for value in normal.tolist()],
        tangent_x_camera=[float(value) for value in np.asarray(pca_rotation[:, 0], dtype=np.float64).tolist()],
        tangent_y_camera=[float(value) for value in np.asarray(pca_rotation[:, 1], dtype=np.float64).tolist()],
        rotation_matrix_camera=[[float(value) for value in row] for row in pca_rotation.tolist()],
        motion_normal_camera=[float(value) for value in normal.tolist()],
        patient_left_outward_normal_camera=_as_unit_float_list(patient_left_outward),
        pca_surface_normal_camera=[float(value) for value in normal.tolist()],
        pca_rotation_matrix_camera=[[float(value) for value in row] for row in pca_rotation.tolist()],
        refinement_status=STATUS_OK,
        refinement_reason=None,
        refinement_points=int(refinement_points.shape[0]),
        target_mode=cfg.target_mode,
        target_region=cfg.target_region,
        lateral_offset_mm=float(cfg.lateral_offset_mm),
        inferior_offset_mm=float(cfg.inferior_offset_mm),
        valid_depth_points=valid_depth_points,
        filtered_center_points=int(filtered_points.shape[0]),
        patch_points=int(final_patch_points.shape[0]),
        plane_rmse_mm=float(plane_rmse_mm),
        depth_outlier_ratio=float(depth_outlier_ratio),
        keypoints=_as_keypoint_map(keypoints),
        center_pixels=_sample_pixels(filtered_pixels),
        patch_pixels=_sample_pixels(final_patch_pixels),
        coarse_patch_pixels=_sample_pixels(coarse_patch_pixels),
        refinement_pixels=_sample_pixels(refinement_pixels),
        **coarse_kwargs,
    )


def _project_point_to_pixel(point_camera_m: list[float] | np.ndarray, camera_matrix: list[list[float]] | np.ndarray) -> tuple[int, int] | None:
    point = np.asarray(point_camera_m, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(point)) or float(point[2]) <= 1e-9:
        return None
    fx, fy, cx, cy = _camera_intrinsics(camera_matrix)
    u = fx * float(point[0]) / float(point[2]) + cx
    v = fy * float(point[1]) / float(point[2]) + cy
    if not math.isfinite(u) or not math.isfinite(v):
        return None
    return int(round(u)), int(round(v))


def extract_local_patch_with_pixels(points: np.ndarray, pixels: np.ndarray, center: np.ndarray, radius_mm: float) -> tuple[np.ndarray, np.ndarray]:
    if points.size == 0:
        return points.reshape(0, 3), np.empty((0, 2), dtype=np.float64)
    radius_m = float(radius_mm) / 1000.0
    distances = np.linalg.norm(points - np.asarray(center, dtype=np.float64).reshape(3), axis=1)
    keep = distances <= radius_m
    return points[keep], np.asarray(pixels, dtype=np.float64)[keep]


def _sample_pixels(pixels: np.ndarray, *, max_points: int = 600) -> list[list[float]]:
    if pixels.size == 0:
        return []
    if pixels.shape[0] > max_points:
        indices = np.linspace(0, pixels.shape[0] - 1, max_points).astype(int)
        pixels = pixels[indices]
    return [[float(x), float(y)] for x, y in np.asarray(pixels, dtype=np.float64).reshape(-1, 2).tolist()]


def draw_debug_overlay(
    image_bgr: np.ndarray,
    estimate: NeckSurfaceEstimate,
    camera_matrix: list[list[float]] | np.ndarray | None = None,
) -> np.ndarray:
    output = image_bgr.copy()
    for name, values in estimate.keypoints.items():
        x, y, confidence = values
        color = (0, 255, 0) if confidence >= 0.5 else (0, 165, 255)
        cv2.circle(output, (int(round(x)), int(round(y))), 4, color, -1)
        cv2.putText(output, name, (int(round(x)) + 4, int(round(y)) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
    if "left_shoulder" in estimate.keypoints and "right_shoulder" in estimate.keypoints:
        ls = estimate.keypoints["left_shoulder"]
        rs = estimate.keypoints["right_shoulder"]
        cv2.line(output, (int(round(ls[0])), int(round(ls[1]))), (int(round(rs[0])), int(round(rs[1]))), (255, 128, 0), 2)
    if estimate.search_window is not None:
        w = estimate.search_window
        cv2.rectangle(output, (w.x, w.y), (w.x2, w.y2), (255, 0, 0), 2)
    if estimate.center_window is not None:
        w = estimate.center_window
        cv2.rectangle(output, (w.x, w.y), (w.x2, w.y2), (0, 255, 255), 2)
    if camera_matrix is not None and estimate.coarse_target_point_camera_m is not None:
        coarse_pixel = _project_point_to_pixel(estimate.coarse_target_point_camera_m, camera_matrix)
        if coarse_pixel is not None:
            cx, cy = coarse_pixel
            height, width = output.shape[:2]
            if 0 <= cx < width and 0 <= cy < height:
                cv2.drawMarker(output, (cx, cy), (0, 180, 255), markerType=cv2.MARKER_DIAMOND, markerSize=18, thickness=2)
                cv2.putText(output, "coarse", (cx + 8, cy + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 255), 1, cv2.LINE_AA)
                if estimate.center_window is not None:
                    wx, wy = estimate.center_window.center
                    cv2.line(output, (int(round(wx)), int(round(wy))), (cx, cy), (0, 180, 255), 1)
    if camera_matrix is not None and estimate.target_point_camera_m is not None:
        target_pixel = _project_point_to_pixel(estimate.target_point_camera_m, camera_matrix)
        if target_pixel is not None:
            tx, ty = target_pixel
            height, width = output.shape[:2]
            if 0 <= tx < width and 0 <= ty < height:
                cv2.circle(output, (tx, ty), 9, (255, 0, 255), 2)
                cv2.drawMarker(output, (tx, ty), (255, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=22, thickness=2)
                cv2.putText(output, estimate.target_region, (tx + 10, ty - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)
                if estimate.coarse_target_point_camera_m is not None:
                    coarse_pixel = _project_point_to_pixel(estimate.coarse_target_point_camera_m, camera_matrix)
                    if coarse_pixel is not None:
                        cv2.line(output, coarse_pixel, (tx, ty), (255, 0, 255), 2)
    label = estimate.status if estimate.status == STATUS_OK else f"{estimate.status}:{estimate.reason}"
    cv2.putText(output, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0) if estimate.ok else (0, 0, 255), 2, cv2.LINE_AA)
    return output


def draw_eval_overlay(
    image_bgr: np.ndarray,
    estimate: NeckSurfaceEstimate,
    camera_matrix: list[list[float]] | np.ndarray,
    eval_payload: dict[str, Any] | None,
) -> np.ndarray:
    output = image_bgr.copy()
    if not eval_payload or eval_payload.get("status") != STATUS_OK:
        reason = eval_payload.get("reason") if eval_payload else "no_eval"
        cv2.putText(output, f"eval:{reason}", (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)
        return output

    marker = eval_payload.get("marker_pose_camera_m")
    target = eval_payload.get("markerless_target_camera_m") or estimate.target_point_camera_m
    marker_pixel = _project_point_to_pixel(marker, camera_matrix) if marker is not None else None
    target_pixel = _project_point_to_pixel(target, camera_matrix) if target is not None else None
    height, width = output.shape[:2]
    if target_pixel is not None:
        tx, ty = target_pixel
        if 0 <= tx < width and 0 <= ty < height:
            cv2.drawMarker(output, (tx, ty), (255, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=28, thickness=2)
            cv2.putText(output, "markerless_target", (tx + 10, ty - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)
    if marker_pixel is not None:
        mx, my = marker_pixel
        if 0 <= mx < width and 0 <= my < height:
            cv2.drawMarker(output, (mx, my), (255, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=28, thickness=2)
            cv2.circle(output, (mx, my), 10, (255, 255, 0), 2)
            cv2.putText(output, "marker_center", (mx + 10, my + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)
            if target_pixel is not None:
                cv2.line(output, (mx, my), target_pixel, (255, 255, 0), 2)
    _draw_normal_arrow(output, camera_matrix, target, eval_payload.get("markerless_normal_camera"), (255, 0, 255), "markerless_n")
    _draw_normal_arrow(output, camera_matrix, marker, eval_payload.get("marker_normal_camera"), (255, 255, 0), "marker_n")
    pos_err = eval_payload.get("error_norm_mm")
    normal_err = eval_payload.get("normal_angle_error_deg")
    label = f"pos {float(pos_err):.1f} mm" if pos_err is not None else "pos n/a"
    if normal_err is not None:
        label += f"  normal {float(normal_err):.1f} deg"
    cv2.putText(output, label, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
    return output


def draw_surface_overlay(
    image_bgr: np.ndarray,
    estimate: NeckSurfaceEstimate,
    camera_matrix: list[list[float]] | np.ndarray | None = None,
) -> np.ndarray:
    output = draw_debug_overlay(image_bgr, estimate, camera_matrix)
    for pixel in estimate.center_pixels:
        cv2.circle(output, (int(round(pixel[0])), int(round(pixel[1]))), 1, (80, 80, 80), -1)
    for pixel in estimate.coarse_patch_pixels:
        cv2.circle(output, (int(round(pixel[0])), int(round(pixel[1]))), 1, (0, 180, 255), -1)
    for pixel in estimate.refinement_pixels:
        cv2.circle(output, (int(round(pixel[0])), int(round(pixel[1]))), 1, (255, 128, 255), -1)
    for pixel in estimate.patch_pixels:
        cv2.circle(output, (int(round(pixel[0])), int(round(pixel[1]))), 1, (0, 255, 0), -1)
    if camera_matrix is not None and estimate.coarse_surface_point_camera_m is not None and estimate.coarse_rotation_matrix_camera is not None:
        _draw_plane_grid(output, camera_matrix, estimate.coarse_surface_point_camera_m, estimate.coarse_rotation_matrix_camera, (0, 150, 255))
        _draw_normal_arrow(output, camera_matrix, estimate.coarse_surface_point_camera_m, estimate.coarse_surface_normal_camera, (0, 180, 255), "coarse_n")
    if camera_matrix is not None and estimate.surface_point_camera_m is not None and estimate.rotation_matrix_camera is not None:
        _draw_plane_grid(output, camera_matrix, estimate.surface_point_camera_m, estimate.rotation_matrix_camera, (0, 180, 0))
        _draw_normal_arrow(output, camera_matrix, estimate.surface_point_camera_m, estimate.motion_normal_camera, (255, 0, 255), "motion_n")
        _draw_normal_arrow(output, camera_matrix, estimate.surface_point_camera_m, estimate.pca_surface_normal_camera or estimate.surface_normal_camera, (0, 0, 255), "pca_n")
    text = f"patch {estimate.patch_points}  rmse {estimate.plane_rmse_mm:.2f} mm" if estimate.plane_rmse_mm is not None else f"patch {estimate.patch_points}  rmse n/a"
    if estimate.depth_outlier_ratio is not None:
        text += f"  outliers {estimate.depth_outlier_ratio:.2f}"
    cv2.putText(output, text, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
    return output


def _draw_normal_arrow(output: np.ndarray, camera_matrix: list[list[float]] | np.ndarray, origin: Any, normal: Any, color: tuple[int, int, int], label: str) -> None:
    if origin is None or normal is None:
        return
    origin_vec = np.asarray(origin, dtype=np.float64).reshape(3)
    normal_vec = np.asarray(normal, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(normal_vec))
    if norm <= 1e-12:
        return
    end_vec = origin_vec + (normal_vec / norm) * 0.08
    start = _project_point_to_pixel(origin_vec, camera_matrix)
    end = _project_point_to_pixel(end_vec, camera_matrix)
    if start is None or end is None:
        return
    cv2.arrowedLine(output, start, end, color, 2, tipLength=0.2)
    cv2.putText(output, label, (end[0] + 6, end[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)


def _draw_plane_grid(
    output: np.ndarray,
    camera_matrix: list[list[float]] | np.ndarray,
    surface_point: Any,
    rotation_matrix: Any,
    color: tuple[int, int, int] = (0, 180, 0),
) -> None:
    center = np.asarray(surface_point, dtype=np.float64).reshape(3)
    rotation = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    x_axis = rotation[:, 0]
    y_axis = rotation[:, 1]
    extent = 0.04
    steps = [-extent, -extent / 2.0, 0.0, extent / 2.0, extent]
    for offset in steps:
        points_a = [center + x_axis * t + y_axis * offset for t in (-extent, extent)]
        points_b = [center + x_axis * offset + y_axis * t for t in (-extent, extent)]
        for segment in (points_a, points_b):
            p0 = _project_point_to_pixel(segment[0], camera_matrix)
            p1 = _project_point_to_pixel(segment[1], camera_matrix)
            if p0 is not None and p1 is not None:
                cv2.line(output, p0, p1, color, 1)


class MediaPipePoseBackend:
    def __init__(self, *, min_detection_confidence: float = 0.5, min_tracking_confidence: float = 0.5) -> None:
        try:
            import mediapipe as mp  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("MediaPipe is required for MediaPipePoseBackend. Install mediapipe in the active Python environment.") from exc
        self._mp = mp
        self._pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=float(min_detection_confidence),
            min_tracking_confidence=float(min_tracking_confidence),
        )

    def detect(self, image_bgr: np.ndarray) -> dict[str, PoseKeypoint]:
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        results = self._pose.process(image_rgb)
        if not results.pose_landmarks:
            return {}
        landmarks = results.pose_landmarks.landmark
        pose_module = self._mp.solutions.pose
        mapping = {
            "nose": pose_module.PoseLandmark.NOSE,
            "left_ear": pose_module.PoseLandmark.LEFT_EAR,
            "right_ear": pose_module.PoseLandmark.RIGHT_EAR,
            "left_shoulder": pose_module.PoseLandmark.LEFT_SHOULDER,
            "right_shoulder": pose_module.PoseLandmark.RIGHT_SHOULDER,
        }
        height, width = image_bgr.shape[:2]
        keypoints: dict[str, PoseKeypoint] = {}
        for name, landmark_id in mapping.items():
            landmark = landmarks[int(landmark_id)]
            confidence = float(getattr(landmark, "visibility", 1.0))
            keypoints[name] = PoseKeypoint(
                name=name,
                x_px=float(landmark.x) * float(width),
                y_px=float(landmark.y) * float(height),
                confidence=confidence,
            )
        return keypoints

    def close(self) -> None:
        self._pose.close()


def write_logging_artifacts(
    output_dir: str | Path,
    image_bgr: np.ndarray,
    depth_image: np.ndarray,
    debug_image: np.ndarray,
    estimate: NeckSurfaceEstimate,
    timestamp_label: str,
    extra_metadata: dict[str, Any] | None = None,
    debug_eval_image: np.ndarray | None = None,
    debug_surface_image: np.ndarray | None = None,
    save_depth: bool = False,
) -> Path:
    frame_dir = Path(output_dir) / str(timestamp_label)
    frame_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(frame_dir / "rgb.png"), image_bgr)
    if save_depth:
        np.save(str(frame_dir / "depth.npy"), depth_image)
    cv2.imwrite(str(frame_dir / "debug.png"), debug_image)
    if debug_eval_image is not None:
        cv2.imwrite(str(frame_dir / "debug_eval.png"), debug_eval_image)
    if debug_surface_image is not None:
        cv2.imwrite(str(frame_dir / "debug_surface.png"), debug_surface_image)
    metadata = estimate.status_payload() | {"keypoints": estimate.keypoints}
    if extra_metadata:
        metadata.update(extra_metadata)
    import json

    (frame_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return frame_dir
