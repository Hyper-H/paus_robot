from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import cv2
import numpy as np

from paus_perception import make_transform_matrix


RGB_PNP_MODE = "rgb_pnp"
DEPTH_ALIGNED_MODE = "depth_aligned"
HYBRID_COMPARE_MODE = "hybrid_compare"
SUPPORTED_OBSERVATION_MODES = (RGB_PNP_MODE, DEPTH_ALIGNED_MODE, HYBRID_COMPARE_MODE)


@dataclass(frozen=True)
class DepthObservationQualityConfig:
    corner_patch_size_px: int = 5
    corner_min_points: int = 8
    max_rgb_depth_delta_ms: float = 100.0
    min_valid_corner_patch_ratio: float = 0.80
    max_local_plane_residual_median_mm: float = 1.0
    max_local_plane_residual_p95_mm: float = 2.0
    min_global_plane_selected_ratio: float = 0.70
    max_board_model_fit_rmse_mm: float = 2.0
    max_board_model_fit_max_mm: float = 5.0
    min_board_center_z_m: float = 0.20
    max_board_center_z_m: float = 0.80
    global_patch_distance_threshold_mm: float = 2.0
    global_patch_angle_threshold_deg: float = 10.0


@dataclass
class PlaneModel:
    residuals_m: np.ndarray
    centroid_m: np.ndarray
    normal_xyz: np.ndarray
    inlier_points_m: np.ndarray


@dataclass
class BoardGeometryEstimate:
    corners_xy: np.ndarray
    center_xy: np.ndarray
    board_mask: np.ndarray


@dataclass
class BoardPoseEstimate:
    camera_to_board_matrix: np.ndarray | None
    quality: dict[str, Any]
    corners_xy: np.ndarray | None = None


def normalize_observation_mode(value: str) -> str:
    mode = str(value or RGB_PNP_MODE).strip().lower()
    if mode not in SUPPORTED_OBSERVATION_MODES:
        raise ValueError(f"Unsupported observation_mode: {value!r}")
    return mode


def rotation_error_deg(lhs_rotation: np.ndarray, rhs_rotation: np.ndarray) -> float:
    delta = np.asarray(lhs_rotation, dtype=np.float64).reshape(3, 3).T @ np.asarray(rhs_rotation, dtype=np.float64).reshape(3, 3)
    cosine = (float(np.trace(delta)) - 1.0) * 0.5
    return float(np.rad2deg(np.arccos(np.clip(cosine, -1.0, 1.0))))


def build_board_object_points(board_rows: int, board_cols: int, square_size_m: float) -> np.ndarray:
    object_points = np.zeros((int(board_rows) * int(board_cols), 3), dtype=np.float64)
    object_points[:, :2] = np.mgrid[0:int(board_cols), 0:int(board_rows)].T.reshape(-1, 2)
    object_points *= float(square_size_m)
    return object_points


def _status_quality(observation_mode: str, accepted: bool, reject_reason: str | None = None, **fields: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "observation_mode": observation_mode,
        "accepted": bool(accepted),
        "status": "accepted" if accepted else "rejected",
    }
    if reject_reason:
        payload["reject_reason"] = reject_reason
    payload.update(fields)
    return payload


def _find_refined_chessboard_corners(rgb_bgr: np.ndarray, board_rows: int, board_cols: int) -> np.ndarray | None:
    if int(board_rows) <= 0 or int(board_cols) <= 0:
        return None
    gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        (int(board_cols), int(board_rows)),
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return refined.reshape((int(board_rows), int(board_cols), 2)).astype(np.float32)


def _estimate_board_geometry(corners_xy: np.ndarray, board_rows: int, board_cols: int, image_shape: tuple[int, ...]) -> BoardGeometryEstimate | None:
    corner_grid = np.asarray(corners_xy, dtype=np.float32).reshape((int(board_rows), int(board_cols), 2))
    yy, xx = np.mgrid[0:int(board_rows), 0:int(board_cols)]
    object_grid = np.column_stack((xx.reshape(-1), yy.reshape(-1))).astype(np.float32)
    homography, _ = cv2.findHomography(object_grid, corner_grid.reshape((-1, 2)), method=0)
    if homography is None:
        return None

    center_grid = np.array([[[0.5 * (float(board_cols) - 1.0), 0.5 * (float(board_rows) - 1.0)]]], dtype=np.float32)
    center_xy = cv2.perspectiveTransform(center_grid, homography).reshape(2)
    outer_points = np.array(
        [
            [-0.5, -0.5],
            [float(board_cols) - 0.5, -0.5],
            [float(board_cols) - 0.5, float(board_rows) - 0.5],
            [-0.5, float(board_rows) - 0.5],
        ],
        dtype=np.float32,
    ).reshape((-1, 1, 2))
    outer_quad = cv2.perspectiveTransform(outer_points, homography).reshape((4, 2))

    height = int(image_shape[0])
    width = int(image_shape[1])
    mask = np.zeros((height, width), dtype=np.uint8)
    quad_i32 = np.round(outer_quad).astype(np.int32)
    quad_i32[:, 0] = np.clip(quad_i32[:, 0], 0, width - 1)
    quad_i32[:, 1] = np.clip(quad_i32[:, 1], 0, height - 1)
    if cv2.contourArea(quad_i32.astype(np.float32)) <= 1.0:
        return None
    cv2.fillConvexPoly(mask, quad_i32, 255)
    return BoardGeometryEstimate(corners_xy=corner_grid, center_xy=center_xy.astype(np.float32), board_mask=mask)


def estimate_rgb_pnp_board_pose(
    rgb_bgr: np.ndarray,
    *,
    board_rows: int,
    board_cols: int,
    square_size_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> BoardPoseEstimate:
    expected_corners = int(board_rows) * int(board_cols)
    corners_xy = _find_refined_chessboard_corners(rgb_bgr, board_rows, board_cols)
    if corners_xy is None:
        quality = _status_quality(
            RGB_PNP_MODE,
            False,
            "chessboard_not_found",
            expected_corners=expected_corners,
            detected_corners=0,
        )
        return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality)

    object_points = build_board_object_points(board_rows, board_cols, square_size_m)
    image_points = corners_xy.reshape((-1, 1, 2)).astype(np.float32)
    success, rvec, tvec = cv2.solvePnP(object_points.astype(np.float64), image_points, camera_matrix, dist_coeffs)
    if not success:
        quality = _status_quality(
            RGB_PNP_MODE,
            False,
            "solvepnp_failed",
            expected_corners=expected_corners,
            detected_corners=int(image_points.shape[0]),
        )
        return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality, corners_xy=corners_xy)

    projected, _ = cv2.projectPoints(object_points.astype(np.float64), rvec, tvec, camera_matrix, dist_coeffs)
    errors_px = np.linalg.norm(projected.reshape((-1, 2)) - image_points.reshape((-1, 2)), axis=1)
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    quality = _status_quality(
        RGB_PNP_MODE,
        True,
        expected_corners=expected_corners,
        detected_corners=int(image_points.shape[0]),
        reprojection_rms_px=float(np.sqrt(np.mean(np.square(errors_px)))),
        reprojection_mean_px=float(np.mean(errors_px)),
        reprojection_max_px=float(np.max(errors_px)),
    )
    return BoardPoseEstimate(
        camera_to_board_matrix=make_transform_matrix(tvec.reshape(3), rotation_matrix),
        quality=quality,
        corners_xy=corners_xy,
    )


def _backproject_depth_patch(
    depth_m: np.ndarray,
    center_xy: tuple[float, float],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    *,
    patch_size: int,
    board_mask: np.ndarray | None = None,
) -> np.ndarray:
    half_size = int(patch_size) // 2
    center_x, center_y = center_xy
    x0 = max(0, int(round(center_x)) - half_size)
    x1 = min(depth_m.shape[1], int(round(center_x)) + half_size + 1)
    y0 = max(0, int(round(center_y)) - half_size)
    y1 = min(depth_m.shape[0], int(round(center_y)) + half_size + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = np.isfinite(patch) & (patch > 0.0)
    if board_mask is not None:
        valid &= board_mask[y0:y1, x0:x1] > 0
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    yy, xx = np.nonzero(valid)
    pixels = np.column_stack((xx + x0, yy + y0)).astype(np.float32).reshape((-1, 1, 2))
    normalized = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs).reshape((-1, 2))
    z = patch[valid].astype(np.float64)
    x = normalized[:, 0].astype(np.float64) * z
    y = normalized[:, 1].astype(np.float64) * z
    return np.column_stack((x, y, z)).astype(np.float64)


def _fit_plane_model_m(points_xyz_m: np.ndarray, *, min_points: int) -> PlaneModel | None:
    points = np.asarray(points_xyz_m, dtype=np.float64)
    if points.shape[0] < int(min_points):
        return None

    def fit_model(candidate_points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        centroid = np.mean(candidate_points, axis=0)
        _, _, vh = np.linalg.svd(candidate_points - centroid, full_matrices=False)
        normal = vh[-1]
        normal = normal / max(np.linalg.norm(normal), 1e-12)
        if normal[2] < 0.0:
            normal = -normal
        residuals = (candidate_points - centroid) @ normal
        return residuals, centroid, normal

    residuals, centroid, normal = fit_model(points)
    residual_median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - residual_median)))
    threshold = max(3.0 * 1.4826 * mad, 0.001)
    inliers = np.abs(residuals - residual_median) <= threshold
    if np.count_nonzero(inliers) >= int(min_points) and np.count_nonzero(inliers) < points.shape[0]:
        points = points[inliers]
        residuals, centroid, normal = fit_model(points)
    return PlaneModel(
        residuals_m=np.asarray(residuals, dtype=np.float64),
        centroid_m=np.asarray(centroid, dtype=np.float64),
        normal_xyz=np.asarray(normal, dtype=np.float64),
        inlier_points_m=np.asarray(points, dtype=np.float64),
    )


def _plane_angle_deg(lhs_normal: np.ndarray, rhs_normal: np.ndarray) -> float:
    lhs = np.asarray(lhs_normal, dtype=np.float64).reshape(3)
    rhs = np.asarray(rhs_normal, dtype=np.float64).reshape(3)
    lhs /= max(np.linalg.norm(lhs), 1e-12)
    rhs /= max(np.linalg.norm(rhs), 1e-12)
    cosine = float(np.clip(abs(np.dot(lhs, rhs)), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _ray_plane_intersection_m(
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    pixel_xy: tuple[float, float],
    plane_point_m: np.ndarray,
    plane_normal_xyz: np.ndarray,
) -> np.ndarray | None:
    pixels = np.asarray([[pixel_xy]], dtype=np.float32)
    normalized = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs).reshape(2)
    ray = np.array([float(normalized[0]), float(normalized[1]), 1.0], dtype=np.float64)
    plane_point = np.asarray(plane_point_m, dtype=np.float64).reshape(3)
    plane_normal = np.asarray(plane_normal_xyz, dtype=np.float64).reshape(3)
    denominator = float(np.dot(plane_normal, ray))
    if abs(denominator) <= 1e-12:
        return None
    t = float(np.dot(plane_normal, plane_point) / denominator)
    if t <= 0.0:
        return None
    return ray * t


def _fit_rigid_transform(source_points: np.ndarray, target_points: np.ndarray) -> np.ndarray:
    source = np.asarray(source_points, dtype=np.float64).reshape((-1, 3))
    target = np.asarray(target_points, dtype=np.float64).reshape((-1, 3))
    if source.shape != target.shape or source.shape[0] < 3:
        raise ValueError("Rigid transform fitting requires matching point arrays with at least three points.")
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    covariance = (source - source_centroid).T @ (target - target_centroid)
    u, _, vh = np.linalg.svd(covariance)
    rotation = vh.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vh[-1, :] *= -1.0
        rotation = vh.T @ u.T
    translation = target_centroid - rotation @ source_centroid
    return make_transform_matrix(translation, rotation)


def _transform_points(matrix: np.ndarray, points_xyz: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float64).reshape((-1, 3))
    rotation = np.asarray(matrix[:3, :3], dtype=np.float64)
    translation = np.asarray(matrix[:3, 3], dtype=np.float64)
    return points @ rotation.T + translation


def estimate_depth_aligned_board_pose(
    rgb_bgr: np.ndarray,
    depth_m: np.ndarray,
    *,
    board_rows: int,
    board_cols: int,
    square_size_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    quality_config: DepthObservationQualityConfig | None = None,
    rgb_depth_delta_ms: float | None = None,
) -> BoardPoseEstimate:
    expected_corners = int(board_rows) * int(board_cols)
    if tuple(depth_m.shape[:2]) != tuple(rgb_bgr.shape[:2]):
        quality = _status_quality(
            DEPTH_ALIGNED_MODE,
            False,
            "rgb_depth_shape_mismatch",
            expected_corners=expected_corners,
            detected_corners=0,
            rgb_height=int(rgb_bgr.shape[0]),
            rgb_width=int(rgb_bgr.shape[1]),
            depth_height=int(depth_m.shape[0]),
            depth_width=int(depth_m.shape[1]),
        )
        return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality)

    corners_xy = _find_refined_chessboard_corners(rgb_bgr, board_rows, board_cols)
    if corners_xy is None:
        quality = _status_quality(
            DEPTH_ALIGNED_MODE,
            False,
            "chessboard_not_found",
            expected_corners=expected_corners,
            detected_corners=0,
        )
        return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality)

    return estimate_depth_aligned_board_pose_from_corners(
        corners_xy,
        depth_m,
        board_rows=board_rows,
        board_cols=board_cols,
        square_size_m=square_size_m,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        quality_config=quality_config,
        rgb_depth_delta_ms=rgb_depth_delta_ms,
    )


def estimate_depth_aligned_board_pose_from_corners(
    corners_xy: np.ndarray,
    depth_m: np.ndarray,
    *,
    board_rows: int,
    board_cols: int,
    square_size_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    quality_config: DepthObservationQualityConfig | None = None,
    rgb_depth_delta_ms: float | None = None,
) -> BoardPoseEstimate:
    config = quality_config or DepthObservationQualityConfig()
    expected_corners = int(board_rows) * int(board_cols)
    corners_grid = np.asarray(corners_xy, dtype=np.float32).reshape((int(board_rows), int(board_cols), 2))
    geometry = _estimate_board_geometry(corners_grid, board_rows, board_cols, depth_m.shape)
    if geometry is None:
        quality = _status_quality(
            DEPTH_ALIGNED_MODE,
            False,
            "invalid_board_geometry",
            expected_corners=expected_corners,
            detected_corners=int(corners_grid.reshape((-1, 2)).shape[0]),
        )
        return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality, corners_xy=corners_grid)

    local_models: list[tuple[int, PlaneModel, float, float]] = []
    residual_std_mm: list[float] = []
    residual_mad_mm: list[float] = []
    patch_point_counts: list[int] = []
    valid_corner_count = 0

    for corner_index, corner in enumerate(geometry.corners_xy.reshape((-1, 2))):
        points = _backproject_depth_patch(
            depth_m,
            (float(corner[0]), float(corner[1])),
            np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
            np.asarray(dist_coeffs, dtype=np.float64).reshape(-1),
            patch_size=int(config.corner_patch_size_px),
            board_mask=geometry.board_mask,
        )
        patch_point_counts.append(int(points.shape[0]))
        plane_model = _fit_plane_model_m(points, min_points=int(config.corner_min_points))
        if plane_model is None:
            continue
        valid_corner_count += 1
        residual_mm = np.asarray(plane_model.residuals_m, dtype=np.float64) * 1000.0
        std_mm = float(np.std(residual_mm))
        mad_mm = float(1.4826 * np.median(np.abs(residual_mm - np.median(residual_mm))))
        residual_std_mm.append(std_mm)
        residual_mad_mm.append(mad_mm)
        if std_mm <= float(config.max_local_plane_residual_p95_mm):
            local_models.append((corner_index, plane_model, std_mm, mad_mm))

    candidate_patch_count = int(len(local_models))
    if not local_models:
        quality = _make_depth_quality_payload(
            config,
            accepted=False,
            reject_reason="no_valid_corner_patches",
            expected_corners=expected_corners,
            detected_corners=int(geometry.corners_xy.reshape((-1, 2)).shape[0]),
            valid_corner_count=valid_corner_count,
            candidate_patch_count=candidate_patch_count,
            selected_patch_count=0,
            patch_point_counts=patch_point_counts,
            residual_std_mm=residual_std_mm,
            residual_mad_mm=residual_mad_mm,
            board_mask=geometry.board_mask,
            depth_m=depth_m,
            rgb_depth_delta_ms=rgb_depth_delta_ms,
        )
        return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality, corners_xy=corners_grid)

    candidate_points = np.concatenate([model.inlier_points_m for _, model, _, _ in local_models], axis=0)
    global_plane = _fit_plane_model_m(candidate_points, min_points=max(3, int(config.corner_min_points)))
    if global_plane is None:
        quality = _make_depth_quality_payload(
            config,
            accepted=False,
            reject_reason="global_plane_unavailable",
            expected_corners=expected_corners,
            detected_corners=int(geometry.corners_xy.reshape((-1, 2)).shape[0]),
            valid_corner_count=valid_corner_count,
            candidate_patch_count=candidate_patch_count,
            selected_patch_count=0,
            patch_point_counts=patch_point_counts,
            residual_std_mm=residual_std_mm,
            residual_mad_mm=residual_mad_mm,
            board_mask=geometry.board_mask,
            depth_m=depth_m,
            rgb_depth_delta_ms=rgb_depth_delta_ms,
        )
        return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality, corners_xy=corners_grid)

    selected_models: list[tuple[int, PlaneModel]] = []
    for patch_index, plane_model, _std_mm, _mad_mm in local_models:
        centroid_distance_mm = float(abs(np.dot(plane_model.centroid_m - global_plane.centroid_m, global_plane.normal_xyz)) * 1000.0)
        normal_angle_deg = _plane_angle_deg(plane_model.normal_xyz, global_plane.normal_xyz)
        if (
            centroid_distance_mm <= float(config.global_patch_distance_threshold_mm)
            and normal_angle_deg <= float(config.global_patch_angle_threshold_deg)
        ):
            selected_models.append((patch_index, plane_model))
    if selected_models:
        selected_points = np.concatenate([model.inlier_points_m for _, model in selected_models], axis=0)
        refined_plane = _fit_plane_model_m(selected_points, min_points=max(3, int(config.corner_min_points)))
        if refined_plane is not None:
            global_plane = refined_plane

    observed_points: list[np.ndarray] = []
    for corner in geometry.corners_xy.reshape((-1, 2)):
        point = _ray_plane_intersection_m(
            np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
            np.asarray(dist_coeffs, dtype=np.float64).reshape(-1),
            (float(corner[0]), float(corner[1])),
            global_plane.centroid_m,
            global_plane.normal_xyz,
        )
        if point is None:
            quality = _make_depth_quality_payload(
                config,
                accepted=False,
                reject_reason="corner_ray_plane_intersection_failed",
                expected_corners=expected_corners,
                detected_corners=int(geometry.corners_xy.reshape((-1, 2)).shape[0]),
                valid_corner_count=valid_corner_count,
                candidate_patch_count=candidate_patch_count,
                selected_patch_count=int(len(selected_models)),
                patch_point_counts=patch_point_counts,
                residual_std_mm=residual_std_mm,
                residual_mad_mm=residual_mad_mm,
                board_mask=geometry.board_mask,
                depth_m=depth_m,
                rgb_depth_delta_ms=rgb_depth_delta_ms,
            )
            return BoardPoseEstimate(camera_to_board_matrix=None, quality=quality, corners_xy=corners_grid)
        observed_points.append(point)

    object_points = build_board_object_points(board_rows, board_cols, square_size_m)
    observed_array = np.stack(observed_points, axis=0)
    camera_to_board = _fit_rigid_transform(object_points, observed_array)
    fitted_points = _transform_points(camera_to_board, object_points)
    fit_errors_mm = np.linalg.norm(fitted_points - observed_array, axis=1) * 1000.0
    board_center_model = np.array(
        [0.5 * (float(board_cols) - 1.0) * float(square_size_m), 0.5 * (float(board_rows) - 1.0) * float(square_size_m), 0.0],
        dtype=np.float64,
    )
    board_center_m = _transform_points(camera_to_board, board_center_model.reshape((1, 3)))[0]
    accepted, reject_reason = _depth_quality_decision(
        config,
        valid_corner_count=valid_corner_count,
        expected_corners=expected_corners,
        candidate_patch_count=candidate_patch_count,
        selected_patch_count=int(len(selected_models)),
        residual_std_mm=residual_std_mm,
        fit_errors_mm=fit_errors_mm,
        board_center_z_m=float(board_center_m[2]),
        rgb_depth_delta_ms=rgb_depth_delta_ms,
    )
    quality = _make_depth_quality_payload(
        config,
        accepted=accepted,
        reject_reason=reject_reason,
        expected_corners=expected_corners,
        detected_corners=int(geometry.corners_xy.reshape((-1, 2)).shape[0]),
        valid_corner_count=valid_corner_count,
        candidate_patch_count=candidate_patch_count,
        selected_patch_count=int(len(selected_models)),
        patch_point_counts=patch_point_counts,
        residual_std_mm=residual_std_mm,
        residual_mad_mm=residual_mad_mm,
        board_mask=geometry.board_mask,
        depth_m=depth_m,
        rgb_depth_delta_ms=rgb_depth_delta_ms,
        board_model_fit_errors_mm=fit_errors_mm,
        board_center_m=board_center_m,
        global_plane=global_plane,
    )
    return BoardPoseEstimate(camera_to_board_matrix=camera_to_board if accepted else None, quality=quality, corners_xy=corners_grid)


def _depth_quality_decision(
    config: DepthObservationQualityConfig,
    *,
    valid_corner_count: int,
    expected_corners: int,
    candidate_patch_count: int,
    selected_patch_count: int,
    residual_std_mm: list[float],
    fit_errors_mm: np.ndarray,
    board_center_z_m: float,
    rgb_depth_delta_ms: float | None,
) -> tuple[bool, str | None]:
    if rgb_depth_delta_ms is not None and float(rgb_depth_delta_ms) > float(config.max_rgb_depth_delta_ms):
        return False, "rgb_depth_delta_too_large"
    valid_corner_ratio = float(valid_corner_count / max(1, expected_corners))
    if valid_corner_ratio < float(config.min_valid_corner_patch_ratio):
        return False, "valid_corner_patch_ratio_too_low"
    if not residual_std_mm:
        return False, "no_plane_residuals"
    median_std = float(np.median(np.asarray(residual_std_mm, dtype=np.float64)))
    p95_std = float(np.percentile(np.asarray(residual_std_mm, dtype=np.float64), 95.0))
    if median_std > float(config.max_local_plane_residual_median_mm):
        return False, "local_plane_residual_median_too_high"
    if p95_std > float(config.max_local_plane_residual_p95_mm):
        return False, "local_plane_residual_p95_too_high"
    selected_ratio = float(selected_patch_count / max(1, expected_corners))
    if selected_ratio < float(config.min_global_plane_selected_ratio):
        return False, "global_plane_selected_ratio_too_low"
    if fit_errors_mm.size == 0:
        return False, "board_model_fit_unavailable"
    rmse = float(np.sqrt(np.mean(np.square(fit_errors_mm))))
    max_error = float(np.max(fit_errors_mm))
    if rmse > float(config.max_board_model_fit_rmse_mm):
        return False, "board_model_fit_rmse_too_high"
    if max_error > float(config.max_board_model_fit_max_mm):
        return False, "board_model_fit_max_too_high"
    if board_center_z_m < float(config.min_board_center_z_m) or board_center_z_m > float(config.max_board_center_z_m):
        return False, "board_center_z_out_of_range"
    return True, None


def _percentile_payload(values: list[float] | np.ndarray, prefix: str) -> dict[str, float | int]:
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    payload: dict[str, float | int] = {f"{prefix}_count": int(array.size)}
    if array.size == 0:
        return payload
    payload.update(
        {
            f"{prefix}_mean": float(np.mean(array)),
            f"{prefix}_median": float(np.median(array)),
            f"{prefix}_p95": float(np.percentile(array, 95.0)),
            f"{prefix}_max": float(np.max(array)),
        }
    )
    return payload


def _make_depth_quality_payload(
    config: DepthObservationQualityConfig,
    *,
    accepted: bool,
    reject_reason: str | None,
    expected_corners: int,
    detected_corners: int,
    valid_corner_count: int,
    candidate_patch_count: int,
    selected_patch_count: int,
    patch_point_counts: list[int],
    residual_std_mm: list[float],
    residual_mad_mm: list[float],
    board_mask: np.ndarray,
    depth_m: np.ndarray,
    rgb_depth_delta_ms: float | None,
    board_model_fit_errors_mm: np.ndarray | None = None,
    board_center_m: np.ndarray | None = None,
    global_plane: PlaneModel | None = None,
) -> dict[str, Any]:
    board_mask_bool = np.asarray(board_mask, dtype=np.uint8) > 0
    board_depth = np.asarray(depth_m, dtype=np.float64)[board_mask_bool]
    valid_board_depth = board_depth[np.isfinite(board_depth) & (board_depth > 0.0)]
    payload = _status_quality(
        DEPTH_ALIGNED_MODE,
        accepted,
        reject_reason,
        expected_corners=int(expected_corners),
        detected_corners=int(detected_corners),
        valid_corner_patches=int(valid_corner_count),
        valid_corner_patch_ratio=float(valid_corner_count / max(1, expected_corners)),
        global_plane_candidate_patches=int(candidate_patch_count),
        global_plane_selected_patches=int(selected_patch_count),
        global_plane_selected_ratio=float(selected_patch_count / max(1, expected_corners)),
        patch_size_px=int(config.corner_patch_size_px),
        min_points_per_patch=int(config.corner_min_points),
        valid_depth_ratio=float(valid_board_depth.size / max(1, board_depth.size)),
        board_mask_area_px=int(np.count_nonzero(board_mask_bool)),
        rgb_depth_delta_ms=None if rgb_depth_delta_ms is None else float(rgb_depth_delta_ms),
    )
    payload.update(_percentile_payload(np.asarray(patch_point_counts, dtype=np.float64), "patch_points"))
    payload.update(_percentile_payload(np.asarray(residual_std_mm, dtype=np.float64), "plane_residual_std_mm"))
    payload.update(_percentile_payload(np.asarray(residual_mad_mm, dtype=np.float64), "plane_residual_mad_mm"))
    if board_model_fit_errors_mm is not None:
        errors = np.asarray(board_model_fit_errors_mm, dtype=np.float64)
        payload.update(
            {
                "board_model_fit_rmse_mm": float(np.sqrt(np.mean(np.square(errors)))) if errors.size else math.nan,
                "board_model_fit_mean_mm": float(np.mean(errors)) if errors.size else math.nan,
                "board_model_fit_max_mm": float(np.max(errors)) if errors.size else math.nan,
            }
        )
    if board_center_m is not None:
        center = np.asarray(board_center_m, dtype=np.float64).reshape(3)
        payload.update(
            {
                "board_center_x_m": float(center[0]),
                "board_center_y_m": float(center[1]),
                "board_center_z_m": float(center[2]),
            }
        )
    if global_plane is not None:
        payload.update(
            {
                "global_plane_point_count": int(global_plane.inlier_points_m.shape[0]),
                "global_plane_normal_x": float(global_plane.normal_xyz[0]),
                "global_plane_normal_y": float(global_plane.normal_xyz[1]),
                "global_plane_normal_z": float(global_plane.normal_xyz[2]),
            }
        )
    return payload


def compare_board_pose_estimates(reference_matrix: np.ndarray, candidate_matrix: np.ndarray) -> dict[str, float]:
    reference = np.asarray(reference_matrix, dtype=np.float64).reshape(4, 4)
    candidate = np.asarray(candidate_matrix, dtype=np.float64).reshape(4, 4)
    translation_delta = candidate[:3, 3] - reference[:3, 3]
    return {
        "translation_delta_mm": float(np.linalg.norm(translation_delta) * 1000.0),
        "z_delta_mm": float(abs(translation_delta[2]) * 1000.0),
        "rotation_delta_deg": rotation_error_deg(reference[:3, :3], candidate[:3, :3]),
        "board_normal_delta_deg": _plane_angle_deg(reference[:3, 2], candidate[:3, 2]),
    }
