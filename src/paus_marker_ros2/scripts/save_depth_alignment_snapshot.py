from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


def _stamp_to_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _image_to_bgr(message: Image) -> np.ndarray:
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding == "bgr8":
        return data.reshape((message.height, message.width, 3)).copy()
    if message.encoding == "rgb8":
        rgb = data.reshape((message.height, message.width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    raise ValueError(f"Unsupported RGB encoding: {message.encoding}")


def _image_to_depth_m(message: Image) -> np.ndarray:
    if message.encoding != "32FC1":
        raise ValueError(f"Unsupported depth encoding: {message.encoding}")
    return np.frombuffer(message.data, dtype=np.float32).reshape((message.height, message.width)).copy()


def _colorize_depth(depth_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict[str, float | int]]:
    valid_mask = np.isfinite(depth_m) & (depth_m > 0.0)
    valid_depth = depth_m[valid_mask]
    stats: dict[str, float | int] = {
        "height": int(depth_m.shape[0]),
        "width": int(depth_m.shape[1]),
        "valid_count": int(valid_depth.size),
        "pixel_count": int(depth_m.size),
        "valid_ratio": float(valid_depth.size / max(1, depth_m.size)),
    }
    if valid_depth.size == 0:
        empty = np.zeros((depth_m.shape[0], depth_m.shape[1], 3), dtype=np.uint8)
        return empty, valid_mask, stats

    low = float(np.nanpercentile(valid_depth, 2.0))
    high = float(np.nanpercentile(valid_depth, 98.0))
    if not math.isfinite(low) or not math.isfinite(high) or high <= low:
        low = float(np.nanmin(valid_depth))
        high = float(np.nanmax(valid_depth))
    if high <= low:
        high = low + 1e-6

    normalized = np.zeros(depth_m.shape, dtype=np.uint8)
    clipped = np.clip((depth_m - low) / (high - low), 0.0, 1.0)
    normalized[valid_mask] = (clipped[valid_mask] * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    color[~valid_mask] = (0, 0, 0)

    stats.update(
        {
            "min_m": float(np.nanmin(valid_depth)),
            "p02_m": low,
            "median_m": float(np.nanmedian(valid_depth)),
            "p98_m": high,
            "max_m": float(np.nanmax(valid_depth)),
        }
    )
    return color, valid_mask, stats


def _make_overlay(rgb_bgr: np.ndarray, depth_color_bgr: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    overlay = rgb_bgr.copy()
    blended = cv2.addWeighted(rgb_bgr, 0.55, depth_color_bgr, 0.45, 0.0)
    overlay[valid_mask] = blended[valid_mask]
    return overlay


def _percentile_stats(values: np.ndarray, suffix: str) -> dict[str, float | int]:
    finite_values = np.asarray(values, dtype=np.float64)
    finite_values = finite_values[np.isfinite(finite_values)]
    stats: dict[str, float | int] = {f"count_{suffix}": int(finite_values.size)}
    if finite_values.size == 0:
        return stats
    stats.update(
        {
            f"mean_{suffix}": float(np.mean(finite_values)),
            f"median_{suffix}": float(np.median(finite_values)),
            f"p95_{suffix}": float(np.percentile(finite_values, 95.0)),
            f"max_{suffix}": float(np.max(finite_values)),
        }
    )
    return stats


@dataclass
class BoardGeometryEstimate:
    corners_xy: np.ndarray
    outer_quad_xy: np.ndarray
    center_xy: np.ndarray
    board_mask: np.ndarray


@dataclass
class PlaneModel:
    residuals_m: np.ndarray
    centroid_m: np.ndarray
    normal_xyz: np.ndarray
    inlier_points_m: np.ndarray


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


def _build_board_inner_grid(board_rows: int, board_cols: int) -> np.ndarray:
    yy, xx = np.mgrid[0:int(board_rows), 0:int(board_cols)]
    return np.column_stack((xx.reshape(-1), yy.reshape(-1))).astype(np.float32)


def _estimate_board_geometry(
    corners_xy: np.ndarray,
    board_rows: int,
    board_cols: int,
    image_shape: tuple[int, ...],
) -> BoardGeometryEstimate | None:
    corner_grid = np.asarray(corners_xy, dtype=np.float32).reshape((int(board_rows), int(board_cols), 2))
    object_points = _build_board_inner_grid(board_rows, board_cols)
    homography, _ = cv2.findHomography(object_points, corner_grid.reshape((-1, 2)), method=0)
    if homography is None:
        return None

    outer_board_points = np.array(
        [
            [-0.5, -0.5],
            [float(board_cols) - 0.5, -0.5],
            [float(board_cols) - 0.5, float(board_rows) - 0.5],
            [-0.5, float(board_rows) - 0.5],
        ],
        dtype=np.float32,
    ).reshape((-1, 1, 2))
    board_center_point = np.array(
        [[[0.5 * (float(board_cols) - 1.0), 0.5 * (float(board_rows) - 1.0)]]],
        dtype=np.float32,
    )
    outer_quad_xy = cv2.perspectiveTransform(outer_board_points, homography).reshape((4, 2))
    center_xy = cv2.perspectiveTransform(board_center_point, homography).reshape(2)

    mask = np.zeros(image_shape[:2], dtype=np.uint8)
    quad_i32 = np.round(outer_quad_xy).astype(np.int32)
    quad_i32[:, 0] = np.clip(quad_i32[:, 0], 0, image_shape[1] - 1)
    quad_i32[:, 1] = np.clip(quad_i32[:, 1], 0, image_shape[0] - 1)
    if cv2.contourArea(quad_i32.astype(np.float32)) <= 1.0:
        return None
    cv2.fillConvexPoly(mask, quad_i32, 255)
    return BoardGeometryEstimate(
        corners_xy=corner_grid,
        outer_quad_xy=outer_quad_xy,
        center_xy=center_xy.astype(np.float32),
        board_mask=mask,
    )


def _detect_chessboard_roi(rgb_bgr: np.ndarray, board_rows: int, board_cols: int) -> np.ndarray | None:
    corners_xy = _find_refined_chessboard_corners(rgb_bgr, board_rows, board_cols)
    if corners_xy is None:
        return None
    geometry = _estimate_board_geometry(corners_xy, board_rows, board_cols, rgb_bgr.shape)
    if geometry is None:
        return None
    return geometry.board_mask


def _compute_edge_alignment(
    rgb_bgr: np.ndarray,
    depth_m: np.ndarray,
    valid_mask: np.ndarray,
    *,
    board_rows: int,
    board_cols: int,
    max_distance_px: float,
) -> tuple[dict[str, float | int | str], np.ndarray, np.ndarray]:
    gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    rgb_edges = cv2.Canny(gray, 60, 140)

    valid_depth = depth_m[valid_mask]
    if valid_depth.size == 0:
        empty = np.zeros_like(gray)
        return {"status": "no_valid_depth"}, empty, empty

    low = float(np.nanpercentile(valid_depth, 2.0))
    high = float(np.nanpercentile(valid_depth, 98.0))
    if not math.isfinite(low) or not math.isfinite(high) or high <= low:
        low = float(np.nanmin(valid_depth))
        high = float(np.nanmax(valid_depth))
    if high <= low:
        high = low + 1e-6

    depth_norm = np.zeros(depth_m.shape, dtype=np.uint8)
    depth_clipped = np.clip((depth_m - low) / (high - low), 0.0, 1.0)
    depth_norm[valid_mask] = (depth_clipped[valid_mask] * 255.0).astype(np.uint8)
    depth_norm = cv2.GaussianBlur(depth_norm, (5, 5), 0)
    valid_u8 = valid_mask.astype(np.uint8) * 255
    coarse_valid = cv2.morphologyEx(valid_u8, cv2.MORPH_CLOSE, np.ones((15, 15), dtype=np.uint8), iterations=1)
    coarse_valid = cv2.morphologyEx(coarse_valid, cv2.MORPH_OPEN, np.ones((7, 7), dtype=np.uint8), iterations=1)
    edge_kernel = np.ones((5, 5), dtype=np.uint8)
    stable_valid = cv2.erode(coarse_valid, edge_kernel, iterations=1)
    roi = cv2.dilate(coarse_valid, np.ones((21, 21), dtype=np.uint8), iterations=1)
    board_roi = _detect_chessboard_roi(rgb_bgr, board_rows, board_cols)
    if board_roi is not None:
        roi = cv2.bitwise_and(roi, board_roi)
    depth_edges = cv2.Canny(depth_norm, 25, 80)
    depth_edges = cv2.bitwise_and(depth_edges, stable_valid)
    depth_boundary = cv2.morphologyEx(coarse_valid, cv2.MORPH_GRADIENT, edge_kernel)
    depth_edges = cv2.bitwise_or(depth_edges, depth_boundary)
    depth_edges = cv2.bitwise_and(depth_edges, roi)
    rgb_edges = cv2.bitwise_and(rgb_edges, roi)

    rgb_edge_bool = rgb_edges > 0
    depth_edge_bool = depth_edges > 0
    if not np.any(rgb_edge_bool) or not np.any(depth_edge_bool):
        debug = rgb_bgr.copy()
        return {
            "status": "missing_edges",
            "rgb_edge_count": int(np.count_nonzero(rgb_edge_bool)),
            "depth_edge_count": int(np.count_nonzero(depth_edge_bool)),
        }, debug, np.zeros_like(gray)

    dist_to_rgb = cv2.distanceTransform((~rgb_edge_bool).astype(np.uint8), cv2.DIST_L2, 3)
    dist_to_depth = cv2.distanceTransform((~depth_edge_bool).astype(np.uint8), cv2.DIST_L2, 3)
    depth_to_rgb = dist_to_rgb[depth_edge_bool]
    rgb_to_depth = dist_to_depth[rgb_edge_bool]
    depth_to_rgb = depth_to_rgb[depth_to_rgb <= float(max_distance_px)]
    rgb_to_depth = rgb_to_depth[rgb_to_depth <= float(max_distance_px)]

    stats: dict[str, float | int | str] = {
        "status": "ok",
        "unit": "px",
        "max_distance_px": float(max_distance_px),
        "rgb_edge_count": int(np.count_nonzero(rgb_edge_bool)),
        "depth_edge_count": int(np.count_nonzero(depth_edge_bool)),
        "edge_roi": "chessboard_roi" if board_roi is not None else "depth_valid_support",
        "primary_metric": "rgb_edge_to_nearest_depth_edge",
    }
    stats.update(_percentile_stats(depth_to_rgb, "depth_to_rgb_px"))
    stats.update(_percentile_stats(rgb_to_depth, "rgb_to_depth_px"))

    debug = rgb_bgr.copy()
    debug[rgb_edge_bool] = (0, 255, 0)
    debug[depth_edge_bool] = (0, 0, 255)
    debug[rgb_edge_bool & depth_edge_bool] = (0, 255, 255)
    cv2.putText(debug, "RGB edges=green  depth edges=red  overlap=yellow", (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 2, cv2.LINE_AA)

    heat = np.zeros(gray.shape, dtype=np.uint8)
    if np.any(depth_edge_bool):
        clipped_distance = np.clip(dist_to_rgb, 0.0, float(max_distance_px))
        heat[depth_edge_bool] = (255.0 * clipped_distance[depth_edge_bool] / max(1e-6, float(max_distance_px))).astype(np.uint8)
    heat_color = cv2.applyColorMap(heat, cv2.COLORMAP_TURBO)
    heat_color[~depth_edge_bool] = (0, 0, 0)
    return stats, debug, heat_color


def _load_camera_model(camera_yaml: str | Path) -> tuple[np.ndarray, np.ndarray]:
    path = Path(camera_yaml)
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    camera_payload = payload.get("camera", {})
    camera_matrix = np.asarray(camera_payload["camera_matrix"], dtype=np.float64).reshape(3, 3)
    dist_payload = camera_payload.get("dist_coeffs", [[]])
    if dist_payload and isinstance(dist_payload[0], list):
        dist_coeffs = np.asarray(dist_payload[0], dtype=np.float64).reshape(-1)
    else:
        dist_coeffs = np.asarray(dist_payload, dtype=np.float64).reshape(-1)
    return camera_matrix, dist_coeffs


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
        board_patch = board_mask[y0:y1, x0:x1] > 0
        valid &= board_patch
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
        return (candidate_points - centroid) @ normal, centroid, normal

    residuals, centroid, normal = fit_model(points)
    residual_median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - residual_median)))
    threshold = max(3.0 * 1.4826 * mad, 0.001)
    inliers = np.abs(residuals - residual_median) <= threshold
    if np.count_nonzero(inliers) >= int(min_points) and np.count_nonzero(inliers) < points.shape[0]:
        residuals, centroid, normal = fit_model(points[inliers])
        points = points[inliers]
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


def _compute_marker_comparison(
    *,
    marker_message: PoseStamped | None,
    board_geometry_center_m: np.ndarray | None,
    rgb_stamp_ns: int,
    max_stamp_delta_ns: int,
) -> dict[str, float | int | str | None]:
    if board_geometry_center_m is None:
        return {"status": "skipped", "reason": "board_geometry_center_unavailable"}
    if marker_message is None:
        return {"status": "skipped", "reason": "marker_message_missing"}
    if str(marker_message.header.frame_id or "") != "camera":
        return {
            "status": "skipped",
            "reason": "marker_frame_mismatch",
            "frame_id": str(marker_message.header.frame_id or ""),
        }

    marker_stamp_ns = _stamp_to_ns(marker_message)
    stamp_delta_ns = abs(int(marker_stamp_ns) - int(rgb_stamp_ns))
    if stamp_delta_ns > int(max_stamp_delta_ns):
        return {
            "status": "skipped",
            "reason": "marker_stamp_too_old",
            "marker_stamp_ns": int(marker_stamp_ns),
            "rgb_stamp_ns": int(rgb_stamp_ns),
            "stamp_delta_ms": float(stamp_delta_ns / 1_000_000.0),
        }

    marker_center = np.array(
        [
            float(marker_message.pose.position.x),
            float(marker_message.pose.position.y),
            float(marker_message.pose.position.z),
        ],
        dtype=np.float64,
    )
    delta = marker_center - np.asarray(board_geometry_center_m, dtype=np.float64).reshape(3)
    return {
        "status": "ok",
        "frame_id": "camera",
        "marker_stamp_ns": int(marker_stamp_ns),
        "rgb_stamp_ns": int(rgb_stamp_ns),
        "stamp_delta_ms": float(stamp_delta_ns / 1_000_000.0),
        "marker_center_x_m": float(marker_center[0]),
        "marker_center_y_m": float(marker_center[1]),
        "marker_center_z_m": float(marker_center[2]),
        "board_center_x_m": float(board_geometry_center_m[0]),
        "board_center_y_m": float(board_geometry_center_m[1]),
        "board_center_z_m": float(board_geometry_center_m[2]),
        "delta_x_m": float(delta[0]),
        "delta_y_m": float(delta[1]),
        "delta_z_m": float(delta[2]),
        "position_error_mm": float(np.linalg.norm(delta) * 1000.0),
    }


def _compute_chessboard_plane_metrics(
    rgb_bgr: np.ndarray,
    depth_m: np.ndarray,
    *,
    board_rows: int,
    board_cols: int,
    patch_size: int,
    min_points: int,
    camera_yaml: str | Path,
) -> tuple[dict[str, float | int | str], np.ndarray]:
    debug = rgb_bgr.copy()
    if int(board_rows) <= 0 or int(board_cols) <= 0:
        return {"status": "skipped", "reason": "board_rows_or_cols_not_set"}, debug

    camera_matrix, dist_coeffs = _load_camera_model(camera_yaml)
    corners_xy = _find_refined_chessboard_corners(rgb_bgr, board_rows, board_cols)
    if corners_xy is None:
        return {"status": "not_found", "expected_corners": int(board_rows) * int(board_cols)}, debug
    geometry = _estimate_board_geometry(corners_xy, board_rows, board_cols, rgb_bgr.shape)
    if geometry is None:
        return {"status": "invalid_board_geometry", "expected_corners": int(board_rows) * int(board_cols)}, debug

    residual_std_mm: list[float] = []
    residual_mad_mm: list[float] = []
    patch_point_counts: list[int] = []
    plane_centers_m: list[np.ndarray] = []
    local_patch_models: list[tuple[int, PlaneModel, float, float]] = []
    corner_records: list[dict[str, object]] = []
    valid_corner_count = 0
    local_plane_std_threshold_mm = 1.0
    global_patch_distance_threshold_mm = 2.0
    global_patch_angle_threshold_deg = 10.0

    for corner_index, corner in enumerate(geometry.corners_xy.reshape((-1, 2))):
        points = _backproject_depth_patch(
            depth_m,
            (float(corner[0]), float(corner[1])),
            camera_matrix,
            dist_coeffs,
            patch_size=int(patch_size),
            board_mask=geometry.board_mask,
        )
        patch_point_counts.append(int(points.shape[0]))
        plane_model = _fit_plane_model_m(points, min_points=int(min_points))
        if plane_model is None:
            color = (128, 128, 128)
            corner_records.append({"corner": corner, "color": color, "selected": False})
        else:
            valid_corner_count += 1
            plane_centers_m.append(np.asarray(plane_model.centroid_m, dtype=np.float64))
            residual_mm = plane_model.residuals_m * 1000.0
            std_mm = float(np.std(residual_mm))
            mad_mm = float(1.4826 * np.median(np.abs(residual_mm - np.median(residual_mm))))
            residual_std_mm.append(std_mm)
            residual_mad_mm.append(mad_mm)
            if std_mm <= local_plane_std_threshold_mm:
                local_patch_models.append((corner_index, plane_model, std_mm, mad_mm))
            if std_mm <= 1.0:
                color = (0, 255, 0)
            elif std_mm <= 2.0:
                color = (0, 255, 255)
            else:
                color = (0, 0, 255)
            corner_records.append(
                {
                    "corner": corner,
                    "color": color,
                    "selected": False,
                    "std_mm": std_mm,
                    "mad_mm": mad_mm,
                }
            )

    global_plane_model: PlaneModel | None = None
    selected_patch_indices: set[int] = set()
    board_geometry_center_m: np.ndarray | None = None
    candidate_patch_count = int(len(local_patch_models))
    if local_patch_models:
        candidate_points = np.concatenate([model.inlier_points_m for _, model, _, _ in local_patch_models], axis=0)
        global_plane_model = _fit_plane_model_m(candidate_points, min_points=max(3, int(min_points)))
        if global_plane_model is not None:
            consistent_models: list[tuple[int, PlaneModel]] = []
            for patch_index, plane_model, _std_mm, _mad_mm in local_patch_models:
                centroid_distance_mm = float(
                    abs(np.dot(plane_model.centroid_m - global_plane_model.centroid_m, global_plane_model.normal_xyz)) * 1000.0
                )
                normal_angle_deg = _plane_angle_deg(plane_model.normal_xyz, global_plane_model.normal_xyz)
                if centroid_distance_mm <= global_patch_distance_threshold_mm and normal_angle_deg <= global_patch_angle_threshold_deg:
                    consistent_models.append((patch_index, plane_model))
            if consistent_models:
                selected_patch_indices = {patch_index for patch_index, _ in consistent_models}
                consistent_points = np.concatenate([model.inlier_points_m for _, model in consistent_models], axis=0)
                refined_global_plane = _fit_plane_model_m(consistent_points, min_points=max(3, int(min_points)))
                if refined_global_plane is not None:
                    global_plane_model = refined_global_plane
            else:
                selected_patch_indices = {patch_index for patch_index, _model, _std_mm, _mad_mm in local_patch_models}
            board_geometry_center_m = _ray_plane_intersection_m(
                camera_matrix,
                dist_coeffs,
                (float(geometry.center_xy[0]), float(geometry.center_xy[1])),
                global_plane_model.centroid_m,
                global_plane_model.normal_xyz,
            )

    for record_index, record in enumerate(corner_records):
        corner = np.asarray(record["corner"], dtype=np.float32)
        color = tuple(int(value) for value in record["color"])
        cv2.circle(debug, (int(round(corner[0])), int(round(corner[1]))), 7, color, 2, cv2.LINE_AA)
        if record_index in selected_patch_indices:
            cv2.circle(debug, (int(round(corner[0])), int(round(corner[1]))), 3, (255, 255, 0), -1, cv2.LINE_AA)

    cv2.polylines(debug, [np.round(geometry.outer_quad_xy).astype(np.int32)], True, (255, 255, 0), 2, cv2.LINE_AA)
    cv2.circle(debug, (int(round(geometry.center_xy[0])), int(round(geometry.center_xy[1]))), 8, (255, 0, 255), 2, cv2.LINE_AA)

    stats: dict[str, float | int | str] = {
        "status": "ok" if board_geometry_center_m is not None else ("global_plane_unavailable" if global_plane_model is None else "board_center_projection_failed"),
        "expected_corners": int(board_rows) * int(board_cols),
        "detected_corners": int(geometry.corners_xy.reshape((-1, 2)).shape[0]),
        "valid_corner_patches": int(valid_corner_count),
        "valid_corner_patch_ratio": float(valid_corner_count / max(1, int(geometry.corners_xy.reshape((-1, 2)).shape[0]))),
        "patch_size_px": int(patch_size),
        "min_points_per_patch": int(min_points),
        "unit": "mm",
        "board_mask_area_px": int(np.count_nonzero(geometry.board_mask)),
        "board_center_u_px": float(geometry.center_xy[0]),
        "board_center_v_px": float(geometry.center_xy[1]),
        "global_plane_candidate_patches": candidate_patch_count,
        "global_plane_selected_patches": int(len(selected_patch_indices)),
        "global_plane_point_count": int(global_plane_model.inlier_points_m.shape[0]) if global_plane_model is not None else 0,
        "local_plane_std_threshold_mm": float(local_plane_std_threshold_mm),
        "global_patch_distance_threshold_mm": float(global_patch_distance_threshold_mm),
        "global_patch_angle_threshold_deg": float(global_patch_angle_threshold_deg),
    }
    if board_geometry_center_m is not None:
        stats.update(
            {
                "board_geometry_center_x_m": float(board_geometry_center_m[0]),
                "board_geometry_center_y_m": float(board_geometry_center_m[1]),
                "board_geometry_center_z_m": float(board_geometry_center_m[2]),
            }
        )
    if plane_centers_m:
        plane_center = np.mean(np.stack(plane_centers_m, axis=0), axis=0)
        stats.update(
            {
                "plane_center_x_m": float(plane_center[0]),
                "plane_center_y_m": float(plane_center[1]),
                "plane_center_z_m": float(plane_center[2]),
                "plane_center_count": int(len(plane_centers_m)),
            }
        )
    stats.update(_percentile_stats(np.asarray(patch_point_counts, dtype=np.float64), "patch_points"))
    stats.update(_percentile_stats(np.asarray(residual_std_mm, dtype=np.float64), "plane_residual_std_mm"))
    stats.update(_percentile_stats(np.asarray(residual_mad_mm, dtype=np.float64), "plane_residual_mad_mm"))

    label = "corner plane std: green<=1mm yellow<=2mm red>2mm gray=invalid"
    cv2.putText(debug, label, (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
    if plane_centers_m:
        cv2.putText(
            debug,
            f"center xyz_m=({plane_center[0]:.3f}, {plane_center[1]:.3f}, {plane_center[2]:.3f})",
            (30, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
    if board_geometry_center_m is not None:
        cv2.putText(
            debug,
            f"board center xyz_m=({board_geometry_center_m[0]:.3f}, {board_geometry_center_m[1]:.3f}, {board_geometry_center_m[2]:.3f})",
            (30, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )
    return stats, debug


class SnapshotNode(Node):
    def __init__(self, rgb_topic: str, depth_topic: str, marker_topic: str | None) -> None:
        super().__init__("depth_alignment_snapshot")
        self.rgb_message: Image | None = None
        self.depth_message: Image | None = None
        self.marker_message: PoseStamped | None = None
        self.create_subscription(Image, rgb_topic, self._on_rgb, 10)
        self.create_subscription(Image, depth_topic, self._on_depth, 10)
        if marker_topic:
            self.create_subscription(PoseStamped, marker_topic, self._on_marker, 10)

    def _on_rgb(self, message: Image) -> None:
        self.rgb_message = message

    def _on_depth(self, message: Image) -> None:
        self.depth_message = message

    def _on_marker(self, message: PoseStamped) -> None:
        self.marker_message = message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Save one RGB/depth alignment snapshot from ROS topics.")
    parser.add_argument("--rgb-topic", default="/camera/image_bridge")
    parser.add_argument("--depth-topic", default="/camera/depth_aligned")
    parser.add_argument("--output-dir", default="results/depth_alignment_snapshot")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--max-stamp-delta-ms", type=float, default=100.0)
    parser.add_argument("--camera-yaml", default="/tmp/paus_robot/camera.yaml")
    parser.add_argument("--marker-topic", default="/marker_pose", help="Optional marker pose topic for center comparison. Use empty string to disable.")
    parser.add_argument("--board-rows", type=int, default=0, help="Chessboard inner-corner rows. Set to enable plane metrics.")
    parser.add_argument("--board-cols", type=int, default=0, help="Chessboard inner-corner columns. Set to enable plane metrics.")
    parser.add_argument("--corner-patch-size", type=int, default=5, help="Odd local patch size around each subpixel corner.")
    parser.add_argument("--corner-min-points", type=int, default=8, help="Minimum valid depth pixels needed to fit a local plane.")
    parser.add_argument("--edge-max-distance-px", type=float, default=30.0)
    parser.add_argument("--marker-max-stamp-delta-ms", type=float, default=100.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = SnapshotNode(args.rgb_topic, args.depth_topic, args.marker_topic.strip() or None)
    deadline = time.monotonic() + float(args.timeout_sec)
    max_delta_ns = int(float(args.max_stamp_delta_ms) * 1_000_000.0)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.rgb_message is None or node.depth_message is None:
                continue
            delta_ns = abs(_stamp_to_ns(node.rgb_message) - _stamp_to_ns(node.depth_message))
            if delta_ns <= max_delta_ns:
                break
        if node.rgb_message is None or node.depth_message is None:
            raise TimeoutError("Timed out waiting for RGB and depth frames.")

        rgb_bgr = _image_to_bgr(node.rgb_message)
        depth_m = _image_to_depth_m(node.depth_message)
        if rgb_bgr.shape[:2] != depth_m.shape:
            raise ValueError(f"RGB/depth shape mismatch: rgb={rgb_bgr.shape[:2]} depth={depth_m.shape}")

        depth_color_bgr, valid_mask, stats = _colorize_depth(depth_m)
        overlay_bgr = _make_overlay(rgb_bgr, depth_color_bgr, valid_mask)
        valid_mask_png = (valid_mask.astype(np.uint8) * 255)
        edge_stats, edge_debug_bgr, edge_distance_bgr = _compute_edge_alignment(
            rgb_bgr,
            depth_m,
            valid_mask,
            board_rows=args.board_rows,
            board_cols=args.board_cols,
            max_distance_px=args.edge_max_distance_px,
        )
        chessboard_stats, chessboard_debug_bgr = _compute_chessboard_plane_metrics(
            rgb_bgr,
            depth_m,
            board_rows=args.board_rows,
            board_cols=args.board_cols,
            patch_size=args.corner_patch_size,
            min_points=args.corner_min_points,
            camera_yaml=args.camera_yaml,
        )
        marker_comparison = _compute_marker_comparison(
            marker_message=node.marker_message,
            board_geometry_center_m=(
                np.array(
                    [
                        float(chessboard_stats["board_geometry_center_x_m"]),
                        float(chessboard_stats["board_geometry_center_y_m"]),
                        float(chessboard_stats["board_geometry_center_z_m"]),
                    ],
                    dtype=np.float64,
                )
                if "board_geometry_center_x_m" in chessboard_stats
                else None
            ),
            rgb_stamp_ns=_stamp_to_ns(node.rgb_message),
            max_stamp_delta_ns=int(float(args.marker_max_stamp_delta_ms) * 1_000_000.0),
        )

        output_dir = Path(args.output_dir) / time.strftime("%Y%m%d_%H%M%S")
        output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_dir / "rgb.png"), rgb_bgr)
        cv2.imwrite(str(output_dir / "depth_color.png"), depth_color_bgr)
        cv2.imwrite(str(output_dir / "depth_overlay.png"), overlay_bgr)
        cv2.imwrite(str(output_dir / "depth_valid_mask.png"), valid_mask_png)
        cv2.imwrite(str(output_dir / "edge_alignment_debug.png"), edge_debug_bgr)
        cv2.imwrite(str(output_dir / "edge_distance_heatmap.png"), edge_distance_bgr)
        cv2.imwrite(str(output_dir / "chessboard_plane_debug.png"), chessboard_debug_bgr)

        stats.update(
            {
                "rgb_topic": args.rgb_topic,
                "depth_topic": args.depth_topic,
                "rgb_stamp_ns": _stamp_to_ns(node.rgb_message),
                "depth_stamp_ns": _stamp_to_ns(node.depth_message),
                "stamp_delta_ms": abs(_stamp_to_ns(node.rgb_message) - _stamp_to_ns(node.depth_message)) / 1_000_000.0,
                "output_dir": str(output_dir),
                "edge_alignment": edge_stats,
                "chessboard_plane": chessboard_stats,
                "marker_comparison": marker_comparison,
                "files": {
                    "rgb": str(output_dir / "rgb.png"),
                    "depth_color": str(output_dir / "depth_color.png"),
                    "depth_overlay": str(output_dir / "depth_overlay.png"),
                    "depth_valid_mask": str(output_dir / "depth_valid_mask.png"),
                    "edge_alignment_debug": str(output_dir / "edge_alignment_debug.png"),
                    "edge_distance_heatmap": str(output_dir / "edge_distance_heatmap.png"),
                    "chessboard_plane_debug": str(output_dir / "chessboard_plane_debug.png"),
                },
            }
        )
        (output_dir / "stats.json").write_text(json.dumps(stats, indent=2, sort_keys=True), encoding="utf-8")
        print(json.dumps(stats, ensure_ascii=False, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
