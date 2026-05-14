from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .calibration import CameraCalibration
from .sdk_camera import CameraRuntime, _require_dkam_sdk, capture_raw_frame, read_runtime_calibration
from .transforms import invert_transform_matrix, make_transform_matrix


DEFAULT_POINT_CHANNEL = 1
DEFAULT_RGB_CHANNEL = 2
DEFAULT_RGB_CAMERA_COUNT = 1
DEFAULT_EXTRINSIC_DIRECTION = "factory_rt_inverse"
DEFAULT_SDK_UNIT_SCALE_TO_M = 0.001


@dataclass(frozen=True)
class DepthAlignmentConfig:
    point_channel: int = DEFAULT_POINT_CHANNEL
    rgb_channel: int = DEFAULT_RGB_CHANNEL
    rgb_camera_count: int = DEFAULT_RGB_CAMERA_COUNT
    extrinsic_direction: str = DEFAULT_EXTRINSIC_DIRECTION
    point_unit_scale_to_m: float = DEFAULT_SDK_UNIT_SCALE_TO_M
    extrinsic_translation_scale_to_m: float = DEFAULT_SDK_UNIT_SCALE_TO_M
    min_depth_m: float = 1e-6


@dataclass
class AlignedDepthFrame:
    aligned_xyz_m: np.ndarray
    aligned_depth_m: np.ndarray
    rgb_width: int
    rgb_height: int
    point_width: int
    point_height: int
    source_point_count: int
    valid_source_point_count: int
    projected_point_count: int
    aligned_pixel_count: int
    coverage_ratio: float
    extrinsic_direction: str
    rgb_camera_count: int


def estimate_pointcloud_capture_buffer_size(runtime: CameraRuntime) -> int:
    return int(runtime.rgb_width) * int(runtime.rgb_height) * 3 * 4


def scale_xyz_to_meters(xyz: np.ndarray, scale_to_m: float) -> np.ndarray:
    return np.ascontiguousarray(np.asarray(xyz, dtype=np.float32) * np.float32(scale_to_m), dtype=np.float32)


def _photo_info_value(photo_info: Any, name: str, default: int = 0) -> int:
    return int(getattr(photo_info, name, default) or default)


def convert_pointcloud_raw_to_xyz_meters(
    *,
    sdk: Any,
    camera_obj: Any,
    point_info: Any,
    point_buffer: bytes,
    point_buffer_size: int | None = None,
    unit_scale_to_m: float = DEFAULT_SDK_UNIT_SCALE_TO_M,
) -> np.ndarray:
    point_width = _photo_info_value(point_info, "pixel_width")
    point_height = _photo_info_value(point_info, "pixel_height")
    payload_size = _photo_info_value(point_info, "payload_size")
    if point_width <= 0 or point_height <= 0:
        raise RuntimeError("Point cloud frame has invalid pixel dimensions.")
    convert_buffer_size = int(point_buffer_size if point_buffer_size is not None else len(point_buffer))
    if convert_buffer_size <= 0:
        convert_buffer_size = payload_size
    if convert_buffer_size <= 0:
        raise RuntimeError("Point cloud frame has empty conversion buffer.")

    point_count = point_width * point_height
    xyz = np.empty(point_count * 3, dtype=np.float32)
    status = sdk.Convert3DPointFromCharToFloatCSharp(camera_obj, point_info, point_buffer, convert_buffer_size, xyz)
    if status is not None:
        try:
            if int(status) != 0:
                raise RuntimeError(f"Convert3DPointFromCharToFloatCSharp failed with code {status}.")
        except TypeError:
            pass
    xyz = xyz.reshape((point_height, point_width, 3))
    return scale_xyz_to_meters(xyz, unit_scale_to_m)


def build_factory_extrinsic_matrix(
    calibration: CameraCalibration,
    *,
    extrinsic_direction: str = DEFAULT_EXTRINSIC_DIRECTION,
    translation_scale_to_m: float = DEFAULT_SDK_UNIT_SCALE_TO_M,
) -> np.ndarray:
    if calibration.rotation_matrix is None or calibration.translation_vector is None:
        raise RuntimeError("Factory extrinsic is missing rotation_matrix or translation_vector.")
    rotation = np.asarray(calibration.rotation_matrix, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(calibration.translation_vector, dtype=np.float64).reshape(3) * float(translation_scale_to_m)
    factory_matrix = make_transform_matrix(translation, rotation)
    if extrinsic_direction == "factory_rt":
        return factory_matrix
    if extrinsic_direction == "factory_rt_inverse":
        return invert_transform_matrix(factory_matrix)
    raise ValueError(f"Unsupported extrinsic_direction: {extrinsic_direction}")


def _distortion_coefficients(calibration: CameraCalibration) -> tuple[float, float, float, float, float]:
    coeffs = list(calibration.dist_coeffs or [])
    padded = (coeffs + [0.0] * 5)[:5]
    return tuple(float(value) for value in padded)  # type: ignore[return-value]


def project_xyz_to_rgb_grid(
    xyz_m: np.ndarray,
    calibration: CameraCalibration,
    *,
    config: DepthAlignmentConfig | None = None,
) -> AlignedDepthFrame:
    config = config or DepthAlignmentConfig()
    xyz_array = np.asarray(xyz_m, dtype=np.float32)
    if xyz_array.ndim == 3:
        point_height, point_width = int(xyz_array.shape[0]), int(xyz_array.shape[1])
    else:
        point_height, point_width = 0, 0
    source_points = xyz_array.reshape((-1, 3))
    source_point_count = int(source_points.shape[0])

    rgb_width = int(calibration.image_width)
    rgb_height = int(calibration.image_height)
    aligned_xyz_flat = np.full((rgb_height * rgb_width, 3), np.nan, dtype=np.float32)
    aligned_depth_flat = np.full(rgb_height * rgb_width, np.nan, dtype=np.float32)

    finite_source = np.isfinite(source_points).all(axis=1)
    nonzero_source = np.linalg.norm(source_points, axis=1) > np.float32(1e-9)
    source_mask = finite_source & nonzero_source
    valid_source_point_count = int(np.count_nonzero(source_mask))
    if valid_source_point_count == 0:
        return AlignedDepthFrame(
            aligned_xyz_m=aligned_xyz_flat.reshape((rgb_height, rgb_width, 3)),
            aligned_depth_m=aligned_depth_flat.reshape((rgb_height, rgb_width)),
            rgb_width=rgb_width,
            rgb_height=rgb_height,
            point_width=point_width,
            point_height=point_height,
            source_point_count=source_point_count,
            valid_source_point_count=0,
            projected_point_count=0,
            aligned_pixel_count=0,
            coverage_ratio=0.0,
            extrinsic_direction=config.extrinsic_direction,
            rgb_camera_count=config.rgb_camera_count,
        )

    transform = build_factory_extrinsic_matrix(
        calibration,
        extrinsic_direction=config.extrinsic_direction,
        translation_scale_to_m=config.extrinsic_translation_scale_to_m,
    )
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    points_rgb = source_points[source_mask].astype(np.float64) @ rotation.T + translation

    z = points_rgb[:, 2]
    depth_mask = np.isfinite(points_rgb).all(axis=1) & (z > float(config.min_depth_m))
    if not np.any(depth_mask):
        return AlignedDepthFrame(
            aligned_xyz_m=aligned_xyz_flat.reshape((rgb_height, rgb_width, 3)),
            aligned_depth_m=aligned_depth_flat.reshape((rgb_height, rgb_width)),
            rgb_width=rgb_width,
            rgb_height=rgb_height,
            point_width=point_width,
            point_height=point_height,
            source_point_count=source_point_count,
            valid_source_point_count=valid_source_point_count,
            projected_point_count=0,
            aligned_pixel_count=0,
            coverage_ratio=0.0,
            extrinsic_direction=config.extrinsic_direction,
            rgb_camera_count=config.rgb_camera_count,
        )

    points_rgb = points_rgb[depth_mask]
    z = points_rgb[:, 2]
    normalized_x = points_rgb[:, 0] / z
    normalized_y = points_rgb[:, 1] / z

    k1, k2, p1, p2, k3 = _distortion_coefficients(calibration)
    r2 = normalized_x * normalized_x + normalized_y * normalized_y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x_distorted = normalized_x * radial + 2.0 * p1 * normalized_x * normalized_y + p2 * (r2 + 2.0 * normalized_x * normalized_x)
    y_distorted = normalized_y * radial + p1 * (r2 + 2.0 * normalized_y * normalized_y) + 2.0 * p2 * normalized_x * normalized_y

    camera_matrix = np.asarray(calibration.camera_matrix, dtype=np.float64).reshape(3, 3)
    u_float = camera_matrix[0, 0] * x_distorted + camera_matrix[0, 2]
    v_float = camera_matrix[1, 1] * y_distorted + camera_matrix[1, 2]
    u = np.rint(u_float).astype(np.int64)
    v = np.rint(v_float).astype(np.int64)
    in_bounds = (u >= 0) & (u < rgb_width) & (v >= 0) & (v < rgb_height) & np.isfinite(u_float) & np.isfinite(v_float)
    projected_point_count = int(np.count_nonzero(in_bounds))
    if projected_point_count == 0:
        return AlignedDepthFrame(
            aligned_xyz_m=aligned_xyz_flat.reshape((rgb_height, rgb_width, 3)),
            aligned_depth_m=aligned_depth_flat.reshape((rgb_height, rgb_width)),
            rgb_width=rgb_width,
            rgb_height=rgb_height,
            point_width=point_width,
            point_height=point_height,
            source_point_count=source_point_count,
            valid_source_point_count=valid_source_point_count,
            projected_point_count=0,
            aligned_pixel_count=0,
            coverage_ratio=0.0,
            extrinsic_direction=config.extrinsic_direction,
            rgb_camera_count=config.rgb_camera_count,
        )

    pixel_indices = v[in_bounds] * rgb_width + u[in_bounds]
    candidate_points = points_rgb[in_bounds]
    candidate_depth = candidate_points[:, 2]
    order = np.lexsort((candidate_depth, pixel_indices))
    sorted_pixel_indices = pixel_indices[order]
    unique_first = np.ones(sorted_pixel_indices.shape[0], dtype=bool)
    unique_first[1:] = sorted_pixel_indices[1:] != sorted_pixel_indices[:-1]
    winning_pixel_indices = sorted_pixel_indices[unique_first]
    winning_points = candidate_points[order][unique_first].astype(np.float32)

    aligned_xyz_flat[winning_pixel_indices] = winning_points
    aligned_depth_flat[winning_pixel_indices] = winning_points[:, 2]
    aligned_pixel_count = int(winning_pixel_indices.shape[0])
    coverage_ratio = float(aligned_pixel_count / max(1, rgb_width * rgb_height))

    return AlignedDepthFrame(
        aligned_xyz_m=np.ascontiguousarray(aligned_xyz_flat.reshape((rgb_height, rgb_width, 3)), dtype=np.float32),
        aligned_depth_m=np.ascontiguousarray(aligned_depth_flat.reshape((rgb_height, rgb_width)), dtype=np.float32),
        rgb_width=rgb_width,
        rgb_height=rgb_height,
        point_width=point_width,
        point_height=point_height,
        source_point_count=source_point_count,
        valid_source_point_count=valid_source_point_count,
        projected_point_count=projected_point_count,
        aligned_pixel_count=aligned_pixel_count,
        coverage_ratio=coverage_ratio,
        extrinsic_direction=config.extrinsic_direction,
        rgb_camera_count=config.rgb_camera_count,
    )


def capture_aligned_depth_frame(
    runtime: CameraRuntime,
    *,
    calibration: CameraCalibration | None = None,
    timeout_us: int = 3_000_000,
    config: DepthAlignmentConfig | None = None,
) -> AlignedDepthFrame:
    config = config or DepthAlignmentConfig()
    DkamSDK = _require_dkam_sdk()
    calibration = calibration or read_runtime_calibration(runtime, camera_count=config.rgb_camera_count)
    buffer_size = estimate_pointcloud_capture_buffer_size(runtime)
    point_info, point_buffer = capture_raw_frame(runtime, config.point_channel, buffer_size, timeout_us=timeout_us)
    xyz_m = convert_pointcloud_raw_to_xyz_meters(
        sdk=DkamSDK,
        camera_obj=runtime.camera_obj,
        point_info=point_info,
        point_buffer=point_buffer,
        point_buffer_size=buffer_size,
        unit_scale_to_m=config.point_unit_scale_to_m,
    )
    return project_xyz_to_rgb_grid(xyz_m, calibration, config=config)
