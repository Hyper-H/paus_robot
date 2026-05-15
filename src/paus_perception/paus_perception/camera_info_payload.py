from __future__ import annotations

from typing import Any

import numpy as np

from .calibration import CameraCalibration


def camera_calibration_to_camera_info_payload(
    calibration: CameraCalibration,
    *,
    frame_id: str = "camera",
    rgb_camera_count: int | None = None,
    source: str = "dkam_sdk",
    distortion_model: str = "plumb_bob",
) -> dict[str, Any]:
    camera_matrix = np.asarray(calibration.camera_matrix, dtype=np.float64).reshape(3, 3)
    k_values = [float(value) for value in camera_matrix.reshape(-1).tolist()]
    dist_coeffs = [float(value) for value in calibration.dist_coeffs]
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    payload: dict[str, Any] = {
        "width": int(calibration.image_width),
        "height": int(calibration.image_height),
        "distortion_model": str(distortion_model),
        "k": k_values,
        "d": dist_coeffs,
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        "frame_id": str(frame_id),
        "source": str(source),
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
    }
    if rgb_camera_count is not None:
        payload["rgb_camera_count"] = int(rgb_camera_count)
    return payload


def camera_info_payload_summary(payload: dict[str, Any]) -> dict[str, Any]:
    k_values = payload.get("k", [])
    fx = payload.get("fx")
    fy = payload.get("fy")
    cx = payload.get("cx")
    cy = payload.get("cy")
    if isinstance(k_values, list) and len(k_values) == 9:
        fx = float(k_values[0])
        fy = float(k_values[4])
        cx = float(k_values[2])
        cy = float(k_values[5])
    return {
        "width": int(payload.get("width", 0) or 0),
        "height": int(payload.get("height", 0) or 0),
        "rgb_camera_count": payload.get("rgb_camera_count"),
        "fx": float(fx) if fx is not None else None,
        "fy": float(fy) if fy is not None else None,
        "cx": float(cx) if cx is not None else None,
        "cy": float(cy) if cy is not None else None,
        "distortion_model": str(payload.get("distortion_model", "")),
        "source": str(payload.get("source", "")),
    }
