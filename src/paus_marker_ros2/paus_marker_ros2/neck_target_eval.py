from __future__ import annotations

import math
from typing import Any

DEFAULT_MARKER_POSE_TOPIC = "/marker_pose"
DEFAULT_NECK_STATUS_TOPIC = "/neck_surface_status"
DEFAULT_EVAL_STATUS_TOPIC = "/neck_target_eval_status"
DEFAULT_LOGGING_OUTPUT_DIR = "runtime_logs/neck_eval"
DEFAULT_MAX_PAIR_DELTA_MS = 300.0


class NeckTargetEvalError(ValueError):
    pass


def vector3_from_payload(payload: dict[str, Any], key: str) -> list[float]:
    value = payload.get(key)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise NeckTargetEvalError(f"{key} must be a 3-element list.")
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except (TypeError, ValueError) as exc:
        raise NeckTargetEvalError(f"{key} must contain numeric values.") from exc


def optional_vector3_from_payload(payload: dict[str, Any], key: str) -> list[float] | None:
    value = payload.get(key)
    if value is None:
        return None
    return vector3_from_payload(payload, key)


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pair_delta_ms(first_stamp_ns: int, second_stamp_ns: int) -> float:
    return abs(float(first_stamp_ns - second_stamp_ns)) / 1_000_000.0


def compute_error_mm(markerless_target_camera_m: list[float], marker_pose_camera_m: list[float]) -> tuple[list[float], float]:
    error_xyz_mm = [
        (float(markerless_target_camera_m[index]) - float(marker_pose_camera_m[index])) * 1000.0
        for index in range(3)
    ]
    error_norm_mm = math.sqrt(sum(value * value for value in error_xyz_mm))
    return error_xyz_mm, error_norm_mm


def build_eval_record(
    neck_payload: dict[str, Any],
    marker_pose_camera_m: list[float],
    *,
    neck_stamp_ns: int,
    marker_stamp_ns: int,
    max_pair_delta_ms: float = DEFAULT_MAX_PAIR_DELTA_MS,
) -> dict[str, Any]:
    delta_ms = pair_delta_ms(neck_stamp_ns, marker_stamp_ns)
    if delta_ms > max_pair_delta_ms:
        return {
            "source": "neck_target_eval",
            "status": "waiting",
            "reason": "pair_delta_exceeded",
            "message": "Latest markerless target and marker pose are too far apart in time.",
            "pair_delta_ms": delta_ms,
            "max_pair_delta_ms": float(max_pair_delta_ms),
        }

    if neck_payload.get("status") != "ok":
        return {
            "source": "neck_target_eval",
            "status": "waiting",
            "reason": "markerless_not_ok",
            "message": "Latest markerless status is not ok.",
            "markerless_status": neck_payload.get("status"),
            "markerless_reason": neck_payload.get("reason"),
        }

    try:
        markerless_target = vector3_from_payload(neck_payload, "target_point_camera_m")
    except NeckTargetEvalError as exc:
        return {
            "source": "neck_target_eval",
            "status": "waiting",
            "reason": "missing_markerless_target",
            "message": str(exc),
        }

    marker_pose = [float(value) for value in marker_pose_camera_m]
    error_xyz_mm, error_norm_mm = compute_error_mm(markerless_target, marker_pose)
    return {
        "source": "neck_target_eval",
        "status": "ok",
        "stamp_ns": int(neck_stamp_ns),
        "pair_delta_ms": delta_ms,
        "max_pair_delta_ms": float(max_pair_delta_ms),
        "marker_pose_camera_m": marker_pose,
        "markerless_target_camera_m": markerless_target,
        "neck_anchor_camera_m": optional_vector3_from_payload(neck_payload, "neck_anchor_camera_m"),
        "error_xyz_mm": error_xyz_mm,
        "error_norm_mm": error_norm_mm,
        "target_region": neck_payload.get("target_region"),
        "lateral_offset_mm": neck_payload.get("lateral_offset_mm"),
        "inferior_offset_mm": neck_payload.get("inferior_offset_mm"),
        "plane_rmse_mm": neck_payload.get("plane_rmse_mm"),
        "patch_points": neck_payload.get("patch_points"),
        "rgb_depth_delta_ms": neck_payload.get("rgb_depth_delta_ms"),
    }
