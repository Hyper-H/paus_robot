from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from paus_perception import (
    quaternion_xyzw_to_rotation_matrix,
    rotation_matrix_to_quaternion_xyzw,
    rotation_matrix_to_rpy_deg,
    rpy_deg_to_rotation_matrix,
)


PRE_APPROACH_STAGE = "pre_approach"
REORIENT_STAGE = "reorient"
FINAL_HOVER_STAGE = "final_hover"
SAFE_LIFT_STAGE = "safe_lift"

DEFAULT_REORIENT_TOLERANCE_DEG = 5.0
DEFAULT_MAX_REORIENT_STEP_DEG = 25.0


@dataclass
class ControlDecision:
    check_passed: bool
    error_message: str
    orientation_mode: str
    target_point_base_mm: list[float]
    target_pose_base_mmdeg: list[float]
    raw_target_pose_base_mmdeg: list[float]
    current_tcp_pose_mmdeg: list[float]
    surface_normal_base: list[float] | None
    final_hover_pose_mmdeg: list[float] | None
    pre_approach_pose_mmdeg: list[float] | None
    candidate_pose_mmdeg: list[float] | None
    candidate_stage: str | None
    step_distance_mm: float | None
    distance_to_target_mm: float | None
    clearance_to_plane_mm: float | None


def point_m_to_mm(point_m: list[float]) -> list[float]:
    return [float(value) * 1000.0 for value in point_m]


def is_finite_point(point_xyz: list[float]) -> bool:
    return all(math.isfinite(float(value)) for value in point_xyz)


def _workspace_error(point_xyz_mm: np.ndarray, workspace_min_mm: list[float], workspace_max_mm: list[float]) -> str | None:
    for axis, value, lower, upper in zip(("x", "y", "z"), point_xyz_mm.tolist(), workspace_min_mm, workspace_max_mm):
        if float(value) < float(lower) or float(value) > float(upper):
            return f"Target {axis} is outside workspace."
    return None


def _unit_vector(vector: np.ndarray, error_message: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        raise ValueError(error_message)
    return vector / norm


def _parse_axis_spec(axis_spec: str) -> np.ndarray:
    mapping = {
        "X": np.array([1.0, 0.0, 0.0], dtype=np.float64),
        "+X": np.array([1.0, 0.0, 0.0], dtype=np.float64),
        "-X": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
        "Y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
        "+Y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
        "-Y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
        "Z": np.array([0.0, 0.0, 1.0], dtype=np.float64),
        "+Z": np.array([0.0, 0.0, 1.0], dtype=np.float64),
        "-Z": np.array([0.0, 0.0, -1.0], dtype=np.float64),
    }
    normalized = axis_spec.strip().upper()
    if normalized not in mapping:
        raise ValueError(f"Unsupported flange_face_axis: {axis_spec}")
    return mapping[normalized]


def _choose_local_anchor_axis(face_axis_local: np.ndarray) -> np.ndarray:
    candidates = (
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
        np.array([0.0, 1.0, 0.0], dtype=np.float64),
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
    )
    for candidate in candidates:
        if abs(float(np.dot(candidate, face_axis_local))) < 0.5:
            return candidate
    return np.array([1.0, 0.0, 0.0], dtype=np.float64)


def _project_onto_plane(vector: np.ndarray, plane_normal: np.ndarray) -> np.ndarray:
    return vector - float(np.dot(vector, plane_normal)) * plane_normal


def _build_face_aligned_rotation(
    surface_normal_base: np.ndarray,
    current_rotation: np.ndarray,
    flange_face_axis: str,
) -> np.ndarray:
    face_axis_local = _parse_axis_spec(flange_face_axis)
    face_axis_local = _unit_vector(face_axis_local, "Flange face axis is degenerate.")

    local_anchor_axis = _choose_local_anchor_axis(face_axis_local)
    local_anchor_axis = _unit_vector(
        _project_onto_plane(local_anchor_axis, face_axis_local),
        "Local anchor axis is degenerate.",
    )
    local_side_axis = _unit_vector(
        np.cross(face_axis_local, local_anchor_axis),
        "Local side axis is degenerate.",
    )

    preferred_world_anchor = current_rotation @ local_anchor_axis
    world_anchor_projection = _project_onto_plane(preferred_world_anchor, surface_normal_base)
    if float(np.linalg.norm(world_anchor_projection)) <= 1e-9:
        preferred_world_side = current_rotation @ local_side_axis
        world_anchor_projection = np.cross(preferred_world_side, surface_normal_base)
    if float(np.linalg.norm(world_anchor_projection)) <= 1e-9:
        for fallback_world_axis in (
            np.array([1.0, 0.0, 0.0], dtype=np.float64),
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
        ):
            world_anchor_projection = _project_onto_plane(fallback_world_axis, surface_normal_base)
            if float(np.linalg.norm(world_anchor_projection)) > 1e-9:
                break

    world_anchor_axis = _unit_vector(world_anchor_projection, "World anchor axis is degenerate.")
    world_side_axis = _unit_vector(
        np.cross(surface_normal_base, world_anchor_axis),
        "World side axis is degenerate.",
    )
    world_anchor_axis = _unit_vector(
        np.cross(world_side_axis, surface_normal_base),
        "World anchor axis is degenerate after orthogonalization.",
    )

    local_basis = np.column_stack((local_anchor_axis, local_side_axis, face_axis_local))
    world_basis = np.column_stack((world_anchor_axis, world_side_axis, surface_normal_base))
    return world_basis @ local_basis.T


def _quaternion_slerp(start_xyzw: np.ndarray, end_xyzw: np.ndarray, fraction: float) -> np.ndarray:
    start = start_xyzw / float(np.linalg.norm(start_xyzw))
    end = end_xyzw / float(np.linalg.norm(end_xyzw))
    dot = float(np.dot(start, end))
    if dot < 0.0:
        end = -end
        dot = -dot
    dot = max(min(dot, 1.0), -1.0)
    if dot > 0.9995:
        blended = start + fraction * (end - start)
        return blended / float(np.linalg.norm(blended))

    theta_0 = math.acos(dot)
    theta = theta_0 * fraction
    sin_theta_0 = math.sin(theta_0)
    sin_theta = math.sin(theta)

    start_scale = math.cos(theta) - dot * sin_theta / sin_theta_0
    end_scale = sin_theta / sin_theta_0
    return start_scale * start + end_scale * end


def _rotation_delta_deg(current_rotation: np.ndarray, target_rotation: np.ndarray) -> float:
    current_quaternion = np.asarray(rotation_matrix_to_quaternion_xyzw(current_rotation), dtype=np.float64)
    target_quaternion = np.asarray(rotation_matrix_to_quaternion_xyzw(target_rotation), dtype=np.float64)
    dot = abs(float(np.dot(current_quaternion, target_quaternion)))
    dot = max(min(dot, 1.0), -1.0)
    return float(np.rad2deg(2.0 * math.acos(dot)))


def compute_normal_alignment_error_deg(
    current_tcp_pose_mmdeg: list[float],
    surface_normal_base: list[float] | np.ndarray,
    flange_face_axis: str,
) -> float:
    current_rotation = rpy_deg_to_rotation_matrix(current_tcp_pose_mmdeg[3:6])
    face_axis_local = _unit_vector(_parse_axis_spec(flange_face_axis), "Flange face axis is degenerate.")
    face_axis_world = _unit_vector(current_rotation @ face_axis_local, "Current flange face axis is degenerate.")
    surface_normal = _unit_vector(np.asarray(surface_normal_base, dtype=np.float64).reshape(3), "Surface normal is degenerate.")
    dot = float(np.dot(face_axis_world, surface_normal))
    dot = max(min(dot, 1.0), -1.0)
    return float(np.rad2deg(math.acos(dot)))


def _interpolate_rotation_towards(
    current_rotation: np.ndarray,
    target_rotation: np.ndarray,
    max_step_deg: float,
) -> np.ndarray:
    delta_deg = _rotation_delta_deg(current_rotation, target_rotation)
    if delta_deg <= max_step_deg:
        return target_rotation

    current_quaternion = np.asarray(rotation_matrix_to_quaternion_xyzw(current_rotation), dtype=np.float64)
    target_quaternion = np.asarray(rotation_matrix_to_quaternion_xyzw(target_rotation), dtype=np.float64)
    fraction = max_step_deg / max(delta_deg, 1e-9)
    interpolated = _quaternion_slerp(current_quaternion, target_quaternion, fraction)
    return quaternion_xyzw_to_rotation_matrix(interpolated.tolist())


def _signed_plane_clearance_mm(point_xyz_mm: np.ndarray, plane_point_mm: np.ndarray, plane_normal_base: np.ndarray) -> float:
    return float(np.dot(point_xyz_mm - plane_point_mm, plane_normal_base))


def _limit_translation_step(current_position_mm: np.ndarray, target_position_mm: np.ndarray, max_step_distance_mm: float) -> tuple[np.ndarray, float]:
    delta = target_position_mm - current_position_mm
    distance = float(np.linalg.norm(delta))
    if distance <= max_step_distance_mm:
        return target_position_mm.copy(), distance
    direction = delta / distance
    return current_position_mm + direction * float(max_step_distance_mm), float(max_step_distance_mm)


def _build_safe_lift_position(
    current_position_mm: np.ndarray,
    marker_center_mm: np.ndarray,
    surface_normal_base: np.ndarray,
    current_clearance_mm: float,
    min_safe_z_mm: float,
    min_plane_clearance_mm: float,
    safe_lift_step_mm: float,
    safe_lift_above_marker_mm: float,
    safe_lift_max_z_mm: float,
) -> np.ndarray:
    normal_z = float(surface_normal_base[2])
    if normal_z <= 1e-6:
        raise ValueError("Safe lift requires a positive surface normal z component.")

    z_for_clearance = float(current_position_mm[2]) + (
        float(min_plane_clearance_mm) - float(current_clearance_mm)
    ) / normal_z
    safe_z = max(
        float(current_position_mm[2]) + float(safe_lift_step_mm),
        float(marker_center_mm[2]) + float(safe_lift_above_marker_mm),
        float(min_safe_z_mm),
        z_for_clearance,
    )
    if safe_z > float(safe_lift_max_z_mm):
        raise ValueError("Safe lift pose exceeds maximum z.")

    safe_lift_position = current_position_mm.copy()
    safe_lift_position[2] = safe_z
    return safe_lift_position


def build_approach_decision(
    target_position_base_m: list[float],
    raw_target_orientation_rpy_deg: list[float],
    frame_id: str,
    current_tcp_pose_mmdeg: list[float],
    orientation_mode: str,
    flange_face_axis: str,
    hover_clearance_mm: float,
    pre_approach_distance_mm: float,
    max_step_distance_mm: float,
    min_safe_z_mm: float,
    min_plane_clearance_mm: float,
    workspace_min_mm: list[float],
    workspace_max_mm: list[float],
    stage_switch_buffer_mm: float,
    prefer_positive_z_surface_normal: bool = True,
    enable_safe_lift_on_low_clearance: bool = True,
    safe_lift_step_mm: float = 80.0,
    safe_lift_above_marker_mm: float = 180.0,
    safe_lift_max_z_mm: float = 500.0,
) -> ControlDecision:
    target_point_base_mm = point_m_to_mm(target_position_base_m)
    raw_target_pose_base_mmdeg = target_point_base_mm + [float(value) for value in raw_target_orientation_rpy_deg]

    def rejection(
        error_message: str,
        *,
        target_pose_base_mmdeg: list[float] | None = None,
        surface_normal_base: list[float] | None = None,
        final_hover_pose_mmdeg: list[float] | None = None,
        pre_approach_pose_mmdeg: list[float] | None = None,
        candidate_pose_mmdeg: list[float] | None = None,
        candidate_stage: str | None = None,
        step_distance_mm: float | None = None,
        distance_to_target_mm: float | None = None,
        clearance_to_plane_mm: float | None = None,
    ) -> ControlDecision:
        return ControlDecision(
            check_passed=False,
            error_message=error_message,
            orientation_mode=orientation_mode,
            target_point_base_mm=target_point_base_mm,
            target_pose_base_mmdeg=target_pose_base_mmdeg or raw_target_pose_base_mmdeg,
            raw_target_pose_base_mmdeg=raw_target_pose_base_mmdeg,
            current_tcp_pose_mmdeg=[float(value) for value in current_tcp_pose_mmdeg],
            surface_normal_base=surface_normal_base,
            final_hover_pose_mmdeg=final_hover_pose_mmdeg,
            pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
            candidate_pose_mmdeg=candidate_pose_mmdeg,
            candidate_stage=candidate_stage,
            step_distance_mm=step_distance_mm,
            distance_to_target_mm=distance_to_target_mm,
            clearance_to_plane_mm=clearance_to_plane_mm,
        )

    if frame_id != "robot_base":
        return rejection(f"Unexpected frame_id: {frame_id}")

    if orientation_mode != "face_marker_normal":
        return rejection(f"Unsupported orientation_mode: {orientation_mode}")

    if not is_finite_point(target_point_base_mm):
        return rejection("Target point contains NaN or Inf.")

    if len(raw_target_orientation_rpy_deg) != 3 or not all(math.isfinite(float(value)) for value in raw_target_orientation_rpy_deg):
        return rejection("Target orientation contains invalid values.")

    current_position = np.asarray(current_tcp_pose_mmdeg[:3], dtype=np.float64)
    current_rotation = rpy_deg_to_rotation_matrix(current_tcp_pose_mmdeg[3:6])
    marker_center = np.asarray(target_point_base_mm, dtype=np.float64)
    raw_target_rotation = rpy_deg_to_rotation_matrix(raw_target_orientation_rpy_deg)

    try:
        raw_surface_normal = _unit_vector(
            np.asarray(raw_target_rotation[:, 2], dtype=np.float64).reshape(3),
            "Marker surface normal is degenerate.",
        )
    except ValueError as exc:
        return rejection(str(exc))

    if float(np.dot(current_position - marker_center, raw_surface_normal)) < 0.0:
        surface_normal = -raw_surface_normal
    else:
        surface_normal = raw_surface_normal

    # Tabletop demos should approach from the positive base-Z side; this keeps
    # small side-of-plane sign noise from pushing pre-approach below the table.
    if prefer_positive_z_surface_normal and float(surface_normal[2]) < 0.0:
        surface_normal = -surface_normal

    try:
        final_hover_rotation = _build_face_aligned_rotation(surface_normal, current_rotation, flange_face_axis)
    except ValueError as exc:
        return rejection(str(exc), surface_normal_base=surface_normal.tolist())

    adjusted_target_orientation_rpy_deg = rotation_matrix_to_rpy_deg(final_hover_rotation)
    target_pose_base_mmdeg = target_point_base_mm + [float(value) for value in adjusted_target_orientation_rpy_deg]

    final_hover_position = marker_center + surface_normal * float(hover_clearance_mm)
    pre_approach_position = final_hover_position + surface_normal * float(pre_approach_distance_mm)

    final_hover_pose_mmdeg = [
        float(final_hover_position[0]),
        float(final_hover_position[1]),
        float(final_hover_position[2]),
        *[float(value) for value in adjusted_target_orientation_rpy_deg],
    ]
    pre_approach_pose_mmdeg = [
        float(pre_approach_position[0]),
        float(pre_approach_position[1]),
        float(pre_approach_position[2]),
        *[float(value) for value in current_tcp_pose_mmdeg[3:6]],
    ]

    for pose_name, pose_position in (
        ("Final hover", final_hover_position),
        ("Pre-approach", pre_approach_position),
    ):
        workspace_error = _workspace_error(pose_position, workspace_min_mm, workspace_max_mm)
        if workspace_error is not None:
            return rejection(
                f"{pose_name} pose is outside workspace.",
                target_pose_base_mmdeg=target_pose_base_mmdeg,
                surface_normal_base=surface_normal.tolist(),
                final_hover_pose_mmdeg=final_hover_pose_mmdeg,
                pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
            )
        if float(pose_position[2]) < float(min_safe_z_mm):
            return rejection(
                f"{pose_name} z is below minimum safe height.",
                target_pose_base_mmdeg=target_pose_base_mmdeg,
                surface_normal_base=surface_normal.tolist(),
                final_hover_pose_mmdeg=final_hover_pose_mmdeg,
                pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
            )
        if _signed_plane_clearance_mm(pose_position, marker_center, surface_normal) < float(min_plane_clearance_mm):
            return rejection(
                f"{pose_name} is below minimum plane clearance.",
                target_pose_base_mmdeg=target_pose_base_mmdeg,
                surface_normal_base=surface_normal.tolist(),
                final_hover_pose_mmdeg=final_hover_pose_mmdeg,
                pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
            )

    distance_to_final_hover = float(np.linalg.norm(final_hover_position - current_position))
    distance_to_pre_approach = float(np.linalg.norm(pre_approach_position - current_position))
    orientation_delta_deg = _rotation_delta_deg(current_rotation, final_hover_rotation)
    current_clearance_to_plane_mm = _signed_plane_clearance_mm(current_position, marker_center, surface_normal)

    if distance_to_final_hover <= float(stage_switch_buffer_mm) and orientation_delta_deg <= float(DEFAULT_REORIENT_TOLERANCE_DEG):
        candidate_stage = FINAL_HOVER_STAGE
        candidate_position = final_hover_position.copy()
        step_distance = distance_to_final_hover
        candidate_rotation = final_hover_rotation
    elif distance_to_pre_approach > float(stage_switch_buffer_mm):
        candidate_stage = PRE_APPROACH_STAGE
        candidate_position, step_distance = _limit_translation_step(
            current_position,
            pre_approach_position,
            float(max_step_distance_mm),
        )
        candidate_rotation = current_rotation
    elif orientation_delta_deg > float(DEFAULT_REORIENT_TOLERANCE_DEG):
        candidate_stage = REORIENT_STAGE
        candidate_position = pre_approach_position.copy()
        step_distance = float(np.linalg.norm(candidate_position - current_position))
        candidate_rotation = _interpolate_rotation_towards(
            current_rotation,
            final_hover_rotation,
            float(DEFAULT_MAX_REORIENT_STEP_DEG),
        )
    else:
        candidate_stage = FINAL_HOVER_STAGE
        candidate_position, step_distance = _limit_translation_step(
            current_position,
            final_hover_position,
            float(max_step_distance_mm),
        )
        candidate_rotation = final_hover_rotation

    clearance_to_plane_mm = _signed_plane_clearance_mm(candidate_position, marker_center, surface_normal)
    if (
        enable_safe_lift_on_low_clearance
        and (
            current_clearance_to_plane_mm < float(min_plane_clearance_mm)
            or clearance_to_plane_mm < float(min_plane_clearance_mm)
            or float(candidate_position[2]) < float(min_safe_z_mm)
        )
    ):
        try:
            candidate_position = _build_safe_lift_position(
                current_position,
                marker_center,
                surface_normal,
                current_clearance_to_plane_mm,
                float(min_safe_z_mm),
                float(min_plane_clearance_mm),
                float(safe_lift_step_mm),
                float(safe_lift_above_marker_mm),
                float(safe_lift_max_z_mm),
            )
        except ValueError as exc:
            return rejection(
                str(exc),
                target_pose_base_mmdeg=target_pose_base_mmdeg,
                surface_normal_base=surface_normal.tolist(),
                final_hover_pose_mmdeg=final_hover_pose_mmdeg,
                pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
                candidate_stage=SAFE_LIFT_STAGE,
                step_distance_mm=None,
                distance_to_target_mm=distance_to_final_hover,
                clearance_to_plane_mm=current_clearance_to_plane_mm,
            )
        candidate_stage = SAFE_LIFT_STAGE
        candidate_rotation = current_rotation
        step_distance = float(np.linalg.norm(candidate_position - current_position))
        clearance_to_plane_mm = _signed_plane_clearance_mm(candidate_position, marker_center, surface_normal)

    workspace_error = _workspace_error(candidate_position, workspace_min_mm, workspace_max_mm)
    if workspace_error is not None:
        return rejection(
            "Candidate pose is outside workspace.",
            target_pose_base_mmdeg=target_pose_base_mmdeg,
            surface_normal_base=surface_normal.tolist(),
            final_hover_pose_mmdeg=final_hover_pose_mmdeg,
            pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
            candidate_stage=candidate_stage,
        )

    if float(candidate_position[2]) < float(min_safe_z_mm):
        return rejection(
            "Candidate z is below minimum safe height.",
            target_pose_base_mmdeg=target_pose_base_mmdeg,
            surface_normal_base=surface_normal.tolist(),
            final_hover_pose_mmdeg=final_hover_pose_mmdeg,
            pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
            candidate_stage=candidate_stage,
            step_distance_mm=step_distance,
            distance_to_target_mm=distance_to_final_hover,
            clearance_to_plane_mm=clearance_to_plane_mm,
        )
    if clearance_to_plane_mm < float(min_plane_clearance_mm):
        return rejection(
            "Candidate pose is below minimum plane clearance.",
            target_pose_base_mmdeg=target_pose_base_mmdeg,
            surface_normal_base=surface_normal.tolist(),
            final_hover_pose_mmdeg=final_hover_pose_mmdeg,
            pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
            candidate_stage=candidate_stage,
            step_distance_mm=step_distance,
            distance_to_target_mm=distance_to_final_hover,
            clearance_to_plane_mm=clearance_to_plane_mm,
        )

    candidate_pose_mmdeg = [
        float(candidate_position[0]),
        float(candidate_position[1]),
        float(candidate_position[2]),
        *[float(value) for value in rotation_matrix_to_rpy_deg(candidate_rotation)],
    ]

    return ControlDecision(
        check_passed=True,
        error_message="",
        orientation_mode=orientation_mode,
        target_point_base_mm=target_point_base_mm,
        target_pose_base_mmdeg=target_pose_base_mmdeg,
        raw_target_pose_base_mmdeg=raw_target_pose_base_mmdeg,
        current_tcp_pose_mmdeg=[float(value) for value in current_tcp_pose_mmdeg],
        surface_normal_base=surface_normal.tolist(),
        final_hover_pose_mmdeg=final_hover_pose_mmdeg,
        pre_approach_pose_mmdeg=pre_approach_pose_mmdeg,
        candidate_pose_mmdeg=candidate_pose_mmdeg,
        candidate_stage=candidate_stage,
        step_distance_mm=step_distance,
        distance_to_target_mm=distance_to_final_hover,
        clearance_to_plane_mm=clearance_to_plane_mm,
    )
