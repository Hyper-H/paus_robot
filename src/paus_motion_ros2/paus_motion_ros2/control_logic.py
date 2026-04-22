from __future__ import annotations

# 导入 dataclass，便于组织控制决策结果。
from dataclasses import dataclass
# 导入 math，用于数值合法性判断。
import math

# 导入 NumPy，用于向量运算。
import numpy as np


# 保存一次控制决策的结果。
@dataclass
class ControlDecision:
    # 当前决策是否通过安全检查。
    check_passed: bool
    # 失败时的原因说明。
    error_message: str
    # 输入目标点，单位毫米。
    target_point_base_mm: list[float]
    # 当前 TCP 位姿，单位 mm/deg。
    current_tcp_pose_mmdeg: list[float]
    # 候选接近位姿，单位 mm/deg。
    candidate_pose_mmdeg: list[float] | None
    # 当前 TCP 到候选位姿的位移长度，单位毫米。
    step_distance_mm: float | None


# 将以米为单位的点转换为毫米。
def point_m_to_mm(point_m: list[float]) -> list[float]:
    return [float(value) * 1000.0 for value in point_m]


# 检查一个三维点是否全是有限数。
def is_finite_point(point_xyz: list[float]) -> bool:
    return all(math.isfinite(float(value)) for value in point_xyz)


# 计算第一版的安全接近位姿。
def build_approach_decision(
    target_point_base_m: list[float],
    frame_id: str,
    current_tcp_pose_mmdeg: list[float],
    final_standoff_mm: float,
    max_step_distance_mm: float,
    min_safe_z_mm: float,
    workspace_min_mm: list[float],
    workspace_max_mm: list[float],
) -> ControlDecision:
    # 先把目标点从米转换到毫米。
    target_point_base_mm = point_m_to_mm(target_point_base_m)

    # 检查 frame_id 是否正确。
    if frame_id != "robot_base":
        return ControlDecision(
            check_passed=False,
            error_message=f"Unexpected frame_id: {frame_id}",
            target_point_base_mm=target_point_base_mm,
            current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
            candidate_pose_mmdeg=None,
            step_distance_mm=None,
        )

    # 检查目标点是否合法。
    if not is_finite_point(target_point_base_mm):
        return ControlDecision(
            check_passed=False,
            error_message="Target point contains NaN or Inf.",
            target_point_base_mm=target_point_base_mm,
            current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
            candidate_pose_mmdeg=None,
            step_distance_mm=None,
        )

    # 检查工作空间边界。
    for axis, value, lower, upper in zip(("x", "y", "z"), target_point_base_mm, workspace_min_mm, workspace_max_mm):
        if float(value) < float(lower) or float(value) > float(upper):
            return ControlDecision(
                check_passed=False,
                error_message=f"Target {axis} is outside workspace.",
                target_point_base_mm=target_point_base_mm,
                current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
                candidate_pose_mmdeg=None,
                step_distance_mm=None,
            )

    # 检查最低安全高度。
    if float(target_point_base_mm[2]) < float(min_safe_z_mm):
        return ControlDecision(
            check_passed=False,
            error_message="Target z is below minimum safe height.",
            target_point_base_mm=target_point_base_mm,
            current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
            candidate_pose_mmdeg=None,
            step_distance_mm=None,
        )

    # 当前 TCP 位姿拆成位置和姿态。
    current_position = np.asarray(current_tcp_pose_mmdeg[:3], dtype=np.float64)
    current_orientation = [float(value) for value in current_tcp_pose_mmdeg[3:6]]
    target_position = np.asarray(target_point_base_mm, dtype=np.float64)

    # 计算当前 TCP 到目标点的距离。
    direction_vector = target_position - current_position
    distance_to_target = float(np.linalg.norm(direction_vector))

    # 如果已经过近，则不给出执行命令。
    if distance_to_target <= float(final_standoff_mm):
        return ControlDecision(
            check_passed=False,
            error_message="Current TCP is already within final standoff distance.",
            target_point_base_mm=target_point_base_mm,
            current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
            candidate_pose_mmdeg=None,
            step_distance_mm=distance_to_target,
        )

    # 计算从当前 TCP 指向目标点的单位方向。
    direction_unit = direction_vector / distance_to_target
    # 计算预接近点：停在目标点前方一定距离，而不是直接贴近目标。
    candidate_position = target_position - direction_unit * float(final_standoff_mm)
    # 计算当前 TCP 到候选点的单次位移。
    step_distance = float(np.linalg.norm(candidate_position - current_position))

    # 检查单次位移是否过大。
    if step_distance > float(max_step_distance_mm):
        return ControlDecision(
            check_passed=False,
            error_message="Step distance exceeds maximum allowed distance.",
            target_point_base_mm=target_point_base_mm,
            current_tcp_pose_mmdeg=current_tcp_pose_mmdeg,
            candidate_pose_mmdeg=None,
            step_distance_mm=step_distance,
        )

    # 组合候选位姿，姿态保持当前 TCP 姿态不变。
    candidate_pose_mmdeg = [
        float(candidate_position[0]),
        float(candidate_position[1]),
        float(candidate_position[2]),
        *current_orientation,
    ]

    return ControlDecision(
        check_passed=True,
        error_message="",
        target_point_base_mm=target_point_base_mm,
        current_tcp_pose_mmdeg=[float(value) for value in current_tcp_pose_mmdeg],
        candidate_pose_mmdeg=candidate_pose_mmdeg,
        step_distance_mm=step_distance,
    )
