from __future__ import annotations

# 导入 dataclass，便于定义结构化外参结果。
from dataclasses import dataclass
# 导入 Path，便于处理 YAML 文件路径。
from pathlib import Path
# 导入 Any，便于描述 YAML 的动态结构。
from typing import Any

# 导入 OpenCV，用于 Rodrigues 公式转换。
import cv2
# 导入 NumPy，用于矩阵运算。
import numpy as np
# 导入 yaml，用于读写外参文件。
import yaml


# 保存 3D 刚体变换的数据结构。
@dataclass
class Transform3D:
    # 目标坐标系下的平移向量，单位米。
    translation_m: list[float]
    # 目标坐标系下的旋转矩阵。
    rotation_matrix: list[list[float]]
    # 目标坐标系下的四元数，顺序为 x y z w。
    rotation_quaternion_xyzw: list[float]
    # 当前变换的父坐标系名字。
    parent_frame: str
    # 当前变换的子坐标系名字。
    child_frame: str


# 保存 eye-to-hand 标定求解结果。
@dataclass
class EyeToHandCalibrationSolution:
    # 当前求解状态。
    status: str
    # 是否求解成功。
    success: bool
    # 样本数量。
    sample_count: int
    # 可读说明。
    message: str
    # 求出的 base->camera 变换。
    base_to_camera: Transform3D | None
    # 工具到标定板的固定平移。
    tool_to_board_translation_m: list[float]
    # 工具到标定板的固定旋转欧拉角。
    tool_to_board_rotation_rpy_deg: list[float]
    # 求解方法说明。
    method: str = "eye_to_hand_board_average"
    # 观测模式。
    observation_mode: str = "rgb_pnp"
    # 会话级残差摘要。
    residuals: CalibrationResidualSummary | None = None
    # 会话级诊断字段。
    session_diagnostics: dict[str, Any] | None = None
    # 样本质量汇总。
    sample_quality_summary: dict[str, Any] | None = None
    # 外参 YAML 来源路径。
    source_path: str | None = None
    # 外参 artifact 类型，例如 calibration_result / identity_example。
    artifact_kind: str = "unknown"
    # 是否为 dummy/example 外参，真机运动时必须拒绝使用。
    is_dummy: bool = False


# 保存一组标定样本回代残差。
@dataclass
class CalibrationResidualSummary:
    translation_rms_mm: float
    translation_mean_mm: float
    translation_max_mm: float
    rotation_rms_deg: float
    rotation_mean_deg: float
    rotation_max_deg: float
    sample_count: int


# 将平移和旋转矩阵拼成 4x4 齐次变换矩阵。
def make_transform_matrix(translation: list[float] | np.ndarray, rotation_matrix: list[list[float]] | np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return matrix


# 将齐次矩阵拆成平移和旋转矩阵。
def split_transform_matrix(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    return matrix[:3, 3].copy(), matrix[:3, :3].copy()


# 求齐次矩阵逆。
def invert_transform_matrix(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


# 将旋转向量转成旋转矩阵。
def rvec_to_rotation_matrix(rvec: list[float] | np.ndarray) -> np.ndarray:
    rotation_vector = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    return rotation_matrix


# 将欧拉角（度）转成旋转矩阵，顺序固定为 XYZ。
def rpy_deg_to_rotation_matrix(rpy_deg: list[float] | np.ndarray) -> np.ndarray:
    roll_deg, pitch_deg, yaw_deg = np.asarray(rpy_deg, dtype=np.float64).reshape(3)
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, np.cos(roll), -np.sin(roll)], [0.0, np.sin(roll), np.cos(roll)]],
        dtype=np.float64,
    )
    ry = np.array(
        [[np.cos(pitch), 0.0, np.sin(pitch)], [0.0, 1.0, 0.0], [-np.sin(pitch), 0.0, np.cos(pitch)]],
        dtype=np.float64,
    )
    rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return rz @ ry @ rx


# 将旋转矩阵反解为欧拉角（度），顺序与 rpy_deg_to_rotation_matrix 保持一致。
def rotation_matrix_to_rpy_deg(rotation_matrix: list[list[float]] | np.ndarray) -> list[float]:
    matrix = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    pitch = np.arcsin(np.clip(-matrix[2, 0], -1.0, 1.0))
    cos_pitch = float(np.cos(pitch))

    if abs(cos_pitch) > 1e-9:
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-matrix[0, 1], matrix[1, 1])

    return [float(np.rad2deg(roll)), float(np.rad2deg(pitch)), float(np.rad2deg(yaw))]


# 将四元数转成旋转矩阵，输入顺序为 x y z w。
def quaternion_xyzw_to_rotation_matrix(quaternion_xyzw: list[float] | np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


# 将旋转矩阵转成四元数，输出顺序为 x y z w。
def rotation_matrix_to_quaternion_xyzw(rotation_matrix: list[list[float]] | np.ndarray) -> list[float]:
    matrix = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    return [float(x), float(y), float(z), float(w)]


# 将平移和旋转矩阵封装成 Transform3D。
def make_transform_struct(translation: list[float] | np.ndarray, rotation_matrix: list[list[float]] | np.ndarray, parent_frame: str, child_frame: str) -> Transform3D:
    rotation_array = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    translation_array = np.asarray(translation, dtype=np.float64).reshape(3)
    return Transform3D(
        translation_m=[float(value) for value in translation_array.tolist()],
        rotation_matrix=[[float(value) for value in row] for row in rotation_array.tolist()],
        rotation_quaternion_xyzw=rotation_matrix_to_quaternion_xyzw(rotation_array),
        parent_frame=parent_frame,
        child_frame=child_frame,
    )


# 对多组旋转矩阵做平均，并投影回合法旋转矩阵。
def average_rotation_matrices(rotation_matrices: list[np.ndarray]) -> np.ndarray:
    mean_matrix = np.mean(np.stack(rotation_matrices, axis=0), axis=0)
    u, _, vh = np.linalg.svd(mean_matrix)
    rotation = u @ vh
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh
    return rotation


# 对多组齐次矩阵做外参平均。
def average_transform_matrices(transform_matrices: list[np.ndarray]) -> np.ndarray:
    if not transform_matrices:
        raise ValueError("No transform matrices provided for averaging.")
    translations = []
    rotations = []
    for matrix in transform_matrices:
        translation, rotation = split_transform_matrix(matrix)
        translations.append(translation)
        rotations.append(rotation)
    mean_translation = np.mean(np.stack(translations, axis=0), axis=0)
    mean_rotation = average_rotation_matrices(rotations)
    return make_transform_matrix(mean_translation, mean_rotation)


# 计算两个旋转矩阵之间的夹角，单位为度。
def rotation_error_deg(lhs_rotation: np.ndarray, rhs_rotation: np.ndarray) -> float:
    delta = np.asarray(lhs_rotation, dtype=np.float64).reshape(3, 3).T @ np.asarray(rhs_rotation, dtype=np.float64).reshape(3, 3)
    cosine = (float(np.trace(delta)) - 1.0) * 0.5
    return float(np.rad2deg(np.arccos(np.clip(cosine, -1.0, 1.0))))


# 回代检查每个样本：base->camera->board 应该接近 base->tool->board。
def evaluate_eye_to_hand_residuals(
    base_to_camera_matrix: np.ndarray,
    base_to_tool_matrices: list[np.ndarray],
    camera_to_board_matrices: list[np.ndarray],
    tool_to_board_matrix: np.ndarray | None = None,
) -> CalibrationResidualSummary:
    if len(base_to_tool_matrices) != len(camera_to_board_matrices):
        raise ValueError("Robot and camera sample counts do not match.")
    if not base_to_tool_matrices:
        raise ValueError("At least one sample is required to evaluate calibration residuals.")

    tool_to_board = np.eye(4, dtype=np.float64) if tool_to_board_matrix is None else np.asarray(tool_to_board_matrix, dtype=np.float64).reshape(4, 4)
    base_to_camera = np.asarray(base_to_camera_matrix, dtype=np.float64).reshape(4, 4)
    translation_errors_mm = []
    rotation_errors_deg = []

    for base_to_tool, camera_to_board in zip(base_to_tool_matrices, camera_to_board_matrices):
        observed_base_to_board = np.asarray(base_to_tool, dtype=np.float64).reshape(4, 4) @ tool_to_board
        predicted_base_to_board = base_to_camera @ np.asarray(camera_to_board, dtype=np.float64).reshape(4, 4)
        translation_errors_mm.append(float(np.linalg.norm(predicted_base_to_board[:3, 3] - observed_base_to_board[:3, 3]) * 1000.0))
        rotation_errors_deg.append(rotation_error_deg(observed_base_to_board[:3, :3], predicted_base_to_board[:3, :3]))

    translation_array = np.asarray(translation_errors_mm, dtype=np.float64)
    rotation_array = np.asarray(rotation_errors_deg, dtype=np.float64)
    return CalibrationResidualSummary(
        translation_rms_mm=float(np.sqrt(np.mean(np.square(translation_array)))),
        translation_mean_mm=float(np.mean(translation_array)),
        translation_max_mm=float(np.max(translation_array)),
        rotation_rms_deg=float(np.sqrt(np.mean(np.square(rotation_array)))),
        rotation_mean_deg=float(np.mean(rotation_array)),
        rotation_max_deg=float(np.max(rotation_array)),
        sample_count=len(base_to_tool_matrices),
    )


# 使用 OpenCV 官方 hand-eye 接口求解 eye-to-hand 外参。
# 输入：
# - `base_to_tool_matrices`：每次采样的 ^bT_g
# - `target_to_camera_matrices`：每次采样的 ^cT_t（PnP 结果）
# 输出：
# - eye-to-hand 场景下的 ^bT_c
def solve_eye_to_hand_opencv_handeye(
    base_to_tool_matrices: list[np.ndarray],
    target_to_camera_matrices: list[np.ndarray],
    method: int = cv2.CALIB_HAND_EYE_PARK,
) -> np.ndarray:
    if len(base_to_tool_matrices) != len(target_to_camera_matrices):
        raise ValueError("Robot and target pose sample counts do not match.")
    if len(base_to_tool_matrices) < 3:
        raise ValueError("At least three samples are required for OpenCV hand-eye calibration.")

    rotation_gripper_to_base = []
    translation_gripper_to_base = []
    rotation_target_to_camera = []
    translation_target_to_camera = []

    for base_to_tool, target_to_camera in zip(base_to_tool_matrices, target_to_camera_matrices):
        gripper_to_base = invert_transform_matrix(base_to_tool)
        rotation_gripper_to_base.append(np.asarray(gripper_to_base[:3, :3], dtype=np.float64))
        translation_gripper_to_base.append(np.asarray(gripper_to_base[:3, 3], dtype=np.float64).reshape(3, 1))
        rotation_target_to_camera.append(np.asarray(target_to_camera[:3, :3], dtype=np.float64))
        translation_target_to_camera.append(np.asarray(target_to_camera[:3, 3], dtype=np.float64).reshape(3, 1))

    rotation_base_to_camera, translation_base_to_camera = cv2.calibrateHandEye(
        rotation_gripper_to_base,
        translation_gripper_to_base,
        rotation_target_to_camera,
        translation_target_to_camera,
        method=method,
    )
    return make_transform_matrix(
        np.asarray(translation_base_to_camera, dtype=np.float64).reshape(3),
        np.asarray(rotation_base_to_camera, dtype=np.float64).reshape(3, 3),
    )


# 根据绝对位姿样本构造 AX=XB 的相对运动对。
def build_ax_xb_motion_pairs(lhs_absolute_matrices: list[np.ndarray], rhs_absolute_matrices: list[np.ndarray]) -> list[tuple[np.ndarray, np.ndarray]]:
    if len(lhs_absolute_matrices) != len(rhs_absolute_matrices):
        raise ValueError("Left and right motion sample counts do not match.")
    if len(lhs_absolute_matrices) < 2:
        raise ValueError("At least two absolute pose samples are required to build AX=XB motion pairs.")

    motion_pairs: list[tuple[np.ndarray, np.ndarray]] = []
    for i in range(len(lhs_absolute_matrices) - 1):
        for j in range(i + 1, len(lhs_absolute_matrices)):
            lhs_relative = np.asarray(lhs_absolute_matrices[i], dtype=np.float64) @ invert_transform_matrix(lhs_absolute_matrices[j])
            rhs_relative = np.asarray(rhs_absolute_matrices[i], dtype=np.float64) @ invert_transform_matrix(rhs_absolute_matrices[j])
            motion_pairs.append((lhs_relative, rhs_relative))
    return motion_pairs


# 使用 Park-Martin 的 AX=XB 方法求解 hand-eye 外参。
def solve_ax_xb_hand_eye_park(lhs_absolute_matrices: list[np.ndarray], rhs_absolute_matrices: list[np.ndarray]) -> np.ndarray:
    motion_pairs = build_ax_xb_motion_pairs(lhs_absolute_matrices, rhs_absolute_matrices)

    rotation_constraints: list[tuple[np.ndarray, np.ndarray]] = []
    for lhs_relative, rhs_relative in motion_pairs:
        lhs_rotation = np.asarray(lhs_relative[:3, :3], dtype=np.float64)
        rhs_rotation = np.asarray(rhs_relative[:3, :3], dtype=np.float64)
        lhs_rvec, _ = cv2.Rodrigues(lhs_rotation)
        rhs_rvec, _ = cv2.Rodrigues(rhs_rotation)
        lhs_axis = lhs_rvec.reshape(3)
        rhs_axis = rhs_rvec.reshape(3)
        if np.linalg.norm(lhs_axis) <= 1e-9 or np.linalg.norm(rhs_axis) <= 1e-9:
            continue
        rotation_constraints.append((lhs_axis, rhs_axis))

    if len(rotation_constraints) < 2:
        raise ValueError("Not enough informative relative motions to solve AX=XB rotation.")

    correlation = np.zeros((3, 3), dtype=np.float64)
    for lhs_axis, rhs_axis in rotation_constraints:
        correlation += np.outer(lhs_axis, rhs_axis)

    u, _, vh = np.linalg.svd(correlation)
    rotation = u @ vh
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh

    lhs_translation_blocks = []
    rhs_translation_blocks = []
    identity = np.eye(3, dtype=np.float64)
    for lhs_relative, rhs_relative in motion_pairs:
        lhs_rotation = np.asarray(lhs_relative[:3, :3], dtype=np.float64)
        lhs_translation = np.asarray(lhs_relative[:3, 3], dtype=np.float64)
        rhs_translation = np.asarray(rhs_relative[:3, 3], dtype=np.float64)
        lhs_translation_blocks.append(lhs_rotation - identity)
        rhs_translation_blocks.append(rotation @ rhs_translation - lhs_translation)

    lhs_stacked = np.concatenate(lhs_translation_blocks, axis=0)
    rhs_stacked = np.concatenate(rhs_translation_blocks, axis=0)
    translation, _, rank, _ = np.linalg.lstsq(lhs_stacked, rhs_stacked, rcond=None)
    if rank < 3:
        raise ValueError("AX=XB translation solve is rank deficient.")

    return make_transform_matrix(translation, rotation)


# 保存 eye-to-hand 标定结果到 YAML。
def save_eye_to_hand_solution(solution: EyeToHandCalibrationSolution, output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_kind = solution.artifact_kind
    if artifact_kind == "unknown" and solution.success and solution.sample_count > 0:
        artifact_kind = "calibration_result"
    payload: dict[str, Any] = {
        "metadata": {
            "artifact_kind": artifact_kind,
            "dummy": bool(solution.is_dummy),
        },
        "extrinsic": {
            "base_to_camera": {
                "translation_m": solution.base_to_camera.translation_m if solution.base_to_camera else None,
                "rotation_matrix": solution.base_to_camera.rotation_matrix if solution.base_to_camera else None,
                "rotation_quaternion_xyzw": solution.base_to_camera.rotation_quaternion_xyzw if solution.base_to_camera else None,
            }
        },
        "calibration": {
            "artifact_kind": artifact_kind,
            "dummy": bool(solution.is_dummy),
            "sample_count": solution.sample_count,
            "method": solution.method,
            "observation_mode": solution.observation_mode,
            "tool_to_board": {
                "translation_m": solution.tool_to_board_translation_m,
                "rotation_rpy_deg": solution.tool_to_board_rotation_rpy_deg,
            },
        },
    }
    if solution.residuals is not None:
        payload["calibration"]["residuals"] = {
            "translation_rms_mm": solution.residuals.translation_rms_mm,
            "translation_mean_mm": solution.residuals.translation_mean_mm,
            "translation_max_mm": solution.residuals.translation_max_mm,
            "rotation_rms_deg": solution.residuals.rotation_rms_deg,
            "rotation_mean_deg": solution.residuals.rotation_mean_deg,
            "rotation_max_deg": solution.residuals.rotation_max_deg,
            "sample_count": solution.residuals.sample_count,
        }
    if solution.session_diagnostics is not None:
        payload["calibration"]["session_diagnostics"] = solution.session_diagnostics
    if solution.sample_quality_summary is not None:
        payload["calibration"]["sample_quality_summary"] = solution.sample_quality_summary
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)


# 读取 eye-to-hand 外参 YAML。
def load_eye_to_hand_solution(input_path: str | Path) -> EyeToHandCalibrationSolution:
    input_path = Path(input_path)
    with input_path.open("r", encoding="utf-8") as handle:
        payload: dict[str, Any] = yaml.safe_load(handle) or {}
    extrinsic_payload = payload.get("extrinsic", {}).get("base_to_camera", {})
    calibration_payload = payload.get("calibration", {})
    metadata_payload = payload.get("metadata", {})
    tool_to_board_payload = calibration_payload.get("tool_to_board", {})
    artifact_kind = str(
        metadata_payload.get("artifact_kind")
        or calibration_payload.get("artifact_kind")
        or ("calibration_result" if calibration_payload.get("sample_count", 0) else "unknown")
    )
    is_dummy = bool(
        metadata_payload.get("dummy", False)
        or calibration_payload.get("dummy", False)
        or artifact_kind in {"dummy", "example", "identity_example", "demo"}
    )
    base_to_camera = None
    if extrinsic_payload.get("translation_m") and extrinsic_payload.get("rotation_matrix"):
        base_to_camera = Transform3D(
            translation_m=[float(value) for value in extrinsic_payload["translation_m"]],
            rotation_matrix=[[float(value) for value in row] for row in extrinsic_payload["rotation_matrix"]],
            rotation_quaternion_xyzw=[float(value) for value in extrinsic_payload.get("rotation_quaternion_xyzw", [])],
            parent_frame="robot_base",
            child_frame="camera",
        )
    return EyeToHandCalibrationSolution(
        status="ok" if base_to_camera is not None else "missing_extrinsic",
        success=base_to_camera is not None,
        sample_count=int(calibration_payload.get("sample_count", 0)),
        message="Extrinsic loaded successfully." if base_to_camera is not None else "Extrinsic file does not contain base_to_camera.",
        base_to_camera=base_to_camera,
        tool_to_board_translation_m=[float(value) for value in tool_to_board_payload.get("translation_m", [0.0, 0.0, 0.0])],
        tool_to_board_rotation_rpy_deg=[float(value) for value in tool_to_board_payload.get("rotation_rpy_deg", [0.0, 0.0, 0.0])],
        method=str(calibration_payload.get("method", "eye_to_hand_board_average")),
        observation_mode=str(calibration_payload.get("observation_mode", "rgb_pnp")),
        residuals=CalibrationResidualSummary(**calibration_payload["residuals"]) if "residuals" in calibration_payload else None,
        session_diagnostics=dict(calibration_payload.get("session_diagnostics", {})) if calibration_payload.get("session_diagnostics") is not None else None,
        sample_quality_summary=dict(calibration_payload.get("sample_quality_summary", {})) if calibration_payload.get("sample_quality_summary") is not None else None,
        source_path=str(input_path),
        artifact_kind=artifact_kind,
        is_dummy=is_dummy,
    )
