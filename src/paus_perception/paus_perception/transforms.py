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


# 保存 eye-to-hand 标定结果到 YAML。
def save_eye_to_hand_solution(solution: EyeToHandCalibrationSolution, output_path: str | Path) -> None:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {
        "extrinsic": {
            "base_to_camera": {
                "translation_m": solution.base_to_camera.translation_m if solution.base_to_camera else None,
                "rotation_matrix": solution.base_to_camera.rotation_matrix if solution.base_to_camera else None,
                "rotation_quaternion_xyzw": solution.base_to_camera.rotation_quaternion_xyzw if solution.base_to_camera else None,
            }
        },
        "calibration": {
            "sample_count": solution.sample_count,
            "method": solution.method,
            "tool_to_board": {
                "translation_m": solution.tool_to_board_translation_m,
                "rotation_rpy_deg": solution.tool_to_board_rotation_rpy_deg,
            },
        },
    }
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)


# 读取 eye-to-hand 外参 YAML。
def load_eye_to_hand_solution(input_path: str | Path) -> EyeToHandCalibrationSolution:
    input_path = Path(input_path)
    with input_path.open("r", encoding="utf-8") as handle:
        payload: dict[str, Any] = yaml.safe_load(handle) or {}
    extrinsic_payload = payload.get("extrinsic", {}).get("base_to_camera", {})
    calibration_payload = payload.get("calibration", {})
    tool_to_board_payload = calibration_payload.get("tool_to_board", {})
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
    )
