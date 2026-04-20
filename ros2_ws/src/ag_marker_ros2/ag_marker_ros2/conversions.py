from __future__ import annotations

# 导入 json，用于序列化检测状态。
import json
# 导入 math，用于旋转矩阵转四元数。
import math
from typing import Any

# 导入 OpenCV，用于将旋转向量转换为旋转矩阵。
import cv2
# 导入 NumPy，用于矩阵运算。
import numpy as np


# 将 rvec 转换为四元数。
def rvec_to_quaternion(rvec: list[float] | tuple[float, float, float]) -> tuple[float, float, float, float]:
    # 将旋转向量整理成 OpenCV 需要的形状。
    rotation_vector = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    # 先将旋转向量转为旋转矩阵。
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    matrix = np.asarray(rotation_matrix, dtype=np.float64)
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])

    # 按标准公式将旋转矩阵转换为四元数。
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale

    # 返回 ROS 常用的 x, y, z, w 顺序。
    return (float(qx), float(qy), float(qz), float(qw))


# 构造状态 JSON 字符串。
def build_status_payload(result: Any) -> str:
    marker = result.marker
    pose = result.pose
    plan = result.approach_plan
    payload = {
        "status": result.status,
        "message": result.message,
        "marker_id": marker.marker_id if marker is not None else None,
        "distance_m": pose.distance_m if pose is not None and pose.status == "ok" else None,
        "selection_reason": marker.selection_reason if marker is not None else None,
        "target_status": plan.target_status if plan is not None else None,
    }
    return json.dumps(payload, ensure_ascii=False)
