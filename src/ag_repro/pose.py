from __future__ import annotations

# 导入 dataclass，便于定义结构化位姿与规划结果。
from dataclasses import dataclass
# 导入 math，便于计算欧氏距离。
import math
# 导入 TYPE_CHECKING，便于只在类型检查时引用检测结果类型。
from typing import TYPE_CHECKING

# 导入 OpenCV，用于求解位姿。
import cv2
# 导入 NumPy，用于矩阵和向量运算。
import numpy as np

# 导入相机标定结果类型。
from .calibration import CameraCalibration

# 仅在类型检查阶段导入 MarkerDetection，避免运行时循环导入。
if TYPE_CHECKING:
    from .detection import MarkerDetection


# 保存 marker 相对相机位姿的数据结构。
@dataclass
class MarkerPose:
    # 旋转向量。
    rvec: list[float]
    # 平移向量。
    tvec: list[float]
    # 相机到 marker 中心的欧氏距离。
    distance_m: float
    # 当前位姿结果所在坐标系。
    frame: str
    # 位姿估计状态。
    status: str
    # 可读说明。
    message: str


# 保存机械臂接近目标规划结果的数据结构。
@dataclass
class ApproachPlan:
    # 当前目标点所在坐标系。
    target_frame: str
    # 目标点坐标。
    target_point: list[float]
    # 建议的停靠距离。
    approach_distance_m: float
    # 当前规划状态。
    target_status: str
    # 可读说明。
    message: str


# 构造 marker 的三维角点模型。
def _build_marker_object_points(marker_length_m: float) -> np.ndarray:
    # 取边长的一半，方便以 marker 中心为原点定义四角点。
    half_length = float(marker_length_m) / 2.0
    # 返回 marker 四个角点的三维坐标，坐标系位于 marker 平面上。
    return np.array(
        [
            [-half_length, half_length, 0.0],
            [half_length, half_length, 0.0],
            [half_length, -half_length, 0.0],
            [-half_length, -half_length, 0.0],
        ],
        dtype=np.float32,
    )


# 根据检测到的四角点和相机内参估计 marker 位姿。
def estimate_marker_pose(marker: MarkerDetection | None, calibration: CameraCalibration | None, marker_length_m: float) -> MarkerPose | None:
    # 如果没有检测到 marker，则无法估计位姿。
    if marker is None:
        return None
    # 如果没有相机标定结果，则无法做真实位姿估计。
    if calibration is None:
        return None

    # 构造 marker 的三维角点。
    object_points = _build_marker_object_points(marker_length_m)
    # 构造图像中的二维角点。
    image_points = np.asarray(marker.corners, dtype=np.float32)
    # 将内参矩阵转成 NumPy 数组。
    camera_matrix = np.asarray(calibration.camera_matrix, dtype=np.float64)
    # 将畸变参数转成 NumPy 数组。
    dist_coeffs = np.asarray(calibration.dist_coeffs, dtype=np.float64)
    # 使用 solvePnP 估计位姿。
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    # 如果求解失败，则返回失败说明。
    if not success:
        return MarkerPose(
            rvec=[],
            tvec=[],
            distance_m=0.0,
            frame="camera",
            status="pose_failed",
            message="Pose estimation failed.",
        )

    # 将结果拍平成一维列表。
    rvec_values = np.asarray(rvec, dtype=np.float64).reshape(-1)
    tvec_values = np.asarray(tvec, dtype=np.float64).reshape(-1)
    # 计算相机到 marker 中心的距离。
    distance_m = math.sqrt(float(np.dot(tvec_values, tvec_values)))
    # 返回成功的位姿结果。
    return MarkerPose(
        rvec=[float(value) for value in rvec_values.tolist()],
        tvec=[float(value) for value in tvec_values.tolist()],
        distance_m=float(distance_m),
        frame="camera",
        status="ok",
        message="Pose estimated successfully.",
    )


# 基于 marker 位姿生成一个接近目标点。
def build_approach_plan(pose: MarkerPose | None, approach_distance_m: float, target_frame: str) -> ApproachPlan | None:
    # 若没有位姿信息，则无法生成规划结果。
    if pose is None or pose.status != "ok":
        return None

    # 将平移向量转成 NumPy 向量。
    translation = np.asarray(pose.tvec, dtype=np.float64)
    # 计算 marker 相对相机的距离。
    distance_m = float(np.linalg.norm(translation))
    # 如果距离过小，则退化为直接输出 marker 中心。
    if distance_m <= 1e-9:
        target_point = translation
    else:
        # 计算单位方向向量，表示从相机指向 marker 的方向。
        direction = translation / distance_m
        # 目标点定义为沿着视线方向停在 marker 前方固定距离的位置。
        target_distance = max(distance_m - float(approach_distance_m), 0.0)
        target_point = direction * target_distance

    # 返回结构化规划结果。
    return ApproachPlan(
        target_frame=str(target_frame),
        target_point=[float(value) for value in target_point.tolist()],
        approach_distance_m=float(approach_distance_m),
        target_status="ok",
        message="Approach target computed successfully.",
    )
