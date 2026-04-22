from __future__ import annotations

# 导入 json，便于保存结构化结果。
import json
# 导入 math，便于计算角度。
import math
# 导入 asdict 和 dataclass，便于定义结构化结果。
from dataclasses import asdict, dataclass
# 导入 Path，便于处理输出路径。
from pathlib import Path
# 导入 Any，便于描述通用配置字典。
from typing import Any

# 导入 OpenCV，用于图像处理和 ArUco 检测。
import cv2
# 导入 NumPy，用于数组计算。
import numpy as np

# 导入位姿与接近目标规划的数据结构。
from .pose import ApproachPlan, MarkerPose


# 保存单个 marker 检测结果。
@dataclass
class MarkerDetection:
    # 当前 marker 的 ID。
    marker_id: int
    # 四个角点坐标。
    corners: list[list[float]]
    # marker 中心点坐标。
    center: list[float]
    # marker 朝向角度。
    angle_deg: float
    # marker 平均边长，单位是像素。
    average_side_length: float
    # 在多个候选中被选中的原因。
    selection_reason: str


# 保存单张图像流水线结果。
@dataclass
class PipelineResult:
    # 当前处理状态，例如 ok、not_found、read_error。
    status: str
    # 2D marker 检测结果。
    marker: MarkerDetection | None
    # 可选的 3D 位姿结果。
    pose: MarkerPose | None
    # 可选的接近目标规划结果。
    approach_plan: ApproachPlan | None
    # 当前图像宽度。
    image_width: int
    # 当前图像高度。
    image_height: int
    # 可读消息。
    message: str


# 收集 OpenCV 中可用的 ArUco 字典常量。
ARUCO_DICTIONARIES = {
    # 遍历 cv2.aruco 中的所有属性。
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    # 仅保留字典常量。
    if name.startswith("DICT_")
}


# 根据名字获取字典对象。
def _get_aruco_dictionary(dictionary_name: str) -> cv2.aruco.Dictionary:
    # 若名字存在，则取出对应常量。
    try:
        dictionary_id = ARUCO_DICTIONARIES[dictionary_name]
    # 若名字不存在，则抛出包含可选项的错误。
    except KeyError as exc:
        available = ", ".join(sorted(ARUCO_DICTIONARIES))
        raise ValueError(f"Unsupported dictionary '{dictionary_name}'. Available: {available}") from exc
    # 返回 OpenCV 的预定义字典对象。
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


# 计算四边形平均边长。
def _compute_average_side_length(corners: np.ndarray) -> float:
    # 用来保存四条边的长度。
    lengths: list[float] = []
    # 四个角点两两相连形成四条边。
    for index in range(4):
        # 当前角点。
        point_a = corners[index]
        # 下一个角点，最后一个点回到第一个点。
        point_b = corners[(index + 1) % 4]
        # 计算欧氏距离并保存。
        lengths.append(float(np.linalg.norm(point_b - point_a)))
    # 返回平均边长。
    return float(sum(lengths) / len(lengths))


# 根据四角点估计 marker 的朝向角。
def _compute_angle_deg(corners: np.ndarray) -> float:
    # 取上边中点。
    top_mid = (corners[0] + corners[1]) / 2.0
    # 取下边中点。
    bottom_mid = (corners[2] + corners[3]) / 2.0
    # 构造从上到下的方向向量。
    direction = bottom_mid - top_mid
    # 使用 atan2 得到角度值。
    return math.degrees(math.atan2(float(direction[1]), float(direction[0])))


# 计算候选四边形面积。
def _candidate_area(corners: np.ndarray) -> float:
    # 调用 OpenCV 计算轮廓面积。
    return float(cv2.contourArea(corners.astype(np.float32)))


# 兼容不同 OpenCV 版本的 ArUco 检测入口。
def _detect_markers(gray: np.ndarray, dictionary: cv2.aruco.Dictionary) -> tuple[list[np.ndarray], np.ndarray | None]:
    # 新版 OpenCV 提供 ArucoDetector 类。
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners_list, ids, _ = detector.detectMarkers(gray)
        return corners_list, ids

    # 旧版 OpenCV 使用 DetectorParameters_create 和 detectMarkers 函数。
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()
    corners_list, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return corners_list, ids


# 执行 marker 检测主流程。
def detect_marker(image_bgr: np.ndarray, config: dict[str, Any]) -> PipelineResult:
    # 先把 RGB/BGR 图像转为灰度图。
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    # 读取 ArUco 配置。
    aruco_config = config["aruco"]
    # 获取当前使用的字典。
    dictionary = _get_aruco_dictionary(aruco_config["dictionary"])
    # 执行兼容不同 OpenCV 版本的 marker 检测。
    corners_list, ids = _detect_markers(gray, dictionary)

    # 如果没有找到任何 marker，则返回未找到状态。
    if ids is None or len(ids) == 0:
        return PipelineResult(
            status="not_found",
            marker=None,
            pose=None,
            approach_plan=None,
            image_width=image_bgr.shape[1],
            image_height=image_bgr.shape[0],
            message="No marker detected.",
        )

    # 从配置中读取目标 ID。
    target_id = aruco_config.get("target_id")
    # 将 OpenCV 的 ids 结果拉平成普通整数列表。
    flat_ids = [int(item[0]) for item in ids.tolist()]

    # 如果指定了目标 ID 且当前结果中存在它，则优先选它。
    if target_id is not None and target_id in flat_ids:
        selected_index = flat_ids.index(int(target_id))
        selection_reason = f"matched_target_id:{target_id}"
    else:
        # 否则退化为选择面积最大的候选 marker。
        areas = [_candidate_area(corners.reshape(4, 2)) for corners in corners_list]
        selected_index = int(np.argmax(areas))
        selection_reason = "largest_area"

    # 取出当前选中的四角点并整理为 4x2 浮点数组。
    selected_corners = corners_list[selected_index].reshape(4, 2).astype(np.float32)
    # 获取当前选中的 marker ID。
    marker_id = flat_ids[selected_index]
    # 计算平均边长。
    average_side_length = _compute_average_side_length(selected_corners)
    # 计算中心点。
    center = selected_corners.mean(axis=0)

    # 整理 marker 检测结果。
    marker = MarkerDetection(
        marker_id=marker_id,
        corners=[[float(x), float(y)] for x, y in selected_corners.tolist()],
        center=[float(center[0]), float(center[1])],
        angle_deg=float(_compute_angle_deg(selected_corners)),
        average_side_length=average_side_length,
        selection_reason=selection_reason,
    )

    # 返回完整的 2D 检测结果。
    return PipelineResult(
        status="ok",
        marker=marker,
        pose=None,
        approach_plan=None,
        image_width=image_bgr.shape[1],
        image_height=image_bgr.shape[0],
        message="Marker detected successfully.",
    )


# 将结果转为普通字典，便于写入 JSON。
def result_to_dict(result: PipelineResult) -> dict[str, Any]:
    # 直接使用 dataclass 的 asdict。
    return asdict(result)


# 将结果保存到 JSON 文件。
def save_result_json(result: PipelineResult, output_path: str | Path) -> None:
    # 标准化输出路径。
    output_path = Path(output_path)
    # 打开文件并保存 JSON。
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(result_to_dict(result), handle, indent=2)


# 计算调试视图，便于理解论文中的传统视觉步骤。
def compute_debug_views(image_bgr: np.ndarray, config: dict[str, Any], marker: MarkerDetection | None) -> dict[str, np.ndarray]:
    # 先将输入图像转灰度。
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    # 读取 ArUco 相关配置。
    aruco_config = config["aruco"]
    # 读取局部自适应阈值窗口大小。
    block_size = int(aruco_config["adaptive_thresh_block_size"])
    # 若窗口大小为偶数，则调整为奇数。
    if block_size % 2 == 0:
        block_size += 1
    # 计算自适应阈值二值图。
    adaptive = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size,
        float(aruco_config["adaptive_thresh_constant"]),
    )

    # 在二值图上提取轮廓。
    contours, _ = cv2.findContours(adaptive, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # 将灰度图转成 BGR，便于绘制彩色轮廓。
    contour_visualization = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    # 用于保存筛选通过的四边形候选。
    candidates: list[np.ndarray] = []

    # 遍历所有轮廓。
    for contour in contours:
        # 计算面积。
        area = cv2.contourArea(contour)
        # 面积太小则跳过。
        if area < float(aruco_config["min_candidate_area"]):
            continue
        # 计算周长。
        perimeter = cv2.arcLength(contour, True)
        # 计算多边形近似参数。
        epsilon = float(aruco_config["approx_epsilon_ratio"]) * perimeter
        # 近似成多边形。
        polygon = cv2.approxPolyDP(contour, epsilon, True)
        # 只保留凸四边形候选。
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            continue
        # 将候选加入列表。
        candidates.append(polygon.reshape(4, 2))

    # 在可视化图中绘制候选四边形。
    cv2.drawContours(
        contour_visualization,
        [candidate.astype(np.int32) for candidate in candidates],
        -1,
        (0, 255, 0),
        int(config["output"]["contour_line_thickness"]),
    )

    # 初始化调试输出字典。
    debug_views: dict[str, np.ndarray] = {
        "gray": gray,
        "adaptive_threshold": adaptive,
        "candidate_contours": contour_visualization,
    }

    # 如果已经检测到了 marker，则额外导出透视展开后的 Otsu 二值图。
    if marker is not None:
        debug_views["marker_otsu"] = _extract_marker_otsu(
            image_bgr,
            np.asarray(marker.corners, dtype=np.float32),
            int(config["output"]["debug_square_size"]),
        )

    # 返回所有调试图。
    return debug_views


# 将 marker 区域透视拉正并做 Otsu 二值化。
def _extract_marker_otsu(image_bgr: np.ndarray, corners: np.ndarray, square_size: int) -> np.ndarray:
    # 构造目标正方形的四角点。
    destination = np.array(
        [
            [0, 0],
            [square_size - 1, 0],
            [square_size - 1, square_size - 1],
            [0, square_size - 1],
        ],
        dtype=np.float32,
    )
    # 计算透视变换矩阵。
    transform = cv2.getPerspectiveTransform(corners, destination)
    # 对原图中的 marker 区域做透视展开。
    warped = cv2.warpPerspective(image_bgr, transform, (square_size, square_size))
    # 转灰度。
    warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    # 用 Otsu 算法做全局二值化。
    _, otsu = cv2.threshold(warped_gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # 返回二值图。
    return otsu


# 将调试图批量保存到输出目录。
def export_debug_images(image_bgr: np.ndarray, config: dict[str, Any], marker: MarkerDetection | None, output_dir: str | Path) -> None:
    # 标准化输出目录。
    output_dir = Path(output_dir)
    # 如果目录不存在则创建。
    output_dir.mkdir(parents=True, exist_ok=True)
    # 计算所有调试视图。
    debug_views = compute_debug_views(image_bgr, config, marker)
    # 逐张保存图像。
    for name, view in debug_views.items():
        cv2.imwrite(str(output_dir / f"{name}.png"), view)


# 在原图上绘制当前检测结果。
def render_visualization(image_bgr: np.ndarray, result: PipelineResult) -> np.ndarray:
    # 复制一份原图，避免就地修改。
    canvas = image_bgr.copy()
    # 如果存在 marker，则绘制四边形、中心点和文字标签。
    if result.marker is not None:
        # 准备整数角点用于绘图。
        corners = np.asarray(result.marker.corners, dtype=np.int32)
        # 绘制 marker 外框。
        cv2.polylines(canvas, [corners], True, (0, 255, 0), 2)
        # 计算中心点。
        center = tuple(int(round(value)) for value in result.marker.center)
        # 绘制中心点。
        cv2.circle(canvas, center, 5, (0, 0, 255), -1)
        # 生成主文本标签。
        label = f"ID {result.marker.marker_id} | angle {result.marker.angle_deg:.1f}"
        # 如果 pose 可用，则追加距离信息。
        if result.pose is not None and result.pose.status == "ok":
            label += f" | dist {result.pose.distance_m:.3f}m"
        # 绘制文本。
        cv2.putText(
            canvas,
            label,
            (center[0] + 10, max(20, center[1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    # 如果存在接近目标规划，则在左上角额外标注目标点信息。
    if result.approach_plan is not None and result.approach_plan.target_status == "ok":
        target = result.approach_plan.target_point
        cv2.putText(
            canvas,
            f"target ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) m",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )
    # 返回叠加结果图。
    return canvas
