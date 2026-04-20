from __future__ import annotations

# 导入 dataclass，便于定义结构化标定结果。
from dataclasses import asdict, dataclass
# 导入 Path，便于处理输入目录和输出文件路径。
from pathlib import Path
# 导入 Any，便于描述读取后的 YAML 结构。
from typing import Any

# 导入 OpenCV，用于棋盘格角点检测和相机标定。
import cv2
# 导入 NumPy，用于组织标定点集。
import numpy as np
# 导入 yaml，用于保存和读取标定结果。
import yaml


# 保存相机内参结果的数据结构。
@dataclass
class CameraCalibration:
    # 标定图像宽度。
    image_width: int
    # 标定图像高度。
    image_height: int
    # 相机内参矩阵。
    camera_matrix: list[list[float]]
    # 畸变参数。
    dist_coeffs: list[float]
    # 重投影误差。
    reprojection_error: float
    # 棋盘格的行数（内角点）。
    board_rows: int
    # 棋盘格的列数（内角点）。
    board_cols: int
    # 棋盘格单格真实边长，单位为米。
    square_size_m: float
    # 可选的外参旋转矩阵。
    rotation_matrix: list[float] | None = None
    # 可选的外参平移向量。
    translation_vector: list[float] | None = None
    # 当前标定结果来源。
    source_type: str = "manual_calibration"
    # 可选的来源文件路径。
    source_path: str = ""


# 保存完整标定过程结果的数据结构。
@dataclass
class CalibrationResult:
    # 标定是否成功。
    status: str
    # 标定是否真正得到可用内参。
    success: bool
    # 标定图像总数。
    image_count: int
    # 成功检测棋盘格的图像数。
    valid_image_count: int
    # 可读的状态描述。
    message: str
    # 成功时的相机标定结果。
    calibration: CameraCalibration | None


# 支持的图片后缀。
SUPPORTED_CALIBRATION_EXTENSIONS = (".jpg", ".jpeg", ".jfif", ".png", ".bmp", ".tif", ".tiff", ".webp")


# 生成棋盘格三维角点模板。
def _build_object_points(board_rows: int, board_cols: int, square_size_m: float) -> np.ndarray:
    # 先创建一个大小为 rows*cols 的三维点数组。
    object_points = np.zeros((board_rows * board_cols, 3), np.float32)
    # 在平面上生成规则网格，z 轴恒为 0。
    object_points[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    # 将网格尺度乘上真实格子大小，得到真实世界坐标。
    object_points *= float(square_size_m)
    # 返回三维点模板。
    return object_points


# 从目录中筛选可用的标定图片。
def _iter_calibration_images(input_dir: Path) -> list[Path]:
    # 遍历目录第一层并按名字排序。
    return sorted(
        [
            path
            for path in input_dir.iterdir()
            if path.is_file() and path.suffix.lower() in SUPPORTED_CALIBRATION_EXTENSIONS
        ],
        key=lambda item: item.name.lower(),
    )


# 将标定结果保存为 YAML 文件。
def save_camera_calibration(calibration: CameraCalibration, output_path: str | Path) -> None:
    # 标准化输出路径。
    output_path = Path(output_path)
    # 如果父目录不存在则自动创建。
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # 整理成更清晰的 YAML 结构。
    payload = {
        "camera": {
            "image_width": calibration.image_width,
            "image_height": calibration.image_height,
            "camera_matrix": calibration.camera_matrix,
            "dist_coeffs": [calibration.dist_coeffs],
            "reprojection_error": calibration.reprojection_error,
        },
        "board": {
            "rows": calibration.board_rows,
            "cols": calibration.board_cols,
            "square_size_m": calibration.square_size_m,
        },
        "extrinsic": {
            "rotation_matrix": calibration.rotation_matrix,
            "translation_vector": calibration.translation_vector,
        },
        "source": {
            "type": calibration.source_type,
            "path": calibration.source_path,
        },
    }
    # 写入 YAML 文件。
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)


# 从 YAML 中读取相机标定结果。
def load_camera_calibration(input_path: str | Path) -> CameraCalibration:
    # 标准化输入路径。
    input_path = Path(input_path)
    # 读取 YAML 内容。
    with input_path.open("r", encoding="utf-8") as handle:
        payload: dict[str, Any] = yaml.safe_load(handle) or {}
    # 读取相机部分字段。
    camera_payload = payload.get("camera", {})
    # 读取棋盘格部分字段。
    board_payload = payload.get("board", {})
    # 读取外参部分字段。
    extrinsic_payload = payload.get("extrinsic", {})
    # 读取来源部分字段。
    source_payload = payload.get("source", {})
    # 返回结构化的相机标定结果。
    return CameraCalibration(
        image_width=int(camera_payload["image_width"]),
        image_height=int(camera_payload["image_height"]),
        camera_matrix=[[float(value) for value in row] for row in camera_payload["camera_matrix"]],
        dist_coeffs=[float(value) for value in camera_payload["dist_coeffs"][0]],
        reprojection_error=float(camera_payload["reprojection_error"]),
        board_rows=int(board_payload["rows"]),
        board_cols=int(board_payload["cols"]),
        square_size_m=float(board_payload["square_size_m"]),
        rotation_matrix=[float(value) for value in (extrinsic_payload.get("rotation_matrix") or [])] or None,
        translation_vector=[float(value) for value in (extrinsic_payload.get("translation_vector") or [])] or None,
        source_type=str(source_payload.get("type", "manual_calibration")),
        source_path=str(source_payload.get("path", "")),
    )


# 核心标定函数：从棋盘格图片目录估计相机内参。
def calibrate_camera_from_directory(input_dir: str | Path, board_rows: int, board_cols: int, square_size_m: float) -> CalibrationResult:
    # 将输入目录转换为 Path。
    input_dir = Path(input_dir)
    # 若目录不存在，则立即返回失败结果。
    if not input_dir.exists() or not input_dir.is_dir():
        return CalibrationResult(
            status="invalid_input",
            success=False,
            image_count=0,
            valid_image_count=0,
            message=f"Input directory does not exist: {input_dir}",
            calibration=None,
        )

    # 找出目录中的所有候选图片。
    image_paths = _iter_calibration_images(input_dir)
    # 若没有找到图片，也直接返回失败结果。
    if not image_paths:
        return CalibrationResult(
            status="no_images",
            success=False,
            image_count=0,
            valid_image_count=0,
            message=f"No calibration images found in: {input_dir}",
            calibration=None,
        )

    # 构造统一的棋盘格三维点模板。
    object_point_template = _build_object_points(board_rows, board_cols, square_size_m)
    # 保存所有图像对应的三维点。
    object_points: list[np.ndarray] = []
    # 保存所有图像对应的二维角点。
    image_points: list[np.ndarray] = []
    # 记录图像大小。
    image_size: tuple[int, int] | None = None

    # 定义亚像素优化的停止条件。
    termination_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    # 逐张处理标定图片。
    for image_path in image_paths:
        # 读取图像。
        image_bgr = cv2.imread(str(image_path))
        # 读图失败则跳过。
        if image_bgr is None:
            continue
        # 转灰度图用于角点检测。
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        # 检测棋盘格内角点。
        found, corners = cv2.findChessboardCorners(gray, (board_cols, board_rows))
        # 未找到棋盘格则跳过当前图像。
        if not found:
            continue
        # 对检测到的角点做亚像素优化。
        refined_corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            termination_criteria,
        )
        # 记录当前图像大小。
        image_size = (gray.shape[1], gray.shape[0])
        # 加入当前图像对应的三维点和二维点。
        object_points.append(object_point_template.copy())
        image_points.append(refined_corners)

    # 如果有效图像不足，则无法稳定标定。
    if len(image_points) < 3 or image_size is None:
        return CalibrationResult(
            status="insufficient_views",
            success=False,
            image_count=len(image_paths),
            valid_image_count=len(image_points),
            message="Not enough valid chessboard views for calibration.",
            calibration=None,
        )

    # 调用 OpenCV 执行相机标定。
    reprojection_error, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    # 将标定结果整理成数据结构。
    calibration = CameraCalibration(
        image_width=int(image_size[0]),
        image_height=int(image_size[1]),
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64).tolist(),
        dist_coeffs=np.asarray(dist_coeffs, dtype=np.float64).reshape(-1).tolist(),
        reprojection_error=float(reprojection_error),
        board_rows=int(board_rows),
        board_cols=int(board_cols),
        square_size_m=float(square_size_m),
    )

    # 返回成功结果。
    return CalibrationResult(
        status="ok",
        success=True,
        image_count=len(image_paths),
        valid_image_count=len(image_points),
        message="Camera calibration completed successfully.",
        calibration=calibration,
    )
