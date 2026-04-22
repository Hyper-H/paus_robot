from __future__ import annotations

# 导入 csv，用于写批处理汇总表。
import csv
# 导入 json，用于保存批处理汇总结果。
import json
# 导入 re，用于清理输出目录名。
import re
# 导入 Path，便于统一处理路径。
from pathlib import Path
# 导入 Any，便于描述灵活的汇总字段。
from typing import Any

# 导入 OpenCV，用于读取图像与保存可视化结果。
import cv2

# 导入相机标定结果读取函数。
from .calibration import CameraCalibration
# 导入检测相关函数和结果结构。
from .detection import PipelineResult, detect_marker, export_debug_images, render_visualization, save_result_json
# 导入位姿与接近目标规划函数。
from .pose import build_approach_plan, estimate_marker_pose


# 定义批处理支持的图片后缀。
SUPPORTED_IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".jfif", ".png", ".bmp", ".webp", ".tif", ".tiff")


# 对内存中的一张图像执行完整检测、位姿估计与接近目标计算。
def process_image_array(
    image_bgr: Any,
    config: dict[str, Any],
    camera_calibration: CameraCalibration | None = None,
) -> PipelineResult:
    # 先做 2D marker 检测。
    result = detect_marker(image_bgr, config)
    # 读取位姿相关配置。
    pose_config = config["pose"]
    # 若有相机标定，则继续估计 pose。
    result.pose = estimate_marker_pose(result.marker, camera_calibration, float(pose_config["marker_length_m"]))
    # 若 pose 有效，则继续生成接近目标规划结果。
    if result.pose is not None and result.pose.status == "ok":
        result.approach_plan = build_approach_plan(
            result.pose,
            float(pose_config["approach_distance_m"]),
            str(pose_config["target_frame"]),
        )

    # 返回仅在内存中的完整结果。
    return result


# 处理单张图像文件，并输出完整结果。
def process_image_file(
    image_path: str | Path,
    output_dir: str | Path,
    config: dict[str, Any],
    camera_calibration: CameraCalibration | None = None,
) -> PipelineResult:
    # 标准化输入图像路径。
    image_path = Path(image_path)
    # 标准化输出目录。
    output_dir = Path(output_dir)
    # 若目录不存在则自动创建。
    output_dir.mkdir(parents=True, exist_ok=True)

    # 读取原始图像。
    image_bgr = cv2.imread(str(image_path))
    # 如果图像无法读取，则返回读图失败结果。
    if image_bgr is None:
        result = PipelineResult(
            status="read_error",
            marker=None,
            pose=None,
            approach_plan=None,
            image_width=0,
            image_height=0,
            message=f"Failed to read image: {image_path}",
        )
        save_result_json(result, output_dir / "detection_result.json")
        return result

    # 复用内存处理函数，避免文件版和 ROS2 在线版出现两套逻辑。
    result = process_image_array(image_bgr, config, camera_calibration=camera_calibration)

    # 保存结构化 JSON 结果。
    save_result_json(result, output_dir / "detection_result.json")
    # 生成可视化图像。
    visualization = render_visualization(image_bgr, result)
    # 保存可视化结果。
    cv2.imwrite(str(output_dir / "visualization.png"), visualization)

    # 如果开启了调试导出，则保存中间图。
    if config["output"]["export_debug"]:
        export_debug_images(image_bgr, config, result.marker, output_dir)

    # 返回完整流水线结果。
    return result


# 为批处理输出目录生成一个稳定的子目录名。
def make_output_subdir_name(image_path: str | Path, used_names: set[str]) -> str:
    # 将输入路径转成 Path。
    image_path = Path(image_path)
    # 将文件名清理成适合作为目录名的形式。
    base_name = re.sub(r"[^A-Za-z0-9._-]+", "_", image_path.stem).strip("._") or "image"
    # 先尝试直接使用当前文件名。
    candidate = base_name
    # 从后缀 2 开始处理重名。
    suffix = 2
    # 如果名字重复，则不断追加编号。
    while candidate in used_names:
        candidate = f"{base_name}_{suffix}"
        suffix += 1
    # 记录该名字已使用。
    used_names.add(candidate)
    # 返回最终子目录名。
    return candidate


# 构造批处理汇总记录。
def build_summary_record(
    image_path: str | Path,
    result: PipelineResult,
    output_subdir: str,
    camera_yaml_used: str = "",
) -> dict[str, Any]:
    # 标准化输入路径。
    image_path = Path(image_path)
    # 取出 marker 结果。
    marker = result.marker
    # 取出 pose 结果。
    pose = result.pose
    # 返回一条完整记录。
    return {
        "image_name": image_path.name,
        "image_path": str(image_path),
        "status": result.status,
        "message": result.message,
        "marker_id": marker.marker_id if marker is not None else "",
        "selection_reason": marker.selection_reason if marker is not None else "",
        "center_x": marker.center[0] if marker is not None else "",
        "center_y": marker.center[1] if marker is not None else "",
        "angle_deg": marker.angle_deg if marker is not None else "",
        "distance_m": pose.distance_m if pose is not None and pose.status == "ok" else "",
        "tvec_x": pose.tvec[0] if pose is not None and pose.status == "ok" else "",
        "tvec_y": pose.tvec[1] if pose is not None and pose.status == "ok" else "",
        "tvec_z": pose.tvec[2] if pose is not None and pose.status == "ok" else "",
        "rvec_x": pose.rvec[0] if pose is not None and pose.status == "ok" else "",
        "rvec_y": pose.rvec[1] if pose is not None and pose.status == "ok" else "",
        "rvec_z": pose.rvec[2] if pose is not None and pose.status == "ok" else "",
        "image_width": result.image_width,
        "image_height": result.image_height,
        "camera_yaml_used": camera_yaml_used,
        "output_subdir": output_subdir,
    }


# 将批处理汇总记录写成 JSON 文件。
def write_summary_json(records: list[dict[str, Any]], output_path: str | Path) -> None:
    # 标准化输出路径。
    output_path = Path(output_path)
    # 写入 JSON。
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(records, handle, indent=2)


# 将批处理汇总记录写成 CSV 文件。
def write_summary_csv(records: list[dict[str, Any]], output_path: str | Path) -> None:
    # 标准化输出路径。
    output_path = Path(output_path)
    # 定义字段顺序。
    fieldnames = [
        "image_name",
        "image_path",
        "status",
        "message",
        "marker_id",
        "selection_reason",
        "center_x",
        "center_y",
        "angle_deg",
        "distance_m",
        "tvec_x",
        "tvec_y",
        "tvec_z",
        "rvec_x",
        "rvec_y",
        "rvec_z",
        "image_width",
        "image_height",
        "camera_yaml_used",
        "output_subdir",
    ]
    # 打开 CSV 文件并写入表头和内容。
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)
