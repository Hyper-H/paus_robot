from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 json，用于输出批处理进度和汇总信息。
import json
# 导入 sys，用于补充源码搜索路径。
import sys
# 导入 Path，便于统一处理路径。
from pathlib import Path

# 计算项目根目录。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
# 计算源码目录。
SRC_ROOT = PROJECT_ROOT / "src"
# 若源码目录尚未加入模块搜索路径，则插入进去。
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

# 导入批处理所需的函数。
from ag_repro import (
    SUPPORTED_IMAGE_EXTENSIONS,
    build_summary_record,
    load_camera_calibration,
    load_config,
    make_output_subdir_name,
    process_image_file,
    write_summary_csv,
    write_summary_json,
)


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Run marker detection on a flat image directory.")
    # 指定输入图片目录。
    parser.add_argument("--input-dir", required=True, help="Path to a single-level image directory.")
    # 指定输出目录。
    parser.add_argument("--output-dir", required=True, help="Directory to store summaries and per-image outputs.")
    # 指定项目配置文件路径。
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "default.yaml"), help="Path to YAML config.")
    # 可选指定相机标定文件路径。
    parser.add_argument("--camera-config", default=None, help="Optional path to camera calibration YAML.")
    # 返回解析结果。
    return parser.parse_args()


# 扫描输入目录中的可处理图片。
def iter_image_files(input_dir: Path) -> list[Path]:
    # 遍历输入目录第一层，筛选支持的图片格式并按名字排序。
    return sorted(
        [
            path
            for path in input_dir.iterdir()
            if path.is_file() and path.suffix.lower() in SUPPORTED_IMAGE_EXTENSIONS
        ],
        key=lambda item: item.name.lower(),
    )


# 批处理主函数。
def main() -> int:
    # 解析命令行参数。
    args = parse_args()
    # 加载项目配置。
    config = load_config(args.config)
    # 读取相机标定文件；若为空则保持 None。
    camera_calibration = load_camera_calibration(args.camera_config) if args.camera_config else None
    # 标准化输入目录。
    input_dir = Path(args.input_dir)
    # 标准化输出目录。
    output_dir = Path(args.output_dir)

    # 检查输入目录是否存在。
    if not input_dir.exists() or not input_dir.is_dir():
        raise FileNotFoundError(f"Input directory does not exist: {input_dir}")

    # 收集支持格式的图片列表。
    image_files = iter_image_files(input_dir)
    # 如果没有找到可处理图片，则抛出错误。
    if not image_files:
        raise FileNotFoundError(f"No supported images found in: {input_dir}")

    # 确保输出目录存在。
    output_dir.mkdir(parents=True, exist_ok=True)
    # 保存已经使用过的子目录名。
    used_names: set[str] = set()
    # 保存所有汇总记录。
    summary_records: list[dict[str, object]] = []

    # 逐张图片处理。
    for image_path in image_files:
        # 为当前图片生成稳定的子目录名。
        subdir_name = make_output_subdir_name(image_path, used_names)
        # 计算当前图片的输出目录。
        image_output_dir = output_dir / subdir_name
        # 执行完整处理流程。
        result = process_image_file(image_path, image_output_dir, config, camera_calibration=camera_calibration)
        # 将当前图片记录加入汇总结果。
        summary_records.append(
            build_summary_record(
                image_path,
                result,
                subdir_name,
                camera_yaml_used=str(Path(args.camera_config)) if args.camera_config else "",
            )
        )
        # 输出当前图片的处理状态。
        print(json.dumps({"image": image_path.name, "status": result.status, "output_subdir": subdir_name}, ensure_ascii=False))

    # 写出 JSON 汇总表。
    write_summary_json(summary_records, output_dir / "summary.json")
    # 写出 CSV 汇总表。
    write_summary_csv(summary_records, output_dir / "summary.csv")

    # 统计成功数量。
    success_count = sum(1 for record in summary_records if record["status"] == "ok")
    # 打印整批处理汇总信息。
    print(
        json.dumps(
            {"processed": len(summary_records), "success": success_count, "output_dir": str(output_dir)},
            ensure_ascii=False,
            indent=2,
        )
    )
    # 若至少有一张图成功，则返回 0，否则返回 1。
    return 0 if success_count > 0 else 1


# 当脚本被直接执行时运行主函数。
if __name__ == "__main__":
    raise SystemExit(main())
