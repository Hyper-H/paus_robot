from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 json，用于把结果打印成结构化文本。
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

# 导入相机标定加载、配置读取、单图处理和结果转换函数。
from ag_repro import load_camera_calibration, load_config, process_image_file, result_to_dict


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Detect one ArUco marker from a single image.")
    # 指定输入图像路径。
    parser.add_argument("--input", required=True, help="Path to one RGB image.")
    # 指定输出目录。
    parser.add_argument("--output-dir", required=True, help="Directory to store JSON, visualization, and debug images.")
    # 指定项目配置文件路径。
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "default.yaml"), help="Path to YAML config.")
    # 可选指定相机标定文件路径。
    parser.add_argument("--camera-config", default=None, help="Optional path to camera calibration YAML.")
    # 返回解析结果。
    return parser.parse_args()


# 单图入口主函数。
def main() -> int:
    # 解析命令行参数。
    args = parse_args()
    # 加载项目配置。
    config = load_config(args.config)
    # 若提供了相机标定文件，则读取它。
    camera_calibration = load_camera_calibration(args.camera_config) if args.camera_config else None
    # 处理单张图片。
    result = process_image_file(args.input, args.output_dir, config, camera_calibration=camera_calibration)
    # 将结果打印到终端，便于快速查看。
    print(json.dumps(result_to_dict(result), indent=2))
    # 根据状态返回退出码。
    return 0 if result.status == "ok" else 1


# 当脚本被直接执行时运行主函数。
if __name__ == "__main__":
    raise SystemExit(main())
