from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 json，用于把结果打印成结构化文本。
import json
# 导入 sys，用于按需补充包搜索路径。
import sys
# 导入 Path，便于统一处理路径。
from pathlib import Path

# 尝试导入 ament 索引工具。
# 如果当前脚本在纯 Python 环境下单独执行，这个导入可能不存在，因此允许降级。
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

# 计算当前 `paus_perception` 包的根目录。
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# 如果脚本是从源码目录直接执行，就把包根目录临时插入 `sys.path`。
# 这样在不经过 `colcon build` 的情况下，也能直接 `import paus_perception`。
if (PACKAGE_ROOT / "setup.py").exists() and str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

# 导入单图处理会用到的核心函数。
from paus_perception import load_camera_calibration, load_config, process_image_file, result_to_dict


# 推导默认配置文件路径。
# 优先从 `paus_bringup` 安装目录中取默认配置；
# 如果当前环境还没有安装包，就回退到源码树中的配置文件。
def _default_config_path() -> str:
    if get_package_share_directory is not None:
        try:
            return str(Path(get_package_share_directory("paus_bringup")) / "configs" / "default.yaml")
        except Exception:
            pass
    return str(PACKAGE_ROOT.parents[1] / "src" / "paus_bringup" / "configs" / "default.yaml")


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Detect one ArUco marker from a single image.")
    # 指定输入图像路径。
    parser.add_argument("--input", required=True, help="Path to one RGB image.")
    # 指定输出目录。
    parser.add_argument("--output-dir", required=True, help="Directory to store JSON, visualization, and debug images.")
    # 指定主配置文件路径。
    parser.add_argument("--config", default=_default_config_path(), help="Path to YAML config.")
    # 可选指定相机标定文件。
    parser.add_argument("--camera-config", default=None, help="Optional path to camera calibration YAML.")
    # 返回解析结果。
    return parser.parse_args()


# 脚本主函数。
def main() -> int:
    # 读取命令行参数。
    args = parse_args()
    # 加载主配置。
    config = load_config(args.config)
    # 如果提供了相机内参文件，则额外加载它。
    camera_calibration = load_camera_calibration(args.camera_config) if args.camera_config else None
    # 执行单张图像处理流程。
    result = process_image_file(args.input, args.output_dir, config, camera_calibration=camera_calibration)
    # 将结果转成 JSON 并打印，方便在命令行直接查看。
    print(json.dumps(result_to_dict(result), indent=2))
    # 若处理成功则返回 0，否则返回 1。
    return 0 if result.status == "ok" else 1


# 当脚本被直接运行时，进入主函数。
if __name__ == "__main__":
    raise SystemExit(main())
