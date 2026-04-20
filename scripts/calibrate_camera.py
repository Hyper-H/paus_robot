from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 json，用于打印标定结果。
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

# 导入标定相关函数和结构。
from ag_repro import calibrate_camera_from_directory, save_camera_calibration


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Calibrate camera from a chessboard image directory.")
    # 指定输入目录。
    parser.add_argument("--input-dir", required=True, help="Directory containing chessboard images.")
    # 指定棋盘格内角点行数。
    parser.add_argument("--rows", required=True, type=int, help="Chessboard inner-corner rows.")
    # 指定棋盘格内角点列数。
    parser.add_argument("--cols", required=True, type=int, help="Chessboard inner-corner cols.")
    # 指定单格真实边长，单位米。
    parser.add_argument("--square-size-m", required=True, type=float, help="Physical square size in meters.")
    # 指定输出标定文件。
    parser.add_argument("--output", required=True, help="Output camera YAML path.")
    # 返回解析结果。
    return parser.parse_args()


# 标定脚本主函数。
def main() -> int:
    # 读取命令行参数。
    args = parse_args()
    # 执行相机标定。
    result = calibrate_camera_from_directory(args.input_dir, args.rows, args.cols, args.square_size_m)
    # 如果标定成功且有内参结果，则保存为 YAML。
    if result.success and result.calibration is not None:
        save_camera_calibration(result.calibration, args.output)
    # 将结构化结果打印出来。
    print(json.dumps(result, default=lambda item: item.__dict__, indent=2))
    # 根据标定是否成功返回退出码。
    return 0 if result.success else 1


# 当脚本被直接执行时运行主函数。
if __name__ == "__main__":
    raise SystemExit(main())
