from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 Path，便于处理输出路径。
from pathlib import Path

# 导入 OpenCV，用于生成 ArUco marker 图像。
import cv2
# 导入 NumPy，用于兼容旧版 OpenCV 的 `drawMarker` 输出方式。
import numpy as np

# 收集 OpenCV 当前版本支持的全部 ArUco 字典常量。
ARUCO_DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_")
}


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Generate one printable ArUco marker image.")
    # 指定要生成的 marker 编号。
    parser.add_argument("--marker-id", type=int, required=True, help="Marker ID inside the selected dictionary.")
    # 指定使用哪个字典。
    parser.add_argument("--dictionary", default="DICT_4X4_50", help="OpenCV ArUco dictionary name.")
    # 指定输出图像边长（像素）。
    parser.add_argument("--size", type=int, default=600, help="Output marker image size in pixels.")
    # 指定输出路径。
    parser.add_argument("--output", required=True, help="Output image path, for example markers/id7.png.")
    # 返回解析结果。
    return parser.parse_args()


# 根据名字取回 OpenCV 字典对象。
def get_dictionary(name: str) -> cv2.aruco.Dictionary:
    # 如果字典名不支持，则抛出可读错误。
    if name not in ARUCO_DICTIONARIES:
        available = ", ".join(sorted(ARUCO_DICTIONARIES))
        raise ValueError(f"Unsupported dictionary '{name}'. Available: {available}")
    # 返回 OpenCV 预定义字典。
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[name])


# 生成 marker 图像的主函数。
def main() -> int:
    # 读取命令行参数。
    args = parse_args()
    # 获取字典对象。
    dictionary = get_dictionary(args.dictionary)
    # 新版 OpenCV 提供 `generateImageMarker`；
    # 旧版则需要先创建空白数组，再通过 `drawMarker` 写入。
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, args.marker_id, args.size)
    else:
        marker = np.zeros((args.size, args.size), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, args.marker_id, args.size, marker, 1)
    # 统一输出路径。
    output_path = Path(args.output)
    # 自动创建输出目录。
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # 将 marker 写到磁盘。
    success = cv2.imwrite(str(output_path), marker)
    # 写出失败时立即报错。
    if not success:
        raise RuntimeError(f"Failed to write marker image to: {output_path}")
    # 在命令行打印生成结果，方便快速确认。
    print(f"Saved marker id={args.marker_id} dictionary={args.dictionary} to {output_path}")
    # 正常结束返回 0。
    return 0


# 当脚本被直接执行时，进入主函数。
if __name__ == "__main__":
    raise SystemExit(main())
