from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 Path，便于处理输出路径。
from pathlib import Path

# 导入 OpenCV，用于生成 ArUco marker 图像。
import cv2


# 收集所有可用的 ArUco 字典常量。
ARUCO_DICTIONARIES = {
    # 遍历 cv2.aruco 下的所有属性。
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    # 只保留 DICT_ 前缀的字典常量。
    if name.startswith("DICT_")
}


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Generate one printable ArUco marker image.")
    # 指定 marker ID。
    parser.add_argument("--marker-id", type=int, required=True, help="Marker ID inside the selected dictionary.")
    # 指定字典名称。
    parser.add_argument("--dictionary", default="DICT_4X4_50", help="OpenCV ArUco dictionary name.")
    # 指定图像像素边长。
    parser.add_argument("--size", type=int, default=600, help="Output marker image size in pixels.")
    # 指定输出路径。
    parser.add_argument("--output", required=True, help="Output image path, for example markers/id7.png.")
    # 返回解析结果。
    return parser.parse_args()


# 根据名字获取字典对象。
def get_dictionary(name: str) -> cv2.aruco.Dictionary:
    # 若字典名不合法，则抛出可读错误。
    if name not in ARUCO_DICTIONARIES:
        available = ", ".join(sorted(ARUCO_DICTIONARIES))
        raise ValueError(f"Unsupported dictionary '{name}'. Available: {available}")
    # 返回对应的 OpenCV 字典对象。
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[name])


# 脚本主函数。
def main() -> int:
    # 解析命令行参数。
    args = parse_args()
    # 读取当前选中的字典。
    dictionary = get_dictionary(args.dictionary)
    # 生成 marker 灰度图。
    marker = cv2.aruco.generateImageMarker(dictionary, args.marker_id, args.size)
    # 标准化输出路径。
    output_path = Path(args.output)
    # 若父目录不存在则自动创建。
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # 将 marker 图像写出到磁盘。
    success = cv2.imwrite(str(output_path), marker)
    # 如果写出失败，则抛出错误。
    if not success:
        raise RuntimeError(f"Failed to write marker image to: {output_path}")
    # 在终端输出生成结果。
    print(f"Saved marker id={args.marker_id} dictionary={args.dictionary} to {output_path}")
    # 正常结束返回 0。
    return 0


# 当脚本被直接执行时运行主函数。
if __name__ == "__main__":
    raise SystemExit(main())
