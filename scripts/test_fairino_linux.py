from __future__ import annotations

# 导入 argparse，用于解析命令行参数。
import argparse
# 导入 json，用于输出结构化测试结果。
import json
# 导入 sys，用于补充包搜索路径。
import sys
# 导入 Path，便于处理项目路径。
from pathlib import Path

# 计算项目根目录。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
# 计算运动包根目录。
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
# 在源码直跑时，把运动包补进搜索路径。
if (MOTION_PACKAGE_ROOT / "setup.py").exists() and str(MOTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTION_PACKAGE_ROOT))

# 导入 Linux FAIRINO SDK 适配层。
from paus_motion_ros2 import FairinoLinuxClient


# 解析命令行参数。
def parse_args() -> argparse.Namespace:
    # 创建参数解析器。
    parser = argparse.ArgumentParser(description="Test the FAIRINO Linux Python SDK directly.")
    # 指定 SDK 根目录。
    parser.add_argument("--sdk-root", default="/opt/fairino_python_sdk/linux", help="Linux FAIRINO SDK root.")
    # 指定机器人控制器 IP。
    parser.add_argument("--robot-ip", default="192.168.58.2", help="Robot controller IP.")
    # 指定要测试的命令类型。
    parser.add_argument("--command", required=True, choices=("connect", "set_speed", "get_joints", "get_tcp", "move_j", "move_l"))
    # `set_speed` 使用的速度。
    parser.add_argument("--speed", type=float, default=5.0, help="Speed for set_speed.")
    # 运动命令用到的 tool / user / vel 参数。
    parser.add_argument("--tool-id", type=int, default=0, help="Tool id for move commands.")
    parser.add_argument("--user-id", type=int, default=0, help="User id for move commands.")
    parser.add_argument("--vel", type=float, default=5.0, help="Velocity for move commands.")
    # `move_j` 所需的 6 关节输入。
    parser.add_argument("--joint-pos", nargs=6, type=float, help="Joint pose for move_j.")
    # `move_l` 所需的 6 维位姿输入。
    parser.add_argument("--pose-mmdeg", nargs=6, type=float, help="Cartesian pose for move_l.")
    # 返回解析结果。
    return parser.parse_args()


# 直连测试主函数。
def main() -> int:
    # 读取命令行参数。
    args = parse_args()
    # 创建客户端并建立连接。
    client = FairinoLinuxClient(args.sdk_root, args.robot_ip)
    client.connect()

    # 仅验证“能否建立连接”。
    if args.command == "connect":
        print(json.dumps({"ok": True, "command": "connect"}))
        return 0
    # 测试设置速度。
    if args.command == "set_speed":
        code = client.set_speed(args.speed)
        print(json.dumps({"ok": code == 0, "command": "set_speed", "code": code}))
        return 0 if code == 0 else 1
    # 测试读取当前关节角。
    if args.command == "get_joints":
        code, joints = client.get_actual_joint_pos_degree()
        print(json.dumps({"ok": code == 0, "command": "get_joints", "code": code, "joint_pos": joints}))
        return 0 if code == 0 else 1
    # 测试读取当前 TCP 位姿。
    if args.command == "get_tcp":
        code, pose = client.get_actual_tcp_pose()
        print(json.dumps({"ok": code == 0, "command": "get_tcp", "code": code, "pose_mmdeg": pose}))
        return 0 if code == 0 else 1
    # 测试关节空间运动。
    if args.command == "move_j":
        if args.joint_pos is None:
            raise SystemExit("--joint-pos is required for move_j")
        code = client.move_j(list(args.joint_pos), tool_id=args.tool_id, user_id=args.user_id, vel=args.vel)
        print(json.dumps({"ok": code == 0, "command": "move_j", "code": code}))
        return 0 if code == 0 else 1
    # 测试笛卡尔空间直线运动。
    if args.command == "move_l":
        if args.pose_mmdeg is None:
            raise SystemExit("--pose-mmdeg is required for move_l")
        code = client.move_l(list(args.pose_mmdeg), tool_id=args.tool_id, user_id=args.user_id, vel=args.vel)
        print(json.dumps({"ok": code == 0, "command": "move_l", "code": code}))
        return 0 if code == 0 else 1

    # 理论上不会走到这里；只是为了保证分支完整。
    raise SystemExit(f"Unsupported command: {args.command}")


# 当脚本被直接执行时，进入主函数。
if __name__ == "__main__":
    raise SystemExit(main())
