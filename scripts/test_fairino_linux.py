from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
if (MOTION_PACKAGE_ROOT / "setup.py").exists() and str(MOTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTION_PACKAGE_ROOT))

from paus_motion_ros2 import FairinoLinuxClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test the FAIRINO Linux Python SDK directly.")
    parser.add_argument("--sdk-root", default="/opt/fairino_python_sdk/linux", help="Linux FAIRINO SDK root.")
    parser.add_argument("--robot-ip", default="192.168.58.2", help="Robot controller IP.")
    parser.add_argument("--command", required=True, choices=("connect", "set_speed", "get_joints", "get_tcp", "move_j", "move_l"))
    parser.add_argument("--speed", type=float, default=5.0, help="Speed for set_speed.")
    parser.add_argument("--tool-id", type=int, default=0, help="Tool id for move commands.")
    parser.add_argument("--user-id", type=int, default=0, help="User id for move commands.")
    parser.add_argument("--vel", type=float, default=5.0, help="Velocity for move commands.")
    parser.add_argument("--joint-pos", nargs=6, type=float, help="Joint pose for move_j.")
    parser.add_argument("--pose-mmdeg", nargs=6, type=float, help="Cartesian pose for move_l.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = FairinoLinuxClient(args.sdk_root, args.robot_ip)
    client.connect()

    if args.command == "connect":
        print(json.dumps({"ok": True, "command": "connect"}))
        return 0
    if args.command == "set_speed":
        code = client.set_speed(args.speed)
        print(json.dumps({"ok": code == 0, "command": "set_speed", "code": code}))
        return 0 if code == 0 else 1
    if args.command == "get_joints":
        code, joints = client.get_actual_joint_pos_degree()
        print(json.dumps({"ok": code == 0, "command": "get_joints", "code": code, "joint_pos": joints}))
        return 0 if code == 0 else 1
    if args.command == "get_tcp":
        code, pose = client.get_actual_tcp_pose()
        print(json.dumps({"ok": code == 0, "command": "get_tcp", "code": code, "pose_mmdeg": pose}))
        return 0 if code == 0 else 1
    if args.command == "move_j":
        if args.joint_pos is None:
            raise SystemExit("--joint-pos is required for move_j")
        code = client.move_j(list(args.joint_pos), tool_id=args.tool_id, user_id=args.user_id, vel=args.vel)
        print(json.dumps({"ok": code == 0, "command": "move_j", "code": code}))
        return 0 if code == 0 else 1
    if args.command == "move_l":
        if args.pose_mmdeg is None:
            raise SystemExit("--pose-mmdeg is required for move_l")
        code = client.move_l(list(args.pose_mmdeg), tool_id=args.tool_id, user_id=args.user_id, vel=args.vel)
        print(json.dumps({"ok": code == 0, "command": "move_l", "code": code}))
        return 0 if code == 0 else 1

    raise SystemExit(f"Unsupported command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
