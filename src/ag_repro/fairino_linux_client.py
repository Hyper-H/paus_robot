from __future__ import annotations

import os
import sys
from pathlib import Path


class FairinoLinuxClient:
    def __init__(self, sdk_root: str | Path, robot_ip: str) -> None:
        self.sdk_root = Path(sdk_root).expanduser().resolve()
        self.robot_ip = str(robot_ip)
        self.robot = None
        self._robot_module = None

    def _ensure_sdk_importable(self) -> None:
        sdk_root_str = str(self.sdk_root)
        if sdk_root_str not in sys.path:
            sys.path.insert(0, sdk_root_str)
        fairino_pkg = self.sdk_root / "fairino"
        if not fairino_pkg.exists():
            raise FileNotFoundError(f"Linux FAIRINO SDK package directory not found: {fairino_pkg}")

        # 官方 Linux SDK 使用 fairino/Robot.py + build/lib.linux-*/Robot*.so。
        build_dirs = sorted((fairino_pkg / "build").glob("lib.linux-*"))
        for build_dir in reversed(build_dirs):
            build_dir_str = str(build_dir)
            if build_dir_str not in sys.path:
                sys.path.insert(0, build_dir_str)

        from fairino import Robot  # type: ignore

        self._robot_module = Robot

    def connect(self) -> None:
        if self._robot_module is None:
            self._ensure_sdk_importable()
        self.robot = self._robot_module.RPC(self.robot_ip)

    def ensure_connection(self) -> None:
        if self.robot is None:
            self.connect()

    def set_speed(self, speed: float) -> int:
        self.ensure_connection()
        return int(self.robot.SetSpeed(float(speed)))

    def get_actual_joint_pos_degree(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        error, values = self.robot.GetActualJointPosDegree()
        return int(error), [float(value) for value in values]

    def get_actual_tcp_pose(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        error, values = self.robot.GetActualTCPPose()
        return int(error), [float(value) for value in values]

    def move_j(self, joint_pos: list[float], tool_id: int, user_id: int, vel: float) -> int:
        self.ensure_connection()
        return int(self.robot.MoveJ([float(v) for v in joint_pos], tool=int(tool_id), user=int(user_id), vel=float(vel)))

    def move_l(self, pose_mmdeg: list[float], tool_id: int, user_id: int, vel: float) -> int:
        self.ensure_connection()
        return int(self.robot.MoveL([float(v) for v in pose_mmdeg], tool=int(tool_id), user=int(user_id), vel=float(vel)))
