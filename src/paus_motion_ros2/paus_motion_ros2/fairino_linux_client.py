from __future__ import annotations

# 导入 os / sys / Path，用于动态组织 Linux SDK 的导入路径。
import os
import sys
from pathlib import Path
from collections.abc import Sequence


# 这个类负责把官方 Linux FAIRINO Python SDK 包装成更稳定的项目内部接口。
# 上层节点只需要调用 connect / move_j / move_l 等方法，不需要关心 SDK 路径细节。
class FairinoLinuxClient:
    # 初始化客户端，保存 SDK 根目录和机器人 IP。
    def __init__(self, sdk_root: str | Path, robot_ip: str) -> None:
        self.sdk_root = Path(sdk_root).expanduser().resolve()
        self.robot_ip = str(robot_ip)
        self.robot = None
        self._robot_module = None

    # 确保官方 SDK 所在目录已经加入 Python 搜索路径，并且成功导入 `fairino.Robot`。
    def _ensure_sdk_importable(self) -> None:
        sdk_root_str = str(self.sdk_root)
        # 把 SDK 根目录加到 `sys.path`。
        if sdk_root_str not in sys.path:
            sys.path.insert(0, sdk_root_str)
        # 校验 SDK 包目录是否真的存在。
        fairino_pkg = self.sdk_root / "fairino"
        if not fairino_pkg.exists():
            raise FileNotFoundError(f"Linux FAIRINO SDK package directory not found: {fairino_pkg}")

        # 官方 Linux SDK 一般会把编译后的 so 放在 `build/lib.linux-*` 里。
        # 这里把这些目录也加入 `sys.path`，避免动态库导入失败。
        build_dirs = sorted((fairino_pkg / "build").glob("lib.linux-*"))
        for build_dir in reversed(build_dirs):
            build_dir_str = str(build_dir)
            if build_dir_str not in sys.path:
                sys.path.insert(0, build_dir_str)

        # 真正导入官方 SDK 模块。
        from fairino import Robot  # type: ignore

        self._robot_module = Robot

    # 建立到机器人控制器的 RPC 连接。
    def connect(self) -> None:
        # 如有需要，先准备 SDK 导入环境。
        if self._robot_module is None:
            self._ensure_sdk_importable()
        # 通过控制器 IP 创建 RPC 客户端对象。
        self.robot = self._robot_module.RPC(self.robot_ip)

    # 确保当前已经有可用连接；如果还没有，就自动连接。
    def ensure_connection(self) -> None:
        if self.robot is None:
            self.connect()

    # 设置机器人速度倍率。
    def set_speed(self, speed: float) -> int:
        self.ensure_connection()
        return int(self.robot.SetSpeed(float(speed)))

    # 读取当前关节角，单位为度。
    def get_actual_joint_pos_degree(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        error, values = self.robot.GetActualJointPosDegree()
        return int(error), [float(value) for value in values]

    # 读取当前 TCP 位姿，位置单位为 mm，姿态单位为 deg。
    def get_actual_tcp_pose(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        try:
            result = self.robot.GetActualTCPPose()
        except TypeError as exc:
            # Some SDK versions read a UDP state struct that may still be the
            # ctypes class, not an instance. Fall back to the blocking XML-RPC
            # call used in FAIRINO's own commented implementation.
            if "_ctypes.CField" not in str(exc) or not hasattr(self.robot, "robot"):
                raise
            result = self.robot.robot.GetActualTCPPose(0)
        return self._normalize_pose_result(result, "GetActualTCPPose")

    def _normalize_pose_result(self, result: object, method_name: str) -> tuple[int, list[float]]:
        if not isinstance(result, Sequence):
            raise RuntimeError(f"{method_name} returned non-sequence result: {result!r}")
        if len(result) == 2 and isinstance(result[1], Sequence):
            error = int(result[0])
            values = result[1]
        elif len(result) >= 7:
            error = int(result[0])
            values = result[1:7]
        else:
            raise RuntimeError(f"{method_name} returned unexpected result: {result!r}")
        return error, [float(value) for value in values]

    # 执行关节空间运动。
    def move_j(self, joint_pos: list[float], tool_id: int, user_id: int, vel: float) -> int:
        self.ensure_connection()
        return int(self.robot.MoveJ([float(v) for v in joint_pos], tool=int(tool_id), user=int(user_id), vel=float(vel)))

    # 执行笛卡尔空间直线运动。
    def move_l(self, pose_mmdeg: list[float], tool_id: int, user_id: int, vel: float) -> int:
        self.ensure_connection()
        return int(self.robot.MoveL([float(v) for v in pose_mmdeg], tool=int(tool_id), user=int(user_id), vel=float(vel)))
