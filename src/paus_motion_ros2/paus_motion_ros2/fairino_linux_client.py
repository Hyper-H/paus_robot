from __future__ import annotations

# 导入 os / sys / Path，用于动态组织 Linux SDK 的导入路径。
import os
import sys
from pathlib import Path
from collections.abc import Sequence
from xmlrpc.client import Fault


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

    def reset_all_error(self) -> int:
        self.ensure_connection()
        return int(self.robot.ResetAllError())

    def robot_enable(self, enabled: bool = True) -> int:
        self.ensure_connection()
        return int(self.robot.RobotEnable(1 if enabled else 0))

    def set_auto_mode(self) -> int:
        self.ensure_connection()
        return int(self.robot.Mode(0))

    def set_manual_mode(self) -> int:
        self.ensure_connection()
        return int(self.robot.Mode(1))

    def prepare_motion(self) -> dict[str, int]:
        reset_error = self.reset_all_error()
        if reset_error != 0:
            return {"ResetAllError": reset_error}
        enable_error = self.robot_enable(True)
        if enable_error != 0:
            return {"ResetAllError": reset_error, "RobotEnable": enable_error}
        mode_error = self.set_auto_mode()
        return {"ResetAllError": reset_error, "RobotEnable": enable_error, "Mode": mode_error}

    # 读取当前关节角，单位为度。
    def get_actual_joint_pos_degree(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.GetActualJointPosDegree()
        return self._normalize_pose_result(result, "GetActualJointPosDegree")

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

    def get_actual_tcp_num(self) -> tuple[int, int | None]:
        self.ensure_connection()
        result = self.robot.GetActualTCPNum(0)
        return self._normalize_int_result(result, "GetActualTCPNum")

    def get_cur_tool_coord(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.GetCurToolCoord()
        return self._normalize_pose_result(result, "GetCurToolCoord")

    def get_tool_coord_with_id(self, tool_id: int) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.GetToolCoordWithID(int(tool_id))
        return self._normalize_pose_result(result, "GetToolCoordWithID")

    def get_tcp_offset(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.GetTCPOffset(0)
        return self._normalize_pose_result(result, "GetTCPOffset")

    def get_actual_tool_flange_pose(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        try:
            result = self.robot.GetActualToolFlangePose()
        except TypeError as exc:
            if "_ctypes.CField" not in str(exc) or not hasattr(self.robot, "robot"):
                raise
            result = self.robot.robot.GetActualToolFlangePose(0)
        return self._normalize_pose_result(result, "GetActualToolFlangePose")

    def tool_trsf_start(self, tool_id: int) -> int:
        self.ensure_connection()
        return int(self.robot.ToolTrsfStart(int(tool_id)))

    def tool_trsf_end(self) -> int:
        self.ensure_connection()
        return int(self.robot.ToolTrsfEnd())

    def _normalize_pose_result(self, result: object, method_name: str) -> tuple[int, list[float]]:
        if not isinstance(result, Sequence):
            raise RuntimeError(f"{method_name} returned non-sequence result: {result!r}")
        if len(result) == 2 and result[1] is None:
            return int(result[0]), []
        if len(result) == 2 and isinstance(result[1], Sequence):
            error = int(result[0])
            values = result[1]
        elif len(result) >= 7:
            error = int(result[0])
            values = result[1:7]
        else:
            raise RuntimeError(f"{method_name} returned unexpected result: {result!r}")
        return error, [float(value) for value in values]

    def _normalize_int_result(self, result: object, method_name: str) -> tuple[int, int | None]:
        if isinstance(result, int):
            return int(result), None
        if not isinstance(result, Sequence):
            raise RuntimeError(f"{method_name} returned non-sequence result: {result!r}")
        if len(result) >= 2:
            error = int(result[0])
            return error, int(result[1]) if error == 0 else None
        raise RuntimeError(f"{method_name} returned unexpected result: {result!r}")

    def _normalize_ft_result(self, result: object, method_name: str) -> tuple[int, list[float]]:
        if isinstance(result, int):
            return int(result), []
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

    # 参考当前关节位姿求指定笛卡尔位姿的逆解，返回关节角。
    def solve_inverse_kin_ref(self, pose_mmdeg: list[float], joint_pos_ref_deg: list[float]) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.GetInverseKinRef(
            0,
            [float(value) for value in pose_mmdeg],
            [float(value) for value in joint_pos_ref_deg],
        )
        return self._normalize_pose_result(result, "GetInverseKinRef")

    # 使用指定工具/工件坐标求逆解，避免 active TCP 与目标 TCP 不一致。
    def solve_inverse_kin_exaxis(self, pose_mmdeg: list[float], tool_id: int, user_id: int) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.GetInverseKinExaxis(
            0,
            [float(value) for value in pose_mmdeg],
            [0.0, 0.0, 0.0, 0.0],
            int(tool_id),
            int(user_id),
        )
        return self._normalize_pose_result(result, "GetInverseKinExaxis")

    # 执行关节空间运动。
    def move_j(
        self,
        joint_pos: list[float],
        tool_id: int,
        user_id: int,
        vel: float,
        acc: float | None = None,
        desc_pos_mmdeg: list[float] | None = None,
        blend_time_ms: float | None = None,
    ) -> int:
        self.ensure_connection()
        kwargs = {"tool": int(tool_id), "user": int(user_id), "vel": float(vel)}
        if acc is not None:
            kwargs["acc"] = float(acc)
        if desc_pos_mmdeg is not None:
            kwargs["desc_pos"] = [float(v) for v in desc_pos_mmdeg]
        if blend_time_ms is not None:
            kwargs["blendT"] = float(blend_time_ms)
        return int(self.robot.MoveJ([float(v) for v in joint_pos], **kwargs))

    # 先求逆解，再执行关节空间运动。
    def move_j_pose(
        self,
        pose_mmdeg: list[float],
        joint_pos_ref_deg: list[float],
        tool_id: int,
        user_id: int,
        vel: float,
        acc: float | None = None,
        blend_time_ms: float | None = None,
    ) -> int:
        try:
            error, joint_pos = self.solve_inverse_kin_exaxis(pose_mmdeg, tool_id=tool_id, user_id=user_id)
        except Fault as exc:
            if "GetInverseKinExaxis" not in str(exc):
                raise
            error, joint_pos = self.solve_inverse_kin_ref(pose_mmdeg, joint_pos_ref_deg)
        if error != 0:
            return int(error)
        return self.move_j(
            joint_pos,
            tool_id=tool_id,
            user_id=user_id,
            vel=vel,
            acc=acc,
            desc_pos_mmdeg=pose_mmdeg,
            blend_time_ms=blend_time_ms,
        )

    # 执行笛卡尔空间直线运动。
    def move_l(self, pose_mmdeg: list[float], tool_id: int, user_id: int, vel: float) -> int:
        self.ensure_connection()
        return int(self.robot.MoveL([float(v) for v in pose_mmdeg], tool=int(tool_id), user=int(user_id), vel=float(vel)))

    # 执行笛卡尔空间点到点运动，由控制器按指定工具/工件坐标求解关节配置。
    def move_cart(self, pose_mmdeg: list[float], tool_id: int, user_id: int, vel: float, acc: float | None = None) -> int:
        self.ensure_connection()
        kwargs = {"tool": int(tool_id), "user": int(user_id), "vel": float(vel)}
        if acc is not None:
            kwargs["acc"] = float(acc)
        return int(self.robot.MoveCart([float(v) for v in pose_mmdeg], **kwargs))

    def ft_activate(self, state: bool) -> int:
        self.ensure_connection()
        return int(self.robot.FT_Activate(1 if state else 0))

    def ft_set_zero(self, state: bool = True) -> int:
        self.ensure_connection()
        return int(self.robot.FT_SetZero(1 if state else 0))

    def ft_get_force_torque_rcs(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.FT_GetForceTorqueRCS()
        return self._normalize_ft_result(result, "FT_GetForceTorqueRCS")

    def ft_get_force_torque_origin(self) -> tuple[int, list[float]]:
        self.ensure_connection()
        result = self.robot.FT_GetForceTorqueOrigin()
        return self._normalize_ft_result(result, "FT_GetForceTorqueOrigin")

    def ft_control(
        self,
        flag: bool,
        sensor_id: int,
        select: list[int],
        force_torque: list[float],
        gain: list[float],
        adj_sign: int = 0,
        ilc_sign: int = 0,
        max_dis: float = 10.0,
        max_ang: float = 0.0,
        m: list[float] | None = None,
        b: list[float] | None = None,
        threshold: list[float] | None = None,
        adjust_coeff: list[float] | None = None,
        polish_radio: float = 0.0,
        filter_sign: int = 0,
        pos_adapt_sign: int = 0,
        is_no_block: int = 1,
    ) -> int:
        self.ensure_connection()
        kwargs = {
            "flag": 1 if flag else 0,
            "sensor_id": int(sensor_id),
            "select": [int(value) for value in select],
            "ft": [float(value) for value in force_torque],
            "ft_pid": [float(value) for value in gain],
            "adj_sign": int(adj_sign),
            "ILC_sign": int(ilc_sign),
            "max_dis": float(max_dis),
            "max_ang": float(max_ang),
            "filter_Sign": int(filter_sign),
            "posAdapt_sign": int(pos_adapt_sign),
            "isNoBlock": int(is_no_block),
        }
        if m is not None:
            kwargs["M"] = [float(value) for value in m]
        if b is not None:
            kwargs["B"] = [float(value) for value in b]
        if threshold is not None:
            kwargs["threshold"] = [float(value) for value in threshold]
        if adjust_coeff is not None:
            kwargs["adjustCoeff"] = [float(value) for value in adjust_coeff]
        if polish_radio:
            kwargs["polishRadio"] = float(polish_radio)
        return int(self.robot.FT_Control(**kwargs))

    def ft_compliance_start(self, p: float, force: float) -> int:
        self.ensure_connection()
        return int(self.robot.FT_ComplianceStart(float(p), float(force)))

    def ft_compliance_stop(self) -> int:
        self.ensure_connection()
        return int(self.robot.FT_ComplianceStop())

    def ft_guard(
        self,
        flag: bool,
        sensor_id: int,
        select: list[int],
        force_torque: list[float],
        max_threshold: list[float],
        min_threshold: list[float],
    ) -> int:
        self.ensure_connection()
        return int(
            self.robot.FT_Guard(
                1 if flag else 0,
                int(sensor_id),
                [int(value) for value in select],
                [float(value) for value in force_torque],
                [float(value) for value in max_threshold],
                [float(value) for value in min_threshold],
            )
        )

    def ft_find_surface(
        self,
        rcs: int,
        direction: int,
        axis: int,
        max_dis: float,
        force_threshold: float,
        lin_v: float = 1.0,
        lin_a: float = 0.0,
    ) -> int:
        self.ensure_connection()
        return int(
            self.robot.FT_FindSurface(
                int(rcs),
                int(direction),
                int(axis),
                float(max_dis),
                float(force_threshold),
                lin_v=float(lin_v),
                lin_a=float(lin_a),
            )
        )

    def servo_move_start(self) -> int:
        self.ensure_connection()
        return int(self.robot.ServoMoveStart())

    def servo_move_end(self) -> int:
        self.ensure_connection()
        return int(self.robot.ServoMoveEnd())

    def get_robot_motion_done(self) -> tuple[int, int | None]:
        self.ensure_connection()
        result = self.robot.GetRobotMotionDone()
        if isinstance(result, tuple):
            code = int(result[0])
            done = int(result[1]) if len(result) > 1 else None
            return code, done
        return int(result), None

    def get_motion_queue_length(self) -> tuple[int, int | None]:
        self.ensure_connection()
        result = self.robot.GetMotionQueueLength()
        if isinstance(result, tuple):
            code = int(result[0])
            length = int(result[1]) if len(result) > 1 else None
            return code, length
        return int(result), None

    def servo_cart_tool_delta(
        self,
        delta_mmdeg: list[float],
        cmd_t: float = 0.008,
        pos_gain: list[float] | None = None,
        mode: int = 2,
    ) -> int:
        self.ensure_connection()
        if pos_gain is None:
            pos_gain = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        desc_pos = [float(value) for value in delta_mmdeg]
        exaxis = [0.0, 0.0, 0.0, 0.0]
        pos_gain = [float(value) for value in pos_gain]
        try:
            return int(
                self.robot.ServoCart(
                    mode=int(mode),
                    desc_pos=desc_pos,
                    exaxis=exaxis,
                    pos_gain=pos_gain,
                    cmdT=float(cmd_t),
                )
            )
        except Exception as exc:
            if "array has 9 items" not in str(exc) or not hasattr(self.robot, "robot"):
                raise
            return int(self.robot.robot.ServoCart(int(mode), desc_pos, exaxis, 0.0, 0.0, float(cmd_t), 0.0, 0.0))

    def stop_motion(self) -> int:
        self.ensure_connection()
        return int(self.robot.StopMotion())
