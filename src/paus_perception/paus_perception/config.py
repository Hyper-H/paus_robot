from __future__ import annotations

# 导入 deepcopy，避免解析路径时修改全局默认配置对象。
from copy import deepcopy
import os
# 导入 Path，便于处理配置文件路径。
from pathlib import Path
# 导入 Any，便于描述灵活的配置字典。
from typing import Any

# 导入 yaml，用来读取 YAML 配置文件。
import yaml


# 定义项目默认配置。
DEFAULT_CONFIG: dict[str, Any] = {
    # ArUco 检测相关配置。
    "aruco": {
        # 当前默认采用的 ArUco 字典。
        "dictionary": "DICT_4X4_50",
        # 目标 marker 的 ID；为空时退化为选择面积最大的候选。
        "target_id": None,
        # 自适应阈值算法中的常数项。
        "adaptive_thresh_constant": 7,
        # 自适应阈值算法中的窗口大小。
        "adaptive_thresh_block_size": 31,
        # 调试轮廓筛选时的最小面积阈值。
        "min_candidate_area": 1200,
        # 多边形近似时的周长比例参数。
        "approx_epsilon_ratio": 0.04,
    },
    # 输出相关配置。
    "output": {
        # 是否输出调试图像。
        "export_debug": True,
        # 对候选 marker 透视展开后用于 Otsu 二值化的图像边长。
        "debug_square_size": 160,
        # 调试轮廓图中的线宽。
        "contour_line_thickness": 2,
    },
    # 位姿估计与接近目标规划相关配置。
    "pose": {
        # 单个 marker 的真实物理边长，单位为米。
        "marker_length_m": 0.04,
        # 机械臂希望停在 marker 前方的安全距离，单位为米。
        "approach_distance_m": 0.05,
        # 当前阶段规划输出所在坐标系默认是相机坐标系。
        "target_frame": "camera",
    },
    # 坐标变换相关配置。
    "transform": {
        # 工作 marker 到目标区域的固定变换。
        "marker_to_target": {
            # 第一版默认向下偏移 0 cm。
            "translation_m": [0.0, 0.0, 0.0],
            # 第一版默认不附加旋转。
            "rotation_rpy_deg": [0.0, 0.0, 0.0],
        }
    },
    # 手眼标定相关配置。
    "calibration": {
        # 棋盘格内角点行数。
        "board_rows": 6,
        # 棋盘格内角点列数。
        "board_cols": 9,
        # 棋盘格单格边长，单位米。
        "square_size_m": 0.01,
        # 手眼标定求解器，默认使用标准 AX=XB Park 方法。
        "solver_method": "opencv_handeye_park",
        # 求解前最少需要的样本数。
        "min_sample_count": 10,
        # 标定结果默认保存路径。
        "output_path": "extrinsics.yaml",
        # 半自动标定示教轨迹保存路径。
        "trajectory_path": "eye_to_hand_trajectory.yaml",
        # 每次半自动标定运行的归档目录根路径。
        "session_root_path": "calibration_sessions",
        # 是否保存每个有效样本图像。
        "save_sample_images": True,
        # 棋盘 solvePnP 质量过滤阈值。
        "max_reprojection_error_px": 0.0,
        "min_board_margin_px": 10.0,
        # 到点后判定 TCP 稳定的阈值。
        "stable_position_tolerance_mm": 0.2,
        "stable_rotation_tolerance_deg": 0.1,
        "stable_window_s": 0.5,
        "stable_timeout_s": 10.0,
        "dwell_s": 0.5,
        # 当前 TCP 到棋盘格中心的固定外参。
        "tool_to_board": {
            "translation_m": [0.0, 0.0, 0.0],
            "rotation_rpy_deg": [0.0, 0.0, 0.0],
        },
    },
    # 机械臂接近控制相关配置。
    "control": {
        # 机械臂控制器 IP。
        "robot_ip": "192.168.58.2",
        # 官方 Linux FAIRINO Python SDK 根目录。
        "linux_fairino_sdk_root": "/opt/fairino_python_sdk/linux",
        # 默认只做 dry-run，不执行真实动作。
        "execute_motion": False,
        # 当前使用的工具坐标系编号。
        "tool_id": 0,
        # 当前使用的用户坐标系编号。
        "user_id": 0,
        # MoveL 速度。
        "move_vel": 10.0,
        # MoveL 加速度。
        "move_acc": 10.0,
        # MoveL 速度倍率。
        "move_ovl": 20.0,
        # MoveL 圆滑半径。
        "blendR": -1.0,
        # 允许的最大单步位移。
        "max_step_distance_mm": 80.0,
        # 停在目标点前的安全距离。
        "final_standoff_mm": 30.0,
        # 最低安全高度。
        "min_safe_z_mm": 50.0,
        # 允许的工作空间最小边界。
        "workspace_min_mm": [-1000.0, -1000.0, 0.0],
        # 允许的工作空间最大边界。
        "workspace_max_mm": [1000.0, 1000.0, 1000.0],
        # 重复目标点去抖阈值。
        "repeat_distance_threshold_mm": 5.0,
        # 是否显式使用 mock 当前 TCP 位姿做 dry-run。
        "use_mock_pose": False,
        # mock 当前 TCP 位姿 [x,y,z,rx,ry,rz]。
        "mock_current_tcp_pose_mmdeg": [0.0, 0.0, 300.0, 180.0, 0.0, -180.0],
    },
}


# 递归合并默认配置和用户配置。
def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    # 先复制基础配置，避免原对象被直接修改。
    merged = deepcopy(base)
    # 遍历用户传入的每个键值对。
    for key, value in override.items():
        # 如果两边都是字典，则继续递归合并。
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        # 否则直接用用户值覆盖。
        else:
            merged[key] = value
    # 返回合并完成的配置。
    return merged


def _infer_project_root(config_path: Path) -> Path:
    resolved_config_path = config_path.expanduser().resolve()
    search_roots = [resolved_config_path.parent, *resolved_config_path.parents]
    for candidate in search_roots:
        if (candidate / "src" / "paus_bringup").exists() and (candidate / "src" / "paus_perception").exists():
            return candidate

    parts = resolved_config_path.parts
    if "src" in parts:
        src_index = parts.index("src")
        if src_index > 0:
            return Path(*parts[:src_index])
    if "install" in parts:
        install_index = parts.index("install")
        if install_index > 0:
            return Path(*parts[:install_index])
    return resolved_config_path.parent


def resolve_config_path(path_value: str | Path, config_path: str | Path) -> str:
    path_text = os.path.expandvars(str(path_value)).strip()
    path = Path(path_text).expanduser()
    if path.is_absolute():
        return str(path)
    return str((_infer_project_root(Path(config_path)) / path).resolve())


def _runtime_root() -> Path:
    override = os.environ.get("PAUS_ROBOT_RUNTIME_DIR")
    if override:
        return Path(override).expanduser()
    xdg_data_home = os.environ.get("XDG_DATA_HOME")
    if xdg_data_home:
        return Path(xdg_data_home).expanduser() / "paus_robot"
    return Path.home() / ".local" / "share" / "paus_robot"


def _resolve_config_artifact_path(path_value: str | Path, config_path: str | Path) -> str:
    path_text = os.path.expandvars(str(path_value)).strip()
    path = Path(path_text).expanduser()
    if path.is_absolute():
        return str(path)
    if len(path.parts) == 1:
        resolved_config_path = Path(config_path).expanduser().resolve()
        if "install" in resolved_config_path.parts:
            return str((_runtime_root() / "configs" / path).resolve())
        return str((resolved_config_path.parent / path).resolve())
    return resolve_config_path(path, config_path)


def _resolve_runtime_data_path(path_value: str | Path, config_path: str | Path) -> str:
    path_text = os.path.expandvars(str(path_value)).strip()
    path = Path(path_text).expanduser()
    if path.is_absolute():
        return str(path)
    resolved_config_path = Path(config_path).expanduser().resolve()
    if "install" in resolved_config_path.parts:
        return str((_runtime_root() / path).resolve())
    return resolve_config_path(path, config_path)


def _resolve_project_paths(config: dict[str, Any], config_path: Path) -> dict[str, Any]:
    calibration = config.get("calibration", {})
    if not isinstance(calibration, dict):
        return config
    for field_name in ("output_path", "trajectory_path"):
        value = calibration.get(field_name)
        if value:
            calibration[field_name] = _resolve_config_artifact_path(value, config_path)
    for field_name in ("session_root_path",):
        value = calibration.get(field_name)
        if value:
            calibration[field_name] = _resolve_runtime_data_path(value, config_path)
    return config


# 加载 YAML 配置文件并返回最终配置。
def load_config(path: str | Path | None = None) -> dict[str, Any]:
    # 如果没有给路径，就直接返回默认配置。
    if path is None:
        return deepcopy(DEFAULT_CONFIG)

    # 将路径统一转成 Path 对象。
    config_path = Path(path)
    # 打开配置文件并读取内容。
    with config_path.open("r", encoding="utf-8") as handle:
        user_config = yaml.safe_load(handle) or {}
    # 将用户配置合并到默认配置上。
    merged = _deep_merge(DEFAULT_CONFIG, user_config)
    return _resolve_project_paths(merged, config_path)
