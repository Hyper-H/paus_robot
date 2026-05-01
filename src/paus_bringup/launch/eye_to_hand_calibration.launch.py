from __future__ import annotations

# 导入 Path，便于定位包内文件。
from pathlib import Path
# 导入 shutil，用于查找 python3 可执行文件。
import shutil

# 导入 ament 索引工具，用于定位安装后的 package share 路径。
from ament_index_python.packages import get_package_share_directory
# 导入 launch 核心对象。
from launch import LaunchDescription
# 导入 launch 动作。
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
# 导入 launch 参数替换对象。
from launch.substitutions import LaunchConfiguration
# 导入 ROS2 Node 启动动作。
from launch_ros.actions import Node
from paus_perception import load_config, resolve_config_artifact_path, resolve_runtime_data_path


# 标定链默认把相机标定输出写到 `/tmp`，避免污染仓库工作树。
RUNTIME_CAMERA_CONFIG = "/tmp/paus_robot/camera.yaml"


# 优先返回工作区源码里的配置；如果当前环境只有 install 副本，再回退到 share 目录。
def _resolve_default_config_path(bringup_share: Path) -> Path:
    workspace_root = bringup_share.parents[3]
    source_config = workspace_root / "src" / "paus_bringup" / "configs" / "default.yaml"
    return source_config if source_config.exists() else bringup_share / "configs" / "default.yaml"


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _arg_or_config(raw_value: str, config_value) -> str:
    stripped = raw_value.strip()
    return str(config_value).lower() if stripped == "" and isinstance(config_value, bool) else (str(config_value) if stripped == "" else raw_value)


def _resolve_launch_path(raw_value: str, config_path: str, *, runtime_data: bool = False) -> str:
    resolver = resolve_runtime_data_path if runtime_data else resolve_config_artifact_path
    return str(Path(resolver(raw_value, config_path)))


# 真正组装标定链启动动作。
def _launch_setup(context, *args, **kwargs):
    # 当前函数不使用额外参数，显式丢弃即可。
    del args, kwargs

    # 找到 bringup 包和 marker 包的安装目录。
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    marker_share = Path(get_package_share_directory("paus_marker_ros2"))
    # 优先使用系统里的 python3。
    python_exec = shutil.which("python3") or "/usr/bin/python3"
    # `camera_bridge.py` 是普通 Python 脚本，不是 ROS2 node。
    camera_bridge_script = marker_share / "scripts" / "camera_bridge.py"

    # 读取 launch 参数。
    config_path = LaunchConfiguration("config_path").perform(context)
    config_payload = load_config(config_path)
    calibration_cfg = config_payload.get("calibration", {})
    control_cfg = config_payload.get("control", {})
    tool_to_board_cfg = calibration_cfg.get("tool_to_board", {}) if isinstance(calibration_cfg.get("tool_to_board", {}), dict) else {}
    default_tool_to_board_translation = tool_to_board_cfg.get("translation_m", [0.0, 0.0, 0.0])
    default_tool_to_board_rotation = tool_to_board_cfg.get("rotation_rpy_deg", [0.0, 0.0, 0.0])
    camera_config_output = _resolve_launch_path(LaunchConfiguration("camera_config_output").perform(context), config_path, runtime_data=True)
    camera_ip = LaunchConfiguration("camera_ip").perform(context).strip()
    camera_index = LaunchConfiguration("camera_index").perform(context).strip()
    board_rows = int(_arg_or_config(LaunchConfiguration("board_rows").perform(context), calibration_cfg.get("board_rows", 6)))
    board_cols = int(_arg_or_config(LaunchConfiguration("board_cols").perform(context), calibration_cfg.get("board_cols", 9)))
    square_size_m = float(_arg_or_config(LaunchConfiguration("square_size_m").perform(context), calibration_cfg.get("square_size_m", 0.01)))
    solver_method = _arg_or_config(LaunchConfiguration("solver_method").perform(context), calibration_cfg.get("solver_method", "joint_absolute")).strip()
    min_sample_count = int(_arg_or_config(LaunchConfiguration("min_sample_count").perform(context), calibration_cfg.get("min_sample_count", 10)))
    output_path = _resolve_launch_path(_arg_or_config(LaunchConfiguration("output_path").perform(context), calibration_cfg.get("output_path", "extrinsics.yaml")), config_path)
    trajectory_path = _resolve_launch_path(_arg_or_config(LaunchConfiguration("trajectory_path").perform(context), calibration_cfg.get("trajectory_path", "eye_to_hand_trajectory.yaml")), config_path)
    session_root_path = _resolve_launch_path(_arg_or_config(LaunchConfiguration("session_root_path").perform(context), calibration_cfg.get("session_root_path", "calibration_sessions")), config_path, runtime_data=True)
    save_sample_images = _as_bool(_arg_or_config(LaunchConfiguration("save_sample_images").perform(context), calibration_cfg.get("save_sample_images", True)))
    max_reprojection_error_px = float(_arg_or_config(LaunchConfiguration("max_reprojection_error_px").perform(context), calibration_cfg.get("max_reprojection_error_px", 0.0)))
    min_board_margin_px = float(_arg_or_config(LaunchConfiguration("min_board_margin_px").perform(context), calibration_cfg.get("min_board_margin_px", 10.0)))
    stable_position_tolerance_mm = float(_arg_or_config(LaunchConfiguration("stable_position_tolerance_mm").perform(context), calibration_cfg.get("stable_position_tolerance_mm", 0.2)))
    stable_rotation_tolerance_deg = float(_arg_or_config(LaunchConfiguration("stable_rotation_tolerance_deg").perform(context), calibration_cfg.get("stable_rotation_tolerance_deg", 0.1)))
    stable_window_s = float(_arg_or_config(LaunchConfiguration("stable_window_s").perform(context), calibration_cfg.get("stable_window_s", 0.5)))
    stable_timeout_s = float(_arg_or_config(LaunchConfiguration("stable_timeout_s").perform(context), calibration_cfg.get("stable_timeout_s", 10.0)))
    dwell_s = float(_arg_or_config(LaunchConfiguration("dwell_s").perform(context), calibration_cfg.get("dwell_s", 0.5)))
    execute_motion = _as_bool(_arg_or_config(LaunchConfiguration("execute_motion").perform(context), control_cfg.get("execute_motion", False)))
    tool_to_board_tx = float(_arg_or_config(LaunchConfiguration("tool_to_board_tx").perform(context), default_tool_to_board_translation[0]))
    tool_to_board_ty = float(_arg_or_config(LaunchConfiguration("tool_to_board_ty").perform(context), default_tool_to_board_translation[1]))
    tool_to_board_tz = float(_arg_or_config(LaunchConfiguration("tool_to_board_tz").perform(context), default_tool_to_board_translation[2]))
    tool_to_board_rx = float(_arg_or_config(LaunchConfiguration("tool_to_board_rx").perform(context), default_tool_to_board_rotation[0]))
    tool_to_board_ry = float(_arg_or_config(LaunchConfiguration("tool_to_board_ry").perform(context), default_tool_to_board_rotation[1]))
    tool_to_board_rz = float(_arg_or_config(LaunchConfiguration("tool_to_board_rz").perform(context), default_tool_to_board_rotation[2]))

    # 拼出相机桥接脚本的命令行。
    camera_bridge_cmd = [
        python_exec,
        str(camera_bridge_script),
        "--host",
        "127.0.0.1",
        "--port",
        "5001",
        "--camera-config-output",
        camera_config_output,
    ]
    if camera_ip:
        camera_bridge_cmd += ["--camera-ip", camera_ip]
    if camera_index:
        camera_bridge_cmd += ["--camera-index", camera_index]

    # 返回完整启动动作。
    return [
        # 先启动图像接收节点。
        Node(
            package="paus_marker_ros2",
            executable="image_receiver_node",
            name="image_receiver_node",
            output="screen",
        ),
        # 相机桥接脚本稍微延迟一点启动，避免 image_receiver_node 端口还没监听就先连接。
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=camera_bridge_cmd,
                    name="camera_bridge_process",
                    output="screen",
                    emulate_tty=True,
                )
            ],
        ),
        # 启动手眼标定节点。
        Node(
            package="paus_marker_ros2",
            executable="eye_to_hand_calibration_node",
            name="eye_to_hand_calibration_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "camera_config_path": camera_config_output,
                    "board_rows": board_rows,
                    "board_cols": board_cols,
                    "square_size_m": square_size_m,
                    "solver_method": solver_method,
                    "min_sample_count": min_sample_count,
                    "output_path": output_path,
                    "trajectory_path": trajectory_path,
                    "session_root_path": session_root_path,
                    "save_sample_images": save_sample_images,
                    "max_reprojection_error_px": max_reprojection_error_px,
                    "min_board_margin_px": min_board_margin_px,
                    "stable_position_tolerance_mm": stable_position_tolerance_mm,
                    "stable_rotation_tolerance_deg": stable_rotation_tolerance_deg,
                    "stable_window_s": stable_window_s,
                    "stable_timeout_s": stable_timeout_s,
                    "dwell_s": dwell_s,
                    "execute_motion": execute_motion,
                    "tool_to_board.translation_m": [tool_to_board_tx, tool_to_board_ty, tool_to_board_tz],
                    "tool_to_board.rotation_rpy_deg": [tool_to_board_rx, tool_to_board_ry, tool_to_board_rz],
                }
            ],
        ),
    ]


# 生成整份标定 launch 描述。
def generate_launch_description() -> LaunchDescription:
    # 找到 bringup 包安装目录，用于提供默认配置和外参输出路径。
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    default_config_file = _resolve_default_config_path(bringup_share)
    default_config_path = str(default_config_file)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config_path,
                description="Path to the main YAML config file.",
            ),
            DeclareLaunchArgument(
                "camera_config_output",
                default_value=RUNTIME_CAMERA_CONFIG,
                description="Path where camera_bridge.py will write camera.yaml.",
            ),
            DeclareLaunchArgument(
                "camera_ip",
                default_value="",
                description="Optional camera IP. Leave empty to use the first discovered camera.",
            ),
            DeclareLaunchArgument(
                "camera_index",
                default_value="",
                description="Optional camera index. Leave empty to use auto-selection.",
            ),
            DeclareLaunchArgument(
                "board_rows",
                default_value="",
                description="Chessboard inner-corner rows.",
            ),
            DeclareLaunchArgument(
                "board_cols",
                default_value="",
                description="Chessboard inner-corner cols.",
            ),
            DeclareLaunchArgument(
                "square_size_m",
                default_value="",
                description="Chessboard square size in meters.",
            ),
            DeclareLaunchArgument(
                "solver_method",
                default_value="",
                description="Hand-eye calibration solver method. joint_absolute jointly estimates base_to_camera and tool_to_board.",
            ),
            DeclareLaunchArgument(
                "min_sample_count",
                default_value="",
                description="Minimum number of samples before solve.",
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value="",
                description="Output path for the solved extrinsics YAML.",
            ),
            DeclareLaunchArgument("trajectory_path", default_value="", description="Semi-auto MoveJ trajectory YAML path."),
            DeclareLaunchArgument("session_root_path", default_value="", description="Directory where calibration sessions are archived."),
            DeclareLaunchArgument("save_sample_images", default_value="", description="Whether to save accepted sample images in the session directory."),
            DeclareLaunchArgument("max_reprojection_error_px", default_value="", description="Maximum accepted chessboard reprojection error in pixels. Set <= 0 to disable this filter."),
            DeclareLaunchArgument("min_board_margin_px", default_value="", description="Minimum chessboard corner margin from image border in pixels."),
            DeclareLaunchArgument("stable_position_tolerance_mm", default_value="", description="TCP position delta threshold for stable sampling."),
            DeclareLaunchArgument("stable_rotation_tolerance_deg", default_value="", description="TCP rotation delta threshold for stable sampling."),
            DeclareLaunchArgument("stable_window_s", default_value="", description="Required stable TCP window before capture."),
            DeclareLaunchArgument("stable_timeout_s", default_value="", description="Timeout for TCP stability wait."),
            DeclareLaunchArgument("dwell_s", default_value="", description="Default dwell time after each waypoint stabilizes."),
            DeclareLaunchArgument("execute_motion", default_value="", description="When false, semi-auto trajectory execution only dry-runs."),
            DeclareLaunchArgument("tool_to_board_tx", default_value="", description="Tool-to-board X translation in meters."),
            DeclareLaunchArgument("tool_to_board_ty", default_value="", description="Tool-to-board Y translation in meters."),
            DeclareLaunchArgument("tool_to_board_tz", default_value="", description="Tool-to-board Z translation in meters."),
            DeclareLaunchArgument("tool_to_board_rx", default_value="", description="Tool-to-board roll in degrees."),
            DeclareLaunchArgument("tool_to_board_ry", default_value="", description="Tool-to-board pitch in degrees."),
            DeclareLaunchArgument("tool_to_board_rz", default_value="", description="Tool-to-board yaw in degrees."),
            OpaqueFunction(function=_launch_setup),
        ]
    )
