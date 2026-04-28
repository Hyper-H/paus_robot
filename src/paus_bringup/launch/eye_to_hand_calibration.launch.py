from __future__ import annotations

# 导入 Path，便于定位包内文件。
from pathlib import Path
# 导入 shutil，用于查找 python3 可执行文件。
import shutil
import yaml

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


# 标定链默认把相机标定输出写到 `/tmp`，避免污染仓库工作树。
RUNTIME_CAMERA_CONFIG = "/tmp/paus_robot/camera.yaml"


# 优先返回工作区源码里的配置；如果当前环境只有 install 副本，再回退到 share 目录。
def _resolve_default_config_path(bringup_share: Path) -> Path:
    workspace_root = bringup_share.parents[3]
    source_config = workspace_root / "src" / "paus_bringup" / "configs" / "default.yaml"
    return source_config if source_config.exists() else bringup_share / "configs" / "default.yaml"


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
    camera_config_output = LaunchConfiguration("camera_config_output").perform(context)
    camera_ip = LaunchConfiguration("camera_ip").perform(context).strip()
    camera_index = LaunchConfiguration("camera_index").perform(context).strip()
    board_rows = int(LaunchConfiguration("board_rows").perform(context))
    board_cols = int(LaunchConfiguration("board_cols").perform(context))
    square_size_m = float(LaunchConfiguration("square_size_m").perform(context))
    solver_method = LaunchConfiguration("solver_method").perform(context).strip()
    min_sample_count = int(LaunchConfiguration("min_sample_count").perform(context))
    output_path = LaunchConfiguration("output_path").perform(context)
    trajectory_path = LaunchConfiguration("trajectory_path").perform(context)
    session_root_path = LaunchConfiguration("session_root_path").perform(context)
    save_sample_images = LaunchConfiguration("save_sample_images").perform(context).strip().lower() in ("1", "true", "yes", "on")
    max_reprojection_error_px = float(LaunchConfiguration("max_reprojection_error_px").perform(context))
    min_board_margin_px = float(LaunchConfiguration("min_board_margin_px").perform(context))
    stable_position_tolerance_mm = float(LaunchConfiguration("stable_position_tolerance_mm").perform(context))
    stable_rotation_tolerance_deg = float(LaunchConfiguration("stable_rotation_tolerance_deg").perform(context))
    stable_window_s = float(LaunchConfiguration("stable_window_s").perform(context))
    stable_timeout_s = float(LaunchConfiguration("stable_timeout_s").perform(context))
    dwell_s = float(LaunchConfiguration("dwell_s").perform(context))
    execute_motion = LaunchConfiguration("execute_motion").perform(context).strip().lower() in ("1", "true", "yes", "on")
    tool_to_board_tx = float(LaunchConfiguration("tool_to_board_tx").perform(context))
    tool_to_board_ty = float(LaunchConfiguration("tool_to_board_ty").perform(context))
    tool_to_board_tz = float(LaunchConfiguration("tool_to_board_tz").perform(context))
    tool_to_board_rx = float(LaunchConfiguration("tool_to_board_rx").perform(context))
    tool_to_board_ry = float(LaunchConfiguration("tool_to_board_ry").perform(context))
    tool_to_board_rz = float(LaunchConfiguration("tool_to_board_rz").perform(context))

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
    with default_config_file.open("r", encoding="utf-8") as handle:
        config_payload = yaml.safe_load(handle) or {}
    calibration_cfg = config_payload.get("calibration", {})
    tool_to_board_cfg = calibration_cfg.get("tool_to_board", {})
    default_extrinsics_path = str(calibration_cfg.get("output_path", bringup_share / "configs" / "extrinsics.yaml"))
    default_board_rows = str(calibration_cfg.get("board_rows", 6))
    default_board_cols = str(calibration_cfg.get("board_cols", 9))
    default_square_size_m = str(calibration_cfg.get("square_size_m", 0.01))
    default_solver_method = str(calibration_cfg.get("solver_method", "joint_absolute"))
    default_min_sample_count = str(calibration_cfg.get("min_sample_count", 10))
    default_trajectory_path = str(calibration_cfg.get("trajectory_path", default_config_file.parent / "eye_to_hand_trajectory.yaml"))
    default_session_root_path = str(calibration_cfg.get("session_root_path", "/home/chen_lab/paus_robot/calibration_sessions"))
    default_save_sample_images = str(calibration_cfg.get("save_sample_images", True)).lower()
    default_max_reprojection_error_px = str(calibration_cfg.get("max_reprojection_error_px", 2.5))
    default_min_board_margin_px = str(calibration_cfg.get("min_board_margin_px", 10.0))
    default_stable_position_tolerance_mm = str(calibration_cfg.get("stable_position_tolerance_mm", 0.2))
    default_stable_rotation_tolerance_deg = str(calibration_cfg.get("stable_rotation_tolerance_deg", 0.1))
    default_stable_window_s = str(calibration_cfg.get("stable_window_s", 0.5))
    default_stable_timeout_s = str(calibration_cfg.get("stable_timeout_s", 10.0))
    default_dwell_s = str(calibration_cfg.get("dwell_s", 0.5))
    default_execute_motion = str(config_payload.get("control", {}).get("execute_motion", False)).lower()
    default_tool_to_board_translation = tool_to_board_cfg.get("translation_m", [0.0, 0.0, 0.0])
    default_tool_to_board_rotation = tool_to_board_cfg.get("rotation_rpy_deg", [0.0, 0.0, 0.0])

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
                default_value=default_board_rows,
                description="Chessboard inner-corner rows.",
            ),
            DeclareLaunchArgument(
                "board_cols",
                default_value=default_board_cols,
                description="Chessboard inner-corner cols.",
            ),
            DeclareLaunchArgument(
                "square_size_m",
                default_value=default_square_size_m,
                description="Chessboard square size in meters.",
            ),
            DeclareLaunchArgument(
                "solver_method",
                default_value=default_solver_method,
                description="Hand-eye calibration solver method. joint_absolute jointly estimates base_to_camera and tool_to_board.",
            ),
            DeclareLaunchArgument(
                "min_sample_count",
                default_value=default_min_sample_count,
                description="Minimum number of samples before solve.",
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value=default_extrinsics_path,
                description="Output path for the solved extrinsics YAML.",
            ),
            DeclareLaunchArgument("trajectory_path", default_value=default_trajectory_path, description="Semi-auto MoveJ trajectory YAML path."),
            DeclareLaunchArgument("session_root_path", default_value=default_session_root_path, description="Directory where calibration sessions are archived."),
            DeclareLaunchArgument("save_sample_images", default_value=default_save_sample_images, description="Whether to save accepted sample images in the session directory."),
            DeclareLaunchArgument("max_reprojection_error_px", default_value=default_max_reprojection_error_px, description="Maximum accepted chessboard reprojection error in pixels."),
            DeclareLaunchArgument("min_board_margin_px", default_value=default_min_board_margin_px, description="Minimum chessboard corner margin from image border in pixels."),
            DeclareLaunchArgument("stable_position_tolerance_mm", default_value=default_stable_position_tolerance_mm, description="TCP position delta threshold for stable sampling."),
            DeclareLaunchArgument("stable_rotation_tolerance_deg", default_value=default_stable_rotation_tolerance_deg, description="TCP rotation delta threshold for stable sampling."),
            DeclareLaunchArgument("stable_window_s", default_value=default_stable_window_s, description="Required stable TCP window before capture."),
            DeclareLaunchArgument("stable_timeout_s", default_value=default_stable_timeout_s, description="Timeout for TCP stability wait."),
            DeclareLaunchArgument("dwell_s", default_value=default_dwell_s, description="Default dwell time after each waypoint stabilizes."),
            DeclareLaunchArgument("execute_motion", default_value=default_execute_motion, description="When false, semi-auto trajectory execution only dry-runs."),
            DeclareLaunchArgument("tool_to_board_tx", default_value=str(default_tool_to_board_translation[0]), description="Tool-to-board X translation in meters."),
            DeclareLaunchArgument("tool_to_board_ty", default_value=str(default_tool_to_board_translation[1]), description="Tool-to-board Y translation in meters."),
            DeclareLaunchArgument("tool_to_board_tz", default_value=str(default_tool_to_board_translation[2]), description="Tool-to-board Z translation in meters."),
            DeclareLaunchArgument("tool_to_board_rx", default_value=str(default_tool_to_board_rotation[0]), description="Tool-to-board roll in degrees."),
            DeclareLaunchArgument("tool_to_board_ry", default_value=str(default_tool_to_board_rotation[1]), description="Tool-to-board pitch in degrees."),
            DeclareLaunchArgument("tool_to_board_rz", default_value=str(default_tool_to_board_rotation[2]), description="Tool-to-board yaw in degrees."),
            OpaqueFunction(function=_launch_setup),
        ]
    )
