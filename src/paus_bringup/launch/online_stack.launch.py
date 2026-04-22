from __future__ import annotations

# 导入 Path，便于定位包内文件。
from pathlib import Path
# 导入 shutil，用于查找 python3 可执行文件。
import shutil

# 导入 ament 索引工具，用于定位安装后的 package share 路径。
from ament_index_python.packages import get_package_share_directory
# 导入 launch 核心对象。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
# 导入 ROS2 Node 启动动作。
from launch_ros.actions import Node


# 将运行时相机配置默认写到 `/tmp`，避免污染仓库工作树。
RUNTIME_CAMERA_CONFIG = "/tmp/paus_robot/camera.yaml"


# 这个函数会在 launch 参数都解析完成之后执行，
# 负责真正拼装出需要启动的外部进程和 ROS2 节点。
def _launch_setup(context, *args, **kwargs):
    # 这里不用额外参数，显式丢弃掉，避免未使用变量告警。
    del args, kwargs
    # 找到 bringup 包和 marker 包的安装目录。
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    marker_share = Path(get_package_share_directory("paus_marker_ros2"))
    # 尽量使用当前系统里的 python3；找不到时回退到常见路径。
    python_exec = shutil.which("python3") or "/usr/bin/python3"
    # `camera_bridge.py` 是一个普通 Python 脚本，不是 ROS2 node。
    camera_bridge_script = marker_share / "scripts" / "camera_bridge.py"

    # 读取 launch 参数。
    config_path = LaunchConfiguration("config_path").perform(context)
    camera_config_output = LaunchConfiguration("camera_config_output").perform(context)
    camera_ip = LaunchConfiguration("camera_ip").perform(context).strip()
    camera_index = LaunchConfiguration("camera_index").perform(context).strip()
    execute_motion = LaunchConfiguration("execute_motion").perform(context).strip().lower()

    # 拼出 `camera_bridge.py` 的实际命令行。
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
    # 如果显式指定了 camera_ip，就把它加进去。
    if camera_ip:
        camera_bridge_cmd += ["--camera-ip", camera_ip]
    # 如果显式指定了 camera_index，也把它加进去。
    if camera_index:
        camera_bridge_cmd += ["--camera-index", camera_index]

    # 返回完整的启动动作列表。
    return [
        # 先启动相机桥接脚本。
        ExecuteProcess(
            cmd=camera_bridge_cmd,
            name="camera_bridge_process",
            output="screen",
            emulate_tty=True,
        ),
        # 启动图像接收节点。
        Node(
            package="paus_marker_ros2",
            executable="image_receiver_node",
            name="image_receiver_node",
            output="screen",
        ),
        # 启动 marker 位姿节点，并把 camera.yaml 路径传进去。
        Node(
            package="paus_marker_ros2",
            executable="marker_pose_node",
            name="marker_pose_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "camera_config_path": camera_config_output,
                }
            ],
        ),
        # 启动基座系目标变换节点。
        Node(
            package="paus_marker_ros2",
            executable="target_transform_node",
            name="target_transform_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "extrinsics_path": str(bringup_share / "configs" / "extrinsics.yaml"),
                }
            ],
        ),
        # 启动运动控制节点，并在这里决定是否允许真实执行。
        Node(
            package="paus_motion_ros2",
            executable="fairino_control_node",
            name="fairino_control_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "execute_motion": execute_motion == "true",
                }
            ],
        ),
    ]


# 生成整份 launch 描述。
def generate_launch_description() -> LaunchDescription:
    # 找到 bringup 包安装目录中的默认配置文件。
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    default_config_path = str(bringup_share / "configs" / "default.yaml")

    # 声明 launch 可用参数，并把 `_launch_setup` 挂进去。
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
                "execute_motion",
                default_value="false",
                description="Whether to allow real robot motion. Defaults to dry-run.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
