from __future__ import annotations

from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


RUNTIME_CAMERA_CONFIG = "/tmp/paus_robot/camera.yaml"


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    marker_share = Path(get_package_share_directory("paus_marker_ros2"))
    python_exec = shutil.which("python3") or "/usr/bin/python3"
    camera_bridge_script = marker_share / "scripts" / "camera_bridge.py"

    config_path = LaunchConfiguration("config_path").perform(context)
    camera_config_output = LaunchConfiguration("camera_config_output").perform(context)
    camera_ip = LaunchConfiguration("camera_ip").perform(context).strip()
    camera_index = LaunchConfiguration("camera_index").perform(context).strip()
    execute_motion = LaunchConfiguration("execute_motion").perform(context).strip().lower()

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

    return [
        ExecuteProcess(
            cmd=camera_bridge_cmd,
            name="camera_bridge_process",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="paus_marker_ros2",
            executable="image_receiver_node",
            name="image_receiver_node",
            output="screen",
        ),
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


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    default_config_path = str(bringup_share / "configs" / "default.yaml")

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
