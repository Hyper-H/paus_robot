from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("listen_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("listen_port", default_value="55001"),
            DeclareLaunchArgument("stale_timeout_s", default_value="1.0"),
            Node(
                package="paus_ultrasound_ros2",
                executable="ultrasound_receiver_node",
                name="ultrasound_receiver_node",
                output="screen",
                parameters=[
                    {
                        "listen_host": LaunchConfiguration("listen_host"),
                        "listen_port": LaunchConfiguration("listen_port"),
                        "stale_timeout_s": LaunchConfiguration("stale_timeout_s"),
                    }
                ],
            ),
        ]
    )
