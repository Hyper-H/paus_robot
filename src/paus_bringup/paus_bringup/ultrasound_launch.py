from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ultrasound_actions() -> list:
    """Optional single-source receiver shared by the operator launch entries."""
    return [
        DeclareLaunchArgument("enable_ultrasound", default_value="false", description="Start the USFR receiver; does not enable robot motion."),
        DeclareLaunchArgument("ultrasound_listen_host", default_value="127.0.0.1", description="USFR bind address; choose the Linux LAN IP for Windows capture."),
        DeclareLaunchArgument("ultrasound_listen_port", default_value="55001"),
        DeclareLaunchArgument("ultrasound_stale_timeout_s", default_value="1.0"),
        Node(
            package="paus_ultrasound_ros2",
            executable="ultrasound_receiver_node",
            name="ultrasound_receiver_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_ultrasound")),
            parameters=[{
                "listen_host": LaunchConfiguration("ultrasound_listen_host"),
                "listen_port": LaunchConfiguration("ultrasound_listen_port"),
                "stale_timeout_s": LaunchConfiguration("ultrasound_stale_timeout_s"),
            }],
        ),
    ]
