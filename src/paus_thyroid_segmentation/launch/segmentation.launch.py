from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "params_file": get_package_share_directory("paus_thyroid_segmentation") + "/config/segmentation.yaml",
        "checkpoint": "/mnt/data/projects/paus_robot/models/thyroid_tg3k_v1/checkpoint/best_model.pth",
        "model_bundle": "/mnt/data/projects/paus_robot/models/thyroid_tg3k_v1",
        "device": "cpu", "input_mode": "frame",
        "frame_topic": "/ultrasound/us/frame", "input_topic": "/ultrasound/us/image_raw",
        "use_sim_time": "false",
        "cpu_fallback": "false",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(
            package="paus_thyroid_segmentation", executable="thyroid_segmentation",
            name="thyroid_segmentation", output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                name: ParameterValue(LaunchConfiguration(name),
                                     value_type=bool if name in {"use_sim_time", "cpu_fallback"} else str)
                for name in defaults if name != "params_file"
            }],
        ),
    ])
