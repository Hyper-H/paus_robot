from setuptools import setup


package_name = "ag_marker_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, "ag_repro"],
    package_dir={
        "ag_marker_ros2": "ag_marker_ros2",
        "ag_repro": "../../../src/ag_repro",
    },
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/configs", ["../../../configs/default.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="ROS2 node for AG marker detection, pose estimation, and approach target publishing.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "image_receiver_node = ag_marker_ros2.image_receiver_node:main",
            "marker_pose_node = ag_marker_ros2.marker_pose_node:main",
            "eye_to_hand_calibration_node = ag_marker_ros2.eye_to_hand_calibration_node:main",
            "target_transform_node = ag_marker_ros2.target_transform_node:main",
            "fairino_control_node = ag_marker_ros2.fairino_control_node:main",
        ],
    },
)
