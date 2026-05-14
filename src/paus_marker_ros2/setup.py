from setuptools import setup


package_name = "paus_marker_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/scripts", ["scripts/camera_bridge.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="ROS2 coarse localization package for PAUS Robot marker perception.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "image_receiver_node = paus_marker_ros2.image_receiver_node:main",
            "marker_pose_node = paus_marker_ros2.marker_pose_node:main",
            "eye_to_hand_calibration_node = paus_marker_ros2.eye_to_hand_calibration_node:main",
            "eye_to_hand_semi_auto = paus_marker_ros2.eye_to_hand_semi_auto_cli:main",
            "target_transform_node = paus_marker_ros2.target_transform_node:main",
        ],
    },
)
