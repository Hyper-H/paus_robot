from glob import glob

from setuptools import setup


package_name = "paus_ultrasound_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "Apache-2.0.txt"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="Verasonics USFR ultrasound image receiver for ROS 2.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "ultrasound_receiver_node = paus_ultrasound_ros2.receiver_node:main",
            "record_ultrasound_bag = paus_ultrasound_ros2.record_bag:main",
        ],
    },
)
