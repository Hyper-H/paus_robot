from setuptools import setup


package_name = "paus_perception"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="Pure Python perception, calibration, transform, and camera utilities for PAUS Robot.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "run_single_image = paus_perception.run_single_image:main",
            "run_image_batch = paus_perception.run_image_batch:main",
            "calibrate_camera = paus_perception.calibrate_camera:main",
            "generate_marker = paus_perception.generate_marker:main",
        ],
    },
)
