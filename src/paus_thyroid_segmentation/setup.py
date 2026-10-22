from glob import glob

from setuptools import setup


package_name = "paus_thyroid_segmentation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="Research-only whole thyroid gland segmentation; no robot control.",
    license="Proprietary",
    entry_points={"console_scripts": [
        "thyroid_segmentation = paus_thyroid_segmentation.node:main",
        "thyroid_frame_bridge = paus_thyroid_segmentation.frame_bridge:main",
        "thyroid_validate = paus_thyroid_segmentation.validation:main",
    ]},
)
