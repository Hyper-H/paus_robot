from glob import glob

from setuptools import setup


package_name = "paus_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/configs", glob("configs/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="Launch and runtime configuration package for PAUS Robot.",
    license="Proprietary",
)
