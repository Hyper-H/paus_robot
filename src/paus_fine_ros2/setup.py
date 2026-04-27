from setuptools import setup


package_name = "paus_fine_ros2"


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
    description="Placeholder package for future fine localization and closed-loop adjustment nodes.",
    license="Proprietary",
)
