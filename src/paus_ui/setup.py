from setuptools import setup


package_name = "paus_ui"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn", "websockets"],
    zip_safe=False,
    maintainer="Local User",
    maintainer_email="local@example.com",
    description="Web UI console for PAUS Robot vision workflows.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "paus_ui_server = paus_ui.web_server:main",
        ],
    },
)
