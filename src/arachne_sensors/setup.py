from setuptools import find_packages, setup

package_name = "arachne_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/gemini335.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arachne Maintainers",
    maintainer_email="maintainer@example.com",
    description="Sensor bringup nodes for Arachne.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gemini335_v4l2_node = arachne_sensors.gemini335_v4l2_node:main",
        ],
    },
)
