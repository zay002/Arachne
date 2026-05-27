from glob import glob

from setuptools import find_packages, setup

package_name = "arachne_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arachne Maintainers",
    maintainer_email="maintainer@example.com",
    description="Interactive demo launch files and controller input tools for Arachne.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "switch_teleop = arachne_demo.switch_teleop:main",
        ],
    },
)
