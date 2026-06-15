from glob import glob

from setuptools import find_packages, setup

package_name = "arachne_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arachne Maintainers",
    maintainer_email="maintainer@example.com",
    description="Lightweight RViz-oriented simulation controllers for Arachne.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_sim_controller = arachne_sim.base_sim_controller:main",
            "base_teleop_gui = arachne_sim.base_teleop_gui:main",
            "moveit_grasp_planning_demo = arachne_sim.moveit_grasp_planning_demo:main",
        ],
    },
)
