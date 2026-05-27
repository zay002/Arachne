from setuptools import find_packages, setup

package_name = "arachne_gripper"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arachne Maintainers",
    maintainer_email="maintainer@example.com",
    description="Simulation and hardware-facing gripper utilities for Arachne.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gripper_sim_controller = arachne_gripper.gripper_sim_controller:main",
            "gripper_state_gui = arachne_gripper.gripper_state_gui:main",
            "joint_state_mux = arachne_gripper.joint_state_mux:main",
        ],
    },
)
