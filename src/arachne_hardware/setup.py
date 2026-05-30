from setuptools import find_packages, setup

package_name = "arachne_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/real_hardware.yaml"]),
        (
            f"share/{package_name}/launch",
            ["launch/real_bringup.launch.py", "launch/mock_bringup.launch.py"],
        ),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Arachne Maintainers",
    maintainer_email="maintainer@example.com",
    description="Real-hardware ROS bringup wrappers for Arachne.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scout_official_status_bridge = arachne_hardware.base_serial_driver:main",
            "scout_waveshare_serial_driver = arachne_hardware.scout_waveshare_serial_driver:main",
            "ms42dc_official_bridge = arachne_hardware.gripper_serial_driver:main",
            "ms42dc_direct_serial_driver = arachne_hardware.ms42dc_direct_serial_driver:main",
            "aubo_official_status_probe = arachne_hardware.aubo_tcp_driver:main",
            "safety_state_machine = arachne_hardware.safety_state_machine:main",
            "safety_cmd_vel_gate = arachne_hardware.safety_cmd_vel_gate:main",
            "hardware_mock = arachne_hardware.hardware_mock:main",
        ],
    },
)
