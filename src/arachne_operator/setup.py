from glob import glob

from setuptools import find_packages, setup

package_name = "arachne_operator"

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
    description="Lightweight operator status panel for Arachne.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "action_chunk_translator = arachne_operator.action_chunk_translator:main",
            "apriltag_hand_eye_calibrator = arachne_operator.apriltag_hand_eye_calibrator:main",
            "apriltag_nav_initializer = arachne_operator.apriltag_nav_initializer:main",
            "demo_orchestrator = arachne_operator.demo_orchestrator:main",
            "grasp_task_server = arachne_operator.grasp_task_server:main",
            "operator_panel = arachne_operator.operator_panel:main",
            "real_hardware_acceptance_test = arachne_operator.real_hardware_acceptance_test:main",
            "road_cleanup_task_server = arachne_operator.road_cleanup_task_server:main",
            "sequence_executor = arachne_operator.sequence_executor:main",
            "teach_panel = arachne_operator.teach_panel:main",
            (
                "teach_visualization_joint_states = "
                "arachne_operator.teach_visualization_joint_states:main"
            ),
        ],
    },
)
