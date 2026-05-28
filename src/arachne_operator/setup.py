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
            "operator_panel = arachne_operator.operator_panel:main",
            "sequence_executor = arachne_operator.sequence_executor:main",
        ],
    },
)
