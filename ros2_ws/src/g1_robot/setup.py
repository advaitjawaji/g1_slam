from setuptools import setup
from glob import glob

package_name = "g1_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "robot_node = g1_robot.robot_node:main",
            "odom_bridge = g1_robot.odom_bridge:main",
        ],
    },
)
