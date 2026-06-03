from setuptools import setup
import os
from glob import glob

package_name = "g1_detection"

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
            "detection_node = g1_detection.detection_node:main",
        ],
    },
)
