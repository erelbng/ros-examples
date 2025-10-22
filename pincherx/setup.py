import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pincherx"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/launch_sim.launch.py"]),
        ("share/" + package_name + "/description", ["description/pincherx100.xml"]),
        ("share/" + package_name + "/description/meshes", glob("description/meshes/*")),
        (
            "share/" + package_name + "/pincherx",
            ["pincherx/pincherx_node.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="e.elbing",
    maintainer_email="e.elbing@deccs.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "pincherx_node = pincherx.pincherx_node:main",
        ],
    },
)
