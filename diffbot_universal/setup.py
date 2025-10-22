from setuptools import find_packages, setup

package_name = "diffbot_universal"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/launch_sim.launch.py"]),
        ("share/" + package_name + "/description", ["description/robot.xml"]),
        (
            "share/" + package_name + "/diffbot_universal",
            ["diffbot_universal/diffbot_node.py"],
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
            "diffbot_node = diffbot_universal.diffbot_node:main",
        ],
    },
)
