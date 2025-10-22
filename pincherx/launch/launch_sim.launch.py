import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_name = "pincherx"
    node_file = os.path.join(
        get_package_share_directory(pkg_name), "pincherx/pincherx_node.py"
    )

    # Use mjpython to launch the node
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["mjpython", node_file],
                output="screen",
            )
        ]
    )
