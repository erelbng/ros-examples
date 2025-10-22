import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess


# def generate_launch_description():
#     return LaunchDescription(
#         [
#             Node(
#                 package="diffbot_universal",
#                 executable="diffbot_node",
#                 name="diffbot_sim",
#                 output="screen",
#             ),
#         ]
#     )
def generate_launch_description():
    pkg_name = "diffbot_universal"
    node_file = os.path.join(
        get_package_share_directory(pkg_name), "diffbot_universal/diffbot_node.py"
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
