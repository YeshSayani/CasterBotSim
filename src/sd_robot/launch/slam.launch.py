import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("sd_robot")

    slam_config = os.path.join(
        pkg_share,
        "config",
        "slam_toolbox.yaml"
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        slam_toolbox_node
    ])