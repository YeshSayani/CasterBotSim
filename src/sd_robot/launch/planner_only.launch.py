import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory("sd_robot")

    params_file = os.path.join(
        pkg_share,
        "config",
        "nav2_params.yaml"
    )

    map_file = "/home/yeshwanth/self_drive_ws/maps/warehouse_world_map.yaml"

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "yaml_filename": map_file
            }
        ]
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file]
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file]
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planner_only",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server"
                ]
            }
        ]
    )

    return LaunchDescription([
    map_server_node,
    amcl_node,
    planner_server_node,

    TimerAction(
        period=3.0,
        actions=[lifecycle_manager_node]
    )
])