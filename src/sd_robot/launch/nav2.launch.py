import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("sd_robot")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    params_file = os.path.join(
        pkg_share,
        "config",
        "nav2_params.yaml"
    )

    map_file = "/home/yeshwanth/self_drive_ws/maps/asymmetric_world_map.yaml"

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "true"
        }.items()
    )

    return LaunchDescription([
        bringup_cmd
    ])