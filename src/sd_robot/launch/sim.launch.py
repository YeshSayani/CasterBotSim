import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "sd_robot"

    pkg_share = get_package_share_directory(package_name)
    models_path = os.path.join(pkg_share, "models")

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            models_path,
            ":",
            os.environ.get("GAZEBO_MODEL_PATH", "")
        ]
    )

    robot_description_path = os.path.join(
        pkg_share,
        "urdf",
        "simple_robot.urdf.xacro"
    )
    
    #world_file = "test_arena.world"
    #world_file = "asymmetric_world.world"
    world_file = "warehouse_world.world"
    #world_file = "downloaded_warehouse.world"
    
    world_path = os.path.join(
        pkg_share,
        "worlds",
        world_file
    )

    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            robot_description_path
        ])
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "world": world_path
        }.items()
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "simple_sd_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.15"
        ],
        output="screen"
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo_node,
        robot_state_publisher_node,
        spawn_robot_node
    ])