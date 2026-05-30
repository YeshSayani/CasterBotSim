# Allows for dynamic pathing
import os 

# LaunchDescription is the object that tells ROS a list of things it wants to start/do/launch. 
from launch import LaunchDescription

# IncludeLaunchDescription is used to include another launch file here: the gazebo's existing launch file.
# SetEnvironmentVariable allows the launch file to set Environment variables prior to launching processes.  
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable  

# Command allows ROS2 launch to run a shell command and use the output of the expression as a parameter. 
from launch.substitutions import Command

# Node action allows ROS to launch/start a Node
from launch_ros.actions import Node

# FindPackageShare allows to find the share directory of a package
from launch_ros.substitutions import FindPackageShare

# PythonLaunchDescriptionSource allows for including Python Launch files
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Get_package_share_directory finds the installed share directory for a ROS 2 package.
from ament_index_python.packages import get_package_share_directory

# The generate_launch_description is mandatory in any python launch file, ROS2 executes what the function returns, is the main PoE to the launch file.
def generate_launch_description():
    # Package name stored in a variable 
    package_name = "sd_robot"
    # Finds the installed share directory of the package
    pkg_share = get_package_share_directory(package_name)
    # Variable to store the path for the models directory - looks for custom models, meshes and objects. 
    models_path = os.path.join(pkg_share, "models")

    # Setting the env variable "GAZEBO_MODEL_PATH", which is used to search for models
    set_gazebo_model_path = SetEnvironmentVariable(
        # Indicates the name of the environment variable to be changed 
        name="GAZEBO_MODEL_PATH",
        # Indicates addition of this package's models folder to the beginnning of the existing Gazebo model path 
        value=[
            models_path,
            ":", # is the Linux path separator.
            os.environ.get("GAZEBO_MODEL_PATH", "") # Gets the existing value of GAZEBO_MODEL_PATH. If it does not exist, uses an empty string.
            # This prevents your launch file from deleting existing Gazebo model paths.
            # Without this, Gazebo may fail to find models in your world.
        ]
    )
    # This builds the full path to your robot Xacro file. Joins pkg_share, "urdf", "simple_robot.urdf.xacro"
    # This file defines your robot: links, joints, wheels, caster, sensors, plugins, visual shapes, collision shapes, inertias, etc.
    # The .xacro extension means it is not plain URDF yet. It is a macro-based URDF file.
    # Xacro lets you use variables, macros, and reusable blocks. ROS/Gazebo need the final expanded URDF, so you later run:
    robot_description_path = os.path.join(
        pkg_share,
        "urdf",
        "simple_robot.urdf.xacro"
    )
    ## world_file chooses which gazebo world to load ##
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
          "-y", "4.0",
          "-z", "0.10",
          "-Y", "-1.57"
        ],
        output="screen"
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo_node,
        robot_state_publisher_node,
        spawn_robot_node
    ])