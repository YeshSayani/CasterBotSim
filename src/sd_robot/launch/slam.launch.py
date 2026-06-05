# Imports Python's OS module
import os

from launch import LaunchDescription # Imports LaunchDescription, a list of things ROS 2 launch should start.
from launch_ros.actions import Node # Imports the Node launch action. You use this when a launch file needs to start a ROS 2 node.
from ament_index_python.packages import get_package_share_directory # Imports a function that finds the installed share directory of a ROS 2 package. 


def generate_launch_description(): # Required function for a Python ROS 2 launch file.
    pkg_share = get_package_share_directory("sd_robot") # Finds the installed share directory of your sd_robot package.

    slam_config = os.path.join( # This builds the full path to slam_toolbox.yaml
        pkg_share,
        "config",
        "slam_toolbox.yaml"
    )

    slam_toolbox_node = Node( # Starts slam toolbox node 
        package="slam_toolbox", # Package name 
        executable="async_slam_toolbox_node", # Executable name
        name="slam_toolbox", # Node name
        output="screen", # Outputs logs to the screen 
        parameters=[ # Parameters to be passed to the slam toolbox node
            slam_config, # Loads Parameters from config/slam_toolbox.yaml
            {"use_sim_time": True} # Tells the node to use simulation time instead of computer's clock time
        ]
    )

    return LaunchDescription([ # Tells ROS2 to launch the node
        slam_toolbox_node
    ])