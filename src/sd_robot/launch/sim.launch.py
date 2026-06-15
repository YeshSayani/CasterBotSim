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
from launch_ros.parameter_descriptions import ParameterValue

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
    
## Builds the full path to the world file. 
    world_path = os.path.join(
        pkg_share,
        "worlds",
        world_file
    )

##  Finding Gazebo's launch file 
    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_ros"), # Finds the installed share directory for the gazebo_ros package.
        "launch",
        "gazebo.launch.py" # Creates /opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py,the official Gazebo ROS launch file.
    )
# Run xacro on simple_robot.urdf.xacro, and store the resulting URDF XML string inside the parameter called robot_description (ROS tools expect the robot model to be available under this parameter name:).
    robot_description = {
    "robot_description": ParameterValue(
        Command([
            "xacro ",
            robot_description_path
        ]),
        value_type=str
    )
}

## Starts the Robot publisher node that takes the URDF as a paramenter 
## Robot state publisher reads your robot URDF and publishes TF (transforms) between robot links.
## For example, your URDF may define:
## base_footprint → base_link
## base_link → left_wheel_link 
## base_link → right_wheel_link
## base_link → lidar_link
## Robot_state_publisher publishes those relationships to /tf and /tf_static.
## This is how RViz can understand where the robot body, wheels, and sensors are located.
## robot_state_publisher does not move the robot in Gazebo.
## It publishes the robot’s link transforms based on the URDF and joint states.
 
    robot_state_publisher_node = Node(
        package="robot_state_publisher", # Package name
        executable="robot_state_publisher", # Executable name
        parameters=[robot_description], # Robot URDF is passed as parameter to the node
        output="screen" # Outputs Node's logs to the screen
    )

## Including Gazebo's launch   

    gazebo_node = IncludeLaunchDescription( # IncludeLaunchDescription instructs to include another launch file, gazebo_launch /opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py
        PythonLaunchDescriptionSource(gazebo_launch), # The launch file being included is a python launch file
        launch_arguments={ # the path to the required world is passed as an argument to the Gazebo's launch file
            "world": world_path # World argument tells which world file to open
        }.items() # The .items() converts the dictionary in to the format expected by IncludeLaunchDescription
    )

## Spawning the Robot in Gazebo. This starts Gazebo’s robot spawning script.

    spawn_robot_node = Node( # Starts spawn_robot_node
        package="gazebo_ros", # Package name
        executable="spawn_entity.py", # Executable name, This script reads a robot model and inserts it into the Gazebo simulation.
        arguments=[ 
          "-topic", "robot_description", # “Reading the robot model (URDF) from the ROS topic/parameter source named robot_description (Published b y robot_state_publisher).
          "-entity", "simple_sd_robot", # Giving the entity a model name in Gazebo
          "-x", "0.0", # Sets the inital x position in the world
          "-y", "4.0", # Sets the inital y position in the world
          "-z", "0.10", # Sets the inital z position in the world
          "-Y", "-1.57" # Sets the inital yaw position in the world, in radians, corresponds to 90 degrees
        ],
        output="screen" # Makes the Node output logs to the terminal screen 
    )
## The final list of items the launch file executes
    return LaunchDescription([
        set_gazebo_model_path, # Sets Gazebo model path, allowing gazebo to find models
        gazebo_node, # Starts Gazebo and loads the world
        robot_state_publisher_node, # Publishes robots TF tree from the URDF
        spawn_robot_node # Spawns the robot in the Gazebo at the location defined
    ])