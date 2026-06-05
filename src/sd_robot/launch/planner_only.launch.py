import os

from launch import LaunchDescription # Imports the main object returned by every Python ROS 2 launch file.
from launch_ros.actions import Node # Lets launch file start ROS 2 nodes.
from ament_index_python.packages import get_package_share_directory # Finds the installed package share folder.
from launch.actions import TimerAction # We use TimerAction to delay the lifecycle manager startup by 3 seconds.

def generate_launch_description(): # File Entry point. ROS 2 calls this function and launches what it returns.
    pkg_share = get_package_share_directory("sd_robot") # finds the installed sd_robot share directory.

    params_file = os.path.join( # builds the full path to nav2_params.yaml, the main nav2 configuration.
        pkg_share,
        "config",
        "nav2_params.yaml"
    )

    map_file = "/home/yeshwanth/self_drive_ws/maps/warehouse_world_map.yaml" # saved map YAML file, the map that map_server loads.

    map_server_node = Node( # Starts the Nav2 map server.
        package="nav2_map_server", # Package name
        executable="map_server", # Executable name
        name="map_server", # Node name, macthes yaml file
        output="screen", # Prints logs to the screen
        parameters=[
            params_file, # Loads the Nav2 YAML config file.
            {
                "use_sim_time": True,
                "yaml_filename": map_file # Tells map server which saved map YAML file to load.
            }
        ]
    )

    amcl_node = Node( # Starts AMCL ( Adaptive Monte Carlo Localization) - Given a saved map and laser scans, estimate where the robot is on the map.
        package="nav2_amcl", # Package name
        executable="amcl", # Executable
        name="amcl", # Node name
        output="screen", # Prints logs to the screen
        parameters=[params_file] # AMCL parameters come from nav2_params.yaml.
    )

    planner_server_node = Node( # Starts the Nav2 global planner. Given a start pose and goal pose, compute a collision-free path through the map.
        package="nav2_planner", # Package name that the node comes from 
        executable="planner_server", # Executable name
        name="planner_server", # Node name
        output="screen", # Prints logs to the screen
        parameters=[params_file] # Uses parameters from nav2_params.yaml.
    )

    # Starts the nav2 lifecycle manager node. 
    # Nav2 nodes are lifecycle nodes. They do not simply start and become active immediately. 
    # They go through the following states: unconfigured, inactive, active, finalized. 
    # The lifecycle manager automatically configures and activates them.
    lifecycle_manager_node = Node( 
        package="nav2_lifecycle_manager", # Comes from Nav2 lifecycle manager package.
        executable="lifecycle_manager", # Executable name
        name="lifecycle_manager_planner_only", # Node name
        output="screen", # Prints logs to screen
        parameters=[
            {
                "use_sim_time": True, # Uses sim time
                "autostart": True, # This tells the lifecycle manager: Automatically configure and activate the listed nodes.
                "node_names": [ # These are the lifecycle nodes it manages.
                    "map_server",
                    "amcl",
                    "planner_server" # The lifecycle manager will try to transition these nodes into active state.
                ]
            }
        ]
    )
    # So your launch file starts:
    # map_server
    # amcl
    # planner_server
    # then waits: 3 seconds
    # then starts:
    # lifecycle_manager
    
    return LaunchDescription([
    map_server_node,
    amcl_node,
    planner_server_node,

    TimerAction( # This delays the lifecycle manager startup by 3 seconds.
        period=3.0,
        actions=[lifecycle_manager_node]
    )
])