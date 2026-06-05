# Launch file to launch the controller of choice. 
# Can Choose one of the following controllers:
# pure_pursuit
# stanley
# lqr
# mppi
# mpc

from launch import LaunchDescription # Imports the object returned by a Python launch file. Tells ROS 2 launch what actions are available.
from launch.actions import DeclareLaunchArgument # This lets the launch file define an argument that the user can pass from the terminal.
from launch.conditions import IfCondition # This lets you conditionally start a node.
# PythonExpression builds a Python-style boolean expression from launch values.
from launch.substitutions import LaunchConfiguration, PythonExpression # LaunchConfiguration reads the value of a launch argument.
from launch_ros.actions import Node # This imports the ROS 2 Node launch action.


def generate_launch_description(): # Entry point to the launch file 
    controller = LaunchConfiguration("controller") # This creates a reference to the launch argument named: controller.
    # At launch time, read the value of the argument called controller.

    # This declares the launch argument named controller. 
    declare_controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="pure_pursuit", # The default value is pure_pursuit.
        description=(
            "Controller to run: "
            "pure_pursuit, stanley, lqr, mppi, or mpc" # The description tells users the allowed options. 
        )
    )

    # Defines the PurePursuit Node
    pure_pursuit_node = Node( 
        package="sd_control", # Package name
        executable="pure_pursuit_plan", #Executable name, must match the setup.py entry point.
        name="pure_pursuit_plan_follower", # the ROS Node is named /pure_pursuit_plan_follower.
        output="screen", # Print logs to the terminal.
        condition=IfCondition( # Only start this node if controller == pure_pursuit.
            PythonExpression(["'", controller, "' == 'pure_pursuit'"])
        )
    )

    # Defines the Stanley Node
    stanley_node = Node(
        package="sd_control", # Package Name
        executable="stanley_controller", # Executable Name
        name="stanley_controller", # the ROS Node is named /Stanley_controller.
        output="screen", # Print logs to the terminal.
        condition=IfCondition( # Only start this node if controller == stanley.
            PythonExpression(["'", controller, "' == 'stanley'"])
        )
    )

    # Defines the LQR Node
    lqr_node = Node(
        package="sd_control", # Package Name
        executable="lqr_controller", # Executable Name
        name="lqr_controller", # the ROS Node is named /lqr_controller.
        output="screen",  # Print logs to the terminal.
        condition=IfCondition( # Only start this node if controller == lqr.
            PythonExpression(["'", controller, "' == 'lqr'"])
        )
    )

    # Defines the MPPI Node
    mppi_node = Node(
        package="sd_control", # Package Name
        executable="mppi_controller", # Executable Name
        name="mppi_controller", # the ROS Node is named /mppi_controller.
        output="screen", # Print logs to the terminal.
        condition=IfCondition( # Only start this node if controller == lqr.
            PythonExpression(["'", controller, "' == 'mppi'"])
        )
    )

    mpc_node = Node(
        package="sd_control", # Package Name
        executable="mpc_controller", # Executable Name
        name="mpc_controller", # the ROS Node is named /mpc_controller.
        output="screen", # Print logs to the terminal.
        condition=IfCondition( # Only start this node if controller == lqr.
            PythonExpression(["'", controller, "' == 'mpc'"])
        )
    )
    
    # Declare the controller argument.
    # Then evaluate each controller node condition.
    # Start only the one whose condition is true.
    return LaunchDescription([
        declare_controller_arg,
        pure_pursuit_node,
        stanley_node,
        lqr_node,
        mppi_node,
        mpc_node,
    ])