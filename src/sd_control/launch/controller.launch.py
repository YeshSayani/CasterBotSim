from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    controller = LaunchConfiguration("controller")

    declare_controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="pure_pursuit",
        description=(
            "Controller to run: "
            "pure_pursuit, stanley, lqr, mppi, or mpc"
        )
    )

    pure_pursuit_node = Node(
        package="sd_control",
        executable="pure_pursuit_plan",
        name="pure_pursuit_plan_follower",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'pure_pursuit'"])
        )
    )

    stanley_node = Node(
        package="sd_control",
        executable="stanley_controller",
        name="stanley_controller",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'stanley'"])
        )
    )

    lqr_node = Node(
        package="sd_control",
        executable="lqr_controller",
        name="lqr_controller",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'lqr'"])
        )
    )

    mppi_node = Node(
        package="sd_control",
        executable="mppi_controller",
        name="mppi_controller",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'mppi'"])
        )
    )

    mpc_node = Node(
        package="sd_control",
        executable="mpc_controller",
        name="mpc_controller",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'mpc'"])
        )
    )

    return LaunchDescription([
        declare_controller_arg,
        pure_pursuit_node,
        stanley_node,
        lqr_node,
        mppi_node,
        mpc_node,
    ])