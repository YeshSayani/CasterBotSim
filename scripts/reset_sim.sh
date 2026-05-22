#!/bin/bash

echo "Stopping Gazebo, RViz, Nav2, and custom controller nodes..."

killall gzserver gzclient gazebo rviz2 2>/dev/null

pkill -f nav2
pkill -f planner_only
pkill -f nav2_plan_client
pkill -f pure_pursuit_plan
pkill -f stanley_controller
pkill -f lqr_controller
pkill -f mppi_controller

echo "Restarting ROS 2 daemon..."

ros2 daemon stop
ros2 daemon start

echo "Done. Simulation/controller processes cleaned up."
