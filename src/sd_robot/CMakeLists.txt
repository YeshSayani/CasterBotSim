cmake_minimum_required(VERSION 3.8) # Requires CMake version 3.8 or newer, CMake is a build system generator used by ament_cmake.
# ROS2 uses CMake to install package resources, colcon calls CMake for this package when run. 
project(sd_robot) # Declares CMake project name, should match package.xml package name
# File tells colcon or CMake how to build and install the sd_robot package
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") # Checks what C++ compiler is being used GCC or Clang, add strict warning flags (Wall, Wextra, Wpedantic)
  add_compile_options(-Wall -Wextra -Wpedantic) # It is commonly included by ROS 2 package templates
endif()

find_package(ament_cmake REQUIRED) # Find the ROS 2 ament_cmake build system package.
# REQUIRED: If ament_cmake is not found, stop with an error.

install(
  DIRECTORY launch urdf worlds config models # Tells the CMake/colcon to install these folders into install/sd_robot/share/sd_robot folder
  DESTINATION share/${PROJECT_NAME} # Why launch file can access:
# os.path.join(pkg_share, "urdf", "simple_robot.urdf.xacro")
# os.path.join(pkg_share, "worlds", "warehouse_world.world")
# os.path.join(pkg_share, "models")
)
# Register this as a ROS 2 package. Export package metadata. Make it discoverable by ROS 2 tooling.
# This is required in an ament_cmake package.
ament_package()