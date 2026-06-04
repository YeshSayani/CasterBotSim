# Python installation recipe for sd_control package
# Since sd_control/package.xml mentions the build type is ament_python
# ROS 2 uses this setup.py file to install the Python package, launch files, and command-line ROS 2 executables.
from setuptools import find_packages, setup # setup is the main function that defines how the Python package is installed. find_packages automatically finds Python packages inside this folder.
import os # Imports Python’s operating-system utilities.
from glob import glob #glob finds files matching a pattern.

package_name = 'sd_control' # Package Name

setup( # Begins the Python package setup definition. Tells Python/ROS 2 how to install the package.
    name=package_name, # Sets Python package distribution name to the package name, should match package name in package.xml
    version='0.0.0', # Python Package version, ideal to match package.xml
    packages=find_packages(exclude=['test']), # Find all python packages in this folder except test. Should find sd_control and install it.
    # Needs __init__.py to work
    data_files=[ # This section tells setuptools to install non-Python files. 
        # Python code gets installed by packages=find_packages(...), but ROS2 files get installed using data_files.
        ('share/ament_index/resource_index/packages', # This installs a resource marker file into the ament index.
            ['resource/' + package_name]), # Tells ROS2 that a package named sd_control exists.
        ('share/' + package_name, ['package.xml']), # Install package.xml into share/sd_control
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Installs launch files into share/sd_control/launch
    ],
    install_requires=['setuptools'], # Says the package requires setuptools
    zip_safe=True, # Mentions whether package can be safely installed as a zip archive
    maintainer='yeshwanth', # Maintainer meta data
    maintainer_email='yeshwanthsayani9@gmail.com', 
    description='Python ROS 2 controllers for a differential-drive self-driving robot simulation.', # Package description
    #license='TO DO: License declaration',
    license='Apache-2.0',
    extras_require={ # If installing test dependencies, include pytest. Corresponds to the same in package.xml
        'test': [
            'pytest',
        ],
    },
    entry_points={ # Tells setuptools: Create command-line executables for these Python node files. This makes ros2 run work. 
    'console_scripts': [ # Every line follows the format: 'executable_name = python_package.python_file:function_name'
        'go_to_goal = sd_control.go_to_goal:main', # Executable name must match the one in the launch file 
        'waypoint_follower = sd_control.waypoint_follower:main', # If executable exists in the launch file but not in this file, launch will fail.
        'pure_pursuit = sd_control.pure_pursuit:main', # Makes the python files like the purepursuit.py runnable. 
        'pure_pursuit_plan = sd_control.pure_pursuit_plan:main',
        'nav2_plan_client = sd_control.nav2_plan_client:main',
        'stanley_controller = sd_control.stanley_controller:main',
        'lqr_controller = sd_control.lqr_controller:main',
        'mppi_controller = sd_control.mppi_controller:main',
        'mpc_controller = sd_control.mpc_controller:main',
        ],
    },
) # ros2 pkg executables sd_control: command to list executables of a package. 
