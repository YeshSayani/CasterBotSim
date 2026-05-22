from setuptools import find_packages, setup

package_name = 'sd_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeshwanth',
    maintainer_email='yeshwanthsayani9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'go_to_goal = sd_control.go_to_goal:main',
        'waypoint_follower = sd_control.waypoint_follower:main',
        'pure_pursuit = sd_control.pure_pursuit:main',
        'pure_pursuit_plan = sd_control.pure_pursuit_plan:main',
        'nav2_plan_client = sd_control.nav2_plan_client:main',
        'stanley_controller = sd_control.stanley_controller:main',
        'lqr_controller = sd_control.lqr_controller:main',
        'mppi_controller = sd_control.mppi_controller:main',
        ],
    },
)
