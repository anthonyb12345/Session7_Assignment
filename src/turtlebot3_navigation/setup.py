from setuptools import find_packages, setup
import os

package_name = 'turtlebot3_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [os.path.join('launch', 'wall_follower_with_gazebo.py')]),
          
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony.bassil3@net.usj.edu.lb',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver = turtlebot3_navigation.robot_driver:main',
            'wall_finder_server = turtlebot3_navigation.wall_finder_server:main',
            'lap_time_server = turtlebot3_navigation.lap_time_server:main',
            'lap_time_client = turtlebot3_navigation.lap_time_client:main',

        ],
    },
)
