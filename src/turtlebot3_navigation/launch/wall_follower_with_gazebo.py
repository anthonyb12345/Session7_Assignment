import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtlebot3_gazebo_launch_dir = get_package_share_directory('turtlebot3_gazebo')
    dqn_stage1_launch_file = os.path.join(turtlebot3_gazebo_launch_dir, 'launch', 'turtlebot3_dqn_stage1.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dqn_stage1_launch_file)
        ),
        Node(
            package='turtlebot3_navigation',
            executable='wall_finder_server',
            name='wall_finder_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='turtlebot3_navigation',
            executable='robot_driver',
            name='robot_driver',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='turtlebot3_navigation',
            executable='lap_time_server',
            name='lap_time_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
         Node(
            package='turtlebot3_navigation',
            executable='lap_time_client',
            name='lap_time_client',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
        
    ])

'''

        Node(
            package='turtlebot3_navigation',
            executable='lap_time_client',
            name='lap_time_client',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
'''

