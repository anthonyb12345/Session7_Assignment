# Session7_Assignment

# TurtleBot3 Navigation and Interaction

## Overview
This ROS 2 project involves multiple nodes and services to control and interact with a TurtleBot3 robot. The key components are:

1. **Wall Finder Service Server (`wall_finder_server.py`)**: This node finds the closest wall and navigates the robot towards it.
2. **Robot Driver Node (`robot_driver.py`)**: This node interacts with the Wall Finder Service to find a wall and then follows it.
3. **Lap Time Action Server (`lap_time_server.py`)**: This node measures the time taken for the robot to complete a lap around an arena.
4. **Lap Time Action Client (`lap_time_client.py`)**: This node interacts with the Lap Time Action Server to start and monitor the lap timing process.


## Launch File

### Functionality
- Launches all the nodes (`wall_finder_server`, `robot_driver`, `lap_time_server`, and `lap_time_client`) together.

## Instructions

### Build the ROS 2 Package

1. Navigate to your ROS 2 workspace:
    ```sh
    cd ~/ros2_ws
    ```
    ```sh
   colcon build
    ```
    ```sh
   source install/setup.bash
    ```
    ```sh
   export TURTLEBOT3_MODEL=burger
    ```
   
### Run the Launch File

   ```sh
   ros2 launch wall_follower_with_gazebo.py

   ```

