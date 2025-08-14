#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('turtlebot3_navigation2')
    use_simulator = LaunchConfiguration('use_simulator')
    # Khởi động Gazebo nếu sử dụng simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ),
        condition=IfCondition(use_simulator)
    )
    
    # Khởi động Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation2.launch.py')
        ),
    )
    
 
   
    
    # Tạo launch description
    ld = LaunchDescription()
 
    ld.add_action(gazebo)
    ld.add_action(nav2)

    # ld.add_action(waypoint_follower)
    
    return ld