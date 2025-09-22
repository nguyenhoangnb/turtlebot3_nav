#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Package Directories
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_slam = get_package_share_directory('turtlebot3_slam')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world.world',
        description='World file name'
    )
    
    # TurtleBot3 Gazebo launch
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ])
    )
    
    # RBPF SLAM node (changed name to match your Python script)
    rbpf_slam_node = Node(
        package='turtlebot3_slam',
        executable='fast_slam.py',
        name='rbpf_slam',  # Changed to match your Python script
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            
            # Frame IDs
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'body_frame_id': 'base_footprint',
            
            # Joint names
            'wheel_left_joint': 'wheel_left_joint',
            'wheel_right_joint': 'wheel_right_joint',
            
            # Robot parameters (TurtleBot3 Burger)
            'wheel_base': 0.16,
            'wheel_radius': 0.033,
            
            # Lidar parameters
            'beam_min': -135.0,  # degrees
            'beam_max': 135.0,   # degrees  
            'beam_delta': 0.25,  # degrees
            'range_min': 0.12,
            'range_max': 3.5,
            
            # Laser model parameters
            'z_hit': 0.8,
            'z_short': 0.1,
            'z_max': 0.05,
            'z_rand': 0.05,
            'sigma_hit': 0.1,
            
            # Particle filter parameters
            'num_particles': 100,
            'num_samples_mode': 10,
            
            # Motion model noise parameters
            'srr': 0.1,
            'srt': 0.1,
            'str': 0.1,
            'stt': 0.1,
            'motion_nosie_theta': 0.01,
            'motion_nosie_x': 0.01,
            'motion_nosie_y': 0.01,
            
            # Sample range parameters
            'sample_range_theta': 0.1,
            'sample_range_x': 0.1,
            'sample_range_y': 0.1,
            
            # Likelihood parameters
            'scan_likelihood_min': 0.01,
            'scan_likelihood_max': 0.99,
            'pose_likelihood_min': 0.01,
            'pose_likelihood_max': 0.99,
            
            # Map parameters
            'map_min': -10.0,
            'map_max': 10.0,
            'map_resolution': 0.05,
        }]
    )
    
    # RViz
    rviz_config_file = os.path.join(
        pkg_turtlebot3_slam, 'rviz', 'slam.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        turtlebot3_gazebo,
        # rbpf_slam_node,  # Changed variable name
        rviz_node,
    ])