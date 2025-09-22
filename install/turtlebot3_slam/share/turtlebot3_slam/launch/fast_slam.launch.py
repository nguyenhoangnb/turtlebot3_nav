#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_turtlebot3_slam = FindPackageShare('turtlebot3_slam')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # FastSLAM node
    fast_slam_node = Node(
        package='turtlebot3_slam',
        executable='fast_slam.py',
        name='fast_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            
            # Frame IDs
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'body_frame_id': 'base_footprint',
            
            # Joint names
            'left_wheel_joint': 'left_wheel_joint',
            'right_wheel_joint': 'right_wheel_joint',
            
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
        }],
        remappings=[
            ('scan', '/scan'),
            ('joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        fast_slam_node,
    ])
