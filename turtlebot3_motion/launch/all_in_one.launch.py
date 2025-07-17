import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Set TURTLEBOT3_MODEL environment variable
    robot_model = 'waffle'
    os.environ['TURTLEBOT3_MODEL'] = robot_model
    
    # Add environment variable action
    set_turtlebot_model_env = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL', 
        robot_model
    )
    
    # Get package directories
    pkg_share = get_package_share_directory('turtlebot3_motion')
    nav2_bringup_dir = get_package_share_directory('turtlebot3_navigation2')
    
    # Create map path with proper directory
    default_map_path = os.path.join(get_package_share_directory('turtlebot3_motion'), 'maps', 'map.yaml')
    

    # Configure launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    use_pure_pursuit = LaunchConfiguration('use_pure_pursuit')
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    use_pure_pursuit_arg = DeclareLaunchArgument(
        'use_pure_pursuit',
        default_value="false"  # Default to PD controller
    )
    
    # Use the full path to the map file
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map_path
    )
    print(default_map_path)
    # Include Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                "launch",
                "turtlebot3_world.launch.py"
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model': 'waffle',
        }.items()
    )
    # print(nav2_bringup_dir)
    # Include Localization
    navigation2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                "launch",
                "navigation2.launch.py"
            )
        ),
        launch_arguments={
            'map': map_file,
            'robot_model': 'waffle',
            'use_sim_time': use_sim_time
        }.items()
    )

    # A* Planner Node
    planning_node = Node(
        package="turtlebot3_planning",
        executable="a_star_planner.py",
        name="a_star_planner",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # PD Motion Controller
    motion_pd_node = Node(
        package="turtlebot3_motion",
        executable="pd_motion_planner.py",
        name="pd_controller",
        output="screen",
        condition=UnlessCondition(use_pure_pursuit)
    )

    # Pure Pursuit Controller
    motion_pure_pursuit_node = Node(
        package="turtlebot3_motion",
        executable="pure_pursuit_planner.py",
        name="pure_pursuit_controller",
        output="screen",
        condition=IfCondition(use_pure_pursuit)
    )

    # Build the launch description
    ld = LaunchDescription()
    # Add environment variable first
    ld.add_action(set_turtlebot_model_env)
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_pure_pursuit_arg)
    ld.add_action(map_arg)
    ld.add_action(gazebo_node)
    ld.add_action(navigation2_node)
    ld.add_action(planning_node)
    ld.add_action(motion_pd_node)
    ld.add_action(motion_pure_pursuit_node)

    return ld