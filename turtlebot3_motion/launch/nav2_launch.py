from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Lấy model turtlebot
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    rosdistro = os.environ.get("ROS_DISTRO", 'humble')
    
    # Đường dẫn tới các file cấu hình
    params_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', rosdistro, f'{turtlebot3_model}.yaml')
    default_map_path = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')
    rviz_config_path = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    
    # Danh sách các node cần quản lý lifecycle
    lifecycle_node = ["behavior_server","controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]
    
    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
    
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map_path,
        description="Full path to map yaml file"
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='Whether to start Gazebo'
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ]
    )
    
    # Lifecycle Manager for Map Server
    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', "amcl"]}
        ]
    )
    
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ),
        condition=IfCondition(use_simulator),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Nav2 Nodes
    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",  # Sửa tên node này từ "planner_server" sang "controller_server"
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    nav2_behaviors = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_node},
            {"use_sim_time": use_sim_time},
            {"auto_start": True}
        ],
    )
    
    # AMCL Node
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Tạo launch description
    ld = LaunchDescription()
    
    # Thêm launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(map_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_simulator_arg)
    
    # Thêm Gazebo và RViz
    ld.add_action(gazebo)
    ld.add_action(rviz)
    
    # Thêm Map Server
    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle_manager)
    ld.add_action(behavior_server)

    # Thêm AMCL cho định vị
    ld.add_action(amcl)

    # Thêm các node Nav2
    ld.add_action(nav2_behaviors)
    ld.add_action(nav2_controller_server)
    ld.add_action(nav2_planner_server)
    ld.add_action(nav2_smoother_server)
    ld.add_action(nav2_lifecycle_manager)

    return ld