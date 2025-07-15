import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    # map file path for simple_map
    map_file_path = os.path.join(
        get_package_share_directory('lab3'), 
        'maps',
        'simple_map.yaml'
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('lab3'),
        'rviz',
        'lab3_config.rviz')

    # map_server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename':map_file_path}]
    )

    # start the node lifecyle manager
    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    ) 

    path_planner = Node(
        package='lab3',
        executable='path_planner.py',
        name='planner',
        output='screen'
    )

    # map to odom static transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map',
        '--child-frame-id', 'odom'], # supposed to use ros2 style args but i dont think it matters tbh
        output='screen'
    )

    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/empty_world.launch.py']),
        launch_arguments = [
            ('use_sim_time', 'True')
        ]
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch'), '/rviz2.launch.py'])
    )

    rviz2 =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    return LaunchDescription([
        path_planner,
        rviz2,
        turtlebot,
        static_tf,
        lifecycle_manager,
        map_server
    ])