#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # file paths
    this_dir = os.path.join(get_package_share_directory('mapping'))
    tb3_gz_pkg = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    sim_pkg_launch = os.path.join(get_package_share_directory('simulation'), 'launch')
    rviz_config_file = os.path.join(get_package_share_directory('mapping'), 'rviz', 'lab4_slam.rviz')
    map_dir = os.path.join(this_dir, 'maps') 

    # variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    slam = LaunchConfiguration('slam', default='True')
    config_file =  LaunchConfiguration('config_file',default=os.path.join(get_package_share_directory('mapping'), 'config', 'tb3_nav.yaml'))

    # launch arguments
    params = DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Full path to param file to load')

    sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

    # simulation package launch file (launches gz_sim)
    small_field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_launch,'world_test.launch.py')
        )
    )
    
    # robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gz_pkg, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # spawn in turtlebot
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gz_pkg, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.06',
            'y_pose': '0.0',
            'use_sim_time': 'true'
        }.items()
    )

    # map to odom static transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map',
        '--child-frame-id', 'odom'], 
        output='screen'
    )

    # slam launch file
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_dir,'launch', 'slam_bringup.launch.py')),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': config_file,
            'slam': slam}.items()
    )

    # rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # path planner node (server)
    path_planner = Node(
        package='pathing',
        executable='path_planner.py',
        name='a_star',
        output='screen'
    )

    # driver node (client)
    driver = Node(
        package='control',
        executable="cb_controller.py",
        name='driver',
        output='screen'
    )

    # frontier node
    frontier = Node(
        package='mapping',
        executable='frontiers.py',
        name='frontier_debug',
        output='screen'
    )

    # laser filter for SLAM
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("laser_filters"),
                "examples", "bin_filter_example.yaml"
            ])]
    )

    return LaunchDescription([
        params,
        sim_time,
        small_field_launch,
        static_tf,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        laser_filter,
        slam_bringup,
        frontier,
        path_planner,
        driver,
        rviz2
    ])