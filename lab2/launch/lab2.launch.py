import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    lab2_path = os.path.join(get_package_share_directory('lab2'))

    arguments = LaunchDescription([
        # set x position of turtlebot3
        DeclareLaunchArgument('x_pose', default_value='.01', description='turtlebot X coord'),
        # set y position of turtlebot3
        DeclareLaunchArgument('y_pose', default_value='0', description='turtlebot Y coord'),
        # set z position of turtlebot3
        DeclareLaunchArgument('z_pose', default_value='0', description='turtlebot Z coord'),
    ])

    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/empty_world.launch.py']),
        launch_arguments = [
            ('x_pose',LaunchConfiguration('x_pose')),
            ('y_pose', LaunchConfiguration('y_pose')),
            ('z_pose', LaunchConfiguration('z_pose')),
            ('use_sim_time', 'True')
        ]
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch'), '/rviz2.launch.py'])
    )

    lab2_node = Node(
        package='lab2',
        executable='lab2_code.py',
        name='lab2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        arguments,
        turtlebot,
        rviz,
        lab2_node
    ])


