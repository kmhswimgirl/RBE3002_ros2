import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    lab2_path = os.path.join(get_package_share_directory('lab2'))

   
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

    lab2_node = Node(
        package='lab2',
        executable='lab2_timer.py',
        name='lab2_t',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        turtlebot,
        rviz,
        lab2_node
    ])


