import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    # map server + rviz + lifecycle node
    map_server_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lab3'), 'launch'), '/map_server.launch.py'])
    )

    path_planner = Node(
        package='lab3',
        executable='path_planner.py',
        name='lab3_paths',
        output='screen'
    )

    return LaunchDescription([
        path_planner,
        map_server_rviz
    ])