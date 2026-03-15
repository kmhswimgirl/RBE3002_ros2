import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():   

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch'), '/rviz2.launch.py'])
    )

    lab2_node = Node(
        package='control',
        executable='tf_controller.py',
        name='lab2_driver',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rviz,
        lab2_node
    ])
